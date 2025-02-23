ARG BASE_IMAGE=ros:humble
FROM ${BASE_IMAGE}

# User creation with host UID/GID sync
ARG USERNAME=turret
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} \
    --create-home --shell /bin/bash ${USERNAME} && \
    # Create .bashrc with essential configurations
    echo "source /usr/share/bash-completion/bash_completion" >> /home/${USERNAME}/.bashrc && \
    echo "alias ll='ls -alF'" >> /home/${USERNAME}/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc && \
    # Install completion packages
    apt-get update && apt-get install -y bash-completion && \
    # Fix ownership
    chown -R ${USER_UID}:${USER_GID} /home/${USERNAME}

RUN echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN sudo apt update && apt upgrade -y
RUN sudo apt install -y python3-pip

WORKDIR /home/turret

# Install ROS dependencies
RUN sudo apt update && apt install -y ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
    ros-humble-xacro ros-humble-ros-core ros-humble-urdf 

# Install prerequisites and setup Gazebo Sim and ROS 2 bridge
RUN sudo apt update && apt install -y \
    wget \
    gnupg2 \
    lsb-release 

# Add Gazebo Sim repository
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg\
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN sudo apt update && \
    sudo apt install -y gz-harmonic ros-humble-ros-gzharmonic libgz-transport14-dev ros-humble-foxglove-bridge ros-humble-rosbag2-storage-mcap

# Set up Gazebo environment variables
# ENV GAZEBO_MODEL_PATH=/usr/share/gz/models
# ENV GZ_SIM_RESOURCE_PATH=/usr/share/gz

# 🔹 Use the 64-bit ARM cross-compiler (for Raspberry Pi)
ENV GNU_HOST=aarch64-linux-gnu
ENV C_COMPILER_ARM_LINUX=$GNU_HOST-gcc
ENV CXX_COMPILER_ARM_LINUX=$GNU_HOST-g++

ENV CROSS_INSTALL_PREFIX=/usr/$GNU_HOST
ENV CROSS_TOOLCHAIN=/arm.toolchain.cmake

# 🔹 Install required cross-compilation tools
RUN sudo apt-get update && \
    sudo apt-get --no-install-recommends install -y \
    autoconf \
    automake \
    build-essential \
    ca-certificates \
    curl \
    # C/C++ cross compilers for 64-bit ARM
    gcc-$GNU_HOST \
    g++-$GNU_HOST \
    binutils-$GNU_HOST \
    libc-dev-arm64-cross \
    git \
    gnupg \
    libssl-dev \
    openssh-client \
    pkg-config \
    software-properties-common \
    wget && \
    rm -rf /var/lib/apt/lists/*

# 🔹 Configure CMake for cross-compilation
WORKDIR /cross-tools
RUN echo "set(CMAKE_SYSTEM_NAME Linux)" >> arm.toolchain.cmake && \
    echo "set(CMAKE_SYSTEM_PROCESSOR aarch64)" >> arm.toolchain.cmake && \
    echo "set(CMAKE_C_COMPILER /usr/bin/$C_COMPILER_ARM_LINUX)" >> arm.toolchain.cmake && \
    echo "set(CMAKE_CXX_COMPILER /usr/bin/$CXX_COMPILER_ARM_LINUX)" >> arm.toolchain.cmake && \
    echo "set(CMAKE_PREFIX_PATH $CROSS_INSTALL_PREFIX)" >> arm.toolchain.cmake

# 🔹 Install Pigpio for 64-bit ARM
RUN PIGPIO_DIR=/workspace/pigpio && \
    git clone --single-branch --branch master https://github.com/joan2937/pigpio.git $PIGPIO_DIR && \
    cd $PIGPIO_DIR && \
    make CROSS_COMPILE=$GNU_HOST- CC=$GNU_HOST-gcc CXX=$GNU_HOST-g++ AR=$GNU_HOST-ar LD=$GNU_HOST-ld STRIP=$GNU_HOST-strip -j$(nproc) && \
    make CROSS_COMPILE=$GNU_HOST- CC=$GNU_HOST-gcc CXX=$GNU_HOST-g++ AR=$GNU_HOST-ar LD=$GNU_HOST-ld STRIP=$GNU_HOST-strip install DESTDIR=$CROSS_INSTALL_PREFIX PREFIX="" && \
    cd /workspace && \
    rm -rf $PIGPIO_DIR

# 🔹 Verify the compiled library architecture
RUN file $CROSS_INSTALL_PREFIX/lib/libpigpio.so

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]
