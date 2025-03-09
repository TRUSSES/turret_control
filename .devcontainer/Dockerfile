ARG TARGET_ARCH=arm64
FROM --platform=linux/${TARGET_ARCH} arm64v8/ros:humble

# Retain user creation
ARG USERNAME=turret
ARG USER_UID=1000
ARG USER_GID=1000

# Use separate layers for caching package installations
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    curl \
    git \
    lsb-release \
    build-essential \
    python3-colcon-common-extensions \
    cmake \
    ninja-build \
    qemu-user-static \
    gcc-aarch64-linux-gnu \
    g++-aarch64-linux-gnu \
    libc-dev \
    libstdc++-12-dev \
    ros-humble-geometry-msgs \
    libncurses-dev \
    && rm -rf /var/lib/apt/lists/*

# Create the user and group
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

USER root
WORKDIR /home/$USERNAME

# Set up ROS 2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# Set up cross-compilation environment
ENV CROSS_COMPILE=/usr/bin/aarch64-linux-gnu-
ENV TARGET_TRIPLE=aarch64-linux-gnu
ENV SYSROOT=/usr/aarch64-linux-gnu
ENV CC=${CROSS_COMPILE}gcc
ENV CXX=${CROSS_COMPILE}g++
ENV PATH=$PATH:/usr/bin/aarch64-linux-gnu

# Force C++17 as the default standard
ENV CXXFLAGS="-std=c++17"
ENV CMAKE_CXX_STANDARD=17

# Install pigpio from source with proper cross-compilation as root
WORKDIR /workspace
RUN PIGPIO_DIR=/workspace/pigpio && \
    git clone --single-branch --branch master https://github.com/joan2937/pigpio.git $PIGPIO_DIR && \
    cd $PIGPIO_DIR && \
    make CROSS_COMPILE=$GNU_HOST- CC=$GNU_HOST-gcc CXX=$GNU_HOST-g++ AR=$GNU_HOST-ar LD=$GNU_HOST-ld STRIP=$GNU_HOST-strip -j$(nproc) && \
    make CROSS_COMPILE=$GNU_HOST- CC=$GNU_HOST-gcc CXX=$GNU_HOST-g++ AR=$GNU_HOST-ar LD=$GNU_HOST-ld STRIP=$GNU_HOST-strip install DESTDIR=$CROSS_INSTALL_PREFIX PREFIX="" && \
    cd /workspace && \
    rm -rf $PIGPIO_DIR

# Switch back to non-root user
USER $USERNAME