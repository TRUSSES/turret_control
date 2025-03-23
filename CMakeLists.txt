cmake_minimum_required(VERSION 3.10)

# Set the project name
project(turret_dev)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the executable target
add_executable(turret
    src/TurretController.cpp
    src/cubemars_control.cpp
    src/main.cpp
)

# Include headers
target_include_directories(turret PRIVATE include)

# Link pigpio library
target_link_libraries(turret PRIVATE pigpio)
