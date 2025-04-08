cmake_minimum_required(VERSION 3.10)

# Set the project name
project(turret_dev)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the executable target for your main application
add_executable(turret
    src/turret_controller.cpp
    src/turret.cpp
    src/spiral_zipper.cpp
    src/servo_city_motor.cpp
    src/encoder.cpp
    src/limit_switch.cpp
    src/cubemars_control.cpp
)

# Include headers from the include directory
target_include_directories(turret PRIVATE include)

# Link pigpio library for the main executable
target_link_libraries(turret PRIVATE pigpio)

# ---------------------------------------------------
# GoogleTest Integration for HIL testing
# ---------------------------------------------------

include(FetchContent)
# Fetch the latest release of GoogleTest
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/release-1.12.1.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Add the test executable (add more test files as needed)
add_executable(turret_tests
    tests/test_limit_switch.cpp
)

# Include directories for tests (your include directory)
target_include_directories(turret_tests PRIVATE include)

# Link libraries: GoogleTest and pigpio (if needed by tests)
target_link_libraries(turret_tests PRIVATE gtest gtest_main pigpio)

# Enable CTest (optional; allows running "ctest" from the build directory)
enable_testing()
add_test(NAME turret_tests COMMAND turret_tests)
