cmake_minimum_required(VERSION 3.10)
project(turret_dev)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# -------------------------------
# Fetch GoogleTest using FetchContent
# -------------------------------
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/release-1.12.1.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# -------------------------------
# Find yaml-cpp (ensure it is installed or use FetchContent as an alternative)
# -------------------------------
find_package(yaml-cpp REQUIRED)

# -------------------------------
# Production target: turret executable
# -------------------------------
add_executable(turret
    src/turret_controller.cpp
    src/turret.cpp
    src/spiral_zipper.cpp
    src/servo_city_motor.cpp
    src/encoder.cpp
    src/limit_switch.cpp
    src/cubemars_control.cpp
    src/config.cpp
)
target_include_directories(turret PRIVATE include)
target_link_libraries(turret PRIVATE pigpio yaml-cpp)

# -------------------------------
# Test target: turret_tests executable
# -------------------------------
add_executable(turret_tests
    tests/test_limit_switches.cpp
    tests/test_spiral_zipper_zero.cpp
    tests/test_spiral_zipper_motor.cpp
    src/config.cpp
    src/limit_switch.cpp
    src/spiral_zipper.cpp
    src/encoder.cpp
    src/servo_city_motor.cpp
    # Include any additional production sources required by your tests.
)

target_include_directories(turret_tests PRIVATE include)
target_link_libraries(turret_tests PRIVATE
    gtest
    gtest_main
    pigpio
    yaml-cpp
)

# Enable CTest so that you can run "ctest" from the build folder.
enable_testing()
add_test(NAME turret_tests COMMAND turret_tests)
