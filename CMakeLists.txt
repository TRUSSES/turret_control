cmake_minimum_required(VERSION 3.8)
project(turret_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# -------------------------------
# Find ROS2 dependencies
# -------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# -------------------------------
# Find yaml-cpp (ensure it is installed or use FetchContent as an alternative)
# -------------------------------
find_package(yaml-cpp REQUIRED)

# -------------------------------
# Generate ROS2 interfaces
# -------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TurretState.msg"
  "msg/ZipperCommand.msg"
  "srv/ZeroTurret.srv"
  "srv/SetState.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# -------------------------------
# Fetch GoogleTest using FetchContent (for tests)
# -------------------------------
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/release-1.12.1.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# -------------------------------
# ROS2 node executable
# -------------------------------
add_executable(turret_ros2_node
    src/turret_ros2_node.cpp
    src/turret.cpp
    src/spiral_zipper.cpp
    src/servo_city_motor.cpp
    src/encoder.cpp
    src/limit_switch.cpp
    src/cubemars_control.cpp
    src/config.cpp
    src/hx711.cpp
    src/e_stop.cpp
    src/load_cell.cpp
)

target_include_directories(turret_ros2_node PRIVATE include)

ament_target_dependencies(turret_ros2_node
  rclcpp
  std_msgs
  geometry_msgs  
  sensor_msgs
  std_srvs
)

target_link_libraries(turret_ros2_node 
  pigpio 
  yaml-cpp
)

# Link generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(turret_ros2_node "${cpp_typesupport_target}")

# Make sure the executable depends on the generated interfaces
add_dependencies(turret_ros2_node ${PROJECT_NAME})

# -------------------------------
# Legacy standalone executable (optional)
# -------------------------------
add_executable(turret_legacy
    src/turret_controller.cpp
    src/turret.cpp
    src/spiral_zipper.cpp
    src/servo_city_motor.cpp
    src/encoder.cpp
    src/limit_switch.cpp
    src/cubemars_control.cpp
    src/config.cpp
    src/hx711.cpp
    src/e_stop.cpp
    src/load_cell.cpp
)
target_include_directories(turret_legacy PRIVATE include)
target_link_libraries(turret_legacy PRIVATE pigpio yaml-cpp)

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

# -------------------------------
# ROS2 Installation
# -------------------------------
install(TARGETS turret_ros2_node turret_legacy
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
