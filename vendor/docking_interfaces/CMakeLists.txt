cmake_minimum_required(VERSION 3.8)
project(docking)

# Keep the existing docking/msg wire types while building inside turret_control.
# This subproject contains interfaces only; it does not build the docking node.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DockingCommand.msg"
  "msg/DockingState.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
