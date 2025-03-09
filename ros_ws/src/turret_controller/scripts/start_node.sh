#!/bin/bash
source /home/turret/.bashrc
source /opt/ros/humble/setup.bash
source /home/turret/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=84
ros2 run spiral_zipper_turret turret_node --turret_number 0
