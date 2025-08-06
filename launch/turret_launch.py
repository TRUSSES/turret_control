#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    # Load GPS config
    config_directory = os.path.join( 
        get_package_share_directory('ublox_gps'), 
        'config' 
    ) 
    params = os.path.join(config_directory, 'zed_f9p.yaml') 
 
    # GPS node 
    ublox_gps_node_spirit = Node( 
         package='ublox_gps', 
         executable='ublox_gps_node', 
         name='ublox_gps_node_spirit', 
         output='both', 
         parameters=[params], 
    ) 

    # Rover base node
    turret_node = Node(
        package='turret_control',
        executable='turret_ros2_node',
        name='turret_control_node',
        output='screen',
    )

    shutdown_on_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=turret_node,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )

    return LaunchDescription([
        ublox_gps_node_spirit,  # Commented out for now
        turret_node,
        shutdown_on_exit,
    ])

