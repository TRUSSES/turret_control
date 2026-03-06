#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    zero_velocity_arg = DeclareLaunchArgument(
        'zero_velocity',
        default_value='-0.2',
        description='Velocity for zeroing operation (rad/s)'
    )

    # Rover base node
    turret_node = Node(
        package='turret_control',
        executable='turret_ros2_node',
        output='screen',
        parameters=[
            {
                'zero_velocity': LaunchConfiguration('zero_velocity')
            }
        ]
    )

    shutdown_on_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=turret_node,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )

    return LaunchDescription([
        zero_velocity_arg,
        turret_node,
        shutdown_on_exit,
    ])
