#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value='config/config.yaml',
        description='Path to turret configuration file'
    )
    
    turret_id_arg = DeclareLaunchArgument(
        'turret_id',
        default_value='1',
        description='Turret ID for topic namespace'
    )
    
    zero_velocity_arg = DeclareLaunchArgument(
        'zero_velocity',
        default_value='-0.2',
        description='Velocity for zeroing operation (rad/s)'
    )

    # Turret control node
    turret_node = Node(
        package='turret_control',
        executable='turret_ros2_node',
        output='both',
        parameters=[
            {
                'config_path': LaunchConfiguration('config_path'),
                'zero_velocity': LaunchConfiguration('zero_velocity')
            }
        ]
    )

    return LaunchDescription([
        config_path_arg,
        turret_id_arg,
        zero_velocity_arg,
        
        # Log startup information
        LogInfo(msg=['Starting Turret Control System (ID: ', LaunchConfiguration('turret_id'), ')']),
        LogInfo(msg=['Using config file: ', LaunchConfiguration('config_path')]),
        LogInfo(msg=['Zero velocity: ', LaunchConfiguration('zero_velocity'), ' rad/s']),
        LogInfo(msg='GPS functionality disabled - use turret_simple.launch.py after installing ublox_gps'),
        
        # Launch turret node only
        turret_node,
    ])
