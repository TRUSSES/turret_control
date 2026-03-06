#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value='config/config.yaml',
        description='Path to turret configuration file'
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

    # Load GPS config
    config_directory = os.path.join(
        get_package_share_directory('ublox_gps'),
        'config'
    )
    params = os.path.join(config_directory, 'zed_f9p.yaml')

    # GPS node
    ublox_gps_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node_turret',
        output='both',
        parameters=[params],
    )

    shutdown_on_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rover_base_node,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )


    return LaunchDescription([
        config_path_arg,
        zero_velocity_arg,
        
        # Log startup information
        LogInfo(msg=['Starting Turret Control System']),
        LogInfo(msg=['Using config file: ', LaunchConfiguration('config_path')]),
        LogInfo(msg=['Zero velocity: ', LaunchConfiguration('zero_velocity'), ' rad/s']),
        
        # Launch nodes
        turret_node,
        ublox_gps_node,
        shutdown_on_exit,
    ])
