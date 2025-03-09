from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import os
from datetime import datetime

def set_rosbag_filename(context):
    # Define the base directory for storing rosbags
    base_dir = os.path.expanduser('~/turret_rosbags')
    
    # Create the base directory if it doesn't exist
    if not os.path.exists(base_dir):
        os.makedirs(base_dir)
    
    # Generate a timestamped rosbag file name
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    rosbag_file = os.path.join(base_dir, f'rosbag_{timestamp}')

    # Set the filename as a launch configuration to use in the ExecuteProcess
    context.launch_configurations['rosbag_file'] = rosbag_file
    return []

def generate_launch_description():
    # Declare the 'turret_number' argument with a default value
    turret_number_arg = DeclareLaunchArgument(
        'turret_number',
        default_value='0',
        description='Turret number for the node instance'
    )

    # Declare an argument to enable or disable rosbag recording
    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='true',
        description='Enable rosbag recording'
    )

    # Configure the Node action with the turret_number argument
    turret_node = Node(
        package='spiral_zipper_turret',
        executable='turret_node',
        output='screen',
        arguments=['--turret_number', LaunchConfiguration('turret_number')]
    )

    # Set rosbag filename with timestamp and store it in context
    set_rosbag_file_action = OpaqueFunction(function=set_rosbag_filename)

    # Rosbag Record Process (records all topics if enabled)
    record_bag = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record_bag')),
        cmd=[
            'ros2', 'bag', 'record', '-a',
            '-o', LaunchConfiguration('rosbag_file')  # Use the dynamically generated filename
        ],
        output='screen'
    )

    return LaunchDescription([
        turret_number_arg,
        record_bag_arg,
        set_rosbag_file_action,
        turret_node,
        record_bag
    ])
