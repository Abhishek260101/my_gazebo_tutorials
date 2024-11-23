from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from datetime import datetime
import os

def generate_launch_description():
    # Create the launch configuration variables
    record_bag = LaunchConfiguration('record_bag')
    bag_path = LaunchConfiguration('bag_path')

    # Declare the launch arguments
    declare_record_bag_cmd = DeclareLaunchArgument(
        'record_bag',
        default_value='false',
        description='Whether to record all topics to a bag file'
    )

    # Get the current timestamp for the bag file name
    timestamp = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    default_bag_path = os.path.join(os.getcwd(), 'bags', f'walker_recording_{timestamp}')

    declare_bag_path_cmd = DeclareLaunchArgument(
        'bag_path',
        default_value=default_bag_path,
        description='Path to store the bag file'
    )

    # Create the walker node
    walker_node = Node(
        package='walker',
        executable='walker_node',
        name='walker_node',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    # Create the rosbag recording process
    # Record all topics except /camera/* topics to save space
    recorder_node = ExecuteProcess(
        condition=IfCondition(record_bag),
        cmd=['ros2', 'bag', 'record',
             '-o', bag_path,
             '-a',               # Record all topics
             '--exclude', '/camera/*',  # Exclude camera topics
             '--compression-mode', 'file',
             '--compression-format', 'zstd'
             ],
        output='screen'
    )

    # Create the launch description and add the actions
    ld = LaunchDescription()

    # Add the launch arguments
    ld.add_action(declare_record_bag_cmd)
    ld.add_action(declare_bag_path_cmd)

    # Add the nodes
    ld.add_action(walker_node)
    ld.add_action(recorder_node)

    return ld