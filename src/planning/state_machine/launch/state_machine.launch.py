import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('state_machine')
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'state_machine_params.yaml'),
        description='Path to state_machine parameters config file'
    )
    
    state_machine_node = Node(
        package='state_machine',
        name='state_machine',
        executable='state_machine',
        parameters=[LaunchConfiguration('config_file')],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        config_file_arg,
        state_machine_node
    ])
