import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('joy_combo_node')
    default_config = os.path.join(pkg_share, 'config', 'joy_combo_params.yaml')

    config_arg = DeclareLaunchArgument(
        'joy_combo_config',
        default_value=default_config,
        description='Path to joy_combo_node configuration YAML'
    )

    node = Node(
        package='joy_combo_node',
        executable='joy_combo_node',
        name='joy_combo_node',
        output='screen',
        parameters=[LaunchConfiguration('joy_combo_config')]
    )

    return LaunchDescription([
        config_arg,
        node
    ])

