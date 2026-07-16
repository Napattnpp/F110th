import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_bringup_share = get_package_share_directory('race_bringup')
    default_param_file = os.path.join(pkg_bringup_share, 'config', 'spline_planner_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_file',
            default_value=default_param_file,
            description='Path to parameter file for spline_planner'
        ),
        Node(
            package='spline_planner',
            executable='spline_planner',
            name='spline_planner',
            output='screen',
            parameters=[LaunchConfiguration('param_file')]
        )
    ])

