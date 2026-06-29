from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='spline_planner',
            executable='spline_planner',
            name='spline_planner',
            output='screen'
        )
    ])
