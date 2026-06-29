from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    lap_analyser_node = Node(
        package='lap_analyser',
        executable='lap_analyser',
        name='lap_analyser',
        output='screen'
    )
    
    return LaunchDescription([
        lap_analyser_node
    ])
