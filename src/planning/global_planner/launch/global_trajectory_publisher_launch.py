from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        description='Path to the map file (yaml or similar)'
    )
    
    global_republisher_node = Node(
        package='global_planner',
        executable='global_trajectory_publisher',
        name='global_republisher',
        output='screen',
        parameters=[
            {'map_path': LaunchConfiguration('map_path')},
            {'publish_markers': True},
            {'publish_shortest_path': True},
            {'publish_centerline': False},
            {'publish_lattice': False}
        ]
    )
    
    return LaunchDescription([
        map_path_arg,
        global_republisher_node
    ])
