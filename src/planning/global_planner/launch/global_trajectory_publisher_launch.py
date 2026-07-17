from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        description='Path to the map file (yaml or similar)'
    )
    
    trajectory_type_arg = DeclareLaunchArgument(
        'trajectory_type',
        default_value='minimum_curvature',
        description='Type of trajectory to publish on /global_waypoints: minimum_curvature, shortest_path, centerline'
    )
    
    global_republisher_node = Node(
        package='global_planner',
        executable='global_trajectory_publisher',
        name='global_republisher',
        output='screen',
        parameters=[
            {'map_path': LaunchConfiguration('map_path')},
            {'trajectory_type': LaunchConfiguration('trajectory_type')},
            {'publish_markers': True},
            {'publish_shortest_path': True},
            {'publish_centerline': False},
            {'publish_lattice': False}
        ]
    )
    
    return LaunchDescription([
        map_path_arg,
        trajectory_type_arg,
        global_republisher_node
    ])
