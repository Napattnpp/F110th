import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package path
    pkg_share = get_package_share_directory('race_bringup')

    # Declare launch arguments
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='latest',
        description='Name of the map directory under race_bringup/maps'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gym/Gazebo) clock if true'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'amcl.yaml'),
        description='Full path to the ROS2 parameters file to use for AMCL'
    )

    # Path to map yaml: race_bringup/maps/<map_name>/<map_name>.yaml
    map_yaml_path = [
        pkg_share,
        '/maps/',
        LaunchConfiguration('map_name'),
        '/',
        LaunchConfiguration('map_name'),
        '.yaml'
    ]

    # Nodes
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml_path},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    return LaunchDescription([
        map_name_arg,
        use_sim_time_arg,
        params_file_arg,
        map_server_node,
        amcl_node,
        lifecycle_manager_node
    ])
