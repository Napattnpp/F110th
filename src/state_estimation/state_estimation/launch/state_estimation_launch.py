import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    pkg_state_estimation = get_package_share_directory('state_estimation')
    try:
        pkg_bringup = get_package_share_directory('race_bringup')
    except Exception:
        pkg_bringup = None

    # Declare arguments
    mapping_bool_arg = DeclareLaunchArgument(
        'mapping_bool',
        default_value='True',
        description='Whether to run in mapping mode'
    )
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='hangar',
        description='Name of the map'
    )
    frenet_bool_arg = DeclareLaunchArgument(
        'frenet_bool',
        default_value='False',
        description='Whether to republish in frenet frame'
    )
    skip_early_fusion_arg = DeclareLaunchArgument(
        'skip_early_fusion',
        default_value='False',
        description='Skip EKF IMU + Odom fusion'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation clock'
    )
    slam_config_path_arg = DeclareLaunchArgument(
        'slam_config_path',
        default_value=os.path.join(pkg_state_estimation, 'config', 'cartographer'),
        description='Path to cartographer config directory'
    )
    localize_bool_arg = DeclareLaunchArgument(
        'localize_bool',
        default_value='True',
        description='Whether to run absolute map-based localization node'
    )
    map_dir_arg = DeclareLaunchArgument(
        'map_dir',
        default_value=os.path.join(pkg_bringup, 'maps') if pkg_bringup else os.path.join(pkg_state_estimation, 'maps'),
        description='Directory containing the map subfolders'
    )
    ekf_config_arg = DeclareLaunchArgument(
        'ekf_config',
        default_value=os.path.join(pkg_state_estimation, 'config', 'ekf.yaml'),
        description='Path to EKF configuration file'
    )

    # Launch Configurations
    mapping_bool = LaunchConfiguration('mapping_bool')
    map_name = LaunchConfiguration('map_name')
    frenet_bool = LaunchConfiguration('frenet_bool')
    skip_early_fusion = LaunchConfiguration('skip_early_fusion')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_config_path = LaunchConfiguration('slam_config_path')
    localize_bool = LaunchConfiguration('localize_bool')
    map_dir = LaunchConfiguration('map_dir')
    ekf_config = LaunchConfiguration('ekf_config')

    # Path to map directory
    map_path = PathJoinSubstitution([map_dir, map_name])

    # 1. EKF Fusion Group
    ekf_group = GroupAction(
        condition=UnlessCondition(skip_early_fusion),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_state_estimation, 'launch', 'ekf_launch.py')
                ),
                launch_arguments={
                    'ekf_config': ekf_config
                }.items()
            )
        ]
    )

    # 2. Localization Group (when mapping_bool is False AND localize_bool is True)
    localization_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", mapping_bool, "' == 'False' and '", localize_bool, "' == 'True'"])),
        actions=[
            # Cartographer localization node
            Node(
                package='cartographer_ros',
                executable='cartographer_node',
                name='cartographer_node',
                output='screen',
                arguments=[
                    '-configuration_directory', slam_config_path,
                    '-configuration_basename', 'f110_2d_loc.lua',
                    '-load_state_filename', [map_path, '/', map_name, '.pbstream'],
                    '--minloglevel', '2'
                ],
                remappings=[('odom', 'early_fusion/odom'), ('imu', '/imu/data_raw')],
                parameters=[{'use_sim_time': use_sim_time}]
            ),
            # Cartographer occupancy grid node
            Node(
                package='cartographer_ros',
                executable='cartographer_occupancy_grid_node',
                name='cartographer_occupancy_grid_node',
                output='screen',
                arguments=['-resolution', '0.05'],
                parameters=[{'use_sim_time': use_sim_time}]
            ),
            # Nav2 Map Server
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[
                    {'yaml_filename': [map_path, '/', map_name, '.yaml']},
                    {'use_sim_time': use_sim_time}
                ]
            ),
            # Nav2 Lifecycle Manager
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager',
                output='screen',
                parameters=[
                    {'autostart': True},
                    {'node_names': ['map_server']},
                    {'use_sim_time': use_sim_time}
                ]
            )
        ]
    )

    # 3. Mapping Group (when mapping_bool is True)
    mapping_group = GroupAction(
        condition=IfCondition(mapping_bool),
        actions=[
            # Cartographer mapping node
            Node(
                package='cartographer_ros',
                executable='cartographer_node',
                name='cartographer_node',
                output='screen',
                arguments=[
                    '-configuration_directory', slam_config_path,
                    '-configuration_basename', 'f110_2d.lua',
                    '--minloglevel', '2'
                ],
                remappings=[('odom', 'early_fusion/odom'), ('imu', '/imu/data_raw')],
                parameters=[{'use_sim_time': use_sim_time}]
            ),
            # Cartographer occupancy grid node
            Node(
                package='cartographer_ros',
                executable='cartographer_occupancy_grid_node',
                name='cartographer_occupancy_grid_node',
                output='screen',
                arguments=['-resolution', '0.05'],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    # 4. Carstate Node
    carstate_node = Node(
        package='state_estimation',
        executable='carstate_node',
        name='carstate_node',
        output='screen',
        remappings=[('/odom_out', '/car_state/odom')],
        parameters=[
            {'frenet_bool': frenet_bool},
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        mapping_bool_arg,
        map_name_arg,
        frenet_bool_arg,
        skip_early_fusion_arg,
        use_sim_time_arg,
        slam_config_path_arg,
        localize_bool_arg,
        map_dir_arg,
        ekf_config_arg,
        ekf_group,
        localization_group,
        mapping_group,
        carstate_node
    ])
