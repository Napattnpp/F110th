import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def get_launch_source(package_name, base_name):
    try:
        pkg_share = get_package_share_directory(package_name)
    except Exception:
        return None
    py_path = os.path.join(pkg_share, 'launch', f"{base_name}.py")
    xml_path = os.path.join(pkg_share, 'launch', f"{base_name}.xml")
    if os.path.exists(py_path):
        return AnyLaunchDescriptionSource(py_path)
    elif os.path.exists(xml_path):
        return AnyLaunchDescriptionSource(xml_path)
    return AnyLaunchDescriptionSource(py_path)

def package_exists(package_name):
    try:
        get_package_share_directory(package_name)
    except Exception:
        return False
    return True

def generate_launch_description():
    pkg_bringup = get_package_share_directory('race_bringup')

    # Arguments read directly from environment or explicit defaults
    racecar_version_arg = DeclareLaunchArgument(
        'racecar_version',
        default_value=os.environ.get('RACECAR_VERSION', 'ORIN1'),
        description='Hardware profile directory under race_bringup/config/ (e.g. ORIN1, SIM)'
    )
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='ifac_roboracer',
        description='Map target path (e.g. ifac_roboracer or rectangle1)'
    )
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='True/False if you want to run simulation'
    )
    localize_arg = DeclareLaunchArgument(
        'localize',
        default_value='true',
        description='Whether to run absolute map-based localization node'
    )
    localize_method_arg = DeclareLaunchArgument(
        'localize_method',
        default_value='amcl',
        description="Localization method: 'amcl' (Adaptive Monte Carlo) / 'pf' (Particle Filter) / 'cartographer'"
    )

    trajectory_type_arg = DeclareLaunchArgument(
        'trajectory_type',
        default_value='minimum_curvature',
        description='Type of trajectory to publish on /global_waypoints: minimum_curvature, shortest_path, centerline'
    )

    # Configuration files path
    sllidar_s1_config = os.path.join(pkg_bringup, 'config', 'sllidar_s1.yaml')
    mux_config = os.path.join(pkg_bringup, 'config', 'mux.yaml')
    joy_teleop_config = PathJoinSubstitution([pkg_bringup, 'config', LaunchConfiguration('racecar_version'), 'joy_teleop.yaml'])
    vesc_config = PathJoinSubstitution([pkg_bringup, 'config', LaunchConfiguration('racecar_version'), 'vesc.yaml'])
    icm20948_config = PathJoinSubstitution([pkg_bringup, 'config', LaunchConfiguration('racecar_version'), 'icm20948.yaml'])
    l1_params = PathJoinSubstitution([
        pkg_bringup, 'config',
        PythonExpression(["'SIM' if '", LaunchConfiguration('sim'), "'.lower() in ('true', '1') else '", LaunchConfiguration('racecar_version'), "'"]),
        'l1_params.yaml'
    ])
    ekf_config = os.path.join(pkg_bringup, 'config', 'ekf.yaml')

    # Path to map directory
    map_path = PathJoinSubstitution([pkg_bringup, 'maps', LaunchConfiguration('map_name')])

    localize_bool_for_state_estimation = PythonExpression([
        "'True' if '", LaunchConfiguration('localize'), "'.lower() in ['true', '1'] and '", LaunchConfiguration('localize_method'), "' == 'cartographer' else 'False'"
    ])

    # 1. On Sim (sim is True)
    sim_actions = []
    f1tenth_gym_source = get_launch_source('race_bringup', 'gym_bridge_launch')
    if f1tenth_gym_source is not None:
        sim_actions.append(
            IncludeLaunchDescription(
                f1tenth_gym_source,
                launch_arguments={
                    'map_yaml_path': PathJoinSubstitution([map_path, 'clean', [LaunchConfiguration('map_name'), '.yaml']])
                }.items()
            )
        )
    frenet_odom_source = get_launch_source('frenet_odom_republisher', 'frenet_odom_republisher_launch')
    if frenet_odom_source is not None:
        sim_actions.append(
            IncludeLaunchDescription(frenet_odom_source)
        )

    sim_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('sim')),
        actions=sim_actions
    )

    # 2. On Physical System (sim is False)
    physical_actions = []
    bringup_source = get_launch_source('f1tenth_stack', 'bringup_launch')
    if bringup_source is not None:
        physical_actions.append(
            IncludeLaunchDescription(
                bringup_source,
                launch_arguments={
                    'joy_config': joy_teleop_config,
                    'vesc_config': vesc_config,
                    'sllidar_s1_config': sllidar_s1_config,
                    'mux_config': mux_config,
                    'icm20948_config': icm20948_config
                }.items()
            )
        )
    state_estimation_source = get_launch_source('state_estimation', 'state_estimation_launch')
    if state_estimation_source is not None:
        physical_actions.append(
            IncludeLaunchDescription(
                state_estimation_source,
                launch_arguments={
                    'racecar_version': LaunchConfiguration('racecar_version'),
                    'mapping_bool': 'False',
                    'map_name': LaunchConfiguration('map_name'),
                    'frenet_bool': 'True',
                    'slam_config_path': PathJoinSubstitution([pkg_bringup, 'config', LaunchConfiguration('racecar_version'), 'slam']),
                    'localize_bool': localize_bool_for_state_estimation,
                    'ekf_config': ekf_config
                }.items()
            )
        )

    # 4. AMCL Localization (Conditional on localize == True and localize_method == amcl)
    amcl_source = get_launch_source('race_bringup', 'amcl_launch')
    amcl_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('localize'), "'.lower() in ['true', '1'] and '", LaunchConfiguration('localize_method'), "' == 'amcl'"])),
        actions=[
            IncludeLaunchDescription(
                amcl_source,
                launch_arguments={
                    'map_name': LaunchConfiguration('map_name'),
                    'use_sim_time': LaunchConfiguration('sim')
                }.items()
            )
        ]
    )
    physical_actions.append(amcl_group)

    # 5. Particle Filter Localization (Conditional on localize == True and localize_method == pf / particle_filter)
    pf_source = get_launch_source('particle_filter', 'localize_launch')
    if pf_source is not None:
        pf_group = GroupAction(
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('localize'), "'.lower() in ['true', '1'] and '", LaunchConfiguration('localize_method'), "'.lower() in ['pf', 'particle_filter']"])),
            actions=[
                IncludeLaunchDescription(
                    pf_source,
                    launch_arguments={
                        'localize_config': os.path.join(pkg_bringup, 'config', 'localize.yaml'),
                        'map_yaml_path': PathJoinSubstitution([map_path, 'clean', [LaunchConfiguration('map_name'), '.yaml']]),
                        'use_sim_time': LaunchConfiguration('sim')
                    }.items()
                )
            ]
        )
        physical_actions.append(pf_group)


    physical_group = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('sim')),
        actions=physical_actions
    )

    # 3. Common nodes & launches
    global_parameters_node = Node(
        package='race_bringup',
        executable='global_parameter_node',
        name='global_parameters',
        output='screen',
        parameters=[
            l1_params,
            {
                'racecar_version': LaunchConfiguration('racecar_version'),
                'sim': LaunchConfiguration('sim'),
                'map_path': map_path,
                'use_sim_time': LaunchConfiguration('sim')
            }
        ]
    )

    global_trajectory_pub = IncludeLaunchDescription(
        get_launch_source('global_planner', 'global_trajectory_publisher_launch'),
        launch_arguments={
            'map_path': map_path,
            'trajectory_type': LaunchConfiguration('trajectory_type')
        }.items()
    )

    sector_tuner_node = Node(
        package='sector_tuner',
        executable='sector_tuner',
        name='sector_tuner',
        output='screen',
        parameters=[
            PathJoinSubstitution([map_path, 'waypoints', 'speed_scaling.yaml'])
        ]
    )

    optional_nodes = []
    if package_exists('lap_analyser'):
        optional_nodes.append(
            Node(
                package='lap_analyser',
                executable='lap_analyser',
                name='lap_analyser',
                output='screen'
            )
        )

    if package_exists('joy_combo_node'):
        joy_combo_config = PathJoinSubstitution([
            FindPackageShare('joy_combo_node'), 'config', 'joy_combo_params.yaml'
        ])
        optional_nodes.append(
            Node(
                package='joy_combo_node',
                executable='joy_combo_node',
                name='joy_combo_node',
                output='screen',
                parameters=[joy_combo_config]
            )
        )

    return LaunchDescription([
        racecar_version_arg,
        map_name_arg,
        sim_arg,
        localize_arg,
        localize_method_arg,
        trajectory_type_arg,
        sim_group,
        physical_group,
        global_parameters_node,
        global_trajectory_pub,
        sector_tuner_node,
        *optional_nodes
    ])

