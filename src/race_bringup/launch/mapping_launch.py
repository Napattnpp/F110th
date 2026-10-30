import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

def get_launch_source(package_name, base_name):
    try:
        pkg_share = get_package_share_directory(package_name)
    except Exception:
        # Fallback if package is not built/found yet in the path
        return None
    py_path = os.path.join(pkg_share, 'launch', f"{base_name}.py")
    xml_path = os.path.join(pkg_share, 'launch', f"{base_name}.xml")
    if os.path.exists(py_path):
        return AnyLaunchDescriptionSource(py_path)
    elif os.path.exists(xml_path):
        return AnyLaunchDescriptionSource(xml_path)
    return AnyLaunchDescriptionSource(py_path)

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
        default_value='rectangle1',
        description='Name of the map to be created'
    )
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Whether to run in simulation mode'
    )
    slam_method_arg = DeclareLaunchArgument(
        'slam_method',
        default_value='cartographer',
        description="SLAM method: 'cartographer' / 'slam_toolbox'"
    )
    run_ftg_arg = DeclareLaunchArgument(
        'run_ftg',
        default_value='False',
        description='Whether to run ftg_mapper node'
    )

    # Configuration files path
    sllidar_s1_config = os.path.join(pkg_bringup, 'config', 'sllidar_s1.yaml')
    mux_config = os.path.join(pkg_bringup, 'config', 'mux.yaml')
    joy_teleop_config = PathJoinSubstitution([pkg_bringup, 'config', LaunchConfiguration('racecar_version'), 'joy_teleop.yaml'])
    vesc_config = PathJoinSubstitution([pkg_bringup, 'config', LaunchConfiguration('racecar_version'), 'vesc.yaml'])
    icm20948_config = PathJoinSubstitution([pkg_bringup, 'config', LaunchConfiguration('racecar_version'), 'icm20948.yaml'])
    global_planner_params = os.path.join(pkg_bringup, 'config', 'global_planner', 'global_planner_params.yaml')
    ekf_config = os.path.join(pkg_bringup, 'config', 'ekf.yaml')

    # Crawl backwards to find maps in the src folder
    # In ROS 2, get_package_share_directory returns the installed share directory.
    # Note: we use LaunchConfiguration and PathJoinSubstitution to handle this dynamically.
    map_src_path = PathJoinSubstitution([
        pkg_bringup, 'maps', LaunchConfiguration('map_name')
    ])

    # 1. On Physical System (sim is False)
    physical_actions = []
    
    # Bringup launch
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
        
    # ftg_mapper node to drive around during mapping (optional)
    physical_actions.append(
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='controller',
                    executable='ftg_mapper',
                    name='ftg_mapper',
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('run_ftg'))
                )
            ]
        )
    )
        
    # State estimation launch
    state_estimation_source = get_launch_source('state_estimation', 'state_estimation_launch')
    
    # Dynamic path for SLAM config
    slam_config_path = PathJoinSubstitution([
        pkg_bringup, 'config', LaunchConfiguration('racecar_version'), 'slam'
    ])

    # Dynamic path for slam_toolbox yaml config
    slam_toolbox_config = PathJoinSubstitution([
        slam_config_path, 'f1tenth_online_async.yaml'
    ])

    if state_estimation_source is not None:
        # Include state estimation (Cartographer mapping mode)
        cartographer_mapping = GroupAction(
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('slam_method'), "' == 'cartographer'"])),
            actions=[
                IncludeLaunchDescription(
                    state_estimation_source,
                    launch_arguments={
                        'racecar_version': LaunchConfiguration('racecar_version'),
                        'mapping_bool': 'True',
                        'map_name': LaunchConfiguration('map_name'),
                        'frenet_bool': 'False',
                        'slam_config_path': slam_config_path,
                        'ekf_config': ekf_config
                    }.items()
                )
            ]
        )
        physical_actions.append(cartographer_mapping)

        # Include state estimation (Slam Toolbox mode - Cartographer disabled)
        slam_toolbox_source = get_launch_source('slam_toolbox', 'online_async_launch')
        if slam_toolbox_source is not None:
            slam_toolbox_mapping = GroupAction(
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration('slam_method'), "' == 'slam_toolbox'"])),
                actions=[
                    IncludeLaunchDescription(
                        state_estimation_source,
                        launch_arguments={
                            'racecar_version': LaunchConfiguration('racecar_version'),
                            'mapping_bool': 'False',
                            'localize_bool': 'False',
                            'map_name': LaunchConfiguration('map_name'),
                            'frenet_bool': 'False',
                            'ekf_config': ekf_config
                        }.items()
                    ),
                    IncludeLaunchDescription(
                        slam_toolbox_source,
                        launch_arguments={
                            'slam_params_file': slam_toolbox_config,
                            'use_sim_time': LaunchConfiguration('sim')
                        }.items()
                    )
                ]
            )
            physical_actions.append(slam_toolbox_mapping)

    physical_group = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('sim')),
        actions=physical_actions
    )

    # 2. On Sim (sim is True)
    sim_actions = []
    
    # Gym bridge launch
    gym_bridge_source = get_launch_source('race_bringup', 'gym_bridge_launch')
    if gym_bridge_source is not None:
        sim_actions.append(
            IncludeLaunchDescription(
                gym_bridge_source,
                launch_arguments={
                    'map_yaml_path': PathJoinSubstitution([
                        map_src_path, 'clean',
                        PythonExpression(["'", LaunchConfiguration('map_name'), "' + '.yaml'"])
                    ])
                }.items()
            )
        )

    sim_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('sim')),
        actions=sim_actions
    )

    ld = LaunchDescription([
        racecar_version_arg,
        map_name_arg,
        sim_arg,
        slam_method_arg,
        run_ftg_arg,
        physical_group,
        sim_group
    ])

    return ld
