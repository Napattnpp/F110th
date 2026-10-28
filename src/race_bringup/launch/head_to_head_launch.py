import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
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

def generate_launch_description():
    pkg_bringup = get_package_share_directory('race_bringup')

    # Arguments read directly from environment or explicit defaults
    racecar_version_arg = DeclareLaunchArgument(
        'racecar_version',
        default_value=os.environ.get('RACECAR_VERSION', 'ORIN1'),
        description='Hardware profile directory under race_bringup/config/ (e.g. ORIN1, SIM)'
    )
    lu_table_arg = DeclareLaunchArgument(
        'LU_table',
        default_value='ORIN1_hangar_pacejka',
        description='LU table parameter'
    )
    overtake_mode_arg = DeclareLaunchArgument(
        'overtake_mode',
        default_value='spliner',
        description='Overtake mode: spliner / graph_based'
    )
    ctrl_algo_arg = DeclareLaunchArgument(
        'ctrl_algo',
        default_value='MAP',
        description='Control algorithm: MAP / PP'
    )
    measure_arg = DeclareLaunchArgument(
        'measure',
        default_value='False',
        description='Measure execution time'
    )
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='ifac_roboracer',
        description='Target map name (e.g. ifac_roboracer)'
    )
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='True/False if you want to run simulation'
    )
    enable_recovery_arg = DeclareLaunchArgument(
        'enable_recovery',
        default_value='true',
        description='Enable autonomous crash recovery'
    )

    # Include controller manager
    controller_launch = IncludeLaunchDescription(
        get_launch_source('controller', 'controller_launch'),
        launch_arguments={
            'mode': LaunchConfiguration('ctrl_algo'),
            'LU_table': LaunchConfiguration('LU_table'),
            'racecar_version': LaunchConfiguration('racecar_version')
        }.items()
    )

    # Include state machine
    state_machine_launch = IncludeLaunchDescription(
        get_launch_source('state_machine', 'state_machine_launch'),
        launch_arguments={
            'mode': 'head_to_head',
            'overtake_mode': LaunchConfiguration('overtake_mode'),
            'racecar_version': LaunchConfiguration('racecar_version'),
            'enable_recovery': LaunchConfiguration('enable_recovery')
        }.items()
    )

    # Spline Planner (Conditional on overtake_mode == 'spliner')
    spline_planner_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('overtake_mode'), "' == 'spliner'"])),
        actions=[
            IncludeLaunchDescription(
                get_launch_source('spline_planner', 'spline_planner_launch')
            )
        ]
    )

    # Graph-Based Interpolator Node (Conditional on overtake_mode == 'graph_based')
    graph_planner_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('overtake_mode'), "' == 'graph_based'"])),
        actions=[
            Node(
                package='sector_tuner',
                executable='ot_interpolator',
                name='ot_interpolator',
                output='screen',
                parameters=[
                    {'map_name': LaunchConfiguration('map_name')}
                ]
            )
        ]
    )

    # Include Perception
    perception_launch = IncludeLaunchDescription(
        get_launch_source('perception', 'perception_launch'),
        launch_arguments={
            'param_file': os.path.join(pkg_bringup, 'config', 'opponent_tracker_params.yaml'),
            'enable_fake_obstacles': LaunchConfiguration('sim')
        }.items()
    )

    actions = [
        racecar_version_arg,
        lu_table_arg,
        overtake_mode_arg,
        ctrl_algo_arg,
        measure_arg,
        map_name_arg,
        sim_arg,
        enable_recovery_arg,
    ]
    actions.extend([
        controller_launch,
        state_machine_launch,
        spline_planner_group,
        graph_planner_group,
        perception_launch
    ])

    return LaunchDescription(actions)

