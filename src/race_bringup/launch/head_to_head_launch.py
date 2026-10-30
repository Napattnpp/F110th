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
        default_value='',
        description='Target map name (optional, only needed for legacy graph_based overtake_mode)'
    )
    fake_obstacles_arg = DeclareLaunchArgument(
        'fake_obstacles',
        default_value='false',
        description='Publish synthetic fake obstacles for standalone tracker benchmarking'
    )
    enable_recovery_arg = DeclareLaunchArgument(
        'enable_recovery',
        default_value='true',
        description='Enable autonomous crash recovery'
    )
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Simulation mode: true/false'
    )

    resolved_racecar_version = PythonExpression([
        "'SIM' if '", LaunchConfiguration('sim'), "'.lower() in ('true', '1') or '", LaunchConfiguration('racecar_version'), "' == 'SIM' else '", LaunchConfiguration('racecar_version'), "'"
    ])
    resolved_lu_table = PythonExpression([
        "'SIM_linear' if '", LaunchConfiguration('sim'), "'.lower() in ('true', '1') or '", LaunchConfiguration('racecar_version'), "' == 'SIM' else '", LaunchConfiguration('LU_table'), "'"
    ])

    # Include controller manager
    controller_source = get_launch_source('controller', 'controller_launch')
    controller_launch = None
    if controller_source is not None:
        controller_launch = IncludeLaunchDescription(
            controller_source,
            launch_arguments={
                'mode': LaunchConfiguration('ctrl_algo'),
                'LU_table': resolved_lu_table,
                'racecar_version': resolved_racecar_version
            }.items()
        )

    # Include state machine
    state_machine_source = get_launch_source('state_machine', 'state_machine_launch')
    state_machine_launch = None
    if state_machine_source is not None:
        state_machine_launch = IncludeLaunchDescription(
            state_machine_source,
            launch_arguments={
                'param_file': os.path.join(pkg_bringup, 'config', 'state_machine_params.yaml'),
                'mode': 'head_to_head',
                'overtake_mode': LaunchConfiguration('overtake_mode'),
                'racecar_version': resolved_racecar_version,
                'enable_recovery': LaunchConfiguration('enable_recovery')
            }.items()
        )

    # Spline Planner (Conditional on overtake_mode == 'spliner')
    spline_planner_source = get_launch_source('spline_planner', 'spline_planner_launch')
    spline_planner_group = None
    if spline_planner_source is not None:
        spline_planner_group = GroupAction(
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('overtake_mode'), "' == 'spliner'"])),
            actions=[
                IncludeLaunchDescription(spline_planner_source)
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
    perception_source = get_launch_source('perception', 'perception_launch')
    perception_launch = None
    if perception_source is not None:
        perception_launch = IncludeLaunchDescription(
            perception_source,
            launch_arguments={
                'param_file': os.path.join(pkg_bringup, 'config', 'opponent_tracker_params.yaml'),
                'enable_fake_obstacles': LaunchConfiguration('fake_obstacles')
            }.items()
        )

    actions = [
        racecar_version_arg,
        lu_table_arg,
        sim_arg,
        overtake_mode_arg,
        ctrl_algo_arg,
        measure_arg,
        map_name_arg,
        fake_obstacles_arg,
        enable_recovery_arg,
    ]
    if controller_launch is not None:
        actions.append(controller_launch)
    if state_machine_launch is not None:
        actions.append(state_machine_launch)
    if spline_planner_group is not None:
        actions.append(spline_planner_group)
    actions.append(graph_planner_group)
    if perception_launch is not None:
        actions.append(perception_launch)

    return LaunchDescription(actions)

