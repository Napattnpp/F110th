import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

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

    try:
        from race_bringup.profile_loader import load_active_profile
        profile = load_active_profile()
    except ImportError:
        profile = {}

    # Arguments
    racecar_version_arg = DeclareLaunchArgument(
        'racecar_version',
        default_value=profile.get('racecar_version', 'NUC1'),
        description='NUC version (e.g. NUC1)'
    )
    lu_table_arg = DeclareLaunchArgument(
        'LU_table',
        default_value=profile.get('LU_table', 'NUC1_hangar_pacejka'),
        description='LU table parameter'
    )
    overtake_mode_arg = DeclareLaunchArgument(
        'overtake_mode',
        default_value=profile.get('overtake_mode', 'spliner'),
        description='Overtake mode: spliner / graph_based'
    )
    ctrl_algo_arg = DeclareLaunchArgument(
        'ctrl_algo',
        default_value=profile.get('ctrl_algo', 'MAP'),
        description='Control algorithm: MAP / PP'
    )
    measure_arg = DeclareLaunchArgument(
        'measure',
        default_value='False',
        description='Measure execution time'
    )
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value=str(profile.get('sim', False)),
        description='True/False if you want to run simulation'
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
            'racecar_version': LaunchConfiguration('racecar_version')
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
        sim_arg,
    ]
    actions.extend([
        controller_launch,
        state_machine_launch,
        spline_planner_group,
        perception_launch
    ])

    return LaunchDescription(actions)
