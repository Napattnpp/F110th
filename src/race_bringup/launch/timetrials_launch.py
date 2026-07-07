import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter

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
    launch_base_arg = DeclareLaunchArgument(
        'launch_base',
        default_value='True',
        description='Whether to also launch base_launch.py'
    )
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value=profile.get('map_name', 'latest'),
        description='Map name in race_bringup/maps'
    )
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value=str(profile.get('sim', False)),
        description='True/False if you want to run simulation'
    )

    # Include controller manager
    controller_source = get_launch_source('controller', 'controller_launch')
    controller_launch = None
    if controller_source is not None:
        controller_launch = IncludeLaunchDescription(
            controller_source,
            launch_arguments={
                'mode': LaunchConfiguration('ctrl_algo'),
                'LU_table': LaunchConfiguration('LU_table'),
                'racecar_version': LaunchConfiguration('racecar_version')
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
                'mode': 'timetrials',
                'overtake_mode': 'spliner',
                'racecar_version': LaunchConfiguration('racecar_version')
            }.items()
        )

    # Perception and spline planner are disabled for time trials

    # Include base system launch (sensors, simulator, etc.) conditionally
    base_launch = None
    base_launch_source = get_launch_source('race_bringup', 'base_launch')
    if base_launch_source is not None:
        base_launch = GroupAction(
            condition=IfCondition(LaunchConfiguration('launch_base')),
            actions=[
                IncludeLaunchDescription(
                    base_launch_source,
                    launch_arguments={
                        'racecar_version': LaunchConfiguration('racecar_version'),
                        'map_name': LaunchConfiguration('map_name'),
                        'sim': LaunchConfiguration('sim')
                    }.items()
                )
            ]
        )

    actions = [
        racecar_version_arg,
        lu_table_arg,
        ctrl_algo_arg,
        measure_arg,
        launch_base_arg,
        map_name_arg,
        sim_arg,
    ]
    if base_launch is not None:
        actions.append(base_launch)
    if controller_launch is not None:
        actions.append(controller_launch)
    if state_machine_launch is not None:
        actions.append(state_machine_launch)
    # Disabled for time trials
    # if perception_launch is not None:
    #     actions.append(perception_launch)
    # if spline_planner_launch is not None:
    #     actions.append(spline_planner_launch)

    return LaunchDescription(actions)
