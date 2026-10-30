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
                'mode': 'timetrials',
                'overtake_mode': 'spliner',
                'racecar_version': resolved_racecar_version,
                'enable_recovery': LaunchConfiguration('enable_recovery')
            }.items()
        )

    # Perception and spline planner are disabled for solo time trials

    actions = [
        racecar_version_arg,
        lu_table_arg,
        sim_arg,
        ctrl_algo_arg,
        measure_arg,
        enable_recovery_arg,
    ]
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
