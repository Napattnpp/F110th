#!/usr/bin/env python3
import os
import pathlib
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def _find_repo_root() -> pathlib.Path:
    curr = pathlib.Path(__file__).resolve().parent
    for _ in range(10):
        if (curr / 'assets' / 'maps').exists() or (curr / '.git').exists():
            return curr
        if curr.parent == curr:
            break
        curr = curr.parent

    cwd = pathlib.Path.cwd()
    for _ in range(10):
        if (cwd / 'assets' / 'maps').exists() or (cwd / '.git').exists():
            return cwd
        if cwd.parent == cwd:
            break
        cwd = cwd.parent

    return pathlib.Path.cwd()

def _resolve_map_dir(map_name_str: str) -> pathlib.Path:
    repo_root = _find_repo_root()
    p = pathlib.Path(map_name_str)
    stem = p.name
    candidates = [
        repo_root / 'assets' / 'maps' / map_name_str,
        repo_root / 'assets' / 'maps' / stem,
    ]
    for cand in candidates:
        if cand.exists():
            return cand.resolve()
    return (repo_root / 'assets' / 'maps' / 'ifac_roboracer').resolve()

def launch_setup(context, *args, **kwargs):
    ld = LaunchDescription()
    pkg_bringup = get_package_share_directory('race_bringup')
    pkg_global_planner = get_package_share_directory('global_planner')
    pkg_frenet = get_package_share_directory('frenet_odom_republisher')
    pkg_pf = get_package_share_directory('particle_filter')

    mode = LaunchConfiguration('mode')
    map_name = LaunchConfiguration('map_name')
    loc = LaunchConfiguration('loc')
    planner = LaunchConfiguration('planner')
    controller = LaunchConfiguration('controller')

    map_name_str = map_name.perform(context)
    map_dir = _resolve_map_dir(map_name_str)
    wp_dir = map_dir / 'waypoints' if (map_dir / 'waypoints').exists() else map_dir

    speed_scaling_yaml = str(wp_dir / 'speed_scaling.yaml') if (wp_dir / 'speed_scaling.yaml').exists() else str(map_dir / 'speed_scaling.yaml')
    ot_sectors_yaml = str(wp_dir / 'ot_sectors.yaml') if (wp_dir / 'ot_sectors.yaml').exists() else str(map_dir / 'ot_sectors.yaml')

    # 1. Base Hardware or Gym Bridge Launch
    base_sim = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'gym_bridge_launch.py')),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"])),
        launch_arguments={
            'map_name': map_name,
            'launch_map_server': PythonExpression(["'false' if '", loc, "' != 'none' else 'true'"])
        }.items()
    )

    base_car = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'base_launch.py')),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'car'"])),
        launch_arguments={'map_name': map_name}.items()
    )

    # 2. Localization Backends
    amcl_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'amcl_launch.py')),
        condition=IfCondition(PythonExpression(["'", loc, "' == 'amcl'"])),
        launch_arguments={'map_name': map_name}.items()
    )

    pf_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(pkg_pf, 'launch', 'localize_launch.py')),
        condition=IfCondition(PythonExpression(["'", loc, "'.lower() in ['pf', 'particle_filter']"])),
        launch_arguments={'map_name': map_name}.items()
    )

    # 3. Global Parameters & Trajectory Publisher
    global_parameters_node = Node(
        package='race_bringup',
        executable='global_parameter_node',
        name='global_parameters',
        output='screen',
        parameters=[{
            'racecar_version': 'ORIN1',
            'sim': PythonExpression(["True if '", mode, "' == 'sim' else False"]),
            'map_path': str(map_dir),
            'use_sim_time': PythonExpression(["True if '", mode, "' == 'sim' else False"])
        }]
    )

    global_trajectory_pub = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(pkg_global_planner, 'launch', 'global_trajectory_publisher_launch.py')),
        launch_arguments={
            'map_path': str(map_dir),
            'trajectory_type': 'minimum_curvature'
        }.items()
    )

    frenet_odom_pub = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(pkg_frenet, 'launch', 'frenet_odom_republisher_launch.py'))
    )

    # 4. Speed Scaling & OT Sector Tuners
    sector_tuner_params = [
        {'map_name': map_name_str}
    ]
    if os.path.exists(speed_scaling_yaml):
        sector_tuner_params.insert(0, speed_scaling_yaml)

    sector_tuner_node = Node(
        package='sector_tuner',
        executable='sector_tuner',
        name='sector_tuner',
        output='screen',
        parameters=sector_tuner_params
    )

    ot_interpolator_params = [
        {'map_name': map_name_str}
    ]
    if os.path.exists(ot_sectors_yaml):
        ot_interpolator_params.insert(0, ot_sectors_yaml)

    ot_interpolator_node = Node(
        package='sector_tuner',
        executable='ot_interpolator',
        name='ot_interpolator',
        output='screen',
        parameters=ot_interpolator_params
    )

    # 5. State Machine Node
    state_machine_node = Node(
        package='state_machine',
        executable='state_machine',
        name='state_machine',
        output='screen',
        parameters=[
            os.path.join(pkg_bringup, 'config', 'state_machine_params.yaml'),
            {'map_name': map_name}
        ]
    )

    # 6. Controller Manager
    controller_node = Node(
        package='controller',
        executable='controller',
        name='controller_manager',
        output='screen',
        parameters=[{
            'algorithm': controller,
            'map_name': map_name,
        }],
        condition=IfCondition(PythonExpression(["'", controller, "' != 'none'"]))
    )

    ld.add_action(LogInfo(msg=PythonExpression(["'Starting F1TENTH in mode=", mode, " with map=", map_name, " (loc=", loc, ", planner=", planner, ", controller=", controller, ")'"])))
    ld.add_action(base_sim)
    ld.add_action(base_car)
    ld.add_action(amcl_node)
    ld.add_action(pf_node)
    ld.add_action(global_parameters_node)
    ld.add_action(global_trajectory_pub)
    ld.add_action(frenet_odom_pub)
    ld.add_action(sector_tuner_node)
    ld.add_action(ot_interpolator_node)
    ld.add_action(state_machine_node)
    ld.add_action(controller_node)

    return ld.entities

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='sim',
            description='Execution mode: sim or car'
        ),
        DeclareLaunchArgument(
            'map_name',
            default_value='ifac_roboracer',
            description='Map target (e.g. ifac_roboracer or rectangle1)'
        ),
        DeclareLaunchArgument(
            'loc',
            default_value='pf',
            description='Localization backend: pf (Particle Filter), amcl, cartographer, or none'
        ),
        DeclareLaunchArgument(
            'planner',
            default_value='global',
            description='Planner module: global, spline, or none'
        ),
        DeclareLaunchArgument(
            'controller',
            default_value='pp',
            description='Control algorithm: pp (Pure Pursuit), mpc, mpcc, ftg, or none'
        ),
        OpaqueFunction(function=launch_setup)
    ])
