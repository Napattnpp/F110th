import os
import pathlib
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _resolve_map_yaml_path(map_name_str: str, pkg_share: str) -> str:
    # If explicit full path is passed
    if map_name_str.startswith('/'):
        p = pathlib.Path(map_name_str)
        return str(p if p.suffix == '.yaml' else p / f"{p.stem}.yaml")

    # Search workspace source assets/maps directory first
    try:
        share_dir = pathlib.Path(pkg_share)
        for parent in share_dir.parents:
            if (parent / 'assets' / 'maps').is_dir():
                maps_dir = parent / 'assets' / 'maps'
                # Check direct path under assets/maps
                cand = maps_dir / map_name_str
                if cand.is_dir():
                    # 1. Prioritize clean/ subfolder
                    clean_yaml = cand / 'clean' / f"{cand.name}.yaml"
                    if clean_yaml.exists():
                        return str(clean_yaml)
                    # 2. Check top-level map directory
                    yaml_cand = cand / f"{cand.name}.yaml"
                    if yaml_cand.exists():
                        return str(yaml_cand)

                # Check recursive search
                target_stem = pathlib.Path(map_name_str).name
                for root, dirs, files in os.walk(maps_dir):
                    rpath = pathlib.Path(root)
                    if rpath.name == target_stem:
                        clean_yaml = rpath / 'clean' / f"{target_stem}.yaml"
                        if clean_yaml.exists():
                            return str(clean_yaml)
                        if (rpath / f"{target_stem}.yaml").exists():
                            return str(rpath / f"{target_stem}.yaml")
    except Exception:
        pass

    # Check share directory
    stem = pathlib.Path(map_name_str).name
    share_clean = pathlib.Path(pkg_share) / 'maps' / map_name_str / 'clean' / f"{stem}.yaml"
    if share_clean.exists():
        return str(share_clean)
    share_raw = pathlib.Path(pkg_share) / 'maps' / map_name_str / f"{stem}.yaml"
    if share_raw.exists():
        return str(share_raw)

    raise FileNotFoundError(
        f"[ADR-005 Fail-Fast] AMCL could not resolve clean map yaml for '{map_name_str}' in assets/maps or package share."
    )

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('race_bringup')
    map_name_str = LaunchConfiguration('map_name').perform(context)
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_str.lower() in ('true', '1')

    map_yaml_path = _resolve_map_yaml_path(map_name_str, pkg_share)
    params_file = LaunchConfiguration('params_file').perform(context)

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml_path},
            {'use_sim_time': use_sim_time}
        ]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    return [map_server_node, amcl_node, lifecycle_manager_node]

def generate_launch_description():
    pkg_share = get_package_share_directory('race_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='ifac_roboracer', description='Map target name'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg_share, 'config', 'amcl.yaml'), description='AMCL params file'),
        OpaqueFunction(function=launch_setup)
    ])
