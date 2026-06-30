import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


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
    return None


def generate_launch_description():
    pkg_bringup = get_package_share_directory('race_bringup')

    # ── Arguments ────────────────────────────────────────────────────────────
    racecar_version_arg = DeclareLaunchArgument(
        'racecar_version',
        default_value='NUC1',
        description='Racecar version profile (e.g. NUC1, NUC2)'
    )
    frenet_bool_arg = DeclareLaunchArgument(
        'frenet_bool',
        default_value='False',
        description='Whether to republish odometry in Frenet frame'
    )

    # ── Config paths ──────────────────────────────────────────────────────────
    racecar_version = LaunchConfiguration('racecar_version')
    ekf_config = os.path.join(pkg_bringup, 'config', 'ekf.yaml')
    joy_teleop_config = PathJoinSubstitution([
        FindPackageShare('race_bringup'), 'config', racecar_version, 'joy_teleop.yaml'
    ])
    vesc_config = PathJoinSubstitution([
        FindPackageShare('race_bringup'), 'config', racecar_version, 'vesc.yaml'
    ])
    sllidar_s1_config = os.path.join(pkg_bringup, 'config', 'sllidar_s1.yaml')
    mux_config = os.path.join(pkg_bringup, 'config', 'mux.yaml')

    # ── 1. Hardware bringup (VESC, LiDAR, joy, mux) ──────────────────────────
    bringup_source = get_launch_source('f1tenth_stack', 'bringup_launch')
    actions = [
        racecar_version_arg,
        frenet_bool_arg,
    ]

    if bringup_source is not None:
        actions.append(
            IncludeLaunchDescription(
                bringup_source,
                launch_arguments={
                    'joy_teleop_config': joy_teleop_config,
                    'vesc_config': vesc_config,
                    'sllidar_s1_config': sllidar_s1_config,
                    'mux_config': mux_config,
                }.items()
            )
        )

    # ── 2. EKF state estimation only (no SLAM, no localization) ──────────────
    state_estimation_source = get_launch_source('state_estimation', 'state_estimation_launch')
    if state_estimation_source is not None:
        actions.append(
            IncludeLaunchDescription(
                state_estimation_source,
                launch_arguments={
                    'mapping_bool': 'False',
                    'localize_bool': 'False',
                    'frenet_bool': LaunchConfiguration('frenet_bool'),
                    'skip_early_fusion': 'False',   # False = EKF runs
                    'ekf_config': ekf_config,        # use race_bringup ekf.yaml
                }.items()
            )
        )

    return LaunchDescription(actions)
