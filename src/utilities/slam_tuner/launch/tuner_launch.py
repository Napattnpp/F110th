import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
    pkg_slam_tuner = get_package_share_directory('slam_tuner')

    # Arguments
    bag_arg = DeclareLaunchArgument(
        'bag',
        default_value='default_bag_folder',
        description='Name of the rosbag folder within slam_tuner/data'
    )
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='default_map_name',
        description='Name of the map'
    )
    racecar_version_arg = DeclareLaunchArgument(
        'racecar_version',
        default_value='NUC2',
        description='NUC version for parameters'
    )
    pub_tf_arg = DeclareLaunchArgument(
        'pub_tf',
        default_value='True'
    )
    bag_ekf_arg = DeclareLaunchArgument(
        'bag_ekf',
        default_value='False',
        description='Use EKF from the bag (True) or run EKF live (False)'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='True'
    )
    frenet_arg = DeclareLaunchArgument(
        'frenet',
        default_value='False'
    )
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Rate of rosbag playback'
    )
    mapping_bool_arg = DeclareLaunchArgument(
        'mapping_bool',
        default_value='False'
    )

    # Dynamic bag path substitution
    bag_path = [pkg_slam_tuner, '/data/', LaunchConfiguration('bag'), '.db3']

    # Static transforms
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    static_baselink_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.27', '0', '0.11', '0', '0', '0', 'base_link', 'laser']
    )
    static_baselink_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_imu',
        arguments=['0', '.07', '0', '0.05', '0', '0', '1.5707963', 'base_link', 'imu']
    )

    # Frenet odom republisher
    frenet_odom_repub = GroupAction(
        condition=IfCondition(LaunchConfiguration('frenet')),
        actions=[
            IncludeLaunchDescription(
                get_launch_source('frenet_odom_republisher', 'frenet_odom_republisher_launch')
            )
        ]
    )

    # State Estimation Launch
    state_estimation_launch = IncludeLaunchDescription(
        get_launch_source('state_estimation', 'state_estimation_launch'),
        launch_arguments={
            'mapping_bool': LaunchConfiguration('mapping_bool'),
            'racecar_version': LaunchConfiguration('racecar_version'),
            'map_name': LaunchConfiguration('map_name'),
            'frenet_bool': LaunchConfiguration('frenet'),
            'slam_config_path': os.path.join(pkg_slam_tuner, 'config'),
            'use_sim_time': 'True',
            'skip_early_fusion': 'True'
        }.items()
    )

    # EKF Sim (if not using EKF from bag)
    ekf_sim = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('bag_ekf')),
        actions=[
            IncludeLaunchDescription(
                get_launch_source('slam_tuner', 'ekf_sim_launch')
            )
        ]
    )

    # Play Rosbag (Without bag EKF)
    play_bag_no_ekf = ExecuteProcess(
        condition=UnlessCondition(LaunchConfiguration('bag_ekf')),
        cmd=[
            'ros2', 'bag', 'play', bag_path,
            '--clock', '--disable-keyboard-controls',
            '--topics', '/scan', '/odom', '/sensors/imu/raw',
            '-r', LaunchConfiguration('rate')
        ],
        output='screen'
    )

    # Play Rosbag (With bag EKF)
    play_bag_ekf = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('bag_ekf')),
        cmd=[
            'ros2', 'bag', 'play', bag_path,
            '--clock', '--disable-keyboard-controls',
            '--topics', '/scan', '/odom', '/sensors/imu/raw', '/early_fusion/odom',
            '-r', LaunchConfiguration('rate')
        ],
        output='screen'
    )

    # RViz node
    rviz_node = GroupAction(
        condition=IfCondition(LaunchConfiguration('rviz')),
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', os.path.join(pkg_slam_tuner, 'rviz', 'tuner_rviz.rviz')],
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        bag_arg,
        map_name_arg,
        racecar_version_arg,
        pub_tf_arg,
        bag_ekf_arg,
        rviz_arg,
        frenet_arg,
        rate_arg,
        mapping_bool_arg,
        static_map_to_odom,
        static_baselink_to_laser,
        static_baselink_to_imu,
        frenet_odom_repub,
        state_estimation_launch,
        ekf_sim,
        play_bag_no_ekf,
        play_bag_ekf,
        rviz_node
    ])
