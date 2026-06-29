from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
    trajectory_arg = DeclareLaunchArgument(
        'trajectory',
        default_value='min_curv',
        description='centerline/min_curv/shortest_path/min_time'
    )
    start_s_arg = DeclareLaunchArgument(
        'start_s',
        default_value='0.0',
        description='Initial starting Frenet coordinate'
    )
    speed_scaler_arg = DeclareLaunchArgument(
        'speed_scaler',
        default_value='0.5',
        description='Speed scaling factor or constant speed value'
    )
    constant_speed_arg = DeclareLaunchArgument(
        'constant_speed',
        default_value='false',
        description='Whether to run at constant speed'
    )
    type_arg = DeclareLaunchArgument(
        'type',
        default_value='lidar',
        description='virtual or lidar'
    )

    # Collision Detector Node
    collision_detector_node = Node(
        package='opponent_publisher',
        executable='collision_detector',
        name='collision_detector',
        output='screen'
    )

    # Obstacle Publisher Node
    obstacle_publisher_node = Node(
        package='opponent_publisher',
        executable='obstacle_publisher',
        name='obstacle_publisher',
        output='screen',
        parameters=[{
            'start_s': LaunchConfiguration('start_s'),
            'trajectory': LaunchConfiguration('trajectory'),
            'speed_scaler': LaunchConfiguration('speed_scaler'),
            'constant_speed': LaunchConfiguration('constant_speed'),
            'type': LaunchConfiguration('type')
        }]
    )

    return LaunchDescription([
        trajectory_arg,
        start_s_arg,
        speed_scaler_arg,
        constant_speed_arg,
        type_arg,
        collision_detector_node,
        obstacle_publisher_node
    ])
