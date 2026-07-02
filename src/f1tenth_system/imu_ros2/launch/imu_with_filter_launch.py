import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('imu_ros2')
    
    sensor_type_arg = DeclareLaunchArgument(
        'sensor_type',
        default_value='icm20948',
        description='Sensor type (icm20948 or bmi160)'
    )
    
    # 1. IMU Driver Node
    imu_node = Node(
        package='imu_ros2',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_dir, 'config', LaunchConfiguration('sensor_type')]),
            # Ensure it outputs raw data to the standard topic
            {'use_sim_time': False}
        ]
    )

    # 2. Madgwick Filter Node
    # This subscribes to /imu/data_raw (and /imu/mag if available) 
    # and publishes to /imu/data which robot_localization uses.
    madgwick_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_mag': True,           # Set to false if using BMI160 or if mag environment is noisy
            'publish_tf': False,       # Let robot_localization handle TF
            'world_frame': 'enu',
            'fixed_frame': 'odom',
            # You may need to tune the gain parameter!
            'gain': 0.1
        }],
        remappings=[
            ('/imu/data_raw', '/imu/data_raw'),
            ('/imu/mag', '/imu/mag'),
            ('/imu/data', '/imu/data')
        ]
    )

    return LaunchDescription([
        sensor_type_arg,
        imu_node,
        madgwick_node
    ])
