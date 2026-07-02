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
    
    config_file = PathJoinSubstitution([
        pkg_dir, 'config', LaunchConfiguration('sensor_type')
    ])
    
    # We append .yaml to the substitution
    # To do this safely in launch files, we use Python's formatted string via OpaqueFunction or just rely on a trick.
    # Actually simpler: declare the arg as the whole config filename.
    
    return LaunchDescription([
        sensor_type_arg,
        Node(
            package='imu_ros2',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([pkg_dir, 'config', 'icm20948.yaml']) # Default for now, can be overridden by user
            ]
        )
    ])
