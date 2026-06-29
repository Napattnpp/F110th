from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('icm20948_ros2'),
        'config'
    )
    
    icm20948_config = os.path.join(config_dir, 'icm20948.yaml')
    
    icm20948_la = DeclareLaunchArgument(
        'icm20948_config',
        default_value=icm20948_config,
        description='Path to ICM20948 config file'
    )
    
    icm20948_node = Node(
        package='icm20948_ros2',
        executable='icm20948_node_exe',
        name='icm20948_node',
        output='screen',
        parameters=[LaunchConfiguration('icm20948_config')]
    )
    
    return LaunchDescription([
        icm20948_la,
        icm20948_node
    ])
