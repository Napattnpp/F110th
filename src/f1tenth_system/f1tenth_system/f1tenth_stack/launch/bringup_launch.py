# MIT License

# Copyright (c) 2025 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    joy_teleop_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'joy_teleop.yaml'
    )
    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'vesc.yaml'
    )
    ust10lx_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'ust10lx.yaml'
    )
    sllidar_s1_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'sllidar_s1.yaml'
    )
    mux_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'mux.yaml'
    )
    icm20948_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'icm20948.yaml'
    )
    bmi160_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'bmi160.yaml'
    )

    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Descriptions for joy and joy_teleop configs')
    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Descriptions for vesc configs')
    ust10lx_la = DeclareLaunchArgument(
        'ust10lx_config',
        default_value=ust10lx_config,
        description='Descriptions for urg sensor configs')
    sllidar_s1_la = DeclareLaunchArgument(
        'sllidar_s1_config',
        default_value=sllidar_s1_config,
        description='Descriptions for sllidar-s1 configs')
    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Descriptions for ackermann mux configs')
    icm20948_la = DeclareLaunchArgument(
        'icm20948_config',
        default_value=icm20948_config,
        description='Descriptions for icm20948 configs')
    bmi160_la = DeclareLaunchArgument(
        'bmi160_config',
        default_value=bmi160_config,
        description='Descriptions for bmi160 configs')

    # LiDAR and IMU model selection
    lidar_model_la = DeclareLaunchArgument(
        'lidar_model',
        default_value='sllidar',
        description='LiDAR model to use (sllidar or urg)'
    )
    imu_model_la = DeclareLaunchArgument(
        'imu_model',
        default_value='icm20948',
        description='IMU model to use (icm20948, bmi160, or none)'
    )

    ld = LaunchDescription([
        joy_la, vesc_la, ust10lx_la, sllidar_s1_la, mux_la, icm20948_la, bmi160_la,
        lidar_model_la, imu_model_la
    ])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')]
    )
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')]
    )
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')],
        remappings=[
            ('commands/motor/speed', 'commands/motor/unsmoothed_speed'),
            ('commands/servo/position', 'commands/servo/unsmoothed_position')
        ]
    )
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    throttle_interpolator_node = Node(
        package='f1tenth_stack',
        executable='throttle_interpolator',
        name='throttle_interpolator',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    # Conditional LiDAR nodes
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[LaunchConfiguration('ust10lx')],
        condition=LaunchConfigurationEquals('lidar_model', 'urg')
    )
    sllidar_ros2_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[LaunchConfiguration('sllidar_s1_config')],
        output='screen',
        condition=LaunchConfigurationEquals('lidar_model', 'sllidar')
    )
    # Conditional IMU nodes
    icm20948_node = Node(
        package='icm20948_ros2',
        executable='icm20948_node_exe',
        name='icm20948_node',
        parameters=[LaunchConfiguration('icm20948_config')],
        output='screen',
        condition=LaunchConfigurationEquals('imu_model', 'icm20948')
    )
    bmi160_node = Node(
        package='bmi160_ros2',
        executable='bmi160_node_exe',
        name='bmi160_node',
        parameters=[LaunchConfiguration('bmi160_config')],
        output='screen',
        condition=LaunchConfigurationEquals('imu_model', 'bmi160')
    )
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    )
    static_tf_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.165', '0.0', '0.21', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )
    static_tf_imu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_imu',
        arguments=['0.28', '0.0', '0.1', '0.0', '0.0', '0.0', 'base_link', 'imu_link']
    )

    # finalize
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    ld.add_action(throttle_interpolator_node)

    # Add conditional lidar and imu
    ld.add_action(urg_node)
    ld.add_action(sllidar_ros2_node)
    ld.add_action(icm20948_node)
    ld.add_action(bmi160_node)

    ld.add_action(ackermann_mux_node)
    ld.add_action(static_tf_lidar_node)
    ld.add_action(static_tf_imu_node)

    return ld