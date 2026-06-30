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

    # bmi160_config = os.path.join(
    #     get_package_share_directory('bmi160_ros2'),
    #     'config',
    #     'bmi160.yaml'
    # )

    icm20948_config = os.path.join(
        get_package_share_directory('icm20948_ros2'),
        'config',
        'icm20948.yaml'
    )

    odom_to_path_config = os.path.join(
        get_package_share_directory('odom_to_path'),
        'config',
        'odom_to_path.yaml'
    )

    joy_teleop_la = DeclareLaunchArgument(
        'joy_teleop_config',
        default_value=joy_teleop_config,
        description='Descriptions for joy and joy_teleop configs'
    )

    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Descriptions for vesc configs'
    )

    sllidar_s1_la = DeclareLaunchArgument(
        'sllidar_s1_config',
        default_value=sllidar_s1_config,
        description='Descriptions for sllidar-s1 configs'
    )

    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Descriptions for ackermann mux configs'
    )

    # bmi160_la = DeclareLaunchArgument(
    #     'bmi160_config',
    #     default_value=bmi160_config,
    #     description='Description for BMI160 IMU driver configs'
    # )

    icm20948_la = DeclareLaunchArgument(
        'icm20948_config',
        default_value=icm20948_config,
        description='Description for ICM20948 IMU driver configs'
    )

    odom_to_path_la = DeclareLaunchArgument(
        'odom_to_path_config',
        default_value=odom_to_path_config,
        description='Description for odom_to_path configs'
    )

    ld = LaunchDescription([joy_teleop_la, vesc_la, sllidar_s1_la, mux_la, icm20948_la, odom_to_path_la])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_teleop_config')]
    )
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_teleop_config')]
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

    sllidar_ros2_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[LaunchConfiguration('sllidar_s1_config')],
        output='screen'
    )

    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')]
    )

    static_tf_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.065', '0.0', '0.21', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )
    static_tf_imu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_imu',
        arguments=['0.075', '0.0', '0.125', '0.0', '0.0', '0.0', 'base_link', 'imu_link']
    )

    # bmi160_node = Node(
    #     package='bmi160_ros2',
    #     executable='bmi160_node_exe',
    #     name='bmi160_node',
    #     parameters=[LaunchConfiguration('bmi160_config')],
    #     output='screen'
    # )

    icm20948_node = Node(
        package='icm20948_ros2',
        executable='icm20948_node_exe',
        name='icm20948_node',
        parameters=[LaunchConfiguration('icm20948_config')],
        output='screen'
    )

    odom_to_path_node = Node(
        package='odom_to_path',
        executable='odom_to_path_node',
        name='odom_to_path_node',
        parameters=[LaunchConfiguration('odom_to_path_config')],
        output='screen'
    )

    # finalize
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)

    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)

    ld.add_action(throttle_interpolator_node)

    ld.add_action(sllidar_ros2_node)

    ld.add_action(ackermann_mux_node)

    ld.add_action(static_tf_lidar_node)
    ld.add_action(static_tf_imu_node)

    ld.add_action(icm20948_node)

    return ld
