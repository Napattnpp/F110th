# MIT License

# Copyright (c) 2020 Hongrui Zheng

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
    #> Napattnpp: replace URG lidar to RP lidar
    # sensors_config = os.path.join(
    #     get_package_share_directory('f1tenth_stack'),
    #     'config',
    #     'sensors.yaml'
    # )
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
    #> Codex: Add EKF config
    ekf_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'ekf.yaml'
    )

    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Descriptions for joy and joy_teleop configs')
    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Descriptions for vesc configs')
    #> Napattnpp: replace URG lidar to RP lidar
    # sensors_la = DeclareLaunchArgument(
    #     'sensors_config',
    #     default_value=sensors_config,
    #     description='Descriptions for sensor configs')
    sllidar_s1_la = DeclareLaunchArgument(
        'sllidar_s1_config',
        default_value=sllidar_s1_config,
        description='Descriptions for sllidar-s1 configs')
    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Descriptions for ackermann mux configs')
    #> Codex: Add EKF launch argument
    ekf_la = DeclareLaunchArgument(
        'ekf_config',
        default_value=ekf_config,
        description='Descriptions for robot_localization EKF configs')

    #> Napattnpp: replace URG lidar to RP lidar
    # ld = LaunchDescription([joy_la, vesc_la, sensors_la, mux_la])
    #> Codex: Launch EKF
    ld = LaunchDescription([joy_la, vesc_la, sllidar_s1_la, mux_la, ekf_la])

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
        parameters=[LaunchConfiguration('vesc_config')]
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
    #> Napattnpp: replace URG lidar to RP lidar
    # urg_node = Node(
    #     package='urg_node',
    #     executable='urg_node_driver',
    #     name='urg_node',
    #     parameters=[LaunchConfiguration('sensors_config')]
    # )
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
        #> Napattnpp: No need to remap for this version
        # remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    )
    #> Napattnpp[TF][LIDAR]:
    static_tf_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        # arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser']
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )
    static_tf_imu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_imu',
        #> Codex[TF][IMU]: Identity TF keeps imu_link explicit while aligned with base_link.
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'imu_link']
    )
    #> Codex: Add EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        #> Codex: Fuse wheel odom + micro-ROS IMU and publish odom->base_link TF.
        parameters=[LaunchConfiguration('ekf_config')],
        output='screen'
    )

    # finalize
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    # ld.add_action(throttle_interpolator_node)
    #> Napattnpp: replace URG lidar to RP lidar
    # ld.add_action(urg_node)
    ld.add_action(sllidar_ros2_node)
    ld.add_action(ackermann_mux_node)
    ld.add_action(static_tf_lidar_node)
    #> Codex: Add EKF node
    ld.add_action(static_tf_imu_node)
    #> Codex: Add EKF node
    ld.add_action(ekf_node)

    return ld
