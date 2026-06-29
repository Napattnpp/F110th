#!/usr/bin/env python3
# MIT License
#
# Copyright (c) 2024 Napattnpp
#
# Launch file for the odom_to_path node.
# Loads parameters from odom_to_path.yaml in the package share directory.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odom_to_path_config = os.path.join(
        get_package_share_directory('odom_to_path'),
        'config',
        'odom_to_path.yaml'
    )

    odom_to_path_la = DeclareLaunchArgument(
        'odom_to_path_config',
        default_value=odom_to_path_config,
        description='Path to odom_to_path parameter yaml file'
    )

    odom_to_path_node = Node(
        package='odom_to_path',
        executable='odom_to_path_node',
        name='odom_to_path_node',
        parameters=[LaunchConfiguration('odom_to_path_config')],
        output='screen'
    )

    ld = LaunchDescription([odom_to_path_la])
    ld.add_action(odom_to_path_node)
    return ld
