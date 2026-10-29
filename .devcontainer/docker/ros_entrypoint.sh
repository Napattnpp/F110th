#!/bin/bash
set -e

# Source ROS 2 (source dustynv install underlay first, then apt underlay)
if [ -f "/opt/ros/$ROS_DISTRO/install/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/install/setup.bash"
fi
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
fi

# Source Micro-ROS (general image only — Jetson image does not install this)
if [ -f "/uros_ws/install/setup.bash" ]; then
    source "/uros_ws/install/setup.bash"
fi

# Source the F110th workspace (colcon install inside out/)
if [ -f "/home/${USER}/F110th/out/install/setup.bash" ]; then
    source "/home/${USER}/F110th/out/install/setup.bash"
fi

exec "$@"