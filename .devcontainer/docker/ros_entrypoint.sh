#!/bin/bash
set -e

# Source ROS 2
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Source Micro-ROS (general image only — Jetson image does not install this)
if [ -f "/uros_ws/install/setup.bash" ]; then
    source "/uros_ws/install/setup.bash"
fi

# Source the F110th workspace (colcon install inside out/)
if [ -f "/home/${USER}/F110th/out/install/setup.bash" ]; then
    source "/home/${USER}/F110th/out/install/setup.bash"
fi

exec "$@"