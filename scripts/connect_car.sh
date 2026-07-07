#!/bin/bash
# ==============================================================================
#  F110th — Connect to Remote Racecar
#  Usage:  source scripts/connect_car.sh [optional_ip_override]
# ==============================================================================

# Find the config file directory relative to this script
SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"
CONFIG_FILE="${SCRIPT_DIR}/../configs/f110th.config"

# Read variables from config file if it exists
if [ -f "$CONFIG_FILE" ]; then
    # Parse lines that look like key=value, ignoring comments
    eval $(grep -v '^#' "$CONFIG_FILE" | grep -v '^$')
fi

# Load variables from .env if present (fallback/override)
ENV_FILE="${SCRIPT_DIR}/../.env"
if [ -f "$ENV_FILE" ]; then
    eval $(grep -v '^#' "$ENV_FILE" | grep -v '^$')
fi

# Apply optional command line override
if [ -n "$1" ]; then
    CAR_IP="$1"
fi

# Default fallbacks
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-21}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export ROS_LOCALHOST_ONLY=0

echo "--------------------------------------------------"
echo " Configuring Remote ROS 2 Connection"
echo "--------------------------------------------------"

if [ "$RMW_IMPLEMENTATION" = "rmw_zenoh_cpp" ]; then
    TARGET_IP=${CAR_IP:-"f1tenth-jetson.local"}
    export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/${TARGET_IP}:7447\"]"
    echo "  Middleware:   Zenoh (rmw_zenoh_cpp)"
    echo "  Target Host:  ${TARGET_IP}"
    echo "  Domain ID:    ${ROS_DOMAIN_ID}"
    echo "--------------------------------------------------"
    echo " Connection active. Launch RViz2/RQT now."
else
    unset ZENOH_CONFIG_OVERRIDE
    echo "  Middleware:   ${RMW_IMPLEMENTATION}"
    echo "  Domain ID:    ${ROS_DOMAIN_ID}"
    echo "--------------------------------------------------"
    echo " Standard DDS active. Ensure both devices share Domain ID ${ROS_DOMAIN_ID}."
fi
