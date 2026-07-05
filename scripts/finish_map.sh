#!/bin/bash
# ==============================================================================
#  F110th — finish_map.sh
#  Automatically detects active SLAM node (Cartographer vs SLAM Toolbox)
#  and saves the maps directly into assets/maps/<map_name>/
# ==============================================================================

if [ -z "$1" ]; then
    echo "Usage: ./finish_map.sh <map_name>"
    exit 1
fi

MAP_NAME=$1
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TARGET_DIR="${PROJECT_ROOT}/assets/maps/${MAP_NAME}"

echo "Creating map directory in assets: ${TARGET_DIR}"
mkdir -p "${TARGET_DIR}"

# Check active ROS 2 services to detect the SLAM system
SERVICES=$(ros2 service list 2>/dev/null)

if echo "$SERVICES" | grep -q "/write_state"; then
    echo "=================================================="
    echo " Detected Google Cartographer SLAM"
    echo "=================================================="
    PBSTREAM_PATH="${TARGET_DIR}/${MAP_NAME}.pbstream"
    MAP_FILESTEM="${TARGET_DIR}/${MAP_NAME}"

    echo "1. Finishing Cartographer trajectory..."
    ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

    echo "2. Saving pbstream map to: ${PBSTREAM_PATH}..."
    ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '${PBSTREAM_PATH}', include_unfinished_submaps: 'true'}"

    echo "3. Converting pbstream to 2D occupancy grid..."
    ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
      -pbstream_filename "${PBSTREAM_PATH}" \
      -map_filestem "${MAP_FILESTEM}"

elif echo "$SERVICES" | grep -q "/slam_toolbox/save_map"; then
    echo "=================================================="
    echo " Detected SLAM Toolbox"
    echo "=================================================="
    
    echo "1. Saving SLAM Toolbox posegraph/map serialized data..."
    ros2 service call /slam_toolbox/save_map slam_toolbox_msgs/srv/SaveMap "{name: {data: '${TARGET_DIR}/${MAP_NAME}'}}"

    echo "2. Generating 2D occupancy grid map files..."
    ros2 run nav2_map_server map_saver_cli -f "${TARGET_DIR}/${MAP_NAME}"

else
    echo "=================================================="
    echo " No SLAM-specific service found. Falling back to generic map saver..."
    echo "=================================================="
    ros2 run nav2_map_server map_saver_cli -f "${TARGET_DIR}/${MAP_NAME}"
fi

echo ""
echo "=================================================="
echo " Map files successfully generated in: ${TARGET_DIR}"
ls -l "${TARGET_DIR}"
echo "=================================================="
echo "To sync these maps to your ROS workspace build, run:"
echo "  make colcon_build"
echo "=================================================="
