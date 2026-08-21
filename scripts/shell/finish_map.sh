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
# Try to find the workspace root (where 'src' or 'assets' lives)
PROJECT_ROOT="${SCRIPT_DIR}"
while [ ! -d "${PROJECT_ROOT}/src" ] && [ "${PROJECT_ROOT}" != "/" ]; do
    PROJECT_ROOT="$(dirname "${PROJECT_ROOT}")"
done
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

# 4. Initializing map file sets: base (<map_name>), raw (<map_name>_raw), and waypoint generator (<map_name>_waypoint_gen)
if [ -f "${TARGET_DIR}/${MAP_NAME}.yaml" ] && [ -f "${TARGET_DIR}/${MAP_NAME}.pgm" ]; then
    echo "4. Creating _raw and _waypoint_gen map file sets..."
    
    # Fix base YAML image path to be simple relative filename
    sed -i "s|image: .*|image: ${MAP_NAME}.pgm|" "${TARGET_DIR}/${MAP_NAME}.yaml" 2>/dev/null || true
    
    # Generate _raw map files
    cp -f "${TARGET_DIR}/${MAP_NAME}.pgm" "${TARGET_DIR}/${MAP_NAME}_raw.pgm"
    cp -f "${TARGET_DIR}/${MAP_NAME}.yaml" "${TARGET_DIR}/${MAP_NAME}_raw.yaml"
    sed -i "s|image: .*|image: ${MAP_NAME}_raw.pgm|" "${TARGET_DIR}/${MAP_NAME}_raw.yaml" 2>/dev/null || true

    # Generate _waypoint_gen map files
    cp -f "${TARGET_DIR}/${MAP_NAME}.pgm" "${TARGET_DIR}/${MAP_NAME}_waypoint_gen.pgm"
    cp -f "${TARGET_DIR}/${MAP_NAME}.yaml" "${TARGET_DIR}/${MAP_NAME}_waypoint_gen.yaml"
    sed -i "s|image: .*|image: ${MAP_NAME}_waypoint_gen.pgm|" "${TARGET_DIR}/${MAP_NAME}_waypoint_gen.yaml" 2>/dev/null || true
fi

# 5. Initializing per-track global_planner_params.yaml template
CENTRAL_PARAMS="${PROJECT_ROOT}/src/race_bringup/config/global_planner/global_planner_params.yaml"
TRACK_PARAMS="${TARGET_DIR}/global_planner_params.yaml"
if [ -f "${CENTRAL_PARAMS}" ] && [ ! -f "${TRACK_PARAMS}" ]; then
    echo "5. Initializing per-track global_planner_params.yaml in ${TARGET_DIR}..."
    cp -f "${CENTRAL_PARAMS}" "${TRACK_PARAMS}"
fi

echo ""
echo "=================================================="
echo " Map files successfully generated in: ${TARGET_DIR}"
ls -l "${TARGET_DIR}"
echo "=================================================="
echo "To sync these maps to your ROS workspace build, run:"
echo "  make colcon_build"
echo "=================================================="
