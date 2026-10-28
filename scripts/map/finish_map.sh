#!/bin/bash
# ==============================================================================
#  F110th — finish_map.sh
#  Automatically detects active SLAM node (Cartographer vs SLAM Toolbox)
#  and saves map assets directly into structured subfolders:
#    assets/maps/<map_name>/
#      ├── raw/
#      ├── clean/
#      ├── waypoint_gen/
#      └── waypoints/
# ==============================================================================

if [ -z "$1" ]; then
    echo "Usage: ./finish_map.sh <map_name>"
    exit 1
fi

MAP_NAME=$1

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Find workspace root
PROJECT_ROOT="${SCRIPT_DIR}"
while [ ! -d "${PROJECT_ROOT}/src" ] && [ "${PROJECT_ROOT}" != "/" ]; do
    PROJECT_ROOT="$(dirname "${PROJECT_ROOT}")"
done

TARGET_DIR="${PROJECT_ROOT}/assets/maps/${MAP_NAME}"
REL_PATH="assets/maps/${MAP_NAME}"

RAW_DIR="${TARGET_DIR}/raw"
CLEAN_DIR="${TARGET_DIR}/clean"
WP_GEN_DIR="${TARGET_DIR}/waypoint_gen"
WAYPOINTS_DIR="${TARGET_DIR}/waypoints"

echo "Creating map directory structure in ${REL_PATH}..."
mkdir -p "${RAW_DIR}" "${CLEAN_DIR}" "${WP_GEN_DIR}" "${WAYPOINTS_DIR}"

# Check active ROS 2 services to detect the SLAM system
SERVICES=$(ros2 service list 2>/dev/null)

TEMP_FILESTEM="${TARGET_DIR}/${MAP_NAME}"

if echo "$SERVICES" | grep -q "/write_state"; then
    echo "=================================================="
    echo " Detected Google Cartographer SLAM"
    echo "=================================================="
    PBSTREAM_PATH="${RAW_DIR}/${MAP_NAME}.pbstream"

    echo "1. Finishing Cartographer trajectory..."
    ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

    echo "2. Saving pbstream map to: ${PBSTREAM_PATH}..."
    ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '${PBSTREAM_PATH}', include_unfinished_submaps: 'true'}"

    echo "3. Converting pbstream to 2D occupancy grid..."
    ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
      -pbstream_filename "${PBSTREAM_PATH}" \
      -map_filestem "${TEMP_FILESTEM}"

elif echo "$SERVICES" | grep -q "/slam_toolbox/save_map"; then
    echo "=================================================="
    echo " Detected SLAM Toolbox"
    echo "=================================================="
    
    echo "1. Saving SLAM Toolbox posegraph/map serialized data..."
    ros2 service call /slam_toolbox/save_map slam_toolbox_msgs/srv/SaveMap "{name: {data: '${RAW_DIR}/${MAP_NAME}'}}"

    echo "2. Generating 2D occupancy grid map files..."
    ros2 run nav2_map_server map_saver_cli -f "${TEMP_FILESTEM}"

else
    echo "=================================================="
    echo " No SLAM-specific service found. Falling back to generic map saver..."
    echo "=================================================="
    ros2 run nav2_map_server map_saver_cli -f "${TEMP_FILESTEM}"
fi

# Move generated PGM and YAML to raw/, clean/, and waypoint_gen/
if [ -f "${TEMP_FILESTEM}.yaml" ] && [ -f "${TEMP_FILESTEM}.pgm" ]; then
    echo "4. Organizing map assets into raw/, clean/, and waypoint_gen/..."

    # 1. Store in raw/
    cp -f "${TEMP_FILESTEM}.pgm" "${RAW_DIR}/${MAP_NAME}_raw.pgm"
    cp -f "${TEMP_FILESTEM}.yaml" "${RAW_DIR}/${MAP_NAME}_raw.yaml"
    sed -i "s|image: .*|image: ${MAP_NAME}_raw.pgm|" "${RAW_DIR}/${MAP_NAME}_raw.yaml" 2>/dev/null || true

    # 2. Store baseline in clean/
    mv -f "${TEMP_FILESTEM}.pgm" "${CLEAN_DIR}/${MAP_NAME}.pgm"
    mv -f "${TEMP_FILESTEM}.yaml" "${CLEAN_DIR}/${MAP_NAME}.yaml"
    sed -i "s|image: .*|image: ${MAP_NAME}.pgm|" "${CLEAN_DIR}/${MAP_NAME}.yaml" 2>/dev/null || true

    # 3. Store baseline in waypoint_gen/
    cp -f "${CLEAN_DIR}/${MAP_NAME}.pgm" "${WP_GEN_DIR}/${MAP_NAME}_waypoint_gen.pgm"
    cp -f "${CLEAN_DIR}/${MAP_NAME}.yaml" "${WP_GEN_DIR}/${MAP_NAME}_waypoint_gen.yaml"
    sed -i "s|image: .*|image: ${MAP_NAME}_waypoint_gen.pgm|" "${WP_GEN_DIR}/${MAP_NAME}_waypoint_gen.yaml" 2>/dev/null || true
fi

# Initializing per-track global_planner_params.yaml template in waypoints/
CENTRAL_PARAMS="${PROJECT_ROOT}/src/race_bringup/config/global_planner/global_planner_params.yaml"
TRACK_PARAMS="${WAYPOINTS_DIR}/global_planner_params.yaml"
if [ -f "${CENTRAL_PARAMS}" ] && [ ! -f "${TRACK_PARAMS}" ]; then
    echo "5. Initializing per-track global_planner_params.yaml in ${WAYPOINTS_DIR}..."
    cp -f "${CENTRAL_PARAMS}" "${TRACK_PARAMS}"
fi

echo ""
echo "=================================================="
echo " Map files successfully generated in: ${TARGET_DIR}"
echo "   - raw/          : Original SLAM outputs"
echo "   - clean/        : Localization map"
echo "   - waypoint_gen/ : Optimization map"
echo "To manually clean/refine thresholding, run:"
echo "  python3 src/utilities/map_utils/map_cleaner_matplotlib.py ${REL_PATH}/raw/${MAP_NAME}_raw.yaml"
echo "=================================================="
