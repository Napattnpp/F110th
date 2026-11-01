#!/bin/bash
# ==============================================================================
#  F110th — finish_map.sh
#  Automatically detects active SLAM node (Cartographer vs SLAM Toolbox)
#  and saves map assets directly into ADR-004 structured subfolders:
#    assets/maps/<map_name>/
#      ├── raw/
#      ├── clean/
#      ├── waypoint_gen/
#      └── waypoints/
# ==============================================================================

set -e

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

# Ensure ROS 2 underlay and workspace overlay are sourced
if [ -z "${ROS_DISTRO}" ]; then
    if [ -f "/opt/ros/humble/install/setup.bash" ]; then
        source /opt/ros/humble/install/setup.bash
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
fi

if [ -f "${PROJECT_ROOT}/out/install/setup.bash" ]; then
    source "${PROJECT_ROOT}/out/install/setup.bash"
elif [ -f "${PROJECT_ROOT}/install/setup.bash" ]; then
    source "${PROJECT_ROOT}/install/setup.bash"
fi

TARGET_DIR="${PROJECT_ROOT}/assets/maps/${MAP_NAME}"
REL_PATH="assets/maps/${MAP_NAME}"

RAW_DIR="${TARGET_DIR}/raw"
CLEAN_DIR="${TARGET_DIR}/clean"
WP_GEN_DIR="${TARGET_DIR}/waypoint_gen"
WAYPOINTS_DIR="${TARGET_DIR}/waypoints"

echo "Creating map directory structure in ${REL_PATH}..."
mkdir -p "${RAW_DIR}" "${CLEAN_DIR}" "${WP_GEN_DIR}" "${WAYPOINTS_DIR}"

# Check active ROS 2 services to detect the SLAM system
SERVICES=$(ros2 service list 2>/dev/null || true)

TEMP_FILESTEM="${TARGET_DIR}/${MAP_NAME}"

if echo "$SERVICES" | grep -q "/write_state"; then
    echo "=================================================="
    echo " Detected Google Cartographer SLAM"
    echo "=================================================="
    PBSTREAM_PATH="${RAW_DIR}/${MAP_NAME}.pbstream"

    echo "1. Finishing Cartographer trajectory..."
    ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}" || true

    echo "2. Saving pbstream map to: ${PBSTREAM_PATH}..."
    ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '${PBSTREAM_PATH}', include_unfinished_submaps: true}"

    echo "3. Converting pbstream to 2D occupancy grid..."
    ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
      -pbstream_filename "${PBSTREAM_PATH}" \
      -map_filestem "${TEMP_FILESTEM}" \
      -resolution 0.05

elif echo "$SERVICES" | grep -q "/slam_toolbox/save_map"; then
    echo "=================================================="
    echo " Detected SLAM Toolbox"
    echo "=================================================="
    
    echo "1. Saving SLAM Toolbox map serialized data..."
    ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '${RAW_DIR}/${MAP_NAME}'}}"
    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: {data: '${RAW_DIR}/${MAP_NAME}'}}" 2>/dev/null || true

    echo "2. Generating 2D occupancy grid map files..."
    ros2 run nav2_map_server map_saver_cli -f "${TEMP_FILESTEM}"

else
    echo "=================================================="
    echo " No SLAM-specific service found. Falling back to generic map saver..."
    echo "=================================================="
    ros2 run nav2_map_server map_saver_cli -f "${TEMP_FILESTEM}"
fi

# Helper function to update image field in YAML file portably
update_yaml_image() {
    local yaml_path="$1"
    local pgm_name="$2"
    if [ -f "${yaml_path}" ]; then
        python3 -c "
import sys
ypath, img = sys.argv[1], sys.argv[2]
with open(ypath, 'r') as f:
    lines = f.readlines()
with open(ypath, 'w') as f:
    for line in lines:
        if line.strip().startswith('image:'):
            f.write(f'image: {img}\n')
        else:
            f.write(line)
" "${yaml_path}" "${pgm_name}" 2>/dev/null || true
    fi
}

# Move generated PGM and YAML to raw/, clean/, and waypoint_gen/
if [ -f "${TEMP_FILESTEM}.yaml" ] && [ -f "${TEMP_FILESTEM}.pgm" ]; then
    echo "4. Organizing map assets into raw/, clean/, and waypoint_gen/..."

    # 1. Store in raw/
    cp -f "${TEMP_FILESTEM}.pgm" "${RAW_DIR}/${MAP_NAME}_raw.pgm"
    cp -f "${TEMP_FILESTEM}.yaml" "${RAW_DIR}/${MAP_NAME}_raw.yaml"
    update_yaml_image "${RAW_DIR}/${MAP_NAME}_raw.yaml" "${MAP_NAME}_raw.pgm"

    # 2. Store baseline in clean/
    mv -f "${TEMP_FILESTEM}.pgm" "${CLEAN_DIR}/${MAP_NAME}.pgm"
    mv -f "${TEMP_FILESTEM}.yaml" "${CLEAN_DIR}/${MAP_NAME}.yaml"
    update_yaml_image "${CLEAN_DIR}/${MAP_NAME}.yaml" "${MAP_NAME}.pgm"

    # 3. Store baseline in waypoint_gen/
    cp -f "${CLEAN_DIR}/${MAP_NAME}.pgm" "${WP_GEN_DIR}/${MAP_NAME}_waypoint_gen.pgm"
    cp -f "${CLEAN_DIR}/${MAP_NAME}.yaml" "${WP_GEN_DIR}/${MAP_NAME}_waypoint_gen.yaml"
    update_yaml_image "${WP_GEN_DIR}/${MAP_NAME}_waypoint_gen.yaml" "${MAP_NAME}_waypoint_gen.pgm"
fi

# Initializing per-track global_planner_params.yaml template in waypoint_gen/ (ADR-004)
TEMPLATE_PARAMS="${PROJECT_ROOT}/src/race_bringup/config/global_planner/global_planner_params.template.yaml"
CENTRAL_PARAMS="${PROJECT_ROOT}/src/race_bringup/config/global_planner/global_planner_params.yaml"
TRACK_PARAMS="${WP_GEN_DIR}/global_planner_params.yaml"

SRC_PARAMS="${CENTRAL_PARAMS}"
if [ -f "${TEMPLATE_PARAMS}" ]; then
    SRC_PARAMS="${TEMPLATE_PARAMS}"
fi

if [ -f "${SRC_PARAMS}" ] && [ ! -f "${TRACK_PARAMS}" ]; then
    echo "5. Initializing per-track global_planner_params.yaml in ${WP_GEN_DIR}..."
    cp -f "${SRC_PARAMS}" "${TRACK_PARAMS}"
fi

echo ""
echo "=================================================="
echo " Map files successfully generated in: ${TARGET_DIR}"
echo "   - raw/          : Original SLAM outputs (.pbstream, raw pgm/yaml)"
echo "   - clean/        : Localization map"
echo "   - waypoint_gen/ : Optimization map"
echo "   - waypoints/    : Raceline & sector tuning directory"
echo "--------------------------------------------------"
echo " Next Steps to Race on '${MAP_NAME}':"
echo "   1. Clean Map & Generate Binary Mask (ADR-004):"
echo "      make clean_map MAP=${MAP_NAME}      # Headless auto-clean"
echo "      make clean_web MAP=${MAP_NAME}      # Interactive browser studio (port 8051)"
echo "   2. Set Start/Finish Line & AMCL Pose:"
echo "      make start_line MAP=${MAP_NAME}"
echo "   3. Optimize Raceline (TUM IQP):"
echo "      make optimize MAP=${MAP_NAME}"
echo "   -- OR run the entire pipeline in one command:"
echo "      make map_pipeline MAP=${MAP_NAME}"
echo "=================================================="
