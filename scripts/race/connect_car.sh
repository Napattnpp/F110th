#!/bin/bash
# ==============================================================================
#  F110th — Connect to Remote Racecar
#  Usage:  source scripts/race/connect_car.sh [optional_ip_override]
# ==============================================================================

get_realpath() {
    if command -v python3 >/dev/null 2>&1; then
        python3 -c 'import os, sys; print(os.path.realpath(sys.argv[1]))' "$1"
    elif command -v realpath >/dev/null 2>&1; then
        realpath "$1"
    else
        readlink -f "$1" 2>/dev/null || echo "$1"
    fi
}

SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"
REPO_DIR="$(get_realpath "${SCRIPT_DIR}/../..")"
ENV_FILE="${REPO_DIR}/.env"

if [ ! -f "$ENV_FILE" ]; then
    echo "[!] .env file not found at $REPO_DIR/.env. Please run 'make setup' (or ./setup.sh) first."
    return 1 2>/dev/null || exit 1
fi

set -a
source "$ENV_FILE"
set +a

# Resolve target racecar IP / Hostname (Command line override -> CAR_HOSTNAME -> CAR_IP -> car01.local)
TARGET_HOST="${1:-${CAR_HOSTNAME:-${CAR_IP:-car01.local}}}"

if [ "$RMW_IMPLEMENTATION" = "rmw_cyclonedds_cpp" ]; then
    DDS_PROFILE="${CYCLONEDDS_PROFILE:-cyclonedds_dev.xml}"
    case "$DDS_PROFILE" in
        cyclonedds_dev.xml|cyclonedds_car.xml|cyclonedds_pit.xml) ;;
        *) echo "[!] Invalid CYCLONEDDS_PROFILE: $DDS_PROFILE"; return 1 2>/dev/null || exit 1 ;;
    esac
    DDS_CONFIG="${REPO_DIR}/configs/dds/${DDS_PROFILE}"
    if [ ! -f "$DDS_CONFIG" ]; then
        echo "[!] CycloneDDS profile not found: $DDS_CONFIG"
        return 1 2>/dev/null || exit 1
    fi
    export CYCLONEDDS_URI="file://${DDS_CONFIG}"
fi

echo "--------------------------------------------------"
echo " Configuring Remote ROS 2 Connection"
echo "--------------------------------------------------"

if [ "$RMW_IMPLEMENTATION" = "rmw_zenoh_cpp" ]; then
    export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/${TARGET_HOST}:7447\"]"
    echo "  Middleware:      Zenoh (rmw_zenoh_cpp)"
    echo "  Target Host:     ${TARGET_HOST}"
    echo "  Domain ID:       ${ROS_DOMAIN_ID}"
    echo "--------------------------------------------------"
    echo " Connection active. Launch RViz2/RQT now."
else
    unset ZENOH_CONFIG_OVERRIDE
    echo "  Middleware:      ${RMW_IMPLEMENTATION}"
    echo "  Discovery Range: ${ROS_AUTOMATIC_DISCOVERY_RANGE}"
    echo "  Domain ID:       ${ROS_DOMAIN_ID}"
    echo "  CycloneDDS URI:  ${CYCLONEDDS_URI}"
    echo "--------------------------------------------------"
    echo " Standard DDS active. Ensure both devices share Domain ID ${ROS_DOMAIN_ID}."
fi
