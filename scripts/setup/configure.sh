#!/bin/bash
# ==============================================================================
#  F110th — Platform & Environment Configuration Script
#  Detects hardware platform and resolves transient runtime variables.
# ==============================================================================
set -e

get_realpath() {
    if command -v python3 >/dev/null 2>&1; then
        python3 -c 'import os, sys; print(os.path.realpath(sys.argv[1]))' "$1"
    elif command -v realpath >/dev/null 2>&1; then
        realpath "$1"
    else
        readlink -f "$1" 2>/dev/null || echo "$1"
    fi
}

SCRIPT_DIR=$(dirname "$(get_realpath "$0")")
REPO_DIR=$(get_realpath "$SCRIPT_DIR/../..")
TARGET_ENV="$REPO_DIR/.env"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

info()    { echo -e "${CYAN}[INFO]${NC}  $*"; }
success() { echo -e "${GREEN}[✓]${NC}    $*"; }
warn()    { echo -e "${YELLOW}[!]${NC}    $*"; }
error()   { echo -e "${RED}[✗]${NC}    $*"; exit 1; }

[ -f "$TARGET_ENV" ] || error ".env file missing. Run scripts/setup/bootstrap.sh first."

# Detect Platform
ARCH=$(uname -m)
OS=$(uname -s)
IS_JETSON="no"

if [ -f /etc/nv_tegra_release ]; then
    IS_JETSON="yes"
fi

if [ "$IS_JETSON" = "yes" ]; then
    DOCKER_PROFILE="jetson"
else
    DOCKER_PROFILE="general"
fi

HOST_UID=$(id -u)
HOST_GID=$(id -g)
HOST_USER=$(whoami)

# Export runtime vars for tooling execution
export DOCKER_PROFILE="$DOCKER_PROFILE"
export HOST_UID="$HOST_UID"
export HOST_GID="$HOST_GID"
export USER="$HOST_USER"

# Update/Ensure derived variables in .env without overwriting user choices
set_env_var() {
    local key="$1"
    local val="$2"
    if grep -q "^${key}=" "$TARGET_ENV"; then
        sed -i.bak "s|^${key}=.*|${key}=${val}|" "$TARGET_ENV" && rm -f "$TARGET_ENV.bak"
    else
        echo "${key}=${val}" >> "$TARGET_ENV"
    fi
}

# Ensure mandatory variables exist in .env (handling migration from legacy variables)
if ! grep -q "^RACECAR_ID=" "$TARGET_ENV"; then
    LEGACY_ID=$(grep "^RACECAR_VERSION=" "$TARGET_ENV" 2>/dev/null | cut -d'=' -f2)
    set_env_var "RACECAR_ID" "${LEGACY_ID:-car01}"
fi

# Load current .env values for validation & sync
source "$TARGET_ENV"

# Derive CYCLONEDDS_PROFILE if not explicitly overridden
# Jetson hardware auto-selects loopback profile (cyclonedds_car.xml)
# Workstation / Mac auto-selects dev profile (cyclonedds_dev.xml)
if [ -z "${CYCLONEDDS_PROFILE:-}" ]; then
    if [ "$IS_JETSON" = "yes" ]; then
        DDS_PROFILE="cyclonedds_car.xml"
    else
        DDS_PROFILE="cyclonedds_dev.xml"
    fi
else
    DDS_PROFILE="${CYCLONEDDS_PROFILE}"
fi

DDS_CONFIG_DIR="$REPO_DIR/configs/dds"
case "$DDS_PROFILE" in
    cyclonedds_dev.xml|cyclonedds_car.xml) ;;
    *) error "Invalid CYCLONEDDS_PROFILE '$DDS_PROFILE'. Choose cyclonedds_dev.xml or cyclonedds_car.xml." ;;
esac
DDS_CONFIG="$DDS_CONFIG_DIR/$DDS_PROFILE"
[ -f "$DDS_CONFIG" ] || error "DDS profile not found: $DDS_CONFIG"

# 1. Validate ROS_DOMAIN_ID (must be integer between 0 and 101)
if ! [[ "${ROS_DOMAIN_ID}" =~ ^[0-9]+$ ]] || [ "${ROS_DOMAIN_ID}" -lt 0 ] || [ "${ROS_DOMAIN_ID}" -gt 101 ]; then
    warn "Invalid ROS_DOMAIN_ID '${ROS_DOMAIN_ID}'. Resetting to default 21."
    ROS_DOMAIN_ID=21
    set_env_var "ROS_DOMAIN_ID" "21"
fi

# 2. Synchronize RACECAR_VERSION for ROS 2 launch backward compatibility
RACECAR_ID_VAL="${RACECAR_ID:-car01}"
set_env_var "RACECAR_VERSION" "$RACECAR_ID_VAL"

set_env_var "DOCKER_PROFILE" "$DOCKER_PROFILE"
set_env_var "HOST_UID" "$HOST_UID"
set_env_var "HOST_GID" "$HOST_GID"
set_env_var "CYCLONEDDS_PROFILE" "$DDS_PROFILE"
if [ "$OS" = "Darwin" ]; then
    # On macOS, ROS 2 runs strictly inside Docker. Point to container-safe mount path.
    set_env_var "CYCLONEDDS_URI" "file:///etc/cyclonedds.xml"
    export CYCLONEDDS_URI="file:///etc/cyclonedds.xml"
else
    set_env_var "CYCLONEDDS_URI" "file://$DDS_CONFIG"
    export CYCLONEDDS_URI="file://$DDS_CONFIG"
fi

success "Configuration resolved:"
info "  • Platform:       ${CYAN}$ARCH ($OS)${NC}"
info "  • Jetson Hardware:${CYAN}$IS_JETSON${NC}"
info "  • Docker Profile: ${CYAN}$DOCKER_PROFILE${NC}"
info "  • User UID/GID:   ${CYAN}$HOST_UID:$HOST_GID${NC}"
