#!/bin/bash
# ==============================================================================
#  F110th — Setup Bootstrap Script
#  Handles system dependency validation, venv setup, and initial .env creation.
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
CONFIG_TEMPLATE="$REPO_DIR/configs/.env.example"
TARGET_ENV="$REPO_DIR/.env"

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

info()    { echo -e "${CYAN}[INFO]${NC}  $*"; }
success() { echo -e "${GREEN}[✓]${NC}    $*"; }
warn()    { echo -e "${YELLOW}[!]${NC}    $*"; }
error()   { echo -e "${RED}[✗]${NC}    $*"; exit 1; }

info "Checking system dependencies..."
command -v git >/dev/null 2>&1 || error "Git is not installed."
command -v python3 >/dev/null 2>&1 || error "Python 3 is not installed."
command -v docker >/dev/null 2>&1 || error "Docker is not installed."
docker compose version >/dev/null 2>&1 || error "'docker compose' plugin is not installed."
docker info >/dev/null 2>&1 || error "Docker daemon is not running. Please start Docker / Docker Desktop."

success "System dependencies and Docker daemon verified."

# 1. Initialize .env with Interactive Wizard if missing or unconfigured
if [ ! -f "$TARGET_ENV" ]; then
    info "Initializing .env configuration wizard..."
    [ -f "$CONFIG_TEMPLATE" ] || error ".env.example not found in configs/"
    cp "$CONFIG_TEMPLATE" "$TARGET_ENV"

    # Interactive wizard (if running in an interactive terminal tty)
    if [ -t 0 ]; then
        echo ""
        echo -e "${CYAN}──────── Workspace Configuration Wizard ────────${NC}"
        echo -e "${YELLOW}Configure your local team settings (press Enter for defaults):${NC}"
        echo ""

        # 1. Operational Role
        echo "Select Operational Mode (F110_ROLE):"
        echo "  1) simulation — Local Mac/Laptop Simulator & Development (Default)"
        echo "  2) vehicle    — Onboard Compute on Physical Car (Jetson / NUC)"
        read -p "Select choice [1-2] (default: 1): " ROLE_CHOICE
        case "$ROLE_CHOICE" in
            2) ROLE="vehicle" ;;
            *) ROLE="simulation" ;;
        esac


        # 2. ROS Domain ID
        read -p "Enter ROS Domain ID [0-101] (default: 21): " DOMAIN_INPUT
        DOMAIN_ID="${DOMAIN_INPUT:-21}"

        # 3. Racecar ID
        read -p "Enter Target Racecar ID (default: car01): " CAR_ID_INPUT
        RACECAR_ID_VAL="${CAR_ID_INPUT:-car01}"

        # 4. Car Hostname
        read -p "Enter Racecar Hostname/IP (default: car01.local): " CAR_HOST_INPUT
        CAR_HOST_VAL="${CAR_HOST_INPUT:-car01.local}"

        # Write choices into .env
        sed -i.bak "s|^F110_ROLE=.*|F110_ROLE=${ROLE}|" "$TARGET_ENV"
        sed -i.bak "s|^ROS_DOMAIN_ID=.*|ROS_DOMAIN_ID=${DOMAIN_ID}|" "$TARGET_ENV"
        sed -i.bak "s|^RACECAR_ID=.*|RACECAR_ID=${RACECAR_ID_VAL}|" "$TARGET_ENV"
        sed -i.bak "s|^CAR_HOSTNAME=.*|CAR_HOSTNAME=${CAR_HOST_VAL}|" "$TARGET_ENV"
        rm -f "$TARGET_ENV.bak"
    fi

    success ".env initialized successfully."
else
    info ".env already exists. Preserving local configuration."
fi

# Display Configuration Summary
source "$TARGET_ENV"
echo ""
echo -e "${CYAN}╔═══════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║ Active Configuration (.env):                              ║${NC}"
echo -e "${CYAN}║ • F110_ROLE:      ${GREEN}${F110_ROLE:-simulation}${NC}"
echo -e "${CYAN}║ • ROS_DOMAIN_ID:  ${GREEN}${ROS_DOMAIN_ID:-21}${NC}"
echo -e "${CYAN}║ • RACECAR_ID:     ${GREEN}${RACECAR_ID:-car01}${NC}"
echo -e "${CYAN}║ • CAR_HOSTNAME:   ${GREEN}${CAR_HOSTNAME:-car01.local}${NC}"
echo -e "${CYAN}╚═══════════════════════════════════════════════════════════╝${NC}"
echo ""

# 2. Virtual Environment Setup
VENV_DIR="$REPO_DIR/.venv"
if [ ! -d "$VENV_DIR" ]; then
    info "Creating Python virtual environment at .venv..."
    python3 -m venv "$VENV_DIR"
fi

if [ -f "$REPO_DIR/configs/requirements.txt" ]; then
    info "Installing host developer requirements..."
    "$VENV_DIR/bin/pip" install --upgrade pip >/dev/null 2>&1
    "$VENV_DIR/bin/pip" install -r "$REPO_DIR/configs/requirements.txt" >/dev/null 2>&1
    success "Host Python dependencies installed."
fi
