#!/bin/bash
# ==============================================================================
#  F110th — Master Setup Script
#  Usage: ./scripts/setup/setup.sh (or `make setup` / `./setup.sh`)
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

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BLUE='\033[0;34m'
NC='\033[0m'

info()    { echo -e "${CYAN}[INFO]${NC}  $*"; }
success() { echo -e "${GREEN}[✓]${NC}    $*"; }
warn()    { echo -e "${YELLOW}[!]${NC}    $*"; }
error()   { echo -e "${RED}[✗]${NC}    $*"; exit 1; }
section() {
    echo ""
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}  $*${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
}

echo ""
echo -e "${CYAN}╔═══════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║  F110th Setup                                             ║${NC}"
echo -e "${CYAN}║  Auto-detects platform • Builds Docker • Launches stack   ║${NC}"
echo -e "${CYAN}╚═══════════════════════════════════════════════════════════╝${NC}"
echo ""

# Step 1: Host System & Hardware Performance Optimization (First!)
if [ -f /etc/nv_tegra_release ]; then
    section "Step 1: Jetson Host Performance & Hardware Optimization"
    info "Configuring host clocks, udev rules, network UDP buffers, user groups, and LiDAR static IP first."
    if [ -t 0 ]; then
        read -p "Run Jetson host hardware performance & latency optimization (sudo required)? (y/n) " -n 1 -r
        echo ""
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            bash "$REPO_DIR/scripts/setup/setup-jetson.sh"
        else
            warn "Jetson host optimization skipped by user."
        fi
    else
        bash "$REPO_DIR/scripts/setup/setup-jetson.sh"
    fi
elif [ "$(uname)" = "Linux" ]; then
    section "Step 1: Linux Host Network & Wi-Fi Latency Tuning"
    if [ -t 0 ]; then
        read -p "Run Linux host network UDP buffer & Wi-Fi latency tuning (sudo required)? (y/n) " -n 1 -r
        echo ""
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            bash "$REPO_DIR/scripts/setup/setup-network-host.sh"
        else
            warn "Linux host network latency tuning skipped by user."
        fi
    else
        bash "$REPO_DIR/scripts/setup/setup-network-host.sh"
    fi
else
    section "Step 1: Host Environment Audit"
    info "Running on macOS developer workstation."
fi

# Step 2: System & Workspace Bootstrap
section "Step 2: System & Workspace Bootstrap"
bash "$REPO_DIR/scripts/setup/bootstrap.sh"

# Step 3: Platform Configuration
section "Step 3: Platform & Environment Configuration"
bash "$REPO_DIR/scripts/setup/configure.sh"

# Load derived configuration
set -a
source "$REPO_DIR/.env"
set +a

SERVICE="${DOCKER_PROFILE:-general}"

# Step 4: Docker Container Stack (The Final Step)
section "Step 4: Build & Launch Container Stack (Final Step)"

info "Validating Docker engine & container runtime..."
if ! command -v docker >/dev/null 2>&1; then
    error "Docker is not installed. Please install Docker before launching the container stack."
fi
if ! docker compose version >/dev/null 2>&1; then
    error "'docker compose' plugin is not installed."
fi
if ! docker info >/dev/null 2>&1; then
    error "Docker daemon is not running or current user lacks group permissions (run: sudo usermod -aG docker \$USER && newgrp docker)."
fi

info "Building Docker image for ${CYAN}$SERVICE${NC} service..."
docker compose --env-file "$REPO_DIR/.env" build "$SERVICE"

info "Launching container stack..."
docker compose --env-file "$REPO_DIR/.env" --profile "$SERVICE" up -d

success "F110th setup completed successfully! 🎉"
info "To attach to container: ${CYAN}make attach${NC} (or ${CYAN}docker compose exec $SERVICE /bin/bash${NC})"
