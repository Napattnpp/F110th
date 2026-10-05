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

# Step 1: Bootstrap (Dependencies, .env initialization, virtual environment)
section "Step 1: System & Workspace Bootstrap"
bash "$REPO_DIR/scripts/setup/bootstrap.sh"

# Step 2: Platform Configuration
section "Step 2: Platform Configuration"
bash "$REPO_DIR/scripts/setup/configure.sh"

# Load derived configuration
set -a
source "$REPO_DIR/.env"
set +a

SERVICE="${DOCKER_PROFILE:-general}"

# Step 3: Host Optimization & Latency Tuning
if [ -f /etc/nv_tegra_release ]; then
    section "Step 3: Jetson Host Optimization"
    read -p "Run Jetson host performance & latency optimization (sudo required)? (y/n) " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        bash "$REPO_DIR/scripts/host/setup-jetson.sh"
    else
        warn "Jetson host optimization skipped."
    fi
elif [ "$(uname)" = "Linux" ]; then
    section "Step 3: Linux Host Network & Wi-Fi Latency Tuning"
    read -p "Run Linux host network UDP buffer & Wi-Fi latency tuning (sudo required)? (y/n) " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        bash "$REPO_DIR/scripts/host/setup-network-host.sh"
    else
        warn "Linux host network latency tuning skipped."
    fi
fi

# Step 4: Docker Build & Launch
section "Step 4: Build & Launch Container Stack"

info "Building Docker image for ${CYAN}$SERVICE${NC} service..."
docker compose --env-file "$REPO_DIR/.env" build "$SERVICE"

info "Launching container stack..."
docker compose --env-file "$REPO_DIR/.env" --profile "$SERVICE" up -d

success "F110th setup completed successfully! 🎉"
info "To attach to container: ${CYAN}make attach${NC} (or ${CYAN}docker compose exec $SERVICE /bin/bash${NC})"
