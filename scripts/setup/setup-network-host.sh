#!/bin/bash
# ==============================================================================
#  F110th — Linux Host Network & Wi-Fi Latency Tuning
#  Applies kernel socket buffer ceiling (2GB) & disables Wi-Fi power save
#  (NetworkManager protocol-level, Intel iwlwifi & Realtek rtw88 driver-level)
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
JETSON_CFG_DIR="$REPO_DIR/configs/jetson"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

info()    { echo -e "${CYAN}[INFO]${NC}  $*"; }
success() { echo -e "${GREEN}[✓]${NC}    $*"; }
warn()    { echo -e "${YELLOW}[!]${NC}    $*"; }

if [ "$(uname)" != "Linux" ]; then
    warn "Host OS is not Linux. Skipping Linux network latency tuning."
    exit 0
fi

info "Starting Linux Host Network & Wi-Fi Latency Optimization..."

# 1. Network UDP Buffer Tuning
info "Applying kernel sysctl UDP socket buffer tuning..."
sudo cp "$JETSON_CFG_DIR/99-f1tenth-network.conf" /etc/sysctl.d/
sudo sysctl -p /etc/sysctl.d/99-f1tenth-network.conf
success "Kernel UDP socket buffers set to 2GB (2147483647 bytes)."

# 2. Wi-Fi Power Saving Disable (Protocol Level)
info "Configuring NetworkManager Wi-Fi latency settings..."
if [ -d /etc/NetworkManager/conf.d ]; then
    sudo cp "$JETSON_CFG_DIR/20-f1tenth-wifi-powersave.conf" /etc/NetworkManager/conf.d/
    sudo systemctl restart NetworkManager || true
    success "Protocol-level Wi-Fi power saving disabled."
fi

# 3. Wi-Fi Power Saving Disable (Intel Driver Level)
if [ -d /etc/modprobe.d ] && [ -f "$JETSON_CFG_DIR/iwlwifi.conf" ]; then
    sudo cp "$JETSON_CFG_DIR/iwlwifi.conf" /etc/modprobe.d/
    success "Intel iwlwifi driver power management forced to performance mode."
fi

# 4. Wi-Fi Power Saving Disable (Realtek Driver Level)
if [ -d /etc/modprobe.d ] && [ -f "$JETSON_CFG_DIR/rtw88.conf" ]; then
    sudo cp "$JETSON_CFG_DIR/rtw88.conf" /etc/modprobe.d/
    success "Realtek rtw88 driver power management forced to performance mode."
fi

info "====================================================="
success "Host network & Wi-Fi latency tuning complete!"
info "====================================================="
