#!/bin/bash
# ==============================================================================
#  F110th — NVIDIA Jetson Host Performance & System Setup
#  Applies all 8 Jetson hardware performance locks idempotently using
#  version-controlled configuration files in configs/jetson/.
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
error()   { echo -e "${RED}[✗]${NC}    $*"; exit 1; }

if [ ! -f /etc/nv_tegra_release ]; then
    warn "NVIDIA Jetson Tegra release file not found. Skipping Jetson host setup."
    exit 0
fi

info "Starting Jetson Host Optimization..."

# 0. User Group Membership (dialout, tty, input, video, i2c, docker)
TARGET_USER="${SUDO_USER:-$USER}"
info "Adding user '${TARGET_USER}' to hardware & container groups..."
sudo usermod -aG dialout,tty,input,video,i2c,docker "$TARGET_USER" || warn "Failed to update user groups"
success "User groups updated (dialout, tty, input, video, i2c, docker)."

# 1. Lock NVPModel MAXN & jetson_clocks
info "Setting NVPModel MAXN (mode 0) & locking clocks..."
sudo nvpmodel -m 0 || warn "Failed to set nvpmodel mode 0"
sudo jetson_clocks || warn "Failed to run jetson_clocks"

# 2. Setup udev rules
info "Installing versioned udev rules..."
sudo cp "$JETSON_CFG_DIR/99-f1tenth-serial.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
success "udev rules installed."

# 3. Network UDP Buffer Tuning
info "Applying kernel sysctl UDP socket buffer tuning..."
sudo cp "$JETSON_CFG_DIR/99-f1tenth-network.conf" /etc/sysctl.d/
sudo sysctl -p /etc/sysctl.d/99-f1tenth-network.conf
success "Kernel UDP socket buffers set to 2GB (2147483647 bytes)."

# 4. USB Kernel Parameters (autosuspend & usbfs memory)
info "Optimizing USB kernel parameters in extlinux.conf..."
EXTLINUX_CONF="/boot/extlinux/extlinux.conf"
if [ -f "$EXTLINUX_CONF" ]; then
    if [ ! -f "$EXTLINUX_CONF.bak" ]; then
        sudo cp "$EXTLINUX_CONF" "$EXTLINUX_CONF.bak"
    fi
    if ! grep -q "usbcore.autosuspend=-1" "$EXTLINUX_CONF"; then
        sudo sed -i 's/\(^[[:space:]]*APPEND[[:space:]]\)/\1usbcore.autosuspend=-1 /' "$EXTLINUX_CONF"
    fi
    if ! grep -q "usbcore.usbfs_memory_mb=" "$EXTLINUX_CONF"; then
        sudo sed -i 's/\(^[[:space:]]*APPEND[[:space:]]\)/\1usbcore.usbfs_memory_mb=1024 /' "$EXTLINUX_CONF"
    fi
    success "USB kernel parameters updated."
fi

# 5. Wi-Fi Power Saving Disable (Protocol & Driver Level)
info "Configuring Wi-Fi latency settings (NetworkManager, Intel, & Realtek drivers)..."
if [ -d /etc/NetworkManager/conf.d ]; then
    sudo cp "$JETSON_CFG_DIR/20-f1tenth-wifi-powersave.conf" /etc/NetworkManager/conf.d/
    sudo nmcli general reload conf 2>/dev/null || sudo systemctl reload NetworkManager || true
    success "Protocol-level Wi-Fi power saving disabled."
fi

if [ -d /etc/modprobe.d ] && [ -f "$JETSON_CFG_DIR/iwlwifi.conf" ]; then
    sudo cp "$JETSON_CFG_DIR/iwlwifi.conf" /etc/modprobe.d/
    success "Intel iwlwifi driver power management forced to performance mode."
fi

if [ -d /etc/modprobe.d ] && [ -f "$JETSON_CFG_DIR/rtw88.conf" ]; then
    sudo cp "$JETSON_CFG_DIR/rtw88.conf" /etc/modprobe.d/
    success "Realtek rtw88 driver power management forced to performance mode."
fi

# 6. Real-time Priority & Memory Locking Limits
info "Installing security limits for real-time priority..."
sudo cp "$JETSON_CFG_DIR/99-f1tenth-realtime.conf" /etc/security/limits.d/
success "Realtime security limits installed."

# 7. CPU Scaling Governor to Performance
info "Setting CPU scaling governor to performance..."
if [ -d /sys/devices/system/cpu/cpu0/cpufreq ]; then
    echo "performance" | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor >/dev/null || true
    success "CPU governor locked to performance mode."
fi

# 8. Persistent jetson_clocks Systemd Service
info "Installing persistent jetson_clocks systemd service..."
sudo cp "$JETSON_CFG_DIR/f1tenth-jetson-clocks.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now f1tenth-jetson-clocks.service || true
success "Persistent jetson_clocks systemd service enabled."

# 9. Ethernet Static IP for Hokuyo LiDAR (ERR-LIDAR-01)
info "Configuring LiDAR Ethernet static IP (192.168.0.15/24)..."
ETH_IFACE=$(ip -o link show 2>/dev/null | awk -F': ' '$2 ~ /^(en|eth)/ {print $2; exit}')
if [ -n "$ETH_IFACE" ]; then
    CON_NAME=$(nmcli -t -f NAME,DEVICE connection show 2>/dev/null | grep ":$ETH_IFACE" | cut -d: -f1 | head -n1 || echo "")
    if [ -z "$CON_NAME" ]; then
        CON_NAME="Wired connection 1"
    fi
    sudo nmcli connection modify "$CON_NAME" ipv4.addresses 192.168.0.15/24 ipv4.method manual ipv4.never-default yes 2>/dev/null || \
    sudo ip addr add 192.168.0.15/24 dev "$ETH_IFACE" 2>/dev/null || true
    success "Ethernet interface '$ETH_IFACE' static IP configured (192.168.0.15/24, never-default: yes)."
else
    warn "No physical Ethernet interface found (skipping LiDAR IP assignment)."
fi

# 10. Tmux Dotfile Symlink
info "Symlinking tmux configuration..."
ln -sf "$REPO_DIR/configs/tmux.conf" ~/.tmux.conf
success "Tmux config symlinked."

# 11. Hardware Diagnostic Audit (Non-destructive)
echo ""
echo -e "${CYAN}╔═══════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║  Physical Hardware & Driver Audit                         ║${NC}"
echo -e "${CYAN}╚═══════════════════════════════════════════════════════════╝${NC}"

# Check VESC
if [ -e /dev/sensors/vesc ]; then
    success "VESC Controller: Found (/dev/sensors/vesc -> $(readlink -f /dev/sensors/vesc))"
else
    warn "VESC Controller: Not detected at /dev/sensors/vesc. Check USB connection."
fi

# Check Gamepad
if ls /dev/input/js* >/dev/null 2>&1; then
    success "Gamepad / Joystick: Found ($(ls /dev/input/js* | tr '\n' ' '))"
else
    warn "Gamepad / Joystick: None connected. Pair via Bluetooth or plug in wireless receiver."
fi

# Check LiDAR ping
if ping -c 1 -W 1 192.168.0.10 >/dev/null 2>&1; then
    success "Hokuyo LiDAR: Reachable at 192.168.0.10 (ping OK)"
else
    warn "Hokuyo LiDAR: 192.168.0.10 not pingable. (Cable disconnected or LiDAR unpowered)"
fi

# Check Clocks & Governor
CUR_GOV=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo "unknown")
if [ "$CUR_GOV" = "performance" ]; then
    success "CPU Governor: performance"
else
    warn "CPU Governor: $CUR_GOV (expected performance)"
fi

info "====================================================="
success "Jetson host performance & hardware setup complete!"
info "====================================================="
