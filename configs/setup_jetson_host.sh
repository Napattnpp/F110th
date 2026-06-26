#!/bin/bash
set -e

# Dynamically find the directory where this script is located
REPO_DIR=$(dirname "$(realpath "$0")")
CONFIG_DIR="$REPO_DIR/configs"

sudo nvpmodel -m 0
sudo jetson_clocks

echo "Starting Jetson Host Setup (Dotfiles Method)..."

# 1. Setup Screen (xhost)
echo "Configuring xhost..."
if ! grep -q "xhost +local:root" ~/.bashrc; then
    echo -e '\n# Allow local root (Docker containers) to access X11 display\nif [ -n "$DISPLAY" ]; then\n    xhost +local:root >/dev/null 2>&1\nfi' >> ~/.bashrc
fi

# 2. Setup udev rules
echo "Copying udev rules..."
sudo cp "$CONFIG_DIR/99-f1tenth-usb.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# 3. UDP Buffer Size
echo "Copying OS UDP configs..."
sudo cp "$CONFIG_DIR/99-jetson-udp.conf" /etc/sysctl.d/
sudo sysctl -p /etc/sysctl.d/99-jetson-udp.conf

# 4. Disable USB Autosuspend & Increase USB FS Memory Limit
echo "Optimizing USB kernel parameters..."
EXTLINUX_CONF="/boot/extlinux/extlinux.conf"
if [ ! -f "$EXTLINUX_CONF.bak" ]; then
    sudo cp "$EXTLINUX_CONF" "$EXTLINUX_CONF.bak"
fi
if ! grep -q "usbcore.autosuspend=-1" "$EXTLINUX_CONF"; then
    sudo sed -i 's/\(^[[:space:]]*APPEND[[:space:]]\)/\1usbcore.autosuspend=-1 /' "$EXTLINUX_CONF"
fi
if ! grep -q "usbcore.usbfs_memory_mb=" "$EXTLINUX_CONF"; then
    sudo sed -i 's/\(^[[:space:]]*APPEND[[:space:]]\)/\1usbcore.usbfs_memory_mb=1024 /' "$EXTLINUX_CONF"
fi

# 5. Docker Aliases
echo "Configuring Bash aliases..."
if ! grep -q "alias startf1" ~/.bashrc; then
    echo -e "\n# F1TENTH Docker Aliases\nalias startf1='sudo docker start f1tenth_jetson'\nalias stopf1='sudo docker stop f1tenth_jetson'\nalias execf1='sudo docker exec -it f1tenth_jetson /bin/bash'" >> ~/.bashrc
fi

# 6. Install jtop
echo "Installing jetson-stats..."
sudo apt-get install -y python3-pip
sudo -H pip3 install -U jetson-stats

# 7. Log Directory
mkdir -p ~/f110th_logs

# 8. Tmux Workspace Script (Symlink)
echo "Symlinking tmux workspace launcher..."
ln -sf "$CONFIG_DIR/start_f1tenth_workspace.sh" ~/start_f1tenth_workspace.sh
chmod +x ~/start_f1tenth_workspace.sh

# 9. Wi-Fi Power Saving
echo "Copying NetworkManager config..."
sudo cp "$CONFIG_DIR/default-wifi-powersave-on.conf" /etc/NetworkManager/conf.d/
sudo systemctl restart NetworkManager

# Wait for network connection to be restored before cloning
echo "Waiting for network connection to be restored..."
for i in {1..30}; do
    if ping -c 1 -W 1 github.com >/dev/null 2>&1; then
        echo "Network connection restored."
        break
    fi
    echo -n "."
    sleep 1
done
echo ""

# 10. Tmux Configuration (Symlink & TPM)
echo "Symlinking .tmux.conf and installing TPM..."
if [ ! -d "$HOME/.tmux/plugins/tpm" ]; then
    git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm
fi
ln -sf "$CONFIG_DIR/.tmux.conf" ~/.tmux.conf

# 11. Enable Real-Time Priority and Memory Locking limits
echo "Configuring real-time scheduling limits..."
sudo bash -c "cat << 'EOF' > /etc/security/limits.d/99-realtime.conf
*    soft    rtprio          99
*    hard    rtprio          99
*    soft    memlock         unlimited
*    hard    memlock         unlimited
EOF"

# 12. Force CPU Scaling Governor to Performance
echo "Setting CPU scaling governor to performance..."
if [ -d /sys/devices/system/cpu/cpu0/cpufreq ]; then
    echo "performance" | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor >/dev/null
fi

# 13. Persistent jetson_clocks service
echo "Configuring persistent jetson_clocks service..."
SERVICE_FILE="/etc/systemd/system/jetson-clocks-init.service"
sudo bash -c "cat << 'EOF' > $SERVICE_FILE
[Unit]
Description=Lock Jetson Clocks to Maximum Performance
After=syslog.target NVPower.service

[Service]
Type=oneshot
ExecStart=/usr/bin/jetson_clocks
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF"
sudo systemctl daemon-reload
sudo systemctl enable jetson-clocks-init.service

echo "====================================================="
echo "Setup complete!"
echo "1. Open tmux and press 'Prefix + I' to fetch plugins."
echo "2. REBOOT your Jetson to apply the boot config changes (UDP buffers & persistent clocks)."
echo "====================================================="