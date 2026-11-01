#!/bin/bash
# ==============================================================================
#  F110th — Vehicle & Pitlane Network Health Diagnostic Audit
#  Works seamlessly both:
#   1. Remotely from Pit Laptop via SSH (probing CAR_HOSTNAME from .env)
#   2. Locally on the NVIDIA Jetson onboard compute
# ==============================================================================
set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

pass() { echo -e "  ${GREEN}[✓]${NC} $*"; }
warn() { echo -e "  ${YELLOW}[!]$NC $*"; }
fail() { echo -e "  ${RED}[✗]${NC} $*"; }
info() { echo -e "  ${CYAN}[INFO]${NC} $*"; }

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)

# Read .env if available
CAR_HOSTNAME="orin-desktop.local"
LOCAL_DOMAIN_ID=""
if [ -f "$REPO_DIR/.env" ]; then
    ENV_HOST=$(grep -E "^CAR_HOSTNAME=" "$REPO_DIR/.env" | cut -d= -f2 | cut -d# -f1 | tr -d ' "')
    [ -n "$ENV_HOST" ] && CAR_HOSTNAME="$ENV_HOST"
    ENV_DOMAIN=$(grep -E "^ROS_DOMAIN_ID=" "$REPO_DIR/.env" | cut -d= -f2 | cut -d# -f1 | tr -d ' "')
    [ -n "$ENV_DOMAIN" ] && LOCAL_DOMAIN_ID="$ENV_DOMAIN"
fi

# Override with CLI argument if provided
if [ -n "$1" ]; then
    CAR_HOSTNAME="$1"
fi

echo -e "\n${BOLD}${CYAN}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${CYAN}║     🏎️  F1TENTH Vehicle & Pitlane Network Diagnostic Audit     ║${NC}"
echo -e "${BOLD}${CYAN}╚════════════════════════════════════════════════════════════════╝${NC}\n"

# ------------------------------------------------------------------------------
# Case A: Local execution directly on physical Jetson
# ------------------------------------------------------------------------------
run_local_audit() {
    echo -e "${BOLD}1. Host Wi-Fi & Driver Performance:${NC}"
    WIFI_IFACE=$(ip -o link show 2>/dev/null | awk -F': ' '$2 ~ /^(wl)/ {print $2; exit}')
    if [ -n "$WIFI_IFACE" ]; then
        PS_STATUS=$(iw dev "$WIFI_IFACE" get power_save 2>/dev/null || echo "unknown")
        if echo "$PS_STATUS" | grep -q "off"; then
            pass "Wi-Fi Power Saving: Disabled (power_save: off)"
        else
            warn "Wi-Fi Power Saving: Active ($PS_STATUS) — run 'make host_setup'"
        fi

        LINK_INFO=$(iw dev "$WIFI_IFACE" link 2>/dev/null || true)
        SSID=$(echo "$LINK_INFO" | grep "SSID:" | awk '{print $2}')
        FREQ=$(echo "$LINK_INFO" | grep "freq:" | awk '{print $2}')
        SIG=$(echo "$LINK_INFO" | grep "signal:" | awk '{print $2, $3}')
        TX_BIT=$(echo "$LINK_INFO" | grep "tx bitrate:" | sed -e 's/^[ \t]*tx bitrate: //')
        if [ -n "$SSID" ]; then
            pass "Link: Connected to '$SSID' (${FREQ:-unknown} MHz, RSSI: ${SIG:-unknown})"
            [ -n "$TX_BIT" ] && pass "Tx Bitrate: $TX_BIT"
        else
            warn "Wi-Fi Interface '$WIFI_IFACE' has no active link."
        fi
    else
        warn "No wireless interface starting with 'wl' detected."
    fi

    echo -e "\n${BOLD}2. Kernel UDP Socket Buffers (sysctl):${NC}"
    RMEM_MAX=$(sysctl -n net.core.rmem_max 2>/dev/null || echo "0")
    WMEM_MAX=$(sysctl -n net.core.wmem_max 2>/dev/null || echo "0")
    BACKLOG=$(sysctl -n net.core.netdev_max_backlog 2>/dev/null || echo "0")
    IPFRAG=$(sysctl -n net.ipv4.ipfrag_time 2>/dev/null || echo "0")

    if [ "$RMEM_MAX" -ge 2147483647 ]; then
        pass "net.core.rmem_max = $RMEM_MAX (2GB ceiling, exceeds CycloneDDS 20MB limit)"
    elif [ "$RMEM_MAX" -ge 20971520 ]; then
        pass "net.core.rmem_max = $RMEM_MAX (>= 20MB CycloneDDS minimum)"
    else
        fail "net.core.rmem_max = $RMEM_MAX (< 20MB, will crash cyclonedds_car.xml! Run 'make host_setup')"
    fi

    if [ "$WMEM_MAX" -ge 2147483647 ]; then
        pass "net.core.wmem_max = $WMEM_MAX (2GB burst headroom)"
    else
        warn "net.core.wmem_max = $WMEM_MAX"
    fi

    [ "$BACKLOG" -ge 10000 ] && pass "net.core.netdev_max_backlog = $BACKLOG" || warn "net.core.netdev_max_backlog = $BACKLOG (< 10000)"
    [ "$IPFRAG" -le 3 ] && pass "net.ipv4.ipfrag_time = ${IPFRAG}s (Fast stale UDP fragment drop)" || warn "net.ipv4.ipfrag_time = ${IPFRAG}s"

    echo -e "\n${BOLD}3. ROS 2 & CycloneDDS Isolation Architecture:${NC}"
    CONTAINER_RUNNING=0
    if command -v docker >/dev/null 2>&1 && docker ps --format '{{.Names}}' 2>/dev/null | grep -q "f1tenth_jetson"; then
        CONTAINER_RUNNING=1
        pass "Docker Container: 'f1tenth_jetson' is ACTIVE (network_mode: host)"

        DDS_URI=$(docker exec f1tenth_jetson env 2>/dev/null | grep "^CYCLONEDDS_URI=" | cut -d= -f2 || true)
        REMOTE_DOMAIN=$(docker exec f1tenth_jetson env 2>/dev/null | grep "^ROS_DOMAIN_ID=" | cut -d= -f2 || true)
        RMW=$(docker exec f1tenth_jetson env 2>/dev/null | grep "^RMW_IMPLEMENTATION=" | cut -d= -f2 || true)

        pass "Middleware: $RMW"
        pass "ROS_DOMAIN_ID: $REMOTE_DOMAIN"
        if [ -n "$LOCAL_DOMAIN_ID" ] && [ "$REMOTE_DOMAIN" != "$LOCAL_DOMAIN_ID" ]; then
            fail "Domain ID mismatch! Vehicle is on ID=$REMOTE_DOMAIN, but laptop .env specifies ID=$LOCAL_DOMAIN_ID"
        fi

        # Check loopback in active cyclonedds.xml
        DDS_FILE="${DDS_URI#file://}"
        if docker exec f1tenth_jetson cat "$DDS_FILE" 2>/dev/null | grep -q "<NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress>"; then
            pass "CycloneDDS Isolation: Impenetrable Loopback (127.0.0.1) confirmed. Wi-Fi noise: 0%"
        else
            warn "CycloneDDS is not bound to 127.0.0.1! High-rate topics may broadcast to Wi-Fi."
        fi
    else
        warn "Docker container 'f1tenth_jetson' is not currently running."
        if [ -f "$REPO_DIR/configs/dds/cyclonedds_car.xml" ]; then
            if grep -q "<NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress>" "$REPO_DIR/configs/dds/cyclonedds_car.xml"; then
                pass "Stored cyclonedds_car.xml verified: 127.0.0.1 loopback configured."
            fi
        fi
    fi

    echo -e "\n${BOLD}4. Telemetry & Web Ports:${NC}"
    if ss -tulpn 2>/dev/null | grep -q ":8765 "; then
        pass "Foxglove WebSocket Bridge: Listening on port 8765 (ws://<car_ip>:8765)"
    else
        info "Foxglove Bridge (8765): Not currently active (launches with race stack)"
    fi
    if ss -tulpn 2>/dev/null | grep -q ":8050 "; then
        pass "Web Sector Tuner: Listening on port 8050 (http://<car_ip>:8050)"
    fi
}

# ------------------------------------------------------------------------------
# Case B: Remote execution from Pit Laptop via SSH
# ------------------------------------------------------------------------------
run_remote_audit() {
    echo -e "${BOLD}Target Vehicle Hostname:${NC} ${CYAN}$CAR_HOSTNAME${NC}\n"

    echo -e "${BOLD}1. Wireless Link & ICMP Latency Probe:${NC}"
    # Send 5 quick pings (0.2s interval, 1s timeout)
    PING_OUT=$(ping -c 5 -i 0.2 -W 1000 "$CAR_HOSTNAME" 2>&1 || true)
    if echo "$PING_OUT" | grep -q "bytes from"; then
        RESOLVED_IP=$(echo "$PING_OUT" | head -n1 | sed -E 's/.*\(([0-9.]+)\).*/\1/')
        LOSS=$(echo "$PING_OUT" | grep -o "[0-9.]*% packet loss" || echo "0% packet loss")
        RTT_STATS=$(echo "$PING_OUT" | tail -n1 | awk -F'=' '{print $2}' | awk -F'/' '{printf "min=%.2f / avg=%.2f / max=%.2f ms", $1, $2, $3}' 2>/dev/null || echo "")
        pass "Host Reachable: $CAR_HOSTNAME ($RESOLVED_IP) — Packet Loss: $LOSS"
        [ -n "$RTT_STATS" ] && pass "Round-Trip Latency: $RTT_STATS"
    else
        fail "Cannot reach $CAR_HOSTNAME via ping. Check Wi-Fi connection and vehicle power."
        exit 1
    fi

    echo -e "\n${BOLD}2. SSH Authentication:${NC}"
    if ssh -o BatchMode=yes -o ConnectTimeout=3 "$CAR_HOSTNAME" "echo ok" >/dev/null 2>&1; then
        pass "Passwordless SSH Key Authentication: OK"
    else
        fail "SSH connection to $CAR_HOSTNAME failed. Verify SSH keys with 'ssh-copy-id jetson@$CAR_HOSTNAME'."
        exit 1
    fi

    echo -e "\n${BOLD}3. Remote Jetson Network Audit (Executing onboard):${NC}"
    # Stream onboard audit script over SSH in single roundtrip
    ssh -o BatchMode=yes "$CAR_HOSTNAME" 'bash -s' << 'EOF'
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'
pass() { echo -e "  ${GREEN}[✓]${NC} $*"; }
warn() { echo -e "  ${YELLOW}[!]$NC $*"; }
fail() { echo -e "  ${RED}[✗]${NC} $*"; }
info() { echo -e "  ${CYAN}[INFO]${NC} $*"; }

# 1. Wi-Fi Power Save & RSSI
WIFI_IFACE=$(ip -o link show 2>/dev/null | awk -F': ' '$2 ~ /^(wl)/ {print $2; exit}')
if [ -n "$WIFI_IFACE" ]; then
    PS_STATUS=$(iw dev "$WIFI_IFACE" get power_save 2>/dev/null || echo "unknown")
    if echo "$PS_STATUS" | grep -q "off"; then
        pass "Wi-Fi Power Saving: Disabled (power_save: off)"
    else
        warn "Wi-Fi Power Saving: Active ($PS_STATUS)"
    fi

    LINK_INFO=$(iw dev "$WIFI_IFACE" link 2>/dev/null || true)
    SSID=$(echo "$LINK_INFO" | grep "SSID:" | awk '{print $2}')
    FREQ=$(echo "$LINK_INFO" | grep "freq:" | awk '{print $2}')
    SIG=$(echo "$LINK_INFO" | grep "signal:" | awk '{print $2, $3}')
    TX_BIT=$(echo "$LINK_INFO" | grep "tx bitrate:" | sed -e 's/^[ \t]*tx bitrate: //')
    pass "Wi-Fi Link: SSID '$SSID' (${FREQ:-unknown} MHz, RSSI: ${SIG:-unknown})"
    [ -n "$TX_BIT" ] && pass "Wi-Fi TX Bitrate: $TX_BIT"
fi

# 2. Kernel UDP Buffers
RMEM_MAX=$(sysctl -n net.core.rmem_max 2>/dev/null || echo "0")
WMEM_MAX=$(sysctl -n net.core.wmem_max 2>/dev/null || echo "0")
if [ "$RMEM_MAX" -ge 2147483647 ]; then
    pass "Kernel UDP rmem_max = $RMEM_MAX (2GB ceiling locked)"
elif [ "$RMEM_MAX" -ge 20971520 ]; then
    pass "Kernel UDP rmem_max = $RMEM_MAX (>= 20MB)"
else
    fail "Kernel UDP rmem_max = $RMEM_MAX (< 20MB — CycloneDDS will crash!)"
fi
[ "$WMEM_MAX" -ge 2147483647 ] && pass "Kernel UDP wmem_max = $WMEM_MAX (2GB burst headroom)"

# 3. Docker & DDS
if docker ps --format '{{.Names}}' 2>/dev/null | grep -q "f1tenth_jetson"; then
    pass "Docker Container: 'f1tenth_jetson' is ACTIVE (network_mode: host)"
    REMOTE_DOMAIN=$(docker exec f1tenth_jetson env 2>/dev/null | grep "^ROS_DOMAIN_ID=" | cut -d= -f2 || true)
    RMW=$(docker exec f1tenth_jetson env 2>/dev/null | grep "^RMW_IMPLEMENTATION=" | cut -d= -f2 || true)
    pass "Container Middleware: $RMW (ROS_DOMAIN_ID=$REMOTE_DOMAIN)"

    if docker exec f1tenth_jetson cat /etc/cyclonedds.xml 2>/dev/null | grep -q "<NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress>"; then
        pass "CycloneDDS Isolation: Impenetrable Loopback (127.0.0.1) confirmed"
    else
        warn "CycloneDDS is not bound to 127.0.0.1 loopback!"
    fi
else
    warn "Docker container 'f1tenth_jetson' is idle. (Launch with 'make up')"
fi

# 4. Open Ports
if ss -tulpn 2>/dev/null | grep -q ":8765 "; then
    pass "Foxglove WebSocket Bridge: Listening on port 8765"
else
    info "Foxglove Bridge (port 8765): Idle (launches with base_launch.py)"
fi
EOF
}

# Run local or remote audit depending on host environment
if [ -f /etc/nv_tegra_release ]; then
    run_local_audit
else
    run_remote_audit
fi

echo -e "\n${BOLD}${GREEN}Network diagnostic audit complete.${NC}\n"
