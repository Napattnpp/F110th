#!/bin/bash
# ==============================================================================
#  F1TENTH Trackside Pitlane Session Orchestrator
#  Launches an interactive, 4-pane tmux session with pre-configured ROS 2 layouts:
#    Pane 1 (Top-Left):     base_launch.py (Sensors, Odometry, EKF, AMCL)
#    Pane 2 (Top-Right):    head_to_head_launch.py (Planner & Controller)
#    Pane 3 (Bottom-Left):  Diagnostics (Topic rates / Teleop override)
#    Pane 4 (Bottom-Right): System Health / Monitor
# ==============================================================================

set -e

# Enforce container-safe CycloneDDS URI if running inside container
if [ -f /.dockerenv ] || [ -f /etc/cyclonedds.xml ]; then
    export CYCLONEDDS_URI="file:///etc/cyclonedds.xml"
fi

SESSION_NAME="f110_pitlane"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

MAP_NAME="${1:-ifac_roboracer}"
if [ -n "$2" ]; then
    SIM_MODE="$2"
elif [ -f /etc/nv_tegra_release ] || [ "${DOCKER_PROFILE:-}" = "jetson" ]; then
    # Physical Jetson onboard compute defaults to real car
    SIM_MODE="false"
else
    # Mac / workstation development defaults to simulation
    SIM_MODE="true"
fi

# If already inside tmux, open a new window instead of nesting
if [ -n "$TMUX" ]; then
    echo "[!] Currently inside tmux. Creating new window 'pitlane'..."
    tmux new-window -n "pitlane" -c "${REPO_ROOT}"
    tmux split-window -h -c "${REPO_ROOT}"
    tmux split-window -v -c "${REPO_ROOT}"
    tmux select-pane -t 1
    tmux split-window -v -c "${REPO_ROOT}"
    tmux select-pane -t 1
    exit 0
fi

# If session already exists, attach to it
if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo "Attaching to existing pitlane session '${SESSION_NAME}'..."
    tmux attach-session -t "$SESSION_NAME"
    exit 0
fi

echo "Creating F1TENTH Pitlane Session: ${SESSION_NAME} (Map: ${MAP_NAME}, Sim: ${SIM_MODE})..."

# Window 1: Racing Dashboard
tmux new-session -d -s "$SESSION_NAME" -n "race" -c "${REPO_ROOT}"

# Enforce clean CycloneDDS environment inside tmux session
if [ -f /.dockerenv ] || [ -f /etc/cyclonedds.xml ]; then
    tmux set-environment -t "$SESSION_NAME" CYCLONEDDS_URI "file:///etc/cyclonedds.xml"
fi

# Split into 4 panes:
# 1. Split vertically into left and right
tmux split-window -h -t "${SESSION_NAME}:1" -c "${REPO_ROOT}"
# 2. Split left half horizontally (top-left & bottom-left)
tmux split-window -v -t "${SESSION_NAME}:1.1" -c "${REPO_ROOT}"
# 3. Split right half horizontally (top-right & bottom-right)
tmux split-window -v -t "${SESSION_NAME}:1.3" -c "${REPO_ROOT}"

# Pre-populate Pane 1 (Top-Left): Base Bringup
tmux send-keys -t "${SESSION_NAME}:1.1" "# 1. Base Bringup (Sensors & Localization)" C-m
tmux send-keys -t "${SESSION_NAME}:1.1" "ros2 launch race_bringup base_launch.py sim:=${SIM_MODE} map_name:=${MAP_NAME}"

# Pre-populate Pane 3 (Top-Right): Head to Head
tmux send-keys -t "${SESSION_NAME}:1.3" "# 2. Race Controller & Head-to-Head Planner" C-m
tmux send-keys -t "${SESSION_NAME}:1.3" "ros2 launch race_bringup head_to_head_launch.py sim:=${SIM_MODE}"

# Pre-populate Pane 2 (Bottom-Left): Foxglove Bridge
tmux send-keys -t "${SESSION_NAME}:1.2" "# 3. Foxglove WebSocket Bridge (ws://localhost:8765)" C-m
tmux send-keys -t "${SESSION_NAME}:1.2" "ros2 launch foxglove_bridge foxglove_bridge_launch.xml"

# Pane 4 (Bottom-Right): Clean Interactive Shell (left empty per operator preference)

# Window 2: SLAM & Map Tools
tmux new-window -t "$SESSION_NAME" -n "mapping" -c "${REPO_ROOT}"
tmux split-window -h -t "${SESSION_NAME}:2" -c "${REPO_ROOT}"
tmux send-keys -t "${SESSION_NAME}:2.1" "# SLAM Mapping" C-m
tmux send-keys -t "${SESSION_NAME}:2.1" "make mapping MAP=${MAP_NAME}"
tmux send-keys -t "${SESSION_NAME}:2.2" "# Finish Map & Pipeline" C-m
tmux send-keys -t "${SESSION_NAME}:2.2" "make finish_map MAP=${MAP_NAME}"

# Select Window 1, Top-Left pane
tmux select-window -t "${SESSION_NAME}:1"
tmux select-pane -t "${SESSION_NAME}:1.1"

echo "Launching tmux dashboard..."
tmux attach-session -t "$SESSION_NAME"

