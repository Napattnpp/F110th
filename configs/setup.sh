#!/bin/bash
# ==============================================================================
#  F110th — Master Setup Script
#
#  USAGE:  ./setup.sh
#
#  This is the ONLY script you need to run. It handles:
#    1. Detects your platform (x86_64, ARM, Jetson, Apple Silicon)
#    2. Validates dependencies (Docker, Python)
#    3. Creates Python venv and installs requirements
#    4. Sets up colcon build output
#    5. Generates .env from f110th.config
#    6. Builds Docker image for your platform
#    7. Launches the container
#    8. (Optional) If Jetson: runs Jetson host optimizations
#
#  Everything is automated. Just run: ./setup.sh
# ==============================================================================
set -e

SCRIPT_DIR=$(dirname "$(realpath "$0")")
REPO_DIR=$(realpath "$SCRIPT_DIR/..")
CONFIG_FILE="$SCRIPT_DIR/f110th.config"
JETSON_HOST_SETUP_SCRIPT="$SCRIPT_DIR/setup_jetson_host.sh"

# ── Color definitions ──────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BLUE='\033[0;34m'
NC='\033[0m'

# ── Logging functions ──────────────────────────────────────────────────────────
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

# ══════════════════════════════════════════════════════════════════════════════
# BANNER
# ══════════════════════════════════════════════════════════════════════════════
echo ""
echo -e "${CYAN}╔═══════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║  F110th Setup                                             ║${NC}"
echo -e "${CYAN}║  Auto-detects platform • Builds Docker • Launches stack   ║${NC}"
echo -e "${CYAN}╚═══════════════════════════════════════════════════════════╝${NC}"
echo ""

# ══════════════════════════════════════════════════════════════════════════════
# STEP 0: Load Configuration
# ══════════════════════════════════════════════════════════════════════════════
section "Step 0: Load Configuration"

[ -f "$CONFIG_FILE" ] || error "f110th.config not found in $REPO_DIR"
source "$CONFIG_FILE"

info "Configuration loaded:"
info "  • ROS Domain ID:  ${CYAN}$ROS_DOMAIN_ID${NC}"
info "  • ROS Middleware: ${CYAN}$RMW_IMPLEMENTATION${NC}"
info "  • Racecar:        ${CYAN}$RACECAR_VERSION${NC}"
info "  • Docker Profile: ${CYAN}$DOCKER_PROFILE${NC}"

# ══════════════════════════════════════════════════════════════════════════════
# STEP 1: Detect Platform
# ══════════════════════════════════════════════════════════════════════════════
section "Step 1: Platform Detection"

ARCH=$(uname -m)
OS=$(uname -s)

info "System detected:"
info "  • Architecture: ${CYAN}$ARCH${NC}"
info "  • OS:           ${CYAN}$OS${NC}"

# Detect Jetson
IS_JETSON="no"
if [ -f /etc/nv_tegra_release ]; then
    IS_JETSON="yes"
    info "  • ${GREEN}✓ NVIDIA Jetson detected${NC}"
fi

# Determine docker service:
#   jetson  → ONLY when DOCKER_PROFILE=jetson AND real Jetson hardware confirmed
#   general → everything else (Pi 5, MacBook, Parallels, x86, unknown aarch64)
if [ "$DOCKER_PROFILE" = "jetson" ] && [ "$IS_JETSON" = "yes" ]; then
    SERVICE="jetson"
    success "Jetson confirmed (config + hardware) → using ${CYAN}jetson${NC} service"
elif [ "$DOCKER_PROFILE" = "jetson" ] && [ "$IS_JETSON" = "no" ]; then
    warn "DOCKER_PROFILE=jetson but no Jetson hardware detected"
    warn "Falling back to ${CYAN}general${NC} service (Pi 5 / MacBook / VM / x86?)"
    SERVICE="general"
else
    # general — covers: x86_64, arm64 (Apple Silicon), aarch64 non-Jetson (Pi 5, Parallels)
    SERVICE="general"
    info "Using ${CYAN}general${NC} service (arch: $ARCH)"
fi

# ══════════════════════════════════════════════════════════════════════════════
# STEP 2: Check Dependencies & Bootstrap Basic Tools
# ══════════════════════════════════════════════════════════════════════════════
section "Step 2: Check Dependencies"

# Bootstrap essential tools (curl, git)
BOOTSTRAP_NEEDED=no
for tool in curl git; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        BOOTSTRAP_NEEDED=yes
    fi
done

if [ "$BOOTSTRAP_NEEDED" = "yes" ]; then
    info "Bootstrap: Installing basic dependencies (curl, git)..."
    if [ "$OS" = "Linux" ]; then
        if command -v apt-get >/dev/null 2>&1; then
            sudo apt-get update
            for tool in curl git; do
                if ! command -v "$tool" >/dev/null 2>&1; then
                    info "Installing $tool..."
                    sudo apt-get install -y "$tool"
                fi
            done
        else
            warn "apt-get not found. Cannot auto-install basic tools (curl, git). Setup may fail if they are missing."
        fi
    elif [ "$OS" = "Darwin" ]; then
        if command -v brew >/dev/null 2>&1; then
            for tool in curl git; do
                if ! command -v "$tool" >/dev/null 2>&1; then
                    info "Installing $tool..."
                    brew install "$tool"
                fi
            done
        else
            warn "Homebrew not found. Cannot auto-install basic tools. Please install Homebrew (https://brew.sh) or install curl and git manually."
        fi
    fi
fi

# Verify bootstrap tools are present
if command -v curl >/dev/null 2>&1; then success "curl found"; else error "curl is missing and could not be installed"; fi
if command -v git  >/dev/null 2>&1; then success "git found";  else error "git is missing and could not be installed";  fi

# Python 3
if command -v python3 >/dev/null 2>&1; then
    PYTHON_VERSION=$(python3 --version)
    success "Python3 found: ${CYAN}$PYTHON_VERSION${NC}"
else
    info "Python3 not found. Attempting auto-installation..."
    if [ "$OS" = "Darwin" ]; then
        if command -v brew >/dev/null 2>&1; then
            info "Installing python3 via Homebrew..."
            brew install python
        else
            error "Python3 not found and Homebrew is not installed. Please install Python 3 manually or install Homebrew: https://brew.sh"
        fi
    elif [ "$OS" = "Linux" ]; then
        if command -v apt-get >/dev/null 2>&1; then
            info "Installing python3 via apt..."
            sudo apt-get update && sudo apt-get install -y python3 python3-venv python3-pip
        else
            error "Python3 not found. Please install Python3 manually."
        fi
    else
        error "Python3 not found. Please install Python3 manually."
    fi
    PYTHON_VERSION=$(python3 --version)
    success "Python3 installed: ${CYAN}$PYTHON_VERSION${NC}"
fi

# Docker (which includes compose in modern installations)
if command -v docker >/dev/null 2>&1; then
    DOCKER_VERSION=$(docker --version)
    success "Docker found: ${CYAN}$DOCKER_VERSION${NC}"
else
    info "Docker not found. Attempting auto-installation..."
    if [ "$OS" = "Darwin" ]; then
        if command -v brew >/dev/null 2>&1; then
            info "Installing Docker (cask) via Homebrew..."
            brew install --cask docker
            warn "Docker Desktop is being installed. Please open Docker from Applications to complete setup, then run this script again."
            exit 0
        else
            error "Docker not found and Homebrew is not installed. Please install Docker manually: https://docs.docker.com/get-docker/"
        fi
    elif [ "$OS" = "Linux" ]; then
        info "Installing Docker using the official convenience script..."
        curl -fsSL https://get.docker.com -o get-docker.sh
        sudo sh get-docker.sh
        rm get-docker.sh
        # Add current user to docker group
        sudo usermod -aG docker "$USER"
        warn "Docker installed successfully! Added user '$USER' to the 'docker' group."
        warn "Please log out and log back in (or restart your terminal session), then run setup.sh again."
        exit 0
    else
        error "Docker not found. Please install Docker manually."
    fi
fi

# Docker Compose
if docker compose version >/dev/null 2>&1; then
    success "Docker Compose found"
else
    info "Docker Compose not found. Attempting auto-installation..."
    if [ "$OS" = "Darwin" ]; then
        error "Docker Compose should be part of Docker Desktop for macOS. Please ensure Docker Desktop is running."
    elif [ "$OS" = "Linux" ]; then
        if command -v apt-get >/dev/null 2>&1; then
            info "Installing docker-compose-plugin..."
            sudo apt-get update && sudo apt-get install -y docker-compose-plugin
        else
            error "Docker Compose not found. Please install docker-compose-plugin or docker-compose manually."
        fi
    else
        error "Docker Compose not found. Please install docker-compose manually."
    fi
    if docker compose version >/dev/null 2>&1; then
        success "Docker Compose installed successfully"
    else
        error "Docker Compose installation check failed. Please install docker compose manually."
    fi
fi

# ══════════════════════════════════════════════════════════════════════════════
# STEP 3: Python Virtual Environment
# ══════════════════════════════════════════════════════════════════════════════
section "Step 3: Python Virtual Environment"

VENV_DIR="$REPO_DIR/.venv"
if [ ! -d "$VENV_DIR" ] || [ ! -f "$VENV_DIR/bin/pip" ] || ! "$VENV_DIR/bin/pip" --version >/dev/null 2>&1; then
    # On Ubuntu/Debian, python3-venv package is often separate from python3.
    # We test if 'ensurepip' module is importable (since lack of ensurepip causes the crash).
    if ! python3 -c "import ensurepip" >/dev/null 2>&1; then
        info "Python ensurepip module not found. Attempting to install the required venv package..."
        if [ "$OS" = "Linux" ] && command -v apt-get >/dev/null 2>&1; then
            # Extract the major.minor version of python3 (e.g., 3.12)
            PY_VERSION=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
            sudo apt-get update && sudo apt-get install -y "python${PY_VERSION}-venv" || sudo apt-get install -y python3-venv
        else
            error "python3-venv package is missing. Please install it using your system package manager."
        fi
    fi

    info "Creating Python venv..."
    # Ensure any existing broken directory is cleared first
    rm -rf "$VENV_DIR"
    python3 -m venv "$VENV_DIR"
    success "venv created at ${CYAN}.venv/${NC}"
else
    info "venv already exists and is healthy — skipping creation"
fi

if [ -f "$SCRIPT_DIR/requirements.txt" ]; then
    info "Installing Python requirements..."
    env -u PYTHONPATH "$VENV_DIR/bin/pip" install --quiet --upgrade pip setuptools wheel
    env -u PYTHONPATH "$VENV_DIR/bin/pip" install --quiet -r "$SCRIPT_DIR/requirements.txt"
    success "Python requirements installed"
else
    warn "requirements.txt not found (optional)"
fi

# ==============================================================================
# STEP 4: Build Output Setup
# ==============================================================================
section "Step 4: Build Output Setup"

BUILD_OUTPUT_DIR="$REPO_DIR/out"
info "Creating build output directories..."
mkdir -p "$BUILD_OUTPUT_DIR/build" "$BUILD_OUTPUT_DIR/install" "$BUILD_OUTPUT_DIR/logs"
success "Output directories ready at ${CYAN}out/{build,install,logs}${NC}"

# Generate X11 authority files for graphical forwarding inside the docker container
XAUTH="/tmp/.docker.xauth"
# Create file if missing
touch "$XAUTH" 2>/dev/null || true
# Extract current display authentication cookies to the shared location
if [ "$OS" != "Darwin" ] && [ -n "$DISPLAY" ]; then
    if command -v xauth >/dev/null 2>&1; then
        xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge - >/dev/null 2>&1 || true
    fi
    chmod a+r "$XAUTH" 2>/dev/null || true
fi

info "Writing environment file from f110th.config..."
cat > "$REPO_DIR/.env" <<EOF
# Auto-generated by setup.sh — do NOT commit this file
USER=${USER}
HOST_UID=$(id -u)
HOST_GID=$(id -g)
ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
RACECAR_VERSION=${RACECAR_VERSION}
DOCKER_PROFILE=${DOCKER_PROFILE}
DOCKERHUB_USER=${DOCKERHUB_USER}
IMAGE_NAME=${IMAGE_NAME}
XAUTHORITY=${XAUTH}
EOF
success ".env generated"

# ══════════════════════════════════════════════════════════════════════════════
# STEP 6: Verify Docker Setup Files
# ══════════════════════════════════════════════════════════════════════════════
section "Step 6: Verify Docker Setup Files"

[ -f "$REPO_DIR/docker-compose.yml" ] || error "docker-compose.yml not found"
success "docker-compose.yml found"

[ -f "$REPO_DIR/.devcontainer/docker/Dockerfile.general" ] || error ".devcontainer/docker/Dockerfile.general not found"
success "docker/Dockerfile.general found"

if [ "$SERVICE" = "jetson" ]; then
    [ -f "$REPO_DIR/.devcontainer/docker/Dockerfile.jetson" ] || error ".devcontainer/docker/Dockerfile.jetson not found"
    success "docker/Dockerfile.jetson found"
fi

# ══════════════════════════════════════════════════════════════════════════════
# STEP 7: Jetson Host Setup (Optional — only on Jetson hardware)
# ══════════════════════════════════════════════════════════════════════════════
if [ "$IS_JETSON" = "yes" ]; then
    section "Step 7: Jetson Host Optimization"

    info "This is a Jetson. Running Jetson-specific host setup..."
    info "This requires ${YELLOW}sudo${NC} for kernel parameters, udev rules, and systemd services."
    echo ""

    read -p "Continue with Jetson optimization? (y/n) " -n 1 -r
    echo ""

    if [[ $REPLY =~ ^[Yy]$ ]]; then
        if [ -f "$JETSON_HOST_SETUP_SCRIPT" ]; then
            bash "$JETSON_HOST_SETUP_SCRIPT"
            success "Jetson host setup completed"
        else
            warn "setup_jetson_host.sh not found — skipping Jetson optimization."
            warn "You can run it manually later: ${CYAN}./setup_jetson_host.sh${NC}"
        fi
    else
        warn "Jetson setup skipped. Run manually later: ${CYAN}./setup_jetson_host.sh${NC}"
    fi
else
    info "(Not a Jetson — skipping Jetson optimizations)"
fi

# ══════════════════════════════════════════════════════════════════════════════
# STEP 8: Build Docker Image
# ══════════════════════════════════════════════════════════════════════════════
section "Step 8: Build Docker Image"

info "Building Docker image for ${CYAN}$SERVICE${NC} service..."
info "(This may take 2–5 minutes on first run)"
if docker compose --env-file "$REPO_DIR/.env" build "$SERVICE"; then
    success "Docker image built successfully"
else
    error "Docker build failed. Check Docker installation and disk space."
fi

# ══════════════════════════════════════════════════════════════════════════════
# STEP 9: Launch Docker Container
# ══════════════════════════════════════════════════════════════════════════════
section "Step 9: Launch Container"

info "Launching container..."
if docker compose --env-file "$REPO_DIR/.env" --profile "$SERVICE" up -d; then
    success "Container launched successfully"
    info "Container name: ${CYAN}f1tenth_${SERVICE}${NC}"
else
    error "Failed to launch container"
fi

# ══════════════════════════════════════════════════════════════════════════════
# DONE
# ══════════════════════════════════════════════════════════════════════════════
section "Setup Complete! 🎉"

echo -e "${GREEN}Your F110th environment is ready.${NC}"
echo -e "To get started, attach to the container: ${CYAN}make docker_attach${NC}"
echo ""
echo "Available make targets:"
make -C "${REPO_DIR}" help
echo ""

if [ "$IS_JETSON" = "yes" ]; then
    echo "Jetson-specific monitor:"
    echo -e "     ${CYAN}jtop${NC}            — Monitor system performance"
    echo ""
    if grep -q "usbcore.autosuspend" /boot/extlinux/extlinux.conf 2>/dev/null; then
        info "USB optimizations already applied (no reboot needed)"
    else
        warn "Some changes require a reboot to take effect:"
        warn "  • Kernel parameters (USB autosuspend, UDP buffers)"
        warn "  • Security limits (real-time scheduling)"
        warn ""
        warn "When ready: ${CYAN}sudo reboot${NC}"
    fi
fi

echo ""
echo -e "${GREEN}Happy racing! 🏎️${NC}"
echo ""
