# F110th

F1TENTH racing stack. Runs entirely inside Docker.

> **TL;DR:** Edit [`f110th.config`](./f110th.config), run `./setup.sh`, use `make` commands.

| Resource | Description |
|----------|-------------|
| [`f110th.config`](./f110th.config) | ✏️ **Edit this** — your configuration |
| [`setup.sh`](./setup.sh) | ▶️ **Run this** — one-time setup |
| [`Makefile`](./Makefile) | 🔧 Day-to-day commands (`make help`) |
| [`docs/QUICK_REF.md`](./docs/QUICK_REF.md) | 📋 Cheat-sheet card |
| [`docs/MAINTAINER.md`](./docs/MAINTAINER.md) | 🛠️ Developer / maintainer guide |

---

## Quick Start

### 1. Prerequisites

- [Docker](https://docs.docker.com/get-docker/) installed and running
- On **Jetson**: also run the Jetson host setup first (see below)

### 2. Clone the repo

```bash
git clone -b humble-dev https://github.com/Napattnpp/F110th.git ~/F110th
cd ~/F110th
```

### 3. Configure

Edit [`f110th.config`](./f110th.config) to match your setup:

```bash
# Key settings to change:
ROS_DOMAIN_ID=22          # Must match all machines in the same session
RACECAR_VERSION=NUC1      # NUC1, NUC2, JET1, ...
DOCKER_PROFILE=general    # 'general' (laptop/x86) or 'jetson' (on-car)
```

### 4. Run setup

```bash
./setup.sh
```

This will automatically:
- Detect your platform (x86 / Apple Silicon / Jetson)
- Build the Docker image
- Launch the container

> **Note:** This project runs entirely inside Docker. Native ROS 2 installation on the host is not required and not supported by this script.

---

## Day-to-Day Commands

```bash
make launch     # Start the container
make attach     # Get a shell inside the container
make stop       # Stop the container
make build      # Rebuild the image (after Dockerfile changes)
make push       # Push image to DockerHub
```

Or manually:
```bash
docker compose --profile general up -d       # laptop
docker compose --profile jetson up -d        # Jetson
docker compose exec general /bin/bash        # attach
```

---

## VSCode Dev Container

Open the repo in VSCode and click **"Reopen in Container"**.

- **Laptop / x86**: uses `.devcontainer/devcontainer.json` → `general` service
- **Jetson**: uses `.devcontainer/jetson/devcontainer.json` → `jetson` service

Extensions (ROS, C++, Python, clang-format, etc.) are installed automatically.

---

## Jetson Host Setup

Run once on a new Jetson to configure the host OS (udev rules, UDP buffers, clocks, aliases):

```bash
cd ~/F110th && ./setup_jetson_host.sh
```

Then reboot, and run `./setup.sh` as normal.

---

## Inside the Container — Common Commands

### Build the workspace

```bash
colcon build --build-base out/build --install-base out/install
source out/install/setup.bash
```

### Launch the full stack

```bash
# Real car
ros2 launch stack_master base_system_launch.xml map_name:=<map> racecar_version:=<NUCx>

# Simulation
ros2 launch stack_master base_system_launch.xml sim:=true
```

### Mapping

```bash
ros2 launch stack_master mapping_launch.xml map_name:=<map_name>
```

### Time trials

```bash
ros2 launch stack_master time_trials_launch.xml
```

### micro-ROS agent (general image only)

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/sensors/micro_ros
```

### Foxglove bridge

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

### SLAM

```bash
# Create map
ros2 launch slam_toolbox online_async_launch.py \
  params_file:=/home/$USER/F110th/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml

# Save map
ros2 run nav2_map_server map_saver_cli -f <map_name>
```

### Algorithms

```bash
ros2 run safety_node safety_node_cpp           # AEB
ros2 run wall_follow wall_follow_node_cpp      # Wall following
ros2 run gap_follow reactive_node_cpp          # Follow the gap
ros2 launch pure_pursuit sim_pure_pursuit_launch.py  # Pure pursuit
```

### Recording

```bash
ros2 bag record -s mcap -o <file_name> -a     # MCAP format
ros2 bag record -o <file_name> -a              # .db3 format
```

---

## Project Structure

```
F110th/
├── f110th.config          ← Edit this to configure your setup
├── setup.sh               ← Run this once to set everything up
├── Makefile               ← Day-to-day Docker commands
├── docker-compose.yml
├── requirements.txt       ← Host-side Python tools (.venv)
│
├── docker/                ← Docker build files
│   ├── Dockerfile.general
│   ├── Dockerfile.jetson
│   ├── cyclonedds.xml
│   └── ros_entrypoint.sh
│
├── .devcontainer/         ← VSCode Dev Container configs
│   ├── devcontainer.json          (general)
│   └── jetson/devcontainer.json   (Jetson)
│
├── src/                   ← ROS 2 packages
├── configs/               ← udev rules, system configs
└── out/          ← colcon build output (gitignored)
```
