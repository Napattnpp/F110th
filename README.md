# F110th

F1TENTH racing stack running entirely inside Docker.

> **TL;DR:** Clone the repo and run `make setup` for an interactive setup wizard.

| Resource | Description |
|----------|-------------|
| [`.env`](./.env) | ✏️ **Local configuration** (auto-configured via `make setup` wizard or copied from [`configs/.env.example`](./configs/.env.example)) |
| [`Makefile`](./Makefile) | 🔧 Workflows & container controls (`make setup`, `make attach`, `make help`) |
| [`docs/QUICK_REF.md`](./docs/QUICK_REF.md) | 📋 Quick reference guide |
| [`docs/MAINTAINER.md`](./docs/MAINTAINER.md) | 🛠️ Maintainer & architecture guide |

---

## Quick Start

### 1. Prerequisites

- [Docker](https://docs.docker.com/get-docker/) installed and running
- On **NVIDIA Jetson**: sudo access to apply hardware performance optimizations

### 2. Run Interactive Setup

```bash
git clone -b humble-dev https://github.com/Napattnpp/F110th.git ~/F110th
cd ~/F110th
make setup
```

`make setup` will launch an **Interactive Setup Wizard** in your terminal:
- Interactively configures your **`F110_ROLE`** (`vehicle` \| `simulation`)
- Prompts for your **`ROS_DOMAIN_ID`** (0–101) to prevent team network cross-talk
- Prompts for your target **`RACECAR_ID`** (e.g. `car01`)
- Auto-detects your platform architecture (x86 / Mac / Jetson Tegra)
- Builds and launches the Docker container stack

---

## Day-to-Day Commands

```bash
make up        # Launch the container stack in detached mode
make attach     # Get an interactive bash shell inside the container
make stop      # Stop the container stack
make build     # Rebuild the Docker image
make push      # Build and push image to DockerHub
make clean     # Clean colcon build output directory (out/)
make prune     # Full reset (stop containers, remove volumes and .env)
```

---

## Jetson Host Optimization

When running on an NVIDIA Jetson, the host script automatically configures:
- NVPModel MAXN power profile (`nvpmodel -m 0`)
- Maximum clock frequencies (`jetson_clocks` persistent systemd service)
- 2GB kernel UDP socket buffer ceiling (`/etc/sysctl.d/99-f1tenth-network.conf`)
- Real-time priority & memory locking (`/etc/security/limits.d/99-f1tenth-realtime.conf`)
- USB autosuspend disable & 1024MB USB memory limit (`extlinux.conf`)
- CPU governor set to `performance`
- Wi-Fi latency tuning (NetworkManager powersave off, Intel `iwlwifi` & Realtek `rtw88` driver performance mode)
- Serial device symlinks for VESC, micro-ROS MKR Zero, and RPLidar S1 (`/etc/udev/rules.d/99-f1tenth-serial.rules`)

---

## Inside the Container — Common Commands

### Build the workspace

```bash
colcon build --base-paths src --build-base out/build --install-base out/install --symlink-install
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

### Algorithms

```bash
ros2 run safety_node safety_node_cpp           # AEB
ros2 run wall_follow wall_follow_node_cpp      # Wall following
ros2 run gap_follow reactive_node_cpp          # Follow the gap
ros2 launch pure_pursuit sim_pure_pursuit_launch.py  # Pure pursuit
```

---

## Project Structure

```
F110th/
├── .env                   ← Active local configuration (from configs/.env.example)
├── Makefile               ← Task & container runner (`make setup`, `make attach`, `make up`)
├── docker-compose.yml     ← Container stack definition
├── configs/               ← Passive project configurations (declarative settings only)
│   ├── .env.example       ← Configuration template
│   ├── cyclonedds.xml     ← Single Source of Truth DDS configuration
│   ├── tmux.conf          ← Tmux dotfile configuration
│   ├── requirements.txt   ← Host developer python packages
│   └── jetson/            ← Versioned Jetson host performance files
│       ├── 99-f1tenth-serial.rules
│       ├── 99-f1tenth-network.conf
│       ├── 20-f1tenth-wifi-powersave.conf
│       ├── iwlwifi.conf
│       ├── rtw88.conf
│       ├── 99-f1tenth-realtime.conf
│       └── f1tenth-jetson-clocks.service
├── scripts/
│   ├── setup/             ← Setup automation (bootstrap.sh, configure.sh)
│   ├── host/              ← Host OS installer (setup-jetson.sh)
│   ├── ops/               ← Vehicle operations & connectivity (connect_car.sh)
│   ├── waypoints/         ← Waypoint tools & visualizers
│   └── tuning_and_benchmarks/ ← Optimization & profiling benchmarks
└── src/                   ← ROS 2 nodes & packages
```
