# ==============================================================================
#  F110th — Makefile
#  Usage: make [target]
# ==============================================================================

ORIGINAL_XAUTHORITY := $(XAUTHORITY)

-include .env
export

.PHONY: help setup bootstrap configure build up attach stop clean push prune tuner optimize start_line benchmark monitor profile init_map finish_map clean_map clean_web map_status map_pipeline

PYTHON := $(if $(wildcard .venv/bin/python),.venv/bin/python,python3)

HOST_OS := $(shell uname)
ifeq ($(HOST_OS),Darwin)
    DISPLAY := host.docker.internal:0
    XAUTHORITY_ENV :=
    LIBGL_ENV := 1
else
    XAUTHORITY_ENV := $(XAUTHORITY)
    LIBGL_ENV :=
endif

ifdef DOCKER_PROFILE
    SERVICE := $(DOCKER_PROFILE)
else
    ifeq ($(wildcard /etc/nv_tegra_release),)
        SERVICE := general
    else
        SERVICE := jetson
    endif
endif

help: ## Show available Makefile targets
	@echo ""
	@echo "  \033[1;36m🏁 F1TENTH Autonomous Racing Stack\033[0m"
	@echo "  Usage: \033[33mmake\033[0m \033[32m<target>\033[0m"
	@echo ""
	@awk 'BEGIN {FS = ":.*##"} \
		/^[a-zA-Z0-9_-]+:.*?##/ { printf "    \033[32m%-18s\033[0m %s\n", $$1, $$2 } \
		/^##@/ { printf "\n  \033[1;36m%s\033[0m\n", substr($$0, 5) }' Makefile
	@echo ""

##@ 💻 Host Setup & Physical Diagnostics
setup: ## First-time setup: host optimization, bootstrap, and container stack (Docker is last)
	@bash scripts/setup/setup.sh

host_setup: ## Configure host performance, udev rules, network, and groups (No Docker required)
	@if [ -f /etc/nv_tegra_release ]; then \
		bash scripts/setup/setup-jetson.sh; \
	elif [ "$$(uname)" = "Linux" ]; then \
		bash scripts/setup/setup-network-host.sh; \
	else \
		echo "Host is macOS. No kernel tuning needed."; \
	fi

hardware_check: ## Run non-destructive audit of VESC, LiDAR, joystick, clocks, and buffers
	@echo "=== Physical Hardware & Interface Audit ==="
	@[ -e /dev/sensors/vesc ] && echo "  \033[32m[✓]\033[0m VESC Controller: Found (/dev/sensors/vesc)" || echo "  \033[33m[!]\033[0m VESC Controller: Not detected at /dev/sensors/vesc"
	@ls /dev/input/js* >/dev/null 2>&1 && echo "  \033[32m[✓]\033[0m Gamepad / Joystick: Found" || echo "  \033[33m[!]\033[0m Gamepad / Joystick: None connected (/dev/input/js*)"
	@ping -c 1 -W 1 192.168.0.10 >/dev/null 2>&1 && echo "  \033[32m[✓]\033[0m Hokuyo LiDAR: Reachable at 192.168.0.10" || echo "  \033[33m[!]\033[0m Hokuyo LiDAR: Not pingable at 192.168.0.10"
	@sysctl net.core.rmem_max 2>/dev/null | grep -q "2147483647" && echo "  \033[32m[✓]\033[0m Kernel UDP Buffers: Locked to 2GB" || echo "  \033[33m[!]\033[0m Kernel UDP Buffers: Below 2GB"

bootstrap: ## Validate system dependencies and initialize .env and python virtual environment
	@bash scripts/setup/bootstrap.sh

configure: ## Resolve platform architecture, user IDs, and dynamic docker profiles
	@bash scripts/setup/configure.sh

##@ 🐳 Docker & Container Management
build: ## Build Docker container image for active profile (make build SERVICE=jetson)
	@echo "Building service: $(SERVICE)"
	docker compose build $(SERVICE)

up: ## Launch container stack in background
	@echo "Launching service: $(SERVICE)"
	docker compose --profile $(SERVICE) up -d
	@echo "Container active. Attach with: make attach"

attach: ## Attach interactive bash session inside active container
	@if [ "$(HOST_OS)" != "Darwin" ] && [ -n "$$DISPLAY" ]; then \
		if [ -d /tmp/.docker.xauth ]; then rm -rf /tmp/.docker.xauth; fi; \
		touch /tmp/.docker.xauth 2>/dev/null || true; \
		if command -v xauth >/dev/null 2>&1; then \
			if [ -n "$(ORIGINAL_XAUTHORITY)" ]; then \
				XAUTHORITY="$(ORIGINAL_XAUTHORITY)" xauth nlist "$$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge - >/dev/null 2>&1 || true; \
			else \
				env -u XAUTHORITY xauth nlist "$$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge - >/dev/null 2>&1 || true; \
			fi; \
		fi; \
		chmod a+r /tmp/.docker.xauth 2>/dev/null || true; \
	fi
	docker compose exec -e DISPLAY=$(DISPLAY) -e XAUTHORITY="$(XAUTHORITY_ENV)" -e LIBGL_ALWAYS_INDIRECT="$(LIBGL_ENV)" -u $(USER) $(SERVICE) /bin/bash

stop: ## Stop active container stack
	docker compose --profile $(SERVICE) stop
	@echo "Container stack stopped."

push: ## Build and push image to DockerHub
	@[ -n "$(DOCKERHUB_USER)" ] && [ "$(DOCKERHUB_USER)" != "your_dockerhub_username" ] || (echo "[!] Error: DOCKERHUB_USER is not configured in .env (still set to placeholder 'your_dockerhub_username')"; exit 1)
	@[ -n "$(IMAGE_NAME)"    ] || (echo "[!] Error: IMAGE_NAME not set in .env"; exit 1)
	docker compose build $(SERVICE)
	docker push $(DOCKERHUB_USER)/$(IMAGE_NAME):humble-$(SERVICE)

prune: stop ## Stop containers, remove volumes, and clean generated build outputs
	docker compose --profile $(SERVICE) down --volumes --remove-orphans
	rm -rf out/ .env
	@echo "Workspace fully pruned."

##@ 🔨 Workspace Build & Clean
colcon_build: ## Build workspace natively into out/
	@echo "Building with colcon to out/ ..."
	@mkdir -p out/build out/install out/logs
	@bash -c "source /opt/ros/humble/setup.bash && PYTHONWARNINGS='ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::setuptools.command.develop' colcon --log-base out/logs build --base-paths src --build-base out/build --install-base out/install --symlink-install"

clean: ## Remove colcon build outputs (out/)
	@echo "Removing out/ build contents..."
	rm -rf out/build out/install out/logs
	@echo "Colcon build output removed."

##@ 🗺️ Map & Raceline Pipeline (Zero X11)
map_status: ## Inspect ADR-004 map compliance across tracks: make map_status [MAP=rectangle1]
	@$(PYTHON) scripts/map/manage_map.py status $(MAP)

init_map: ## Initialize 4-subfolder architecture for a map: make init_map MAP=rectangle1
	@[ -n "$(MAP)" ] || (echo "[!] Error: MAP argument required (e.g. make init_map MAP=rectangle1)"; exit 1)
	@echo "Standardizing Map structure using $(PYTHON)..."
	@$(PYTHON) scripts/map/init_map.py $(MAP)

clean_map: ## Clean occupancy grid & generate binary mask (Headless): make clean_map MAP=rectangle1
	@[ -n "$(MAP)" ] || (echo "[!] Error: MAP argument required (e.g. make clean_map MAP=rectangle1)"; exit 1)
	@$(PYTHON) scripts/map/manage_map.py clean $(MAP)

clean_web: ## Launch Web Map Cleaner (Interactive Studio / Zero X11): make clean_web [MAP=rectangle1] [PORT=8051]
	@$(PYTHON) scripts/map/map_cleaner_web.py $(if $(MAP),--map $(MAP),) --port $(or $(PORT),8051)

optimize: ## Run offline waypoint optimizer on host: make optimize [MAP=rectangle1]
	@echo "Running Waypoint Optimizer using $(PYTHON)..."
	@$(PYTHON) src/planning/global_planner/global_planner/generate_waypoints.py $(if $(MAP),-m $(MAP),)

start_line: ## Set Start/Finish Line & AMCL initial pose on host: make start_line MAP=rectangle1
	@[ -n "$(MAP)" ] || (echo "[!] Error: MAP argument required (e.g. make start_line MAP=rectangle1)"; exit 1)
	@echo "Launching Start Line selector using $(PYTHON)..."
	@$(PYTHON) scripts/map/set_start_line.py $(MAP)

map_pipeline: ## End-to-end automated map build (init -> clean -> optimize): make map_pipeline MAP=rectangle1
	@[ -n "$(MAP)" ] || (echo "[!] Error: MAP argument required (e.g. make map_pipeline MAP=rectangle1)"; exit 1)
	@$(PYTHON) scripts/map/manage_map.py pipeline $(MAP)

finish_map: ## Save SLAM map and organize into 4-subfolder assets: make finish_map MAP=rectangle1
	@[ -n "$(MAP)" ] || (echo "[!] Error: MAP argument required (e.g. make finish_map MAP=rectangle1)"; exit 1)
	@echo "Finishing SLAM map and organizing assets..."
	@bash scripts/map/finish_map.sh $(MAP)

##@ 🏎️ Race Operations & Telemetry
profile: ## View or switch active racecar profile: make profile [CAR=ORIN1|SIM]
	@if [ -n "$(CAR)" ]; then \
		sed -i '' 's/^RACECAR_VERSION=.*/RACECAR_VERSION=$(CAR)/' .env 2>/dev/null || sed -i 's/^RACECAR_VERSION=.*/RACECAR_VERSION=$(CAR)/' .env; \
		echo "Switched active racecar profile to: $(CAR)"; \
	else \
		echo "Current active profile in .env: $$(grep '^RACECAR_VERSION=' .env | cut -d= -f2)"; \
		echo "Available hardware profiles in src/race_bringup/config/:"; \
		for dir in src/race_bringup/config/*/; do \
			base=$$(basename "$$dir"); \
			if [ "$$base" != "global_planner" ]; then \
				echo "  - $$base"; \
			fi; \
		done; \
	fi

tuner: ## Launch Web Sector Tuner (speed & overtaking sectors): make tuner [PORT=8050]
	@echo "Launching Web Sector Tuner using $(PYTHON)..."
	@$(PYTHON) scripts/race/sector_tuner_web.py --port $(or $(PORT),8050)

monitor: ## Open Foxglove Studio / Web connected to live vehicle bridge (ws://localhost:8765)
	@echo "Connecting to Foxglove WebSocket on ws://localhost:8765..."
	@python3 -c "import webbrowser, subprocess; (subprocess.run(['open', 'foxglove://open?ds=foxglove-websocket&ds.url=ws://localhost:8765'], capture_output=True).returncode == 0) or webbrowser.open('https://app.foxglove.dev/?ds=foxglove-websocket&ds.url=ws://localhost:8765')"

benchmark: ## Run raceline benchmark comparison across tracks
	@echo "Running Benchmarks using $(PYTHON)..."
	@$(PYTHON) scripts/benchmarks/benchmark_optimizers.py
