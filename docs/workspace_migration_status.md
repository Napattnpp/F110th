# F1Tenth Workspace Migration & Status Report

> **Session Metadata**
> * **Pair Programming Partner**: Antigravity (Google DeepMind)
> * **Conversation ID**: `05ffe5fe-3290-4444-9b99-b3891b43db5c`
> * **Date/Time**: Sunday, June 28, 2026, 05:44 AM (GMT+7)
> * **Local Logs Path**: `/Users/napat/.gemini/antigravity/brain/05ffe5fe-3290-4444-9b99-b3891b43db5c/.system_generated/logs/transcript.jsonl`

This document tracks the migration progress, resolved issues, current architecture status, and future steps for the modernized **`F110th`** workspace.

---

## 1. Work Completed & Fixed

### 🛠️ Runtime & Dependency Fixes (Docker)
* **CycloneDDS Shared Memory Patch**: Mounted the host `cyclonedds.xml` file into the container (`docker-compose.yml`) with shared memory disabled. This permanently prevents DDS hanging/crashes when the RouDi daemon is not active on host developer platforms.
* **NumPy 2.x Conflict Resolution**: Pinned NumPy to version `1.26.4` inside the Docker image/container to prevent binary C-API mismatches and import crashes with older system-compiled packages (like `matplotlib`).
* **Headless Matplotlib Rendering**: Patched `global_planner` nodes (`global_planner_utils.py` and `global_planner_logic.py`) to detect and use the headless `Agg` backend when `MPLBACKEND=Agg` is set, preventing GUI crashes in headless container environments.
* **System & Python Packages Added**: Baked key ROS and Python dependencies into `Dockerfile.general` and `Dockerfile.jetson` so they build automatically next time:
  * `gfortran` (required to compile quadratic programming solvers on Jetson ARM/aarch64).
  * `ros-humble-tf-transformations` (geometry helper libraries).
  * `scikit-image` (segmentation/watershed helpers).
  * `trajectory-planning-helpers` (FTM optimization library).
  * `casadi` (numerical optimization framework).
  * `scikit-learn` (machine learning/regression estimators).

### 🚀 Launch File & Map Resolvers
* **Lifecycle Node Collisions**: Moved the AMCL localization group inside the `physical_actions` group in `base_launch.py`. When `sim:=True` is run, duplicate map servers and lifecycle managers no longer conflict.
* **YAML Map Path Resolver**: Fixed `mapping_launch.py` to pass the parent map directory instead of the `.yaml` file path to the trajectory publisher, preventing `NotADirectoryError` crashes.
* **Map Configuration Restored**: Restored the default simulation track files for `levine` and generated standard configuration tunings (`speed_scaling.yaml`, `ot_sectors.yaml`) in the workspace assets.

### 📦 Package Integration
* **Perception Package Sync**: Successfully copied the LiDAR opponent-tracking (`perception`) package from `race_stack` into `F110th/src/perception` and verified that the entire workspace builds error-free.

---

## 2. Current Architecture Mapping

The `F110th` workspace is a complete, modernized port of the `race_stack`. Here is where all essential components reside:

| Component | `race_stack` Path | `F110th` Path | Description |
| :--- | :--- | :--- | :--- |
| **Bringup Orchestration** | `stack_master` | `src/race_bringup` | Master orchestrator launch scripts. |
| **Gym Simulator** | `base_system/f110_simulator` | `src/f1tenth_gym_ros` | ROS 2 simulation bridge. |
| **Steering LUT** | `system_identification/steering_lookup` | `src/control/steering_lookup` | Pacejka-model lookup tables. |
| **LiDAR tracking** | `perception` | `src/perception` | Target detection/Kalman filters. |
| **Controller Nodes** | `controller` | `src/control/controller` | Speed/steering controller. |
| **Planning Logic** | `planner` | `src/planning` | Spline path planners. |

---

## 3. Future Steps & Checklist

- [ ] **Foxglove Gamepad Calibration**: Connect your physical gamepad to your MacBook and map the axes inside Foxglove Studio's Teleop panel to publish to `/cmd_vel` to test steering/throttle manually.
- [ ] **First Track Mapping**: Run the mapping launch file in simulation to generate and save your own track centerline/raceline:
  ```bash
  MPLBACKEND=Agg ros2 launch race_bringup mapping_launch.py sim:=True map_name:=hangar_1905_v0
  ```
- [ ] **Deploying to Physical Robot**:
  * Copy/clone this branch to your physical Jetson board.
  * Run the host setup script (`configs/setup_jetson_host.sh`) on the Jetson to configure kernel limits, UDP buffers, and host aliases.
  * Build the Jetson Docker container:
    ```bash
    docker compose build jetson
    ```
