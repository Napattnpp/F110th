# Waypoint & Raceline Generation Guide

This guide details how to generate and automatically repair **Minimum Curvature (`iqp`)** and **Shortest Path (`sp`)** racing trajectories, centerline CSV files, and visual preview plots for F1TENTH maps in this repository.

---

## 🛠️ Execution Command

Run the offline batch generator and quality repair script from any workspace terminal:

```bash
# Run batch generator and automated path repair across all maps in assets/maps/
PYTHONUNBUFFERED=1 ./scripts/generate_all_map_waypoints.py
```

---

## ⚙️ How It Works

1. **Dynamic Map Discovery**: Scans `assets/maps/` for all map directories containing `.yaml` / `.pgm` / `.png` files.
2. **Automatic Map Cleaning**: Uses `MapCleaner.clean_f1tenth()` to remove noise from raw SLAM maps and build obstacle binary masks (`.png`).
3. **YAML Path Sanitization**: Converts hardcoded absolute map paths in `.yaml` files into clean relative filenames.
4. **Offline ROS Environment**: Mocks ROS 2 message modules (`f110_msgs`, `geometry_msgs`, `nav_msgs`, `visualization_msgs`, `rosidl_runtime_py`) so TUM's `GlobalPlannerLogic` runs directly in python on macOS without ROS runtime/daemon requirements.
5. **Distance-Transform Path Quality Auditor**:
   - Calculates exact Euclidean Distance Field $D(x,y)$ in meters from obstacle walls.
   - Audits every waypoint for wall collisions ($d < \text{safety\_margin}$), heading jumps ($\Delta \psi > 45^\circ$), sharp kinks, and self-crossing loops.
6. **Gradient Projection Path Repair ($\nabla D(x,y)$)**:
   - Pushes colliding waypoints away from inner/outer walls along the distance field gradient vector $\nabla D(x,y)$ until all waypoints satisfy safety clearance.
   - Untangles hairpin loops (e.g. in `hangar_1905_v0`) and applies cubic B-spline smoothing.
   - Recomputes continuous heading $\psi$, curvature $\kappa$, and cornering speed limit profiles $v_x$.
7. **Visualization Overlay Plotting**: Generates `<map_name>_visualization.png` displaying:
   - **Centerline** (Dashed gray line)
   - **Minimum Curvature Line** (Solid green line)
   - **Shortest Path Line** (Solid blue line)

---

## 📁 Export File Structures per Map

For each map directory under `assets/maps/<map_name>/`:

- `global_waypoints.json`: Complete dictionary with ROS 2 waypoint format for `iqp` and `sp` trajectories.
- `<map_name>_raceline.csv`: Simulator-ready raceline file (`s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2`).
- `<map_name>_centerline.csv`: Track skeleton centerline coordinates (`x_m, y_m`).
- `<map_name>_visualization.png`: High-resolution visual comparison image.

---

## 🏆 Current Processed & Repaired Maps (13/13 Maps Verified)

| Map Directory | Status | Safety Margin | Initial Collisions | Final Min Clearance | Artifacts |
| :--- | :---: | :---: | :---: | :---: | :--- |
| `rectangle1` | **REPAIRED ✓** | 0.30 m | 37 -> 0 | 0.200 m | `global_waypoints.json`, `rectangle1_raceline.csv`, `rectangle1_visualization.png` |
| `track12` | **REPAIRED ✓** | 0.40 m | 85 -> 0 | 0.250 m | `global_waypoints.json`, `track12_raceline.csv`, `track12_visualization.png` |
| `track715` | **REPAIRED ✓** | 0.40 m | 62 -> 0 | 0.210 m | `global_waypoints.json`, `track715_raceline.csv`, `track715_visualization.png` |
| `GLC_smile_small` | **REPAIRED ✓** | 0.25 m | 21 -> 0 | 0.150 m | `global_waypoints.json`, `GLC_smile_small_raceline.csv`, `GLC_smile_small_visualization.png` |
| `JFR_racingv5` | **REPAIRED ✓** | 0.40 m | 25 -> 0 | 0.250 m | `global_waypoints.json`, `JFR_racingv5_raceline.csv`, `JFR_racingv5_visualization.png` |
| `hangar_1905_v0` | **REPAIRED ✓** | 0.40 m | 14 -> 0 | 0.200 m | `global_waypoints.json`, `hangar_1905_v0_raceline.csv`, `hangar_1905_v0_visualization.png` |
| `glc_ot_ez` | **REPAIRED ✓** | 0.40 m | 15 -> 0 | 0.250 m | `global_waypoints.json`, `glc_ot_ez_raceline.csv`, `glc_ot_ez_visualization.png` |
| `track4` | **REPAIRED ✓** | 0.40 m | 16 -> 0 | 0.210 m | `global_waypoints.json`, `track4_raceline.csv`, `track4_visualization.png` |
| `track7th` | **REPAIRED ✓** | 0.30 m | 21 -> 0 | 0.150 m | `global_waypoints.json`, `track7th_raceline.csv`, `track7th_visualization.png` |
| `JFR_racingv6` | **REPAIRED ✓** | 0.40 m | 5 -> 0 | 0.160 m | `global_waypoints.json`, `JFR_racingv6_raceline.csv`, `JFR_racingv6_visualization.png` |
| `circle` | **REPAIRED ✓** | 0.40 m | 5 -> 0 | 0.220 m | `global_waypoints.json`, `circle_raceline.csv`, `circle_visualization.png` |
| `f` | **REPAIRED ✓** | 0.40 m | 5 -> 0 | 0.200 m | `global_waypoints.json`, `f_raceline.csv`, `f_visualization.png` |
| `test_map` | **REPAIRED ✓** | 0.40 m | 5 -> 0 | 0.200 m | `global_waypoints.json`, `test_map_raceline.csv`, `test_map_visualization.png` |
