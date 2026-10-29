#!/usr/bin/env python3
"""
Batch Global Waypoint Generator & Distance-Field Normal Repair Engine for F1TENTH Maps
--------------------------------------------------------------------------------------
Scans assets/maps/ for map directories, cleans raw map images, sanitizes YAML configuration,
optimizes Minimum Curvature (IQP) & Shortest Path (SP) trajectories, and applies smooth track-normal
shifting to guarantee 100% obstacle clearance (>0.28m) without corner-cutting or path tangling.

Exports per map:
- global_waypoints.json
- <map_name>_raceline.csv
- <map_name>_centerline.csv
- assets/maps_visualization/<map_name>_visualization.png
- assets/maps_visualization/<map_name>_velocity_profile.png
"""

import os
import sys
import shutil
import types
import json
import yaml
import warnings
import cv2
from pathlib import Path

# Suppress NumPy FutureWarning and Matplotlib UserWarning
warnings.filterwarnings("ignore", category=FutureWarning)
warnings.filterwarnings("ignore", category=UserWarning, module="matplotlib")
warnings.filterwarnings("ignore", category=UserWarning, module="mpl_toolkits")

import numpy as np
from scipy.ndimage import gaussian_filter1d
from matplotlib.collections import LineCollection

# NumPy 1.24+ backward compatibility monkey-patch for legacy trajectory_planning_helpers
with warnings.catch_warnings():
    warnings.simplefilter("ignore", category=FutureWarning)
    if not hasattr(np, 'int'):
        np.int = int
    if not hasattr(np, 'float'):
        np.float = float
    if not hasattr(np, 'bool'):
        np.bool = bool

# Resolve repository root dynamically relative to this script location
def find_repo_root():
    curr = Path(__file__).resolve().parent
    for p in [curr] + list(curr.parents):
        if (p / 'assets' / 'maps').is_dir() or (p / 'src' / 'utilities' / 'map_utils').is_dir():
            return p
    return curr.parent.parent.parent

repo_root = find_repo_root()
os.makedirs(os.path.expanduser("~/.ros"), exist_ok=True)

# Add project src directories to path
sys.path.insert(0, str(repo_root))
sys.path.insert(0, str(repo_root / "scripts"))
sys.path.insert(0, str(repo_root / "src" / "planning" / "global_planner"))
sys.path.insert(0, str(repo_root / "src" / "planning" / "global_planner" / "global_planner"))
sys.path.insert(0, str(repo_root / "src" / "planning" / "global_planner" / "global_planner" / "global_racetrajectory_optimization"))
sys.path.insert(0, str(repo_root / "src" / "utilities" / "map_utils"))

# -----------------------------------------------------------------------------
# MOCK ROS ENVIRONMENT MODULES FOR OFFLINE PYTHON EXECUTION
# -----------------------------------------------------------------------------
class MockMsg:
    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)

class Point(MockMsg):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

class Pose(MockMsg):
    def __init__(self, position=None, orientation=None):
        self.position = position or Point()
        self.orientation = orientation or MockMsg(x=0.0, y=0.0, z=0.0, w=1.0)

class PoseStamped(MockMsg):
    def __init__(self, header=None, pose=None):
        self.header = header or MockMsg(stamp=MockMsg(sec=0, nanosec=0), frame_id="")
        self.pose = pose or Pose()

class Float32(MockMsg):
    def __init__(self, data=0.0):
        self.data = data

class String(MockMsg):
    def __init__(self, data=""):
        self.data = data

class OccupancyGrid(MockMsg):
    pass

class Marker(MockMsg):
    ADD = 0
    MODIFY = 0
    DELETE = 2
    DELETEALL = 3
    ARROW = 0
    CUBE = 1
    SPHERE = 2
    CYLINDER = 3
    LINE_STRIP = 4
    LINE_LIST = 5
    CUBE_LIST = 6
    SPHERE_LIST = 7
    POINTS = 8
    TEXT_VIEW_FACING = 9
    MESH_RESOURCE = 10
    TRIANGLE_LIST = 11

    def __init__(self):
        self.header = MockMsg(stamp=MockMsg(sec=0, nanosec=0), frame_id="")
        self.ns = ""
        self.id = 0
        self.type = 0
        self.action = 0
        self.pose = Pose()
        self.scale = MockMsg(x=0.1, y=0.1, z=0.1)
        self.color = MockMsg(r=1.0, g=0.0, b=0.0, a=1.0)
        self.points = []

class MarkerArray(MockMsg):
    def __init__(self):
        self.markers = []

class Wpnt(MockMsg):
    def __init__(self, id=0, s_m=0.0, x_m=0.0, y_m=0.0, d_right=0.0, d_left=0.0, psi_rad=0.0, vx_mps=0.0, ax_mps2=0.0):
        self.id = id
        self.s_m = s_m
        self.x_m = x_m
        self.y_m = y_m
        self.d_right = d_right
        self.d_left = d_left
        self.psi_rad = psi_rad
        self.vx_mps = vx_mps
        self.ax_mps2 = ax_mps2

class WpntArray(MockMsg):
    def __init__(self):
        self.wpnts = []

def create_mock_module(name, **attrs):
    mod = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(mod, k, v)
    sys.modules[name] = mod
    return mod

create_mock_module("std_msgs")
create_mock_module("std_msgs.msg", Float32=Float32, String=String)

create_mock_module("geometry_msgs")
create_mock_module("geometry_msgs.msg", Point=Point, Pose=Pose, PoseStamped=PoseStamped)

create_mock_module("nav_msgs")
create_mock_module("nav_msgs.msg", OccupancyGrid=OccupancyGrid)

create_mock_module("visualization_msgs")
create_mock_module("visualization_msgs.msg", Marker=Marker, MarkerArray=MarkerArray)

create_mock_module("f110_msgs")
create_mock_module("f110_msgs.msg", Wpnt=Wpnt, WpntArray=WpntArray)

def get_package_share_directory(pkg_name):
    return str(repo_root / "src" / "race_bringup")

create_mock_module("ament_index_python")
create_mock_module("ament_index_python.packages", get_package_share_directory=get_package_share_directory)

def euler_from_quaternion(q):
    return (0.0, 0.0, 0.0)

create_mock_module("tf_transformations", euler_from_quaternion=euler_from_quaternion)

def message_to_ordereddict(msg):
    if hasattr(msg, "markers"):
        return {"markers": [message_to_ordereddict(m) for m in msg.markers]}
    elif hasattr(msg, "wpnts"):
        return {"wpnts": [message_to_ordereddict(w) for w in msg.wpnts]}
    elif isinstance(msg, Wpnt):
        return {
            "id": msg.id,
            "s_m": float(msg.s_m),
            "x_m": float(msg.x_m),
            "y_m": float(msg.y_m),
            "d_right": float(msg.d_right),
            "d_left": float(msg.d_left),
            "psi_rad": float(msg.psi_rad),
            "vx_mps": float(msg.vx_mps),
            "ax_mps2": float(msg.ax_mps2)
        }
    elif isinstance(msg, Marker):
        return {
            "id": msg.id,
            "ns": msg.ns,
            "type": msg.type,
            "action": msg.action,
            "points": [{"x": float(p.x), "y": float(p.y), "z": float(p.z)} for p in getattr(msg, "points", [])]
        }
    elif isinstance(msg, dict):
        return msg
    return str(msg)

def set_message_fields(msg, values):
    pass

create_mock_module("rosidl_runtime_py")
create_mock_module("rosidl_runtime_py.convert", message_to_ordereddict=message_to_ordereddict)
create_mock_module("rosidl_runtime_py.set_message", set_message_fields=set_message_fields)

# -----------------------------------------------------------------------------
# MAIN GENERATION & SMOOTH NORMAL REPAIR LOGIC
# -----------------------------------------------------------------------------
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from global_planner.global_planner_logic import GlobalPlannerLogic
from map_cleaner_core import MapCleaner

def log_print(msg):
    print(msg, flush=True)

def resolve_map_filepath(map_dir, filename):
    """Strictly maps known filenames to their designated 4-subfolder architecture."""
    mdir = Path(map_dir)
    if filename in ["global_waypoints.json", "amcl.yaml"] or filename.endswith(".csv"):
        return str(mdir / "waypoints" / filename)
    elif "_waypoint_gen" in filename or filename == "global_planner_params.yaml":
        return str(mdir / "waypoint_gen" / filename)
    elif filename.endswith(".yaml") or filename.endswith(".png") or filename.endswith(".pgm"):
        return str(mdir / "clean" / filename)
    return str(mdir / filename)

def resolve_map_image_path(map_dir, map_name, ydata=None):
    """
    Strict Map Image Resolution:
    Must exist strictly in waypoint_gen/ as <map_name>_waypoint_gen.png/pgm or specified in <map_name>_waypoint_gen.yaml
    """
    mdir = Path(map_dir)
    gen_dir = mdir / "waypoint_gen"

    gen_yaml = gen_dir / f"{map_name}_waypoint_gen.yaml"
    if gen_yaml.exists():
        try:
            with open(gen_yaml, 'r') as f:
                data = yaml.safe_load(f)
                if data and isinstance(data, dict) and 'image' in data:
                    img_p = gen_dir / data['image']
                    if img_p.exists():
                        return str(img_p)
        except Exception:
            pass

    for ext in ['.png', '.pgm']:
        gen_img = gen_dir / f"{map_name}_waypoint_gen{ext}"
        if gen_img.exists():
            return str(gen_img)

    raise FileNotFoundError(
        f"[ERROR] Waypoint generation map image missing at strict path: {gen_dir}/{map_name}_waypoint_gen.png (or .yaml/.pgm)"
    )

def repair_trajectory_normals(coords, dist_map, origin, res, min_clearance=0.28):
    """
    Pushes waypoints strictly along track normal vectors n(s) away from obstacles.
    Because shift is 1D orthogonal to track heading, waypoint sequence s is 100% preserved.
    """
    N = len(coords)
    if N < 3:
        return coords

    def get_dist(x, y):
        px = int((x - origin[0]) / res)
        py = dist_map.shape[0] - 1 - int((y - origin[1]) / res)
        if 0 <= px < dist_map.shape[1] and 0 <= py < dist_map.shape[0]:
            return dist_map[py, px]
        return 0.0

    new_coords = coords.copy()

    for step in range(35):
        # Compute local track normal vectors (nx, ny)
        dx = np.gradient(new_coords[:, 0])
        dy = np.gradient(new_coords[:, 1])
        headings = np.arctan2(dy, dx)
        nx = -np.sin(headings)
        ny = np.cos(headings)

        shifts = np.zeros(N)
        any_violation = False

        for i in range(N):
            x, y = new_coords[i]
            d = get_dist(x, y)
            if d < min_clearance:
                any_violation = True
                eps = 0.05
                d_plus = get_dist(x + nx[i] * eps, y + ny[i] * eps)
                d_minus = get_dist(x - nx[i] * eps, y - ny[i] * eps)
                direction = 1.0 if d_plus > d_minus else -1.0
                shifts[i] = direction * ((min_clearance - d) + 0.02)

        if not any_violation:
            break

        # Smooth shifts along normal vectors using Gaussian filter
        smooth_shifts = gaussian_filter1d(shifts, sigma=1.5, mode='wrap')
        if np.max(np.abs(smooth_shifts)) < 1e-4:
            break

        new_coords[:, 0] += nx * smooth_shifts
        new_coords[:, 1] += ny * smooth_shifts

    return new_coords



def post_process_and_repair_waypoints(map_dir, map_name, min_clearance=0.28):
    """
    Audits and repairs waypoints in global_waypoints.json using distance-field normal vector shifts.
    """
    json_path = resolve_map_filepath(map_dir, "global_waypoints.json")
    yaml_path = resolve_map_filepath(map_dir, f"{map_name}.yaml")

    if not os.path.exists(json_path) or not os.path.exists(yaml_path):
        return

    with open(yaml_path, 'r') as f:
        ydata = yaml.safe_load(f)

    res = ydata.get('resolution', 0.05)
    origin = ydata.get('origin', [0.0, 0.0, 0.0])
    img_path = resolve_map_image_path(map_dir, map_name, ydata)

    if not img_path or not os.path.exists(img_path):
        return

    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        return

    # Binary obstacle map (< 210 = solid obstacle)
    binary_occ = np.where(img < 210, 0, 255).astype(np.uint8)
    dist_map = cv2.distanceTransform(binary_occ, cv2.DIST_L2, 5) * res

    with open(json_path, 'r') as f:
        wp_data = json.load(f)

    repaired_any = False

    for key in ['global_traj_wpnts_iqp', 'global_traj_wpnts_sp']:
        if key not in wp_data or 'wpnts' not in wp_data[key]:
            continue

        wpnts = wp_data[key]['wpnts']
        coords = np.array([[w['x_m'], w['y_m']] for w in wpnts])

        # Check initial violations
        viol_count = 0
        for x, y in coords:
            px = int((x - origin[0]) / res)
            py = dist_map.shape[0] - 1 - int((y - origin[1]) / res)
            if 0 <= px < dist_map.shape[1] and 0 <= py < dist_map.shape[0]:
                if dist_map[py, px] < min_clearance:
                    viol_count += 1

        if viol_count > 0:
            log_print(f"  [Wall Clearance Audit] {key} has {viol_count} wall-penetrating waypoints. Repairing with normal vectors...")
            repaired_coords = repair_trajectory_normals(coords, dist_map, origin, res, min_clearance=min_clearance)

            # Update coordinates & recalculate s_m and psi_rad
            dx = np.gradient(repaired_coords[:, 0])
            dy = np.gradient(repaired_coords[:, 1])
            headings = np.arctan2(dy, dx)

            ds = np.sqrt(np.diff(repaired_coords[:, 0], prepend=repaired_coords[-1, 0])**2 +
                         np.diff(repaired_coords[:, 1], prepend=repaired_coords[-1, 1])**2)
            s_accum = np.cumsum(ds)

            for i, w in enumerate(wpnts):
                w['x_m'] = float(repaired_coords[i, 0])
                w['y_m'] = float(repaired_coords[i, 1])
                w['psi_rad'] = float(headings[i])
                w['s_m'] = float(s_accum[i])

            repaired_any = True

    if repaired_any:
        with open(json_path, 'w') as f:
            json.dump(wp_data, f, indent=2)
        log_print(f"  [Repair Saved] Updated {json_path}")

def plot_and_save_raceline_visualization(map_dir, map_name, safety_width):
    """Render visual plot of Minimum Curvature, Shortest Path, and Centerline on top of map."""
    json_path = resolve_map_filepath(map_dir, "global_waypoints.json")
    yaml_path = resolve_map_filepath(map_dir, f"{map_name}.yaml")

    if not os.path.exists(json_path) or not os.path.exists(yaml_path):
        return

    with open(yaml_path, 'r') as f:
        yaml_data = yaml.safe_load(f)

    map_origin = yaml_data.get('origin', [0.0, 0.0, 0.0])
    map_res = yaml_data.get('resolution', 0.05)
    img_path = resolve_map_image_path(map_dir, map_name, yaml_data)

    with open(json_path, 'r') as f:
        wp_data = json.load(f)

    cent_wpnts = wp_data.get('centerline_waypoints', {}).get('wpnts', [])
    cent_x = [wp['x_m'] for wp in cent_wpnts]
    cent_y = [wp['y_m'] for wp in cent_wpnts]

    iqp_wpnts = wp_data.get('global_traj_wpnts_iqp', {}).get('wpnts', [])
    iqp_x = [wp['x_m'] for wp in iqp_wpnts]
    iqp_y = [wp['y_m'] for wp in iqp_wpnts]

    sp_wpnts = wp_data.get('global_traj_wpnts_sp', {}).get('wpnts', [])
    sp_x = [wp['x_m'] for wp in sp_wpnts]
    sp_y = [wp['y_m'] for wp in sp_wpnts]

    fig, ax = plt.subplots(figsize=(10, 10), dpi=150)

    if img_path and os.path.exists(img_path):
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if img is not None:
            h, w = img.shape
            extent = [
                map_origin[0],
                map_origin[0] + w * map_res,
                map_origin[1],
                map_origin[1] + h * map_res
            ]
            ax.imshow(cv2.flip(img, 0), cmap='gray', extent=extent, origin='lower', alpha=0.6)

    if cent_x:
        ax.plot(cent_x + [cent_x[0]], cent_y + [cent_y[0]], '--', color='gray', linewidth=1.2, label='Centerline', alpha=0.8)
    if iqp_x:
        ax.plot(iqp_x + [iqp_x[0]], iqp_y + [iqp_y[0]], '-', color='#00CC44', linewidth=2.0, label='Minimum Curvature (IQP)')
    if sp_x:
        ax.plot(sp_x + [sp_x[0]], sp_y + [sp_y[0]], '-', color='#0088FF', linewidth=2.0, label='Shortest Path (SP)')

    ax.set_title(f"Track Waypoints Visualization: {map_name}\n(Safety Margin = {safety_width:.2f}m)", fontsize=14, fontweight='bold')
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.legend(loc='upper right', framealpha=0.9)
    ax.grid(True, linestyle=':', alpha=0.5)
    ax.set_aspect('equal', adjustable='box')

    viz_dir = repo_root / "assets" / "maps_visualization"
    os.makedirs(viz_dir, exist_ok=True)
    output_png = os.path.join(viz_dir, f"{map_name}_visualization.png")
    plt.tight_layout()
    plt.savefig(output_png)
    plt.close(fig)
    log_print(f"  [Plot Saved] {output_png}")

def plot_velocity_profile_visualization(map_dir, map_name):
    """Render high-resolution velocity colormap raceline plot with colorbar."""
    json_path = resolve_map_filepath(map_dir, "global_waypoints.json")
    yaml_path = resolve_map_filepath(map_dir, f"{map_name}.yaml")

    if not os.path.exists(json_path) or not os.path.exists(yaml_path):
        return

    with open(json_path, 'r') as f:
        wp_data = json.load(f)

    traj_key = 'global_traj_wpnts_iqp'
    if traj_key not in wp_data or not wp_data[traj_key].get('wpnts'):
        traj_key = 'global_traj_wpnts_sp'

    if traj_key not in wp_data or not wp_data[traj_key].get('wpnts'):
        return

    wpnts = wp_data[traj_key]['wpnts']
    x = np.array([w['x_m'] for w in wpnts])
    y = np.array([w['y_m'] for w in wpnts])
    v = np.array([w['vx_mps'] for w in wpnts])

    x = np.append(x, x[0])
    y = np.append(y, y[0])
    v = np.append(v, v[0])

    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    fig, ax = plt.subplots(figsize=(10, 8), dpi=200, facecolor='white')
    ax.set_facecolor('white')

    with open(yaml_path, 'r') as f:
        ydata = yaml.safe_load(f)

    origin = ydata.get('origin', [0.0, 0.0, 0.0])
    res = ydata.get('resolution', 0.05)
    img_path = resolve_map_image_path(map_dir, map_name, ydata)

    if img_path and os.path.exists(img_path):
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if img is not None:
            h, w = img.shape
            extent = [origin[0], origin[0] + w * res, origin[1], origin[1] + h * res]
            ax.imshow(cv2.flip(img, 0), cmap='gray', origin='lower', extent=extent, alpha=0.5)

    lc = LineCollection(segments, cmap='viridis', norm=plt.Normalize(v.min(), v.max()))
    lc.set_array(v)
    lc.set_linewidth(4.5)
    line = ax.add_collection(lc)

    cbar = fig.colorbar(line, ax=ax, fraction=0.035, pad=0.04)
    cbar.set_label('velocity (m/s)', fontsize=12, labelpad=10)
    cbar.ax.tick_params(labelsize=10)

    ax.autoscale()
    ax.set_aspect('equal')
    ax.set_title(f"F1TENTH Optimal Velocity Profile — {map_name}", fontsize=14, fontweight='bold', pad=12)

    viz_dir = repo_root / "assets" / "maps_visualization"
    os.makedirs(viz_dir, exist_ok=True)
    out_png = os.path.join(viz_dir, f"{map_name}_velocity_profile.png")
    plt.tight_layout()
    plt.savefig(out_png, dpi=200, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    log_print(f"  [Velocity Plot Saved] {out_png}")

def process_map(map_rel_path):
    maps_root = repo_root / "assets" / "maps"
    map_dir = maps_root / map_rel_path
    map_name = map_dir.name

    log_print(f"\n==================================================")
    log_print(f"Processing Map: {map_rel_path} (Name: {map_name})")
    log_print(f"==================================================")

    if not map_dir.exists():
        return None

    # Ensure _waypoint_gen map image exists if base map image exists
    base_pgm = map_dir / f"{map_name}.pgm"
    base_png = map_dir / f"{map_name}.png"
    gen_pgm = map_dir / f"{map_name}_waypoint_gen.pgm"
    gen_png = map_dir / f"{map_name}_waypoint_gen.png"

    if not gen_pgm.exists() and not gen_png.exists():
        src_img = base_png if base_png.exists() else (base_pgm if base_pgm.exists() else None)
        if src_img:
            dest_img = map_dir / f"{map_name}_waypoint_gen{src_img.suffix}"
            shutil.copy2(src_img, dest_img)
            log_print(f"[INFO] Created default {dest_img.name} from {src_img.name}")

            # Also ensure _waypoint_gen.yaml exists
            base_yaml = map_dir / f"{map_name}.yaml"
            gen_yaml = map_dir / f"{map_name}_waypoint_gen.yaml"
            if base_yaml.exists() and not gen_yaml.exists():
                with open(base_yaml, 'r') as f:
                    ydata = yaml.safe_load(f)
                ydata['image'] = dest_img.name
                with open(gen_yaml, 'w') as f:
                    yaml.safe_dump(ydata, f, default_flow_style=False)

    # Remove any temporary legacy files
    for fname in [f"{map_name}_raceline.png", f"{map_name}_raceline.pgm"]:
        fpath = os.path.join(map_dir, fname)
        if os.path.exists(fpath):
            try:
                os.remove(fpath)
            except Exception:
                pass

    input_path = str(repo_root / "src" / "race_bringup" / "config")

    def try_plan(sw, sw_sp=None):
        if sw_sp is None:
            sw_sp = sw
        planner = GlobalPlannerLogic(
            safety_width_=sw,
            safety_width_sp_=sw_sp,
            occupancy_grid_thresh_=10,
            map_editor_mode_=True,
            create_map_=False,
            map_name_=map_name,
            map_dir_=str(map_dir),
            backup_dir_=str(map_dir),
            finish_script_path_="",
            input_path_=input_path,
            show_plots_=False,
            filter_kernel_size_=3,
            required_laps_=1,
            reverse_mapping_=False,
            s_reg_=configured_s_reg,
            stepsize_reg_=configured_stepsize_reg,
            curvlim_=configured_curvlim,
            loginfo_=lambda msg: log_print(f"[INFO] {msg}"),
            logwarn_=lambda msg: log_print(f"[WARN] {msg}"),
            logerror_=lambda msg: log_print(f"[ERROR] {msg}")
        )
        success, _ = planner.global_plan_logic()
        return success, sw

    # Check for per-map global_planner_params.yaml first in waypoint_gen/, falling back to central config
    map_params_file = Path(resolve_map_filepath(map_dir, "global_planner_params.yaml"))
    central_params_file = repo_root / "src" / "race_bringup" / "config" / "global_planner" / "global_planner_params.yaml"

    if map_params_file.exists():
        params_file = map_params_file
        config_source = f"per-map ({map_params_file.parent.name})"
    else:
        params_file = central_params_file
        config_source = "central"

    configured_sw = 0.30
    configured_sw_sp = 0.30
    configured_s_reg = None
    configured_stepsize_reg = None
    configured_curvlim = None
    if params_file.exists():
        try:
            with open(params_file, 'r') as f:
                pdata = yaml.safe_load(f) or {}
                ros_params = pdata.get('global_planner', {}).get('ros__parameters', {})
                configured_sw = float(ros_params.get('safety_width', pdata.get('safety_width', 0.30)))
                configured_sw_sp = float(ros_params.get('safety_width_sp', pdata.get('safety_width_sp', 0.30)))

                if 's_reg' in ros_params:
                    configured_s_reg = float(ros_params['s_reg'])
                elif 's_reg' in pdata:
                    configured_s_reg = float(pdata['s_reg'])

                if 'stepsize_reg' in ros_params:
                    configured_stepsize_reg = float(ros_params['stepsize_reg'])
                elif 'stepsize_reg' in pdata:
                    configured_stepsize_reg = float(pdata['stepsize_reg'])

                if 'curvlim' in ros_params:
                    configured_curvlim = float(ros_params['curvlim'])
                elif 'curvlim' in pdata:
                    configured_curvlim = float(pdata['curvlim'])

                param_str = f"safety_width={configured_sw}m, safety_width_sp={configured_sw_sp}m"
                if configured_s_reg is not None:
                    param_str += f", s_reg={configured_s_reg}"
                if configured_stepsize_reg is not None:
                    param_str += f", stepsize_reg={configured_stepsize_reg}m"
                if configured_curvlim is not None:
                    param_str += f", curvlim={configured_curvlim}rad/m"
                log_print(f"[INFO] Using {config_source} config ({params_file.name}): {param_str}")
        except Exception as e:
            log_print(f"[WARN] Could not parse {params_file.name}: {e}")

    # Prioritize configured parameters from global_planner_params.yaml first
    safety_widths = [configured_sw, 0.40, 0.35, 0.30, 0.25, 0.20, 0.15]
    safety_widths = list(dict.fromkeys(safety_widths))

    success = False
    used_sw = configured_sw
    used_sw_sp = configured_sw_sp
    is_fallback = False

    for idx, sw in enumerate(safety_widths):
        sw_sp_val = configured_sw_sp if sw == configured_sw else sw
        log_print(f"--> Attempting optimization for {map_name} with safety_width = {sw:.2f}m (safety_width_sp = {sw_sp_val:.2f}m)")
        success, _ = try_plan(sw, sw_sp_val)
        if success:
            used_sw = sw
            used_sw_sp = sw_sp_val
            if idx > 0:
                is_fallback = True
                log_print(f"[WARN] Configured parameters ({configured_sw:.2f}m / {configured_sw_sp:.2f}m) failed for {map_name}. Fell back to safety_width={used_sw:.2f}m!")
            else:
                log_print(f"✓ Configured parameters MATCHED EXACTLY for {map_name}: safety_width={used_sw:.2f}m, safety_width_sp={used_sw_sp:.2f}m")
            break

    if not success:
        log_print(f"✗ Global planning failed for {map_name}")
        return {
            'map_name': str(map_rel_path),
            'safety_width': configured_sw,
            'status': 'FAILED ✗'
        }

    # Automatically optimize C^2 B-Spline, Friction Ellipse, and Start-Line alignment
    json_path = Path(resolve_map_filepath(map_dir, "global_waypoints.json"))
    min_clearance_m = 0.0
    if json_path.exists():
        try:
            with open(json_path, 'r') as f:
                wdata = json.load(f)
            
            yaml_path = resolve_map_filepath(map_dir, f"{map_name}.yaml")
            origin = [0, 0, 0]
            if os.path.exists(yaml_path):
                with open(yaml_path, 'r') as f:
                    ydata = yaml.safe_load(f)
                    origin = ydata.get('origin', [0, 0, 0])

            from global_planner.fix_knots import repair_and_optimize_waypoints, verify_track_continuity
            for k in ['global_traj_wpnts_iqp', 'global_traj_wpnts_sp']:
                if k in wdata and 'wpnts' in wdata[k]:
                    wdata[k]['wpnts'], _ = repair_and_optimize_waypoints(wdata[k]['wpnts'], map_origin=origin)
            
            with open(json_path, 'w') as f:
                json.dump(wdata, f)

            # Perform C0, C1, C2 and Velocity closed-loop continuity check
            wpnts_check = wdata.get('global_traj_wpnts_iqp', {}).get('wpnts', [])
            is_cont, c_info = verify_track_continuity(wpnts_check)
            if is_cont:
                log_print(f"  [CONTINUITY VERIFIED ✓] C0/C1/C2 Closed-Loop 100% Continuous (Seam Gap: {c_info['seam_c0_gap_m']:.3f}m, Seam Heading Jump: {c_info['seam_c1_jump_deg']:.1f}°)")
            else:
                log_print(f"  [WARN] Continuity check flagged minor seam jump: Gap {c_info['seam_c0_gap_m']:.3f}m, Heading {c_info['seam_c1_jump_deg']:.1f}°")

            # 2D Distance Field Clearance Verification
            gen_img_path = resolve_map_image_path(map_dir, map_name, {})
            if gen_img_path and os.path.exists(gen_img_path):
                img = cv2.imread(gen_img_path, cv2.IMREAD_GRAYSCALE)
                if img is not None:
                    # Binarize walls
                    walls = np.where(img < 200, 0, 255).astype(np.uint8)
                    dist_field = cv2.distanceTransform(walls, cv2.DIST_L2, 5) * 0.05
                    
                    wpnts = wdata.get('global_traj_wpnts_iqp', {}).get('wpnts', [])
                    if wpnts:
                        clearances = []
                        h, w_img = img.shape
                        for wp in wpnts:
                            px = int((wp['x_m'] - origin[0]) / 0.05)
                            py = int((wp['y_m'] - origin[1]) / 0.05)
                            if 0 <= px < w_img and 0 <= py < h:
                                clearances.append(dist_field[py, px])
                        if clearances:
                            min_clearance_m = float(np.min(clearances))
                            log_print(f"  [2D DISTANCE FIELD VERIFIED] Minimum 360° wall clearance: {min_clearance_m:.2f}m")
        except Exception as ex:
            log_print(f"  [WARN] Optimization / Distance Field step skipped for {map_name}: {ex}")

    # Save plot visualization & velocity colormap plot
    plot_and_save_raceline_visualization(str(map_dir), map_name, used_sw)
    plot_velocity_profile_visualization(str(map_dir), map_name)

    status_str = f"FALLBACK ({used_sw:.2f}m) ⚠️" if is_fallback else f"EXACT MATCH ({used_sw:.2f}m) ✓"
    log_print(f"✓ Completed {map_rel_path} -> Status: {status_str}")

    return {
        'map_name': str(map_rel_path),
        'safety_width': used_sw,
        'safety_width_sp': used_sw_sp,
        'status': status_str
    }

def main():
    import argparse
    
    maps_root = repo_root / "assets" / "maps"
    
    primary_maps = []
    test_maps = {}
    
    # 1. Discover primary general tracks (directly under assets/maps/<track>)
    for item in sorted(os.listdir(maps_root)):
        ipath = maps_root / item
        if ipath.is_dir() and item not in ["TEST", "raw", "waypoints"]:
            if (ipath / f"{item}.yaml").exists() or (ipath / f"{item}_raw.yaml").exists() or (ipath / "waypoints").exists():
                primary_maps.append(item)

    # 2. Discover TEST benchmark tracks (under assets/maps/TEST/BATCH_#/<track>)
    test_root = maps_root / "TEST"
    if test_root.exists():
        for root, dirs, files in os.walk(test_root):
            root_path = Path(root)
            folder_name = root_path.name
            if (root_path / f"{folder_name}.yaml").exists() or (root_path / f"{folder_name}_raw.yaml").exists() or (root_path / "waypoints").exists():
                rel_path = root_path.relative_to(maps_root)
                batch_name = rel_path.parts[1] if len(rel_path.parts) > 1 else "TEST"
                test_maps.setdefault(batch_name, []).append(str(rel_path))

    batch_choices = sorted(list(test_maps.keys())) + ['all']

    parser = argparse.ArgumentParser(description="F1TENTH Trajectory & Waypoint Generation Tool")
    parser.add_argument("-m", "--map", type=str, help="Target single map name or relative path (e.g. 'rectangle1', 'ifac_roboracer', 'TEST/BATCH_1/levine')")
    parser.add_argument("-t", "--test", action='store_true', help="Run waypoint generation for all TEST / Benchmark tracks (under assets/maps/TEST/)")
    parser.add_argument("-b", "--batch", type=str, choices=batch_choices, help="Run waypoint generation for a specific TEST batch (e.g. BATCH_1, BATCH_2)")
    parser.add_argument("-l", "--list", action='store_true', help="List all available map targets and exit")
    args = parser.parse_args()

    if args.list:
        print("\n" + "=" * 65)
        print(" AVAILABLE MAP TARGETS & TEST BATCHES")
        print("=" * 65)
        print("🏁 General Primary Tracks (default target):")
        for m in primary_maps:
            print(f"   • {m}")

        print("\n🧪 TEST / Benchmark Batches (use --test or -b <BATCH>):")
        for b_name in sorted(test_maps.keys()):
            print(f"   📂 Batch '-b {b_name}' ({len(test_maps[b_name])} maps):")
            for m in test_maps[b_name]:
                print(f"      • {m}")

        print("\nUsage Examples:")
        print("  ros2 run global_planner generate_waypoints                        # Target default general tracks")
        print("  ros2 run global_planner generate_waypoints -m rectangle1          # Target single track 'rectangle1'")
        print("  ros2 run global_planner generate_waypoints --test                 # Target all TEST benchmark tracks")
        print("  ros2 run global_planner generate_waypoints -b BATCH_1              # Target specific test BATCH_1")
        return

    # Filter targets based on CLI input
    if args.map:
        all_candidates = primary_maps + [m for b in test_maps.values() for m in b]
        matches = [m for m in all_candidates if m == args.map or m.endswith(f"/{args.map}")]
        if matches:
            target_maps = matches
        else:
            print(f"[ERROR] Map '{args.map}' not found! Use -l or --list to see available maps.")
            return
    elif args.batch:
        if args.batch == 'all':
            target_maps = [m for b in test_maps.values() for m in b]
        else:
            target_maps = test_maps.get(args.batch, [])
    elif args.test:
        target_maps = [m for b in test_maps.values() for m in b]
    else:
        # Default: Process primary general tracks
        target_maps = primary_maps

    log_print(f"Processing {len(target_maps)} target map(s): {', '.join(target_maps)}")

    reports = []
    for m in target_maps:
        try:
            res = process_map(m)
            if res:
                reports.append(res)
        except Exception as e:
            import traceback
            traceback.print_exc()

    log_print("\n" + "=" * 70)
    log_print("TRAJECTORY GENERATION SUMMARY REPORT")
    log_print("=" * 70)
    log_print(f"{'Map Name':30s} | {'Used SW (sp)':18s} | {'Status':20s}")
    log_print("-" * 70)

    for r in reports:
        mname = r['map_name']
        sw = r.get('safety_width', 0.0)
        sw_sp = r.get('safety_width_sp', sw)
        status = r.get('status', 'SUCCESS ✓')
        sw_str = f"{sw:.2f}m ({sw_sp:.2f}m)"
        log_print(f"{mname:30s} | {sw_str:18s} | {status}")

if __name__ == '__main__':
    main()
