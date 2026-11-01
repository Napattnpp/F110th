#!/usr/bin/env python3
"""
F1TENTH Web-Based Simulation Obstacle & Opponent Studio
--------------------------------------------------------
A zero-dependency, browser-based interactive studio for placing, dragging,
and managing static obstacles and dynamic opponents in the F1TENTH simulator.

Features:
- Zero X11 / Display forwarding required (ADR-003).
- Full Track Map & Raceline: Renders clean occupancy grid and global waypoints.
- Live Interactive Placement: Click to drop, drag to reposition obstacles on track.
- Dual Obstacle Support:
    1. Static Obstacles (configurable size, Frenet s/d, Cartesian x/y).
    2. Dynamic Opponents (speed scaler, constant speed, trajectory path).
- Live ROS 2 Publisher Bridge:
    - Publishes to /perception/fake_obstacles (ObstacleArray)
    - Publishes to /perception/dummy_obstacle_markers (MarkerArray for RViz)
    - Subscribes to /car_state/odom for live ego car visualization
- Standalone / Host Fallback: Runs on macOS host or in Docker without crashing.
- Scenario Save/Load: Persists scenarios to JSON for automated replay.
"""

import os
import sys
import json
import math
import time
import io
import threading
import argparse
from pathlib import Path
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
from typing import Any

# Locate project repository root
def find_repo_root():
    curr = Path(__file__).resolve().parent
    for p in [curr] + list(curr.parents):
        if (p / 'assets' / 'maps').is_dir() or (p / 'docker-compose.yml').is_file():
            return p
    return curr.parent.parent

REPO_ROOT = find_repo_root()
MAPS_ROOT = REPO_ROOT / "assets" / "maps"

# Optional YAML helper
try:
    import yaml
    HAVE_PYYAML = True
except ImportError:
    HAVE_PYYAML = False

def parse_yaml_file(fpath):
    if not os.path.exists(fpath):
        return None
    if HAVE_PYYAML:
        try:
            with open(fpath, 'r') as f:
                return yaml.safe_load(f)
        except Exception:
            pass

    # Built-in lightweight fallback parser for map YAMLs
    try:
        with open(fpath, 'r') as f:
            lines = f.readlines()
        data = {}
        current_list_key = None
        for line in lines:
            raw = line.rstrip()
            line_str = raw.strip()
            if not line_str or line_str.startswith('#'):
                continue
            if line_str.startswith('- '):
                # Multiline list item (e.g. origin coordinates)
                val_str = line_str[2:].strip()
                try:
                    val = float(val_str) if '.' in val_str else int(val_str)
                except ValueError:
                    val = val_str
                if current_list_key and current_list_key in data and isinstance(data[current_list_key], list):
                    data[current_list_key].append(val)
                continue

            current_list_key = None
            if ':' not in line_str:
                continue
            k, v = line_str.split(':', 1)
            k = k.strip()
            v = v.strip()
            if not v:
                current_list_key = k
                data[k] = []
            elif v.startswith('[') and v.endswith(']'):
                items = [x.strip() for x in v[1:-1].split(',') if x.strip()]
                parsed_items = []
                for it in items:
                    try:
                        parsed_items.append(float(it) if '.' in it else int(it))
                    except ValueError:
                        parsed_items.append(it)
                data[k] = parsed_items
            else:
                try:
                    data[k] = float(v) if '.' in v else int(v)
                except ValueError:
                    data[k] = v
        return data
    except Exception:
        return None

def detect_active_ros_map():
    """Auto-detects the currently running map from ROS parameters, env, or configs."""
    # 1. Environment variable
    env_map = os.environ.get('MAP')
    if env_map and (MAPS_ROOT / env_map).is_dir():
        return env_map

    # 2. Query ROS 2 parameters
    queries = [
        'ros2 param get /global_republisher map_path',
        'ros2 param get /global_parameters map_path',
        'ros2 param get /map_server yaml_filename'
    ]
    for q in queries:
        try:
            import subprocess
            res = subprocess.run(
                ['bash', '-c', f'source /opt/ros/humble/setup.bash >/dev/null 2>&1 && {q}'],
                capture_output=True, text=True, timeout=1.5
            )
            if res.returncode == 0:
                for line in res.stdout.splitlines():
                    if 'String value is:' in line:
                        p = line.split('String value is:', 1)[1].strip()
                        parts = Path(p).parts
                        for i, part in enumerate(parts):
                            if part in ['maps', 'race_bringup'] and i + 1 < len(parts):
                                cand = parts[i + 1]
                                if (MAPS_ROOT / cand).is_dir():
                                    return cand
                        m_name = os.path.basename(p.rstrip('/'))
                        if (MAPS_ROOT / m_name).is_dir():
                            return m_name
        except Exception:
            pass

    return "ifac_roboracer"

# -----------------------------------------------------------------------------
# Global Scenario State & In-Memory Store
# -----------------------------------------------------------------------------
class ScenarioStore:
    def __init__(self):
        self.lock = threading.Lock()
        self.map_obstacles = {} # map_name -> list of dicts: id, is_static, s, d, x, y, size, vs, label, yaw
        self.map_tracks = {}    # map_name -> {'waypoints': list, 'track_length': float}
        self.next_id = 901
        self.ego_pose = None # {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 's': 0.0, 'vs': 0.0}
        self.current_map = "ifac_roboracer"
        self.waypoints = []
        self.track_length = 0.0

    @property
    def obstacles(self):
        if self.current_map not in self.map_obstacles:
            self.map_obstacles[self.current_map] = []
        return self.map_obstacles[self.current_map]

    @obstacles.setter
    def obstacles(self, val):
        self.map_obstacles[self.current_map] = list(val)

    def load_track(self, map_name, waypoints, track_length):
        with self.lock:
            self.map_tracks[map_name] = {
                'waypoints': list(waypoints),
                'track_length': float(track_length)
            }
            self.current_map = map_name
            self.waypoints = list(waypoints)
            self.track_length = float(track_length)

            if map_name not in self.map_obstacles:
                # Auto-load saved scenario if it exists for this map
                sc_path = MAPS_ROOT / map_name / "waypoints" / "sim_scenario.json"
                if not sc_path.exists():
                    sc_path = MAPS_ROOT / map_name / "sim_scenario.json"
                if sc_path.exists():
                    try:
                        with open(sc_path, 'r') as f:
                            sc_data = json.load(f)
                            self.map_obstacles[map_name] = [dict(o) for o in sc_data.get('obstacles', [])]
                    except Exception:
                        self.map_obstacles[map_name] = []
                else:
                    self.map_obstacles[map_name] = []

            # Re-project Frenet (s, d) to Cartesian (x, y, yaw) using this map's active waypoints
            if self.track_length > 0 and self.waypoints:
                for obs in self.map_obstacles[map_name]:
                    s = float(obs.get('s', 0.0))
                    d = float(obs.get('d', 0.0))
                    x, y, yaw = self._frenet_to_cartesian_locked(s, d, map_name=map_name)
                    obs['x'] = x
                    obs['y'] = y
                    obs['yaw'] = yaw

    def step(self, dt: float):
        with self.lock:
            for m_name, obs_list in list(self.map_obstacles.items()):
                track_info = self.map_tracks.get(m_name)
                if not track_info or track_info['track_length'] <= 0:
                    continue
                t_len = track_info['track_length']
                for obs in obs_list:
                    if not obs.get('is_static', True) and not obs.get('is_dragging', False):
                        vs = float(obs.get('vs', 1.2))
                        if vs > 0:
                            s = (float(obs.get('s', 0.0)) + vs * dt) % t_len
                            obs['s'] = s
                            d = float(obs.get('d', 0.0))
                            x, y, yaw = self._frenet_to_cartesian_locked(s, d, map_name=m_name)
                            obs['x'] = x
                            obs['y'] = y
                            obs['yaw'] = yaw

    def _frenet_to_cartesian_locked(self, s: float, d: float, map_name: str = None):
        track_info = self.map_tracks.get(map_name) if map_name else None
        if track_info and track_info['waypoints']:
            total_len = track_info['track_length']
            wpnts = track_info['waypoints']
        else:
            total_len = self.track_length
            wpnts = self.waypoints

        if not wpnts or total_len <= 0:
            return 0.0, 0.0, 0.0

        mod_s = ((s % total_len) + total_len) % total_len
        n = len(wpnts)
        if n < 2:
            return 0.0, 0.0, 0.0

        w1 = wpnts[0]
        w2 = wpnts[1]
        for i in range(n):
            if wpnts[i]['s'] >= mod_s:
                w2 = wpnts[i]
                w1 = wpnts[(i - 1 + n) % n]
                break

        span = (w2['s'] - w1['s'] + total_len) % total_len or 0.1
        ratio = max(0.0, min(1.0, ((mod_s - w1['s'] + total_len) % total_len) / span))
        ref_x = w1['x'] + ratio * (w2['x'] - w1['x'])
        ref_y = w1['y'] + ratio * (w2['y'] - w1['y'])
        
        # Smooth yaw interpolation with angle wrap handling
        d_psi = math.atan2(math.sin(w2['psi'] - w1['psi']), math.cos(w2['psi'] - w1['psi']))
        psi = w1['psi'] + ratio * d_psi
        psi = math.atan2(math.sin(psi), math.cos(psi))

        x = ref_x - d * math.sin(psi)
        y = ref_y + d * math.cos(psi)
        return x, y, psi

    def get_obstacles(self, map_name=None):
        with self.lock:
            m = map_name or self.current_map
            return [dict(o) for o in self.map_obstacles.get(m, [])]

    def set_obstacles(self, obs_list, map_name=None):
        with self.lock:
            m = map_name or self.current_map
            self.map_obstacles[m] = [dict(o) for o in obs_list]

    def add_obstacle(self, obs, map_name=None):
        with self.lock:
            m = map_name or self.current_map
            if m not in self.map_obstacles:
                self.map_obstacles[m] = []
            if 'id' not in obs or not obs['id']:
                obs['id'] = self.next_id
                self.next_id += 1
            s = float(obs.get('s', 0.0))
            d = float(obs.get('d', 0.0))
            x, y, yaw = self._frenet_to_cartesian_locked(s, d, map_name=m)
            obs['x'] = x
            obs['y'] = y
            obs['yaw'] = yaw
            self.map_obstacles[m].append(obs)
            return obs

    def update_obstacle(self, obs_id, updates, map_name=None):
        with self.lock:
            m = map_name or self.current_map
            for obs in self.map_obstacles.get(m, []):
                if obs['id'] == obs_id or str(obs['id']) == str(obs_id):
                    obs.update(updates)
                    s = float(obs.get('s', 0.0))
                    d = float(obs.get('d', 0.0))
                    x, y, yaw = self._frenet_to_cartesian_locked(s, d, map_name=m)
                    obs['yaw'] = yaw
                    if 's' in updates or 'd' in updates or 'is_static' in updates:
                        obs['x'] = x
                        obs['y'] = y
                    return obs
            return None

    def delete_obstacle(self, obs_id, map_name=None):
        with self.lock:
            m = map_name or self.current_map
            self.map_obstacles[m] = [o for o in self.map_obstacles.get(m, []) if o['id'] != obs_id and str(o['id']) != str(obs_id)]

    def clear(self, map_name=None):
        with self.lock:
            m = map_name or self.current_map
            self.map_obstacles[m] = []

    def set_ego_pose(self, pose):
        with self.lock:
            self.ego_pose = pose

    def get_ego_pose(self):
        with self.lock:
            return dict(self.ego_pose) if self.ego_pose else None

STORE = ScenarioStore()

# -----------------------------------------------------------------------------
# ROS 2 Publisher Bridge (Non-blocking daemon thread if rclpy is installed)
# -----------------------------------------------------------------------------
ROS_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from f110_msgs.msg import ObstacleArray, Obstacle
    from visualization_msgs.msg import Marker, MarkerArray
    from nav_msgs.msg import Odometry
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

class SimObstacleRosNode:
    def __init__(self, store: ScenarioStore):
        self.store = store
        self.node = None
        self.pub_obstacles = None
        self.pub_markers = None
        self.sub_odom = None
        self.thread = None
        self.prev_marker_ids = set()

    def start(self):
        if not ROS_AVAILABLE:
            print("[INFO] ROS 2 (rclpy/f110_msgs) not available on host. Running in Standalone Web Mode.")
            return
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        try:
            if not rclpy.ok():
                rclpy.init()
            self.node = Node('sim_obstacle_web_studio')
            self.pub_obstacles = self.node.create_publisher(ObstacleArray, '/perception/fake_obstacles', 10)
            self.pub_markers = self.node.create_publisher(MarkerArray, '/perception/dummy_obstacle_markers', 10)
            self.sub_odom = self.node.create_subscription(Odometry, '/car_state/odom', self._odom_cb, 10)

            timer_period = 0.05 # 20 Hz
            self.node.create_timer(timer_period, self._publish_cycle)
            print("[INFO] ROS 2 Sim Obstacle Bridge active -> Publishing to /perception/fake_obstacles at 20 Hz")
            rclpy.spin(self.node)
        except Exception as e:
            print(f"[WARN] ROS 2 Bridge error: {e}. Falling back to standalone mode.")

    def _odom_cb(self, msg: Any):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        # Compute yaw from quaternion
        siny_cosp = 2 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1 - 2 * (o.y * o.y + o.z * o.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        vs = msg.twist.twist.linear.x
        self.store.set_ego_pose({'x': p.x, 'y': p.y, 'yaw': yaw, 'vs': vs})

    def _publish_cycle(self):
        obs_list = self.store.get_obstacles()
        obs_msg = ObstacleArray()
        obs_msg.header.stamp = self.node.get_clock().now().to_msg()
        obs_msg.header.frame_id = 'frenet'

        mrk_msg = MarkerArray()
        active_m = self.store.current_map
        track_info = self.store.map_tracks.get(active_m)
        track_len = track_info['track_length'] if track_info and track_info['track_length'] > 0 else (self.store.track_length if self.store.track_length > 0 else 1000.0)

        current_ids = set()
        for o in obs_list:
            obs_id = int(o.get('id', 999))
            current_ids.add(obs_id)

        # Clear removed obstacle markers immediately from Foxglove and RViz
        removed_ids = self.prev_marker_ids - current_ids
        for r_id in removed_ids:
            del_mrk = Marker()
            del_mrk.header.stamp = obs_msg.header.stamp
            del_mrk.header.frame_id = 'map'
            del_mrk.ns = 'sim_studio_obstacles'
            del_mrk.id = r_id
            del_mrk.action = Marker.DELETE
            mrk_msg.markers.append(del_mrk)

        if not current_ids:
            del_all = Marker()
            del_all.header.stamp = obs_msg.header.stamp
            del_all.header.frame_id = 'map'
            del_all.ns = 'sim_studio_obstacles'
            del_all.action = Marker.DELETEALL
            mrk_msg.markers.append(del_all)

        self.prev_marker_ids = current_ids

        for o in obs_list:
            is_static = bool(o.get('is_static', True))
            s = float(o.get('s', 0.0))
            d = float(o.get('d', 0.0))
            size = float(o.get('size', 0.40))
            vs = float(o.get('vs', 0.0)) if not is_static else 0.0

            # Obstacle message
            obs = Obstacle()
            obs.id = int(o.get('id', 999))
            obs.s_center = s
            obs.d_center = d
            obs.s_start = (s - size / 2.0) % track_len
            obs.s_end = (s + size / 2.0) % track_len
            obs.d_left = d + size / 2.0
            obs.d_right = d - size / 2.0
            obs.size = size
            obs.vs = vs
            obs.vd = 0.0
            obs.is_static = is_static
            obs.is_visible = True
            obs.is_actually_a_gap = False
            obs_msg.obstacles.append(obs)

            # RViz Marker message
            mrk = Marker()
            mrk.header.stamp = obs_msg.header.stamp
            mrk.header.frame_id = 'map'
            mrk.ns = 'sim_studio_obstacles'
            mrk.id = obs.id
            mrk.type = Marker.CUBE
            mrk.action = Marker.ADD
            # Non-zero lifetime (0.3s) ensures removed obstacles automatically disappear in Foxglove without restart
            mrk.lifetime.sec = 0
            mrk.lifetime.nanosec = 300000000
            mrk.pose.position.x = float(o.get('x', 0.0))
            mrk.pose.position.y = float(o.get('y', 0.0))
            mrk.pose.position.z = 0.15
            yaw = float(o.get('yaw', 0.0))
            mrk.pose.orientation.z = math.sin(yaw / 2.0)
            mrk.pose.orientation.w = math.cos(yaw / 2.0)
            if is_static:
                mrk.scale.x = size
                mrk.scale.y = size
                mrk.scale.z = 0.30
                mrk.color.a = 0.90
                mrk.color.r = 1.0
                mrk.color.g = 0.45
                mrk.color.b = 0.0
            else:
                mrk.scale.x = size * 1.5
                mrk.scale.y = size * 0.8
                mrk.scale.z = 0.25
                mrk.color.a = 0.95
                mrk.color.r = 0.95
                mrk.color.g = 0.15
                mrk.color.b = 0.15
            mrk_msg.markers.append(mrk)

        self.pub_obstacles.publish(obs_msg)
        self.pub_markers.publish(mrk_msg)

# -----------------------------------------------------------------------------
# Map Discovery and Data Loader Helpers
# -----------------------------------------------------------------------------
def list_available_maps():
    available = []
    if not MAPS_ROOT.is_dir():
        return available
    for root, dirs, files in os.walk(MAPS_ROOT):
        if 'global_waypoints.json' in files:
            rel_dir = Path(root).relative_to(MAPS_ROOT)
            if rel_dir.name == "waypoints":
                map_rel = str(rel_dir.parent)
            else:
                map_rel = str(rel_dir)
            if map_rel not in available:
                available.append(map_rel)
    return sorted(available)

def get_map_metadata(map_rel):
    map_dir = MAPS_ROOT / map_rel
    map_name = map_dir.name

    yaml_candidates = [
        map_dir / "clean" / f"{map_name}.yaml",
        map_dir / "waypoint_gen" / f"{map_name}_waypoint_gen.yaml",
        map_dir / f"{map_name}.yaml",
    ]
    ypath = next((p for p in yaml_candidates if p.exists()), None)
    if not ypath:
        return {"has_image": False}

    ydata = parse_yaml_file(ypath) or {}
    resolution = float(ydata.get("resolution", 0.05))
    origin = ydata.get("origin", [0.0, 0.0, 0.0])
    if isinstance(origin, list) and len(origin) >= 2:
        flat_origin = []
        for it in origin:
            if isinstance(it, list):
                flat_origin.extend(it)
            else:
                flat_origin.append(it)
        origin = flat_origin
    else:
        origin = [0.0, 0.0, 0.0]

    image_filename = ydata.get("image", f"{map_name}.png")
    image_dir = ypath.parent
    img_path = image_dir / image_filename
    if not img_path.exists():
        for ext in [".png", ".pgm"]:
            alt = image_dir / f"{map_name}{ext}"
            if alt.exists():
                img_path = alt
                break

    width_px, height_px = 0, 0
    if img_path.exists():
        if img_path.suffix == ".png":
            try:
                with open(img_path, 'rb') as f:
                    f.seek(16)
                    hdr = f.read(8)
                    if len(hdr) == 8:
                        import struct
                        width_px, height_px = struct.unpack('>II', hdr)
            except Exception:
                pass
        elif img_path.suffix == ".pgm":
            try:
                with open(img_path, 'rb') as f:
                    line = f.readline().decode('latin1').strip()
                    if line in ['P5', 'P2']:
                        line = f.readline().decode('latin1').strip()
                        while line.startswith('#'):
                            line = f.readline().decode('latin1').strip()
                        parts = line.split()
                        width_px, height_px = int(parts[0]), int(parts[1])
            except Exception:
                pass

    return {
        "has_image": img_path.exists(),
        "img_path": str(img_path) if img_path.exists() else None,
        "image_url": f"/api/map_image?map={map_rel}",
        "resolution": resolution,
        "origin": origin,
        "width_px": width_px,
        "height_px": height_px,
        "width_m": width_px * resolution,
        "height_m": height_px * resolution
    }

def load_map_data(map_rel):
    map_dir = MAPS_ROOT / map_rel
    wp_path = map_dir / "waypoints" / "global_waypoints.json"
    if not wp_path.exists():
        wp_path = map_dir / "global_waypoints.json"
    if not wp_path.exists():
        return {"error": f"Waypoints not found for map {map_rel}"}

    with open(wp_path, 'r') as f:
        wp_raw = json.load(f)

    # Extract waypoints
    if isinstance(wp_raw, dict):
        if 'global_traj_wpnts_iqp' in wp_raw and 'wpnts' in wp_raw['global_traj_wpnts_iqp']:
            wpnts = wp_raw['global_traj_wpnts_iqp']['wpnts']
        elif 'global_traj_wpnts_sp' in wp_raw and 'wpnts' in wp_raw['global_traj_wpnts_sp']:
            wpnts = wp_raw['global_traj_wpnts_sp']['wpnts']
        elif 'wpnts' in wp_raw:
            wpnts = wp_raw['wpnts']
        else:
            wpnts = []
    elif isinstance(wp_raw, list):
        wpnts = wp_raw
    else:
        wpnts = []

    compact_wpnts = []
    for w in wpnts:
        compact_wpnts.append({
            'x': float(w.get('x_m', 0.0)),
            'y': float(w.get('y_m', 0.0)),
            's': float(w.get('s_m', 0.0)),
            'v': float(w.get('vx_mps', 0.0)),
            'psi': float(w.get('psi_rad', 0.0)),
            'd_left': float(w.get('d_left', 0.5)),
            'd_right': float(w.get('d_right', -0.5)),
            'kappa': float(w.get('kappa_radpm', 0.0))
        })

    meta = get_map_metadata(map_rel)
    track_len = compact_wpnts[-1]['s'] if compact_wpnts else 0.0
    STORE.load_track(map_rel, compact_wpnts, track_len)

    return {
        "map_name": map_rel,
        "map_meta": meta,
        "track_length": track_len,
        "num_waypoints": len(compact_wpnts),
        "waypoints": compact_wpnts
    }

# -----------------------------------------------------------------------------
# Web Application Frontend (Single-Page App, Pure HTML/CSS/JS, Zero Dependency)
# -----------------------------------------------------------------------------
HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Sim Obstacle & Opponent Studio — F1TENTH</title>
  <style>
    :root {
      --bg-base: #0b0f19;
      --bg-surface: #121826;
      --bg-card: #182234;
      --bg-card-hover: #1f2c42;
      --border-subtle: #26354d;
      --border-active: #3b82f6;
      --text-main: #f0f4fc;
      --text-muted: #8ca0bc;
      --text-dim: #546884;
      --primary: #3b82f6;
      --primary-hover: #2563eb;
      --accent-orange: #f97316;
      --accent-red: #ef4444;
      --accent-emerald: #10b981;
      --accent-purple: #8b5cf6;
      --accent-amber: #f59e0b;
      --font-mono: 'JetBrains Mono', 'Fira Code', Menlo, Consolas, monospace;
      --font-sans: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      background-color: var(--bg-base);
      color: var(--text-main);
      font-family: var(--font-sans);
      height: 100vh;
      overflow: hidden;
      display: flex;
      flex-direction: column;
    }
    header {
      background-color: var(--bg-surface);
      border-bottom: 1px solid var(--border-subtle);
      height: 52px;
      padding: 0 16px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      z-index: 20;
    }
    .brand {
      display: flex;
      align-items: center;
      gap: 10px;
      font-weight: 700;
      font-size: 14px;
      letter-spacing: 0.5px;
    }
    .badge {
      background: linear-gradient(135deg, var(--accent-orange), var(--accent-red));
      color: white;
      padding: 3px 8px;
      border-radius: 4px;
      font-size: 11px;
      font-weight: 800;
    }
    .header-controls {
      display: flex;
      align-items: center;
      gap: 12px;
    }
    .select-map {
      background-color: var(--bg-card);
      border: 1px solid var(--border-subtle);
      color: var(--text-main);
      padding: 6px 12px;
      border-radius: 6px;
      font-size: 13px;
      font-family: var(--font-mono);
      cursor: pointer;
    }
    .btn {
      padding: 6px 14px;
      border-radius: 6px;
      font-size: 12px;
      font-weight: 600;
      border: 1px solid transparent;
      cursor: pointer;
      display: inline-flex;
      align-items: center;
      gap: 6px;
      transition: all 0.15s ease;
    }
    .btn-primary { background-color: var(--primary); color: white; }
    .btn-primary:hover { background-color: var(--primary-hover); }
    .btn-secondary { background-color: var(--bg-card); border-color: var(--border-subtle); color: var(--text-main); }
    .btn-secondary:hover { background-color: var(--bg-card-hover); }
    .btn-danger { background-color: rgba(239, 68, 68, 0.15); border-color: rgba(239, 68, 68, 0.4); color: var(--accent-red); }
    .btn-danger:hover { background-color: rgba(239, 68, 68, 0.25); }

    .app-body {
      display: flex;
      flex: 1;
      height: calc(100vh - 52px);
      position: relative;
    }
    .canvas-container {
      flex: 1;
      position: relative;
      background: radial-gradient(circle at center, #111827 0%, #080c14 100%);
      overflow: hidden;
      cursor: grab;
    }
    .canvas-container.grabbing { cursor: grabbing; }
    .canvas-container.crosshair { cursor: crosshair; }
    canvas { display: block; width: 100%; height: 100%; }

    .canvas-hud {
      position: absolute;
      top: 14px;
      left: 14px;
      display: flex;
      gap: 12px;
      z-index: 10;
      pointer-events: none;
    }
    .hud-card {
      background: rgba(18, 24, 38, 0.85);
      backdrop-filter: blur(8px);
      border: 1px solid var(--border-subtle);
      border-radius: 6px;
      padding: 6px 12px;
      display: flex;
      flex-direction: column;
    }
    .hud-label { font-size: 9px; font-weight: 700; color: var(--text-dim); letter-spacing: 0.5px; }
    .hud-val { font-size: 14px; font-family: var(--font-mono); font-weight: 700; color: var(--text-main); }

    .canvas-toolbar {
      position: absolute;
      top: 14px;
      right: 14px;
      display: flex;
      gap: 6px;
      background: rgba(18, 24, 38, 0.85);
      backdrop-filter: blur(8px);
      border: 1px solid var(--border-subtle);
      padding: 4px;
      border-radius: 6px;
      z-index: 10;
    }
    .tool-btn {
      background: transparent;
      border: none;
      color: var(--text-muted);
      padding: 6px 10px;
      border-radius: 4px;
      font-size: 12px;
      cursor: pointer;
      font-weight: 500;
      display: flex;
      align-items: center;
      gap: 5px;
    }
    .tool-btn:hover { color: var(--text-main); background: var(--bg-card); }
    .tool-btn.active { color: white; background: var(--primary); }

    .sidebar {
      width: 340px;
      background-color: var(--bg-surface);
      border-left: 1px solid var(--border-subtle);
      display: flex;
      flex-direction: column;
      z-index: 15;
    }
    .sidebar-header {
      padding: 14px 16px;
      border-bottom: 1px solid var(--border-subtle);
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    .sidebar-title { font-size: 13px; font-weight: 700; color: var(--text-main); }
    .sidebar-content {
      flex: 1;
      overflow-y: auto;
      padding: 14px 16px;
      display: flex;
      flex-direction: column;
      gap: 14px;
    }

    .card {
      background-color: var(--bg-card);
      border: 1px solid var(--border-subtle);
      border-radius: 6px;
      padding: 12px;
      display: flex;
      flex-direction: column;
      gap: 10px;
    }
    .card-title {
      font-size: 11px;
      font-weight: 700;
      color: var(--text-muted);
      letter-spacing: 0.5px;
      text-transform: uppercase;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    .param-row {
      display: flex;
      align-items: center;
      justify-content: space-between;
      font-size: 12px;
    }
    .param-label { color: var(--text-muted); }
    .param-input {
      background: var(--bg-surface);
      border: 1px solid var(--border-subtle);
      color: var(--text-main);
      padding: 4px 8px;
      border-radius: 4px;
      font-family: var(--font-mono);
      font-size: 12px;
      width: 90px;
      text-align: right;
    }

    .obs-item {
      background-color: var(--bg-card);
      border: 1px solid var(--border-subtle);
      border-radius: 6px;
      padding: 10px;
      display: flex;
      flex-direction: column;
      gap: 6px;
      cursor: pointer;
      transition: all 0.15s ease;
    }
    .obs-item:hover { background-color: var(--bg-card-hover); }
    .obs-item.selected { border-color: var(--accent-orange); background: rgba(249, 115, 22, 0.08); }
    .obs-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    .obs-tag {
      font-size: 11px;
      font-weight: 700;
      display: flex;
      align-items: center;
      gap: 6px;
    }
    .dot {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      display: inline-block;
    }
    .dot-static { background: var(--accent-orange); }
    .dot-dynamic { background: var(--accent-red); }

    .toast {
      position: fixed;
      bottom: 20px;
      right: 20px;
      background: var(--bg-surface);
      border: 1px solid var(--border-active);
      color: var(--text-main);
      padding: 10px 18px;
      border-radius: 6px;
      font-size: 13px;
      font-weight: 500;
      box-shadow: 0 8px 24px rgba(0,0,0,0.5);
      transform: translateY(100px);
      opacity: 0;
      transition: all 0.25s ease;
      z-index: 100;
    }
    .toast.show { transform: translateY(0); opacity: 1; }
  </style>
</head>
<body>
  <header>
    <div class="brand">
      <span class="badge">SIM STUDIO</span>
      <span>F1TENTH Obstacle & Opponent Studio</span>
    </div>
    <div class="header-controls">
      <select id="mapSelect" class="select-map"></select>
      <button class="btn btn-secondary" onclick="reloadMap()">↺ Reload</button>
      <button class="btn btn-secondary" onclick="clearAllObstacles()">🗑️ Clear</button>
      <button class="btn btn-primary" onclick="saveScenario()">💾 Save Scenario</button>
    </div>
  </header>

  <div class="app-body">
    <div class="canvas-container" id="canvasContainer">
      <canvas id="trackCanvas"></canvas>

      <div class="canvas-hud">
        <div class="hud-card">
          <span class="hud-label">TRACK LENGTH</span>
          <span class="hud-val" id="hudLength">-- m</span>
        </div>
        <div class="hud-card">
          <span class="hud-label">ACTIVE OBSTACLES</span>
          <span class="hud-val" id="hudCount">0</span>
        </div>
        <div class="hud-card">
          <span class="hud-label">CURSOR (S / D)</span>
          <span class="hud-val" id="hudCursor">-- / --</span>
        </div>
        <div class="hud-card" id="hudEgoCard" style="display:none;">
          <span class="hud-label">EGO SPEED</span>
          <span class="hud-val" id="hudEgoSpeed" style="color:var(--accent-emerald);">-- m/s</span>
        </div>
      </div>

      <div class="canvas-toolbar">
        <button class="tool-btn active" id="toolSelect" onclick="setTool('select')">👆 Select / Drag</button>
        <button class="tool-btn" id="toolStatic" onclick="setTool('static')">🟧 + Static</button>
        <button class="tool-btn" id="toolDynamic" onclick="setTool('dynamic')">🏎️ + Dynamic</button>
        <button class="tool-btn" onclick="resetView()">🎯 Fit View</button>
      </div>
    </div>

    <div class="sidebar">
      <div class="sidebar-header">
        <div class="sidebar-title">Scenario Inspector</div>
      </div>
      <div class="sidebar-content" id="sidebarContent">
        <!-- Rendered dynamically -->
      </div>
    </div>
  </div>

  <div id="toast" class="toast">Notice</div>

  <script>
    let currentMap = "ifac_roboracer";
    let mapData = null;
    let mapImg = null;
    let obstacles = [];
    let selectedObsId = null;
    let activeTool = 'select'; // 'select', 'static', 'dynamic'

    // Canvas State
    const canvas = document.getElementById('trackCanvas');
    const ctx = canvas.getContext('2d');
    const container = document.getElementById('canvasContainer');

    let viewX = 0, viewY = 0, viewScale = 30;
    let isPanning = false, panStartX = 0, panStartY = 0;
    let isDraggingObs = false, draggedObsId = null;

    window.addEventListener('load', async () => {
      resizeCanvas();
      window.addEventListener('resize', resizeCanvas);
      setupInteractions();
      await fetchMaps();
      // Polling loop for live ROS updates
      setInterval(pollLiveStatus, 100);
    });

    function resizeCanvas() {
      const dpr = window.devicePixelRatio || 1;
      canvas.width = container.clientWidth * dpr;
      canvas.height = container.clientHeight * dpr;
      ctx.scale(dpr, dpr);
      render();
    }

    async function fetchMaps() {
      try {
        const res = await fetch('/api/maps');
        const maps = await res.json();
        const sel = document.getElementById('mapSelect');
        sel.innerHTML = '';
        maps.forEach(m => {
          const opt = document.createElement('option');
          opt.value = m;
          opt.innerText = m;
          sel.appendChild(opt);
        });

        // Auto-detect active ROS map
        let defaultMap = maps.includes('ifac_roboracer') ? 'ifac_roboracer' : maps[0];
        try {
          const actRes = await fetch('/api/active_ros_map');
          const actData = await actRes.json();
          if (actData.active_map && maps.includes(actData.active_map)) {
            defaultMap = actData.active_map;
          }
        } catch (e) {}

        const urlParams = new URLSearchParams(window.location.search);
        const qMap = urlParams.get('map');
        if (maps.length > 0) {
          if (qMap && maps.includes(qMap)) {
            sel.value = qMap;
          } else {
            sel.value = defaultMap;
          }
          currentMap = sel.value;
          await loadMap(currentMap);
        }
        sel.addEventListener('change', () => {
          currentMap = sel.value;
          loadMap(currentMap);
        });
      } catch (err) {
        showToast('Failed to load map list: ' + err);
      }
    }

    async function loadMap(m) {
      try {
        selectedObsId = null;
        const res = await fetch(`/api/map_data?map=${encodeURIComponent(m)}`);
        mapData = await res.json();
        document.getElementById('hudLength').innerText = `${mapData.track_length.toFixed(1)} m`;

        if (mapData.map_meta && mapData.map_meta.has_image) {
          mapImg = new Image();
          mapImg.src = mapData.map_meta.image_url;
          mapImg.onload = () => render();
        } else {
          mapImg = null;
        }

        // Fetch existing obstacles for this specific map from server
        const obsRes = await fetch(`/api/obstacles?map=${encodeURIComponent(m)}`);
        obstacles = await obsRes.json();
        updateHudCount();

        resetView();
        renderSidebar();
      } catch (e) {
        showToast('Error loading map: ' + e);
      }
    }

    function reloadMap() { if (currentMap) loadMap(currentMap); }

    function resetView() {
      if (!mapData || !mapData.waypoints || mapData.waypoints.length === 0) return;
      let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
      mapData.waypoints.forEach(w => {
        minX = Math.min(minX, w.x);
        maxX = Math.max(maxX, w.x);
        minY = Math.min(minY, w.y);
        maxY = Math.max(maxY, w.y);
      });
      const cW = canvas.width / (window.devicePixelRatio || 1);
      const cH = canvas.height / (window.devicePixelRatio || 1);
      const trackW = Math.max(maxX - minX, 1);
      const trackH = Math.max(maxY - minY, 1);
      viewScale = Math.min((cW * 0.75) / trackW, (cH * 0.75) / trackH);
      viewX = cW / 2 - ((minX + maxX) / 2) * viewScale;
      viewY = cH / 2 + ((minY + maxY) / 2) * viewScale;
      render();
    }

    function setTool(t) {
      activeTool = t;
      ['toolSelect', 'toolStatic', 'toolDynamic'].forEach(id => {
        document.getElementById(id).classList.remove('active');
      });
      if (t === 'select') document.getElementById('toolSelect').classList.add('active');
      if (t === 'static') document.getElementById('toolStatic').classList.add('active');
      if (t === 'dynamic') document.getElementById('toolDynamic').classList.add('active');
      container.className = 'canvas-container ' + (t === 'select' ? '' : 'crosshair');
    }

    function worldToScreen(x, y) {
      return { x: x * viewScale + viewX, y: -y * viewScale + viewY };
    }
    function screenToWorld(sx, sy) {
      return { x: (sx - viewX) / viewScale, y: -(sy - viewY) / viewScale };
    }

    // Convert (x, y) to Frenet (s, d) using nearest waypoint
    function cartesianToFrenet(x, y) {
      if (!mapData || !mapData.waypoints) return { s: 0, d: 0, psi: 0 };
      let bestDist = Infinity, bestWp = mapData.waypoints[0];
      for (const w of mapData.waypoints) {
        const d = Math.hypot(w.x - x, w.y - y);
        if (d < bestDist) {
          bestDist = d;
          bestWp = w;
        }
      }
      const dx = x - bestWp.x;
      const dy = y - bestWp.y;
      const lat_d = -dx * Math.sin(bestWp.psi) + dy * Math.cos(bestWp.psi);
      return { s: bestWp.s, d: lat_d, psi: bestWp.psi };
    }

    // Convert Frenet (s, d) to Cartesian (x, y)
    function frenetToCartesian(s, d) {
      if (!mapData || !mapData.waypoints || mapData.waypoints.length < 2) return { x: 0, y: 0 };
      const totalLen = mapData.track_length;
      const modS = ((s % totalLen) + totalLen) % totalLen;
      const wpnts = mapData.waypoints;
      const n = wpnts.length;
      let w1 = wpnts[0], w2 = wpnts[1];
      for (let i = 0; i < n; i++) {
        if (wpnts[i].s >= modS) {
          w2 = wpnts[i];
          w1 = wpnts[(i - 1 + n) % n];
          break;
        }
      }
      const span = (w2.s - w1.s + totalLen) % totalLen || 0.1;
      const ratio = Math.max(0, Math.min(1, ((modS - w1.s + totalLen) % totalLen) / span));
      const refX = w1.x + ratio * (w2.x - w1.x);
      const refY = w1.y + ratio * (w2.y - w1.y);
      let d_psi = ((w2.psi - w1.psi + 3 * Math.PI) % (2 * Math.PI)) - Math.PI;
      const psi = w1.psi + ratio * d_psi;
      return {
        x: refX - d * Math.sin(psi),
        y: refY + d * Math.cos(psi)
      };
    }

    function setupInteractions() {
      container.addEventListener('mousedown', (e) => {
        const rect = canvas.getBoundingClientRect();
        const mx = e.clientX - rect.left;
        const my = e.clientY - rect.top;
        const world = screenToWorld(mx, my);

        if (activeTool === 'static' || activeTool === 'dynamic') {
          spawnObstacleAt(world.x, world.y, activeTool === 'static');
          setTool('select');
          return;
        }

        // Check if an obstacle was clicked
        const clickedObs = obstacles.find(o => Math.hypot(o.x - world.x, o.y - world.y) < (o.size || 0.4) * 1.5);
        if (clickedObs) {
          selectedObsId = clickedObs.id;
          isDraggingObs = true;
          draggedObsId = clickedObs.id;
          renderSidebar();
          render();
          return;
        }

        // Otherwise Pan
        isPanning = true;
        panStartX = mx - viewX;
        panStartY = my - viewY;
        container.classList.add('grabbing');
      });

      window.addEventListener('mousemove', (e) => {
        const rect = canvas.getBoundingClientRect();
        const mx = e.clientX - rect.left;
        const my = e.clientY - rect.top;
        const world = screenToWorld(mx, my);

        // Update cursor HUD
        if (mapData) {
          const fn = cartesianToFrenet(world.x, world.y);
          document.getElementById('hudCursor').innerText = `${fn.s.toFixed(1)}m / ${fn.d > 0 ? '+' : ''}${fn.d.toFixed(2)}m`;
        }

        if (isDraggingObs && draggedObsId) {
          const obs = obstacles.find(o => o.id === draggedObsId);
          if (obs) {
            obs.x = world.x;
            obs.y = world.y;
            const fn = cartesianToFrenet(world.x, world.y);
            obs.s = fn.s;
            obs.d = fn.d;
            render();
            syncObstacleUpdate(obs);
          }
          return;
        }

        if (isPanning) {
          viewX = mx - panStartX;
          viewY = my - panStartY;
          render();
        }
      });

      window.addEventListener('mouseup', () => {
        isPanning = false;
        isDraggingObs = false;
        draggedObsId = null;
        container.classList.remove('grabbing');
      });

      container.addEventListener('wheel', (e) => {
        e.preventDefault();
        const rect = canvas.getBoundingClientRect();
        const mx = e.clientX - rect.left;
        const my = e.clientY - rect.top;
        const factor = e.deltaY < 0 ? 1.15 : 0.85;
        viewX = mx - (mx - viewX) * factor;
        viewY = my - (my - viewY) * factor;
        viewScale *= factor;
        render();
      }, { passive: false });

      window.addEventListener('keydown', (e) => {
        if ((e.key === 'Delete' || e.key === 'Backspace') && selectedObsId && document.activeElement.tagName !== 'INPUT') {
          deleteObstacle(selectedObsId);
        }
      });
    }

    function spawnObstacleAt(x, y, isStatic) {
      const fn = cartesianToFrenet(x, y);
      // Auto-snap dynamic opponents to raceline center (d = 0.0) when spawned near raceline
      const dVal = (!isStatic && Math.abs(fn.d) < 0.8) ? 0.0 : fn.d;
      const cart = frenetToCartesian(fn.s, dVal);
      const newObs = {
        id: Math.floor(1000 + Math.random() * 9000),
        is_static: isStatic,
        s: fn.s,
        d: dVal,
        x: cart.x,
        y: cart.y,
        size: isStatic ? 0.40 : 0.35,
        vs: isStatic ? 0.0 : 2.0,
        label: isStatic ? 'Static Obstacle' : 'Dynamic Opponent'
      };
      obstacles.push(newObs);
      selectedObsId = newObs.id;
      updateHudCount();
      renderSidebar();
      render();
      syncObstacleAdd(newObs);
      showToast(`Added ${newObs.label} (#${newObs.id})`);
    }

    async function syncObstacleAdd(obs) {
      try {
        await fetch('/api/obstacles', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ action: 'add', map: currentMap, obstacle: obs })
        });
      } catch (e) {}
    }

    async function syncObstacleUpdate(obs) {
      try {
        await fetch('/api/obstacles', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ action: 'update', map: currentMap, id: obs.id, updates: obs })
        });
      } catch (e) {}
    }

    async function deleteObstacle(id) {
      obstacles = obstacles.filter(o => o.id !== id);
      if (selectedObsId === id) selectedObsId = null;
      updateHudCount();
      renderSidebar();
      render();
      try {
        await fetch('/api/obstacles', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ action: 'delete', map: currentMap, id: id })
        });
        showToast('Obstacle removed');
      } catch (e) {}
    }

    async function clearAllObstacles() {
      obstacles = [];
      selectedObsId = null;
      updateHudCount();
      renderSidebar();
      render();
      try {
        await fetch('/api/obstacles', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ action: 'clear', map: currentMap })
        });
        showToast('All obstacles cleared');
      } catch (e) {}
    }

    async function saveScenario() {
      try {
        const res = await fetch('/api/save_scenario', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ map: currentMap, obstacles: obstacles })
        });
        const out = await res.json();
        showToast(out.message || 'Scenario saved successfully!');
      } catch (e) {
        showToast('Error saving: ' + e);
      }
    }

    function updateHudCount() {
      document.getElementById('hudCount').innerText = obstacles.length;
    }

    let liveEgo = null;
    async function pollLiveStatus() {
      try {
        const res = await fetch(`/api/live_status?map=${encodeURIComponent(currentMap)}`);
        const data = await res.json();
        if (data.ego) {
          liveEgo = data.ego;
          document.getElementById('hudEgoCard').style.display = 'flex';
          document.getElementById('hudEgoSpeed').innerText = `${liveEgo.vs.toFixed(2)} m/s`;
        }
        if (data.obstacles && !isDraggingObs) {
          data.obstacles.forEach(remote => {
            const local = obstacles.find(o => o.id === remote.id);
            if (local) {
              local.is_static = remote.is_static;
              local.size = remote.size;
              local.label = remote.label;
              if (!local.is_static) {
                local.s = remote.s;
                local.x = remote.x;
                local.y = remote.y;
                local.yaw = remote.yaw;
                local.vs = remote.vs;
              }
            } else {
              obstacles.push(remote);
              updateHudCount();
            }
          });
          const remoteIds = new Set(data.obstacles.map(o => o.id));
          obstacles = obstacles.filter(o => remoteIds.has(o.id));
          render();

          if (selectedObsId) {
            const sel = obstacles.find(o => o.id === selectedObsId);
            if (sel && !sel.is_static) {
              const sInput = document.getElementById('propFrenetS');
              if (sInput && document.activeElement !== sInput) {
                sInput.value = sel.s.toFixed(2);
              }
            }
          }
        }
      } catch (e) {}
    }

    function render() {
      const cW = canvas.width / (window.devicePixelRatio || 1);
      const cH = canvas.height / (window.devicePixelRatio || 1);
      ctx.clearRect(0, 0, cW, cH);

      // 1. Draw Occupancy Grid Image
      if (mapImg && mapData && mapData.map_meta) {
        const meta = mapData.map_meta;
        const originX = meta.origin[0];
        const originY = meta.origin[1];
        const wM = meta.width_m;
        const hM = meta.height_m;
        const scPos = worldToScreen(originX, originY + hM);
        ctx.save();
        ctx.globalAlpha = 0.55;
        ctx.drawImage(mapImg, scPos.x, scPos.y, wM * viewScale, hM * viewScale);
        ctx.restore();
      }

      // 2. Draw Raceline Waypoints
      if (mapData && mapData.waypoints && mapData.waypoints.length > 0) {
        const wp = mapData.waypoints;
        ctx.beginPath();
        const p0 = worldToScreen(wp[0].x, wp[0].y);
        ctx.moveTo(p0.x, p0.y);
        for (let i = 1; i < wp.length; i++) {
          const pt = worldToScreen(wp[i].x, wp[i].y);
          ctx.lineTo(pt.x, pt.y);
        }
        ctx.closePath();
        ctx.strokeStyle = '#06b6d4';
        ctx.lineWidth = 2.5;
        ctx.stroke();

        // Direction arrows / waypoint markers
        for (let i = 0; i < wp.length; i += 10) {
          const pt = worldToScreen(wp[i].x, wp[i].y);
          ctx.beginPath();
          ctx.arc(pt.x, pt.y, 2, 0, Math.PI * 2);
          ctx.fillStyle = '#22d3ee';
          ctx.fill();
        }
      }

      // 3. Draw Placed Obstacles
      obstacles.forEach(o => {
        const sc = worldToScreen(o.x, o.y);
        const szPx = (o.size || 0.40) * viewScale;
        const isSel = o.id === selectedObsId;

        ctx.save();
        ctx.translate(sc.x, sc.y);

        if (o.is_static) {
          // Static box
          ctx.fillStyle = isSel ? '#fb923c' : '#f97316';
          ctx.strokeStyle = isSel ? '#ffffff' : '#c2410c';
          ctx.lineWidth = isSel ? 2.5 : 1.5;
          ctx.fillRect(-szPx / 2, -szPx / 2, szPx, szPx);
          ctx.strokeRect(-szPx / 2, -szPx / 2, szPx, szPx);
        } else {
          // Dynamic car marker (oriented car chassis with nose arrow)
          ctx.fillStyle = isSel ? '#f87171' : '#ef4444';
          ctx.strokeStyle = isSel ? '#ffffff' : '#b91c1c';
          ctx.lineWidth = isSel ? 2.5 : 1.5;
          if (o.yaw !== undefined) {
            ctx.rotate(-o.yaw);
          }
          ctx.fillRect(-szPx * 0.75, -szPx * 0.4, szPx * 1.5, szPx * 0.8);
          ctx.strokeRect(-szPx * 0.75, -szPx * 0.4, szPx * 1.5, szPx * 0.8);

          // Arrow pointing forward (+x in local frame)
          ctx.fillStyle = '#ffffff';
          ctx.beginPath();
          ctx.moveTo(szPx * 0.75, 0);
          ctx.lineTo(szPx * 0.35, -szPx * 0.25);
          ctx.lineTo(szPx * 0.35, szPx * 0.25);
          ctx.closePath();
          ctx.fill();
        }

        // Label badge (drawn unrotated for readability)
        if (!o.is_static && o.yaw !== undefined) {
          ctx.rotate(o.yaw); // un-rotate for text
        }
        ctx.fillStyle = '#ffffff';
        ctx.font = '10px var(--font-mono)';
        ctx.textAlign = 'center';
        ctx.fillText(`#${o.id} (s=${o.s.toFixed(1)}m)`, 0, -szPx / 2 - 5);

        ctx.restore();
      });

      // 4. Draw Live Ego Vehicle (if ROS active)
      if (liveEgo) {
        const sc = worldToScreen(liveEgo.x, liveEgo.y);
        ctx.save();
        ctx.translate(sc.x, sc.y);
        ctx.rotate(-liveEgo.yaw); // invert Y
        ctx.fillStyle = '#10b981';
        ctx.beginPath();
        ctx.moveTo(12, 0);
        ctx.lineTo(-8, -6);
        ctx.lineTo(-8, 6);
        ctx.closePath();
        ctx.fill();
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 1.5;
        ctx.stroke();
        ctx.restore();
      }
    }

    function renderSidebar() {
      const container = document.getElementById('sidebarContent');
      if (obstacles.length === 0) {
        container.innerHTML = `
          <div style="color:var(--text-dim); font-size:12px; text-align:center; padding:30px 10px;">
            No obstacles in scenario.<br><br>
            Click <strong style="color:var(--text-main);">+ Static</strong> or <strong style="color:var(--text-main);">+ Dynamic</strong> above and click anywhere on the track to drop an obstacle!
          </div>
        `;
        return;
      }

      let html = '';
      const selected = obstacles.find(o => o.id === selectedObsId);

      if (selected) {
        html += `
          <div class="card" style="border-color:var(--accent-orange);">
            <div class="card-title">
              <span>Selected Obstacle #${selected.id}</span>
              <button class="btn btn-danger" style="padding:2px 6px; font-size:10px;" onclick="deleteObstacle(${selected.id})">Delete</button>
            </div>
            <div class="param-row">
              <span class="param-label">Type:</span>
              <select class="select-map" style="padding:2px 6px; font-size:11px;" onchange="updateSelectedType(this.value)">
                <option value="static" ${selected.is_static ? 'selected' : ''}>Static Obstacle</option>
                <option value="dynamic" ${!selected.is_static ? 'selected' : ''}>Dynamic Opponent</option>
              </select>
            </div>
            <div class="param-row">
              <span class="param-label">Frenet s [m]:</span>
              <input id="propFrenetS" type="number" step="0.1" value="${selected.s.toFixed(2)}" class="param-input" onchange="updateSelectedFrenet('s', this.value)">
            </div>
            <div class="param-row">
              <span class="param-label">Frenet d [m]:</span>
              <div style="display:flex; gap:4px; align-items:center;">
                <input id="propFrenetD" type="number" step="0.05" value="${selected.d.toFixed(2)}" class="param-input" style="width:70px;" onchange="updateSelectedFrenet('d', this.value)">
                <button class="btn btn-secondary" style="padding:2px 6px; font-size:10px;" onclick="snapSelectedToCenter()">Center (d=0)</button>
              </div>
            </div>
            <div class="param-row">
              <span class="param-label">Size [m]:</span>
              <input type="number" step="0.05" min="0.1" value="${selected.size}" class="param-input" onchange="updateSelectedProp('size', this.value)">
            </div>
            ${!selected.is_static ? `
              <div class="param-row">
                <span class="param-label">Speed vs [m/s]:</span>
                <input type="number" step="0.1" min="0" value="${selected.vs}" class="param-input" onchange="updateSelectedProp('vs', this.value)">
              </div>
            ` : ''}
          </div>
        `;
      }

      html += `<div class="card-title" style="margin-top:8px;">Active Scenario List (${obstacles.length})</div>`;
      obstacles.forEach(o => {
        const isSel = o.id === selectedObsId;
        html += `
          <div class="obs-item ${isSel ? 'selected' : ''}" onclick="selectObstacle(${o.id})">
            <div class="obs-header">
              <div class="obs-tag">
                <span class="dot ${o.is_static ? 'dot-static' : 'dot-dynamic'}"></span>
                <span>#${o.id} ${o.is_static ? 'Static' : 'Dynamic'}</span>
              </div>
              <span style="font-family:var(--font-mono); font-size:11px; color:var(--text-muted);">s=${o.s.toFixed(1)}m</span>
            </div>
            <div style="font-size:11px; color:var(--text-dim);">
              d=${o.d > 0 ? '+' : ''}${o.d.toFixed(2)}m | size=${o.size}m ${!o.is_static ? `| v=${o.vs}m/s` : ''}
            </div>
          </div>
        `;
      });

      container.innerHTML = html;
    }

    function selectObstacle(id) {
      selectedObsId = id;
      renderSidebar();
      render();
    }

    function updateSelectedType(val) {
      const o = obstacles.find(x => x.id === selectedObsId);
      if (!o) return;
      o.is_static = (val === 'static');
      if (o.is_static) {
        o.vs = 0.0;
        o.label = 'Static Obstacle';
      } else {
        if (!o.vs || o.vs === 0.0) o.vs = 1.20;
        o.label = 'Dynamic Opponent';
      }
      const xy = frenetToCartesian(o.s, o.d);
      o.x = xy.x;
      o.y = xy.y;
      renderSidebar();
      render();
      syncObstacleUpdate(o);
      showToast(`Set #${o.id} to ${o.label}`);
    }

    function updateSelectedProp(key, val) {
      const o = obstacles.find(x => x.id === selectedObsId);
      if (!o) return;
      o[key] = parseFloat(val);
      render();
      syncObstacleUpdate(o);
    }

    function updateSelectedFrenet(key, val) {
      const o = obstacles.find(x => x.id === selectedObsId);
      if (!o) return;
      o[key] = parseFloat(val);
      const xy = frenetToCartesian(o.s, o.d);
      o.x = xy.x;
      o.y = xy.y;
      render();
      syncObstacleUpdate(o);
    }

    function snapSelectedToCenter() {
      const o = obstacles.find(x => x.id === selectedObsId);
      if (!o) return;
      o.d = 0.0;
      const xy = frenetToCartesian(o.s, 0.0);
      o.x = xy.x;
      o.y = xy.y;
      const dInput = document.getElementById('propFrenetD');
      if (dInput) dInput.value = "0.00";
      render();
      syncObstacleUpdate(o);
      showToast(`Snapped #${o.id} to Raceline Center (d=0)`);
    }

    function showToast(msg) {
      const t = document.getElementById('toast');
      t.innerText = msg;
      t.classList.add('show');
      setTimeout(() => t.classList.remove('show'), 2800);
    }
  </script>
</body>
</html>
"""

# -----------------------------------------------------------------------------
# HTTP Request Handler
# -----------------------------------------------------------------------------
class SimStudioHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def _send_json(self, data, status=200):
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode('utf-8'))

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        qs = parse_qs(parsed.query)

        if path in ['/', '/index.html']:
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode('utf-8'))
            return

        elif path == '/api/maps':
            self._send_json(list_available_maps())
            return

        elif path == '/api/map_data':
            m = qs.get('map', [''])[0]
            if not m:
                self._send_json({'error': 'Missing map query param'}, 400)
                return
            STORE.current_map = m
            self._send_json(load_map_data(m))
            return

        elif path == '/api/map_image':
            m = qs.get('map', [''])[0]
            if not m:
                self.send_response(400)
                self.end_headers()
                return
            meta = get_map_metadata(m)
            if not meta.get('has_image') or not meta.get('img_path'):
                self.send_response(404)
                self.end_headers()
                return
            img_path = meta['img_path']
            if img_path.endswith('.png'):
                with open(img_path, 'rb') as f:
                    content = f.read()
                self.send_response(200)
                self.send_header('Content-Type', 'image/png')
                self.send_header('Content-Length', str(len(content)))
                self.end_headers()
                self.wfile.write(content)
                return
            elif img_path.endswith('.pgm'):
                # Try PIL convert
                try:
                    from PIL import Image
                    im = Image.open(img_path)
                    buf = io.BytesIO()
                    im.save(buf, format='PNG')
                    content = buf.getvalue()
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/png')
                    self.end_headers()
                    self.wfile.write(content)
                    return
                except Exception:
                    self.send_response(404)
                    self.end_headers()
                    return

        elif path == '/api/active_ros_map':
            self._send_json({'active_map': detect_active_ros_map()})
            return

        elif path == '/api/obstacles':
            m = qs.get('map', [STORE.current_map])[0]
            self._send_json(STORE.get_obstacles(m))
            return

        elif path == '/api/live_status':
            m = qs.get('map', [STORE.current_map])[0]
            if m and m != STORE.current_map and m in STORE.map_tracks:
                STORE.current_map = m
            self._send_json({
                'active_map': STORE.current_map,
                'ego': STORE.get_ego_pose(),
                'obstacles': STORE.get_obstacles(m)
            })
            return

        self.send_response(404)
        self.end_headers()

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path

        length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(length) if length > 0 else b'{}'
        try:
            payload = json.loads(body.decode('utf-8'))
        except Exception:
            payload = {}

        if path == '/api/obstacles':
            action = payload.get('action')
            m = payload.get('map', STORE.current_map)
            if action == 'add':
                obs = payload.get('obstacle', {})
                STORE.add_obstacle(obs, map_name=m)
                self._send_json({'success': True, 'obstacle': obs})
            elif action == 'update':
                obs_id = payload.get('id')
                updates = payload.get('updates', {})
                res = STORE.update_obstacle(obs_id, updates, map_name=m)
                self._send_json({'success': res is not None})
            elif action == 'delete':
                obs_id = payload.get('id')
                STORE.delete_obstacle(obs_id, map_name=m)
                self._send_json({'success': True})
            elif action == 'clear':
                STORE.clear(map_name=m)
                self._send_json({'success': True})
            else:
                self._send_json({'error': 'Unknown action'}, 400)
            return

        elif path == '/api/save_scenario':
            map_name = payload.get('map', STORE.current_map)
            obs_list = payload.get('obstacles', [])
            STORE.set_obstacles(obs_list, map_name=map_name)

            # Persist to JSON under map directory
            out_dir = MAPS_ROOT / map_name / "waypoints"
            out_dir.mkdir(parents=True, exist_ok=True)
            out_file = out_dir / "sim_scenario.json"
            try:
                with open(out_file, 'w') as f:
                    json.dump({'map': map_name, 'timestamp': time.time(), 'obstacles': obs_list}, f, indent=2)
                self._send_json({'success': True, 'message': f'Saved {len(obs_list)} obstacles to {out_file.name}'})
            except Exception as e:
                self._send_json({'error': str(e)}, 500)
            return

        self.send_response(404)
        self.end_headers()

# -----------------------------------------------------------------------------
# Main Entry Point
# -----------------------------------------------------------------------------
def main():
    detected_map = detect_active_ros_map()
    parser = argparse.ArgumentParser(description="F1TENTH Web Sim Obstacle Studio")
    parser.add_argument("--port", type=int, default=8052, help="HTTP Server port (default: 8052)")
    parser.add_argument("--map", type=str, default="", help="Default map to load (auto-detected if omitted)")
    args = parser.parse_args()

    active_map = args.map if args.map else detected_map
    STORE.current_map = active_map

    # Preload map waypoints into store
    if STORE.current_map:
        try:
            init_data = load_map_data(STORE.current_map)
            if 'waypoints' in init_data:
                STORE.load_track(STORE.current_map, init_data['waypoints'], init_data['track_length'])
        except Exception:
            pass

    # Start motion propagation thread (25 Hz)
    def _motion_ticker():
        last_t = time.perf_counter()
        while True:
            time.sleep(0.04)
            now = time.perf_counter()
            dt = now - last_t
            last_t = now
            STORE.step(dt)

    motion_thread = threading.Thread(target=_motion_ticker, daemon=True)
    motion_thread.start()

    # Start ROS 2 Bridge if available
    ros_bridge = SimObstacleRosNode(STORE)
    ros_bridge.start()

    server = HTTPServer(('0.0.0.0', args.port), SimStudioHandler)
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    try:
        from common.network_info import print_service_banner
        print_service_banner(
            title="🏎️  F1TENTH Web Sim Obstacle Studio",
            subtitle="Interactive Obstacle & Dynamic Opponent Placement",
            port=args.port,
            query=f"map={args.map}" if args.map else ""
        )
    except Exception:
        print(f"\n=======================================================")
        print(f"  🏎️  F1TENTH Web Sim Obstacle Studio")
        print(f"  → Local URL: http://localhost:{args.port}")
        print(f"=======================================================\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down Sim Obstacle Studio...")
        server.server_close()

if __name__ == '__main__':
    main()
