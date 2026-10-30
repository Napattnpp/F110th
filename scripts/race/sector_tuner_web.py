#!/usr/bin/env python3
"""
F1TENTH Web-Based Sector Tuner & Visualizer
--------------------------------------------
A modern, zero-dependency browser-based visualizer and editor for:
  1. Speed Scaling Sectors  (assets/maps/<map>/waypoints/speed_scaling.yaml)
  2. Overtaking Sectors     (assets/maps/<map>/waypoints/ot_sectors.yaml)

Features:
- Zero X11 / XQuartz / Display forwarding required.
- Full Map Display: Renders the occupancy grid map (walls, obstacles, free space) with dark-mode inversion & opacity.
- Interactive 2D track canvas with 60fps pan, zoom, click-to-split, and drag-to-adjust.
- Color-coded velocity and overtaking heatmaps.
- Real-time estimated lap time calculations based on sector scaling.
- 1-click persistent save to standard YAML files.
"""

import os
import sys
import json
import re
import io
import argparse
from pathlib import Path
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

# Locate project repository root
def find_repo_root():
    curr = Path(__file__).resolve().parent
    for p in [curr] + list(curr.parents):
        if (p / 'assets' / 'maps').is_dir() or (p / 'docker-compose.yml').is_file():
            return p
    return curr.parent.parent

REPO_ROOT = find_repo_root()
MAPS_ROOT = REPO_ROOT / "assets" / "maps"

# -----------------------------------------------------------------------------
# YAML Helper (Supports PyYAML if installed, with lossless builtin fallback)
# -----------------------------------------------------------------------------
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

    # Built-in lightweight fallback parser for F1TENTH sector & map YAMLs
    try:
        with open(fpath, 'r') as f:
            lines = f.readlines()
        data = {}
        stack = [(0, data)]
        last_key = None
        for line in lines:
            line_str = line.rstrip()
            if not line_str or line_str.lstrip().startswith('#'):
                continue
            indent = len(line_str) - len(line_str.lstrip())

            # Handle list items (e.g. origin: [-2.5, -2.4, 0.0] or - -2.5)
            stripped = line_str.strip()
            if stripped.startswith('- '):
                val_str = stripped[2:].strip()
                try:
                    val = float(val_str) if '.' in val_str or 'e' in val_str.lower() else int(val_str)
                except ValueError:
                    val = val_str

                while stack and indent < stack[-1][0]:
                    stack.pop()

                parent = stack[-1][1]
                if isinstance(parent, list):
                    parent.append(val)
                elif isinstance(parent, dict):
                    # Convert empty placeholder dict to list for block sequences
                    if len(parent) == 0 and len(stack) > 1 and last_key:
                        new_list = [val]
                        gp = stack[-2][1]
                        if isinstance(gp, dict):
                            gp[last_key] = new_list
                        stack[-1] = (stack[-1][0], new_list)
                continue

            key_val = stripped.split(':', 1)
            key = key_val[0].strip()
            last_key = key
            val_str = key_val[1].strip() if len(key_val) > 1 else ""

            while stack and indent <= stack[-1][0] and len(stack) > 1:
                stack.pop()
            parent = stack[-1][1]

            if not val_str:
                new_dict = {}
                if isinstance(parent, dict):
                    parent[key] = new_dict
                stack.append((indent, new_dict))
            else:
                if val_str.startswith('[') and val_str.endswith(']'):
                    raw_items = val_str[1:-1].split(',')
                    val = []
                    for it in raw_items:
                        it = it.strip()
                        try:
                            val.append(float(it) if '.' in it or 'e' in it.lower() else int(it))
                        except ValueError:
                            val.append(it)
                elif val_str.lower() in ['true', 'yes']:
                    val = True
                elif val_str.lower() in ['false', 'no']:
                    val = False
                else:
                    try:
                        val = float(val_str) if '.' in val_str or 'e' in val_str.lower() else int(val_str)
                    except ValueError:
                        val = val_str
                if isinstance(parent, dict):
                    parent[key] = val
        return data
    except Exception as e:
        print(f"[WARN] Fallback parser failed for {fpath}: {e}")
        return None

def dump_speed_scaling_yaml(global_limit, sectors):
    """Generate speed_scaling.yaml text exactly adhering to ROS 2 parameters."""
    n_sectors = len(sectors)
    lines = [
        "sector_tuner:",
        "  ros__parameters:",
        f"    global_limit: {float(global_limit):.4f}",
        f"    n_sectors: {n_sectors}"
    ]
    for idx, s in enumerate(sectors):
        lines.append(f"    Sector{idx}:")
        lines.append(f"      start: {int(s['start'])}")
        lines.append(f"      end: {int(s['end'])}")
        lines.append(f"      scaling: {float(s.get('scaling', 1.0)):.4f}")
        only_ftg = str(bool(s.get('only_FTG', False))).lower()
        no_ftg = str(bool(s.get('no_FTG', False))).lower()
        lines.append(f"      only_FTG: {only_ftg}")
        lines.append(f"      no_FTG: {no_ftg}")
    lines.append("")
    return "\n".join(lines)

def dump_ot_sectors_yaml(yeet_factor, spline_len, ot_sector_begin, sectors):
    """Generate ot_sectors.yaml text exactly adhering to ROS 2 parameters."""
    n_sectors = len(sectors)
    lines = [
        "ot_interpolator:",
        "  ros__parameters:",
        f"    n_sectors: {n_sectors}",
        f"    yeet_factor: {float(yeet_factor):.4f}",
        f"    spline_len: {int(spline_len)}",
        f"    ot_sector_begin: {float(ot_sector_begin):.4f}"
    ]
    for idx, s in enumerate(sectors):
        lines.append(f"    Overtaking_sector{idx}:")
        lines.append(f"      start: {int(s['start'])}")
        lines.append(f"      end: {int(s['end'])}")
        ot_flag = str(bool(s.get('ot_flag', False))).lower()
        lines.append(f"      ot_flag: {ot_flag}")
    lines.append("")
    return "\n".join(lines)

# -----------------------------------------------------------------------------
# Data Loaders & Map Metadata
# -----------------------------------------------------------------------------
def list_available_maps():
    """Discover all maps containing global_waypoints.json under assets/maps/"""
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
    """Resolves map YAML and image paths (clean/, waypoint_gen/, raw/)."""
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
        # Flatten any nested lists if parsed as [[-2.5, ...]]
        flat_origin = []
        for it in origin:
            if isinstance(it, list):
                flat_origin.extend(it)
            else:
                flat_origin.append(float(it))
        origin = flat_origin
    else:
        origin = [0.0, 0.0, 0.0]

    img_name = ydata.get("image", f"{map_name}.png")
    img_dir = ypath.parent
    img_path = img_dir / img_name
    if not img_path.exists():
        base = img_path.stem
        for ext in [".png", ".pgm", ".jpg"]:
            cand = img_dir / f"{base}{ext}"
            if cand.exists():
                img_path = cand
                break

    if not img_path.exists():
        return {"has_image": False}

    width_px = 0
    height_px = 0
    try:
        from PIL import Image
        with Image.open(img_path) as im:
            width_px, height_px = im.size
    except Exception:
        try:
            with open(img_path, 'rb') as f:
                header = f.readline().decode('latin1').strip()
                if header in ['P5', 'P2']:
                    line = f.readline().decode('latin1').strip()
                    while line.startswith('#'):
                        line = f.readline().decode('latin1').strip()
                    parts = line.split()
                    width_px = int(parts[0])
                    height_px = int(parts[1])
        except Exception:
            pass

    return {
        "has_image": True,
        "image_url": f"/api/map_image?map={map_rel}",
        "resolution": resolution,
        "origin": origin,
        "width_px": width_px,
        "height_px": height_px,
        "width_m": width_px * resolution,
        "height_m": height_px * resolution,
        "img_path": str(img_path)
    }

def load_map_data(map_rel):
    map_dir = MAPS_ROOT / map_rel
    wp_dir = map_dir / "waypoints"
    json_path = wp_dir / "global_waypoints.json"
    if not json_path.exists():
        json_path = map_dir / "global_waypoints.json"

    if not json_path.exists():
        return {"error": f"global_waypoints.json not found for map: {map_rel}"}

    with open(json_path, 'r') as f:
        wp_data = json.load(f)

    # 1. Trajectory points (prioritize IQP raceline, fallback to SP or centerline)
    traj_key = 'global_traj_wpnts_iqp'
    if traj_key not in wp_data or not wp_data[traj_key].get('wpnts'):
        traj_key = 'global_traj_wpnts_sp'
    if traj_key not in wp_data or not wp_data[traj_key].get('wpnts'):
        traj_key = 'centerline_waypoints'

    raceline = wp_data.get(traj_key, {}).get('wpnts', [])
    centerline = wp_data.get('centerline_waypoints', {}).get('wpnts', [])
    sp_line = wp_data.get('global_traj_wpnts_sp', {}).get('wpnts', [])

    # 2. Existing speed_scaling.yaml
    speed_yaml_path = wp_dir / "speed_scaling.yaml"
    speed_cfg = parse_yaml_file(speed_yaml_path)
    speed_sectors = []
    global_limit = 1.0

    if speed_cfg and 'sector_tuner' in speed_cfg and 'ros__parameters' in speed_cfg['sector_tuner']:
        params = speed_cfg['sector_tuner']['ros__parameters']
        global_limit = float(params.get('global_limit', 1.0))
        n_sec = int(params.get('n_sectors', 1))
        for i in range(n_sec):
            sec_key = f"Sector{i}"
            if sec_key in params:
                s_obj = params[sec_key]
                speed_sectors.append({
                    "id": i,
                    "start": int(s_obj.get('start', 0)),
                    "end": int(s_obj.get('end', len(raceline))),
                    "scaling": float(s_obj.get('scaling', 1.0)),
                    "only_FTG": bool(s_obj.get('only_FTG', False)),
                    "no_FTG": bool(s_obj.get('no_FTG', False))
                })

    if speed_sectors and raceline:
        if len(speed_sectors) == 1:
            speed_sectors[0]["start"] = 0
            speed_sectors[0]["end"] = len(raceline)
        elif speed_sectors[-1]["end"] != len(raceline):
            speed_sectors[-1]["end"] = len(raceline)
    elif not speed_sectors and raceline:
        speed_sectors = [{
            "id": 0,
            "start": 0,
            "end": len(raceline),
            "scaling": 1.0,
            "only_FTG": False,
            "no_FTG": False
        }]

    # 3. Existing ot_sectors.yaml
    ot_yaml_path = wp_dir / "ot_sectors.yaml"
    ot_cfg = parse_yaml_file(ot_yaml_path)
    ot_sectors = []
    yeet_factor = 1.25
    spline_len = 30
    ot_sector_begin = 0.5

    if ot_cfg and 'ot_interpolator' in ot_cfg and 'ros__parameters' in ot_cfg['ot_interpolator']:
        ot_params = ot_cfg['ot_interpolator']['ros__parameters']
        yeet_factor = float(ot_params.get('yeet_factor', 1.25))
        spline_len = int(ot_params.get('spline_len', 30))
        ot_sector_begin = float(ot_params.get('ot_sector_begin', 0.5))
        n_ot_sec = int(ot_params.get('n_sectors', 1))
        for i in range(n_ot_sec):
            ot_key = f"Overtaking_sector{i}"
            if ot_key in ot_params:
                ot_obj = ot_params[ot_key]
                ot_sectors.append({
                    "id": i,
                    "start": int(ot_obj.get('start', 0)),
                    "end": int(ot_obj.get('end', len(raceline))),
                    "ot_flag": bool(ot_obj.get('ot_flag', False))
                })

    if ot_sectors and raceline:
        if len(ot_sectors) == 1:
            ot_sectors[0]["start"] = 0
            ot_sectors[0]["end"] = len(raceline)
        elif ot_sectors[-1]["end"] != len(raceline):
            ot_sectors[-1]["end"] = len(raceline)
    elif not ot_sectors and raceline:
        ot_sectors = [{
            "id": 0,
            "start": 0,
            "end": len(raceline),
            "ot_flag": False
        }]

    # 4. Map Metadata & Image Resolution
    meta = get_map_metadata(map_rel)

    return {
        "map_name": map_rel,
        "raceline_key": traj_key,
        "raceline": raceline,
        "centerline": centerline,
        "shortest_path": sp_line,
        "map_meta": meta,
        "speed_config": {
            "global_limit": global_limit,
            "sectors": speed_sectors
        },
        "ot_config": {
            "yeet_factor": yeet_factor,
            "spline_len": spline_len,
            "ot_sector_begin": ot_sector_begin,
            "sectors": ot_sectors
        }
    }

# -----------------------------------------------------------------------------
# Embedded Web Interface
# -----------------------------------------------------------------------------
HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>F1TENTH Sector Tuner & Visualizer</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
  <link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;600;700&family=Outfit:wght@400;500;600;700&display=swap" rel="stylesheet">
  <style>
    :root {
      --bg-dark: #0a0e17;
      --card-bg: #121824;
      --panel-bg: #0f1521;
      --border-color: #1e283d;
      --border-active: #2b3956;
      --primary-cyan: #00e5ff;
      --primary-emerald: #00e676;
      --accent-amber: #ffb300;
      --accent-red: #ff3d71;
      --text-main: #f0f4fc;
      --text-muted: #8b9bb4;
      --radius: 8px;
    }

    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: 'Outfit', -apple-system, BlinkMacSystemFont, sans-serif;
      background-color: var(--bg-dark);
      color: var(--text-main);
      display: flex;
      flex-direction: column;
      height: 100vh;
      overflow: hidden;
      user-select: none;
    }

    input, select, button, label {
      user-select: auto;
    }

    header {
      background: var(--panel-bg);
      border-bottom: 1px solid var(--border-color);
      height: 60px;
      padding: 0 20px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      z-index: 10;
    }

    .brand {
      display: flex;
      align-items: center;
      gap: 12px;
      font-weight: 700;
      font-size: 1.15rem;
      letter-spacing: -0.02em;
    }
    .brand-badge {
      background: rgba(0, 229, 255, 0.12);
      color: var(--primary-cyan);
      border: 1px solid rgba(0, 229, 255, 0.3);
      padding: 2px 8px;
      border-radius: 4px;
      font-size: 0.75rem;
      font-family: 'JetBrains Mono', monospace;
    }

    .header-controls {
      display: flex;
      align-items: center;
      gap: 16px;
    }

    select, button, input[type="number"], input[type="text"] {
      font-family: inherit;
      font-size: 0.85rem;
      border-radius: var(--radius);
      outline: none;
    }

    .select-map {
      background: var(--card-bg);
      color: var(--text-main);
      border: 1px solid var(--border-color);
      padding: 7px 12px;
      cursor: pointer;
      min-width: 220px;
      font-weight: 500;
    }

    .tab-switch {
      display: flex;
      background: var(--card-bg);
      border: 1px solid var(--border-color);
      border-radius: var(--radius);
      padding: 3px;
      gap: 4px;
    }
    .tab-btn {
      background: transparent;
      border: none;
      color: var(--text-muted);
      padding: 5px 14px;
      border-radius: 6px;
      cursor: pointer;
      font-weight: 600;
      display: flex;
      align-items: center;
      gap: 6px;
      transition: all 0.15s ease;
    }
    .tab-btn.active {
      background: var(--primary-cyan);
      color: #050811;
      box-shadow: 0 0 12px rgba(0, 229, 255, 0.4);
    }
    .tab-btn.active.ot-mode {
      background: var(--primary-emerald);
      box-shadow: 0 0 12px rgba(0, 230, 118, 0.4);
    }

    .btn {
      padding: 8px 16px;
      border: none;
      cursor: pointer;
      font-weight: 600;
      display: flex;
      align-items: center;
      gap: 6px;
      transition: transform 0.1s, opacity 0.15s;
    }
    .btn:active { transform: scale(0.98); }
    .btn-save {
      background: var(--primary-emerald);
      color: #050811;
      box-shadow: 0 0 14px rgba(0, 230, 118, 0.35);
    }
    .btn-save:hover { filter: brightness(1.1); }
    .btn-secondary {
      background: var(--card-bg);
      color: var(--text-main);
      border: 1px solid var(--border-color);
    }
    .btn-secondary:hover { border-color: var(--border-active); }

    /* Main Container */
    .app-body {
      display: flex;
      flex: 1;
      height: calc(100vh - 60px);
      position: relative;
    }

    /* Canvas Section */
    .canvas-container {
      flex: 1;
      position: relative;
      background: radial-gradient(circle at center, #111726 0%, #080b12 100%);
      overflow: hidden;
      cursor: crosshair;
    }
    canvas {
      position: absolute;
      top: 0;
      left: 0;
      width: 100%;
      height: 100%;
    }

    .canvas-hud {
      position: absolute;
      top: 16px;
      left: 16px;
      background: rgba(15, 21, 33, 0.88);
      backdrop-filter: blur(10px);
      border: 1px solid var(--border-color);
      border-radius: var(--radius);
      padding: 10px 14px;
      display: flex;
      gap: 16px;
      font-size: 0.8rem;
      font-family: 'JetBrains Mono', monospace;
      pointer-events: auto;
      user-select: none;
      cursor: default;
      z-index: 10;
      box-shadow: 0 4px 16px rgba(0, 0, 0, 0.4);
    }
    .hud-metric { display: flex; flex-direction: column; gap: 2px; }
    .hud-label { color: var(--text-muted); font-size: 0.7rem; }
    .hud-val { color: var(--primary-cyan); font-weight: 700; font-size: 0.95rem; }

    .canvas-toolbar {
      position: absolute;
      bottom: 20px;
      left: 20px;
      background: rgba(15, 21, 33, 0.92);
      backdrop-filter: blur(12px);
      border: 1px solid var(--border-color);
      border-radius: var(--radius);
      display: flex;
      flex-wrap: wrap;
      align-items: center;
      padding: 6px 12px;
      gap: 10px;
      z-index: 10;
      box-shadow: 0 4px 20px rgba(0, 0, 0, 0.5);
      max-width: calc(100% - 40px);
      user-select: none;
    }
    .tool-btn {
      background: transparent;
      border: 1px solid var(--border-color);
      color: var(--text-main);
      padding: 6px 12px;
      border-radius: 4px;
      cursor: pointer;
      font-size: 0.75rem;
      font-weight: 600;
      transition: background 0.15s, border-color 0.15s;
    }
    .tool-btn:hover { background: rgba(255, 255, 255, 0.08); border-color: var(--border-active); }
    .tool-btn.active {
      background: rgba(0, 229, 255, 0.15);
      border-color: var(--primary-cyan);
      color: var(--primary-cyan);
    }

    .tool-slider-wrap {
      display: flex;
      align-items: center;
      gap: 6px;
      font-size: 0.75rem;
      color: var(--text-muted);
      border-left: 1px solid var(--border-color);
      padding-left: 10px;
    }
    .tool-slider-wrap input[type="range"] {
      width: 75px;
      accent-color: var(--primary-cyan);
      cursor: pointer;
    }

    /* Side Panel */
    .sidebar {
      width: 410px;
      background: var(--panel-bg);
      border-left: 1px solid var(--border-color);
      display: flex;
      flex-direction: column;
      z-index: 5;
    }

    .sidebar-header {
      padding: 16px 20px;
      border-bottom: 1px solid var(--border-color);
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    .sidebar-title {
      font-weight: 700;
      font-size: 0.95rem;
      display: flex;
      align-items: center;
      gap: 8px;
    }

    .sector-list {
      flex: 1;
      overflow-y: auto;
      padding: 14px 18px;
      display: flex;
      flex-direction: column;
      gap: 12px;
    }

    .global-params-card {
      background: var(--card-bg);
      border: 1px solid var(--border-color);
      border-radius: var(--radius);
      padding: 14px 16px;
      margin: 14px 18px 0;
      display: flex;
      flex-direction: column;
      gap: 10px;
    }
    .param-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      font-size: 0.85rem;
    }
    .param-label { color: var(--text-muted); font-size: 0.8rem; }
    .param-input-wrap { display: flex; align-items: center; gap: 8px; }
    .param-input {
      background: #0a0e17;
      border: 1px solid var(--border-color);
      color: var(--text-main);
      padding: 4px 8px;
      width: 70px;
      text-align: right;
      font-family: 'JetBrains Mono', monospace;
    }

    /* Sector Card */
    .sector-card {
      background: var(--card-bg);
      border: 1px solid var(--border-color);
      border-radius: var(--radius);
      padding: 12px 14px;
      display: flex;
      flex-direction: column;
      gap: 10px;
      transition: border-color 0.15s, box-shadow 0.15s;
    }
    .sector-card:hover, .sector-card.active {
      border-color: var(--border-active);
    }
    .sector-card.active {
      border-color: var(--primary-cyan);
      box-shadow: 0 0 10px rgba(0, 229, 255, 0.15);
    }

    .sector-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    .sector-tag {
      display: flex;
      align-items: center;
      gap: 8px;
      font-weight: 700;
      font-size: 0.85rem;
    }
    .color-dot {
      width: 10px;
      height: 10px;
      border-radius: 50%;
    }
    .sector-range {
      font-family: 'JetBrains Mono', monospace;
      font-size: 0.75rem;
      color: var(--text-muted);
    }

    .slider-row {
      display: flex;
      align-items: center;
      gap: 10px;
    }
    .slider-row input[type="range"] {
      flex: 1;
      accent-color: var(--primary-cyan);
      cursor: pointer;
    }
    .val-badge {
      font-family: 'JetBrains Mono', monospace;
      font-size: 0.85rem;
      font-weight: 700;
      width: 52px;
      text-align: right;
    }

    .flag-switches {
      display: flex;
      gap: 16px;
      font-size: 0.8rem;
      user-select: none;
    }
    .flag-label {
      display: inline-flex;
      align-items: center;
      gap: 7px;
      color: var(--text-muted);
      cursor: pointer;
      user-select: none;
      font-size: 0.78rem;
      font-family: 'JetBrains Mono', monospace;
      padding: 3px 6px;
      border-radius: 4px;
      transition: color 0.15s, background-color 0.15s;
    }
    .flag-label:hover {
      color: var(--text-main);
      background: rgba(255, 255, 255, 0.05);
    }
    .flag-label input[type="checkbox"] {
      width: 16px;
      height: 16px;
      accent-color: var(--primary-cyan);
      cursor: pointer;
      margin: 0;
    }

    .ot-toggle-label {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      cursor: pointer;
      user-select: none;
      font-family: 'JetBrains Mono', monospace;
      font-size: 0.82rem;
      padding: 3px 6px;
      border-radius: 4px;
      transition: background-color 0.15s;
    }
    .ot-toggle-label:hover {
      background: rgba(255, 255, 255, 0.05);
    }
    .ot-toggle-label input[type="checkbox"] {
      width: 16px;
      height: 16px;
      accent-color: var(--primary-emerald);
      cursor: pointer;
      margin: 0;
    }

    .sector-actions {
      display: flex;
      justify-content: flex-end;
      gap: 6px;
      margin-top: 4px;
    }
    .btn-tiny {
      background: rgba(255, 255, 255, 0.04);
      border: 1px solid var(--border-color);
      color: var(--text-main);
      padding: 4px 10px;
      border-radius: 4px;
      cursor: pointer;
      font-size: 0.75rem;
      font-weight: 600;
      transition: background 0.15s, border-color 0.15s, color 0.15s;
    }
    .btn-tiny:hover {
      background: rgba(0, 229, 255, 0.12);
      border-color: var(--primary-cyan);
      color: var(--primary-cyan);
    }
    .btn-tiny.delete:hover {
      background: rgba(255, 61, 113, 0.15);
      border-color: var(--accent-red);
      color: var(--accent-red);
    }

    /* Toast Notification */
    .toast {
      position: fixed;
      bottom: 24px;
      right: 24px;
      background: var(--card-bg);
      border: 1px solid var(--border-color);
      padding: 12px 18px;
      border-radius: var(--radius);
      display: flex;
      align-items: center;
      gap: 10px;
      box-shadow: 0 10px 30px rgba(0, 0, 0, 0.5);
      transform: translateY(100px);
      opacity: 0;
      transition: all 0.25s ease;
      z-index: 100;
      font-size: 0.85rem;
    }
    .toast.show {
      transform: translateY(0);
      opacity: 1;
    }
    .toast.success { border-color: var(--primary-emerald); color: var(--primary-emerald); }
    .toast.error { border-color: var(--accent-red); color: var(--accent-red); }
  </style>
</head>
<body>

  <header>
    <div class="brand">
      <span>🏎️ F1TENTH</span>
      <span class="brand-badge">SECTOR TUNER</span>
    </div>

    <div class="header-controls">
      <select id="mapSelect" class="select-map">
        <option value="">Loading maps...</option>
      </select>

      <div class="tab-switch">
        <button id="tabSpeed" class="tab-btn active" onclick="switchMode('speed')">⚡ Speed Scaling</button>
        <button id="tabOT" class="tab-btn" onclick="switchMode('ot')">🏁 Overtaking</button>
      </div>

      <button class="btn btn-secondary" onclick="reloadMapData()">↺ Reload</button>
      <button class="btn btn-save" onclick="saveConfiguration()">💾 Save to Map</button>
    </div>
  </header>

  <div class="app-body">
    <!-- Visual Track Canvas -->
    <div class="canvas-container" id="canvasContainer">
      <canvas id="trackCanvas"></canvas>

      <div class="canvas-hud">
        <div class="hud-metric">
          <span class="hud-label">TRACK LENGTH</span>
          <span class="hud-val" id="hudLength">-- m</span>
        </div>
        <div class="hud-metric">
          <span class="hud-label">WAYPOINTS</span>
          <span class="hud-val" id="hudWaypoints">--</span>
        </div>
        <div class="hud-metric">
          <span class="hud-label">EST. LAP TIME</span>
          <span class="hud-val" id="hudLapTime">-- s</span>
        </div>
        <div class="hud-metric">
          <span class="hud-label">TIME DELTA</span>
          <span class="hud-val" id="hudTimeDelta" style="color: var(--accent-amber);">0.00s</span>
        </div>
      </div>

      <div class="canvas-toolbar">
        <button class="tool-btn" onclick="resetView()">🎯 Fit View</button>
        <button class="tool-btn" onclick="zoom(1.25)">🔍 +</button>
        <button class="tool-btn" onclick="zoom(0.8)">🔍 -</button>
        <button class="tool-btn active" id="btnMap" onclick="toggleMap()">🗺️ Map: ON</button>
        <button class="tool-btn active" id="btnInvert" onclick="toggleInvertMap()">🌓 Dark Walls</button>
        <div class="tool-slider-wrap">
          <span>Opacity:</span>
          <input type="range" min="0.1" max="1.0" step="0.05" value="0.75" id="sliderMapOpacity" oninput="setMapOpacity(this.value)">
        </div>
        <button class="tool-btn" onclick="toggleCenterline()" id="btnCenterline">Centerline: ON</button>
      </div>
    </div>

    <!-- Sidebar Controls -->
    <div class="sidebar">
      <div class="sidebar-header">
        <div class="sidebar-title" id="panelTitle">⚡ Speed Scaling Configuration</div>
        <button class="btn-tiny" onclick="addSectorAtEnd()" title="Divide the selected or longest sector into two">+ Add Sector</button>
      </div>

      <!-- Mode-specific Global Parameters -->
      <div class="global-params-card" id="globalParamsCard">
        <!-- Rendered dynamically -->
      </div>

      <!-- Sector Cards List -->
      <div class="sector-list" id="sectorList">
        <!-- Sector cards rendered here -->
      </div>
    </div>
  </div>

  <div id="toast" class="toast">Action completed</div>

  <script>
    // State
    let currentMap = '';
    let mapData = null;
    let activeMode = 'speed'; // 'speed' | 'ot'
    let activeSectorId = null;
    let showCenterline = true;

    // Map Underlay Image State
    let showMap = true;
    let invertMap = true;
    let mapOpacity = 0.75;
    let mapImg = new Image();

    // Canvas & View State
    const canvas = document.getElementById('trackCanvas');
    const ctx = canvas.getContext('2d');
    let viewX = 0, viewY = 0, viewScale = 30; // pixels per meter
    let isDragging = false, dragStartX = 0, dragStartY = 0;
    let hoveredWpIdx = -1;

    // Palette generator for sectors
    const PALETTE = [
      '#00e5ff', '#00e676', '#ffb300', '#b388ff', '#ff3d71',
      '#1de9b6', '#ffd600', '#ff6d00', '#7c4dff', '#00b0ff'
    ];

    function showToast(msg, type = 'success') {
      const t = document.getElementById('toast');
      t.innerText = msg;
      t.className = `toast show ${type}`;
      setTimeout(() => { t.className = 'toast'; }, 3000);
    }

    // Initialize application
    async function init() {
      resizeCanvas();
      window.addEventListener('resize', resizeCanvas);
      setupCanvasInteractions();

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

        if (maps.length > 0) {
          sel.value = maps.includes('rectangle1') ? 'rectangle1' : maps[0];
          currentMap = sel.value;
          await loadMap(currentMap);
        }
        sel.addEventListener('change', () => {
          currentMap = sel.value;
          loadMap(currentMap);
        });
      } catch (err) {
        showToast('Failed to load map list: ' + err, 'error');
      }
    }

    async function loadMap(mapName) {
      try {
        const res = await fetch(`/api/map_data?map=${encodeURIComponent(mapName)}`);
        const data = await res.json();
        if (data.error) {
          showToast(data.error, 'error');
          return;
        }
        mapData = data;

        // Load background map image
        if (mapData.map_meta && mapData.map_meta.has_image) {
          mapImg = new Image();
          mapImg.src = mapData.map_meta.image_url;
          mapImg.onload = () => { renderCanvas(); };
        } else {
          mapImg = new Image();
        }

        resetView();
        renderSidebar();
        renderCanvas();
      } catch (err) {
        showToast('Error loading map data: ' + err, 'error');
      }
    }

    function reloadMapData() {
      if (currentMap) loadMap(currentMap);
    }

    function switchMode(mode) {
      activeMode = mode;
      document.getElementById('tabSpeed').className = `tab-btn ${mode === 'speed' ? 'active' : ''}`;
      document.getElementById('tabOT').className = `tab-btn ${mode === 'ot' ? 'active ot-mode' : ''}`;
      document.getElementById('panelTitle').innerText = mode === 'speed' ? '⚡ Speed Scaling Configuration' : '🏁 Overtaking Configuration';
      renderSidebar();
      renderCanvas();
    }

    function toggleMap() {
      showMap = !showMap;
      const btn = document.getElementById('btnMap');
      btn.innerText = `🗺️ Map: ${showMap ? 'ON' : 'OFF'}`;
      btn.className = `tool-btn ${showMap ? 'active' : ''}`;
      renderCanvas();
    }

    function toggleInvertMap() {
      invertMap = !invertMap;
      const btn = document.getElementById('btnInvert');
      btn.innerText = invertMap ? '🌓 Dark Walls' : '☀️ Raw Grid';
      btn.className = `tool-btn ${invertMap ? 'active' : ''}`;
      renderCanvas();
    }

    function setMapOpacity(val) {
      mapOpacity = parseFloat(val);
      renderCanvas();
    }

    function resizeCanvas() {
      const container = document.getElementById('canvasContainer');
      canvas.width = container.clientWidth * window.devicePixelRatio;
      canvas.height = container.clientHeight * window.devicePixelRatio;
      renderCanvas();
    }

    function resetView() {
      if (!mapData || !mapData.raceline || mapData.raceline.length === 0) return;
      const pts = mapData.raceline;
      let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
      pts.forEach(p => {
        if (p.x_m < minX) minX = p.x_m;
        if (p.x_m > maxX) maxX = p.x_m;
        if (p.y_m < minY) minY = p.y_m;
        if (p.y_m > maxY) maxY = p.y_m;
      });

      // Expand slightly to include surrounding walls if map metadata is present
      if (mapData.map_meta && mapData.map_meta.has_image) {
        const meta = mapData.map_meta;
        const mx0 = meta.origin[0];
        const my0 = meta.origin[1];
        const mx1 = mx0 + meta.width_m;
        const my1 = my0 + meta.height_m;
        minX = Math.min(minX, mx0 + meta.width_m * 0.1);
        maxX = Math.max(maxX, mx1 - meta.width_m * 0.1);
        minY = Math.min(minY, my0 + meta.height_m * 0.1);
        maxY = Math.max(maxY, my1 - meta.height_m * 0.1);
      }

      const trackW = Math.max(maxX - minX, 1);
      const trackH = Math.max(maxY - minY, 1);
      const cW = canvas.width / window.devicePixelRatio;
      const cH = canvas.height / window.devicePixelRatio;

      const scaleX = (cW * 0.78) / trackW;
      const scaleY = (cH * 0.78) / trackH;
      viewScale = Math.min(scaleX, scaleY);

      const midTrackX = (minX + maxX) / 2;
      const midTrackY = (minY + maxY) / 2;
      viewX = cW / 2 - midTrackX * viewScale;
      viewY = cH / 2 + midTrackY * viewScale; // inverted Y

      renderCanvas();
    }

    function zoom(factor) {
      const cW = canvas.width / (2 * window.devicePixelRatio);
      const cH = canvas.height / (2 * window.devicePixelRatio);
      viewX = cW - (cW - viewX) * factor;
      viewY = cH - (cH - viewY) * factor;
      viewScale *= factor;
      renderCanvas();
    }

    function toggleCenterline() {
      showCenterline = !showCenterline;
      document.getElementById('btnCenterline').innerText = `Centerline: ${showCenterline ? 'ON' : 'OFF'}`;
      renderCanvas();
    }

    function worldToScreen(x, y) {
      return {
        x: x * viewScale + viewX,
        y: -y * viewScale + viewY
      };
    }

    function screenToWorld(sx, sy) {
      return {
        x: (sx - viewX) / viewScale,
        y: -(sy - viewY) / viewScale
      };
    }

    // -------------------------------------------------------------------------
    // Canvas Rendering Loop
    // -------------------------------------------------------------------------
    function renderCanvas() {
      if (!ctx) return;
      const dpr = window.devicePixelRatio || 1;
      ctx.save();
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.scale(dpr, dpr);

      if (!mapData || !mapData.raceline || mapData.raceline.length === 0) {
        ctx.restore();
        return;
      }

      const raceline = mapData.raceline;
      const sectors = activeMode === 'speed' ? mapData.speed_config.sectors : mapData.ot_config.sectors;

      // 0. Occupancy Grid Map Image Underlay
      if (showMap && mapImg.complete && mapImg.naturalWidth > 0 && mapData.map_meta && mapData.map_meta.has_image) {
        const meta = mapData.map_meta;
        const x0 = meta.origin[0];
        const y0 = meta.origin[1];
        const w_m = meta.width_m;
        const h_m = meta.height_m;

        // Top-left in world is (x0, y0 + h_m)
        const screenTL = worldToScreen(x0, y0 + h_m);
        const screenW = w_m * viewScale;
        const screenH = h_m * viewScale;

        ctx.save();
        ctx.globalAlpha = mapOpacity;
        if (invertMap) {
          // Neon dark-mode: Invert black walls to crisp white, darken floor
          ctx.filter = 'invert(100%) contrast(150%) brightness(85%)';
        }
        ctx.drawImage(mapImg, screenTL.x, screenTL.y, screenW, screenH);
        ctx.restore();
      }

      // 1. Centerline
      if (showCenterline && mapData.centerline && mapData.centerline.length > 0) {
        ctx.beginPath();
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.25)';
        ctx.lineWidth = 1.2;
        ctx.setLineDash([4, 4]);
        mapData.centerline.forEach((p, i) => {
          const pt = worldToScreen(p.x_m, p.y_m);
          if (i === 0) ctx.moveTo(pt.x, pt.y);
          else ctx.lineTo(pt.x, pt.y);
        });
        ctx.closePath();
        ctx.stroke();
        ctx.setLineDash([]);
      }

      // 1.5 Full Raceline Baseline Underlay (Ensures track geometry is never broken)
      if (raceline && raceline.length > 1) {
        ctx.beginPath();
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.18)';
        ctx.lineWidth = 2.0;
        raceline.forEach((p, i) => {
          const pt = worldToScreen(p.x_m, p.y_m);
          if (i === 0) ctx.moveTo(pt.x, pt.y);
          else ctx.lineTo(pt.x, pt.y);
        });
        ctx.closePath();
        ctx.stroke();
      }

      // 2. Raceline by Sector
      sectors.forEach((sec, sIdx) => {
        ctx.beginPath();
        let color;
        if (activeMode === 'speed') {
          const scale = sec.scaling * mapData.speed_config.global_limit;
          if (scale >= 1.0) color = '#00e676';
          else if (scale >= 0.8) color = '#ffb300';
          else color = '#ff3d71';
        } else {
          color = sec.ot_flag ? '#00e676' : '#ff3d71';
        }

        ctx.strokeStyle = color;
        ctx.lineWidth = sIdx === activeSectorId ? 5.5 : 3.5;
        ctx.shadowColor = color;
        ctx.shadowBlur = sIdx === activeSectorId ? 10 : 0;

        const startIdx = Math.max(0, Math.min(sec.start, raceline.length - 1));
        const endIdx = Math.max(0, Math.min(sec.end, raceline.length));

        let first = true;
        if (startIdx <= endIdx) {
          for (let i = startIdx; i <= endIdx && i < raceline.length; i++) {
            const pt = worldToScreen(raceline[i].x_m, raceline[i].y_m);
            if (first) { ctx.moveTo(pt.x, pt.y); first = false; }
            else ctx.lineTo(pt.x, pt.y);
          }
          // If sector spans to the end of the raceline, draw closing segment back to WP 0
          if (endIdx >= raceline.length && raceline.length > 0) {
            const pt0 = worldToScreen(raceline[0].x_m, raceline[0].y_m);
            ctx.lineTo(pt0.x, pt0.y);
          }
        } else {
          // Wrapped sector: startIdx > endIdx (spans through start/finish line)
          for (let i = startIdx; i < raceline.length; i++) {
            const pt = worldToScreen(raceline[i].x_m, raceline[i].y_m);
            if (first) { ctx.moveTo(pt.x, pt.y); first = false; }
            else ctx.lineTo(pt.x, pt.y);
          }
          for (let i = 0; i <= endIdx && i < raceline.length; i++) {
            const pt = worldToScreen(raceline[i].x_m, raceline[i].y_m);
            ctx.lineTo(pt.x, pt.y);
          }
        }
        ctx.stroke();
        ctx.shadowBlur = 0;
      });

      // 3. Sector Gate Dividers & Handles
      sectors.forEach((sec, sIdx) => {
        const p = raceline[Math.min(sec.start, raceline.length - 1)];
        if (!p) return;
        const pt = worldToScreen(p.x_m, p.y_m);

        // Gate marker pin
        ctx.fillStyle = PALETTE[sIdx % PALETTE.length];
        ctx.beginPath();
        ctx.arc(pt.x, pt.y, sIdx === activeSectorId ? 7 : 5, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 1.5;
        ctx.stroke();

        // Label flag
        ctx.fillStyle = 'rgba(10, 14, 23, 0.88)';
        ctx.fillRect(pt.x + 8, pt.y - 14, 26, 16);
        ctx.fillStyle = '#ffffff';
        ctx.font = '600 10px JetBrains Mono';
        ctx.fillText(`S${sIdx}`, pt.x + 11, pt.y - 2);
      });

      // 4. Start / Finish Line (Waypoint 0) with Heading Arrow
      if (raceline.length > 1) {
        const p0 = raceline[0];
        const p1 = raceline[1];
        const pt0 = worldToScreen(p0.x_m, p0.y_m);
        const pt1 = worldToScreen(p1.x_m, p1.y_m);
        const dx = pt1.x - pt0.x;
        const dy = pt1.y - pt0.y;
        const angle = Math.atan2(dy, dx);

        ctx.save();
        ctx.translate(pt0.x, pt0.y);
        ctx.rotate(angle);
        ctx.strokeStyle = '#00e5ff';
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(0, -18);
        ctx.lineTo(0, 18);
        ctx.stroke();

        ctx.fillStyle = '#00e5ff';
        ctx.beginPath();
        ctx.moveTo(14, 0);
        ctx.lineTo(4, -6);
        ctx.lineTo(4, 6);
        ctx.closePath();
        ctx.fill();
        ctx.restore();
      }

      // 5. Hover indicator
      if (hoveredWpIdx >= 0 && hoveredWpIdx < raceline.length) {
        const wp = raceline[hoveredWpIdx];
        const pt = worldToScreen(wp.x_m, wp.y_m);
        ctx.beginPath();
        ctx.arc(pt.x, pt.y, 8, 0, Math.PI * 2);
        ctx.fillStyle = 'rgba(0, 229, 255, 0.4)';
        ctx.fill();
        ctx.strokeStyle = '#00e5ff';
        ctx.lineWidth = 2;
        ctx.stroke();
      }

      ctx.restore();
      updateHUD();
    }

    // -------------------------------------------------------------------------
    // HUD & Lap Time Calculations
    // -------------------------------------------------------------------------
    function updateHUD() {
      if (!mapData || !mapData.raceline) return;
      const pts = mapData.raceline;
      const N = pts.length;
      document.getElementById('hudWaypoints').innerText = N;

      let totalDist = 0;
      let baseTime = 0;
      let scaledTime = 0;

      const sectors = mapData.speed_config.sectors;
      const globLimit = mapData.speed_config.global_limit || 1.0;

      for (let i = 0; i < N; i++) {
        const next = pts[(i + 1) % N];
        const ds = Math.sqrt((next.x_m - pts[i].x_m)**2 + (next.y_m - pts[i].y_m)**2);
        totalDist += ds;

        const v = Math.max(pts[i].vx_mps || 2.0, 0.1);
        baseTime += ds / v;

        let sMul = 1.0;
        for (let s of sectors) {
          if (i >= s.start && i <= s.end) {
            sMul = s.scaling;
            break;
          }
        }
        const vScaled = Math.max(v * sMul * globLimit, 0.1);
        scaledTime += ds / vScaled;
      }

      document.getElementById('hudLength').innerText = `${totalDist.toFixed(1)} m`;
      document.getElementById('hudLapTime').innerText = `${scaledTime.toFixed(2)} s`;

      const delta = scaledTime - baseTime;
      const deltaEl = document.getElementById('hudTimeDelta');
      if (Math.abs(delta) < 0.01) {
        deltaEl.innerText = '0.00s (Baseline)';
        deltaEl.style.color = 'var(--text-muted)';
      } else if (delta > 0) {
        deltaEl.innerText = `+${delta.toFixed(2)}s (Slower)`;
        deltaEl.style.color = 'var(--accent-red)';
      } else {
        deltaEl.innerText = `${delta.toFixed(2)}s (Faster)`;
        deltaEl.style.color = 'var(--primary-emerald)';
      }
    }

    // -------------------------------------------------------------------------
    // Sidebar Rendering & Editing Logic
    // -------------------------------------------------------------------------
    function renderSidebar() {
      if (!mapData) return;
      const gCard = document.getElementById('globalParamsCard');
      const sList = document.getElementById('sectorList');
      sList.innerHTML = '';

      if (activeMode === 'speed') {
        const speedCfg = mapData.speed_config;
        gCard.innerHTML = `
          <div class="param-row">
            <span class="param-label">GLOBAL SPEED LIMIT</span>
            <div class="param-input-wrap">
              <input type="range" min="0.1" max="1.5" step="0.05" value="${speedCfg.global_limit}"
                oninput="setGlobalLimit(this.value)" style="width: 120px;">
              <span class="val-badge" id="globValBadge">${speedCfg.global_limit.toFixed(2)}x</span>
            </div>
          </div>
        `;

        speedCfg.sectors.forEach((sec, idx) => {
          const card = document.createElement('div');
          card.className = `sector-card ${idx === activeSectorId ? 'active' : ''}`;
          card.onclick = (e) => {
            if (e.target.closest('input, button, label, .slider-row, .flag-switches, .sector-actions')) return;
            selectSector(idx);
          };

          const dist = calculateSectorDistance(sec.start, sec.end);
          card.innerHTML = `
            <div class="sector-header">
              <div class="sector-tag">
                <span class="color-dot" style="background: ${PALETTE[idx % PALETTE.length]}"></span>
                <span>Sector ${idx}</span>
              </div>
              <span class="sector-range">WP ${sec.start} ➔ ${sec.end} (${dist.toFixed(1)}m)</span>
            </div>

            <div class="slider-row">
              <span class="param-label">Scale:</span>
              <input type="range" min="0.1" max="1.5" step="0.05" value="${sec.scaling}"
                oninput="updateSectorScale(${idx}, this.value)">
              <span class="val-badge" id="sVal_${idx}">${sec.scaling.toFixed(2)}x</span>
            </div>

            <div class="flag-switches" onclick="event.stopPropagation()">
              <label class="flag-label" for="cb_only_FTG_${idx}">
                <input type="checkbox" id="cb_only_FTG_${idx}" ${sec.only_FTG ? 'checked' : ''}
                  onchange="toggleFlag(event, ${idx}, 'only_FTG', this.checked)">
                <span>only_FTG</span>
              </label>
              <label class="flag-label" for="cb_no_FTG_${idx}">
                <input type="checkbox" id="cb_no_FTG_${idx}" ${sec.no_FTG ? 'checked' : ''}
                  onchange="toggleFlag(event, ${idx}, 'no_FTG', this.checked)">
                <span>no_FTG</span>
              </label>
            </div>

            <div class="sector-actions">
              <button class="btn-tiny" onclick="event.stopPropagation(); splitSector(${idx})">✂️ Split</button>
              ${speedCfg.sectors.length > 1 ? `<button class="btn-tiny delete" onclick="event.stopPropagation(); deleteSector(${idx})">🗑️ Merge</button>` : ''}
            </div>
          `;
          sList.appendChild(card);
        });

      } else {
        const otCfg = mapData.ot_config;
        gCard.innerHTML = `
          <div class="param-row">
            <span class="param-label">YEET FACTOR</span>
            <input type="number" step="0.05" value="${otCfg.yeet_factor}" class="param-input"
              onchange="otCfg.yeet_factor = parseFloat(this.value)">
          </div>
          <div class="param-row">
            <span class="param-label">SPLINE LENGTH</span>
            <input type="number" step="1" value="${otCfg.spline_len}" class="param-input"
              onchange="otCfg.spline_len = parseInt(this.value)">
          </div>
          <div class="param-row">
            <span class="param-label">OT SECTOR BEGIN</span>
            <input type="number" step="0.05" value="${otCfg.ot_sector_begin}" class="param-input"
              onchange="otCfg.ot_sector_begin = parseFloat(this.value)">
          </div>
        `;

        otCfg.sectors.forEach((sec, idx) => {
          const card = document.createElement('div');
          card.className = `sector-card ${idx === activeSectorId ? 'active' : ''}`;
          card.onclick = (e) => {
            if (e.target.closest('input, button, label, .ot-toggle-label, .param-row, .sector-actions')) return;
            selectSector(idx);
          };

          const dist = calculateSectorDistance(sec.start, sec.end);
          card.innerHTML = `
            <div class="sector-header">
              <div class="sector-tag">
                <span class="color-dot" id="ot_dot_${idx}" style="background: ${sec.ot_flag ? 'var(--primary-emerald)' : 'var(--accent-red)'}"></span>
                <span>OT Sector ${idx}</span>
              </div>
              <span class="sector-range">WP ${sec.start} ➔ ${sec.end} (${dist.toFixed(1)}m)</span>
            </div>

            <div class="param-row" style="margin-top: 4px;" onclick="event.stopPropagation()">
              <span class="param-label">OVERTAKING ALLOWED</span>
              <label class="ot-toggle-label" for="cb_ot_${idx}">
                <input type="checkbox" id="cb_ot_${idx}" ${sec.ot_flag ? 'checked' : ''}
                  onchange="toggleOTFlag(event, ${idx}, this.checked)">
                <span id="ot_status_${idx}" style="font-weight: 700; color: ${sec.ot_flag ? 'var(--primary-emerald)' : 'var(--accent-red)'}">
                  ${sec.ot_flag ? 'YES' : 'NO'}
                </span>
              </label>
            </div>

            <div class="sector-actions">
              <button class="btn-tiny" onclick="event.stopPropagation(); splitSector(${idx})">✂️ Split</button>
              ${otCfg.sectors.length > 1 ? `<button class="btn-tiny delete" onclick="event.stopPropagation(); deleteSector(${idx})">🗑️ Merge</button>` : ''}
            </div>
          `;
          sList.appendChild(card);
        });
      }
    }

    function calculateSectorDistance(start, end) {
      if (!mapData || !mapData.raceline || mapData.raceline.length === 0) return 0;
      const pts = mapData.raceline;
      const n = pts.length;
      let d = 0;
      if (start <= end) {
        for (let i = start; i < end && i < n - 1; i++) {
          d += Math.hypot(pts[i+1].x_m - pts[i].x_m, pts[i+1].y_m - pts[i].y_m);
        }
        if (end >= n) {
          d += Math.hypot(pts[0].x_m - pts[n-1].x_m, pts[0].y_m - pts[n-1].y_m);
        }
      } else {
        for (let i = start; i < n; i++) {
          const next = (i + 1) % n;
          d += Math.hypot(pts[next].x_m - pts[i].x_m, pts[next].y_m - pts[i].y_m);
        }
        for (let i = 0; i < end; i++) {
          d += Math.hypot(pts[i+1].x_m - pts[i].x_m, pts[i+1].y_m - pts[i].y_m);
        }
      }
      return d;
    }

    function selectSector(idx) {
      activeSectorId = idx;
      const cards = document.querySelectorAll('.sector-card');
      cards.forEach((c, i) => {
        c.classList.toggle('active', i === idx);
        if (i === idx) {
          c.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
        }
      });
      renderCanvas();
    }

    function setGlobalLimit(val) {
      const v = parseFloat(val);
      mapData.speed_config.global_limit = v;
      document.getElementById('globValBadge').innerText = `${v.toFixed(2)}x`;
      renderCanvas();
      updateHUD();
    }

    function updateSectorScale(idx, val) {
      const v = parseFloat(val);
      mapData.speed_config.sectors[idx].scaling = v;
      document.getElementById(`sVal_${idx}`).innerText = `${v.toFixed(2)}x`;
      renderCanvas();
      updateHUD();
    }

    function toggleFlag(e, idx, flag, checked) {
      if (e) e.stopPropagation();
      const sec = mapData.speed_config.sectors[idx];
      sec[flag] = checked;

      // ROS 2 mutual exclusion: only_FTG and no_FTG cannot both be active
      if (checked) {
        if (flag === 'only_FTG') {
          sec.no_FTG = false;
          const otherCb = document.getElementById(`cb_no_FTG_${idx}`);
          if (otherCb) otherCb.checked = false;
        } else if (flag === 'no_FTG') {
          sec.only_FTG = false;
          const otherCb = document.getElementById(`cb_only_FTG_${idx}`);
          if (otherCb) otherCb.checked = false;
        }
      }
    }

    function toggleOTFlag(e, idx, checked) {
      if (e) e.stopPropagation();
      const sec = mapData.ot_config.sectors[idx];
      sec.ot_flag = checked;
      const lbl = document.getElementById(`ot_status_${idx}`);
      const dot = document.getElementById(`ot_dot_${idx}`);
      if (lbl) {
        lbl.innerText = checked ? 'YES' : 'NO';
        lbl.style.color = checked ? 'var(--primary-emerald)' : 'var(--accent-red)';
      }
      if (dot) {
        dot.style.background = checked ? 'var(--primary-emerald)' : 'var(--accent-red)';
      }
      renderCanvas();
    }

    function addSectorAtEnd() {
      if (!mapData || !mapData.raceline || mapData.raceline.length === 0) return;
      const sectors = activeMode === 'speed' ? mapData.speed_config.sectors : mapData.ot_config.sectors;
      if (!sectors || sectors.length === 0) return;

      // If user has an active sector selected, divide that one; otherwise divide the longest sector
      let targetIdx = (activeSectorId !== null && activeSectorId >= 0 && activeSectorId < sectors.length)
        ? activeSectorId
        : -1;

      if (targetIdx === -1) {
        let maxLen = -1;
        for (let i = 0; i < sectors.length; i++) {
          const len = sectors[i].end - sectors[i].start;
          if (len > maxLen) {
            maxLen = len;
            targetIdx = i;
          }
        }
      }

      const target = sectors[targetIdx];
      if (target.end - target.start < 4) {
        showToast('Selected sector is too short to divide further', 'error');
        return;
      }

      const mid = Math.floor((target.start + target.end) / 2);
      const newSec = JSON.parse(JSON.stringify(target));
      target.end = mid;
      newSec.start = mid;
      sectors.splice(targetIdx + 1, 0, newSec);
      reindexSectors(sectors);
      activeSectorId = targetIdx + 1;
      renderSidebar();
      renderCanvas();
      updateHUD();
      showToast(`Added Sector ${targetIdx + 1} (divided Sector ${targetIdx} at WP #${mid})`);
    }

    function splitSector(idx) {
      const sectors = activeMode === 'speed' ? mapData.speed_config.sectors : mapData.ot_config.sectors;
      const target = sectors[idx];
      if (target.end - target.start < 4) {
        showToast('Sector too short to split', 'error');
        return;
      }
      const mid = Math.floor((target.start + target.end) / 2);
      const newSec = JSON.parse(JSON.stringify(target));
      target.end = mid;
      newSec.start = mid;
      sectors.splice(idx + 1, 0, newSec);
      reindexSectors(sectors);
      activeSectorId = idx + 1;
      renderSidebar();
      renderCanvas();
      updateHUD();
      showToast(`Split Sector ${idx} into Sector ${idx} and Sector ${idx + 1}`);
    }

    function deleteSector(idx) {
      const sectors = activeMode === 'speed' ? mapData.speed_config.sectors : mapData.ot_config.sectors;
      if (sectors.length <= 1) return;
      if (idx === 0) {
        sectors[1].start = 0;
      } else {
        sectors[idx - 1].end = sectors[idx].end;
      }
      sectors.splice(idx, 1);
      if (sectors.length === 1 && mapData.raceline) {
        sectors[0].start = 0;
        sectors[0].end = mapData.raceline.length;
      }
      reindexSectors(sectors);
      activeSectorId = null;
      renderSidebar();
      renderCanvas();
      updateHUD();
      showToast(`Merged Sector into adjacent sector`);
    }

    function reindexSectors(sectors) {
      sectors.forEach((s, i) => {
        s.id = i;
        if (s.start >= s.end && mapData.raceline) {
          s.end = mapData.raceline.length;
        }
      });
    }

    function addSectorAtSplit(wpIdx) {
      const sectors = activeMode === 'speed' ? mapData.speed_config.sectors : mapData.ot_config.sectors;
      for (let i = 0; i < sectors.length; i++) {
        const s = sectors[i];
        if (wpIdx > s.start && wpIdx < s.end) {
          const newSec = JSON.parse(JSON.stringify(s));
          s.end = wpIdx;
          newSec.start = wpIdx;
          sectors.splice(i + 1, 0, newSec);
          reindexSectors(sectors);
          activeSectorId = i + 1;
          renderSidebar();
          renderCanvas();
          updateHUD();
          showToast(`Added Sector divider at WP #${wpIdx}`);
          return;
        }
      }
    }

    // -------------------------------------------------------------------------
    // Canvas Mouse / Gesture Interaction
    // -------------------------------------------------------------------------
    function setupCanvasInteractions() {
      const c = document.getElementById('canvasContainer');
      const cv = document.getElementById('trackCanvas');
      const hud = document.querySelector('.canvas-hud');
      const toolbar = document.querySelector('.canvas-toolbar');

      // Floating overlays act as independent layers: stop event bubbling so canvas never captures them
      [hud, toolbar].forEach(el => {
        if (!el) return;
        el.addEventListener('mousedown', (e) => e.stopPropagation());
        el.addEventListener('click', (e) => e.stopPropagation());
        el.addEventListener('dblclick', (e) => e.stopPropagation());
        el.addEventListener('wheel', (e) => e.stopPropagation(), { passive: true });
      });

      c.addEventListener('wheel', (e) => {
        if (e.target.closest && e.target.closest('.canvas-hud, .canvas-toolbar')) return;
        e.preventDefault();
        const factor = e.deltaY < 0 ? 1.15 : 0.87;
        const rect = c.getBoundingClientRect();
        const mouseX = e.clientX - rect.left;
        const mouseY = e.clientY - rect.top;

        viewX = mouseX - (mouseX - viewX) * factor;
        viewY = mouseY - (mouseY - viewY) * factor;
        viewScale *= factor;
        renderCanvas();
      }, { passive: false });

      c.addEventListener('mousedown', (e) => {
        // Only initiate canvas panning if the click was directly on the canvas background
        if (e.target !== cv) return;
        if (e.button === 0) {
          isDragging = true;
          dragStartX = e.clientX - viewX;
          dragStartY = e.clientY - viewY;
        }
      });

      window.addEventListener('mousemove', (e) => {
        if (isDragging) {
          viewX = e.clientX - dragStartX;
          viewY = e.clientY - dragStartY;
          renderCanvas();
          return;
        }

        // If cursor is over floating overlay bars, do not test or highlight waypoints
        if (e.target.closest && e.target.closest('.canvas-hud, .canvas-toolbar')) {
          if (hoveredWpIdx !== -1) {
            hoveredWpIdx = -1;
            renderCanvas();
          }
          return;
        }

        // Hover test on raceline
        if (!mapData || !mapData.raceline) return;
        const rect = c.getBoundingClientRect();
        const mx = e.clientX - rect.left;
        const my = e.clientY - rect.top;
        const worldPos = screenToWorld(mx, my);

        let closestIdx = -1;
        let minDistSq = Infinity;
        const pts = mapData.raceline;
        for (let i = 0; i < pts.length; i++) {
          const d2 = (pts[i].x_m - worldPos.x)**2 + (pts[i].y_m - worldPos.y)**2;
          if (d2 < minDistSq) {
            minDistSq = d2;
            closestIdx = i;
          }
        }

        if (minDistSq < 0.36) {
          if (hoveredWpIdx !== closestIdx) {
            hoveredWpIdx = closestIdx;
            renderCanvas();
          }
        } else if (hoveredWpIdx !== -1) {
          hoveredWpIdx = -1;
          renderCanvas();
        }
      });

      window.addEventListener('mouseup', () => { isDragging = false; });

      c.addEventListener('click', (e) => {
        // Only create sector gates if clicking directly on the canvas
        if (e.target !== cv) return;
        if (hoveredWpIdx >= 0) {
          addSectorAtSplit(hoveredWpIdx);
        }
      });
    }

    // -------------------------------------------------------------------------
    // Save to Disk API
    // -------------------------------------------------------------------------
    async function saveConfiguration() {
      if (!mapData) return;
      try {
        if (activeMode === 'speed') {
          const payload = {
            map_name: currentMap,
            global_limit: mapData.speed_config.global_limit,
            sectors: mapData.speed_config.sectors
          };
          const res = await fetch('/api/save_speed_scaling', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(payload)
          });
          const result = await res.json();
          if (result.success) {
            showToast(`✓ Saved speed_scaling.yaml (${result.path})`);
          } else {
            showToast('Failed to save: ' + result.error, 'error');
          }
        } else {
          const payload = {
            map_name: currentMap,
            yeet_factor: mapData.ot_config.yeet_factor,
            spline_len: mapData.ot_config.spline_len,
            ot_sector_begin: mapData.ot_config.ot_sector_begin,
            sectors: mapData.ot_config.sectors
          };
          const res = await fetch('/api/save_ot_sectors', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(payload)
          });
          const result = await res.json();
          if (result.success) {
            showToast(`✓ Saved ot_sectors.yaml (${result.path})`);
          } else {
            showToast('Failed to save: ' + result.error, 'error');
          }
        }
      } catch (err) {
        showToast('Error saving: ' + err, 'error');
      }
    }

    window.onload = init;
  </script>
</body>
</html>
"""

def pgm_to_png_bytes(pgm_path):
    """Pure-Python zero-dependency PGM (P5/P2) to PNG converter using built-in zlib & struct."""
    import zlib
    import struct
    try:
        with open(pgm_path, 'rb') as f:
            magic = f.readline().strip()
            if magic not in (b'P5', b'P2'):
                return None
            line = f.readline().strip()
            while line.startswith(b'#'):
                line = f.readline().strip()
            w, h = map(int, line.split())
            maxval = int(f.readline().strip())
            if magic == b'P5':
                raw_pixels = f.read()
            else:
                tokens = f.read().split()
                raw_pixels = bytes([int(t) for t in tokens])

        # Prepend filter byte 0 to each scanline
        scanlines = []
        for y in range(h):
            scanlines.append(b'\x00' + raw_pixels[y * w : (y + 1) * w])
        idat_data = zlib.compress(b''.join(scanlines))

        def make_chunk(chunk_type, data):
            c = chunk_type + data
            crc = struct.pack('>I', zlib.crc32(c) & 0xffffffff)
            return struct.pack('>I', len(data)) + c + crc

        png_header = b'\x89PNG\r\n\x1a\n'
        ihdr = make_chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 0, 0, 0, 0))
        idat = make_chunk(b'IDAT', idat_data)
        iend = make_chunk(b'IEND', b'')
        return png_header + ihdr + idat + iend
    except Exception as e:
        print(f"[WARN] Pure-python PGM converter failed for {pgm_path}: {e}")
        return None

# -----------------------------------------------------------------------------
# HTTP Server Handler
# -----------------------------------------------------------------------------
class SectorTunerHandler(BaseHTTPRequestHandler):
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
            maps = list_available_maps()
            self._send_json(maps)
            return

        elif path == '/api/map_data':
            map_name = qs.get('map', [''])[0]
            if not map_name:
                self._send_json({'error': 'Missing map query param'}, 400)
                return
            data = load_map_data(map_name)
            self._send_json(data)
            return

        elif path == '/api/map_image':
            map_name = qs.get('map', [''])[0]
            if not map_name:
                self.send_response(400)
                self.end_headers()
                return

            meta = get_map_metadata(map_name)
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
                self.send_header('Cache-Control', 'public, max-age=86400')
                self.send_header('Content-Length', str(len(content)))
                self.end_headers()
                self.wfile.write(content)
                return
            elif img_path.endswith('.pgm'):
                # 1. Prefer existing pre-converted sibling PNG if available
                png_sibling = Path(img_path).with_suffix('.png')
                if png_sibling.exists():
                    with open(png_sibling, 'rb') as f:
                        content = f.read()
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/png')
                    self.send_header('Cache-Control', 'public, max-age=86400')
                    self.send_header('Content-Length', str(len(content)))
                    self.end_headers()
                    self.wfile.write(content)
                    return

                # 2. Try PIL if installed
                content = None
                try:
                    from PIL import Image
                    im = Image.open(img_path)
                    buf = io.BytesIO()
                    im.save(buf, format='PNG')
                    content = buf.getvalue()
                except Exception:
                    pass

                # 3. Fallback to built-in pure-Python PGM->PNG converter (zero-dependency)
                if content is None:
                    content = pgm_to_png_bytes(img_path)

                if content is not None:
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/png')
                    self.send_header('Cache-Control', 'public, max-age=86400')
                    self.send_header('Content-Length', str(len(content)))
                    self.end_headers()
                    self.wfile.write(content)
                    return
                else:
                    self.send_response(500)
                    self.end_headers()
                    return

        self.send_response(404)
        self.end_headers()

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        content_len = int(self.headers.get('Content-Length', 0))
        post_data = self.rfile.read(content_len).decode('utf-8')

        try:
            payload = json.loads(post_data)
        except Exception as e:
            self._send_json({'error': f'Invalid JSON: {e}'}, 400)
            return

        map_name = payload.get('map_name')
        if not map_name:
            self._send_json({'error': 'Missing map_name in payload'}, 400)
            return

        map_dir = MAPS_ROOT / map_name
        wp_dir = map_dir / "waypoints"
        wp_dir.mkdir(parents=True, exist_ok=True)

        if path == '/api/save_speed_scaling':
            global_limit = payload.get('global_limit', 1.0)
            sectors = payload.get('sectors', [])
            yaml_text = dump_speed_scaling_yaml(global_limit, sectors)
            out_file = wp_dir / "speed_scaling.yaml"

            with open(out_file, 'w') as f:
                f.write(yaml_text)

            print(f"[✓] Saved speed_scaling.yaml -> {out_file}")
            self._send_json({
                'success': True,
                'path': str(out_file.relative_to(REPO_ROOT))
            })
            return

        elif path == '/api/save_ot_sectors':
            yeet_factor = payload.get('yeet_factor', 1.25)
            spline_len = payload.get('spline_len', 30)
            ot_sector_begin = payload.get('ot_sector_begin', 0.5)
            sectors = payload.get('sectors', [])
            yaml_text = dump_ot_sectors_yaml(yeet_factor, spline_len, ot_sector_begin, sectors)
            out_file = wp_dir / "ot_sectors.yaml"

            with open(out_file, 'w') as f:
                f.write(yaml_text)

            print(f"[✓] Saved ot_sectors.yaml -> {out_file}")
            self._send_json({
                'success': True,
                'path': str(out_file.relative_to(REPO_ROOT))
            })
            return

        self.send_response(404)
        self.end_headers()

def main():
    parser = argparse.ArgumentParser(description="F1TENTH Web-Based Sector Tuner")
    parser.add_argument("-p", "--port", type=int, default=8050, help="Port to serve the Web UI on (default: 8050)")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host interface to bind (default: 0.0.0.0)")
    args = parser.parse_args()

    server_address = (args.host, args.port)
    httpd = HTTPServer(server_address, SectorTunerHandler)

    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    try:
        from common.network_info import print_service_banner
        print_service_banner(
            title="🏎️  F1TENTH Web Sector Tuner & Visualizer",
            subtitle="[Live Sector Speed & Overtaking Boundary Tuning / Zero X11]",
            port=args.port
        )
    except Exception:
        print("=" * 65)
        print(f"Server active at: http://localhost:{args.port}")
        print("=" * 65)

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[INFO] Sector tuner server stopped.")

if __name__ == '__main__':
    main()
