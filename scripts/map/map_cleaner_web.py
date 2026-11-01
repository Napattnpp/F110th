#!/usr/bin/env python3
"""
F1TENTH Web-Based Map Cleaner & Binary Mask Studio (ADR-003 & ADR-004)
----------------------------------------------------------------------
A focused, robust browser-based visual studio with strict 2-Mode Architecture:

  Mode 1: Raw -> Clean (Localization Grid Generator)
    - Input: Original unedited SLAM map in assets/maps/<map>/raw/<map>_raw.pgm
    - Processing: Morphological cleaning (noise removal, track extraction, spline smoothing)
    - Output: assets/maps/<map>/clean/<map>.pgm & .yaml
    - Purpose: Pure occupancy grid for AMCL particle filter localization.
      No artificial walls or hand-drawn obstacles that break scan matching.

  Mode 2: Clean -> Waypoint Gen (Trajectory Optimizer Mask Editor)
    - Input: Clean map from Mode 1 in assets/maps/<map>/clean/<map>.pgm
    - Auto-Processing: Strict 2-color threshold (< 210 -> 0 wall, else 255 free space)
    - Interactive Tools: Wall Brush (✏️) and Straight Wall Line (📏) to seal room doors,
      pit exits, and branching hallways into a single closed circuit for TUM IQP.
    - Output: assets/maps/<map>/waypoint_gen/<map>_waypoint_gen.pgm, .png, .yaml
    - Pipeline: Direct one-click trigger for generate_waypoints.py raceline solver.
"""

import os
import sys
import json
import base64
import argparse
import subprocess
import shutil
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
SCRIPT_DIR = Path(__file__).resolve().parent

if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

try:
    import cv2
    import numpy as np
    from map_cleaner_core import MapCleaner
except ImportError as e:
    print(f"[!] Warning: Missing map cleaner dependencies: {e}")
    print("    Please run via: .venv/bin/python or install requirements (opencv-python, numpy, scipy).")

try:
    import yaml
    HAVE_PYYAML = True
except ImportError:
    HAVE_PYYAML = False


# -----------------------------------------------------------------------------
# Map Discovery & Strict File Resolvers
# -----------------------------------------------------------------------------

def list_available_maps():
    """List all tracks under assets/maps/ with their ADR-004 readiness status."""
    maps = []
    if not MAPS_ROOT.exists():
        return maps

    for d in sorted(MAPS_ROOT.iterdir()):
        if not d.is_dir() or d.name == "TEST" or d.name.startswith("."):
            continue
        
        name = d.name
        raw_dir = d / "raw"
        clean_dir = d / "clean"
        wp_gen_dir = d / "waypoint_gen"
        wpts_dir = d / "waypoints"

        has_raw = any(raw_dir.glob("*.pgm")) or any(d.glob(f"{name}_raw.pgm"))
        has_clean = any(clean_dir.glob("*.pgm")) and any(clean_dir.glob("*.yaml"))
        has_wp_gen = any(wp_gen_dir.glob("*waypoint_gen.pgm"))
        has_raceline = (wpts_dir / f"{name}_raceline.csv").exists() or any(wpts_dir.glob("*raceline.csv"))

        status = "READY" if (has_raw and has_clean and has_wp_gen and has_raceline) else "NEEDS_CLEAN" if has_raw else "EMPTY"
        maps.append({
            "name": name,
            "has_raw": has_raw,
            "has_clean": has_clean,
            "has_wp_gen": has_wp_gen,
            "has_raceline": has_raceline,
            "status": status
        })
    return maps


def resolve_raw_map_paths(map_name):
    """
    Locate raw PGM and YAML for Mode 1 (Raw -> Clean).
    STRICT POLICY: Input MUST be the original raw SLAM map in raw/.
    Never falls back to clean/ or waypoint_gen/ as cleaning input.
    """
    map_dir = MAPS_ROOT / map_name
    if not map_dir.exists():
        raise FileNotFoundError(f"Map directory not found: {map_dir}")

    raw_dir = map_dir / "raw"
    raw_pgm_candidates = []

    if raw_dir.exists():
        raw_named = raw_dir / f"{map_name}_raw.pgm"
        if raw_named.exists():
            raw_pgm_candidates.append(raw_named)
        else:
            raw_pgm_candidates.extend(sorted(raw_dir.glob("*_raw.pgm")))
            raw_pgm_candidates.extend(sorted(raw_dir.glob("*.pgm")))

    if not raw_pgm_candidates:
        legacy_raw = map_dir / f"{map_name}_raw.pgm"
        if legacy_raw.exists():
            raw_pgm_candidates.append(legacy_raw)

    if not raw_pgm_candidates:
        raise FileNotFoundError(
            f"ADR-004 Violation: No raw SLAM map found for '{map_name}' under {raw_dir}. "
            f"Map cleaning strictly requires the original immutable raw map (e.g. {map_name}_raw.pgm)."
        )

    raw_pgm = raw_pgm_candidates[0]

    raw_yaml = None
    if raw_dir.exists():
        raw_yaml_named = raw_dir / f"{map_name}_raw.yaml"
        if raw_yaml_named.exists():
            raw_yaml = raw_yaml_named
        else:
            cands = list(raw_dir.glob("*.yaml"))
            if cands:
                raw_yaml = cands[0]
    if not raw_yaml and (map_dir / f"{map_name}_raw.yaml").exists():
        raw_yaml = map_dir / f"{map_name}_raw.yaml"

    resolution = 0.05
    origin = [0.0, 0.0, 0.0]
    if raw_yaml and raw_yaml.exists():
        try:
            if HAVE_PYYAML:
                with open(raw_yaml, 'r') as f:
                    ydata = yaml.safe_load(f) or {}
                    resolution = float(ydata.get('resolution', 0.05))
                    origin = ydata.get('origin', [0.0, 0.0, 0.0])
            else:
                resolution = MapCleaner.read_ros_map_resolution(raw_yaml, default=0.05)
        except Exception:
            pass

    return {
        "map_name": map_name,
        "map_dir": map_dir,
        "raw_pgm": raw_pgm,
        "raw_yaml": raw_yaml,
        "resolution": resolution,
        "origin": origin
    }


def resolve_clean_map_paths(map_name):
    """
    Locate clean PGM and YAML for Mode 2 (Clean -> Waypoint Gen).
    STRICT POLICY: Input MUST be the cleaned localization map in clean/.
    """
    map_dir = MAPS_ROOT / map_name
    if not map_dir.exists():
        raise FileNotFoundError(f"Map directory not found: {map_dir}")

    clean_dir = map_dir / "clean"
    clean_pgm = clean_dir / f"{map_name}.pgm"
    if not clean_pgm.exists():
        cands = sorted(clean_dir.glob("*.pgm"))
        clean_pgm = cands[0] if cands else None

    if not clean_pgm or not clean_pgm.exists():
        raise FileNotFoundError(
            f"No clean map found for '{map_name}' under {clean_dir}. "
            f"Please run Mode 1 (Raw -> Clean) first to generate the clean localization map."
        )

    clean_yaml = clean_dir / f"{map_name}.yaml"
    resolution = 0.05
    origin = [0.0, 0.0, 0.0]
    if clean_yaml.exists():
        try:
            if HAVE_PYYAML:
                with open(clean_yaml, 'r') as f:
                    ydata = yaml.safe_load(f) or {}
                    resolution = float(ydata.get('resolution', 0.05))
                    origin = ydata.get('origin', [0.0, 0.0, 0.0])
            else:
                resolution = MapCleaner.read_ros_map_resolution(clean_yaml, default=0.05)
        except Exception:
            pass

    return {
        "map_name": map_name,
        "map_dir": map_dir,
        "clean_pgm": clean_pgm,
        "clean_yaml": clean_yaml,
        "resolution": resolution,
        "origin": origin
    }


def array_to_png_data_url(arr):
    """Encode numpy uint8 array to a base64 PNG data URL."""
    success, encoded = cv2.imencode('.png', arr)
    if not success:
        return ""
    b64 = base64.b64encode(encoded.tobytes()).decode('ascii')
    return f"data:image/png;base64,{b64}"


def apply_user_strokes(img, strokes):
    """Apply manual pencil/straight-line/eraser strokes onto a uint8 image."""
    res = img.copy()
    for s in strokes:
        tool = s.get("tool", "wall")
        val = 0 if tool in ["wall", "line"] else 255
        size = max(1, int(s.get("size", 4)))
        pts = s.get("points", [])
        if not pts:
            continue

        if len(pts) == 1:
            p = (int(round(pts[0][0])), int(round(pts[0][1])))
            cv2.circle(res, p, max(1, size // 2), val, -1)
        elif len(pts) == 2 and tool == "line":
            p1 = (int(round(pts[0][0])), int(round(pts[0][1])))
            p2 = (int(round(pts[1][0])), int(round(pts[1][1])))
            cv2.line(res, p1, p2, val, thickness=size)
            cv2.circle(res, p1, max(1, size // 2), val, -1)
            cv2.circle(res, p2, max(1, size // 2), val, -1)
        else:
            for i in range(len(pts) - 1):
                p1 = (int(round(pts[i][0])), int(round(pts[i][1])))
                p2 = (int(round(pts[i+1][0])), int(round(pts[i+1][1])))
                cv2.line(res, p1, p2, val, thickness=size)
                cv2.circle(res, p2, max(1, size // 2), val, -1)
            cv2.circle(res, (int(round(pts[0][0])), int(round(pts[0][1]))), max(1, size // 2), val, -1)
    return res


def compute_map_stats(img, resolution):
    """Calculate percentage composition and physical dimensions."""
    h, w = img.shape
    total = float(h * w) if (h * w) > 0 else 1.0
    walls_pct = round(float(np.sum(img == 0)) / total * 100.0, 1)
    free_pct = round(float(np.sum(img == 255)) / total * 100.0, 1)
    unknown_pct = round(float(np.sum((img > 0) & (img < 255))) / total * 100.0, 1)
    return {
        "width_px": w,
        "height_px": h,
        "width_m": round(w * resolution, 2),
        "height_m": round(h * resolution, 2),
        "resolution": resolution,
        "walls_pct": walls_pct,
        "free_pct": free_pct,
        "unknown_pct": unknown_pct
    }


# -----------------------------------------------------------------------------
# Embedded Single-Page Frontend (2-Mode Architecture & Dual Synchronized Canvases)
# -----------------------------------------------------------------------------

HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>F1TENTH Map Studio</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <style>
    :root {
      --bg-dark: #0a0e17;
      --bg-panel: #111827;
      --bg-card: #1f2937;
      --bg-input: #374151;
      --border: #374151;
      --border-focus: #3b82f6;
      --text: #f3f4f6;
      --text-muted: #9ca3af;
      --primary: #3b82f6;
      --primary-hover: #2563eb;
      --accent-green: #10b981;
      --accent-green-hover: #059669;
      --accent-orange: #f59e0b;
      --accent-red: #ef4444;
      --font-mono: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      background: var(--bg-dark);
      color: var(--text);
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
      height: 100vh;
      overflow: hidden;
      display: flex;
      flex-direction: column;
    }
    /* Top Header */
    header {
      background: var(--bg-panel);
      border-bottom: 1px solid var(--border);
      padding: 10px 20px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      flex-shrink: 0;
    }
    .brand {
      display: flex;
      align-items: center;
      gap: 12px;
    }
    .brand h1 {
      font-size: 1.15rem;
      font-weight: 700;
      letter-spacing: -0.02em;
      color: #fff;
    }
    .badge {
      background: #1e3a8a;
      color: #93c5fd;
      font-size: 0.72rem;
      font-weight: 600;
      padding: 3px 8px;
      border-radius: 9999px;
      text-transform: uppercase;
      letter-spacing: 0.05em;
    }
    .badge.raw { background: #064e3b; color: #6ee7b7; }
    .badge.clean { background: #1e3a8a; color: #93c5fd; }
    .badge.optimizer { background: #7c2d12; color: #fdba74; }

    /* Mode Switcher Tabs */
    .mode-tabs {
      display: flex;
      gap: 8px;
      margin-bottom: 12px;
    }
    .mode-tab {
      flex: 1;
      padding: 10px 12px;
      background: var(--bg-card);
      border: 1px solid var(--border);
      border-radius: 8px;
      cursor: pointer;
      text-align: left;
      color: var(--text-muted);
      transition: all 0.2s ease;
      display: flex;
      align-items: center;
      gap: 8px;
    }
    .mode-tab:hover {
      background: #283548;
      color: #fff;
    }
    .mode-tab.active {
      background: #1e293b;
      border-color: var(--primary);
      color: #fff;
      box-shadow: 0 0 12px rgba(59, 130, 246, 0.25);
    }
    .mode-tab-num {
      display: inline-flex;
      align-items: center;
      justify-content: center;
      width: 22px;
      height: 22px;
      border-radius: 50%;
      background: var(--bg-input);
      color: #fff;
      font-size: 0.75rem;
      font-weight: 700;
      flex-shrink: 0;
    }
    .mode-tab.active .mode-tab-num {
      background: var(--primary);
    }
    .mode-tab-content {
      line-height: 1.25;
    }
    .mode-tab-title {
      font-size: 0.82rem;
      font-weight: 600;
      display: block;
    }
    .mode-tab-desc {
      font-size: 0.68rem;
      color: var(--text-muted);
    }
    .mode-tab.active .mode-tab-desc {
      color: #93c5fd;
    }

    /* Main Layout */
    .workspace {
      display: flex;
      flex: 1;
      min-height: 0;
    }
    /* Sidebar */
    .sidebar {
      width: 370px;
      background: var(--bg-panel);
      border-right: 1px solid var(--border);
      display: flex;
      flex-direction: column;
      flex-shrink: 0;
      overflow-y: auto;
    }
    .sidebar-section {
      padding: 16px;
      border-bottom: 1px solid var(--border);
    }
    .sidebar-section h3 {
      font-size: 0.8rem;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--text-muted);
      margin-bottom: 12px;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    /* Form controls */
    .form-group { margin-bottom: 14px; }
    .form-group:last-child { margin-bottom: 0; }
    .form-label {
      display: flex;
      justify-content: space-between;
      font-size: 0.85rem;
      font-weight: 500;
      margin-bottom: 6px;
    }
    .form-val {
      font-family: var(--font-mono);
      color: var(--primary);
      font-weight: 600;
    }
    select, input[type="text"], input[type="number"] {
      width: 100%;
      background: var(--bg-card);
      border: 1px solid var(--border);
      color: #fff;
      padding: 8px 10px;
      border-radius: 6px;
      font-size: 0.9rem;
      outline: none;
    }
    select:focus, input:focus { border-color: var(--border-focus); }
    input[type="range"] {
      width: 100%;
      accent-color: var(--primary);
    }
    .toggle-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      cursor: pointer;
      font-size: 0.85rem;
    }
    .toggle-row input[type="checkbox"] {
      width: 18px;
      height: 18px;
      accent-color: var(--accent-green);
      cursor: pointer;
    }
    /* Buttons */
    .btn {
      background: var(--bg-card);
      color: #fff;
      border: 1px solid var(--border);
      padding: 7px 12px;
      border-radius: 6px;
      font-size: 0.82rem;
      font-weight: 600;
      cursor: pointer;
      display: inline-flex;
      align-items: center;
      gap: 6px;
      transition: all 0.15s ease;
    }
    .btn:hover { background: var(--bg-input); }
    .btn-primary { background: var(--primary); border-color: var(--primary); }
    .btn-primary:hover { background: var(--primary-hover); }
    .btn-success { background: var(--accent-green); border-color: var(--accent-green); }
    .btn-success:hover { background: var(--accent-green-hover); }
    .btn-block { width: 100%; justify-content: center; padding: 10px; }
    .btn-sm { padding: 4px 8px; font-size: 0.75rem; }

    /* Viewport Area */
    .viewport {
      flex: 1;
      display: flex;
      flex-direction: column;
      min-width: 0;
      background: #000;
      position: relative;
    }
    /* Toolbar */
    .toolbar {
      background: var(--bg-panel);
      border-bottom: 1px solid var(--border);
      padding: 8px 16px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      flex-shrink: 0;
      z-index: 10;
    }
    .tool-group {
      display: flex;
      align-items: center;
      gap: 6px;
    }
    .tool-btn {
      background: var(--bg-card);
      border: 1px solid var(--border);
      color: var(--text-muted);
      padding: 6px 10px;
      border-radius: 6px;
      font-size: 0.8rem;
      font-weight: 600;
      cursor: pointer;
      display: inline-flex;
      align-items: center;
      gap: 6px;
      transition: all 0.15s ease;
    }
    .tool-btn:hover { color: #fff; background: var(--bg-input); }
    .tool-btn.active {
      background: #1e3a8a;
      border-color: var(--primary);
      color: #93c5fd;
    }

    /* Dual Canvas Container */
    .canvas-container {
      flex: 1;
      display: flex;
      position: relative;
      overflow: hidden;
      user-select: none;
    }
    .viewport-pane {
      flex: 1;
      position: relative;
      overflow: hidden;
      border-right: 1px solid var(--border);
      background-size: 20px 20px;
      background-image: 
        linear-gradient(to right, rgba(255, 255, 255, 0.04) 1px, transparent 1px),
        linear-gradient(to bottom, rgba(255, 255, 255, 0.04) 1px, transparent 1px);
    }
    .viewport-pane:last-child { border-right: none; }
    .pane-badge {
      position: absolute;
      top: 12px;
      left: 12px;
      background: rgba(17, 24, 39, 0.85);
      backdrop-filter: blur(4px);
      border: 1px solid var(--border);
      color: #93c5fd;
      padding: 4px 10px;
      border-radius: 6px;
      font-size: 0.75rem;
      font-weight: 600;
      z-index: 5;
      pointer-events: none;
    }
    .pane-badge.clean-badge { color: #6ee7b7; border-color: rgba(16, 185, 129, 0.4); }
    .pane-badge.optimizer-badge { color: #fdba74; border-color: rgba(249, 115, 22, 0.4); }

    .map-canvas {
      position: absolute;
      transform-origin: 0 0;
      image-rendering: pixelated;
      cursor: crosshair;
    }

    /* Status Bar */
    .statusbar {
      background: var(--bg-panel);
      border-top: 1px solid var(--border);
      padding: 6px 16px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      font-size: 0.75rem;
      color: var(--text-muted);
      flex-shrink: 0;
      font-family: var(--font-mono);
    }
    .status-items { display: flex; gap: 16px; }
    .status-item span { color: #fff; font-weight: 600; }

    /* Toast Notification */
    #toast {
      position: fixed;
      bottom: 24px;
      right: 24px;
      background: #111827;
      border: 1px solid var(--border);
      color: #fff;
      padding: 12px 18px;
      border-radius: 8px;
      box-shadow: 0 10px 25px rgba(0,0,0,0.5);
      font-size: 0.85rem;
      z-index: 1000;
      display: none;
      max-width: 420px;
      line-height: 1.4;
    }
    #toast.success { border-color: var(--accent-green); color: #34d399; }
    #toast.error { border-color: var(--accent-red); color: #f87171; }
    #toast.info { border-color: var(--primary); color: #60a5fa; }

    /* Optimizer Modal */
    .modal-overlay {
      position: fixed;
      inset: 0;
      background: rgba(0, 0, 0, 0.75);
      backdrop-filter: blur(4px);
      display: none;
      align-items: center;
      justify-content: center;
      z-index: 2000;
    }
    .modal {
      background: var(--bg-panel);
      border: 1px solid var(--border);
      border-radius: 10px;
      width: 720px;
      max-width: 90vw;
      max-height: 80vh;
      display: flex;
      flex-direction: column;
      box-shadow: 0 20px 40px rgba(0,0,0,0.6);
    }
    .modal-header {
      padding: 14px 20px;
      border-bottom: 1px solid var(--border);
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    .modal-header h3 { font-size: 1rem; color: #fff; }
    .modal-body {
      padding: 16px;
      flex: 1;
      overflow-y: auto;
      background: #000;
      font-family: var(--font-mono);
      font-size: 0.82rem;
      line-height: 1.5;
      color: #34d399;
      white-space: pre-wrap;
    }
    .modal-footer {
      padding: 12px 20px;
      border-top: 1px solid var(--border);
      display: flex;
      justify-content: flex-end;
      gap: 10px;
    }
  </style>
</head>
<body>

  <!-- Header -->
  <header>
    <div class="brand">
      <h1>F1TENTH Map Studio</h1>
      <span class="badge" id="headerModeBadge">Mode 1: Localization</span>
    </div>
    <div class="header-actions">
      <button class="btn btn-sm btn-primary" onclick="toggleHelpModal()">📖 Workflow Help</button>
    </div>
  </header>

  <!-- Workspace -->
  <div class="workspace">
    
    <!-- Left Sidebar: Controls -->
    <div class="sidebar">
      
      <!-- Track Selection -->
      <div class="sidebar-section">
        <h3>Track Target</h3>
        <div class="form-group">
          <select id="mapSelect" onchange="loadSelectedMap()">
            <option value="">Loading maps...</option>
          </select>
        </div>
      </div>

      <!-- Mode Switcher Tabs -->
      <div class="sidebar-section" style="padding-bottom: 12px;">
        <h3>Workflow Mode</h3>
        <div class="mode-tabs">
          <div class="mode-tab active" id="modeTab1" onclick="switchStudioMode(1)">
            <span class="mode-tab-num">1</span>
            <div class="mode-tab-content">
              <span class="mode-tab-title">Raw → Clean</span>
              <span class="mode-tab-desc">Localization Grid</span>
            </div>
          </div>
          <div class="mode-tab" id="modeTab2" onclick="switchStudioMode(2)">
            <span class="mode-tab-num">2</span>
            <div class="mode-tab-content">
              <span class="mode-tab-title">Clean → Gen</span>
              <span class="mode-tab-desc">Optimizer Mask</span>
            </div>
          </div>
        </div>
      </div>

      <!-- Mode Provenance & Intent Card -->
      <div class="sidebar-section">
        <h3 id="provTitle">Ground Truth Provenance</h3>
        <div style="font-size: 0.78rem; line-height: 1.45; color: var(--text-muted);">
          <div>Source: <code id="stat_prov_src" style="color: #93c5fd; font-family: var(--font-mono); font-size: 0.75rem;">raw/...</code></div>
          <div style="margin-top: 4px;">Target: <code id="stat_prov_target" style="color: #6ee7b7; font-family: var(--font-mono); font-size: 0.75rem;">clean/...</code></div>
          <div id="stat_prov_box" style="margin-top: 8px; padding: 8px 10px; background: rgba(0,0,0,0.3); border-radius: 4px; border-left: 2px solid #10b981;">
            🔒 <strong>Mode 1 Mandate:</strong> Algorithmic cleaning strictly processes original SLAM input. Produces pure 3-value grid for AMCL scan matching without synthetic walls.
          </div>
        </div>
      </div>

      <!-- MODE 1 CONTROLS: Morphological Cleaning -->
      <div id="controls_mode1">
        <div class="sidebar-section">
          <h3>
            Morphological Cleaning
            <div style="display:flex; gap:6px;">
              <button class="btn btn-sm" onclick="runAutoTune()">⚡ Auto-Tune</button>
              <button class="btn btn-sm" onclick="resetToDefaults()">Reset</button>
            </div>
          </h3>
          
          <!-- Free Threshold -->
          <div class="form-group">
            <div class="form-label">
              <span>White Threshold</span>
              <span class="form-val" id="val_free_th">222</span>
            </div>
            <input type="range" id="param_free_th" min="150" max="254" value="222" oninput="onParamChange()">
          </div>

          <!-- Wall Thickness -->
          <div class="form-group">
            <div class="form-label">
              <span>Wall Thickness</span>
              <span class="form-val" id="val_wall_thickness">3 px</span>
            </div>
            <input type="range" id="param_wall_thickness" min="1" max="24" value="3" oninput="onParamChange()">
          </div>

          <!-- Smoothing Radius -->
          <div class="form-group">
            <div class="form-label">
              <span>Track Smooth Radius</span>
              <span class="form-val" id="val_smooth_radius">5 px</span>
            </div>
            <input type="range" id="param_smooth_radius" min="0" max="15" value="5" oninput="onParamChange()">
          </div>

          <!-- Min Blob -->
          <div class="form-group">
            <div class="form-label">
              <span>Min Track Blob</span>
              <span class="form-val" id="val_min_blob">80 px</span>
            </div>
            <input type="range" id="param_min_blob" min="10" max="500" value="80" oninput="onParamChange()">
          </div>

          <!-- Edge Smoothing (Locked to Spline Curves) -->
          <div class="form-group">
            <div class="form-label">
              <span>Edge Smoothing</span>
              <span class="badge" style="background:#1e3a8a; color:#93c5fd; font-size:0.7rem;">Spline Curves</span>
            </div>
            <div class="form-label" style="margin-top:6px;">
              <span>Spline Smoothing Strength</span>
              <span class="form-val" id="val_edge_iters">1</span>
            </div>
            <input type="range" id="param_edge_iters" min="0" max="3" value="1" oninput="onParamChange()">
          </div>

          <!-- Toggles -->
          <div class="form-group">
            <label class="toggle-row">
              <span>Cover track edge with wall</span>
              <input type="checkbox" id="param_cover_edge" onchange="onParamChange()">
            </label>
          </div>
          <div class="form-group">
            <label class="toggle-row">
              <span>Keep largest track circuit only</span>
              <input type="checkbox" id="param_largest_track" checked onchange="onParamChange()">
            </label>
          </div>
        </div>

        <!-- Mode 1 Export Pipeline -->
        <div class="sidebar-section">
          <h3>Export Localization Map</h3>
          <p style="font-size:0.78rem; color:var(--text-muted); margin-bottom:12px; line-height:1.4;">
            Saves clean 3-value occupancy grid to <code style="color:#6ee7b7">clean/&lt;map&gt;.pgm</code> &amp; <code style="color:#6ee7b7">.yaml</code> for AMCL localization.
          </p>
          <button class="btn btn-success btn-block" onclick="saveMapFiles()">💾 Save Clean Localization Map</button>
        </div>
      </div>

      <!-- MODE 2 CONTROLS: Optimizer Binary Mask Editor -->
      <div id="controls_mode2" style="display:none;">
        <div class="sidebar-section">
          <h3>2-Color Base Conversion</h3>
          <p style="font-size:0.78rem; color:var(--text-muted); margin-bottom:12px; line-height:1.4;">
            Inherits directly from <code style="color:#93c5fd">clean/</code> map and thresholds into strict binary 2-color space (0 = Wall, 255 = Free).
          </p>
          <div class="form-group">
            <div class="form-label">
              <span>Binary Threshold</span>
              <span class="form-val" id="val_binary_th">210</span>
            </div>
            <input type="range" id="param_binary_th" min="150" max="254" value="210" oninput="onBinaryThreshChange()">
            <div style="font-size:0.72rem; color:#9ca3af; margin-top:4px;">
              Values &lt; <span id="val_binary_th_dyn">210</span> &rarr; Wall (0), remaining &rarr; Track (255)
            </div>
          </div>
        </div>

        <!-- Mode 2 Drawing Tools Guide -->
        <div class="sidebar-section">
          <h3>Drawing Tools Guide</h3>
          <div style="font-size:0.78rem; color:var(--text-muted); line-height:1.45;">
            <div style="margin-bottom:6px;">
              ✏️ <strong>Seal Wall:</strong> Freehand draw walls across open rooms, corridors, or doors.
            </div>
            <div style="margin-bottom:6px;">
              📏 <strong>Straight Wall:</strong> Click &amp; drag straight wall barriers to seal doorways instantly.
            </div>
            <div style="margin-bottom:6px;">
              🧽 <strong>Clear Track:</strong> Erase debris or barriers to clear track surface.
            </div>
            <div style="margin-top:8px; padding:6px 8px; background:rgba(249, 115, 22, 0.15); border-left:2px solid #f97316; border-radius:4px; color:#fdba74;">
              💡 <strong>Goal:</strong> Ensure track circuit forms a single continuous closed manifold for TUM IQP raceline solver.
            </div>
          </div>
        </div>

        <!-- Mode 2 Export & Optimization Pipeline -->
        <div class="sidebar-section">
          <h3>Export &amp; Raceline Solver</h3>
          <p style="font-size:0.78rem; color:var(--text-muted); margin-bottom:10px; line-height:1.4;">
            Saves strict 2-color mask to <code style="color:#fdba74">waypoint_gen/</code> and solves optimal raceline.
          </p>
          <button class="btn btn-success btn-block" style="margin-bottom:8px;" onclick="saveMapFiles()">💾 Save Waypoint Gen Mask</button>
          <button class="btn btn-primary btn-block" onclick="runOptimizerModal()">🚀 Optimize Waypoints &amp; Raceline</button>
        </div>
      </div>

    </div>

    <!-- Viewport Area -->
    <div class="viewport">
      <!-- Toolbar -->
      <div class="toolbar">
        <!-- Interactive Tools -->
        <div class="tool-group">
          <button class="tool-btn active" id="tool_pan" onclick="setTool('pan')">🔍 Pan/Zoom</button>
          <button class="tool-btn" id="tool_wall" onclick="setTool('wall')">✏️ Seal Wall</button>
          <button class="tool-btn" id="tool_line" onclick="setTool('line')">📏 Straight Wall</button>
          <button class="tool-btn" id="tool_track" onclick="setTool('track')">🧽 Clear Track</button>
          <button class="tool-btn" id="tool_measure" onclick="setTool('measure')">📐 Measure</button>
        </div>

        <!-- Brush Sizes & Edits History -->
        <div class="tool-group">
          <span style="font-size:0.8rem; color:var(--text-muted);">Brush:</span>
          <select id="brushSize" style="width: auto; padding: 4px 8px; font-size: 0.8rem;">
            <option value="2">2 px</option>
            <option value="4" selected>4 px</option>
            <option value="8">8 px</option>
            <option value="16">16 px</option>
            <option value="24">24 px</option>
            <option value="32">32 px</option>
          </select>
          <button class="btn btn-sm" onclick="undoStroke()" title="Undo (Ctrl+Z)">↩ Undo</button>
          <button class="btn btn-sm" onclick="clearAllStrokes()" title="Clear Edits">🗑 Clear</button>
          <span id="strokeCount" style="font-size:0.75rem; color:#9ca3af; margin-left:4px;">0 edits</span>
        </div>

        <!-- View Mode: Side-by-Side vs Single -->
        <div class="tool-group">
          <button class="tool-btn active" id="vmode_split" onclick="setViewMode('split')">🪟 Side-by-Side</button>
          <button class="tool-btn" id="vmode_clean" onclick="setViewMode('clean')">🏁 Right Only</button>
          <button class="tool-btn" id="vmode_raw" onclick="setViewMode('raw')">📷 Left Only</button>
          <button class="btn btn-sm" onclick="resetZoom()" style="margin-left:6px;">Fit View</button>
        </div>
      </div>

      <!-- Canvas Container (Dual Synchronized Viewports) -->
      <div class="canvas-container" id="canvasContainer">
        
        <!-- Missing Clean Map Overlay (Mode 2 only) -->
        <div id="missingCleanOverlay" style="display:none; position:absolute; inset:0; background:rgba(10, 14, 23, 0.94); z-index:100; flex-direction:column; align-items:center; justify-content:center; text-align:center; padding:30px;">
          <div style="font-size:3rem; margin-bottom:12px;">⚠️</div>
          <h2 style="font-size:1.25rem; margin-bottom:8px; color:#f3f4f6;">Clean Localization Map Not Found</h2>
          <p style="color:#9ca3af; max-width:480px; font-size:0.88rem; line-height:1.5; margin-bottom:20px;">
            Mode 2 inherits directly from the cleaned localization map in <code style="color:#6ee7b7">clean/</code>. No clean map exists yet for this track.
          </p>
          <button class="btn btn-primary" onclick="switchStudioMode(1)" style="padding:10px 20px; font-size:0.9rem;">
            &larr; Switch to Mode 1 (Raw &rarr; Clean)
          </button>
        </div>

        <!-- Left Pane (Raw SLAM in Mode 1 / Clean Reference in Mode 2) -->
        <div class="viewport-pane" id="rawPane">
          <div class="pane-badge" id="leftBadge">📷 Raw SLAM (Untouched Input)</div>
          <canvas id="rawCanvas" class="map-canvas"></canvas>
        </div>
        
        <!-- Right Pane (Clean Result in Mode 1 / 2-Color Mask Canvas in Mode 2) -->
        <div class="viewport-pane" id="cleanPane">
          <div class="pane-badge clean-badge" id="rightBadge">🏁 Cleaned Track (Processed Output)</div>
          <canvas id="cleanCanvas" class="map-canvas"></canvas>
        </div>
      </div>

      <!-- Status Bar -->
      <div class="statusbar">
        <div class="status-items">
          <div class="status-item">Dimensions: <span id="stat_dim">-</span></div>
          <div class="status-item">Resolution: <span id="stat_res">0.050 m/px</span></div>
          <div class="status-item">Track: <span id="stat_track">-</span></div>
          <div class="status-item">Walls: <span id="stat_walls">-</span></div>
        </div>
        <div id="stat_measure" style="color:#fbbf24; font-weight:600;"></div>
      </div>
    </div>

  </div>

  <!-- Toast Notification -->
  <div id="toast"></div>

  <!-- Optimizer Console Modal -->
  <div class="modal-overlay" id="optimizerModal">
    <div class="modal">
      <div class="modal-header">
        <h3>TUM IQP Raceline Optimizer</h3>
        <button class="btn btn-sm" onclick="closeOptimizerModal()">✕</button>
      </div>
      <div class="modal-body" id="optimizerConsole">Running optimizer...</div>
      <div class="modal-footer">
        <button class="btn btn-primary" onclick="closeOptimizerModal()">Done</button>
      </div>
    </div>
  </div>

  <script>
    // Studio State variables
    let currentStudioMode = 1; // 1 = Raw -> Clean, 2 = Clean -> Waypoint Gen
    let currentMap = "";
    let mapMeta = null;
    let activeTool = "pan"; // "pan", "wall", "line", "track", "measure"
    let viewMode = "split"; // "split", "clean", "raw"
    
    // Independent stroke histories per mode
    let strokesMode1 = [];
    let strokesMode2 = [];
    let activeStroke = null; // currently drawing stroke
    let measurePoints = []; // [{x, y}]

    // Image caches
    let leftImage = new Image();
    let rightImage = new Image();

    // Canvases
    const leftCanvas = document.getElementById("rawCanvas");
    const leftCtx = leftCanvas.getContext("2d");
    const rightCanvas = document.getElementById("cleanCanvas");
    const rightCtx = rightCanvas.getContext("2d");
    const container = document.getElementById("canvasContainer");

    // Shared Viewport Transform
    let scale = 1.0;
    let panX = 0;
    let panY = 0;
    let isDragging = false;
    let dragStartX = 0;
    let dragStartY = 0;
    let debounceTimer = null;

    function showToast(msg, type = "success") {
      const t = document.getElementById("toast");
      t.innerText = msg;
      t.className = type;
      t.style.display = "block";
      setTimeout(() => { t.style.display = "none"; }, 3500);
    }

    window.addEventListener("DOMContentLoaded", async () => {
      setupCanvasEvents();
      await fetchMapsList();
    });

    async function fetchMapsList() {
      try {
        const res = await fetch("/api/maps");
        const maps = await res.json();
        const sel = document.getElementById("mapSelect");
        sel.innerHTML = "";
        
        let preferred = "";
        const urlParams = new URLSearchParams(window.location.search);
        const mapArg = urlParams.get("map");

        maps.forEach(m => {
          const opt = document.createElement("option");
          opt.value = m.name;
          opt.innerText = `${m.name} [${m.status}]`;
          sel.appendChild(opt);
          if (m.name === mapArg || (!preferred && m.name === "ifac_roboracer") || (!preferred && m.has_raw)) {
            preferred = m.name;
          }
        });

        if (preferred) {
          sel.value = preferred;
        }
        await loadSelectedMap();
      } catch (err) {
        showToast("Failed to load maps list: " + err, "error");
      }
    }

    async function switchStudioMode(mode) {
      if (currentStudioMode === mode) return;
      currentStudioMode = mode;
      activeStroke = null;
      measurePoints = [];
      document.getElementById("stat_measure").innerText = "";

      // Update tabs UI
      document.getElementById("modeTab1").classList.toggle("active", mode === 1);
      document.getElementById("modeTab2").classList.toggle("active", mode === 2);
      
      const headerBadge = document.getElementById("headerModeBadge");
      headerBadge.innerText = (mode === 1) ? "Mode 1: Localization" : "Mode 2: Optimizer Mask";
      headerBadge.className = "badge " + ((mode === 1) ? "clean" : "optimizer");

      // Toggle sidebar controls sections
      document.getElementById("controls_mode1").style.display = (mode === 1) ? "block" : "none";
      document.getElementById("controls_mode2").style.display = (mode === 2) ? "block" : "none";

      // Update Provenance Card Box
      const pTitle = document.getElementById("provTitle");
      const pBox = document.getElementById("stat_prov_box");
      if (mode === 1) {
        pTitle.innerText = "Ground Truth Provenance";
        pBox.style.borderLeftColor = "#10b981";
        pBox.innerHTML = `🔒 <strong>Mode 1 Mandate:</strong> Algorithmic cleaning strictly processes original SLAM input. Produces pure 3-value grid for AMCL scan matching without synthetic walls.`;
      } else {
        pTitle.innerText = "Optimizer Provenance";
        pBox.style.borderLeftColor = "#f97316";
        pBox.innerHTML = `🏎️ <strong>Mode 2 Mandate:</strong> Inherits clean map &amp; auto-converts to 2-color base. Use drawing tools (Wall / Straight Line) to seal doorways and rooms into a closed circuit.`;
      }

      // Default tool switch
      if (mode === 2 && activeTool === "pan") {
        setTool("wall");
      } else if (mode === 1 && activeTool === "line") {
        setTool("pan");
      }

      updateStrokeCountUI();
      await loadSelectedMap();
    }

    async function loadSelectedMap() {
      const sel = document.getElementById("mapSelect");
      currentMap = sel.value;
      if (!currentMap) return;

      const mode = currentStudioMode;
      showToast(`Loading ${currentMap} (Mode ${mode})...`, "info");
      document.getElementById("missingCleanOverlay").style.display = "none";
      activeStroke = null;
      measurePoints = [];
      updateStrokeCountUI();
      document.getElementById("stat_measure").innerText = "";

      try {
        const res = await fetch(`/api/map_data?map=${currentMap}&mode=${mode}`);
        const data = await res.json();
        if (data.error) {
          if (mode === 2 && data.code === "CLEAN_MAP_NOT_FOUND") {
            document.getElementById("missingCleanOverlay").style.display = "flex";
            showToast("Clean map not found. Please run Mode 1 first.", "error");
            return;
          }
          showToast(data.error, "error");
          return;
        }

        mapMeta = data;
        document.getElementById("stat_dim").innerText = `${data.width_px} × ${data.height_px} px (${data.width_m} × ${data.height_m} m)`;
        document.getElementById("stat_res").innerText = `${data.resolution.toFixed(3)} m/px`;
        
        if (mode === 1) {
          document.getElementById("stat_prov_src").innerText = data.raw_path;
          document.getElementById("stat_prov_target").innerText = `clean/${currentMap}.pgm`;
          document.getElementById("leftBadge").innerText = data.left_title || "📷 Raw SLAM (Untouched Input)";
          document.getElementById("leftBadge").className = "pane-badge";
          document.getElementById("rightBadge").innerText = data.right_title || "🏁 Clean Occupancy Grid (Localization Map)";
          document.getElementById("rightBadge").className = "pane-badge clean-badge";
          if (data.params) applyParamsToControls(data.params);
        } else {
          document.getElementById("stat_prov_src").innerText = data.clean_path;
          document.getElementById("stat_prov_target").innerText = `waypoint_gen/${currentMap}_waypoint_gen.pgm`;
          document.getElementById("leftBadge").innerText = data.left_title || "🏁 Clean Map Reference (clean/)";
          document.getElementById("leftBadge").className = "pane-badge clean-badge";
          document.getElementById("rightBadge").innerText = data.right_title || "🏎️ 2-Color Optimizer Mask (waypoint_gen/)";
          document.getElementById("rightBadge").className = "pane-badge optimizer-badge";
          if (data.binary_threshold !== undefined) {
            document.getElementById("param_binary_th").value = data.binary_threshold;
            document.getElementById("val_binary_th").innerText = data.binary_threshold;
            document.getElementById("val_binary_th_dyn").innerText = data.binary_threshold;
          }
        }

        await Promise.all([
          loadImagePromise(leftImage, data.left_url),
          loadImagePromise(rightImage, data.right_url)
        ]);

        leftCanvas.width = data.width_px;
        leftCanvas.height = data.height_px;
        rightCanvas.width = data.width_px;
        rightCanvas.height = data.height_px;

        updateStatsReadout(data.stats);
        resetZoom();
        renderCanvases();
        showToast(`Loaded ${currentMap} successfully (Mode ${mode})`, "success");
      } catch (err) {
        showToast("Failed loading map: " + err, "error");
      }
    }

    function loadImagePromise(img, url) {
      return new Promise((resolve) => {
        img.onload = () => resolve();
        img.onerror = () => resolve();
        img.src = url;
      });
    }

    function applyParamsToControls(p) {
      if (p.free_threshold !== undefined) {
        document.getElementById("param_free_th").value = p.free_threshold;
        document.getElementById("val_free_th").innerText = p.free_threshold;
      }
      if (p.wall_thickness !== undefined) {
        document.getElementById("param_wall_thickness").value = p.wall_thickness;
        document.getElementById("val_wall_thickness").innerText = p.wall_thickness + " px";
      }
      if (p.smooth_radius !== undefined) {
        document.getElementById("param_smooth_radius").value = p.smooth_radius;
        document.getElementById("val_smooth_radius").innerText = p.smooth_radius + " px";
      }
      if (p.min_track_blob !== undefined || p.min_blob !== undefined) {
        const b = p.min_track_blob || p.min_blob;
        document.getElementById("param_min_blob").value = b;
        document.getElementById("val_min_blob").innerText = b + " px";
      }
      if (p.cover_track_edge !== undefined) {
        document.getElementById("param_cover_edge").checked = !!p.cover_track_edge;
      }
      if (p.keep_largest_track !== undefined) {
        document.getElementById("param_largest_track").checked = !!p.keep_largest_track;
      }
      if (p.edge_smooth_iterations !== undefined) {
        document.getElementById("param_edge_iters").value = p.edge_smooth_iterations;
        document.getElementById("val_edge_iters").innerText = p.edge_smooth_iterations;
      }
    }

    function getControlParams() {
      return {
        free_threshold: parseInt(document.getElementById("param_free_th").value),
        wall_thickness: parseInt(document.getElementById("param_wall_thickness").value),
        smooth_radius: parseInt(document.getElementById("param_smooth_radius").value),
        min_blob: parseInt(document.getElementById("param_min_blob").value),
        cover_track_edge: document.getElementById("param_cover_edge").checked,
        keep_largest_track: document.getElementById("param_largest_track").checked,
        edge_smooth_method: "spline",
        edge_smooth_iterations: parseInt(document.getElementById("param_edge_iters").value)
      };
    }

    function onParamChange() {
      document.getElementById("val_free_th").innerText = document.getElementById("param_free_th").value;
      document.getElementById("val_wall_thickness").innerText = document.getElementById("param_wall_thickness").value + " px";
      document.getElementById("val_smooth_radius").innerText = document.getElementById("param_smooth_radius").value + " px";
      document.getElementById("val_min_blob").innerText = document.getElementById("param_min_blob").value + " px";
      document.getElementById("val_edge_iters").innerText = document.getElementById("param_edge_iters").value;

      clearTimeout(debounceTimer);
      debounceTimer = setTimeout(() => {
        requestReClean();
      }, 50);
    }

    function onBinaryThreshChange() {
      const val = document.getElementById("param_binary_th").value;
      document.getElementById("val_binary_th").innerText = val;
      document.getElementById("val_binary_th_dyn").innerText = val;

      clearTimeout(debounceTimer);
      debounceTimer = setTimeout(() => {
        requestReClean();
      }, 50);
    }

    async function requestReClean() {
      if (!currentMap) return;
      const mode = currentStudioMode;
      const currentStrokes = (mode === 1) ? strokesMode1 : strokesMode2;

      const payload = {
        map: currentMap,
        mode: mode,
        strokes: currentStrokes
      };

      if (mode === 1) {
        payload.params = getControlParams();
      } else {
        payload.binary_threshold = parseInt(document.getElementById("param_binary_th").value);
      }

      try {
        const res = await fetch("/api/reclean", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(payload)
        });
        const data = await res.json();
        if (data.error) return;

        await loadImagePromise(rightImage, data.clean_url);
        updateStatsReadout(data.stats);
        renderCanvases();
      } catch (err) {
        console.error("Re-clean error:", err);
      }
    }

    function updateStatsReadout(stats) {
      if (!stats) return;
      document.getElementById("stat_track").innerText = `${stats.free_pct}%`;
      document.getElementById("stat_walls").innerText = `${stats.walls_pct}% (Unknown: ${stats.unknown_pct}%)`;
    }

    async function runAutoTune() {
      if (!currentMap) return;
      try {
        showToast("Auto-tuning parameters from raw SLAM contours...", "info");
        const res = await fetch(`/api/auto_tune?map=${currentMap}`);
        const data = await res.json();
        if (data.params) {
          applyParamsToControls(data.params);
          onParamChange();
          showToast("Parameters auto-tuned from RAW", "success");
        }
      } catch (err) {
        showToast("Auto-tune failed: " + err, "error");
      }
    }

    function resetToDefaults() {
      applyParamsToControls({
        free_threshold: 222,
        wall_thickness: 3,
        smooth_radius: 5,
        min_blob: 80,
        cover_track_edge: false,
        keep_largest_track: true,
        edge_smooth_iterations: 1
      });
      onParamChange();
    }

    // -------------------------------------------------------------------------
    // View Modes & Tool Engine
    // -------------------------------------------------------------------------

    function setViewMode(mode) {
      viewMode = mode;
      document.getElementById("vmode_split").classList.toggle("active", mode === "split");
      document.getElementById("vmode_clean").classList.toggle("active", mode === "clean");
      document.getElementById("vmode_raw").classList.toggle("active", mode === "raw");

      const leftP = document.getElementById("rawPane");
      const rightP = document.getElementById("cleanPane");

      if (mode === "split") {
        leftP.style.display = "block";
        rightP.style.display = "block";
      } else if (mode === "clean") {
        leftP.style.display = "none";
        rightP.style.display = "block";
      } else if (mode === "raw") {
        leftP.style.display = "block";
        rightP.style.display = "none";
      }
      resetZoom();
    }

    function setTool(tool) {
      activeTool = tool;
      document.querySelectorAll(".tool-btn").forEach(btn => {
        if (btn.id.startsWith("tool_")) btn.classList.remove("active");
      });
      const target = document.getElementById("tool_" + tool);
      if (target) target.classList.add("active");

      if (tool !== "measure") {
        measurePoints = [];
        document.getElementById("stat_measure").innerText = "";
      } else {
        document.getElementById("stat_measure").innerText = "Measure: Click two points across track";
      }
      renderCanvases();
    }

    function updateStrokeCountUI() {
      const currentStrokes = (currentStudioMode === 1) ? strokesMode1 : strokesMode2;
      const el = document.getElementById("strokeCount");
      if (el) {
        el.innerText = `${currentStrokes.length} edit${currentStrokes.length === 1 ? '' : 's'}`;
      }
    }

    function renderCanvases() {
      if (!leftCanvas.width || !leftCanvas.height) return;

      // 1. Render Left Pane
      leftCtx.clearRect(0, 0, leftCanvas.width, leftCanvas.height);
      if (leftImage.complete && leftImage.naturalWidth) {
        leftCtx.drawImage(leftImage, 0, 0);
      }

      // 2. Render Right Pane
      rightCtx.clearRect(0, 0, rightCanvas.width, rightCanvas.height);
      if (rightImage.complete && rightImage.naturalWidth) {
        rightCtx.drawImage(rightImage, 0, 0);
      }

      // 3. Transient drawing stroke preview (ONLY visible while dragging mouse)
      if (activeStroke && activeStroke.points.length > 0) {
        const isWall = (activeStroke.tool === "wall" || activeStroke.tool === "line");
        rightCtx.strokeStyle = isWall ? "rgba(239, 68, 68, 0.85)" : "rgba(16, 185, 129, 0.85)";
        rightCtx.fillStyle = rightCtx.strokeStyle;
        rightCtx.lineWidth = activeStroke.size;
        rightCtx.lineCap = "round";
        rightCtx.lineJoin = "round";

        if (activeStroke.points.length === 1) {
          rightCtx.beginPath();
          rightCtx.arc(activeStroke.points[0][0], activeStroke.points[0][1], Math.max(1, activeStroke.size / 2), 0, Math.PI * 2);
          rightCtx.fill();
        } else if (activeStroke.tool === "line" && activeStroke.points.length === 2) {
          rightCtx.beginPath();
          rightCtx.moveTo(activeStroke.points[0][0], activeStroke.points[0][1]);
          rightCtx.lineTo(activeStroke.points[1][0], activeStroke.points[1][1]);
          rightCtx.stroke();
          rightCtx.beginPath();
          rightCtx.arc(activeStroke.points[0][0], activeStroke.points[0][1], Math.max(1, activeStroke.size / 2), 0, Math.PI * 2);
          rightCtx.arc(activeStroke.points[1][0], activeStroke.points[1][1], Math.max(1, activeStroke.size / 2), 0, Math.PI * 2);
          rightCtx.fill();
        } else {
          rightCtx.beginPath();
          rightCtx.moveTo(activeStroke.points[0][0], activeStroke.points[0][1]);
          for (let i = 1; i < activeStroke.points.length; i++) {
            rightCtx.lineTo(activeStroke.points[i][0], activeStroke.points[i][1]);
          }
          rightCtx.stroke();
        }
      }

      // 4. Render Measurement Tool Line (Metric-First, Razor Thin)
      if (activeTool === "measure" && measurePoints.length > 0) {
        const mLineWidth = Math.max(0.4, 1.2 / scale);
        const mDotRadius = Math.max(0.8, 2.5 / scale);

        const targetCtx = rightCtx;
        targetCtx.fillStyle = "#fbbf24";
        targetCtx.strokeStyle = "#fbbf24";
        targetCtx.lineWidth = mLineWidth;

        measurePoints.forEach(p => {
          targetCtx.beginPath();
          targetCtx.arc(p.x, p.y, mDotRadius, 0, Math.PI * 2);
          targetCtx.fill();
        });

        if (measurePoints.length === 2) {
          targetCtx.beginPath();
          targetCtx.setLineDash([Math.max(2, 4 / scale), Math.max(2, 4 / scale)]);
          targetCtx.moveTo(measurePoints[0].x, measurePoints[0].y);
          targetCtx.lineTo(measurePoints[1].x, measurePoints[1].y);
          targetCtx.stroke();
          targetCtx.setLineDash([]);

          // Compute exact metric distance
          const dx = measurePoints[1].x - measurePoints[0].x;
          const dy = measurePoints[1].y - measurePoints[0].y;
          const pxDist = Math.hypot(dx, dy);
          const res = mapMeta ? mapMeta.resolution : 0.05;
          const mDist = pxDist * res;

          document.getElementById("stat_measure").innerText = `Track Width: ${mDist.toFixed(3)} m`;

          // Clean, metric-only floating badge
          const midX = (measurePoints[0].x + measurePoints[1].x) / 2;
          const midY = (measurePoints[0].y + measurePoints[1].y) / 2;
          const label = `${mDist.toFixed(3)} m`;

          targetCtx.save();
          const fontSize = Math.max(7, 11 / scale);
          targetCtx.font = `600 ${fontSize}px sans-serif`;
          const textWidth = targetCtx.measureText(label).width;
          const padX = 5 / scale;
          const padY = 2 / scale;

          targetCtx.fillStyle = "rgba(15, 23, 42, 0.9)";
          targetCtx.strokeStyle = "#fbbf24";
          targetCtx.lineWidth = Math.max(0.5, 1.0 / scale);
          targetCtx.fillRect(midX - textWidth / 2 - padX, midY - fontSize - padY * 2, textWidth + padX * 2, fontSize + padY * 2);
          targetCtx.strokeRect(midX - textWidth / 2 - padX, midY - fontSize - padY * 2, textWidth + padX * 2, fontSize + padY * 2);

          targetCtx.fillStyle = "#fbbf24";
          targetCtx.textAlign = "center";
          targetCtx.textBaseline = "middle";
          targetCtx.fillText(label, midX, midY - fontSize / 2 - padY);
          targetCtx.restore();
        }
      }
    }

    function setupCanvasEvents() {
      container.addEventListener("wheel", onWheel, { passive: false });
      container.addEventListener("mousedown", onMouseDown);
      window.addEventListener("mousemove", onMouseMove);
      window.addEventListener("mouseup", onMouseUp);

      window.addEventListener("keydown", (e) => {
        if ((e.ctrlKey || e.metaKey) && e.key === "z") {
          e.preventDefault();
          undoStroke();
        }
        if (e.key === "Escape") {
          if (activeTool === "measure") {
            measurePoints = [];
            document.getElementById("stat_measure").innerText = "Measure: Click two points";
            renderCanvases();
          }
        }
      });
    }

    function getCanvasCoords(e) {
      const activePane = (viewMode === "raw") ? document.getElementById("rawPane") : document.getElementById("cleanPane");
      const rect = activePane.getBoundingClientRect();
      const mouseX = e.clientX - rect.left;
      const mouseY = e.clientY - rect.top;

      const imgX = (mouseX - panX) / scale;
      const imgY = (mouseY - panY) / scale;
      return { x: imgX, y: imgY };
    }

    function onMouseDown(e) {
      if (e.button !== 0) return;
      const pt = getCanvasCoords(e);

      if (activeTool === "pan" || e.shiftKey) {
        isDragging = true;
        dragStartX = e.clientX - panX;
        dragStartY = e.clientY - panY;
      } else if (activeTool === "wall" || activeTool === "track") {
        isDragging = true;
        const bSize = parseInt(document.getElementById("brushSize").value);
        activeStroke = {
          tool: activeTool,
          size: bSize,
          points: [[pt.x, pt.y]]
        };
        renderCanvases();
      } else if (activeTool === "line") {
        isDragging = true;
        const bSize = parseInt(document.getElementById("brushSize").value);
        activeStroke = {
          tool: "line",
          size: bSize,
          points: [[pt.x, pt.y], [pt.x, pt.y]]
        };
        renderCanvases();
      } else if (activeTool === "measure") {
        if (measurePoints.length >= 2) {
          measurePoints = [];
        }
        measurePoints.push(pt);
        renderCanvases();
      }
    }

    function onMouseMove(e) {
      if (!isDragging) return;

      if (activeTool === "pan" || e.shiftKey) {
        panX = e.clientX - dragStartX;
        panY = e.clientY - dragStartY;
        applyTransform();
      } else if ((activeTool === "wall" || activeTool === "track") && activeStroke) {
        const pt = getCanvasCoords(e);
        activeStroke.points.push([pt.x, pt.y]);
        renderCanvases();
      } else if (activeTool === "line" && activeStroke) {
        const pt = getCanvasCoords(e);
        activeStroke.points[1] = [pt.x, pt.y];
        renderCanvases();
      }
    }

    function onMouseUp() {
      if (isDragging && activeStroke) {
        isDragging = false;
        const currentStrokes = (currentStudioMode === 1) ? strokesMode1 : strokesMode2;
        currentStrokes.push(activeStroke);
        activeStroke = null;
        updateStrokeCountUI();
        requestReClean();
      }
      isDragging = false;
    }

    function onWheel(e) {
      e.preventDefault();
      const activePane = (viewMode === "raw") ? document.getElementById("rawPane") : document.getElementById("cleanPane");
      const rect = activePane.getBoundingClientRect();
      const mouseX = e.clientX - rect.left;
      const mouseY = e.clientY - rect.top;

      const zoomFactor = e.deltaY < 0 ? 1.15 : 0.85;
      const newScale = Math.min(Math.max(scale * zoomFactor, 0.1), 30.0);

      panX = mouseX - (mouseX - panX) * (newScale / scale);
      panY = mouseY - (mouseY - panY) * (newScale / scale);
      scale = newScale;

      applyTransform();
      renderCanvases();
    }

    function applyTransform() {
      leftCanvas.style.transform = `translate(${panX}px, ${panY}px) scale(${scale})`;
      rightCanvas.style.transform = `translate(${panX}px, ${panY}px) scale(${scale})`;
    }

    function resetZoom() {
      if (!rightCanvas.width) return;
      const activePane = (viewMode === "raw") ? document.getElementById("rawPane") : document.getElementById("cleanPane");
      if (!activePane.clientWidth) return;

      const scaleX = (activePane.clientWidth - 40) / rightCanvas.width;
      const scaleY = (activePane.clientHeight - 40) / rightCanvas.height;
      scale = Math.min(scaleX, scaleY, 2.0);
      panX = (activePane.clientWidth - rightCanvas.width * scale) / 2;
      panY = (activePane.clientHeight - rightCanvas.height * scale) / 2;
      applyTransform();
      renderCanvases();
    }

    function undoStroke() {
      const currentStrokes = (currentStudioMode === 1) ? strokesMode1 : strokesMode2;
      if (currentStrokes.length > 0) {
        currentStrokes.pop();
        updateStrokeCountUI();
        requestReClean();
      }
    }

    function clearAllStrokes() {
      const currentStrokes = (currentStudioMode === 1) ? strokesMode1 : strokesMode2;
      if (currentStrokes.length > 0 || measurePoints.length > 0) {
        if (currentStudioMode === 1) strokesMode1 = [];
        else strokesMode2 = [];
        activeStroke = null;
        measurePoints = [];
        document.getElementById("stat_measure").innerText = "";
        updateStrokeCountUI();
        requestReClean();
      }
    }

    // -------------------------------------------------------------------------
    // API Actions (Save & Optimize)
    // -------------------------------------------------------------------------

    async function saveMapFiles() {
      if (!currentMap) return;
      const mode = currentStudioMode;
      const currentStrokes = (mode === 1) ? strokesMode1 : strokesMode2;

      showToast(`Saving Mode ${mode} map assets...`, "info");

      const payload = {
        map: currentMap,
        mode: mode,
        strokes: currentStrokes
      };

      if (mode === 1) {
        payload.params = getControlParams();
      } else {
        payload.binary_threshold = parseInt(document.getElementById("param_binary_th").value);
      }

      try {
        const res = await fetch("/api/save", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(payload)
        });
        const data = await res.json();
        if (data.success) {
          showToast("✓ " + data.message, "success");
        } else {
          showToast("✗ Save failed: " + data.error, "error");
        }
      } catch (err) {
        showToast("Save error: " + err, "error");
      }
    }

    async function runOptimizerModal() {
      if (!currentMap) return;
      document.getElementById("optimizerModal").style.display = "flex";
      document.getElementById("optimizerConsole").innerText = `Running TUM IQP Optimizer on ${currentMap}...\\n`;

      try {
        const res = await fetch(`/api/optimize?map=${currentMap}`, { method: "POST" });
        const data = await res.json();
        document.getElementById("optimizerConsole").innerText += "\\n" + data.output;
        if (data.success) {
          showToast("Optimizer finished successfully!", "success");
        } else {
          showToast("Optimizer completed with warnings", "error");
        }
      } catch (err) {
        document.getElementById("optimizerConsole").innerText += "\\nError running optimizer: " + err;
      }
    }

    function closeOptimizerModal() {
      document.getElementById("optimizerModal").style.display = "none";
    }

    function toggleHelpModal() {
      alert(
        "F1TENTH 2-Mode Map Studio Guide:\\n\\n" +
        "Mode 1: Raw -> Clean (Localization)\\n" +
        "  - Inputs original SLAM map from raw/.\\n" +
        "  - Adjust morphological filters (White Threshold, Spline Smoothing).\\n" +
        "  - Click 'Save Clean Localization Map'. Produces clean/<map>.pgm for AMCL.\\n\\n" +
        "Mode 2: Clean -> Waypoint Gen (Raceline Optimizer)\\n" +
        "  - Inputs clean/<map>.pgm and converts to 2-color binary mask.\\n" +
        "  - Use 'Seal Wall' (pencil) or 'Straight Wall' (line tool) to close doorways, rooms, and pits.\\n" +
        "  - Click 'Save Waypoint Gen Mask', then 'Optimize Waypoints & Raceline'!"
      );
    }
  </script>
</body>
</html>
"""


# -----------------------------------------------------------------------------
# HTTP Server & API Request Handler
# -----------------------------------------------------------------------------

class MapCleanerHandler(BaseHTTPRequestHandler):
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
            mode = int(qs.get('mode', [1])[0])
            if not map_name:
                self._send_json({"error": "No map name provided"}, status=400)
                return

            try:
                cleaner = MapCleaner(verbose=False)
                if mode == 1:
                    paths = resolve_raw_map_paths(map_name)
                    raw_img = cleaner.load_pgm(str(paths['raw_pgm']))
                    tuned_params = cleaner.auto_tune_f1tenth(raw_img)
                    clean_img = cleaner.clean_f1tenth(
                        raw_img,
                        free_threshold=tuned_params.get('free_threshold', 222),
                        wall_thickness=tuned_params.get('wall_thickness', 3),
                        smooth_radius=tuned_params.get('smooth_radius', 5),
                        min_track_blob=tuned_params.get('min_track_blob', 80),
                        cover_track_edge=tuned_params.get('cover_track_edge', False),
                        keep_largest_track=tuned_params.get('keep_largest_track', True),
                        edge_smooth_method='spline',
                        edge_smooth_iterations=1
                    )
                    stats = compute_map_stats(clean_img, paths['resolution'])
                    response = {
                        "mode": 1,
                        "map_name": map_name,
                        "resolution": paths['resolution'],
                        "origin": paths['origin'],
                        "raw_path": str(paths['raw_pgm'].relative_to(REPO_ROOT)),
                        "width_px": stats['width_px'],
                        "height_px": stats['height_px'],
                        "width_m": stats['width_m'],
                        "height_m": stats['height_m'],
                        "params": tuned_params,
                        "stats": stats,
                        "left_url": array_to_png_data_url(raw_img),
                        "right_url": array_to_png_data_url(clean_img),
                        "left_title": "📷 Raw SLAM (Untouched Input)",
                        "right_title": "🏁 Clean Occupancy Grid (Localization Map)"
                    }
                    self._send_json(response)

                else:
                    # Mode 2: Clean -> Waypoint Gen
                    try:
                        paths = resolve_clean_map_paths(map_name)
                    except FileNotFoundError as fnf:
                        self._send_json({"error": str(fnf), "code": "CLEAN_MAP_NOT_FOUND"}, status=404)
                        return

                    clean_img = cleaner.load_pgm(str(paths['clean_pgm']))
                    binary_thresh = int(qs.get('binary_threshold', [210])[0])
                    mask_img = np.where(clean_img < binary_thresh, 0, 255).astype(np.uint8)
                    stats = compute_map_stats(mask_img, paths['resolution'])

                    response = {
                        "mode": 2,
                        "map_name": map_name,
                        "resolution": paths['resolution'],
                        "origin": paths['origin'],
                        "clean_path": str(paths['clean_pgm'].relative_to(REPO_ROOT)),
                        "width_px": stats['width_px'],
                        "height_px": stats['height_px'],
                        "width_m": stats['width_m'],
                        "height_m": stats['height_m'],
                        "binary_threshold": binary_thresh,
                        "stats": stats,
                        "left_url": array_to_png_data_url(clean_img),
                        "right_url": array_to_png_data_url(mask_img),
                        "left_title": "🏁 Clean Map Reference (clean/)",
                        "right_title": "🏎️ 2-Color Optimizer Mask (waypoint_gen/)"
                    }
                    self._send_json(response)

            except Exception as e:
                self._send_json({"error": str(e)}, status=500)
            return

        elif path == '/api/auto_tune':
            map_name = qs.get('map', [''])[0]
            try:
                paths = resolve_raw_map_paths(map_name)
                cleaner = MapCleaner(verbose=False)
                raw_img = cleaner.load_pgm(str(paths['raw_pgm']))
                params = cleaner.auto_tune_f1tenth(raw_img)
                self._send_json({"params": params})
            except Exception as e:
                self._send_json({"error": str(e)}, status=500)
            return

        self.send_response(404)
        self.end_headers()

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(length) if length > 0 else b'{}'

        try:
            req_data = json.loads(body.decode('utf-8'))
        except Exception:
            req_data = {}

        if path == '/api/reclean':
            map_name = req_data.get('map', '')
            mode = int(req_data.get('mode', 1))
            strokes = req_data.get('strokes', [])

            try:
                cleaner = MapCleaner(verbose=False)
                if mode == 1:
                    paths = resolve_raw_map_paths(map_name)
                    raw_img = cleaner.load_pgm(str(paths['raw_pgm']))
                    params = req_data.get('params', {})

                    clean_img = cleaner.clean_f1tenth(
                        raw_img,
                        free_threshold=params.get('free_threshold', 222),
                        wall_thickness=params.get('wall_thickness', 3),
                        smooth_radius=params.get('smooth_radius', 5),
                        min_track_blob=params.get('min_blob', 80),
                        cover_track_edge=params.get('cover_track_edge', False),
                        keep_largest_track=params.get('keep_largest_track', True),
                        edge_smooth_method='spline',
                        edge_smooth_iterations=params.get('edge_smooth_iterations', 1),
                        fill_unknown_as_wall=params.get('fill_unknown_as_wall', False)
                    )
                    if strokes:
                        clean_img = apply_user_strokes(clean_img, strokes)

                    stats = compute_map_stats(clean_img, paths['resolution'])
                    self._send_json({
                        "clean_url": array_to_png_data_url(clean_img),
                        "stats": stats
                    })
                else:
                    # Mode 2: Clean -> Waypoint Gen
                    paths = resolve_clean_map_paths(map_name)
                    clean_img = cleaner.load_pgm(str(paths['clean_pgm']))
                    binary_thresh = int(req_data.get('binary_threshold', 210))
                    mask_img = np.where(clean_img < binary_thresh, 0, 255).astype(np.uint8)

                    if strokes:
                        mask_img = apply_user_strokes(mask_img, strokes)
                    mask_img = np.where(mask_img < 128, 0, 255).astype(np.uint8)

                    stats = compute_map_stats(mask_img, paths['resolution'])
                    self._send_json({
                        "clean_url": array_to_png_data_url(mask_img),
                        "stats": stats
                    })
            except Exception as e:
                self._send_json({"error": str(e)}, status=500)
            return

        elif path == '/api/save':
            map_name = req_data.get('map', '')
            mode = int(req_data.get('mode', 1))
            strokes = req_data.get('strokes', [])

            try:
                cleaner = MapCleaner(verbose=False)
                map_dir = MAPS_ROOT / map_name
                clean_dir = map_dir / "clean"
                wp_gen_dir = map_dir / "waypoint_gen"
                clean_dir.mkdir(parents=True, exist_ok=True)
                wp_gen_dir.mkdir(parents=True, exist_ok=True)

                if mode == 1:
                    paths = resolve_raw_map_paths(map_name)
                    raw_img = cleaner.load_pgm(str(paths['raw_pgm']))
                    params = req_data.get('params', {})

                    clean_img = cleaner.clean_f1tenth(
                        raw_img,
                        free_threshold=params.get('free_threshold', 222),
                        wall_thickness=params.get('wall_thickness', 3),
                        smooth_radius=params.get('smooth_radius', 5),
                        min_track_blob=params.get('min_blob', 80),
                        cover_track_edge=params.get('cover_track_edge', False),
                        keep_largest_track=params.get('keep_largest_track', True),
                        edge_smooth_method='spline',
                        edge_smooth_iterations=params.get('edge_smooth_iterations', 1),
                        fill_unknown_as_wall=params.get('fill_unknown_as_wall', False)
                    )
                    if strokes:
                        clean_img = apply_user_strokes(clean_img, strokes)

                    clean_pgm_path = clean_dir / f"{map_name}.pgm"
                    clean_png_path = clean_dir / f"{map_name}.png"
                    clean_yaml_path = clean_dir / f"{map_name}.yaml"
                    
                    cleaner.save_pgm(str(clean_pgm_path), clean_img)
                    cv2.imwrite(str(clean_png_path), clean_img)

                    yaml_data = {
                        "resolution": paths['resolution'],
                        "origin": paths['origin'],
                        "negate": 0,
                        "occupied_thresh": 0.65,
                        "free_thresh": 0.196
                    }
                    if paths['raw_yaml'] and Path(paths['raw_yaml']).exists():
                        try:
                            if HAVE_PYYAML:
                                with open(paths['raw_yaml'], 'r') as f:
                                    loaded = yaml.safe_load(f)
                                    if loaded:
                                        yaml_data.update(loaded)
                        except Exception:
                            pass

                    clean_yaml = dict(yaml_data)
                    clean_yaml['image'] = f"{map_name}.pgm"
                    with open(clean_yaml_path, 'w') as f:
                        if HAVE_PYYAML:
                            yaml.safe_dump(clean_yaml, f, default_flow_style=False)
                        else:
                            f.write(f"image: {map_name}.pgm\nresolution: {paths['resolution']}\norigin: {paths['origin']}\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")

                    self._send_json({
                        "success": True,
                        "message": f"Successfully exported clean/{map_name}.pgm and clean/{map_name}.yaml for localization! Switch to Mode 2 to edit the optimizer mask."
                    })

                else:
                    # Mode 2: Clean -> Waypoint Gen
                    paths = resolve_clean_map_paths(map_name)
                    clean_img = cleaner.load_pgm(str(paths['clean_pgm']))
                    binary_thresh = int(req_data.get('binary_threshold', 210))
                    mask_img = np.where(clean_img < binary_thresh, 0, 255).astype(np.uint8)

                    if strokes:
                        mask_img = apply_user_strokes(mask_img, strokes)
                    mask_img = np.where(mask_img < 128, 0, 255).astype(np.uint8)

                    wp_pgm_path = wp_gen_dir / f"{map_name}_waypoint_gen.pgm"
                    wp_png_path = wp_gen_dir / f"{map_name}_waypoint_gen.png"
                    wp_yaml_path = wp_gen_dir / f"{map_name}_waypoint_gen.yaml"

                    cleaner.save_pgm(str(wp_pgm_path), mask_img)
                    cv2.imwrite(str(wp_png_path), mask_img)

                    yaml_data = {
                        "resolution": paths['resolution'],
                        "origin": paths['origin'],
                        "negate": 0,
                        "occupied_thresh": 0.65,
                        "free_thresh": 0.196
                    }
                    if paths['clean_yaml'] and Path(paths['clean_yaml']).exists():
                        try:
                            if HAVE_PYYAML:
                                with open(paths['clean_yaml'], 'r') as f:
                                    loaded = yaml.safe_load(f)
                                    if loaded:
                                        yaml_data.update(loaded)
                        except Exception:
                            pass

                    wp_yaml = dict(yaml_data)
                    wp_yaml['image'] = f"{map_name}_waypoint_gen.pgm"
                    with open(wp_yaml_path, 'w') as f:
                        if HAVE_PYYAML:
                            yaml.safe_dump(wp_yaml, f, default_flow_style=False)
                        else:
                            f.write(f"image: {map_name}_waypoint_gen.pgm\nresolution: {paths['resolution']}\norigin: {paths['origin']}\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")

                    # Defensively ensure waypoint_gen/global_planner_params.yaml exists
                    track_params = wp_gen_dir / "global_planner_params.yaml"
                    if not track_params.exists():
                        template_path = REPO_ROOT / "src" / "race_bringup" / "config" / "global_planner" / "global_planner_params.template.yaml"
                        central_path = REPO_ROOT / "src" / "race_bringup" / "config" / "global_planner" / "global_planner_params.yaml"
                        src_tmpl = template_path if template_path.exists() else central_path
                        if src_tmpl.exists():
                            shutil.copy2(src_tmpl, track_params)

                    self._send_json({
                        "success": True,
                        "message": f"Successfully exported waypoint_gen/{map_name}_waypoint_gen.pgm and .png! Ready for raceline optimization."
                    })

            except Exception as e:
                self._send_json({"error": str(e)}, status=500)
            return

        elif path == '/api/optimize':
            qs = parse_qs(parsed.query)
            map_name = qs.get('map', [''])[0] or req_data.get('map', '')
            if not map_name:
                self._send_json({"error": "No map name provided"}, status=400)
                return

            try:
                opt_script = REPO_ROOT / "src" / "planning" / "global_planner" / "global_planner" / "generate_waypoints.py"
                cmd = [sys.executable, str(opt_script), "-m", map_name]
                res = subprocess.run(cmd, capture_output=True, text=True, timeout=90)
                output = res.stdout + ("\nErrors:\n" + res.stderr if res.stderr else "")
                self._send_json({
                    "success": (res.returncode == 0),
                    "output": output
                })
            except Exception as e:
                self._send_json({"error": str(e), "output": str(e)}, status=500)
            return

        self.send_response(404)
        self.end_headers()


# -----------------------------------------------------------------------------
# Main CLI Entry Point
# -----------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="F1TENTH Web-Based Map Cleaner (ADR-003 & ADR-004)")
    parser.add_argument("--map", "-m", help="Target map name (e.g. rectangle1, ifac_roboracer, yee2)")
    parser.add_argument("--port", "-p", type=int, default=8051, help="Server port (default: 8051)")
    parser.add_argument("--host", default="0.0.0.0", help="Bind host (default: 0.0.0.0)")
    args = parser.parse_args()

    target_map = args.map
    if not target_map:
        env_file = REPO_ROOT / ".env"
        if env_file.exists():
            try:
                with open(env_file, 'r') as f:
                    for line in f:
                        if line.strip().startswith('MAP='):
                            val = line.strip().split('=', 1)[1].strip().strip('"').strip("'")
                            if val and (MAPS_ROOT / val).is_dir():
                                target_map = val
                                break
            except Exception:
                pass
    if not target_map:
        target_map = "ifac_roboracer"

    server_address = (args.host, args.port)
    httpd = HTTPServer(server_address, MapCleanerHandler)

    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    try:
        from common.network_info import print_service_banner
        print_service_banner(
            title="🗺️  F1TENTH Web Map Cleaner & Binary Mask Studio",
            subtitle="Interactive Map Inpainting & Track Boundary Generation",
            port=args.port,
            query=f"map={target_map}" if target_map else ""
        )
    except Exception:
        map_query = f"?map={target_map}" if target_map else ""
        print(f"\n  → Local URL:     http://localhost:{args.port}/{map_query}\n")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[!] Shutting down Web Map Cleaner server.")
        httpd.server_close()


if __name__ == "__main__":
    main()
