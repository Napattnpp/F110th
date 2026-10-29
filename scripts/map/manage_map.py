#!/usr/bin/env python3
"""
F1TENTH Unified Map Lifecycle Pipeline Manager
----------------------------------------------
Single entry point for all map operations:
  - status: Inspect track compliance with ADR-004 4-subfolder contract.
  - init: Standardize a raw SLAM directory into the 4 subfolders.
  - clean: Clean occupancy grid and generate binary mask (100% headless / No X11 needed).
  - optimize: Run TUM IQP raceline & centerline optimizer.
  - pipeline: Automated end-to-end map build (init -> clean -> optimize).

Usage:
  python3 scripts/map/manage_map.py status [MAP_NAME]
  python3 scripts/map/manage_map.py init <MAP_NAME>
  python3 scripts/map/manage_map.py clean <MAP_NAME> [--gui]
  python3 scripts/map/manage_map.py optimize <MAP_NAME>
  python3 scripts/map/manage_map.py pipeline <MAP_NAME>
"""

import os
import sys
import argparse
import subprocess
import numpy as np
from pathlib import Path

# Add project root and map_utils to path
REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MAP_UTILS_DIR = REPO_ROOT / "src" / "utilities" / "map_utils"
MAPS_ROOT = REPO_ROOT / "assets" / "maps"

if str(MAP_UTILS_DIR) not in sys.path:
    sys.path.insert(0, str(MAP_UTILS_DIR))


def find_map_dir(map_name: str) -> Path:
    """Locate map directory in assets/maps."""
    p = Path(map_name)
    if p.is_dir():
        return p.resolve()
    target = MAPS_ROOT / map_name
    if target.is_dir():
        return target.resolve()
    for root, dirs, _ in os.walk(MAPS_ROOT):
        if Path(root).name == map_name:
            return Path(root).resolve()
    raise FileNotFoundError(f"Map '{map_name}' not found under {MAPS_ROOT}")


def cmd_status(args):
    """Check map status against ADR-004 4-subfolder structure."""
    if args.map:
        dirs_to_check = [find_map_dir(args.map)]
    else:
        dirs_to_check = sorted([d for d in MAPS_ROOT.iterdir() if d.is_dir() and d.name != "TEST"])

    print(f"\n{'='*75}")
    print(f" F1TENTH Map Status & ADR-004 Compliance Audit")
    print(f"{'='*75}")

    for d in dirs_to_check:
        name = d.name
        raw = d / "raw"
        clean = d / "clean"
        wp_gen = d / "waypoint_gen"
        wpts = d / "waypoints"

        has_raw = any(raw.glob("*.pgm")) or any(raw.glob("*.pbstream"))
        has_clean = any(clean.glob("*.pgm")) and any(clean.glob("*.yaml"))
        has_wp_gen = any(wp_gen.glob("*waypoint_gen.pgm"))
        has_raceline = (wpts / f"{name}_raceline.csv").exists() or any(wpts.glob("*raceline.csv"))
        has_json = (wpts / "global_waypoints.json").exists()
        has_sectors = (wpts / "speed_scaling.yaml").exists()

        status_str = "✅ READY" if (has_raw and has_clean and has_wp_gen and has_raceline and has_json) else "⚠️ INCOMPLETE"

        print(f"\n📍 Track: {name} [{status_str}]")
        print(f"   Directory: {d.relative_to(REPO_ROOT)}")
        print(f"   - 1. raw/          : {'✓ Present' if has_raw else '✗ Missing'}")
        print(f"   - 2. clean/        : {'✓ Present' if has_clean else '✗ Missing'}")
        print(f"   - 3. waypoint_gen/ : {'✓ Present (Binary Mask)' if has_wp_gen else '✗ Missing'}")
        print(f"   - 4. waypoints/    : Raceline={'✓' if has_raceline else '✗'}, JSON={'✓' if has_json else '✗'}, Sectors={'✓' if has_sectors else '✗'}")


def cmd_init(args):
    """Standardize a map folder using init_map.py."""
    init_script = REPO_ROOT / "scripts" / "map" / "init_map.py"
    cmd = [sys.executable, str(init_script), args.map]
    print(f"Running map initialization for: {args.map}")
    res = subprocess.run(cmd)
    if res.returncode != 0:
        sys.exit(res.returncode)


def cmd_clean(args):
    """Clean map and generate binary mask (Headless by default / zero X11 required)."""
    map_dir = find_map_dir(args.map)
    raw_dir = map_dir / "raw"
    clean_dir = map_dir / "clean"
    wp_gen_dir = map_dir / "waypoint_gen"

    raw_pgm_candidates = list(raw_dir.glob("*.pgm")) or list(map_dir.glob("*.pgm"))
    if not raw_pgm_candidates:
        print(f"[!] Error: No raw .pgm image found in {raw_dir}")
        sys.exit(1)

    raw_pgm = raw_pgm_candidates[0]
    print(f"Selected source raw map: {raw_pgm.name}")

    # If --web requested, launch browser-based Map Cleaner (ADR-003 Zero X11)
    if getattr(args, "web", False):
        web_script = REPO_ROOT / "scripts" / "map" / "map_cleaner_web.py"
        port = str(getattr(args, "port", 8051))
        cmd = [sys.executable, str(web_script), "--map", args.map, "--port", port]
        subprocess.run(cmd)
        return

    # If --web or legacy --gui requested, launch canonical Web Map Cleaner (ADR-003 & ADR-010)
    if getattr(args, "web", False) or getattr(args, "gui", False):
        if getattr(args, "gui", False) and not getattr(args, "web", False):
            print("\n[!] Notice: Legacy Matplotlib GUI retired under ADR-010.")
            print("    Launching canonical Web Map Studio instead (http://localhost:8051)...")
        web_script = REPO_ROOT / "scripts" / "map" / "map_cleaner_web.py"
        port = str(getattr(args, "port", 8051))
        cmd = [sys.executable, str(web_script), "--map", args.map, "--port", port]
        subprocess.run(cmd)
        return

    # Headless cleaning via MapCleaner core (0% X11 dependency)
    print("Executing Headless Map Cleaning & Binary Mask Generation (0% X11)...")
    from map_cleaner_core import MapCleaner
    cleaner = MapCleaner(verbose=True)

    raw_img = cleaner.load_pgm(str(raw_pgm))
    print(f"Loaded image: {raw_img.shape[1]}x{raw_img.shape[0]} px")

    # Run F1TENTH racetrack cleaning algorithm
    clean_img, params = cleaner.auto_clean_f1tenth(raw_img)
    print(f"Auto-tuning complete: {params}")

    clean_dir.mkdir(parents=True, exist_ok=True)
    wp_gen_dir.mkdir(parents=True, exist_ok=True)

    # 1. Save Clean Occupancy Grid
    base_name = map_dir.name
    clean_pgm_path = clean_dir / f"{base_name}.pgm"
    cleaner.save_pgm(str(clean_pgm_path), clean_img)
    print(f"✓ Saved Clean Map: {clean_pgm_path.relative_to(REPO_ROOT)}")

    # 2. Save Strict 2-Color Binary Mask (0 wall, 255 drivable track)
    wp_gen_pgm_path = wp_gen_dir / f"{base_name}_waypoint_gen.pgm"
    binary_mask = np.zeros_like(clean_img, dtype=np.uint8)
    binary_mask[clean_img == 255] = 255  # Only free space is 255; walls & unknown become 0
    cleaner.save_pgm(str(wp_gen_pgm_path), binary_mask)
    print(f"✓ Saved Binary Mask (0 unknown): {wp_gen_pgm_path.relative_to(REPO_ROOT)}")

    # Copy YAMLs if present in raw
    raw_yamls = list(raw_dir.glob("*.yaml")) or list(map_dir.glob("*.yaml"))
    if raw_yamls:
        import yaml
        with open(raw_yamls[0], 'r') as f:
            ydata = yaml.safe_load(f) or {}

        # Save clean.yaml
        clean_yaml_path = clean_dir / f"{base_name}.yaml"
        clean_ydata = dict(ydata)
        clean_ydata['image'] = f"{base_name}.pgm"
        with open(clean_yaml_path, 'w') as f:
            yaml.safe_dump(clean_ydata, f, default_flow_style=False)
        print(f"✓ Saved Clean YAML: {clean_yaml_path.relative_to(REPO_ROOT)}")

        # Save waypoint_gen.yaml
        wp_gen_yaml_path = wp_gen_dir / f"{base_name}_waypoint_gen.yaml"
        wp_ydata = dict(ydata)
        wp_ydata['image'] = f"{base_name}_waypoint_gen.pgm"
        with open(wp_gen_yaml_path, 'w') as f:
            yaml.safe_dump(wp_ydata, f, default_flow_style=False)
        print(f"✓ Saved Waypoint Gen YAML: {wp_gen_yaml_path.relative_to(REPO_ROOT)}")


def cmd_optimize(args):
    """Run TUM IQP raceline optimization."""
    opt_script = REPO_ROOT / "src" / "planning" / "global_planner" / "global_planner" / "generate_waypoints.py"
    cmd = [sys.executable, str(opt_script), "-m", args.map]
    print(f"Running TUM IQP Waypoint Optimizer for: {args.map}")
    res = subprocess.run(cmd)
    if res.returncode != 0:
        sys.exit(res.returncode)


def cmd_pipeline(args):
    """Run automated end-to-end map pipeline."""
    print(f"\n========================================================")
    print(f"  STARTING AUTOMATED MAP PIPELINE: {args.map}")
    print(f"========================================================")
    
    print("\n[Stage 1/3] Initializing 4-subfolder structure...")
    cmd_init(args)

    print(f"\n[Stage 2/3] Cleaning map & generating binary mask ({'Web Studio' if getattr(args, 'web', False) else 'Headless'})...")
    cmd_clean(args)

    print("\n[Stage 3/3] Optimizing raceline & generating waypoints...")
    cmd_optimize(args)

    print("\n[Stage Complete] Auditing track status:")
    cmd_status(args)
    print(f"\n🎉 Track '{args.map}' is race-ready! Next: run 'make tuner' to inspect speed & overtaking sectors.\n")


def main():
    parser = argparse.ArgumentParser(description="F1TENTH Unified Map Pipeline Manager")
    subparsers = parser.add_subparsers(dest="command", required=True)

    # status
    p_status = subparsers.add_parser("status", help="Inspect ADR-004 map compliance")
    p_status.add_argument("map", nargs="?", default="", help="Specific map name (optional)")
    p_status.set_defaults(func=cmd_status)

    # init
    p_init = subparsers.add_parser("init", help="Initialize 4-subfolder architecture")
    p_init.add_argument("map", help="Target map name")
    p_init.set_defaults(func=cmd_init)

    # clean
    p_clean = subparsers.add_parser("clean", help="Clean occupancy grid & generate binary mask (Headless by default)")
    p_clean.add_argument("map", help="Target map name")
    p_clean.add_argument("--web", action="store_true", help="Launch interactive Web Map Cleaner in browser (Zero X11 / ADR-003)")
    p_clean.add_argument("--gui", action="store_true", help="Launch legacy Matplotlib GUI (requires X11)")
    p_clean.add_argument("--port", type=int, default=8051, help="Port for Web Map Cleaner (default: 8051)")
    p_clean.set_defaults(func=cmd_clean)

    # optimize
    p_opt = subparsers.add_parser("optimize", help="Run TUM IQP raceline optimizer")
    p_opt.add_argument("map", help="Target map name")
    p_opt.set_defaults(func=cmd_optimize)

    # pipeline
    p_pipe = subparsers.add_parser("pipeline", help="Automated end-to-end map build (init -> clean -> optimize)")
    p_pipe.add_argument("map", help="Target map name")
    p_pipe.add_argument("--web", action="store_true", help="Use Web Map Cleaner for cleaning stage")
    p_pipe.add_argument("--gui", action="store_true", help="Use legacy GUI for cleaning stage")
    p_pipe.add_argument("--port", type=int, default=8051, help="Port for Web Map Cleaner (default: 8051)")
    p_pipe.set_defaults(func=cmd_pipeline)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
