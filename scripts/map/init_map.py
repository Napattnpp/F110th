#!/usr/bin/env python3
"""
F1TENTH Map Standardizer / Initializer Tool
-------------------------------------------
Converts any flat or external map directory (assets/maps/<map_name>/) into our
standardized subfolder architecture:

  assets/maps/<map_name>/
  ├── raw/                         # Original unedited SLAM / imported files
  │   ├── <map_name>_raw.pgm
  │   └── <map_name>_raw.yaml
  ├── clean/                       # Localization map (amcl / particle filter)
  │   ├── <map_name>.pgm
  │   └── <map_name>.yaml
  ├── waypoint_gen/                # Raceline solver map (generate_waypoints.py)
  │   ├── <map_name>_waypoint_gen.pgm
  │   └── <map_name>_waypoint_gen.yaml
  └── waypoints/                   # Planning CSVs, JSON, sectors & parameters
      ├── global_planner_params.yaml
      ├── <map_name>_centerline.csv
      ├── <map_name>_raceline.csv
      └── global_waypoints.json

Usage:
  python3 scripts/map/init_map.py <map_name_or_directory>

Examples:
  python3 scripts/map/init_map.py rectangle1
  python3 scripts/map/init_map.py assets/maps/provided_track
"""

import os
import sys
import shutil
import argparse
import yaml
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent


def find_map_directory(target_str: str) -> Path:
    """Locate target map directory under assets/maps or explicit path."""
    p = Path(target_str)
    if p.is_dir():
        return p.resolve()

    maps_root = REPO_ROOT / "assets" / "maps"
    if (maps_root / target_str).is_dir():
        return (maps_root / target_str).resolve()

    # Search recursively inside assets/maps
    for root, dirs, _ in os.walk(maps_root):
        if Path(root).name == target_str:
            return Path(root).resolve()

    raise FileNotFoundError(f"Could not find map directory for: '{target_str}' inside {maps_root}")


def format_map_directory(map_dir: Path):
    map_name = map_dir.name
    print(f"==================================================")
    print(f" Standardizing Map Architecture: {map_name}")
    print(f" Directory: {map_dir}")
    print(f"==================================================")

    raw_dir = map_dir / "raw"
    clean_dir = map_dir / "clean"
    wp_gen_dir = map_dir / "waypoint_gen"
    waypoints_dir = map_dir / "waypoints"

    for d in [raw_dir, clean_dir, wp_gen_dir, waypoints_dir]:
        d.mkdir(parents=True, exist_ok=True)

    # 1. Search for source image & YAML files
    img_candidates = list(map_dir.glob("*.pgm")) + list(map_dir.glob("*.png")) + \
                     list(raw_dir.glob("*.pgm")) + list(raw_dir.glob("*.png")) + \
                     list(clean_dir.glob("*.pgm")) + list(clean_dir.glob("*.png"))

    yaml_candidates = list(map_dir.glob("*.yaml")) + list(map_dir.glob("*.yml")) + \
                      list(raw_dir.glob("*.yaml")) + list(clean_dir.glob("*.yaml"))

    # Exclude YAML files inside waypoints/
    yaml_candidates = [y for y in yaml_candidates if y.parent.name != "waypoints"]

    if not img_candidates or not yaml_candidates:
        raise FileNotFoundError(f"Missing required .pgm/.png or .yaml map files in {map_dir}")

    src_img = img_candidates[0]
    src_yaml = yaml_candidates[0]

    print(f"Found source map files:")
    print(f"  - Image: {src_img.name}")
    print(f"  - YAML:  {src_yaml.name}")

    with open(src_yaml, 'r') as f:
        yaml_data = yaml.safe_load(f)

    # -------------------------------------------------------------------------
    # 2. POPULATE raw/ SUBFOLDER
    # -------------------------------------------------------------------------
    raw_img_path = raw_dir / f"{map_name}_raw{src_img.suffix}"
    raw_yaml_path = raw_dir / f"{map_name}_raw.yaml"

    if src_img.resolve() != raw_img_path.resolve():
        shutil.copy2(src_img, raw_img_path)
    raw_yaml_data = dict(yaml_data)
    raw_yaml_data['image'] = raw_img_path.name
    with open(raw_yaml_path, 'w') as f:
        yaml.dump(raw_yaml_data, f, default_flow_style=False)
    print(f"✓ Populated raw/ ({raw_img_path.name}, {raw_yaml_path.name})")

    # -------------------------------------------------------------------------
    # 3. POPULATE clean/ SUBFOLDER
    # -------------------------------------------------------------------------
    clean_img_path = clean_dir / f"{map_name}{src_img.suffix}"
    clean_yaml_path = clean_dir / f"{map_name}.yaml"

    if not clean_img_path.exists():
        shutil.copy2(src_img, clean_img_path)
    clean_yaml_data = dict(yaml_data)
    clean_yaml_data['image'] = clean_img_path.name
    with open(clean_yaml_path, 'w') as f:
        yaml.dump(clean_yaml_data, f, default_flow_style=False)
    print(f"✓ Populated clean/ ({clean_img_path.name}, {clean_yaml_path.name})")

    # -------------------------------------------------------------------------
    # 4. POPULATE waypoint_gen/ SUBFOLDER
    # -------------------------------------------------------------------------
    wp_gen_img_path = wp_gen_dir / f"{map_name}_waypoint_gen{src_img.suffix}"
    wp_gen_yaml_path = wp_gen_dir / f"{map_name}_waypoint_gen.yaml"

    # Search if _waypoint_gen image already existed in old waypoints/ folder
    old_gen_img = waypoints_dir / f"{map_name}_waypoint_gen{src_img.suffix}"
    if old_gen_img.exists():
        shutil.move(str(old_gen_img), str(wp_gen_img_path))
    elif not wp_gen_img_path.exists():
        shutil.copy2(clean_img_path, wp_gen_img_path)

    wp_gen_yaml_data = dict(yaml_data)
    wp_gen_yaml_data['image'] = wp_gen_img_path.name
    with open(wp_gen_yaml_path, 'w') as f:
        yaml.dump(wp_gen_yaml_data, f, default_flow_style=False)
    print(f"✓ Populated waypoint_gen/ ({wp_gen_img_path.name}, {wp_gen_yaml_path.name})")

    # -------------------------------------------------------------------------
    # 5. POPULATE waypoints/ SUBFOLDER
    # -------------------------------------------------------------------------
    # Move loose trajectory CSVs, JSON, sectors, parameters into waypoints/
    for item in map_dir.iterdir():
        if item.is_file() and item.name not in [src_img.name, src_yaml.name]:
            if item.suffix in ['.csv', '.json', '.yaml', '.yml'] or 'sector' in item.name or 'speed' in item.name:
                dest = waypoints_dir / item.name
                shutil.move(str(item), str(dest))

    # Initialize per-track global_planner_params.yaml template in waypoint_gen/ if missing
    track_params = wp_gen_dir / "global_planner_params.yaml"
    old_params = waypoints_dir / "global_planner_params.yaml"
    if old_params.exists() and not track_params.exists():
        shutil.move(str(old_params), str(track_params))
        print(f"✓ Moved global_planner_params.yaml to waypoint_gen/")
    elif not track_params.exists():
        central_params = REPO_ROOT / "src" / "race_bringup" / "config" / "global_planner" / "global_planner_params.yaml"
        if central_params.exists():
            shutil.copy2(central_params, track_params)
            print(f"✓ Initialized waypoint_gen/global_planner_params.yaml template")

    # Remove all loose top-level files from map_dir root (keep ONLY the 4 subfolders)
    for item in list(map_dir.iterdir()):
        if item.is_file() and item.name != '.DS_Store':
            item.unlink()
            print(f"✓ Removed top-level legacy file: {item.name}")

    print(f"==================================================")
    print(f" Map '{map_name}' successfully standardized!")
    print(f" Subfolders ready:")
    print(f"   - {raw_dir.relative_to(REPO_ROOT)}")
    print(f"   - {clean_dir.relative_to(REPO_ROOT)}")
    print(f"   - {wp_gen_dir.relative_to(REPO_ROOT)}")
    print(f"   - {waypoints_dir.relative_to(REPO_ROOT)}")
    print(f"==================================================")


def main():
    parser = argparse.ArgumentParser(description="F1TENTH Map Standardizer")
    parser.add_argument("map", type=str, help="Target map name (e.g. rectangle1) or path to map directory")
    args = parser.parse_args()

    try:
        map_dir = find_map_directory(args.map)
        format_map_directory(map_dir)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
