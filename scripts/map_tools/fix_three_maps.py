#!/usr/bin/env python3
import os
import sys
import cv2
import yaml
import numpy as np
from pathlib import Path

repo_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(repo_root / "src" / "planning" / "global_planner"))
sys.path.insert(0, str(repo_root / "src" / "utilities" / "map_utils"))

maps_to_fix = [
    "test/JFR_racingv6",
    "test/f",
    "test/hangar_1905_v0"
]

maps_root = repo_root / "assets" / "maps"

for map_rel in maps_to_fix:
    map_dir = maps_root / map_rel
    map_name = map_dir.name
    
    img_path = map_dir / f"{map_name}.png"
    if not img_path.exists():
        img_path = map_dir / f"{map_name}.pgm"
        
    print(f"\n[CLEANING] Processing map: {map_rel}")
    img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"Error loading {img_path}")
        continue
        
    yaml_path = map_dir / f"{map_name}.yaml"
    free_thresh = 0.196
    if yaml_path.exists():
        with open(yaml_path, 'r') as f:
            ydata = yaml.safe_load(f)
            free_thresh = float(ydata.get('free_thresh', 0.196))
            
    pixel_thresh = int((1.0 - free_thresh) * 255.0)
    binary = np.where(img >= pixel_thresh, 255, 0).astype(np.uint8)
    
    # Morphological closing to fill wall gaps and round sharp inner wall spikes
    kernel_large = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    
    walls = 255 - binary
    walls_closed = cv2.morphologyEx(walls, cv2.MORPH_CLOSE, kernel_large)
    
    cleaned_binary = 255 - walls_closed
    cleaned_binary = cv2.medianBlur(cleaned_binary, 7)
    
    out_png = map_dir / f"{map_name}_waypoint_gen.png"
    out_pgm = map_dir / f"{map_name}_waypoint_gen.pgm"
    
    cv2.imwrite(str(out_png), cleaned_binary)
    cv2.imwrite(str(out_pgm), cleaned_binary)
    print(f"[SAVED] Cleaned map saved to {out_png.name}")
