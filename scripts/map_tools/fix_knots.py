#!/usr/bin/env python3
"""
Automated Waypoint Knot & Teardrop Loop Repair Engine
------------------------------------------------------
Detects and repairs self-intersecting loops and normal crossover knots at sharp hairpin
apexes across all generated map global_waypoints.json files.
"""
import os
import sys
import json
import numpy as np
from pathlib import Path
from scipy.interpolate import splprep, splev

repo_root = Path(__file__).resolve().parent.parent.parent

def repair_waypoint_array(wpnts):
    """
    Detects self-intersecting loops / teardrop knots at sharp hairpin apexes
    and re-interpolates the raceline smoothly using B-splines.
    """
    if len(wpnts) < 10:
        return wpnts, False

    x = np.array([w['x_m'] for w in wpnts])
    y = np.array([w['y_m'] for w in wpnts])
    v = np.array([w['vx_mps'] for w in wpnts])

    N = len(x)
    dx = np.diff(np.append(x, x[0]))
    dy = np.diff(np.append(y, y[0]))
    angles = np.arctan2(dy, dx)
    dangles = np.abs(np.diff(np.append(angles, angles[0])))
    dangles = np.minimum(dangles, 2 * np.pi - dangles)

    # Detect sharp turnaround loops (heading jump > 1.7 rad within 0.5m)
    knot_mask = np.zeros(N, dtype=bool)
    for i in range(N):
        prev_idx = (i - 2) % N
        dist = np.sqrt((x[i] - x[prev_idx])**2 + (y[i] - y[prev_idx])**2)
        if dangles[i] > 1.7 and dist < 0.5:
            knot_mask[i] = True
            knot_mask[(i - 1) % N] = True
            knot_mask[(i + 1) % N] = True

    clean_indices = np.where(~knot_mask)[0]
    if len(clean_indices) < N and len(clean_indices) > 5:
        cx = x[clean_indices]
        cy = y[clean_indices]
        cv = v[clean_indices]

        # Fit periodic smoothing B-spline
        tck, u = splprep([cx, cy], s=0.08, per=True, k=3)
        u_new = np.linspace(0, 1, N, endpoint=False)
        new_x, new_y = splev(u_new, tck)

        # Smooth velocity profile across repaired apex
        new_v = np.interp(np.linspace(0, len(cv), N, endpoint=False), np.arange(len(cv)), cv)

        for i, w in enumerate(wpnts):
            w['x_m'] = float(new_x[i])
            w['y_m'] = float(new_y[i])
            w['vx_mps'] = float(new_v[i])

        return wpnts, True

    return wpnts, False

def repair_all_maps():
    maps_root = repo_root / "assets" / "maps"
    json_files = list(maps_root.rglob("global_waypoints.json"))

    print(f"[REPAIR] Scanning {len(json_files)} maps for hairpin knots/loops...")
    for jfile in json_files:
        map_dir = jfile.parent
        map_name = map_dir.name

        with open(jfile, 'r') as f:
            data = json.load(f)

        repaired_any = False
        for key in ['global_traj_wpnts_iqp', 'global_traj_wpnts_sp']:
            if key in data and 'wpnts' in data[key]:
                data[key]['wpnts'], rep = repair_waypoint_array(data[key]['wpnts'])
                if rep:
                    repaired_any = True

        if repaired_any:
            with open(jfile, 'w') as f:
                json.dump(data, f)
            print(f"  ✓ Successfully repaired hairpin knots in {map_name}")

if __name__ == '__main__':
    repair_all_maps()
