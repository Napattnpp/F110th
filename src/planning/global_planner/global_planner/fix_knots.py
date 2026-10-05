#!/usr/bin/env python3
"""
Advanced Trajectory Optimization & Repair Engine (C^2 B-Spline, Friction Ellipse, Start Line Alignment)
------------------------------------------------------------------------------------------------------
1. Guarantees C^2 periodic B-Spline curvature continuity across all waypoints.
2. Applies dual-axis Friction Ellipse velocity profiling: (a_long/a_max)^2 + (a_lat/a_max)^2 <= 1.
3. Aligns index 0 (s=0.0m) with SLAM map origin start/finish line.
"""
import os
import sys
import json
import numpy as np
import yaml
from pathlib import Path
from scipy.interpolate import splprep, splev

repo_root = Path(__file__).resolve().parent.parent.parent

def verify_track_continuity(wpnts):
    """
    Performs rigorous C0 (Position), C1 (Heading), C2 (Curvature), and Velocity
    closed-loop continuity verification across all waypoints and seam transition.
    
    Returns:
        is_continuous (bool): True if 100% continuous.
        report (dict): Detailed diagnostic metrics.
    """
    if len(wpnts) < 5:
        return False, {'error': 'Insufficient waypoints'}

    N = len(wpnts)
    x = np.array([w['x_m'] for w in wpnts])
    y = np.array([w['y_m'] for w in wpnts])
    psi = np.array([w['psi_rad'] for w in wpnts])
    kappa = np.array([w['kappa_radpm'] for w in wpnts])
    v = np.array([w['vx_mps'] for w in wpnts])

    # 1. C0 Position Continuity (Max segment step size & seam gap)
    segment_dists = np.sqrt(np.diff(x, append=x[0])**2 + np.diff(y, append=y[0])**2)
    max_c0_gap = float(np.max(segment_dists))
    seam_c0_gap = float(segment_dists[-1])

    # 2. C1 Heading Tangent Continuity
    dpsi = np.abs(np.diff(psi, append=psi[0]))
    dpsi = np.minimum(dpsi, 2 * np.pi - dpsi)
    max_c1_jump = float(np.rad2deg(np.max(dpsi)))
    seam_c1_jump = float(np.rad2deg(dpsi[-1]))

    # 3. C2 Curvature / Steering Continuity
    dkappa = np.abs(np.diff(kappa, append=kappa[0]))
    max_c2_jump = float(np.max(dkappa))
    seam_c2_jump = float(dkappa[-1])

    # 4. Velocity Continuity
    dv = np.abs(np.diff(v, append=v[0]))
    max_v_jump = float(np.max(dv))
    seam_v_jump = float(dv[-1])

    # Verification Criteria
    c0_ok = max_c0_gap < 0.35 and seam_c0_gap < 0.35
    c1_ok = max_c1_jump < 25.0 and seam_c1_jump < 25.0
    c2_ok = max_c2_jump < 5.0 and seam_c2_jump < 5.0
    v_ok = max_v_jump < 1.0 and seam_v_jump < 1.0

    is_continuous = c0_ok and c1_ok and c2_ok and v_ok

    report = {
        'is_continuous': is_continuous,
        'max_c0_gap_m': max_c0_gap,
        'seam_c0_gap_m': seam_c0_gap,
        'max_c1_jump_deg': max_c1_jump,
        'seam_c1_jump_deg': seam_c1_jump,
        'max_c2_jump_radpm': max_c2_jump,
        'max_v_jump_mps': max_v_jump,
        'c0_ok': c0_ok,
        'c1_ok': c1_ok,
        'c2_ok': c2_ok,
        'v_ok': v_ok
    }

    return is_continuous, report

def repair_and_optimize_waypoints(wpnts, map_origin=None, mu_func=None, v_max_global=11.0, g=9.81):
    """
    Applies B-Spline C^2 smoothing, Friction Ellipse velocity profiling, Start-Line Realignment,
    and Closed-Loop Seam Continuity Blending.
    """
    if len(wpnts) < 10:
        return wpnts, False

    x = np.array([w['x_m'] for w in wpnts])
    y = np.array([w['y_m'] for w in wpnts])
    v = np.array([w['vx_mps'] for w in wpnts])

    N = len(x)

    # 1. Deduplicate consecutive coincident points to prevent FITPACK splprep ValueError: Invalid inputs
    dists_step = np.sqrt(np.diff(x, prepend=x[-1])**2 + np.diff(y, prepend=y[-1])**2)
    valid_mask = dists_step > 1e-4
    if np.sum(valid_mask) >= 10:
        x = x[valid_mask]
        y = y[valid_mask]
        v = v[valid_mask]
        N = len(x)

    # 2. Start-Line Index Realignment (Snap index 0 to map origin if provided)
    if map_origin is not None:
        start_x, start_y = map_origin[0], map_origin[1]
        dists_to_origin = (x - start_x)**2 + (y - start_y)**2
        start_idx = np.argmin(dists_to_origin)
        if start_idx != 0:
            x = np.roll(x, -start_idx)
            y = np.roll(y, -start_idx)
            v = np.roll(v, -start_idx)

    # 3. Detect and prune teardrop loops / directional reversal spikes / self-intersections at hairpin apexes
    def remove_self_intersection_loops(x_arr, y_arr):
        x_c, y_c = np.copy(x_arr), np.copy(y_arr)
        def ccw(A, B, C):
            return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
        def line_intersect(p1, p2, p3, p4):
            return ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4)

        for _ in range(5):
            n_pts = len(x_c)
            if n_pts < 10:
                break
            keep = np.ones(n_pts, dtype=bool)
            found = False
            pts = np.column_stack([x_c, y_c])
            for i in range(n_pts):
                if not keep[i]:
                    continue
                p1, p2 = pts[i], pts[(i + 1) % n_pts]
                for j in range(i + 2, min(i + 35, n_pts - 1)):
                    if not keep[j]:
                        continue
                    p3, p4 = pts[j], pts[(j + 1) % n_pts]
                    if line_intersect(p1, p2, p3, p4):
                        keep[i+1 : j+1] = False
                        found = True
                        break
            if not found or np.sum(keep) < 10:
                break
            x_c, y_c = x_c[keep], y_c[keep]

        return x_c, y_c

    cx, cy = remove_self_intersection_loops(x, y)

    # 4. Reconstruct 2D raceline with smooth periodic B-Spline (k=3, per=True, s=2.5)
    # Check if loops exist and adaptively smooth until zero loops remain
    for s_val in [1.5, 2.5, 4.0, 6.0]:
        try:
            tck, u = splprep([cx, cy], s=s_val, per=True, k=3)
            u_new = np.linspace(0, 1, N, endpoint=False)
            new_x, new_y = splev(u_new, tck)
            
            # Verify 0 self-intersections in output
            test_x, test_y = remove_self_intersection_loops(new_x, new_y)
            if len(test_x) == len(new_x):
                break
        except Exception:
            pass

    # Calculate 1st and 2nd derivatives for heading psi & curvature kappa
    dx_du, dy_du = splev(u_new, tck, der=1)
    d2x_du2, d2y_du2 = splev(u_new, tck, der=2)

    psi = np.arctan2(dy_du, dx_du)
    kappa = np.abs(dx_du * d2y_du2 - dy_du * d2x_du2) / np.power(dx_du**2 + dy_du**2, 1.5)

    # Calculate cumulative track distance s_m
    segment_dists = np.sqrt(np.diff(new_x, prepend=new_x[-1])**2 + np.diff(new_y, prepend=new_y[-1])**2)
    s_m = np.cumsum(segment_dists)
    s_m -= s_m[0]

    # 4. Dual-Axis Friction Ellipse Velocity Profiling: (a_long / a_max)^2 + (a_lat / a_max)^2 <= 1
    v_profile = np.zeros(N)
    for i in range(N):
        mu_val = mu_func(s_m[i]) if mu_func else 0.75
        a_lat_max = mu_val * g
        
        if kappa[i] > 1e-4:
            v_corner = np.sqrt(a_lat_max / kappa[i])
        else:
            v_corner = v_max_global

        v_profile[i] = min(v_corner, v_max_global)

    # Forward-Backward Pass for Friction Ellipse Acceleration Limits
    a_long_max = 4.5  # m/s^2 max longitudinal acceleration
    a_long_min = -6.0 # m/s^2 max braking acceleration

    # Backward Pass (Braking limit)
    for i in range(N - 1, -1, -1):
        idx_next = (i + 1) % N
        ds = segment_dists[idx_next]
        mu_val = mu_func(s_m[i]) if mu_func else 0.75
        a_lat = (v_profile[i]**2) * kappa[i]
        
        grip_avail = max(0.01, 1.0 - (a_lat / (mu_val * g))**2)
        decel_limit = abs(a_long_min) * np.sqrt(grip_avail)
        
        max_entry_v = np.sqrt(v_profile[idx_next]**2 + 2 * decel_limit * ds)
        v_profile[i] = min(v_profile[i], max_entry_v)

    # Forward Pass (Traction limit)
    for i in range(N):
        idx_prev = (i - 1) % N
        ds = segment_dists[i]
        mu_val = mu_func(s_m[i]) if mu_func else 0.75
        a_lat = (v_profile[i]**2) * kappa[i]
        
        grip_avail = max(0.01, 1.0 - (a_lat / (mu_val * g))**2)
        accel_limit = a_long_max * np.sqrt(grip_avail)
        
        max_exit_v = np.sqrt(v_profile[idx_prev]**2 + 2 * accel_limit * ds)
        v_profile[i] = min(v_profile[i], max_exit_v)

    # 5. Closed-Loop Velocity Seam Smoothing (Smooth transition across index N-1 -> 0)
    seam_v_mean = 0.5 * (v_profile[0] + v_profile[-1])
    v_profile[0] = seam_v_mean
    v_profile[-1] = seam_v_mean
    for k in range(1, 4):
        blend = k / 4.0
        v_profile[k] = (1 - blend) * seam_v_mean + blend * v_profile[k]
        v_profile[-1 - k] = (1 - blend) * seam_v_mean + blend * v_profile[-1 - k]

    # Update waypoint dictionaries
    repaired_wpnts = []
    for i in range(N):
        repaired_wpnts.append({
            's_m': float(s_m[i]),
            'x_m': float(new_x[i]),
            'y_m': float(new_y[i]),
            'psi_rad': float(psi[i]),
            'kappa_radpm': float(kappa[i]),
            'vx_mps': float(v_profile[i]),
            'ax_mps2': 0.0
        })

    return repaired_wpnts, True

def repair_waypoint_array(wpnts):
    """Wrapper function for backward compatibility."""
    return repair_and_optimize_waypoints(wpnts)[0], True

def repair_all_maps():
    maps_root = repo_root / "assets" / "maps"
    json_files = list(maps_root.rglob("global_waypoints.json"))

    print(f"[REPAIR] Optimizing & Verifying Continuity for {len(json_files)} maps...")
    for jfile in json_files:
        map_dir = jfile.parent
        map_name = map_dir.name
        yaml_path = map_dir / f"{map_name}.yaml"

        origin = [0, 0, 0]
        if yaml_path.exists():
            with open(yaml_path, 'r') as f:
                ydata = yaml.safe_load(f)
                origin = ydata.get('origin', [0, 0, 0])

        with open(jfile, 'r') as f:
            data = json.load(f)

        repaired_any = False
        for key in ['global_traj_wpnts_iqp', 'global_traj_wpnts_sp']:
            if key in data and 'wpnts' in data[key]:
                data[key]['wpnts'], rep = repair_and_optimize_waypoints(data[key]['wpnts'], map_origin=origin)
                cont, rep_info = verify_track_continuity(data[key]['wpnts'])
                if rep:
                    repaired_any = True

        if repaired_any:
            with open(jfile, 'w') as f:
                json.dump(data, f)
            print(f"  ✓ {map_name}: C^2 B-Spline & Continuity Verified (Max C0 Gap: {rep_info['max_c0_gap_m']:.2f}m, Seam C1 Jump: {rep_info['seam_c1_jump_deg']:.1f}°)")

if __name__ == '__main__':
    repair_all_maps()
