'''
Shared functions to read and write map information (global waypoints)
'''
import os
import json
from pathlib import Path
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields

import numpy as np
from visualization_msgs.msg import MarkerArray
from f110_msgs.msg import WpntArray
from std_msgs.msg import String, Float32
from typing import Tuple, List, Dict


def write_global_waypoints(map_dir: str,
                           map_info_str: str,
                           est_lap_time: Float32,
                           centerline_markers: MarkerArray,
                           centerline_waypoints: WpntArray,
                           global_traj_markers_iqp: MarkerArray,
                           global_traj_wpnts_iqp: WpntArray,
                           global_traj_markers_sp: MarkerArray,
                           global_traj_wpnts_sp: WpntArray,
                           trackbounds_markers: MarkerArray
                           ) -> None:
    '''
    Writes map information to a JSON file in the specified `map_dir` directory.
    - map_info_str
        - from topic /map_infos: python str
    - est_lap_time
        - from topic /estimated_lap_time: Float32
    - centerline_markers
        - from topic /centerline_waypoints/markers: MarkerArray
    - centerline_waypoints
        - from topic /centerline_waypoints: WpntArray
    - global_traj_markers_iqp
        - from topic /global_waypoints: MarkerArray
    - global_traj_wpnts_iqp
        - from topic /global_waypoints/markers: WpntArray
    - global_traj_markers_sp
        - from topic /global_waypoints/shortest_path: MarkerArray
    - global_traj_wpnts_sp
        - from topic /global_waypoitns/shortest_path/markers: WpntArray
    - trackbounds_markers
        - from topic /trackbounds/markers: MarkerArray
    - write_path: Full path of JSON file to write
    '''

    wp_dir = os.path.join(map_dir, 'waypoints')
    os.makedirs(wp_dir, exist_ok=True)
    path = os.path.join(wp_dir, 'global_waypoints.json')

    print(f"[INFO] WRITE_GLOBAL_WAYPOINTS: Writing global waypoints to {path}")

    # Dictionary will be converted into a JSON for serialization
    d: Dict[str, Dict] = {}
    d['map_info_str'] = {'data': map_info_str}
    d['est_lap_time'] = {'data': est_lap_time.data}
    d['centerline_markers'] = message_to_ordereddict(centerline_markers)
    d['centerline_waypoints'] = message_to_ordereddict(centerline_waypoints)
    d['global_traj_markers_iqp'] = message_to_ordereddict(global_traj_markers_iqp)
    d['global_traj_wpnts_iqp'] = message_to_ordereddict(global_traj_wpnts_iqp)
    d['global_traj_markers_sp'] = message_to_ordereddict(global_traj_markers_sp)
    d['global_traj_wpnts_sp'] = message_to_ordereddict(global_traj_wpnts_sp)
    d['trackbounds_markers'] = message_to_ordereddict(trackbounds_markers)

    # serialize
    with open(path, 'w') as f:
        json.dump(d, f)


def _resolve_map_dir(map_dir: str) -> str:
    import pathlib
    p = pathlib.Path(map_dir)
    if (p / 'waypoints' / 'global_waypoints.json').exists() or (p / 'global_waypoints.json').exists():
        return str(p.resolve())

    curr = pathlib.Path(__file__).resolve()
    for _ in range(6):
        if (curr / 'assets' / 'maps').exists():
            break
        curr = curr.parent

    repo_root = curr
    stem = p.name
    candidates = [
        repo_root / 'assets' / 'maps' / map_dir,
        repo_root / 'assets' / 'maps' / stem,
    ]
    for cand in candidates:
        if (cand / 'waypoints' / 'global_waypoints.json').exists() or (cand / 'global_waypoints.json').exists():
            return str(cand.resolve())

    return map_dir


def read_global_waypoints(map_dir: str) -> Tuple[
    String, Float32, MarkerArray, WpntArray, MarkerArray, WpntArray, MarkerArray, WpntArray, MarkerArray
]:
    resolved_dir = _resolve_map_dir(map_dir)
    wp_dir = os.path.join(resolved_dir, 'waypoints') if os.path.exists(os.path.join(resolved_dir, 'waypoints')) else resolved_dir
    path = os.path.join(wp_dir, 'global_waypoints.json')
    if not os.path.exists(path):
        raise FileNotFoundError(f"[ERROR] global_waypoints.json not found at path: {path}")

    print(f"[INFO] READ_GLOBAL_WAYPOINTS: Reading global waypoints from {path}")
    # Deserialize JSON and Reconstruct the maps elements
    with open(path, 'r') as f:
        d: Dict[str, List] = json.load(f)

    map_info_str = String()
    set_message_fields(map_info_str, d['map_info_str'])

    est_lap_time = Float32()
    set_message_fields(est_lap_time, d['est_lap_time'])

    centerline_markers = MarkerArray()
    set_message_fields(centerline_markers, d['centerline_markers'])

    centerline_waypoints = WpntArray()
    set_message_fields(centerline_waypoints, d['centerline_waypoints'])

    global_traj_markers_iqp = MarkerArray()
    set_message_fields(global_traj_markers_iqp, d['global_traj_markers_iqp'])

    global_traj_wpnts_iqp = WpntArray()
    set_message_fields(global_traj_wpnts_iqp, d['global_traj_wpnts_iqp'])

    global_traj_markers_sp = MarkerArray()
    set_message_fields(global_traj_markers_sp, d['global_traj_markers_sp'])

    global_traj_wpnts_sp = WpntArray()
    set_message_fields(global_traj_wpnts_sp, d['global_traj_wpnts_sp'])

    trackbounds_markers = MarkerArray()
    set_message_fields(trackbounds_markers, d['trackbounds_markers'])

    # Ensure raceline waypoints have true physical track boundary distances (d_left and d_right)
    # derived accurately from the centerline boundary envelope rather than naive 1D s-interpolation.
    if len(centerline_waypoints.wpnts) > 0:
        try:
            cl_xy = np.array([[w.x_m, w.y_m] for w in centerline_waypoints.wpnts])
            cl_psi = np.array([w.psi_rad for w in centerline_waypoints.wpnts])
            cl_dl = np.array([w.d_left for w in centerline_waypoints.wpnts])
            cl_dr = np.array([w.d_right for w in centerline_waypoints.wpnts])

            normals = np.column_stack([-np.sin(cl_psi), np.cos(cl_psi)])
            bound_l = cl_xy + normals * cl_dl[:, None]
            bound_r = cl_xy - normals * cl_dr[:, None]

            from scipy.spatial import cKDTree
            tree_l = cKDTree(bound_l)
            tree_r = cKDTree(bound_r)

            for traj_wpnts in [global_traj_wpnts_sp, global_traj_wpnts_iqp]:
                if len(traj_wpnts.wpnts) > 0:
                    traj_xy = np.array([[w.x_m, w.y_m] for w in traj_wpnts.wpnts])
                    dists_l, _ = tree_l.query(traj_xy)
                    dists_r, _ = tree_r.query(traj_xy)
                    for i, w in enumerate(traj_wpnts.wpnts):
                        w.d_left = float(dists_l[i])
                        w.d_right = float(dists_r[i])
        except Exception as ex:
            print(f"[WARN] Failed to compute exact track boundaries from centerline: {ex}")
            cl_s = np.array([w.s_m for w in centerline_waypoints.wpnts])
            cl_dl = np.array([w.d_left for w in centerline_waypoints.wpnts])
            cl_dr = np.array([w.d_right for w in centerline_waypoints.wpnts])
            for traj_wpnts in [global_traj_wpnts_sp, global_traj_wpnts_iqp]:
                if len(traj_wpnts.wpnts) > 0 and traj_wpnts.wpnts[0].d_left == 0.0:
                    for w in traj_wpnts.wpnts:
                        w.d_left = float(np.interp(w.s_m, cl_s, cl_dl))
                        w.d_right = float(np.interp(w.s_m, cl_s, cl_dr))

    for ma in [centerline_markers, global_traj_markers_iqp, global_traj_markers_sp, trackbounds_markers]:
        for m in ma.markers:
            if not m.header.frame_id:
                m.header.frame_id = 'map'

    return map_info_str, est_lap_time, \
        centerline_markers, centerline_waypoints, \
        global_traj_markers_iqp, global_traj_wpnts_iqp, \
        global_traj_markers_sp, global_traj_wpnts_sp, trackbounds_markers
