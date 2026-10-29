#!/usr/bin/env python3
"""
Interactive Start Line & AMCL Initial Pose Selector for F1TENTH

Usage:
  python3 scripts/set_start_line.py track4

Features:
- Does NOT allow editing/dragging individual waypoints.
- Click anywhere on the track to set where laps are counted (Waypoint 0 / s = 0.0).
- Displays the Start/Finish line with a green marker and heading direction.
- Automatically saves:
  1. global_waypoints.json & <map_name>_raceline.csv (shifted Waypoint 0)
  2. amcl.yaml initial_pose (x, y, yaw) across src and install directories
  3. Broadcasts live /initialpose topic to ROS 2 if AMCL is active.
"""

import sys
import os
import json
import yaml
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from PIL import Image
import subprocess

def find_repo_root():
    curr = os.path.dirname(os.path.abspath(__file__))
    while curr != os.path.dirname(curr):
        if os.path.isdir(os.path.join(curr, "assets", "maps")):
            return curr
        curr = os.path.dirname(curr)
    return os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

PROJECT_ROOT = find_repo_root()

def load_map_and_waypoints(map_name):
    map_dir = os.path.join(PROJECT_ROOT, "assets", "maps", map_name)
    if not os.path.exists(map_dir):
        print(f"Error: Map directory does not exist: {map_dir}")
        sys.exit(1)

    # Subdirectories
    wp_dir = os.path.join(map_dir, "waypoints")
    gen_dir = os.path.join(map_dir, "waypoint_gen")
    clean_dir = os.path.join(map_dir, "clean")

    # Locate global_waypoints.json strictly in waypoints/
    json_path = os.path.join(wp_dir, "global_waypoints.json")
    if not os.path.exists(json_path):
        print(f"Error: Could not find global_waypoints.json at strict path: {json_path}")
        sys.exit(1)

    with open(json_path, 'r') as f:
        wp_data = json.load(f)

    # Locate YAML and map image strictly in clean/ or waypoint_gen/
    yaml_path = os.path.join(clean_dir, f"{map_name}.yaml")
    if not os.path.exists(yaml_path):
        yaml_path = os.path.join(gen_dir, f"{map_name}_waypoint_gen.yaml")

    map_img = None
    extent = None
    if os.path.exists(yaml_path):
        try:
            with open(yaml_path, 'r') as f:
                map_meta = yaml.safe_load(f)
            img_filename = map_meta.get("image", f"{map_name}.png")
            parent_dir = os.path.dirname(yaml_path)
            img_path = os.path.join(parent_dir, img_filename)
            if os.path.exists(img_path):
                map_img = Image.open(img_path)
                map_img = np.array(map_img)
                res = map_meta.get("resolution", 0.05)
                origin = map_meta.get("origin", [0.0, 0.0, 0.0])
                h, w = map_img.shape[:2]
                extent = [origin[0], origin[0] + w * res, origin[1], origin[1] + h * res]
        except Exception as e:
            print(f"Warning: Could not load map image background ({e})")

    return json_path, wp_data, map_img, extent, map_name

class StartLineSelector:
    def __init__(self, json_path, data, map_img=None, extent=None, map_name="track4"):
        self.json_path = json_path
        self.data = data
        self.map_img = map_img
        self.extent = extent
        self.map_name = map_name

        # Extract main raceline coordinates
        self.key = "global_traj_wpnts_iqp" if "global_traj_wpnts_iqp" in data else "global_traj_wpnts_sp"
        if self.key not in data or "wpnts" not in data[self.key]:
            self.key = "wpnts"

        pts = []
        for wp in data[self.key]["wpnts"]:
            pts.append([wp["x_m"], wp["y_m"]])
        self.coords = np.array(pts, dtype=float)
        self.orig_coords = self.coords.copy()

        self.start_idx = 0

        # Plot GUI
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        plt.subplots_adjust(bottom=0.15)
        
        self.ax.set_title(f"Start Line & AMCL Pose Selector ({map_name})\nClick anywhere on the raceline to set Start Line (Waypoint 0)", fontsize=11)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True, linestyle='--', alpha=0.5)
        self.ax.set_aspect('equal')

        if self.map_img is not None and self.extent is not None:
            self.ax.imshow(np.flipud(self.map_img), cmap='gray', extent=self.extent, origin='lower', alpha=0.6)

        self.ax.plot(self.coords[:, 0], self.coords[:, 1], 'b-', linewidth=2.5, label='Raceline')

        self.start_marker = None
        self.quiver = None
        self.draw_start_marker()

        self.fig.canvas.mpl_connect('button_press_event', self.on_click)

        ax_reset = plt.axes([0.30, 0.03, 0.18, 0.06])
        ax_save = plt.axes([0.52, 0.03, 0.22, 0.06])

        self.btn_reset = Button(ax_reset, 'Reset to Default')
        self.btn_save = Button(ax_save, 'Save & Set Pose')

        self.btn_reset.on_clicked(self.reset_start_line)
        self.btn_save.on_clicked(self.save_and_apply)

    def draw_start_marker(self):
        if self.start_marker is not None:
            self.start_marker.remove()
            self.start_marker = None
        if self.quiver is not None:
            self.quiver.remove()
            self.quiver = None

        if len(self.coords) > 0:
            idx = self.start_idx % len(self.coords)
            next_idx = (idx + 1) % len(self.coords)
            x0, y0 = self.coords[idx, 0], self.coords[idx, 1]
            dx = self.coords[next_idx, 0] - x0
            dy = self.coords[next_idx, 1] - y0
            heading = np.arctan2(dy, dx)

            self.start_marker = self.ax.scatter(x0, y0, c='lime', s=180, marker='*', zorder=10, label=f'Start Line (Wp 0)')
            self.quiver = self.ax.quiver(x0, y0, np.cos(heading), np.sin(heading), color='lime', scale=15, zorder=11)
            self.ax.legend(loc='upper right')
            self.fig.canvas.draw_idle()

    def on_click(self, event):
        if event.inaxes != self.ax or event.xdata is None:
            return
        click_x, click_y = float(event.xdata), float(event.ydata)
        dists = np.hypot(self.coords[:, 0] - click_x, self.coords[:, 1] - click_y)
        self.start_idx = int(np.argmin(dists))
        print(f"--> Selected Start Line at Waypoint {self.start_idx}: ({self.coords[self.start_idx, 0]:.3f}, {self.coords[self.start_idx, 1]:.3f})")
        self.draw_start_marker()

    def reset_start_line(self, event):
        self.start_idx = 0
        print("--> Reset Start Line to default Waypoint 0.")
        self.draw_start_marker()

    def save_and_apply(self, event):
        shift_idx = self.start_idx % len(self.coords)
        print(f"\n--> Saving Start Line shift by {shift_idx} waypoints...")

        # 1. Roll JSON waypoint arrays and marker arrays and recalculate monotonic s_m
        for k in ["global_traj_wpnts_iqp", "global_traj_wpnts_sp", "centerline_waypoints"]:
            if k in self.data and "wpnts" in self.data[k]:
                arr = self.data[k]["wpnts"]
                s_shift = shift_idx % len(arr)
                arr = arr[s_shift:] + arr[:s_shift]
                
                pts_k = np.array([[w["x_m"], w["y_m"]] for w in arr])
                dx_k = np.diff(pts_k[:, 0], append=pts_k[0, 0])
                dy_k = np.diff(pts_k[:, 1], append=pts_k[0, 1])
                ds_k = np.maximum(np.hypot(dx_k, dy_k), 0.01)
                sm_k = np.cumsum(ds_k) - ds_k[0]
                for idx, w in enumerate(arr):
                    w["id"] = idx
                    w["s_m"] = round(float(sm_k[idx]), 4)
                self.data[k]["wpnts"] = arr

        marker_keys = {
            "global_traj_wpnts_iqp": "global_traj_markers_iqp",
            "global_traj_wpnts_sp": "global_traj_markers_sp",
            "centerline_waypoints": "centerline_markers"
        }
        for wp_k, mk_k in marker_keys.items():
            if mk_k in self.data and "markers" in self.data[mk_k] and wp_k in self.data and "wpnts" in self.data[wp_k]:
                m_arr = self.data[mk_k]["markers"]
                s_shift = shift_idx % len(m_arr)
                m_arr = m_arr[s_shift:] + m_arr[:s_shift]
                for idx, m in enumerate(m_arr):
                    m["id"] = idx
                self.data[mk_k]["markers"] = m_arr

        # 2. Save JSON & CSV files to waypoints/ subfolder
        target_dirs = [
            os.path.join(PROJECT_ROOT, "assets", "maps", self.map_name, "waypoints"),
            os.path.join(PROJECT_ROOT, "assets", "maps", self.map_name),
        ]

        for target_dir in target_dirs:
            if not os.path.exists(target_dir):
                continue
            
            # Save global_waypoints.json
            json_file = os.path.join(target_dir, "global_waypoints.json")
            with open(os.path.realpath(json_file), 'w') as f:
                json.dump(self.data, f)

            # Save raceline CSV
            raceline_csv = os.path.join(target_dir, f"{self.map_name}_raceline.csv")
            pts = np.roll(self.coords, -shift_idx, axis=0)
            num_pts = len(pts)
            dx = np.diff(pts[:, 0], append=pts[0, 0])
            dy = np.diff(pts[:, 1], append=pts[0, 1])
            ds = np.hypot(dx, dy)
            ds = np.maximum(ds, 0.01)
            s_m = np.cumsum(ds) - ds[0]
            psi = np.arctan2(dy, dx)

            vx = np.full(num_pts, 3.0)
            kappa = np.zeros(num_pts)
            ax = np.zeros(num_pts)

            if self.key in self.data and "wpnts" in self.data[self.key]:
                for i, wp in enumerate(self.data[self.key]["wpnts"]):
                    if i < num_pts:
                        vx[i] = wp.get("vx_mps", 3.0)
                        kappa[i] = wp.get("kappa_radpm", 0.0)
                        ax[i] = wp.get("ax_mps2", 0.0)

            with open(os.path.realpath(raceline_csv), 'w', newline='') as f:
                w = csv.writer(f, delimiter=';')
                w.writerow(['# Raceline with shifted Waypoint 0'])
                w.writerow(['# format: s_m;x_m;y_m;psi_rad;kappa_radpm;vx_mps;ax_mps2'])
                w.writerow(['s_m', 'x_m', 'y_m', 'psi_rad', 'kappa_radpm', 'vx_mps', 'ax_mps2'])
                for i in range(num_pts):
                    w.writerow([
                        round(float(s_m[i]), 4),
                        round(float(pts[i, 0]), 4),
                        round(float(pts[i, 1]), 4),
                        round(float(psi[i]), 4),
                        round(float(kappa[i]), 4),
                        round(float(vx[i]), 4),
                        round(float(ax[i]), 4)
                    ])

        # 3. Save AMCL initial pose to amcl.yaml across src & install paths
        pts = np.roll(self.coords, -shift_idx, axis=0)
        dx_0 = pts[1, 0] - pts[0, 0]
        dy_0 = pts[1, 1] - pts[0, 1]
        yaw_0 = float(np.arctan2(dy_0, dx_0))
        x_0 = round(float(pts[0, 0]), 4)
        y_0 = round(float(pts[0, 1]), 4)

        amcl_paths = [
            os.path.join(PROJECT_ROOT, "src", "race_bringup", "config", "amcl.yaml"),
            os.path.join(PROJECT_ROOT, "out", "install", "race_bringup", "share", "race_bringup", "config", "amcl.yaml"),
            os.path.join(PROJECT_ROOT, "install", "race_bringup", "share", "race_bringup", "config", "amcl.yaml")
        ]

        for amcl_path in amcl_paths:
            if os.path.exists(amcl_path):
                try:
                    with open(amcl_path, 'r') as f:
                        amcl_cfg = yaml.safe_load(f)
                    if "amcl" in amcl_cfg and "ros__parameters" in amcl_cfg["amcl"]:
                        params = amcl_cfg["amcl"]["ros__parameters"]
                        params["set_initial_pose"] = True
                        params["initial_pose"] = {
                            "x": x_0,
                            "y": y_0,
                            "z": 0.0,
                            "yaw": round(yaw_0, 4)
                        }
                    with open(amcl_path, 'w') as f:
                        yaml.dump(amcl_cfg, f, default_flow_style=False, sort_keys=False)
                    print(f"Saved AMCL initial pose to {amcl_path}")
                except Exception as e:
                    print(f"Warning: Could not update {amcl_path} ({e})")

        # 4. Broadcast live ROS 2 /initialpose
        try:
            qz = float(np.sin(yaw_0 / 2.0))
            qw = float(np.cos(yaw_0 / 2.0))
            pub_cmd = (
                f"source /opt/ros/humble/setup.bash && ros2 topic pub -w 1 --once /initialpose "
                f"geometry_msgs/msg/PoseWithCovarianceStamped "
                f"\"{{header: {{frame_id: 'map'}}, pose: {{pose: {{position: {{x: {x_0}, y: {y_0}, z: 0.0}}, "
                f"orientation: {{z: {qz:.4f}, w: {qw:.4f}}}}}}}}}\" > /dev/null 2>&1"
            )
            subprocess.Popen(pub_cmd, shell=True, executable='/bin/bash')
            print(f"Broadcasted live ROS 2 /initialpose: (x={x_0}, y={y_0}, yaw={yaw_0:.4f})")
        except Exception as e:
            print(f"Could not publish live /initialpose: {e}")

        print(f"\nSUCCESS! Applied Start Line at Waypoint {shift_idx} (x={x_0}, y={y_0}, yaw={yaw_0:.4f})!")
        plt.close(self.fig)

def main():
    map_name = "track4"
    if len(sys.argv) > 1:
        map_name = sys.argv[1]
        
    json_path, data, map_img, extent, map_name = load_map_and_waypoints(map_name)
    selector = StartLineSelector(json_path, data, map_img, extent, map_name)
    plt.show()

if __name__ == "__main__":
    main()
