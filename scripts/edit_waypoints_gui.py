#!/usr/bin/env python3
"""
Interactive Multi-Trajectory Waypoint Editor for F1TENTH
Supports editing ALL trajectories in global_waypoints.json:
- Minimum Curvature (IQP Main Raceline)
- Shortest Path (SP)
- Centerline

Saves modifications to:
1. global_waypoints.json (in assets/maps and install share path)
2. <map_name>_raceline.csv (used by gym_bridge / simulation / controller)
3. <map_name>_centerline.csv
"""

import sys
import os
import json
import yaml
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, RadioButtons
from PIL import Image
from scipy.interpolate import splprep, splev

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def load_map_and_waypoints(map_name):
    map_dir = os.path.join(PROJECT_ROOT, "assets", "maps", map_name)
    if not os.path.exists(map_dir):
        map_dir = os.path.join(PROJECT_ROOT, "out", "install", "race_bringup", "share", "race_bringup", "maps", map_name)
        
    json_path = os.path.join(map_dir, "global_waypoints.json")
    yaml_path = os.path.join(map_dir, f"{map_name}.yaml")
    
    if not os.path.exists(json_path):
        print(f"Error: Could not find global_waypoints.json at {json_path}")
        sys.exit(1)
        
    with open(json_path, 'r') as f:
        wp_data = json.load(f)
        
    map_img = None
    extent = None
    if os.path.exists(yaml_path):
        try:
            with open(yaml_path, 'r') as f:
                map_meta = yaml.safe_load(f)
            img_filename = map_meta.get("image", f"{map_name}.pgm")
            img_path = os.path.join(map_dir, img_filename)
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

class MultiWaypointEditor:
    def __init__(self, json_path, data, map_img=None, extent=None, map_name="track4"):
        self.json_path = json_path
        self.data = data
        self.map_img = map_img
        self.extent = extent
        self.map_name = map_name
        self.selected_point = None

        # Discover available trajectory keys
        self.available_trajs = {}
        if "global_traj_wpnts_iqp" in data and "wpnts" in data["global_traj_wpnts_iqp"]:
            self.available_trajs["MinCurv (IQP)"] = "global_traj_wpnts_iqp"
        if "global_traj_wpnts_sp" in data and "wpnts" in data["global_traj_wpnts_sp"]:
            self.available_trajs["Shortest Path (SP)"] = "global_traj_wpnts_sp"
        if "centerline_waypoints" in data and "wpnts" in data["centerline_waypoints"]:
            self.available_trajs["Centerline"] = "centerline_waypoints"
        if not self.available_trajs:
            self.available_trajs["Default Line"] = "wpnts"

        self.current_label = list(self.available_trajs.keys())[0]

        # Load all coordinates into a dictionary
        self.coords_dict = {}
        self.orig_coords_dict = {}
        
        for label, key in self.available_trajs.items():
            pts = []
            if key in data and "wpnts" in data[key]:
                for wp in data[key]["wpnts"]:
                    pts.append([wp["x_m"], wp["y_m"]])
            self.coords_dict[label] = np.array(pts, dtype=float)
            self.orig_coords_dict[label] = np.array(pts, dtype=float).copy()

        self.coords = self.coords_dict[self.current_label]

        # Set up GUI plot
        self.fig, self.ax = plt.subplots(figsize=(11, 9))
        plt.subplots_adjust(bottom=0.18, left=0.22)
        
        self.ax.set_title("Interactive Multi-Trajectory Waypoint Editor\n(Select line, Click & Drag points away from walls, click 'Save All')", fontsize=11)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True, linestyle='--', alpha=0.5)
        self.ax.set_aspect('equal')

        if self.map_img is not None and self.extent is not None:
            self.ax.imshow(np.flipud(self.map_img), cmap='gray', extent=self.extent, origin='lower', alpha=0.6)

        orig_active = self.orig_coords_dict[self.current_label]
        self.orig_line, = self.ax.plot(orig_active[:, 0], orig_active[:, 1], 'cyan', linestyle='--', label='Original Line', alpha=0.7)
        self.line, = self.ax.plot(self.coords[:, 0], self.coords[:, 1], 'r-', linewidth=2, label=f'Active: {self.current_label}')
        self.scat = self.ax.scatter(self.coords[:, 0], self.coords[:, 1], c='blue', s=20, picker=True, pickradius=8, zorder=5)

        self.ax.legend(loc='upper right')

        self.fig.canvas.mpl_connect('pick_event', self.on_pick)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)

        ax_radio = plt.axes([0.02, 0.40, 0.18, 0.20])
        self.radio = RadioButtons(ax_radio, list(self.available_trajs.keys()))
        self.radio.on_clicked(self.on_traj_select)

        ax_smooth = plt.axes([0.25, 0.04, 0.20, 0.06])
        ax_reset = plt.axes([0.50, 0.04, 0.20, 0.06])
        ax_save = plt.axes([0.75, 0.04, 0.20, 0.06])

        self.btn_smooth = Button(ax_smooth, 'Smooth Curve')
        self.btn_reset = Button(ax_reset, 'Reset Active')
        self.btn_save = Button(ax_save, 'Save All to JSON/CSV')

        self.btn_smooth.on_clicked(self.smooth_trajectory)
        self.btn_reset.on_clicked(self.reset_trajectory)
        self.btn_save.on_clicked(self.save_trajectory)

    def on_traj_select(self, label):
        self.current_label = label
        self.coords = self.coords_dict[self.current_label]
        orig = self.orig_coords_dict[self.current_label]
        
        self.orig_line.set_xdata(orig[:, 0])
        self.orig_line.set_ydata(orig[:, 1])
        self.line.set_label(f'Active: {label}')
        self.ax.legend(loc='upper right')
        self.update_plot()

    def on_pick(self, event):
        if event.artist != self.scat:
            return
        ind = event.ind[0]
        self.selected_point = ind

    def on_motion(self, event):
        if self.selected_point is None or event.inaxes != self.ax:
            return
        
        idx = self.selected_point
        dx = event.xdata - self.coords[idx, 0]
        dy = event.ydata - self.coords[idx, 1]

        num_pts = len(self.coords)
        nudge_radius = 6
        for k in range(-nudge_radius, nudge_radius + 1):
            neighbor_idx = (idx + k) % num_pts
            weight = np.exp(-0.5 * (k / (nudge_radius / 2.0)) ** 2)
            self.coords[neighbor_idx, 0] += dx * weight
            self.coords[neighbor_idx, 1] += dy * weight

        self.update_plot()

    def on_release(self, event):
        self.selected_point = None

    def update_plot(self):
        self.line.set_xdata(self.coords[:, 0])
        self.line.set_ydata(self.coords[:, 1])
        self.scat.set_offsets(self.coords)
        self.fig.canvas.draw_idle()

    def smooth_trajectory(self, event):
        try:
            tck, u = splprep([self.coords[:, 0], self.coords[:, 1]], s=0.02, per=True)
            u_new = np.linspace(0, 1, len(self.coords))
            x_new, y_new = splev(u_new, tck)
            self.coords[:, 0] = x_new
            self.coords[:, 1] = y_new
            self.update_plot()
            print(f"Smoothed '{self.current_label}' curve with B-spline!")
        except Exception as e:
            print(f"Smoothing error: {e}")

    def reset_trajectory(self, event):
        self.coords = self.orig_coords_dict[self.current_label].copy()
        self.coords_dict[self.current_label] = self.coords
        self.update_plot()
        print(f"Reset '{self.current_label}' to original points.")

    def save_trajectory(self, event):
        # Update JSON dictionary
        for label, key in self.available_trajs.items():
            pts = self.coords_dict[label]
            if key in self.data and "wpnts" in self.data[key]:
                for i, wp in enumerate(self.data[key]["wpnts"]):
                    if i < len(pts):
                        wp["x_m"] = float(pts[i, 0])
                        wp["y_m"] = float(pts[i, 1])

        if "Centerline" in self.coords_dict and "centerline_markers" in self.data and "markers" in self.data["centerline_markers"]:
            pts = self.coords_dict["Centerline"]
            for i, m in enumerate(self.data["centerline_markers"]["markers"]):
                if i < len(pts):
                    m["pose"]["position"]["x"] = float(pts[i, 0])
                    m["pose"]["position"]["y"] = float(pts[i, 1])

        # Define map directories (assets and install build share)
        asset_dir = os.path.join(PROJECT_ROOT, "assets", "maps", self.map_name)
        install_dir = os.path.join(PROJECT_ROOT, "out", "install", "race_bringup", "share", "race_bringup", "maps", self.map_name)
        
        target_dirs = [asset_dir]
        if os.path.exists(install_dir):
            target_dirs.append(install_dir)

        for target_dir in target_dirs:
            if not os.path.exists(target_dir):
                continue
                
            # 1. Save global_waypoints.json
            json_file = os.path.join(target_dir, "global_waypoints.json")
            real_json = os.path.realpath(json_file)
            with open(real_json, 'w') as f:
                json.dump(self.data, f)
            print(f"Saved JSON: {real_json}")

            # 2. Save <map_name>_raceline.csv (used by gym_bridge / simulation)
            raceline_csv = os.path.join(target_dir, f"{self.map_name}_raceline.csv")
            real_csv = os.path.realpath(raceline_csv)
            
            pts = self.coords_dict.get("MinCurv (IQP)")
            if pts is None:
                pts = self.coords_dict.get("Shortest Path (SP)")

            if pts is not None:
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

                key = "global_traj_wpnts_iqp" if "global_traj_wpnts_iqp" in self.data else "global_traj_wpnts_sp"
                if key in self.data and "wpnts" in self.data[key]:
                    for i, wp in enumerate(self.data[key]["wpnts"]):
                        if i < num_pts:
                            vx[i] = wp.get("vx_mps", 3.0)
                            kappa[i] = wp.get("kappa_radpm", 0.0)
                            ax[i] = wp.get("ax_mps2", 0.0)

                with open(real_csv, 'w', newline='') as f:
                    w = csv.writer(f, delimiter=';')
                    w.writerow(['# Shortest path / IQP raceline generated from global_waypoints.json'])
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
                print(f"Saved Raceline CSV: {real_csv}")

            # 3. Save <map_name>_centerline.csv
            if "Centerline" in self.coords_dict:
                c_pts = self.coords_dict["Centerline"]
                cent_csv = os.path.realpath(os.path.join(target_dir, f"{self.map_name}_centerline.csv"))
                with open(cent_csv, 'w', newline='') as f:
                    w = csv.writer(f)
                    w.writerow(['x_m', 'y_m'])
                    for i in range(len(c_pts)):
                        w.writerow([round(float(c_pts[i, 0]), 4), round(float(c_pts[i, 1]), 4)])
                print(f"Saved Centerline CSV: {cent_csv}")

        print(f"\nSUCCESS: Saved all modified JSON and CSV files for '{self.map_name}'!")
        plt.close(self.fig)

def main():
    map_name = "track4"
    if len(sys.argv) > 1:
        map_name = sys.argv[1]
        
    json_path, data, map_img, extent, map_name = load_map_and_waypoints(map_name)
    editor = MultiWaypointEditor(json_path, data, map_img, extent, map_name)
    plt.show()

if __name__ == "__main__":
    main()
