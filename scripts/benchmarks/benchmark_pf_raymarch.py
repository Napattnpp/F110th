#!/usr/bin/env python3
import os
import sys
import time
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml
from pathlib import Path

# ROS 2 & RangeLibc
import range_libc

class ParticleFilterBenchmarker:
    def __init__(self, map_path, raceline_csv_path, output_dir):
        self.map_path = Path(map_path)
        self.raceline_csv_path = Path(raceline_csv_path)
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Load Map
        print(f"[INFO] Loading map YAML: {self.map_path}")
        with open(self.map_path, 'r') as f:
            self.map_info = yaml.safe_load(f)
        
        img_name = self.map_info['image']
        img_path = self.map_path.parent / img_name
        
        import cv2
        img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(f"Map image not found: {img_path}")
        img = cv2.flip(img, 0)
        
        self.resolution = float(self.map_info['resolution'])
        self.origin = [float(x) for x in self.map_info['origin']]
        self.height, self.width = img.shape
        
        from nav_msgs.msg import OccupancyGrid
        grid_msg = OccupancyGrid()
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        grid_msg.info.resolution = self.resolution
        grid_msg.info.origin.position.x = self.origin[0]
        grid_msg.info.origin.position.y = self.origin[1]
        grid_msg.info.origin.orientation.w = 1.0
        
        flat_img = img.flatten()
        grid_data = np.zeros_like(flat_img, dtype=np.int8)
        grid_data[flat_img < 128] = 100 # Wall
        grid_data[flat_img >= 128] = 0   # Free track
        grid_msg.data = grid_data.tolist()
        
        # Setup RangeLibc PyOMap & Ray Marching
        print("[INFO] Initializing RangeLibc PyOMap and RayMarching...")
        self.omap = range_libc.PyOMap(grid_msg)
        self.max_range_meters = 10.0
        self.max_range_px = int(self.max_range_meters / self.resolution)
        
        if range_libc.SHOULD_USE_CUDA:
            print("[INFO] RangeLibc CUDA Acceleration ACTIVE!")
            self.range_method = range_libc.PyRayMarchingGPU(self.omap, self.max_range_px)
        else:
            print("[WARN] RangeLibc CUDA disabled, using CPU RayMarching")
            self.range_method = range_libc.PyRayMarching(self.omap, self.max_range_px)

        self.num_rays = 108
        self.fov_min = -math.radians(135)
        self.fov_max = math.radians(135)
        self.ray_angles = np.linspace(self.fov_min, self.fov_max, self.num_rays, dtype=np.float32)
        
        self.table_width = self.max_range_px + 1
        sensor_model_lut = self._precompute_sensor_model(
            z_hit=0.75, z_short=0.01, z_max=0.07, z_rand=0.12, sigma_hit=3.0
        )
        self.range_method.set_sensor_model(sensor_model_lut)

        # Load Raceline Trajectory
        print(f"[INFO] Loading raceline trajectory from {self.raceline_csv_path}")
        raw_rows = []
        with open(self.raceline_csv_path, 'r') as f:
            for line in f:
                line_str = line.strip()
                if not line_str or line_str.startswith('#') or 's_m' in line_str or 'x_m' in line_str:
                    continue
                parts = [float(val) for val in line_str.replace('"', '').split(';')]
                if len(parts) >= 3:
                    raw_rows.append(parts)
                    
        data = np.array(raw_rows)
        # Format: s_m (0), x_m (1), y_m (2), psi_rad (3)
        if data.shape[1] >= 4:
            self.gt_x = data[:, 1]
            self.gt_y = data[:, 2]
            self.gt_theta = data[:, 3]
        else:
            self.gt_x = data[:, 0]
            self.gt_y = data[:, 1]
            dx = np.diff(self.gt_x)
            dy = np.diff(self.gt_y)
            self.gt_theta = np.arctan2(dy, dx)
            self.gt_theta = np.append(self.gt_theta, self.gt_theta[-1])
        
        self.trajectory_length = len(self.gt_x)
        print(f"[INFO] Trajectory loaded: {self.trajectory_length} waypoints.")

    def _precompute_sensor_model(self, z_hit, z_short, z_max, z_rand, sigma_hit):
        table = np.zeros((self.table_width, self.table_width), dtype=np.float64)
        sigma_px = sigma_hit / self.resolution
        
        for r_exp in range(self.table_width):
            for r_meas in range(self.table_width):
                p_hit = math.exp(-0.5 * ((r_meas - r_exp) / sigma_px) ** 2) / (math.sqrt(2 * math.pi) * sigma_px)
                p_short = (2.0 / r_exp * (1.0 - r_meas / r_exp)) if (r_exp > 0 and r_meas <= r_exp) else 0.0
                p_max = 1.0 if r_meas == (self.table_width - 1) else 0.0
                p_rand = 1.0 / (self.table_width - 1)
                
                prob = z_hit * p_hit + z_short * p_short + z_max * p_max + z_rand * p_rand
                table[r_meas, r_exp] = prob
                
            s = np.sum(table[r_exp, :])
            if s > 0:
                table[r_exp, :] /= s
        return table

    def simulate_lidar_scan(self, pose):
        query = np.array([[pose[0], pose[1], pose[2]]], dtype=np.float32)
        ranges = np.zeros(self.num_rays, dtype=np.float32)
        self.range_method.calc_range_repeat_angles(query, self.ray_angles, ranges)
        ranges += np.random.normal(0.0, 0.03, size=self.num_rays)
        ranges = np.clip(ranges, 0.0, self.max_range_meters)
        return ranges

    def run_benchmark(self, config_name, num_particles=3000, motion_params=None):
        print(f"\n==================================================")
        print(f" Running Evaluation Scenario: {config_name}")
        print(f"==================================================")
        
        if motion_params is None:
            motion_params = {
                'disp_x': 0.05,
                'disp_y': 0.025,
                'disp_theta': 0.20,
                'squash': 1.8
            }

        particles = np.zeros((num_particles, 3), dtype=np.float32)
        start_x, start_y, start_theta = self.gt_x[0], self.gt_y[0], self.gt_theta[0]
        
        particles[:, 0] = start_x + np.random.normal(0.0, 0.20, num_particles)
        particles[:, 1] = start_y + np.random.normal(0.0, 0.20, num_particles)
        particles[:, 2] = start_theta + np.random.normal(0.0, 0.10, num_particles)
        weights = np.ones(num_particles, dtype=np.float64) / num_particles

        estimated_poses = []
        gt_poses = []
        pos_errors = []
        theta_errors = []
        runtimes = []

        inv_squash = 1.0 / motion_params['squash']

        for i in range(1, self.trajectory_length):
            gt_prev = np.array([self.gt_x[i-1], self.gt_y[i-1], self.gt_theta[i-1]])
            gt_curr = np.array([self.gt_x[i], self.gt_y[i], self.gt_theta[i]])
            
            dx_global = gt_curr[0] - gt_prev[0]
            dy_global = gt_curr[1] - gt_prev[1]
            dtheta = (gt_curr[2] - gt_prev[2] + np.pi) % (2 * np.pi) - np.pi
            
            noisy_dx_local = math.cos(-gt_prev[2]) * dx_global - math.sin(-gt_prev[2]) * dy_global + np.random.normal(0.0, 0.015)
            noisy_dy_local = math.sin(-gt_prev[2]) * dx_global + math.cos(-gt_prev[2]) * dy_global + np.random.normal(0.0, 0.01)
            noisy_dtheta = dtheta + np.random.normal(0.0, 0.02)

            t0 = time.perf_counter()
            
            cos_p = np.cos(particles[:, 2])
            sin_p = np.sin(particles[:, 2])
            
            nx = noisy_dx_local + np.random.normal(0.0, motion_params['disp_x'], num_particles)
            ny = noisy_dy_local + np.random.normal(0.0, motion_params['disp_y'], num_particles)
            nth = noisy_dtheta + np.random.normal(0.0, motion_params['disp_theta'], num_particles)

            particles[:, 0] += cos_p * nx - sin_p * ny
            particles[:, 1] += sin_p * nx + cos_p * ny
            particles[:, 2] = (particles[:, 2] + nth + np.pi) % (2 * np.pi) - np.pi
            obs_scan = self.simulate_lidar_scan(gt_curr)
            obs_ranges = obs_scan.astype(np.float32)
            
            if hasattr(self.range_method, 'calc_range_many_eval_sensor_gpu'):
                particle_ranges_px = np.zeros(num_particles * self.num_rays, dtype=np.uint16)
                self.range_method.calc_range_many_eval_sensor_gpu(
                    particles, (obs_ranges / self.resolution).astype(np.int32), weights, particle_ranges_px
                )
            else:
                self.range_method.calc_range_repeat_angles_eval_sensor_model(
                    particles, self.ray_angles, obs_ranges, weights
                )

            weights = np.power(weights, inv_squash)
            weight_sum = np.sum(weights)
            if weight_sum > 0:
                weights /= weight_sum
            else:
                weights.fill(1.0 / num_particles)

            best_idx = np.argmax(weights)
            best_p = particles[best_idx]
            
            dists = np.hypot(particles[:, 0] - best_p[0], particles[:, 1] - best_p[1])
            near_mask = dists < 0.5
            if np.sum(near_mask) > 5:
                w_near = weights[near_mask] / np.sum(weights[near_mask])
                p_near = particles[near_mask]
                est_x = np.sum(p_near[:, 0] * w_near)
                est_y = np.sum(p_near[:, 1] * w_near)
                est_th = math.atan2(np.sum(np.sin(p_near[:, 2]) * w_near), np.sum(np.cos(p_near[:, 2]) * w_near))
            else:
                est_x, est_y, est_th = best_p[0], best_p[1], best_p[2]

            ess = 1.0 / np.sum(weights ** 2)
            if ess < (0.8 * num_particles):
                cdf = np.cumsum(weights)
                step = 1.0 / num_particles
                rand_start = np.random.uniform(0.0, step)
                indices = np.zeros(num_particles, dtype=np.int32)
                
                i_eff = 0
                for j in range(num_particles):
                    val = rand_start + j * step
                    while val > cdf[i_eff] and i_eff < num_particles - 1:
                        i_eff += 1
                    indices[j] = i_eff
                    
                particles = particles[indices]
                particles[:, 0] += np.random.normal(0.0, 0.005, num_particles)
                particles[:, 1] += np.random.normal(0.0, 0.005, num_particles)
                particles[:, 2] += np.random.normal(0.0, 0.005, num_particles)
                weights.fill(1.0 / num_particles)

            t1 = time.perf_counter()
            runtimes.append((t1 - t0) * 1000.0)

            pos_err = math.hypot(est_x - gt_curr[0], est_y - gt_curr[1])
            th_err = abs((est_th - gt_curr[2] + np.pi) % (2 * np.pi) - np.pi)
            
            gt_poses.append(gt_curr)
            estimated_poses.append([est_x, est_y, est_th])
            pos_errors.append(pos_err)
            theta_errors.append(math.degrees(th_err))

        gt_poses = np.array(gt_poses)
        estimated_poses = np.array(estimated_poses)
        pos_errors = np.array(pos_errors)
        theta_errors = np.array(theta_errors)
        
        rmse_pos = np.sqrt(np.mean(pos_errors**2))
        rmse_th = np.sqrt(np.mean(theta_errors**2))
        max_err = np.max(pos_errors)
        avg_runtime = np.mean(runtimes)
        
        print(f" Result Metrics for [{config_name}]:")
        print(f"   - Position RMSE:    {rmse_pos:.4f} m")
        print(f"   - Heading RMSE:     {rmse_th:.2f} deg")
        print(f"   - Max Position Err: {max_err:.4f} m")
        print(f"   - Avg Execution:    {avg_runtime:.2f} ms ({1000.0/avg_runtime:.1f} Hz)")

        return {
            'name': config_name,
            'gt': gt_poses,
            'est': estimated_poses,
            'pos_errors': pos_errors,
            'theta_errors': theta_errors,
            'rmse_pos': rmse_pos,
            'rmse_th': rmse_th,
            'max_err': max_err,
            'avg_runtime': avg_runtime
        }

    def generate_plots(self, results):
        print("\n[INFO] Generating comparison plots...")
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        
        ax_map = axes[0, 0]
        ax_map.set_title("Trajectory Comparison: Ground Truth vs Particle Filter", fontsize=12, fontweight='bold')
        
        gt0 = results[0]['gt']
        ax_map.plot(gt0[:, 0], gt0[:, 1], 'k--', linewidth=2.5, label='Ground Truth Path', alpha=0.8)
        
        colors = ['#e74c3c', '#3498db', '#2ecc71']
        for idx, res in enumerate(results):
            est = res['est']
            ax_map.plot(est[:, 0], est[:, 1], color=colors[idx % len(colors)], linewidth=1.8, 
                        label=f"{res['name']} (RMSE: {res['rmse_pos']:.3f}m)", alpha=0.9)
            
        ax_map.set_xlabel("X [m]")
        ax_map.set_ylabel("Y [m]")
        ax_map.grid(True, linestyle=':', alpha=0.6)
        ax_map.legend(loc='best')
        ax_map.axis('equal')

        ax_err = axes[0, 1]
        ax_err.set_title("Position Tracking Error Over Time", fontsize=12, fontweight='bold')
        for idx, res in enumerate(results):
            ax_err.plot(res['pos_errors'], color=colors[idx % len(colors)], linewidth=1.5, label=res['name'], alpha=0.85)
        ax_err.set_xlabel("Waypoint Step")
        ax_err.set_ylabel("Position Error [m]")
        ax_err.grid(True, linestyle=':', alpha=0.6)
        ax_err.legend(loc='best')

        ax_th = axes[1, 0]
        ax_th.set_title("Heading Angle Tracking Error", fontsize=12, fontweight='bold')
        for idx, res in enumerate(results):
            ax_th.plot(res['theta_errors'], color=colors[idx % len(colors)], linewidth=1.5, label=res['name'], alpha=0.85)
        ax_th.set_xlabel("Waypoint Step")
        ax_th.set_ylabel("Heading Error [deg]")
        ax_th.grid(True, linestyle=':', alpha=0.6)
        ax_th.legend(loc='best')

        ax_bar = axes[1, 1]
        ax_bar.set_title("Summary Metric Comparison", fontsize=12, fontweight='bold')
        names = [r['name'] for r in results]
        rmses = [r['rmse_pos'] for r in results]
        maxs = [r['max_err'] for r in results]
        
        x = np.arange(len(names))
        width = 0.35
        
        rects1 = ax_bar.bar(x - width/2, rmses, width, label='Position RMSE (m)', color='#3498db')
        rects2 = ax_bar.bar(x + width/2, maxs, width, label='Max Error (m)', color='#e74c3c')
        
        ax_bar.set_xticks(x)
        ax_bar.set_xticklabels(names)
        ax_bar.set_ylabel("Error [m]")
        ax_bar.grid(True, linestyle=':', alpha=0.6)
        ax_bar.legend(loc='best')
        
        for rect in rects1 + rects2:
            height = rect.get_height()
            ax_bar.annotate(f'{height:.3f}m',
                            xy=(rect.get_x() + rect.get_width() / 2, height),
                            xytext=(0, 3),
                            textcoords="offset points",
                            ha='center', va='bottom', fontsize=9)

        plt.tight_layout()
        plot_path = self.output_dir / "pf_error_analysis.png"
        plt.savefig(plot_path, dpi=200)
        plt.close()
        print(f"[✓] Saved benchmark plot to: {plot_path}")

        plt.figure(figsize=(10, 8))
        plt.title("Particle Filter Raceline Tracking Visualization (ifac_roboracer)", fontsize=14, fontweight='bold')
        plt.plot(gt0[:, 0], gt0[:, 1], 'k--', linewidth=3.0, label='Ground Truth Raceline', alpha=0.7)
        tuned_res = results[-1]
        plt.plot(tuned_res['est'][:, 0], tuned_res['est'][:, 1], color='#27ae60', linewidth=2.0, label=f"Tuned PF Pose (RMSE: {tuned_res['rmse_pos']:.3f}m)")
        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.grid(True, linestyle='--', alpha=0.5)
        plt.legend(loc='best', fontsize=11)
        plt.axis('equal')
        
        traj_plot_path = self.output_dir / "pf_trajectory_comparison.png"
        plt.savefig(traj_plot_path, dpi=200)
        plt.close()
        print(f"[✓] Saved trajectory visualization to: {traj_plot_path}")


def _find_repo_root() -> Path:
    curr = Path(__file__).resolve().parent
    for _ in range(10):
        if (curr / 'assets' / 'maps').exists() or (curr / '.git').exists():
            return curr
        if curr.parent == curr:
            break
        curr = curr.parent
    return Path.cwd()

def main():
    repo_root = _find_repo_root()
    map_yaml = repo_root / "assets" / "maps" / "ifac_roboracer" / "clean" / "ifac_roboracer.yaml"
    raceline_csv = repo_root / "assets" / "maps" / "ifac_roboracer" / "waypoints" / "ifac_roboracer_raceline.csv"
    output_dir = repo_root / "assets" / "maps_visualization"

    benchmarker = ParticleFilterBenchmarker(map_yaml, raceline_csv, output_dir)

    res_baseline = benchmarker.run_benchmark(
        config_name="Baseline (Untuned)",
        num_particles=2000,
        motion_params={'disp_x': 0.12, 'disp_y': 0.08, 'disp_theta': 0.35, 'squash': 2.5}
    )

    res_cand_a = benchmarker.run_benchmark(
        config_name="Config A (Default Arc)",
        num_particles=3000,
        motion_params={'disp_x': 0.06, 'disp_y': 0.03, 'disp_theta': 0.20, 'squash': 2.0}
    )

    res_tuned = benchmarker.run_benchmark(
        config_name="Config B (Optimized Arc)",
        num_particles=4000,
        motion_params={'disp_x': 0.035, 'disp_y': 0.018, 'disp_theta': 0.12, 'squash': 1.5}
    )

    res_fine = benchmarker.run_benchmark(
        config_name="Config C (Fine-Tuned Arc)",
        num_particles=4000,
        motion_params={'disp_x': 0.015, 'disp_y': 0.008, 'disp_theta': 0.05, 'squash': 1.1}
    )

    benchmarker.generate_plots([res_baseline, res_cand_a, res_tuned, res_fine])

if __name__ == '__main__':
    main()
