#!/usr/bin/env python3
import time
import numpy as np
import range_libc
from nav_msgs.msg import OccupancyGrid

def main():
    print("==================================================")
    print("   GPU Particle Filter Benchmark (Jetson Orin)   ")
    print("==================================================")
    
    # 1. Test RangeLibc CUDA status
    print(f"CUDA enabled in RangeLibc: {range_libc.SHOULD_USE_CUDA}")
    if not range_libc.SHOULD_USE_CUDA:
        print("ERROR: RangeLibc is not compiled with CUDA support!")
        return

    # 2. Setup map
    width, height = 400, 400
    resolution = 0.05
    grid_msg = OccupancyGrid()
    grid_msg.info.width = width
    grid_msg.info.height = height
    grid_msg.info.resolution = resolution
    grid_msg.info.origin.position.x = -10.0
    grid_msg.info.origin.position.y = -10.0
    grid_msg.info.origin.orientation.w = 1.0
    grid_msg.data = [0] * (width * height)

    # Border walls
    for x in range(width):
        grid_msg.data[x] = 100
        grid_msg.data[(height - 1) * width + x] = 100
    for y in range(height):
        grid_msg.data[y * width] = 100
        grid_msg.data[y * width + (width - 1)] = 100

    omap = range_libc.PyOMap(grid_msg)
    max_range_px = int(10.0 / resolution)
    rmgpu = range_libc.PyRayMarchingGPU(omap, max_range_px)

    # Precompute sensor table
    sensor_table = np.eye(max_range_px + 1, dtype=np.float64)
    rmgpu.set_sensor_model(sensor_table)

    # 3. Benchmark parameters
    num_particles = 4000
    num_rays = 108
    num_iters = 500

    queries = np.zeros((num_particles, 3), dtype=np.float32)
    queries[:, 0] = np.random.uniform(-5.0, 5.0, num_particles)
    queries[:, 1] = np.random.uniform(-5.0, 5.0, num_particles)
    queries[:, 2] = np.random.uniform(-np.pi, np.pi, num_particles)

    angles = np.linspace(-1.5, 1.5, num_rays, dtype=np.float32)
    ranges = np.zeros(num_particles * num_rays, dtype=np.float32)
    obs = np.full(num_rays, 5.0, dtype=np.float32)
    weights = np.ones(num_particles, dtype=np.float64) / float(num_particles)

    # Warmup
    for _ in range(10):
        rmgpu.calc_range_repeat_angles(queries, angles, ranges)
        rmgpu.eval_sensor_model(obs, ranges, weights, num_rays, num_particles)

    # Timed benchmark: GPU Raymarching
    t0 = time.time()
    for _ in range(num_iters):
        rmgpu.calc_range_repeat_angles(queries, angles, ranges)
    t1 = time.time()
    raymarch_time_ms = ((t1 - t0) / num_iters) * 1000.0
    print(f"GPU Raymarching (4000 particles x 108 rays): {raymarch_time_ms:.3f} ms / frame ({1000.0 / raymarch_time_ms:.1f} Hz)")

    # Timed benchmark: Sensor Model Evaluation
    t0 = time.time()
    for _ in range(num_iters):
        rmgpu.eval_sensor_model(obs, ranges, weights, num_rays, num_particles)
    t1 = time.time()
    sensor_time_ms = ((t1 - t0) / num_iters) * 1000.0
    print(f"Sensor Model Evaluation: {sensor_time_ms:.3f} ms / frame ({1000.0 / sensor_time_ms:.1f} Hz)")

    # Timed benchmark: Vectorized Resampling
    t0 = time.time()
    for _ in range(num_iters):
        positions = (np.random.random() + np.arange(num_particles)) / num_particles
        cumsum = np.cumsum(weights)
        indices = np.searchsorted(cumsum, positions)
        np.clip(indices, 0, num_particles - 1, out=indices)
    t1 = time.time()
    resample_time_ms = ((t1 - t0) / num_iters) * 1000.0
    print(f"Vectorized Low-Variance Resampling: {resample_time_ms:.3f} ms / frame ({1000.0 / resample_time_ms:.1f} Hz)")

    # Total pipeline frame time estimate
    total_frame_ms = raymarch_time_ms + sensor_time_ms + resample_time_ms
    print("--------------------------------------------------")
    print(f"Total Particle Filter Cycle Time: {total_frame_ms:.3f} ms ({1000.0 / total_frame_ms:.1f} Hz)")
    print("==================================================")

if __name__ == "__main__":
    main()
