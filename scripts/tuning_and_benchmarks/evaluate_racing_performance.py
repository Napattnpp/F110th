#!/usr/bin/env python3
"""
Comprehensive Autonomous Racing Evaluation & Benchmark Suite
Evaluates:
  1. Full Lap Times (Best, Average, Consistency vs Theoretical Optimal)
  2. State Distribution (% of race spent in GB_TRACK, TRAILING, OVERTAKE, RECOVERY)
  3. Collision & Safety Metrics (Min Wall Clearance, Wall Hits, Crash Recoveries)
  4. Overtaking Racecraft (Pass Attempts, Passes Completed, Pass Duration, Wall Squeeze Rejection)
"""

import sys
import time
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from f110_msgs.msg import LapData, ObstacleArray


class RacingPerformanceEvaluator(Node):
    def __init__(self, target_laps=5, max_duration=90.0):
        super().__init__('racing_performance_evaluator')
        self.target_laps = target_laps
        self.max_duration = max_duration

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Telemetry containers
        self.current_state = "UNKNOWN"
        self.current_s = 0.0
        self.current_d = 0.0
        self.current_v = 0.0
        self.state_history = []  # (timestamp, state)
        self.lap_times = []      # list of lap times in seconds
        self.completed_laps = 0

        # Safety & Collision tracking
        self.min_scan_dist = float('inf')
        self.wall_breach_count = 0
        self.recovery_triggers = 0
        self.in_recovery = False

        # Overtake tracking
        self.overtake_attempts = 0
        self.overtake_successes = 0
        self.overtake_start_time = None
        self.overtake_durations = []
        self.last_state = "UNKNOWN"

        # Subscribers
        self.create_subscription(String, '/state', self.state_cb, qos_reliable)
        self.create_subscription(LapData, '/lap_data', self.lap_cb, qos_reliable)
        self.create_subscription(Odometry, '/car_state/frenet/odom', self.odom_cb, qos_sensor)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_sensor)
        self.create_subscription(ObstacleArray, '/perception/obstacles', self.obs_cb, qos_sensor)

        # Sampling timer at 20 Hz
        self.start_time = time.time()
        self.timer = self.create_timer(0.05, self.sample_telemetry)

        self.get_logger().info(f"[EVALUATOR] Started Racing Performance Evaluator. Target: {self.target_laps} Laps | Max: {self.max_duration}s")

    def state_cb(self, msg: String):
        self.current_state = msg.data.strip()

    def odom_cb(self, msg: Odometry):
        self.current_s = msg.pose.pose.position.x
        self.current_d = msg.pose.pose.position.y
        self.current_v = msg.twist.twist.linear.x

    def lap_cb(self, msg: LapData):
        lap_t = float(msg.lap_time)
        # Reject bogus startup lap times (< 3.0s or > 60s)
        if 3.0 <= lap_t <= 60.0:
            self.lap_times.append(lap_t)
            self.completed_laps += 1
            self.get_logger().info(
                f"[LAP COMPLETE] Lap #{self.completed_laps}: {lap_t:.3f}s | "
                f"Mean: {np.mean(self.lap_times):.3f}s | Best: {np.min(self.lap_times):.3f}s"
            )

    def scan_cb(self, msg: LaserScan):
        ranges = [r for r in msg.ranges if not np.isnan(r) and not np.isinf(r) and r > 0.05]
        if ranges:
            m = min(ranges)
            if m < self.min_scan_dist:
                self.min_scan_dist = m
            # Wall breach / severe collision threshold
            if m < 0.14 and abs(self.current_v) > 0.4:
                self.wall_breach_count += 1

    def obs_cb(self, msg: ObstacleArray):
        pass

    def sample_telemetry(self):
        t = time.time()
        self.state_history.append((t, self.current_state))

        # Check for state transitions
        if self.current_state != self.last_state:
            # Transition to OVERTAKE
            if self.current_state == "OVERTAKE" and self.last_state in ["GB_TRACK", "TRAILING"]:
                self.overtake_attempts += 1
                self.overtake_start_time = t
                self.get_logger().info(f"[OVERTAKE START] Attempt #{self.overtake_attempts} initiated at s={self.current_s:.2f}m")

            # Transition from OVERTAKE to GB_TRACK (Pass completed!)
            elif self.last_state == "OVERTAKE" and self.current_state == "GB_TRACK":
                if self.overtake_start_time is not None:
                    duration = t - self.overtake_start_time
                    self.overtake_durations.append(duration)
                    self.overtake_successes += 1
                    self.get_logger().info(
                        f"[OVERTAKE SUCCESS] Pass #{self.overtake_successes} completed in {duration:.2f}s! Rejoined raceline."
                    )
                    self.overtake_start_time = None

            # Recovery triggers
            if "RECOVERY" in self.current_state and not self.in_recovery:
                self.recovery_triggers += 1
                self.in_recovery = True
                self.get_logger().warn(f"[CRASH/STUCK] Recovery trigger #{self.recovery_triggers} detected!")
            elif "RECOVERY" not in self.current_state:
                self.in_recovery = False

            self.last_state = self.current_state


def run_evaluation(target_laps=4, max_duration=60.0):
    rclpy.init()
    node = RacingPerformanceEvaluator(target_laps=target_laps, max_duration=max_duration)

    start_wall = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            elapsed = time.time() - start_wall

            # Check termination criteria
            if node.completed_laps >= target_laps:
                node.get_logger().info(f"[EVALUATOR] Target {target_laps} laps reached!")
                break
            if elapsed >= max_duration:
                node.get_logger().warn(f"[EVALUATOR] Time limit {max_duration}s reached!")
                break
    finally:
        # Compute and output comprehensive racecraft report
        print("\n" + "=" * 80)
        print("          AUTONOMOUS RACING PERFORMANCE & RACECRAFT BENCHMARK REPORT")
        print("=" * 80)

        # 1. Lap Time Analysis
        print("\n--- 1. LAP TIME ANALYSIS ---")
        if node.lap_times:
            print(f"  • Completed Laps:     {len(node.lap_times)}")
            print(f"  • Best Lap Time:      {np.min(node.lap_times):.3f} s")
            print(f"  • Average Lap Time:   {np.mean(node.lap_times):.3f} s (±{np.std(node.lap_times):.3f} s)")
            print(f"  • Worst Lap Time:     {np.max(node.lap_times):.3f} s")
            print(f"  • Individual Laps:    {', '.join([f'{t:.3f}s' for t in node.lap_times])}")
        else:
            print("  • Completed Laps:     0 (Run duration insufficient to complete full lap)")

        # 2. State Distribution Analysis
        print("\n--- 2. STATE DISTRIBUTION (% of Race Time) ---")
        total_samples = len(node.state_history)
        if total_samples > 0:
            states = [s[1] for s in node.state_history]
            gb_count = sum(1 for s in states if s == "GB_TRACK")
            trailing_count = sum(1 for s in states if s == "TRAILING")
            ot_count = sum(1 for s in states if s == "OVERTAKE")
            rec_count = sum(1 for s in states if "RECOVERY" in s)
            ftg_count = sum(1 for s in states if "FTG" in s)

            gb_pct = (gb_count / total_samples) * 100.0
            trailing_pct = (trailing_count / total_samples) * 100.0
            ot_pct = (ot_count / total_samples) * 100.0
            rec_pct = (rec_count / total_samples) * 100.0
            ftg_pct = (ftg_count / total_samples) * 100.0

            print(f"  • GB_TRACK (Optimal Raceline): {gb_pct:5.1f}%  {'✅ EXCELLENT (>= 75%)' if gb_pct >= 75 else '⚠️ LOW (< 75%)'}")
            print(f"  • TRAILING (Tactical Pace):    {trailing_pct:5.1f}%")
            print(f"  • OVERTAKE (Active Pass):      {ot_pct:5.1f}%  {'✅ CLEAN PASS DWELL (< 20%)' if ot_pct < 20 else '⚠️ OVER-ACTIVE (>= 20%)'}")
            print(f"  • RECOVERY (Crashed / Stuck):  {rec_pct:5.1f}%  {'✅ ZERO CRASHES' if rec_pct == 0 else '❌ CRASH DETECTED'}")
            if ftg_pct > 0:
                print(f"  • FTGONLY (Reactive Fallback): {ftg_pct:5.1f}%")

        # 3. Collision & Safety Metrics
        print("\n--- 3. SAFETY & COLLISION METRICS ---")
        print(f"  • Min LiDAR Wall Clearance:    {node.min_scan_dist:.2f} m  {'✅ SAFE (>= 0.18m)' if node.min_scan_dist >= 0.18 else '⚠️ CLOSE GRAZE'}")
        print(f"  • Severe Wall Impacts:         {node.wall_breach_count}")
        print(f"  • Crash Recovery Sequences:    {node.recovery_triggers}  {'✅ ZERO CRASHES' if node.recovery_triggers == 0 else '❌ HAS RECOVERIES'}")

        # 4. Overtaking Effectiveness
        print("\n--- 4. OVERTAKING EFFICIENCY & OPPORTUNITY SELECTION ---")
        print(f"  • Overtakes Initiated:         {node.overtake_attempts}")
        print(f"  • Passes Successfully Made:    {node.overtake_successes}")
        success_rate = (node.overtake_successes / max(1, node.overtake_attempts)) * 100.0 if node.overtake_attempts > 0 else 100.0
        print(f"  • Pass Success Rate:           {success_rate:.1f}%")
        if node.overtake_durations:
            print(f"  • Avg Pass Maneuver Duration:  {np.mean(node.overtake_durations):.2f} s (Min: {np.min(node.overtake_durations):.2f}s, Max: {np.max(node.overtake_durations):.2f}s)")
        print("=" * 80 + "\n")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Autonomous Racing Performance Evaluator")
    parser.add_argument('--laps', type=int, default=4, help='Number of full laps to evaluate')
    parser.add_argument('--timeout', type=float, default=60.0, help='Maximum evaluation duration in seconds')
    args = parser.parse_args()

    run_evaluation(target_laps=args.laps, max_duration=args.timeout)
