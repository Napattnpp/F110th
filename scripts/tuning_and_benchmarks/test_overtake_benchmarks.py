#!/usr/bin/env python3
"""
Automated Overtaking Benchmark & Simulation Placement Tool
Enables instant repeatability by placing the ego vehicle behind the opponent
via /initialpose and measuring pass completion rate across track sectors.
"""

import sys
import time
import math
import csv
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from f110_msgs.msg import ObstacleArray


class OvertakeBenchmarkNode(Node):
    def __init__(self, raceline_path: str):
        super().__init__('overtake_benchmark_tool')
        
        # Load raceline
        self.s_arr, self.x_arr, self.y_arr, self.psi_arr, self.k_arr, self.v_arr = self._load_raceline(raceline_path)
        self.track_length = 41.52 # Track length for ifac_roboracer
        self.get_logger().info(f"Loaded raceline: {len(self.s_arr)} waypoints, track length: {self.track_length:.2f}m")

        # Live State Cache
        self.ego_s = None
        self.ego_d = None
        self.ego_vs = None
        
        self.opp_s = None
        self.opp_d = None
        self.opp_vs = None
        
        self.current_state = "UNKNOWN"
        
        # Publisher
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Subscribers
        self.create_subscription(Odometry, '/car_state/frenet/odom', self._frenet_cb, 10)
        self.create_subscription(ObstacleArray, '/perception/obstacles', self._obs_cb, 10)
        self.create_subscription(String, '/state', self._state_cb, 10)

    def _load_raceline(self, path: str):
        s, x, y, psi, k, v = [], [], [], [], [], []
        with open(path, 'r') as f:
            reader = csv.reader(f, delimiter=';')
            for row in reader:
                if not row or row[0].startswith('#') or row[0] == 's_m':
                    continue
                s.append(float(row[0]))
                x.append(float(row[1]))
                y.append(float(row[2]))
                psi.append(float(row[3]))
                k.append(float(row[4]))
                v.append(float(row[5]))
        return np.array(s), np.array(x), np.array(y), np.array(psi), np.array(k), np.array(v)

    def _frenet_cb(self, msg: Odometry):
        self.ego_s = msg.pose.pose.position.x
        self.ego_d = msg.pose.pose.position.y
        self.ego_vs = msg.twist.twist.linear.x

    def _obs_cb(self, msg: ObstacleArray):
        if len(msg.obstacles) > 0:
            obs = msg.obstacles[0]
            self.opp_s = obs.s_center
            self.opp_d = obs.d_center
            self.opp_vs = obs.vs
        else:
            self.opp_s = None

    def _state_cb(self, msg: String):
        self.current_state = msg.data

    def get_waypoint_at_s(self, target_s: float):
        target_s = target_s % self.track_length
        idx = int(np.argmin(np.abs(self.s_arr - target_s)))
        return self.s_arr[idx], self.x_arr[idx], self.y_arr[idx], self.psi_arr[idx], self.k_arr[idx], self.v_arr[idx]

    def place_ego_behind_opponent(self, gap_meters: float = 2.8):
        """Teleport ego vehicle behind current opponent pose along the raceline."""
        if self.opp_s is None:
            self.get_logger().error("Cannot place ego: opponent obstacle not detected yet.")
            return False, 0.0, 0.0

        s_opp = self.opp_s
        s_ego_target = (s_opp - gap_meters) % self.track_length
        s_wpnt, x, y, psi, k, v = self.get_waypoint_at_s(s_ego_target)

        # Build PoseWithCovarianceStamped
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(psi / 2.0)
        msg.pose.pose.orientation.w = math.cos(psi / 2.0)
        
        # Tight covariance for immediate particle filter convergence
        cov = [0.0] * 36
        cov[0] = 0.01   # x var
        cov[7] = 0.01   # y var
        cov[35] = 0.01  # yaw var
        msg.pose.covariance = cov

        self.initialpose_pub.publish(msg)
        return True, s_wpnt, s_opp


def run_single_trial(node: OvertakeBenchmarkNode, gap: float = 2.8, timeout_sec: float = 8.5):
    """Executes a single overtaking trial, logs progression, and determines outcome."""
    ok, s_ego_init, s_opp_init = node.place_ego_behind_opponent(gap_meters=gap)
    if not ok:
        return {"status": "OPPONENT_NOT_FOUND", "elapsed": 0.0, "max_ego_speed": 0.0, "sector_s": 0.0}

    # Allow 0.5s settle time
    t0 = time.time()
    while time.time() - t0 < 0.5:
        rclpy.spin_once(node, timeout_sec=0.05)

    start_time = time.time()
    saw_trailing = False
    saw_overtake = False
    passed_opponent = False
    crashed = False

    ego_speeds = []
    
    while time.time() - start_time < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.05)

        if node.ego_s is None or node.opp_s is None:
            continue

        ego_v = node.ego_vs if node.ego_vs is not None else 0.0
        ego_speeds.append(ego_v)

        # Calculate forward Frenet delta: positive means ego is ahead
        delta_s = (node.ego_s - node.opp_s)
        if delta_s > node.track_length / 2.0:
            delta_s -= node.track_length
        elif delta_s < -node.track_length / 2.0:
            delta_s += node.track_length

        state = node.current_state
        if state == "TRAILING":
            saw_trailing = True
        elif state == "OVERTAKE":
            saw_overtake = True
        elif state == "RECOVERY":
            crashed = True
            break

        elapsed = time.time() - start_time
        if int(elapsed * 10) % 10 == 0:
            sys.stdout.write(f"\r   [{elapsed:4.1f}s] State: {state:<9} | Ego: s={node.ego_s:4.1f}m, v={ego_v:4.2f}m/s | Opp: s={node.opp_s:4.1f}m | Δs={delta_s:+4.2f}m")
            sys.stdout.flush()

        # Pass condition: ego pulled ahead by >= 0.8m and is driving forward cleanly
        if (saw_overtake or saw_trailing) and delta_s > 0.8 and ego_v > 1.2:
            passed_opponent = True
            # Confirm stability for 0.8s
            confirm_t = time.time()
            clean = True
            while time.time() - confirm_t < 0.8:
                rclpy.spin_once(node, timeout_sec=0.05)
                if node.current_state == "RECOVERY":
                    clean = False
                    crashed = True
                    break
            if clean:
                break

    sys.stdout.write("\n")
    sys.stdout.flush()
    total_time = time.time() - start_time
    max_v = max(ego_speeds) if ego_speeds else 0.0

    if crashed:
        status = "CRASH / RECOVERY"
    elif passed_opponent:
        status = "SUCCESSFUL_PASS"
    elif saw_overtake:
        status = "OVERTAKE_STALLED"
    else:
        status = "TRAILING_TIMEOUT"

    return {
        "status": status,
        "elapsed": total_time,
        "saw_overtake": saw_overtake,
        "max_ego_speed": max_v,
        "sector_s": s_opp_init
    }


def main():
    parser = argparse.ArgumentParser(description="Autonomous Overtake Benchmark & Placement Tool")
    parser.add_argument('--raceline', type=str, default='assets/maps/ifac_roboracer/waypoints/ifac_roboracer_raceline.csv')
    parser.add_argument('--trials', type=int, default=5, help='Number of trials')
    parser.add_argument('--gap', type=float, default=2.8, help='Distance behind opponent (m)')
    args = parser.parse_args()

    rclpy.init()
    node = OvertakeBenchmarkNode(args.raceline)

    # Wait for messages
    print("Waiting for /car_state/frenet/odom and /perception/obstacles...")
    t_wait = time.time()
    while (node.ego_s is None or node.opp_s is None) and time.time() - t_wait < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.ego_s is None or node.opp_s is None:
        print("ERROR: Timeout waiting for vehicle telemetry. Make sure simulation is running.")
        sys.exit(1)

    results = []
    print("\n" + "="*75)
    print("STARTING AUTONOMOUS OVERTAKING BENCHMARK SUITE")
    print(f"Total Trials: {args.trials} | Inter-Vehicle Gap: {args.gap}m")
    print("="*75)

    for trial_idx in range(args.trials):
        # Refresh current opponent pose
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)

        s_opp_curr = node.opp_s if node.opp_s is not None else 0.0
        _, _, _, _, kappa, _ = node.get_waypoint_at_s(s_opp_curr)
        zone_type = "Corner (κ >= 0.08)" if abs(kappa) >= 0.08 else "Straight (κ < 0.08)"

        print(f"\n--- [TRIAL {trial_idx + 1}/{args.trials}] Opponent at s={s_opp_curr:.1f}m ({zone_type}, κ={kappa:.3f}) ---")
        res = run_single_trial(node, gap=args.gap, timeout_sec=10.0)
        results.append(res)
        
        status_symbol = "✅" if res["status"] == "SUCCESSFUL_PASS" else "❌"
        print(f"Result: {status_symbol} {res['status']} | Duration: {res['elapsed']:.2f}s | Max Ego Speed: {res['max_ego_speed']:.2f} m/s")

        # Brief settle delay between trials
        time.sleep(1.2)

    # Summary
    success_count = sum(1 for r in results if r["status"] == "SUCCESSFUL_PASS")
    rate = (success_count / len(results)) * 100.0
    print("\n" + "="*75)
    print(f"BENCHMARK SUMMARY: {success_count}/{len(results)} Passes Completed ({rate:.1f}% Success Rate)")
    print("="*75)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
