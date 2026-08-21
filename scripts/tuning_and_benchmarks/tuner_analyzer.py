#!/usr/bin/env python3

import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
from f110_msgs.msg import WpntArray, OTWpntArray, ObstacleArray, LapData

from scipy.spatial import cKDTree

class TunerAnalyzer(Node):
    def __init__(self):
        super().__init__('tuner_analyzer')
        
        # Lightweight QoS profiles
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Subscriptions (Zero-Overhead Memory Appends)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/car_state/pose',
            self.pose_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/car_state/odom',
            self.odom_callback,
            10
        )
        
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped,
            '/drive',
            self.drive_callback,
            10
        )
        
        self.waypoints_sub = self.create_subscription(
            WpntArray,
            '/global_waypoints',
            self.waypoints_callback,
            qos_best_effort
        )

        self.state_sub = self.create_subscription(
            String,
            '/state',
            self.state_callback,
            10
        )

        self.otwpnts_sub = self.create_subscription(
            OTWpntArray,
            '/planner/avoidance/otwpnts',
            self.otwpnts_callback,
            10
        )

        self.obstacles_sub = self.create_subscription(
            ObstacleArray,
            '/perception/obstacles',
            self.obstacles_callback,
            10
        )

        self.lap_data_sub = self.create_subscription(
            LapData,
            '/lap_data',
            self.lap_data_callback,
            10
        )

        # Telemetry storage
        self.waypoints = None
        self.kdtree = None
        self.pose_history = []
        self.odom_history = []
        self.drive_history = []
        self.state_history = []
        self.avoidance_history = []
        self.obstacle_history = []
        self.lap_history = []
        
        self.report_file_path = "/tmp/f1tenth_telemetry_report.txt"
        self.report_timer = self.create_timer(3.0, self.generate_report)
        
        self.last_pose_time = 0.0
        self.start_time = time.time()
        self.get_logger().info("=========================================")
        self.get_logger().info(" F1TENTH High-Precision Analyzer Active")
        self.get_logger().info(" Writing live report to /tmp/f1tenth_telemetry_report.txt")
        self.get_logger().info(" Waiting for global waypoints...")
        self.get_logger().info("=========================================")

    def waypoints_callback(self, msg):
        if self.waypoints is None:
            self.waypoints = np.array([[wp.x_m, wp.y_m] for wp in msg.wpnts])
            self.kdtree = cKDTree(self.waypoints)
            self.get_logger().info(f"Received {len(self.waypoints)} global waypoints. Built O(log N) cKDTree index!")

    def pose_callback(self, msg):
        if self.kdtree is None:
            return
        
        t = time.time()
        # Rate-throttle to 20Hz max for ZERO CPU overhead on Jetson (< 2 microseconds per query)
        if t - self.last_pose_time < 0.05:
            return
        self.last_pose_time = t

        x = msg.pose.position.x
        y = msg.pose.position.y
        
        # O(log N) ultra-fast cKDTree spatial query
        lat_err, min_idx = self.kdtree.query([x, y])
        
        self.pose_history.append((t, x, y, lat_err, min_idx))

    def odom_callback(self, msg):
        t = time.time()
        speed = msg.twist.twist.linear.x
        self.odom_history.append((t, speed))

    def drive_callback(self, msg):
        t = time.time()
        steer = msg.drive.steering_angle
        cmd_speed = msg.drive.speed
        self.drive_history.append((t, steer, cmd_speed))

    def state_callback(self, msg):
        t = time.time()
        self.state_history.append((t, msg.data))

    def otwpnts_callback(self, msg):
        t = time.time()
        num_wpnts = len(msg.otwpnts) if hasattr(msg, 'otwpnts') else 0
        side = getattr(msg, 'side', 'unknown')
        self.avoidance_history.append((t, num_wpnts, side))

    def obstacles_callback(self, msg):
        t = time.time()
        num_obs = len(msg.obstacles) if hasattr(msg, 'obstacles') else 0
        self.obstacle_history.append((t, num_obs))

    def lap_data_callback(self, msg):
        t = time.time()
        lap_count = getattr(msg, 'lap_count', 0)
        lap_time = getattr(msg, 'lap_time', 0.0)
        avg_lat_err = getattr(msg, 'avg_lat_err', 0.0)
        max_lat_err = getattr(msg, 'max_lat_err', 0.0)
        self.lap_history.append((t, lap_count, lap_time, avg_lat_err, max_lat_err))

    def generate_report(self):
        duration = time.time() - self.start_time
        self.get_logger().info("\n\n")
        self.get_logger().info("==================================================================")
        self.get_logger().info("                F1TENTH RUN ANALYSIS REPORT                       ")
        self.get_logger().info("==================================================================")
        self.get_logger().info(f"Run Duration: {duration:.2f} seconds")
        
        if not self.pose_history:
            self.get_logger().info("No pose/tracking data collected. Make sure the car stack is active.")
            return

        # 1. Lateral Error Metrics
        lat_errors = [p[3] for p in self.pose_history]
        avg_lat_err = np.mean(lat_errors)
        max_lat_err = np.max(lat_errors)
        std_lat_err = np.std(lat_errors)
        
        self.get_logger().info("------------------------------------------------------------------")
        self.get_logger().info(" TRACKING METRICS:")
        self.get_logger().info(f"  Average Lateral Error (MAE): {avg_lat_err:.3f} m")
        self.get_logger().info(f"  Maximum Lateral Error (Max): {max_lat_err:.3f} m")
        self.get_logger().info(f"  Standard Deviation of Error: {std_lat_err:.3f} m")
        
        # 2. Speed metrics
        avg_speed = 0.0
        max_speed = 0.0
        if self.odom_history:
            speeds = [o[1] for o in self.odom_history]
            avg_speed = np.mean(speeds)
            max_speed = np.max(speeds)
            self.get_logger().info(f"  Average Speed:               {avg_speed:.2f} m/s")
            self.get_logger().info(f"  Maximum Speed:               {max_speed:.2f} m/s")

        # 3. Steering Oscillation Metrics
        oscillation_rate = 0.0
        if len(self.drive_history) > 2:
            steers = np.array([d[1] for d in self.drive_history])
            times = np.array([d[0] for d in self.drive_history])
            
            # Count direction changes larger than 0.02 radians
            diffs = np.diff(steers)
            sign_changes = 0
            prev_diff = 0
            for val in diffs:
                if abs(val) > 0.02: # Significant movement
                    if (val > 0 and prev_diff < 0) or (val < 0 and prev_diff > 0):
                        sign_changes += 1
                    prev_diff = val
            
            oscillation_rate = sign_changes / duration if duration > 0 else 0
            self.get_logger().info(f"  Steering Direction Changes:  {sign_changes} times")
            self.get_logger().info(f"  Oscillation Frequency:       {oscillation_rate:.2f} Hz (changes/sec)")

        # 4. Speed & Acceleration PID Metrics
        speed_lag = 0.0
        ss_error = 0.0
        speed_overshoot = 0.0
        if self.odom_history and self.drive_history:
            odom_t = [o[0] for o in self.odom_history]
            odom_v = [o[1] for o in self.odom_history]
            
            cmd_t = [d[0] for d in self.drive_history]
            cmd_v = [d[2] for d in self.drive_history]
            
            try:
                interp_actual_v = np.interp(cmd_t, odom_t, odom_v)
                raw_errors = np.array(cmd_v) - interp_actual_v
                speed_errors = np.abs(raw_errors)
                
                speed_lag = np.mean(speed_errors)
                ss_error = np.mean(raw_errors)  # Positive means actual < cmd (under-speed)
                speed_overshoot = np.max(interp_actual_v - np.array(cmd_v)) if len(cmd_v) > 0 else 0.0
                
                self.get_logger().info(f"  Average Speed Tracking Lag (P-Err): {speed_lag:.3f} m/s")
                self.get_logger().info(f"  Steady-State Velocity Bias (I-Err): {ss_error:.3f} m/s")
                self.get_logger().info(f"  Max Speed Overshoot (D-Err):       {max(0.0, speed_overshoot):.3f} m/s")
            except Exception:
                pass

        # 5. Avoidance & State Machine Metrics
        self.get_logger().info("------------------------------------------------------------------")
        self.get_logger().info(" AVOIDANCE & STATE MACHINE METRICS:")
        if self.state_history:
            unique_states = sorted(list(set([s[1] for s in self.state_history])))
            self.get_logger().info(f"  State Transitions Logged:    {len(self.state_history)} (States: {', '.join(unique_states)})")
        else:
            self.get_logger().info("  State Machine:               No state messages received.")

        if self.avoidance_history:
            self.get_logger().info(f"  Spline Avoidance Triggers:   {len(self.avoidance_history)} times")
            avg_ot_wpnts = np.mean([a[1] for a in self.avoidance_history])
            self.get_logger().info(f"  Avg Avoidance Traj Length:  {avg_ot_wpnts:.1f} waypoints")
        else:
            self.get_logger().info("  Spline Avoidance:            No avoidance paths triggered.")

        if self.obstacle_history:
            obs_counts = [o[1] for o in self.obstacle_history if o[1] > 0]
            if obs_counts:
                self.get_logger().info(f"  Obstacles Detected:          {len(obs_counts)} instances (Max: {max(obs_counts)} obstacles)")
            else:
                self.get_logger().info("  Obstacles Detected:          None (clear track)")

        # 6. Lap Data Metrics
        self.get_logger().info("------------------------------------------------------------------")
        self.get_logger().info(" LAP ANALYZER DATA (/lap_data):")
        if self.lap_history:
            self.get_logger().info(f"  Total Laps Completed:        {len(self.lap_history)}")
            for i, lap in enumerate(self.lap_history):
                _, l_cnt, l_time, a_err, m_err = lap
                self.get_logger().info(f"    Lap {l_cnt if l_cnt > 0 else i+1}: Time = {l_time:.3f}s | Avg Error = {a_err:.3f}m | Max Error = {m_err:.3f}m")
        else:
            self.get_logger().info("  Lap Data:                    No lap completions recorded yet.")

        self.get_logger().info("==================================================================")
        self.get_logger().info("                  TUNING RECOMMENDATIONS                          ")
        self.get_logger().info("==================================================================")
        
        recommendations = []
        
        # Rule 1: Lookahead oscillation check
        if oscillation_rate > 1.8:
            recommendations.append(
                "⚠️ STEERING OSCILLATION DETECTED:\n"
                "   Your lookahead distance is too short, causing the steering to over-correct and oscillate.\n"
                "   -> Increase 't_clip_min' by 0.05m (e.g. 0.45 -> 0.50) in l1_params.yaml.\n"
                "   -> Alternatively, increase 'q_l1' (base lookahead) by 0.03."
            )
        
        # Rule 2: Corner cutting or slow response
        if avg_lat_err > 0.15 and oscillation_rate <= 1.2:
            recommendations.append(
                "📐 CORNER CUTTING / UNDERSTEER DETECTED:\n"
                "   The car is tracking stably but stays too far away from the global path (cutting corners).\n"
                "   -> Decrease lookahead distance to make steering more aggressive.\n"
                "   -> Lower 'm_l1' by 0.10 (e.g. 0.90 -> 0.80) or 'q_l1' by 0.04 in l1_params.yaml."
            )
            
        # Rule 3: High maximum error (overshooting in turns)
        if max_lat_err > 0.35:
            recommendations.append(
                "💥 CORNER OVERSHOOT / LARGE PEAK ERROR:\n"
                "   The car is overshooting or deviating heavily at specific high-curvature sections.\n"
                "   -> Increase 'speed_lookahead' by 0.05 (e.g. 0.25 -> 0.30) to start braking earlier.\n"
                "   -> Lower the speed scaling in the map's 'speed_scaling.yaml' for the sharp turn sectors."
            )

        # Rule 4: Steering authority downscale at speed
        if max_speed > 6.0 and avg_lat_err > 0.12:
            recommendations.append(
                "🏎️ HIGH-SPEED STEERING SLIP:\n"
                "   At speeds > 6 m/s, your lateral error is high. This can be caused by steer downscaling.\n"
                "   -> In 'l1_params.yaml', decrease 'downscale_factor' (e.g. 0.20 -> 0.10) to give more steering authority."
            )

        # Rule 5: Speed controller PID tuning checks
        if speed_lag > 0.4:
            if ss_error > 0.3:
                recommendations.append(
                    "⚙️ VELOCITY CONTROLLER UNDER-SPEED (P/I GAIN TOO LOW):\n"
                    "   Car drives consistently slower than commanded velocity.\n"
                    "   -> Increase 'vel_ctr_p_gain' in 'l1_params.yaml' (e.g., 1.8 -> 2.2).\n"
                    "   -> Slightly increase 'vel_ctr_i_gain' (e.g., 0.0 -> 0.05) to eliminate steady-state lag."
                )
            elif speed_overshoot > 0.3:
                recommendations.append(
                    "⚡ VELOCITY CONTROLLER OVERSHOOT (D GAIN TOO LOW / P TOO HIGH):\n"
                    "   Motor speed surges or overshoots commanded target velocity.\n"
                    "   -> Increase 'vel_ctr_d_gain' in 'l1_params.yaml' (e.g., 0.2 -> 0.4) for damping.\n"
                    "   -> Slightly reduce 'vel_ctr_p_gain' if motor chatter occurs."
                )
            else:
                recommendations.append(
                    "⚙️ VELOCITY CONTROLLER LAG:\n"
                    "   The actual speed is lagging behind commanded speed.\n"
                    "   -> Increase 'vel_ctr_p_gain' in 'l1_params.yaml' (e.g., 1.8 -> 2.2) to make acceleration snappier.\n"
                    "   -> Verify VESC battery voltage or current limits."
                )

        if not recommendations:
            recommendations.append(
                "✅ EXCELLENT TUNING!\n"
                "   Your lateral error is low, steering is stable, and speed tracking is tight.\n"
                "   To push the car faster, gradually increase sector scaling values in speed_scaling.yaml."
            )
            
        report_lines = []
        report_lines.append("==================================================================")
        report_lines.append("                F1TENTH RUN ANALYSIS REPORT                       ")
        report_lines.append("==================================================================")
        report_lines.append(f"Run Duration: {duration:.2f} seconds")
        report_lines.append("------------------------------------------------------------------")
        report_lines.append(" TRACKING METRICS:")
        report_lines.append(f"  Average Lateral Error (MAE): {avg_lat_err:.3f} m")
        report_lines.append(f"  Maximum Lateral Error (Max): {max_lat_err:.3f} m")
        report_lines.append(f"  Standard Deviation of Error: {std_lat_err:.3f} m")
        if self.odom_history:
            report_lines.append(f"  Average Speed:               {avg_speed:.2f} m/s")
            report_lines.append(f"  Maximum Speed:               {max_speed:.2f} m/s")
        if len(self.drive_history) > 2:
            report_lines.append(f"  Steering Direction Changes:  {sign_changes} times")
            report_lines.append(f"  Oscillation Frequency:       {oscillation_rate:.2f} Hz (changes/sec)")
        if self.odom_history and self.drive_history:
            report_lines.append(f"  Average Speed Tracking Lag (P-Err): {speed_lag:.3f} m/s")
            report_lines.append(f"  Steady-State Velocity Bias (I-Err): {ss_error:.3f} m/s")
            report_lines.append(f"  Max Speed Overshoot (D-Err):       {max(0.0, speed_overshoot):.3f} m/s")
        report_lines.append("------------------------------------------------------------------")
        report_lines.append(" AVOIDANCE & STATE MACHINE METRICS:")
        if self.state_history:
            unique_states = sorted(list(set([s[1] for s in self.state_history])))
            report_lines.append(f"  State Transitions Logged:    {len(self.state_history)} (States: {', '.join(unique_states)})")
        else:
            report_lines.append("  State Machine:               No state messages received.")

        if self.avoidance_history:
            report_lines.append(f"  Spline Avoidance Triggers:   {len(self.avoidance_history)} times")
            avg_ot_wpnts = np.mean([a[1] for a in self.avoidance_history])
            report_lines.append(f"  Avg Avoidance Traj Length:  {avg_ot_wpnts:.1f} waypoints")
        else:
            report_lines.append("  Spline Avoidance:            No avoidance paths triggered.")

        if self.obstacle_history:
            obs_counts = [o[1] for o in self.obstacle_history if o[1] > 0]
            if obs_counts:
                report_lines.append(f"  Obstacles Detected:          {len(obs_counts)} instances (Max: {max(obs_counts)} obstacles)")
            else:
                report_lines.append("  Obstacles Detected:          None (clear track)")

        report_lines.append("------------------------------------------------------------------")
        report_lines.append(" LAP ANALYZER DATA (/lap_data):")
        if self.lap_history:
            report_lines.append(f"  Total Laps Completed:        {len(self.lap_history)}")
            for i, lap in enumerate(self.lap_history):
                _, l_cnt, l_time, a_err, m_err = lap
                report_lines.append(f"    Lap {l_cnt if l_cnt > 0 else i+1}: Time = {l_time:.3f}s | Avg Error = {a_err:.3f}m | Max Error = {m_err:.3f}m")
        else:
            report_lines.append("  Lap Data:                    No lap completions recorded yet.")

        report_lines.append("==================================================================")
        report_lines.append("                  TUNING RECOMMENDATIONS                          ")
        report_lines.append("==================================================================")
        for rec in recommendations:
            report_lines.append(rec)
            report_lines.append("------------------------------------------------------------------")
        report_lines.append("==================================================================")

        report_text = "\n".join(report_lines)
        try:
            with open(self.report_file_path, "w") as f:
                f.write(report_text + "\n")
        except Exception:
            pass

import signal

def main(args=None):
    rclpy.init(args=args)
    node = TunerAnalyzer()

    def handle_shutdown(signum, frame):
        node.generate_report()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node.generate_report()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
