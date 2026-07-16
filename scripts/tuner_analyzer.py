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
from f110_msgs.msg import WpntArray

class TunerAnalyzer(Node):
    def __init__(self):
        super().__init__('tuner_analyzer')
        
        # QoS profiles to match the publishers
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Subscriptions
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

        # Telemetry storage
        self.waypoints = None
        self.pose_history = []
        self.odom_history = []
        self.drive_history = []
        
        self.start_time = time.time()
        self.get_logger().info("=========================================")
        self.get_logger().info(" F1TENTH Telemetry Analyzer Initialized")
        self.get_logger().info(" Waiting for global waypoints...")
        self.get_logger().info(" Press Ctrl+C at any time to generate report.")
        self.get_logger().info("=========================================")

    def waypoints_callback(self, msg):
        if self.waypoints is None:
            self.waypoints = np.array([[wp.x_m, wp.y_m] for wp in msg.wpnts])
            self.get_logger().info(f"Received {len(self.waypoints)} global waypoints. Ready to analyze!")

    def pose_callback(self, msg):
        if self.waypoints is None:
            return
        
        x = msg.pose.position.x
        y = msg.pose.position.y
        t = time.time()
        
        # Compute lateral error (cross-track error)
        dists = np.linalg.norm(self.waypoints - np.array([x, y]), axis=1)
        min_idx = np.argmin(dists)
        lat_err = dists[min_idx]
        
        self.pose_history.append((t, x, y, lat_err))

    def odom_callback(self, msg):
        t = time.time()
        speed = msg.twist.twist.linear.x
        self.odom_history.append((t, speed))

    def drive_callback(self, msg):
        t = time.time()
        steer = msg.drive.steering_angle
        cmd_speed = msg.drive.speed
        self.drive_history.append((t, steer, cmd_speed))

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

        # 4. Speed Lag Metrics
        speed_lag = 0.0
        if self.odom_history and self.drive_history:
            odom_t = [o[0] for o in self.odom_history]
            odom_v = [o[1] for o in self.odom_history]
            
            cmd_t = [d[0] for d in self.drive_history]
            cmd_v = [d[2] for d in self.drive_history]
            
            try:
                interp_actual_v = np.interp(cmd_t, odom_t, odom_v)
                speed_errors = np.abs(np.array(cmd_v) - interp_actual_v)
                speed_lag = np.mean(speed_errors)
                self.get_logger().info(f"  Average Speed Tracking Lag:  {speed_lag:.3f} m/s")
            except Exception:
                pass

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

        # Rule 5: Speed controller lag
        if speed_lag > 0.5:
            recommendations.append(
                "⚙️ VELOCITY CONTROLLER LAG:\n"
                "   The actual speed is lagging significantly behind commanded speed.\n"
                "   -> Increase 'vel_ctr_p_gain' in 'l1_params.yaml' (e.g., from 1.8 to 2.2) to make acceleration snappier.\n"
                "   -> Verify if VESC hardware current limits or battery level are restricting motor power."
            )

        if not recommendations:
            recommendations.append(
                "✅ EXCELLENT TUNING!\n"
                "   Your lateral error is low, steering is stable, and speed tracking is tight.\n"
                "   To push the car faster, gradually increase sector scaling values in speed_scaling.yaml."
            )
            
        for rec in recommendations:
            self.get_logger().info(rec)
            self.get_logger().info("-" * 66)
        self.get_logger().info("==================================================================")

def main(args=None):
    rclpy.init(args=args)
    node = TunerAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.generate_report()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
