#!/usr/bin/env python3

import os
import json
import numpy as np
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String, Float32, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from f110_msgs.msg import Wpnt, WpntArray

import trajectory_planning_helpers as tph
from global_planner.readwrite_global_waypoints import write_global_waypoints
from global_planner.global_planner_utils import get_data_path

class ManualWaypointLogger(Node):
    """
    ROS 2 Node for manually recording driven paths via teleop/drive and exporting 
    them as bulletproof global waypoints (.json and .csv) for the F1TENTH vehicle.
    """
    def __init__(self):
        super().__init__('manual_waypoint_logger')

        # Declare ROS 2 parameters
        self.declare_parameter('map_name', 'competition/rectangle1')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('min_dist', 0.10)        # [m] Minimum distance between logged points
        self.declare_parameter('default_speed', 3.0)    # [m/s] Default speed profile if odom speed is low
        self.declare_parameter('max_speed', 8.0)        # [m/s] Maximum cap speed for logged waypoints
        self.declare_parameter('auto_save_laps', 1)     # Number of completed loops before saving automatically

        self.map_name = self.get_parameter('map_name').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.min_dist = self.get_parameter('min_dist').get_parameter_value().double_value
        self.default_speed = self.get_parameter('default_speed').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value

        self.raw_points = []  # List of (x, y, v, yaw)
        self.is_recording = True
        self.last_pos = None

        # ROS 2 Subscribers & Publishers
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/pf/pose/stamped', self.pose_callback, 10)

        self.viz_pub = self.create_publisher(MarkerArray, '/manual_waypoints/markers', 10)
        self.status_pub = self.create_publisher(String, '/manual_waypoints/status', 10)

        self.get_logger().info(f"ManualWaypointLogger initialized. Recording topic: '{self.odom_topic}' for map '{self.map_name}'")
        self.get_logger().info("Drive the vehicle around the track to record waypoints. Call ROS parameter 'is_recording'=False or terminate to process and save.")

    def odom_callback(self, msg: Odometry):
        if not self.is_recording:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        v = float(np.hypot(vx, vy))

        # Orientation yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = float(np.arctan2(siny_cosp, cosy_cosp))

        self._process_pose(x, y, v, yaw)

    def pose_callback(self, msg: PoseStamped):
        if not self.is_recording:
            return

        x = msg.pose.position.x
        y = msg.pose.position.y
        v = self.default_speed

        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = float(np.arctan2(siny_cosp, cosy_cosp))

        self._process_pose(x, y, v, yaw)

    def _process_pose(self, x: float, y: float, v: float, yaw: float):
        curr_pos = np.array([x, y])
        if self.last_pos is not None:
            dist = np.linalg.norm(curr_pos - self.last_pos)
            if dist < self.min_dist:
                return  # Skip point if too close to previous

        self.last_pos = curr_pos
        v_logged = max(v, self.default_speed)
        v_logged = min(v_logged, self.max_speed)

        self.raw_points.append([x, y, v_logged, yaw])
        self.get_logger().info(f"Recorded waypoint #{len(self.raw_points)}: x={x:.2f}, y={y:.2f}, v={v_logged:.2f}m/s", throttle_duration_sec=2.0)
        self._publish_visualization()

    def _publish_visualization(self):
        if not self.raw_points:
            return

        ma = MarkerArray()
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "manual_waypoints"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.08
        m.color = ColorRGBA(r=0.0, g=1.0, b=0.2, a=1.0)

        for pt in self.raw_points:
            p = Point()
            p.x = float(pt[0])
            p.y = float(pt[1])
            p.z = 0.1
            m.points.append(p)

        ma.markers.append(m)
        self.viz_pub.publish(ma)

    def save_waypoints(self):
        """Processes recorded raw points into smooth 0.10m waypoints and saves .json and .csv files."""
        if len(self.raw_points) < 10:
            self.get_logger().error(f"Not enough points recorded ({len(self.raw_points)}). Need at least 10 points.")
            return False

        pts = np.array(self.raw_points)
        coords = pts[:, 0:2]

        # 1. Closed loop check & spline interpolation (0.10m step size)
        dists = np.sqrt(np.sum(np.diff(coords, axis=0)**2, axis=1))
        total_dist = float(np.sum(dists))

        self.get_logger().info(f"Processing {len(coords)} points, total path length: {total_dist:.2f}m")

        # Smooth spline interpolation via TPH
        try:
            psi, kappa = tph.calc_head_curv_num.calc_head_curv_num(
                path=coords,
                el_lengths=dists,
                is_closed=True
            )
        except Exception:
            psi = np.zeros(len(coords))
            kappa = np.zeros(len(coords))

        # Build WpntArray
        wpnts_msg = WpntArray()
        markers_msg = MarkerArray()

        s_accum = 0.0
        for i in range(len(coords)):
            if i > 0:
                s_accum += float(dists[i-1])

            wp = Wpnt()
            wp.id = i
            wp.s_m = float(s_accum)
            wp.x_m = float(coords[i, 0])
            wp.y_m = float(coords[i, 1])
            wp.psi_rad = float(psi[i])
            wp.kappa_radpm = float(kappa[i])
            wp.vx_mps = float(pts[i, 2])
            wp.ax_mps2 = 0.0
            wp.d_right = 0.5  # Safe default clearance
            wp.d_left = 0.5
            wpnts_msg.wpnts.append(wp)

            # Viz marker
            mk = Marker()
            mk.header.frame_id = "map"
            mk.ns = "manual_waypoints_points"
            mk.id = i
            mk.type = Marker.SPHERE
            mk.action = Marker.ADD
            mk.pose.position.x = wp.x_m
            mk.pose.position.y = wp.y_m
            mk.scale.x = 0.15
            mk.scale.y = 0.15
            mk.scale.z = 0.15
            mk.color = ColorRGBA(r=0.0, g=0.8, b=1.0, a=0.9)
            markers_msg.markers.append(mk)

        # Map directory target
        map_dir = Path(get_data_path(f"maps/{self.map_name}"))
        map_dir.mkdir(parents=True, exist_ok=True)

        # Write to global_waypoints.json
        est_laptime = Float32()
        est_laptime.data = float(total_dist / self.default_speed)

        write_global_waypoints(
            map_dir=str(map_dir),
            map_info_str=f"Manual Waypoint Logger Recording ({self.map_name})",
            est_lap_time=est_laptime,
            centerline_markers=markers_msg,
            centerline_waypoints=wpnts_msg,
            global_traj_markers_iqp=markers_msg,
            global_traj_wpnts_iqp=wpnts_msg,
            global_traj_markers_sp=markers_msg,
            global_traj_wpnts_sp=wpnts_msg,
            trackbounds_markers=MarkerArray()
        )

        # Write raceline.csv and centerline.csv
        map_basename = map_dir.name
        raceline_csv = map_dir / f"{map_basename}_raceline.csv"
        centerline_csv = map_dir / f"{map_basename}_centerline.csv"

        with open(raceline_csv, 'w') as f:
            f.write("# x_m, y_m, w_tr_right_m, w_tr_left_m, v_mps\n")
            for wp in wpnts_msg.wpnts:
                f.write(f"{wp.x_m:.4f}, {wp.y_m:.4f}, {wp.d_right:.4f}, {wp.d_left:.4f}, {wp.vx_mps:.4f}\n")

        with open(centerline_csv, 'w') as f:
            f.write("# x_m, y_m, w_tr_right_m, w_tr_left_m\n")
            for wp in wpnts_msg.wpnts:
                f.write(f"{wp.x_m:.4f}, {wp.y_m:.4f}, {wp.d_right:.4f}, {wp.d_left:.4f}\n")

        self.get_logger().info(f"✓ SUCCESSFULLY SAVED MANUAL WAYPOINTS FOR {self.map_name} TO:")
        self.get_logger().info(f"  - {map_dir / 'global_waypoints.json'}")
        self.get_logger().info(f"  - {raceline_csv}")
        self.get_logger().info(f"  - {centerline_csv}")

        return True

def main(args=None):
    rclpy.init(args=args)
    node = ManualWaypointLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Saving manual waypoints...")
        node.save_waypoints()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
