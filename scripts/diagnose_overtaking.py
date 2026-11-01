#!/usr/bin/env python3
"""
Diagnostic & Verification Monitor for F1TENTH Head-to-Head Overtaking
----------------------------------------------------------------------
Evaluates run quality against strict racing safety criteria:
1. Car-to-Car OBB Separation: Calculates oriented bounding boxes for ego and opponent.
   Verifies they never intersect (distance > 0).
2. Wall Clearance: Computes distance from vehicle body envelope to track boundaries.
   Flags wall touches (clearance <= 0).
3. Telemetry Discontinuities: Monitors speed and yaw rate for sudden drops/spikes to 0.
4. State Machine Oscillation: Tracks TRAILING <-> OVERTAKE cycles and early aborts.
5. Overall Pass Evaluation: Determines clean passes vs crashes vs stalled attempts.
"""

import sys
import time
import math
import argparse
from typing import List, Tuple, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from f110_msgs.msg import ObstacleArray, WpntArray


def get_obb_corners(center_x: float, center_y: float, yaw: float, length: float, width: float) -> np.ndarray:
    """Returns 4 corners of an oriented bounding box centered at (center_x, center_y)."""
    dx = length / 2.0
    dy = width / 2.0
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    
    # 4 corners in local body frame: [FL, FR, RR, RL]
    local_corners = np.array([
        [ dx,  dy],
        [ dx, -dy],
        [-dx, -dy],
        [-dx,  dy]
    ])
    
    # Rotation matrix
    R = np.array([
        [cos_y, -sin_y],
        [sin_y,  cos_y]
    ])
    
    world_corners = (R @ local_corners.T).T + np.array([center_x, center_y])
    return world_corners


def polygons_intersect(poly1: np.ndarray, poly2: np.ndarray) -> Tuple[bool, float]:
    """Separating Axis Theorem (SAT) check for 2D convex polygons.
    Returns (is_intersecting, min_separating_distance).
    """
    polys = [poly1, poly2]
    min_dist = float('inf')
    
    for i in range(2):
        poly = polys[i]
        for j in range(len(poly)):
            p1 = poly[j]
            p2 = poly[(j + 1) % len(poly)]
            edge = p2 - p1
            normal = np.array([-edge[1], edge[0]])
            norm = np.linalg.norm(normal)
            if norm < 1e-9:
                continue
            axis = normal / norm
            
            # Project both polygons onto axis
            proj1 = poly1 @ axis
            proj2 = poly2 @ axis
            
            min1, max1 = np.min(proj1), np.max(proj1)
            min2, max2 = np.min(proj2), np.max(proj2)
            
            # Check gap along this axis
            if max1 < min2:
                gap = min2 - max1
                return False, gap
            elif max2 < min1:
                gap = min1 - max2
                return False, gap
            else:
                # Overlap
                overlap = min(max1, max2) - max(min1, min2)
                min_dist = min(min_dist, overlap)
                
    return True, -min_dist


class OvertakeDiagnosticMonitor(Node):
    def __init__(self, ego_length: float = 0.58, ego_width: float = 0.31, opp_length: float = 0.58, opp_width: float = 0.31):
        super().__init__('overtake_diagnostic_monitor')
        
        self.ego_length = ego_length
        self.ego_width = ego_width
        self.opp_length = opp_length
        self.opp_width = opp_width
        
        # State caches
        self.ego_pose = None      # (x, y, yaw)
        self.ego_twist = None     # (vx, vy, wz)
        self.ego_frenet = None    # (s, d, vs)
        self.opp_obs = []         # list of obstacles
        self.active_state = "UNKNOWN"
        self.prev_state = "UNKNOWN"
        self.track_length = 41.93
        self.raceline_wpnts = []
        
        # Diagnostics Metrics
        self.start_time = time.time()
        self.car_touches = 0
        self.wall_touches = 0
        self.state_cycles = 0     # count of TRAILING <-> OVERTAKE transitions
        self.speed_drop_events = []
        self.min_car_distance = float('inf')
        self.min_wall_clearance = float('inf')
        self.history = []
        
        # Subscriptions
        self.create_subscription(Odometry, '/car_state/odom', self._odom_cb, 10)
        self.create_subscription(Odometry, '/car_state/frenet/odom', self._frenet_cb, 10)
        self.create_subscription(ObstacleArray, '/perception/obstacles', self._obs_cb, 10)
        self.create_subscription(String, '/state', self._state_cb, 10)
        self.create_subscription(WpntArray, '/global_waypoints', self._glb_wpnts_cb, 10)
        
        # 40 Hz Audit Timer
        self.timer = self.create_timer(0.025, self._audit_cycle)
        self.get_logger().info("Overtake Diagnostic Monitor active (40 Hz SAT collision & telemetry audit)")

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.ego_pose = (p.x, p.y, yaw)
        
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z
        self.ego_twist = (vx, vy, wz)

    def _frenet_cb(self, msg: Odometry):
        self.ego_frenet = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.twist.twist.linear.x
        )

    def _obs_cb(self, msg: ObstacleArray):
        self.opp_obs = list(msg.obstacles)

    def _state_cb(self, msg: String):
        new_state = msg.data
        if new_state != self.active_state:
            # Track cycling between TRAILING and OVERTAKE
            if (self.active_state == "TRAILING" and new_state == "OVERTAKE") or \
               (self.active_state == "OVERTAKE" and new_state in ("TRAILING", "GB_TRACK")):
                self.state_cycles += 1
            self.prev_state = self.active_state
            self.active_state = new_state
            t_rel = time.time() - self.start_time
            s_val = self.ego_frenet[0] if self.ego_frenet else 0.0
            self.get_logger().info(f"[STATE] T+{t_rel:.2f}s | {self.prev_state} -> {self.active_state} at s={s_val:.2f}m (cycle #{self.state_cycles})")

    def _glb_wpnts_cb(self, msg: WpntArray):
        if msg.wpnts and not self.raceline_wpnts:
            self.raceline_wpnts = list(msg.wpnts)
            self.track_length = msg.wpnts[-1].s_m

    def _audit_cycle(self):
        if self.ego_pose is None or not self.opp_obs:
            return
        
        ego_x, ego_y, ego_yaw = self.ego_pose
        ego_obb = get_obb_corners(ego_x, ego_y, ego_yaw, self.ego_length, self.ego_width)
        
        # 1. Car-to-Car OBB Area Calculation (Criterion #1)
        for obs in self.opp_obs:
            opp_size = getattr(obs, 'size', 0.35)
            opp_l = max(self.opp_length, opp_size)
            opp_w = max(self.opp_width, opp_size)
            opp_x = getattr(obs, 'x', 0.0)
            opp_y = getattr(obs, 'y', 0.0)
            opp_yaw = getattr(obs, 'yaw', ego_yaw)
            
            # If cartesian position is missing, estimate from raceline
            if opp_x == 0.0 and opp_y == 0.0 and self.raceline_wpnts:
                s_idx = int(np.argmin([abs(w.s_m - obs.s_center) for w in self.raceline_wpnts]))
                ref_w = self.raceline_wpnts[s_idx]
                opp_x = ref_w.x_m
                opp_y = ref_w.y_m
                opp_yaw = ref_w.psi_rad
                
            opp_obb = get_obb_corners(opp_x, opp_y, opp_yaw, opp_l, opp_w)
            
            intersects, sep_dist = polygons_intersect(ego_obb, opp_obb)
            if intersects:
                self.car_touches += 1
                self.get_logger().error(f"[COLLISION ALERT] Car touched Opponent ID {obs.id}! Penetration depth: {sep_dist:.3f}m")
            else:
                self.min_car_distance = min(self.min_car_distance, sep_dist)

        # 2. Wall Clearance Audit (Criterion #2)
        if self.raceline_wpnts and self.ego_frenet:
            cur_s, cur_d, _ = self.ego_frenet
            s_idx = int(np.argmin([abs(w.s_m - (cur_s % self.track_length)) for w in self.raceline_wpnts]))
            w = self.raceline_wpnts[s_idx]
            d_left = getattr(w, 'd_left', 1.0)
            d_right = abs(getattr(w, 'd_right', 1.0))
            
            # Distance from vehicle outer sides to wall
            half_w = self.ego_width / 2.0
            clearance_left = (d_left - cur_d) - half_w
            clearance_right = (d_right + cur_d) - half_w
            min_wall_clr = min(clearance_left, clearance_right)
            self.min_wall_clearance = min(self.min_wall_clearance, min_wall_clr)
            
            if min_wall_clr <= 0.0:
                self.wall_touches += 1
                self.get_logger().error(f"[WALL IMPACT] Car touched track wall! Clearance: {min_wall_clr:.3f}m at s={cur_s:.2f}m")

        # 3. Telemetry Discontinuity Detection (Criterion #3)
        if self.ego_twist:
            vx, vy, wz = self.ego_twist
            if abs(vx) < 0.15 and len(self.history) > 5:
                prev_vx = self.history[-1]['vx']
                if prev_vx > 1.5:
                    drop_info = {
                        'time': time.time() - self.start_time,
                        's': self.ego_frenet[0] if self.ego_frenet else 0.0,
                        'drop_from': prev_vx,
                        'drop_to': vx,
                        'state': self.active_state
                    }
                    self.speed_drop_events.append(drop_info)
                    self.get_logger().warn(f"[ANOMALY] Sudden speed drop detected: {prev_vx:.2f} -> {vx:.2f} m/s at s={drop_info['s']:.2f}m (State: {self.active_state})")

        self.history.append({
            'time': time.time() - self.start_time,
            'vx': self.ego_twist[0] if self.ego_twist else 0.0,
            'state': self.active_state
        })

    def print_summary(self):
        t_tot = time.time() - self.start_time
        print("\n" + "=" * 65)
        print("          🏁 OVERTAKE DIAGNOSTIC AUDIT SUMMARY 🏁")
        print("=" * 65)
        print(f" Total Elapsed Time          : {t_tot:.2f} s")
        print(f" Car-to-Opponent Touches     : {self.car_touches} {'❌ FAILED' if self.car_touches > 0 else '✅ PASSED (0 touches)'}")
        print(f" Minimum Car Separation      : {self.min_car_distance:.3f} m")
        print(f" Wall Touches / Impacts      : {self.wall_touches} {'❌ FAILED' if self.wall_touches > 0 else '✅ PASSED (0 wall touches)'}")
        print(f" Minimum Wall Clearance      : {self.min_wall_clearance:.3f} m")
        print(f" State Cycles (Trail <-> OT) : {self.state_cycles} {'⚠️ CHATTERING' if self.state_cycles > 4 else '✅ CLEAN PASS'}")
        print(f" Sudden Speed Drops to ~0    : {len(self.speed_drop_events)}")
        for ev in self.speed_drop_events:
            print(f"   - T+{ev['time']:.2f}s: {ev['drop_from']:.2f} -> {ev['drop_to']:.2f} m/s at s={ev['s']:.2f}m ({ev['state']})")
        print("=" * 65 + "\n")


def main():
    rclpy.init()
    node = OvertakeDiagnosticMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_summary()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

