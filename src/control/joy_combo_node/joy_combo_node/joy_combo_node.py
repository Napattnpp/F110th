#!/usr/bin/env python3
"""
Joy Combo Node - Dual-Function Gamepad Modifier and Action Dispatcher.

Features:
- Symmetrical modifiers: LT (Left Trigger) and RT (Right Trigger) act as function keys.
- Base Layer (no triggers): All standalone face buttons and D-pad default to 'none'
  to prevent accidental triggers during trackside handling.
- LT Layer (Vehicle & Autonomy Operations):
    LT + A: reset_localizer (initialpose from raceline start)
    LT + B: emergency_stop (0.0 m/s on /e_stop and pause_navigation = True)
    LT + X: toggle_ftg_mode (/state_machine/force_ftg)
    LT + Y: foxglove_bookmark (/foxglove/event JSON + /foxglove/bookmark_marker)
- RT Layer (2D Remote Tuning & Navigation):
    RT + D-pad Up   : step_value_up
    RT + D-pad Down : step_value_down
    RT + D-pad Left : step_param_prev
    RT + D-pad Right: step_param_next
- Clean edge-triggered debouncing on both buttons and D-pad axes.
- Fully parameter-driven mappings via YAML.
"""

import json
import math
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

try:
    from f110_msgs.msg import WpntArray
except ImportError:
    WpntArray = None


class JoyComboNode(Node):
    def __init__(self):
        super().__init__(
            'joy_combo_node',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        # 1. Hardware axis and button index configuration
        self.lt_axis = self._get_param_int('lt_axis', 2)
        self.rt_axis = self._get_param_int('rt_axis', 5)
        self.trigger_threshold = self._get_param_float('trigger_threshold', -0.5)

        self.dpad_x_axis = self._get_param_int('dpad_x_axis', 6)
        self.dpad_y_axis = self._get_param_int('dpad_y_axis', 7)
        self.dpad_threshold = self._get_param_float('dpad_threshold', 0.5)

        self.btn_map = {
            'btn_a': self._get_param_int('btn_a', 0),
            'btn_b': self._get_param_int('btn_b', 1),
            'btn_x': self._get_param_int('btn_x', 2),
            'btn_y': self._get_param_int('btn_y', 3),
            'btn_lb': self._get_param_int('btn_lb', 4),
            'btn_rb': self._get_param_int('btn_rb', 5),
        }

        # 2. Action layer configuration
        self.lt_actions = self._load_action_layer('lt_actions', {
            'btn_a': 'reset_localizer',
            'btn_b': 'emergency_stop',
            'btn_x': 'toggle_ftg_mode',
            'btn_y': 'foxglove_bookmark',
            'dpad_up': 'none',
            'dpad_down': 'none',
            'dpad_left': 'none',
            'dpad_right': 'none',
        })

        self.rt_actions = self._load_action_layer('rt_actions', {
            'btn_a': 'none',
            'btn_b': 'none',
            'btn_x': 'none',
            'btn_y': 'none',
            'dpad_up': 'step_value_up',
            'dpad_down': 'step_value_down',
            'dpad_left': 'step_param_prev',
            'dpad_right': 'step_param_next',
        })

        self.base_actions = self._load_action_layer('base_actions', {
            'btn_a': 'none',
            'btn_b': 'none',
            'btn_x': 'none',
            'btn_y': 'none',
            'btn_lb': 'none',
            'btn_rb': 'toggle_autonomy',
            'dpad_up': 'none',
            'dpad_down': 'none',
            'dpad_left': 'none',
            'dpad_right': 'none',
        })

        # 3. State management & debouncing
        self.prev_buttons: List[int] = []
        self.prev_dpad_x: int = 0
        self.prev_dpad_y: int = 0

        self.pause_nav_state: bool = True
        self.first_waypoint = None
        self.latest_pose: Optional[Tuple[float, float, float]] = None  # (x, y, yaw)
        self.bookmark_id: int = 0
        self.ftg_mode_active: bool = False
        self.e_stop_burst_count: int = 0

        # 4. Publishers
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.e_stop_pub = self.create_publisher(AckermannDriveStamped, '/e_stop', 10)
        self.pause_nav_pub = self.create_publisher(Bool, '/pause_navigation', 10)
        self.force_ftg_pub = self.create_publisher(Bool, '/state_machine/force_ftg', 10)
        self.foxglove_event_pub = self.create_publisher(String, '/foxglove/event', 10)
        self.bookmark_marker_pub = self.create_publisher(Marker, '/foxglove/bookmark_marker', 10)
        self.dpad_step_pub = self.create_publisher(String, '/joy_combo/dpad_step', 10)

        # 5. Subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        if WpntArray is not None:
            self.waypoints_sub = self.create_subscription(
                WpntArray, '/global_waypoints', self.waypoints_callback, 10
            )

        self.odom_sub = self.create_subscription(Odometry, '/car_state/odom', self.odom_callback, 10)
        self.ego_odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/car_state/pose', self.pose_callback, 10)

        # 6. E-stop active burst timer (20Hz) & pause navigation state heartbeat
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Immediately lock autonomy on boot (Safe Startup Guard)
        init_pause = Bool()
        init_pause.data = self.pause_nav_state
        self.pause_nav_pub.publish(init_pause)

        self.get_logger().info(
            f"[{self.get_name()}] Started with LT modifier (axis {self.lt_axis}) "
            f"and RT modifier (axis {self.rt_axis}). Base layer buttons unassigned (safe). "
            f"Autonomy initialized in PAUSED mode (/pause_navigation = True). Press RB to toggle."
        )

    def _get_param_int(self, name: str, default: int) -> int:
        if self.has_parameter(name):
            return int(self.get_parameter(name).value)
        return default

    def _get_param_float(self, name: str, default: float) -> float:
        if self.has_parameter(name):
            return float(self.get_parameter(name).value)
        return default

    def _load_action_layer(self, prefix: str, defaults: Dict[str, str]) -> Dict[str, str]:
        layer = dict(defaults)
        for key in defaults.keys():
            param_name = f"{prefix}.{key}"
            if self.has_parameter(param_name):
                layer[key] = str(self.get_parameter(param_name).value).strip().lower()
        return layer

    # ------------------ Topic Callbacks ------------------

    def waypoints_callback(self, msg):
        if hasattr(msg, 'wpnts') and len(msg.wpnts) > 0:
            self.first_waypoint = msg.wpnts[0]

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        self.latest_pose = (x, y, yaw)

    def pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        self.latest_pose = (x, y, yaw)

    def timer_callback(self):
        """Maintain burst of zero-speed commands when E-STOP is active, and publish pause lock heartbeat."""
        if self.e_stop_burst_count > 0:
            self.e_stop_burst_count -= 1
            e_stop_msg = AckermannDriveStamped()
            e_stop_msg.header.stamp = self.get_clock().now().to_msg()
            e_stop_msg.header.frame_id = 'base_link'
            e_stop_msg.drive.speed = 0.0
            e_stop_msg.drive.steering_angle = 0.0
            self.e_stop_pub.publish(e_stop_msg)

        # Continuous heartbeat to ensure ackermann_mux lock stays synchronized across node launches
        pause_msg = Bool()
        pause_msg.data = self.pause_nav_state
        self.pause_nav_pub.publish(pause_msg)

    # ------------------ Main Gamepad Processing ------------------

    def joy_callback(self, msg: Joy):
        # 1. Determine active modifier key (LT vs RT vs None)
        lt_held = False
        rt_held = False

        if len(msg.axes) > self.lt_axis:
            lt_held = (msg.axes[self.lt_axis] <= self.trigger_threshold)
        if len(msg.axes) > self.rt_axis:
            rt_held = (msg.axes[self.rt_axis] <= self.trigger_threshold)

        # Select layer
        if lt_held and not rt_held:
            layer = self.lt_actions
            layer_name = "LT"
        elif rt_held and not lt_held:
            layer = self.rt_actions
            layer_name = "RT"
        elif not lt_held and not rt_held:
            layer = self.base_actions
            layer_name = "BASE"
        else:
            # Both modifiers pressed simultaneously: ignore combos for safety
            return

        # 2. Check face & bumper button rising edges
        for btn_key, btn_idx in self.btn_map.items():
            if btn_idx < len(msg.buttons):
                curr_val = msg.buttons[btn_idx]
                prev_val = self.prev_buttons[btn_idx] if btn_idx < len(self.prev_buttons) else 0

                # Rising edge: 0 -> 1
                if curr_val == 1 and prev_val == 0:
                    action = layer.get(btn_key, "none")
                    if action and action != "none":
                        self.get_logger().debug(f"[{layer_name}] {btn_key} pressed -> action '{action}'")
                        self.dispatch_action(action)

        # 3. Check D-pad rising edges
        curr_dpad_x = 0
        if len(msg.axes) > self.dpad_x_axis:
            val_x = msg.axes[self.dpad_x_axis]
            if val_x > self.dpad_threshold:
                curr_dpad_x = 1  # Left
            elif val_x < -self.dpad_threshold:
                curr_dpad_x = -1  # Right

        curr_dpad_y = 0
        if len(msg.axes) > self.dpad_y_axis:
            val_y = msg.axes[self.dpad_y_axis]
            if val_y > self.dpad_threshold:
                curr_dpad_y = 1  # Up
            elif val_y < -self.dpad_threshold:
                curr_dpad_y = -1  # Down

        # D-pad Up rising edge
        if curr_dpad_y == 1 and self.prev_dpad_y != 1:
            action = layer.get('dpad_up', 'none')
            if action and action != 'none':
                self.dispatch_action(action)

        # D-pad Down rising edge
        if curr_dpad_y == -1 and self.prev_dpad_y != -1:
            action = layer.get('dpad_down', 'none')
            if action and action != 'none':
                self.dispatch_action(action)

        # D-pad Left rising edge
        if curr_dpad_x == 1 and self.prev_dpad_x != 1:
            action = layer.get('dpad_left', 'none')
            if action and action != 'none':
                self.dispatch_action(action)

        # D-pad Right rising edge
        if curr_dpad_x == -1 and self.prev_dpad_x != -1:
            action = layer.get('dpad_right', 'none')
            if action and action != 'none':
                self.dispatch_action(action)

        # Update previous states
        self.prev_buttons = list(msg.buttons)
        self.prev_dpad_x = curr_dpad_x
        self.prev_dpad_y = curr_dpad_y

    # ------------------ Action Dispatcher ------------------

    def dispatch_action(self, action: str):
        if action == 'reset_localizer':
            self._action_reset_localizer()
        elif action == 'emergency_stop':
            self._action_emergency_stop()
        elif action in ('toggle_autonomy', 'toggle_pause_nav'):
            self._action_toggle_autonomy()
        elif action == 'toggle_ftg_mode':
            self._action_toggle_ftg_mode()
        elif action == 'foxglove_bookmark':
            self._action_foxglove_bookmark()
        elif action in ('step_value_up', 'step_value_down', 'step_param_prev', 'step_param_next'):
            self._action_dpad_step(action)
        else:
            self.get_logger().warn(f"[{self.get_name()}] Unknown action requested: '{action}'")

    def _action_reset_localizer(self):
        """Query raceline start waypoint and publish to /initialpose."""
        x, y, yaw = 0.0, 0.0, 0.0
        source = "origin fallback"

        if self.first_waypoint is not None:
            x = float(getattr(self.first_waypoint, 'x_m', 0.0))
            y = float(getattr(self.first_waypoint, 'y_m', 0.0))
            yaw = float(getattr(self.first_waypoint, 'psi_rad', 0.0))
            source = "raceline waypoint 0"
        elif self.latest_pose is not None:
            x, y, yaw = self.latest_pose
            source = "current odometry"

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Default standard covariance matrix
        cov = [0.0] * 36
        cov[0] = 0.25   # x variance
        cov[7] = 0.25   # y variance
        cov[35] = 0.0685  # yaw variance (~15 deg)
        pose_msg.pose.covariance = cov

        self.initialpose_pub.publish(pose_msg)
        self.get_logger().info(
            f"[{self.get_name()}] [RESET_LOCALIZER] Published /initialpose from {source}: "
            f"x={x:.2f}m, y={y:.2f}m, yaw={yaw:.2f}rad"
        )

    def _action_emergency_stop(self):
        """Fail-fast emergency stop: publish 0 m/s on /e_stop and lock autonomous drive."""
        self.e_stop_burst_count = 10  # 10 ticks @ 20Hz = 0.5s burst

        e_stop_msg = AckermannDriveStamped()
        e_stop_msg.header.stamp = self.get_clock().now().to_msg()
        e_stop_msg.header.frame_id = 'base_link'
        e_stop_msg.drive.speed = 0.0
        e_stop_msg.drive.steering_angle = 0.0
        self.e_stop_pub.publish(e_stop_msg)

        # Lock navigation topic via mux pause lock
        self.pause_nav_state = True
        pause_msg = Bool()
        pause_msg.data = True
        self.pause_nav_pub.publish(pause_msg)

        self.get_logger().warn(
            f"[{self.get_name()}] 🚨 EMERGENCY STOP ACTIVATED 🚨 "
            f"Sent 0.0 m/s to /e_stop (prio 200) and locked /pause_navigation = True."
        )

    def _action_toggle_autonomy(self):
        """Toggle autonomous navigation pause state on /pause_navigation."""
        self.pause_nav_state = not self.pause_nav_state
        pause_msg = Bool()
        pause_msg.data = self.pause_nav_state
        self.pause_nav_pub.publish(pause_msg)

        status = "PAUSED" if self.pause_nav_state else "RUNNING"
        self.get_logger().info(
            f"[{self.get_name()}] [AUTONOMY_TOGGLE] Navigation state: {status} "
            f"(/pause_navigation = {self.pause_nav_state})"
        )

    def _action_toggle_ftg_mode(self):
        """Toggle force Follow-The-Gap mode on /state_machine/force_ftg."""
        self.ftg_mode_active = not self.ftg_mode_active
        msg = Bool()
        msg.data = self.ftg_mode_active
        self.force_ftg_pub.publish(msg)

        mode_str = "ENABLED (Forcing FTG mode)" if self.ftg_mode_active else "DISABLED (Normal Racing)"
        self.get_logger().info(f"[{self.get_name()}] [FTG_TOGGLE] Force FTG mode: {mode_str}")

    def _action_foxglove_bookmark(self):
        """Drop a spatial marker in Foxglove 3D view and emit event JSON."""
        cur_x, cur_y = 0.0, 0.0
        if self.latest_pose is not None:
            cur_x, cur_y, _ = self.latest_pose

        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # 1. Foxglove JSON event
        event_dict = {
            "type": "manual_bookmark",
            "bookmark_id": self.bookmark_id,
            "timestamp": now_sec,
            "x": round(cur_x, 3),
            "y": round(cur_y, 3),
            "label": f"Trackside Bookmark #{self.bookmark_id}"
        }
        event_msg = String()
        event_msg.data = json.dumps(event_dict)
        self.foxglove_event_pub.publish(event_msg)

        # 2. Foxglove 3D Cylinder Pin Marker
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'foxglove_bookmarks'
        marker.id = self.bookmark_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = cur_x
        marker.pose.position.y = cur_y
        marker.pose.position.z = 0.2
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.4
        # Vivid Amber / Gold pin color
        marker.color.r = 1.0
        marker.color.g = 0.75
        marker.color.b = 0.0
        marker.color.a = 0.95

        self.bookmark_marker_pub.publish(marker)
        self.get_logger().info(
            f"[{self.get_name()}] [BOOKMARK] Bookmark #{self.bookmark_id} "
            f"pinned at (x={cur_x:.2f}m, y={cur_y:.2f}m) published to Foxglove."
        )
        self.bookmark_id += 1

    def _action_dpad_step(self, action: str):
        """Dispatch remote tuning 2D stepping event."""
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        direction = 1 if 'up' in action or 'next' in action else -1
        axis = 'value' if 'value' in action else 'param'

        payload = {
            "action": action,
            "axis": axis,
            "direction": direction,
            "timestamp": now_sec
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.dpad_step_pub.publish(msg)

        self.get_logger().info(f"[{self.get_name()}] [2D_MENU] {action} (axis={axis}, dir={direction:+d})")


def main(args=None):
    rclpy.init(args=args)
    node = JoyComboNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

