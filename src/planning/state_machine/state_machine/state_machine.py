import os
import yaml
import rclpy
import numpy as np
from builtin_interfaces.msg import Time
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import Parameter
from ament_index_python import get_package_share_directory
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler
from typing import List
from f110_msgs.msg import Wpnt, WpntArray, OTWpntArray, ObstacleArray, Obstacle
import random
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan, Imu
try:
    from vesc_msgs.msg import VescStateStamped
except ImportError:
    VescStateStamped = None
import json
from state_machine.transitions import dummy_transition, timetrials_transition, head_to_head_transition, qualifying_transition, get_qualifying_phase, hierarchical_transition
from state_machine.state_types import StateType, TopLevelMode
from state_machine.states import DefaultStateLogic
from race_bringup.parameter_event_handler import ParameterEventHandler
from state_machine.state_machine_params import StateMachineParams


def time_to_float(time_instant: Time):
    return time_instant.sec + time_instant.nanosec * 1e-9

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        # PARAMETER DECLARATION
        self.params = StateMachineParams(self)
        

        # Get sim and map_path parameters from global_parameters node to automatically switch profiles
        self.global_param_client = self.create_client(GetParameters, '/global_parameters/get_parameters')
        self.global_param_client.wait_for_service()
        req = GetParameters.Request()
        req.names = ['sim', 'map_path']
        future = self.global_param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.sim = False
        self.active_map_name = ""
        if future.result() is not None:
            self.sim = future.result().values[0].bool_value
            if len(future.result().values) > 1:
                p_val = future.result().values[1].string_value
                if p_val:
                    self.active_map_name = os.path.basename(p_val.rstrip('/'))
        self.get_logger().info(f"Is simulation: {self.sim}, Active map: {self.active_map_name}")

        # Automate test_on_car alignment
        self.params.test_on_car = not self.sim

        # update on parameter changes for rate
        self.add_on_set_parameters_callback(self.params.parameters_callback)
        
        # Manual FTG force override (from joy_combo_node or operator)
        self.force_ftg = False
        self.create_subscription(Bool, '/state_machine/force_ftg', self._force_ftg_cb, 10)

        # SUBSCRIPTIONS
        volt_thresh = self.params.volt_threshold if self.params.volt_threshold is not None else 9.5
        self.battery_level = volt_thresh + 1.0
        if self.params.test_on_car:
            self.battery_sub = self.create_subscription(VescStateStamped, "/sensors/core", self.battery_cb, 10)
            self.battery_sub_legacy = self.create_subscription(VescStateStamped, "/vesc/sensors/core", self.battery_cb, 10)
        self.max_speed = None
        self.create_subscription(WpntArray, "/global_waypoints", self.glb_wpnts_cb, 10)
        while self.max_speed is None: # equivalent of wait for message
            self.get_logger().info("Waiting for global waypoints message", throttle_duration_sec=0.5)
            rclpy.spin_once(self)

        
        self.glb_wpnts = None
        self.num_glb_wpnts = 0
        self.track_length = 1
        self.gb_max_idx = 10
        self.create_subscription(WpntArray, "/global_waypoints_scaled", self.glb_wpnts_scaled_cb, 10)
        while self.glb_wpnts is None: # equivalent of wait for message
            self.get_logger().info("Waiting for scaled global waypoints message", throttle_duration_sec=0.5)
            rclpy.spin_once(self)

        
        self.cur_s = None
        self.cur_d = None
        self.cur_vs = 0.0
        self._prev_s = None
        self.create_subscription(Odometry, '/car_state/frenet/odom',self.car_state_frenet_cb, 10) # car frenet coordinates
        while self.cur_s is None:
            self.get_logger().info("Waiting for car state frenet message", throttle_duration_sec=0.5)
            rclpy.spin_once(self)

        if self.params.mode in ['head_to_head', 'qualifying']:
            self.current_position = None
            self.create_subscription(PoseStamped, '/car_state/pose', self.car_state_cb, 10)
            while self.current_position is None:
                self.get_logger().info("Waiting for car state pose message", throttle_duration_sec=0.5)
                rclpy.spin_once(self)

            if self.params.mode in ["head_to_head", "qualifying"] and self.params.overtake_mode == "spliner":
                self.last_valid_avoidance_wpnts = None
                self.splini_ttl_counter = 0
                self.avoidance_wpnts = None

                self.create_subscription(OTWpntArray, '/planner/avoidance/otwpnts', self.avoidance_cb, 10)

            self.obstacles = []
            self.create_subscription(ObstacleArray, "/perception/obstacles", self.obstacles_cb, 10)
        
            # TODO setup things for sectors and overtaking sectors
            self.only_ftg_zones = []
            self.ftg_counter = 0
            self.gb_occupied_counter = 0

            self.overtake_zones = []

        # HSM & RECOVERY INITIALIZATIONS
        self.top_level_mode = TopLevelMode.NORMAL_RACING
        self.recovery_start_time = 0.0
        self.recovery_substate_start = 0.0
        self.recovery_steer_dir = 0.0  # +1.0 for left, -1.0 for right steer during reverse
        self.stuck_counter = 0
        self.latest_scan = None
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # Multi-modal crash detection telemetry (IMU shock + VESC current)
        self.latest_imu_ax = 0.0
        self.latest_imu_ay = 0.0
        self.latest_imu_wz = 0.0
        self.vesc_current = 0.0
        self.imu_sub = self.create_subscription(Imu, '/imu/data_raw', self.imu_cb, 10)
        self.imu_sub_sim = self.create_subscription(Imu, '/imu', self.imu_cb, 10)

        # Localization health & auto-relocalization
        self.loc_cov_trace = 0.0
        self.last_reloc_time = 0.0
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)

        # INITIALIZATIONS        
        self.waypoints_dist = 0.1
        self.glb_s_array = np.array([])
        
        self.state = StateType(self.params.initial_state)
        self.state_start_time = self.get_clock().now()
        self.state_dwell_time = 0.0
        self.last_ot_exit_time = None
        self.local_waypoints = WpntArray()
        self.first_visualization = True
        self.x_viz = 0
        self.y_viz = 0

        # Lap counter (used by qualifying mode)
        self.lap_count = 0
        self._prev_s = None           # previous cur_s for wrap-around detection
        self._lap_wrap_threshold = 0  # set after track_length is known

        # Speed scale for qualifying buffer laps (1.0 = full speed)
        self.qual_speed_scale = 1.0

        # Hierarchical State Machine transition logic
        self.state_transition = hierarchical_transition

                
        # choose what to do in the different states
        self.state_logic = DefaultStateLogic
            
        # PUBLICATIONS
        self.state_pub = self.create_publisher(String, 'state', 10)
        self.qual_state_pub = self.create_publisher(String, 'qualifying_state', 10)
        self.qual_info_pub = self.create_publisher(String, 'qualifying/info', 10)
        self.telemetry_marker_pub = self.create_publisher(Marker, 'telemetry_marker', 10)
        self.state_marker_pub = self.create_publisher(Marker, 'state_marker', 10)
        self.loc_wpnt_pub = self.create_publisher(WpntArray, 'local_waypoints', 10)
        self.vis_loc_wpnt_pub = self.create_publisher(MarkerArray, 'local_waypoints/markers', 10)
        
        # main loop
        self.main_loop = self.create_timer(1/self.params.rate_hz, self.main_loop_callback)

        # set up ot param reading
        self.init_ot_params()

    #############
    # CALLBACKS #
    #############
    def _force_ftg_cb(self, msg: Bool):
        self.force_ftg = msg.data
        self.get_logger().info(f"[HSM] Force FTG mode updated: {self.force_ftg}")

    def battery_cb(self, msg: VescStateStamped):
        self.battery_level = msg.state.voltage_input
        self.vesc_current = getattr(msg.state, 'current_motor', 0.0)

    def imu_cb(self, msg: Imu):
        self.latest_imu_ax = float(msg.linear_acceleration.x)
        self.latest_imu_ay = float(msg.linear_acceleration.y)
        self.latest_imu_wz = float(msg.angular_velocity.z)

    def scan_cb(self, msg: LaserScan):
        self.latest_scan = msg


    def glb_wpnts_scaled_cb(self, data: WpntArray):
        """
        Callback function of velocity interpolator subscriber.

        Parameters
        ----------
        data
            Data received from velocity interpolator topic
        """
        if len(data.wpnts) >= 2:
            p0 = data.wpnts[0]
            p_last = data.wpnts[-1]
            # Check if last point is a closing duplicate of first point
            if np.hypot(p_last.x_m - p0.x_m, p_last.y_m - p0.y_m) < 0.02:
                self.glb_wpnts = data.wpnts[:-1]
            else:
                self.glb_wpnts = data.wpnts
        else:
            self.glb_wpnts = data.wpnts

        self.num_glb_wpnts = len(self.glb_wpnts)
        self.track_length = data.wpnts[-1].s_m
        self.glb_s_array = np.array([w.s_m for w in self.glb_wpnts])
        # Get spacing between wpnts for rough approximations
        self.wpnt_dist = data.wpnts[1].s_m - data.wpnts[0].s_m if len(data.wpnts) >= 2 else 0.1
        self.waypoints_dist = self.wpnt_dist
        self.gb_max_idx = data.wpnts[-1].id

        if len(self.glb_wpnts) >= 2:
            x0, y0 = self.glb_wpnts[0].x_m, self.glb_wpnts[0].y_m
            x1, y1 = self.glb_wpnts[1].x_m, self.glb_wpnts[1].y_m
            norm = np.linalg.norm([y1 - y0, x0 - x1])
            if norm > 1e-4:
                xy_norm = -np.array([y1 - y0, x0 - x1]) / norm * 1.25 * self.glb_wpnts[0].d_left
                self.x_viz = x0 + xy_norm[0]
                self.y_viz = y0 + xy_norm[1]

    def glb_wpnts_cb(self, data: WpntArray):
        if data.wpnts:
            max_sp = max([wpnt.vx_mps for wpnt in data.wpnts])
            self.max_speed = max_sp if max_sp > 0 else 1.0
        else:
            self.max_speed = 1.0

    def car_state_frenet_cb(self, msg: Odometry):
        self.cur_s = msg.pose.pose.position.x
        self.cur_d = msg.pose.pose.position.y
        self.cur_vs = msg.twist.twist.linear.x  # longitudinal speed in frenet frame

        # Extract localization covariance trace from pose (x and y variances)
        if hasattr(msg.pose, 'covariance') and len(msg.pose.covariance) >= 8:
            cov_x = float(msg.pose.covariance[0])
            cov_y = float(msg.pose.covariance[7])
            self.loc_cov_trace = cov_x + cov_y
        else:
            self.loc_cov_trace = 0.0

        # Lap counting: detect wrap-around when cur_s jumps backwards past the
        # start/finish line.  A drop of more than half the track length is
        # treated as a lap completion regardless of absolute position.
        if self._prev_s is not None and self.track_length > 0:
            drop = self._prev_s - self.cur_s
            if drop > self.track_length * 0.5:
                self.lap_count += 1
                self.get_logger().info(
                    f"[{self.params.mode}] Lap {self.lap_count} completed (s: {self._prev_s:.2f} "
                    f"-> {self.cur_s:.2f}, track_length: {self.track_length:.2f})"
                )
        self._prev_s = self.cur_s

    def avoidance_cb(self, data: OTWpntArray):
        """Subscribes to spliner waypoints"""
        if len(data.wpnts) > 0:
            self.splini_ttl_counter = int(self.params.splini_ttl * self.params.rate_hz)
            self.avoidance_wpnts = data
            self.last_valid_avoidance_wpnts = data.wpnts.copy()
        else:
            self.avoidance_wpnts = data

    def obstacles_cb(self, data):
        if len(data.obstacles) != 0:
            self.obstacles = data.obstacles
        else:
            self.obstacles = []
    
    def car_state_cb(self, data: PoseStamped):
        x = data.pose.position.x
        y = data.pose.position.y
        rot = Rotation.from_quat([data.pose.orientation.x, data.pose.orientation.y, 
                                       data.pose.orientation.z, data.pose.orientation.w])
        rot_euler = rot.as_euler('xyz', degrees=False)
        theta = rot_euler[2]

        self.current_position = [x, y, theta]
    
    @property
    def _check_stuck(self) -> bool:
        """
        Evaluates if car has crashed or is stuck against a wall.
        Fused multi-modal crash detection:
        1. High-G IMU deceleration shock (>18 m/s^2, ~1.8-2.0g) upon wall impact.
        2. VESC motor stall current (>14A) while speed is zero (wheel spinning on slick floor).
        3. Front LiDAR bumper proximity (<0.35m) while stopped or very slow.
        4. Sim Watchdog: In simulation, front clearance < 0.28m rapidly triggers recovery
           before AMCL particles or Pure Pursuit can oscillate.
        5. Cornering / Apex Gating: yaw rate > 0.45 rad/s with high forward velocity suppresses
           false triggers when cutting apexes close to barriers.
        """
        if not self.params.enable_recovery:
            return False

        front_dist, _ = self._evaluate_scan_clearance()
        
        # Effective speed threshold: if pressed against a wall (<0.28m), relax threshold to 0.35 m/s
        # to catch spinning wheels on the wall surface
        speed_thresh = 0.35 if front_dist < 0.28 else self.params.stuck_speed_threshold
        is_slow = abs(self.cur_vs) < speed_thresh
        is_front_blocked = front_dist < self.params.crash_dist_threshold

        # High-G IMU deceleration shock
        imu_shock = False
        shock_thresh = getattr(self.params, 'imu_shock_threshold', 18.0)
        if hasattr(self, 'latest_imu_ax') and self.latest_imu_ax is not None:
            if self.latest_imu_ax < -shock_thresh:
                imu_shock = True

        # Cornering / Apex Gating: suppress false-triggers on apex walls
        is_cornering = False
        corner_wz_thresh = getattr(self.params, 'cornering_yaw_rate_thresh', 0.45)
        if hasattr(self, 'latest_imu_wz') and self.latest_imu_wz is not None:
            if abs(self.latest_imu_wz) > corner_wz_thresh and self.cur_vs > 1.0:
                is_cornering = True

        # VESC motor stall against barrier
        motor_stall = False
        stall_curr_thresh = getattr(self.params, 'stall_current_threshold', 14.0)
        if hasattr(self, 'vesc_current') and self.vesc_current is not None:
            if abs(self.vesc_current) > stall_curr_thresh and is_slow:
                motor_stall = True

        # Multi-modal evaluation
        if (imu_shock and front_dist < 0.45) or (motor_stall and front_dist < 0.40):
            # Instant collision or stall confirmation: rapid accumulation
            self.stuck_counter += 2
        elif is_front_blocked and is_slow and not is_cornering:
            self.stuck_counter += 2 if self.sim else 1
        elif front_dist < 0.22 and not is_cornering:
            # Physical bumper penetration / hard contact against wall in sim or real
            self.stuck_counter += 2
        else:
            self.stuck_counter = max(0, self.stuck_counter - 1)

        threshold_ticks = int(self.params.stuck_time_threshold * self.params.rate_hz)
        return self.stuck_counter >= threshold_ticks

    def _evaluate_scan_clearance(self) -> tuple[float, float]:
        """Calculates front bumper clearance and rear clearance from LiDAR scans.
        Uses exact beam angles from LaserScan to reliably cover the entire front bumper
        arc ([-55 deg, +55 deg]) regardless of whether the sensor has 270 deg or 360 deg FOV.
        """
        if self.latest_scan is None or len(self.latest_scan.ranges) == 0:
            return 1.0, 1.0

        ranges = np.array(self.latest_scan.ranges)
        ranges = np.nan_to_num(ranges, nan=10.0, posinf=10.0, neginf=0.0)
        n = len(ranges)

        angle_min = getattr(self.latest_scan, 'angle_min', -2.356)  # default -135 deg
        angle_max = getattr(self.latest_scan, 'angle_max', 2.356)   # default +135 deg
        angles = np.linspace(angle_min, angle_max, n)

        # Front bumper arc: [-55 deg, +55 deg] (covers head-on and oblique angle wall collisions)
        front_mask = np.abs(angles) <= np.deg2rad(55.0)
        front_dist = float(np.min(ranges[front_mask])) if np.any(front_mask) else 1.0

        # Rear clearance: angles beyond +/- 115 deg (towards the rear corners)
        rear_mask = np.abs(angles) >= np.deg2rad(115.0)
        rear_dist = float(np.min(ranges[rear_mask])) if np.any(rear_mask) else 1.0

        return front_dist, rear_dist

    def _evaluate_recovery_steer_dir(self):
        """Compares left vs right side LiDAR clearance to choose reverse steering direction."""
        if self.latest_scan is None or len(self.latest_scan.ranges) == 0:
            self.recovery_steer_dir = 1.0
            return

        ranges = np.array(self.latest_scan.ranges)
        ranges = np.nan_to_num(ranges, nan=10.0, posinf=10.0, neginf=0.0)
        n = len(ranges)

        angle_min = getattr(self.latest_scan, 'angle_min', -2.356)
        angle_max = getattr(self.latest_scan, 'angle_max', 2.356)
        angles = np.linspace(angle_min, angle_max, n)

        # Right side arc: [-75 deg, -15 deg]
        right_mask = (angles >= np.deg2rad(-75.0)) & (angles <= np.deg2rad(-15.0))
        # Left side arc: [+15 deg, +75 deg]
        left_mask = (angles >= np.deg2rad(15.0)) & (angles <= np.deg2rad(75.0))

        mean_right = float(np.mean(ranges[right_mask])) if np.any(right_mask) else 1.0
        mean_left = float(np.mean(ranges[left_mask])) if np.any(left_mask) else 1.0

        # If left wall is closer (left <= right), steer left (+1.0) while reversing
        # so the rear swings left and the front swings RIGHT into open space away from the left wall
        if mean_left <= mean_right:
            self.recovery_steer_dir = 1.0
        else:
            self.recovery_steer_dir = -1.0
        # so the rear swings left and the front swings RIGHT into open space away from the left wall
        if mean_left <= mean_right:
            self.recovery_steer_dir = 1.0   # steer left -> front swings right away from wall
        else:
            self.recovery_steer_dir = -1.0  # steer right -> front swings left away from wall

        self.get_logger().info(f"[RECOVERY] Side clearance - Left: {mean_left:.2f}m, Right: {mean_right:.2f}m -> Reverse steer dir: {self.recovery_steer_dir}")

    ### Properties to evaulate state transitions
    @property
    def _low_bat(self)->bool:
        battery = getattr(self, 'battery_level', None)
        if battery is not None and battery < self.params.volt_threshold:
            return True
        return False
    
    @property
    def _check_only_ftg_zone(self) -> bool:
        ftg_only = False
        # check if the car is in a ftg only zone, but only if there is an only ftg zone
        if len(self.only_ftg_zones) != 0:
            cur_idx = self.get_closest_glb_wpnt_idx(self.cur_s if self.cur_s is not None else 0.0)
            for sector in self.only_ftg_zones:
                if sector[0] <= cur_idx <= sector[1]:
                    ftg_only = True
                    # rospy.logwarn(f"[{self.name}] IN FTG ONLY ZONE")
                    break  # cannot be in two ftg zones
        return ftg_only

    @property
    def _check_close_to_raceline(self) -> bool:
        return np.abs(self.cur_d) < self.params.gb_ego_width_m  # [m]

    @property
    def _check_localization_healthy(self) -> bool:
        """Evaluates whether position covariance trace is within safe racing limits."""
        if self.loc_cov_trace <= 0.0:
            return True
        max_trace = getattr(self.params, "max_loc_cov_trace", 0.15)
        return self.loc_cov_trace < max_trace

    @property
    def _check_heading_aligned(self) -> bool:
        """Evaluates whether vehicle heading is reasonably aligned with raceline direction (|yaw_err| < 45 deg).
        Prevents violent steering lock snaps when returning from recovery to global raceline tracking.
        """
        if self.cur_s is None or not self.glb_wpnts or not hasattr(self, 'current_position') or self.current_position is None:
            return True
        idx = self.get_closest_glb_wpnt_idx(self.cur_s)
        wpnt = self.glb_wpnts[idx]
        target_yaw = getattr(wpnt, 'psi_rad', 0.0)
        veh_yaw = self.current_position[2]
        delta_yaw = abs((veh_yaw - target_yaw + np.pi) % (2.0 * np.pi) - np.pi)
        return delta_yaw < np.deg2rad(30.0)

    def trigger_relocalization(self):
        """Re-seeds particle filter via /initialpose near the last known raceline s when in open track."""
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - self.last_reloc_time < 3.0:
            return

        # Crucial safety check: Only re-seed if the car has backed away into open space
        # (front_dist >= 0.45m). Never re-seed while pressed nose-first into a barrier!
        front_dist, _ = self._evaluate_scan_clearance()
        if front_dist < 0.45:
            self.get_logger().warn(
                f"[Relocalization] Front clearance too close ({front_dist:.2f}m < 0.45m). "
                f"Deferring /initialpose re-seed until vehicle backs out into open corridor.",
                throttle_duration_sec=1.0
            )
            return

        self.last_reloc_time = now_sec
        if self.glb_wpnts is None or len(self.glb_wpnts) == 0:
            return

        cur_s = self.cur_s if self.cur_s is not None else 0.0
        closest_idx = self.get_closest_glb_wpnt_idx(cur_s)
        wpnt = self.glb_wpnts[closest_idx]

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = float(wpnt.x_m)
        pose_msg.pose.pose.position.y = float(wpnt.y_m)
        pose_msg.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, float(wpnt.psi_rad))
        pose_msg.pose.pose.orientation.x = q[0]
        pose_msg.pose.pose.orientation.y = q[1]
        pose_msg.pose.pose.orientation.z = q[2]
        pose_msg.pose.pose.orientation.w = q[3]

        cov = [0.0] * 36
        cov[0] = 0.16   # var(x)
        cov[7] = 0.16   # var(y)
        cov[35] = 0.08  # var(yaw)
        pose_msg.pose.covariance = cov

        self.get_logger().warn(
            f"[Relocalization] Kidnapped / lost pose detected (cov_trace={self.loc_cov_trace:.3f} m^2). "
            f"Auto-reseeding particle filter at raceline s={cur_s:.2f}m (x={wpnt.x_m:.2f}, y={wpnt.y_m:.2f})."
        )
        self.initialpose_pub.publish(pose_msg)

    @property
    def _check_opponent_behind(self) -> bool:
        """Returns True if the primary tracked obstacle is cleanly behind ego (pass completed)."""
        if not self.obstacles or self.cur_s is None or self.track_length is None:
            return False
        half_track = self.track_length / 2.0
        for obs in self.obstacles:
            # relative distance: positive = ahead of ego, negative = behind ego
            rel_s = (obs.s_center - self.cur_s + half_track) % self.track_length - half_track
            # Only consider passed once ego is cleanly ahead (obstacle is at least 1.2m behind ego center)
            if -5.5 <= rel_s <= -1.2:
                return True
        return False

    @property
    def _check_alongside_obstacle(self) -> bool:
        """Returns True if ego is currently physically alongside or overlapping any tracked obstacle.
        Used to prevent aborting an overtake maneuver while alongside, which would cause the vehicle
        to brake to a dead stop side-by-side with the obstacle.
        """
        if not self.obstacles or self.cur_s is None or self.track_length is None:
            return False
        half_track = self.track_length / 2.0
        for obs in self.obstacles:
            rel_s = (obs.s_center - self.cur_s + half_track) % self.track_length - half_track
            # Physical obstacle influence window (-1.2m behind to +1.2m ahead)
            if -1.2 <= rel_s <= 1.2:
                return True
        return False

    @property
    def _check_valid_opponent_ahead(self) -> bool:
        """Dynamic Overtake Guardrail:
        Verifies that at least one tracked obstacle ahead qualifies as a real opponent
        within the active passing window rather than distant artifacts.
        """
        if not self.obstacles or self.cur_s is None or self.track_length is None:
            return False

        horizon = getattr(self.params, 'overtaking_horizon_m', 3.5)
        for obs in self.obstacles:
            gap = (obs.s_center - self.cur_s) % self.track_length
            if 0.50 <= gap <= horizon:
                obs_size = getattr(obs, 'size', 0.0)
                if obs_size <= 0.0 and hasattr(obs, 's_start') and hasattr(obs, 's_end'):
                    obs_size = abs(obs.s_end - obs.s_start)
                if obs_size >= 0.15:
                    return True
        return False

    @property
    def _is_primary_obstacle_static(self) -> bool:
        """Determines if the closest obstacle ahead is stationary (< 0.5 m/s or marked static)."""
        if not self.obstacles or self.cur_s is None or self.track_length is None:
            return False
        closest_gap = float('inf')
        is_static = False
        for obs in self.obstacles:
            gap = (obs.s_center - self.cur_s) % self.track_length
            if gap < closest_gap:
                closest_gap = gap
                obs_speed = getattr(obs, 'vs', 0.0)
                is_static = getattr(obs, 'is_static', False) or (obs_speed < 0.5)
        return is_static

    def _check_dynamic_track_feasibility(self, cur_idx: int, runway_wpnts: int = 15, max_curv: float = 0.65) -> bool:
        """Evaluates track geometry over the upcoming passing horizon (Pillar 1: Dynamic Horizon Feasibility).
        Ensures upcoming track curvature is within safe tire limits and track has sufficient drivable width.
        Speed-Adaptive: When passing a stationary obstacle or moving at approach speed (< 2.0 m/s),
        allowable curvature scales dynamically with physical tire grip limit (v^2 * kappa <= a_lat_max).
        """
        if not self.glb_wpnts or len(self.glb_wpnts) == 0:
            return True
        N = len(self.glb_wpnts)
        ego_speed = max(0.5, self.cur_vs if self.cur_vs is not None else 0.0)

        # Dynamic curvature threshold based on physical tire friction demand
        # Moving racecars at 4-6 m/s need max_curv <= 0.65 rad/m to prevent high-speed slide
        # Crawling / stationary obstacle avoidance (< 2.0 m/s) allows up to 1.35 rad/m
        if self._is_primary_obstacle_static or ego_speed < 2.0:
            eff_max_curv = max(max_curv, min(1.35, 6.0 / (ego_speed**2 + 1.0)))
            eff_runway = min(runway_wpnts, 8)
            eff_min_width = 1.10  # Allow narrower corridors for low-speed pass
        else:
            eff_max_curv = max_curv
            eff_runway = runway_wpnts
            eff_min_width = 1.40

        wpnt_spacing = (self.track_length / N) if (self.track_length is not None and N > 0) else 0.1
        dyn_runway = max(eff_runway, int((ego_speed * 0.8) / max(0.05, wpnt_spacing)))

        for offset in range(dyn_runway):
            idx = (cur_idx + offset) % N
            w = self.glb_wpnts[idx]
            curv = abs(getattr(w, 'kappa_radpm', 0.0))
            if curv > eff_max_curv:
                return False
            d_left = getattr(w, 'd_left', 1.0)
            d_right = abs(getattr(w, 'd_right', 1.0))
            if d_left > 0.05 and d_right > 0.05:
                width = d_left + d_right
                if width < eff_min_width:
                    return False
        return True

    @property
    def _check_ot_sector(self) -> bool:
        cur_idx = self.get_closest_glb_wpnt_idx(self.cur_s if self.cur_s is not None else 0.0)
        if self.params.overtake_mode == "spliner":
            # Guardrail 1: Dynamic Track Feasibility Check (Absolute Safety Floor)
            if not self._check_dynamic_track_feasibility(cur_idx, runway_wpnts=12, max_curv=0.65):
                return False

            # Guardrail 2: Sector Gating (Static ot_sectors is strictly authoritative!)
            # Hairpins and chicanes marked with ot_flag: false must NEVER be bypassed!
            if hasattr(self, 'ot_sectors') and len(self.ot_sectors) > 0:
                for sector in self.ot_sectors:
                    if sector['ot_flag']:
                        if sector['start'] <= sector['end']:
                            if sector['start'] <= cur_idx <= (sector['end'] + 1):
                                return True
                        else:  # Track finish line wrap-around
                            if cur_idx >= sector['start'] or cur_idx <= (sector['end'] + 1):
                                return True
                return False
            return True

        for sector in (getattr(self, 'ot_sectors', []) or []):
            if sector['ot_flag']:
                if sector['start'] <= sector['end']:
                    if sector['start'] <= cur_idx <= (sector['end'] + 1):
                        return True
                else:
                    if cur_idx >= sector['start'] or cur_idx <= (sector['end'] + 1):
                        return True
        return False

    @property
    def _check_can_initiate_overtake(self) -> bool:
        """Evaluates whether all dynamic and sector racecraft conditions are met
        to safely launch an overtake from TRAILING mode.
        """
        # 0. Overtake Cooldown: enforce settling in TRAILING before re-attacking moving opponents
        # Prevents high-frequency chattering and perpetual offline dwelling.
        # Static obstacles (stationary barriers) bypass cooldown to enable instant continuous evasion.
        now = self.get_clock().now()
        is_any_obs_static = any(
            (getattr(o, 'is_static', False) or (getattr(o, 'vs', 0.0) < 0.5))
            for o in (self.obstacles or [])
        )
        if not is_any_obs_static and hasattr(self, 'last_ot_exit_time') and self.last_ot_exit_time is not None:
            time_since_ot = (now - self.last_ot_exit_time).nanoseconds / 1e9
            if time_since_ot < 0.8:
                return False

        # 1. Must be in an approved overtaking sector
        if not self._check_ot_sector:
            return False

        cur_idx = self.get_closest_glb_wpnt_idx(self.cur_s if self.cur_s is not None else 0.0)

        # 2. Runway enforcement: Must have at least 20 waypoints (~2.0m) remaining in passing sector
        # Prohibits launching late passes that cannot finish before the braking zone/corner.
        if hasattr(self, 'ot_sectors') and len(self.ot_sectors) > 0:
            in_safe_runway = False
            N = len(self.glb_wpnts) if self.glb_wpnts else 1000
            for sector in self.ot_sectors:
                if sector['ot_flag']:
                    # For full-track continuous loops (end - start >= N - 5), runway is always safe
                    if (sector['end'] - sector['start'] >= N - 5):
                        in_safe_runway = True
                        break
                    min_runway_wpnts = 20
                    if sector['start'] <= cur_idx <= max(sector['start'], sector['end'] - min_runway_wpnts):
                        in_safe_runway = True
                        break
            if not in_safe_runway:
                return False

        # Dynamic curvature feasibility across the runway
        if not self._check_dynamic_track_feasibility(cur_idx, runway_wpnts=12, max_curv=0.65):
            return False

        # 3. Must be within the decisive passing window
        # For static obstacles: initiate early (up to gb_horizon_m, default 4.5m) to avoid deceleration lag
        # For dynamic opponents: wait for slipstream closing window (0.60m <= gap <= 3.60m)
        if not self.obstacles or self.cur_s is None or self.track_length is None:
            return False

        in_passing_window = False
        opp_speed = 0.0
        is_static = False
        horizon_m = getattr(self.params, 'gb_horizon_m', 4.5)

        for obs in self.obstacles:
            half_track = self.track_length / 2.0
            s_rel = (obs.s_center - self.cur_s + half_track) % self.track_length - half_track
            obs_speed = getattr(obs, 'vs', 0.0)
            obs_is_static = getattr(obs, 'is_static', False) or (obs_speed < 0.5)

            # Static obstacles can be passed from the full detection horizon down through alongside (-2.5m)
            # Moving opponents use the tactical slipstream window (0.40m to 3.60m)
            max_gap = max(3.60, min(5.50, horizon_m)) if obs_is_static else 3.60
            min_gap = -2.50 if obs_is_static else 0.40
            if min_gap <= s_rel <= max_gap:
                obs_size = getattr(obs, 'size', 0.0)
                if obs_size <= 0.0 and hasattr(obs, 's_start') and hasattr(obs, 's_end'):
                    obs_size = abs(obs.s_end - obs.s_start)
                if obs_size >= 0.15 or obs_is_static:
                    in_passing_window = True
                    opp_speed = obs_speed
                    is_static = obs_is_static
                    break

        if not in_passing_window:
            return False

        # 4. Trailing Dwell Stability: ego must have settled in TRAILING for at least 0.25s
        # Guarantees PID trailing speed synchronization and stable raceline tracking before pulling out.
        # Bypass dwell for static obstacles: stationary obstacles require zero speed synchronization!
        if not is_static and getattr(self, 'state_dwell_time', 0.0) < 0.25:
            return False

        # 5. Speed / momentum requirement:
        # Dynamic slipstream check applies only when attacking a MOVING opponent (vs >= 0.5 m/s).
        # Static obstacles (vs < 0.5 m/s or is_static) can be overtaken immediately (even from a stop)
        # as long as the avoidance corridor is clear and available.
        ego_speed = self.cur_vs if self.cur_vs is not None else 0.0
        if not is_static and opp_speed >= 0.5:
            if ego_speed < max(1.2, opp_speed - 0.35):
                return False

        # 6. Avoidance corridor availability and clearance
        if not self._check_availability_splini_wpts or not self._check_ofree:
            return False

        return True

    @property
    def _check_ofree(self) -> bool:
        o_free = True

        if self.params.overtake_mode == "spliner":
            if self.last_valid_avoidance_wpnts is not None and len(self.last_valid_avoidance_wpnts) > 0:
                horizon = self.params.overtaking_horizon_m  # Horizon in front of cur_s [m]
                half_track = self.track_length / 2.0

                for obs in self.obstacles:
                    obs_s = obs.s_center
                    # Wrapping distance to check if in front
                    dist_to_obj = (obs_s - self.cur_s) % self.track_length
                    if dist_to_obj < horizon:
                        # 1. Live alongside check: if ego is physically alongside the obstacle right now,
                        # verify obstacle has not swerved laterally into ego's path
                        ego_s_diff = abs((self.cur_s - obs_s + half_track) % self.track_length - half_track)
                        obs_len = max(0.30, abs(getattr(obs, 's_end', obs_s) - getattr(obs, 's_start', obs_s)))
                        if ego_s_diff < (obs_len / 2.0 + 0.20):
                            ego_lat_clearance = abs(self.cur_d - obs.d_center)
                            if ego_lat_clearance < 0.15:
                                o_free = False
                                self.get_logger().warn(
                                    f"[HSM] O_FREE False: obstacle encroaching alongside ego (lat_diff: {ego_lat_clearance:.3f}m < 0.15m)",
                                    throttle_duration_sec=1.0
                                )
                                break

                        # 2. Apex clearance check: verify the planned avoidance path provides safe lateral passing distance
                        peak_avoid = max(self.last_valid_avoidance_wpnts, key=lambda w: abs(w.d_m))
                        obs_d_center = obs.d_center
                        obs_rad = max(0.12, abs(getattr(obs, 'd_left', obs_d_center + 0.15) - getattr(obs, 'd_right', obs_d_center - 0.15)) / 2.0)
                        effective_margin = min(float(self.params.lateral_width_ot_m), 0.12)

                        # Check peak clearance of the avoidance corridor plateau
                        peak_clearance = abs(peak_avoid.d_m - obs_d_center) - obs_rad
                        if peak_clearance < effective_margin:
                            o_free = False
                            self.get_logger().info(
                                f"[HSM] O_FREE False: planned apex clearance ({peak_clearance:.3f} m) < safety margin ({effective_margin:.3f} m)",
                                throttle_duration_sec=1.0
                            )
                            break
            else:
                o_free = True
            return o_free
        else:
            self.get_logger().error(f"Unknown overtake planner")
            raise NotImplementedError

    @property
    def _check_gbfree(self) -> bool:
        gb_free = True
        # If we are in time trial only mode -> return free overtake i.e. GB_FREE True
        horizon = self.params.gb_horizon_m  # Horizon in front of cur_s [m]

        for obs in self.obstacles:
            obs_s = (obs.s_start + obs.s_end) / 2
            obs_s = obs.s_center
            gap = (obs_s - self.cur_s) % self.track_length
            if gap < horizon:
                obs_d = obs.d_center
                # Get d wrt to mincurv from the overtaking line
                if abs(obs_d) < self.params.lateral_width_gb_m:
                    gb_free = False
                    #self.get_logger().info(f"GB_FREE False, obs dist to ot lane: {obs_d} m")
                    break

        return gb_free

    @property
    def _check_enemy_in_front(self) -> bool:
        horizon = self.params.gb_horizon_m  # Horizon in front of cur_s [m]
        for obs in self.obstacles:
            gap = (obs.s_start - self.cur_s) % self.track_length
            if gap < horizon:
                return True
        return False

    @property
    def _check_availability_splini_wpts(self) -> bool:
        if self.avoidance_wpnts is None:
            return False
        elif len(self.avoidance_wpnts.wpnts) == 0:
            return False
        # Say no to the ot line if the last switch was less than 0.75 seconds ago
        elif (
            abs(time_to_float(self.avoidance_wpnts.header.stamp) - time_to_float(self.avoidance_wpnts.last_switch_time))
            < self.params.splini_hyst_timer_sec
        ):
            self.get_logger().debug(f"Still too fresh into the switch...{abs(time_to_float(self.avoidance_wpnts.last_switch_time) - time_to_float(self.get_clock().now().to_msg()))}")
            return False
        else:
            # If the splinis are valid update the last valid ones
            self.last_valid_avoidance_wpnts = self.avoidance_wpnts.wpnts.copy()
            return True

    @property
    def _check_ftg(self) -> bool:
        # If we have been standing still for 3 seconds inside TRAILING -> FTG
        threshold = self.params.ftg_timer_sec * self.params.rate_hz
        if self.params.ftg_disabled:
            return False
        else:
            if self.state == StateType.TRAILING and self.cur_vs < self.params.ftg_threshold_speed:
                self.ftg_counter += 1
                self.get_logger().warn(f"[{self.get_name()}] FTG counter: {self.ftg_counter}/{threshold}")
            else:
                self.ftg_counter = 0

            if self.ftg_counter > threshold:
                return True
            else:
                return False

    @property
    def _check_emergency_break(self) -> bool:
        # NOTE: unused flag, but could be useful
        emergency_break = False
        if self.obstacles != []:
            horizon = self.emergency_break_horizon # Horizon in front of cur_s [m]

            for obs in self.obstacles:
                # Wrapping madness to check if infront
                dist_to_obj = (obs.s_center - self.cur_s) % self.track_length
                if dist_to_obj < horizon:
            
                    # Get d wrt to mincurv from the overtaking line
                    local_wpnt_idx = np.argmin(
                        np.array([abs(avoid_s.s_m - obs.s_center) for avoid_s in self.local_waypoints.wpnts])
                    )
                    ot_d = self.local_waypoints.wpnts[local_wpnt_idx].d_m
                    ot_obs_dist = ot_d - obs.d_center
                    if abs(ot_obs_dist) < self.params.lateral_width_ot_m:
                        emergency_break = True
                        self.get_logger().info("emergency break")
        else:
            emergency_break = False
        return emergency_break
    ###########
    def get_closest_glb_wpnt_idx(self, s: float) -> int:
        """Find the index of the global waypoint with s_m closest to the given s."""
        if self.glb_wpnts is None or self.num_glb_wpnts == 0:
            return 0
        if not hasattr(self, 'glb_s_array') or len(self.glb_s_array) == 0:
            self.glb_s_array = np.array([w.s_m for w in self.glb_wpnts])
        s_wrapped = s % self.track_length if self.track_length > 0 else s
        idx = int(np.searchsorted(self.glb_s_array, s_wrapped))
        if idx >= self.num_glb_wpnts:
            idx = 0
        elif idx > 0:
            if abs(self.glb_s_array[idx - 1] - s_wrapped) < abs(self.glb_s_array[idx] - s_wrapped):
                idx = idx - 1
        return idx

    def get_avoidance_local_wpnts(self) -> List[Wpnt]:
        """Directly construct the continuous n_loc_wpnts local trajectory from the
        avoidance spline ahead of the car, seamlessly continuing on the global path.
        """
        avoid_wpnts = self.last_valid_avoidance_wpnts
        if avoid_wpnts is None or len(avoid_wpnts) == 0:
            s_idx = self.get_closest_glb_wpnt_idx(self.cur_s if self.cur_s is not None else 0.0)
            return [self.glb_wpnts[(s_idx + i) % self.num_glb_wpnts] for i in range(self.params.n_loc_wpnts)]

        cur_s = (self.cur_s % self.track_length) if (self.cur_s is not None and self.track_length > 0) else 0.0
        # Determine if avoidance spline wraps around the track end
        wraps = avoid_wpnts[-1].s_m < avoid_wpnts[0].s_m

        # Filter avoidance waypoints ahead of the car
        if wraps:
            ahead_avoid = [w for w in avoid_wpnts if w.s_m >= cur_s or w.s_m <= avoid_wpnts[-1].s_m]
        else:
            ahead_avoid = [w for w in avoid_wpnts if w.s_m >= cur_s]

        # If car is past the avoidance spline or empty, fallback to global
        if len(ahead_avoid) == 0:
            s_idx = self.get_closest_glb_wpnt_idx(cur_s)
            return [self.glb_wpnts[(s_idx + i) % self.num_glb_wpnts] for i in range(self.params.n_loc_wpnts)]

        # Cap avoidance points to n_loc_wpnts
        ahead_avoid = ahead_avoid[:self.params.n_loc_wpnts]

        # Seamlessly pad the remaining waypoints from the global raceline resuming after the spline
        needed = self.params.n_loc_wpnts - len(ahead_avoid)
        if needed > 0:
            end_s = avoid_wpnts[-1].s_m
            resume_idx = (self.get_closest_glb_wpnt_idx(end_s) + 1) % self.num_glb_wpnts
            resume_pts = [self.glb_wpnts[(resume_idx + i) % self.num_glb_wpnts] for i in range(needed)]
            return ahead_avoid + resume_pts
        return ahead_avoid

    def get_splini_wpts(self) -> WpntArray:
        """Obtain the waypoints by fusing those obtained by spliner with the
        global ones.
        """
        splini_glob = self.glb_wpnts.copy()

        # Handle wrapping
        if self.last_valid_avoidance_wpnts is not None:
            if len(self.last_valid_avoidance_wpnts) == 0:
                splini_idxs = [] # Avoid index error if empty
            else:
                start_s = self.get_closest_glb_wpnt_idx(self.last_valid_avoidance_wpnts[0].s_m)
                end_s = self.get_closest_glb_wpnt_idx(self.last_valid_avoidance_wpnts[-1].s_m)
                if end_s >= start_s:
                    splini_idxs = list(range(start_s, end_s + 1))
                else:
                    splini_idxs = list(range(start_s, self.num_glb_wpnts)) + list(range(0, end_s + 1))

            M = len(self.last_valid_avoidance_wpnts)
            K = len(splini_idxs)
            for j, s in enumerate(splini_idxs):
                i = round(j * (M - 1) / (K - 1)) if K > 1 else 0
                splini_glob[s] = self.last_valid_avoidance_wpnts[min(i, M - 1)]

        # If the last valid points have been reset, then we just pass the global waypoints
        else:
            self.get_logger().warn(f"No valid avoidance waypoints, passing global waypoints")
            pass

        return splini_glob

    def init_ot_params(self):
        """Obtain overtaking sector definitions from the active map's ot_sectors.yaml file,
        or fall back to the parameter server if running graph_based mode.
        """
        self.n_ot_sectors = None
        self.ot_param_names = None
        self.ot_sectors = []

        # 1. Attempt direct YAML load from maps folder (Single Source of Truth for both spliner & graph_based)
        map_name = getattr(self, 'active_map_name', '') or getattr(self.params, 'map_name', '') or 'ifac_roboracer'
        candidates = [
            f"/home/napat/F110th/assets/maps/{map_name}/waypoints/ot_sectors.yaml",
            f"/home/napat/F110th/assets/maps/{map_name}/ot_sectors.yaml",
            f"/Users/napat/Developer/F110th/assets/maps/{map_name}/waypoints/ot_sectors.yaml",
            f"/Users/napat/Developer/F110th/assets/maps/{map_name}/ot_sectors.yaml",
        ]
        for c in candidates:
            if os.path.exists(c):
                try:
                    with open(c, 'r') as f:
                        data = yaml.safe_load(f)
                    ot_params = data.get('ot_interpolator', {}).get('ros__parameters', {})
                    n_sectors = ot_params.get('n_sectors', 0)
                    for i in range(n_sectors):
                        sec = ot_params.get(f'Overtaking_sector{i}', {})
                        if 'start' in sec and 'end' in sec:
                            self.ot_sectors.append({
                                'start': int(sec['start']),
                                'end': int(sec['end']),
                                'ot_flag': bool(sec.get('ot_flag', False))
                            })
                    if len(self.ot_sectors) > 0:
                        self.n_ot_sectors = len(self.ot_sectors)
                        self.get_logger().info(f"[State Machine] Successfully loaded {self.n_ot_sectors} OT sectors from {c}: {self.ot_sectors}")
                        return
                except Exception as e:
                    self.get_logger().warn(f"[State Machine] Error parsing {c}: {e}")

        if self.params.overtake_mode != "graph_based":
            return

        self.parameter_client = self.create_client(GetParameters, '/ot_interpolator/get_parameters')
        if not self.parameter_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('ot_interpolator service not available, skipping OT parameters.')
            return
        
        request = GetParameters.Request()
        request.names = ['n_sectors']
        future = self.parameter_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # TODO structure of request is also very ugly dependent on the parameter structure on the other side, could be improved
        if future.result() is not None:
            self.n_ot_sectors = future.result().values[0].integer_value # TODO careful with this types, TY util could be better
            self.get_logger().info(f'Got {self.n_ot_sectors} sectors')

            starts = [f'Overtaking_sector{i}.start' for i in range(self.n_ot_sectors)]
            ends = [f'Overtaking_sector{i}.end' for i in range(self.n_ot_sectors)]
            ot_flags = [f'Overtaking_sector{i}.ot_flag' for i in range(self.n_ot_sectors)]
            self.ot_param_names = ot_flags
            self.ot_sectors = []

            request = GetParameters.Request()
            request.names = starts + ends + ot_flags
            self.get_logger().info(f'Getting OT params: {request.names}')
            future = self.parameter_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                for i in range(self.n_ot_sectors):
                    start = future.result().values[i].integer_value # TODO careful with this types, TY util could be better
                    end = future.result().values[i + self.n_ot_sectors].integer_value # TODO careful with this types, TY util could be better
                    ot_flag = future.result().values[i + 2 * self.n_ot_sectors].bool_value # TODO careful with this types, TY util could be better
                    self.ot_sectors.append({'start': start, 'end': end, 'ot_flag': ot_flag})
                self.get_logger().info(f'OT sectors obtained: {self.ot_sectors}')
            else:
                self.get_logger().error(f'Service call failed {future.exception()}')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
        
        # once initialization is finished, the event handler can be used
        self.handler = ParameterEventHandler(self)

        for ot_flag in ot_flags:
            self.handler.add_parameter_callback(
                ot_flag,
                self.ot_flag_cb,
                'ot_interpolator'
            )

    def ot_flag_cb(self, p: Parameter):
        self.get_logger().info(f'OT flag changed: {p.name} {p.value.bool_value}')
        changed_sector_int = int(p.name.split('.')[0][-1]) # TODO ultra hardcoded ugliness
        self.ot_sectors[changed_sector_int]['ot_flag'] = p.value.bool_value # TODO careful with this types, TY util could be better

    def get_ot_params(self):
        # the first time we need to get the number of sectors
        self.get_logger().info(f'Getting only ot flags params')
        # otherwise we only update the flags
        request = GetParameters.Request()
        request.names = self.ot_param_names
        future = self.parameter_client.call_async(request)
        self.executor.spin_until_future_complete(future)
        if future.result() is not None:
            for i in range(self.n_ot_sectors):
                self.ot_sectors[i]['ot_flag'] = future.result().values[i].bool_value
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')

        self.get_logger().info(f'OT sectors: {self.ot_sectors}')

    ###############
    # PUB HELPERS #
    ###############
    def _pub_local_waypoints(self, wpts: WpntArray):
        loc_markers = MarkerArray()
        # Add a DELETEALL marker to clear previous markers and prevent stale/ghost markers
        del_mrk = Marker()
        del_mrk.action = Marker.DELETEALL
        del_mrk.header.frame_id = "map"
        loc_markers.markers.append(del_mrk)

        loc_wpnts = wpts
        # set stamp to now         
        loc_wpnts.header.stamp = self.get_clock().now().to_msg()
        loc_wpnts.header.frame_id = "map"

        # Apply speed scaling during qualifying buffer laps or OVERTAKE mode
        active_scale = 1.0
        if self.params.mode == 'qualifying' and self.qual_speed_scale != 1.0:
            active_scale *= self.qual_speed_scale
        if self.state == StateType.OVERTAKE and hasattr(self.params, 'overtake_speed_scale') and self.params.overtake_speed_scale != 1.0:
            active_scale *= self.params.overtake_speed_scale

        if active_scale != 1.0:
            from f110_msgs.msg import Wpnt
            scaled_wpnts = []
            for wpnt in loc_wpnts.wpnts:
                new_wpnt = Wpnt()
                new_wpnt.id = wpnt.id
                new_wpnt.s_m = wpnt.s_m
                new_wpnt.d_m = wpnt.d_m
                new_wpnt.x_m = wpnt.x_m
                new_wpnt.y_m = wpnt.y_m
                new_wpnt.d_right = wpnt.d_right
                new_wpnt.d_left = wpnt.d_left
                new_wpnt.psi_rad = wpnt.psi_rad
                new_wpnt.kappa_radpm = wpnt.kappa_radpm
                new_wpnt.vx_mps = wpnt.vx_mps * active_scale
                new_wpnt.ax_mps2 = wpnt.ax_mps2
                scaled_wpnts.append(new_wpnt)
            loc_wpnts.wpnts = scaled_wpnts

        for i, wpnt in enumerate(loc_wpnts.wpnts):
            mrk = Marker()
            mrk.header.frame_id = "map"
            mrk.type = mrk.SPHERE
            mrk.scale.x = 0.15
            mrk.scale.y = 0.15
            mrk.scale.z = 0.15
            mrk.color.a = 1.0
            mrk.color.g = 1.0

            mrk.id = i
            mrk.pose.position.x = wpnt.x_m
            mrk.pose.position.y = wpnt.y_m
            # Keep markers at a constant z slightly above the map plane (0.05m)
            # to prevent 3D perspective distortion or diving below the surface on deceleration
            mrk.pose.position.z = 0.05
            mrk.pose.orientation.w = 1.0
            loc_markers.markers.append(mrk)

        # ...

        if len(loc_wpnts.wpnts) == 0:
            if self.state not in [StateType.FTGONLY, StateType.RECOVERY_STUCK, StateType.RECOVERY_REVERSE, StateType.RECOVERY_ALIGN, StateType.LOW_BAT]:
                self.get_logger().warn("No local waypoints published...")
        else:
            self.loc_wpnt_pub.publish(loc_wpnts)

        self.vis_loc_wpnt_pub.publish(loc_markers)
    
    def visualize_state(self, state: StateType):
        """
        Function that visualizes the state of the car by displaying a colored cube in RVIZ.

        Parameters
        ----------
        action
            Current state of the car to be displayed
        """
        if self.first_visualization:
            self.first_visualization = False
            x0 = self.glb_wpnts[0].x_m
            y0 = self.glb_wpnts[0].y_m
            x1 = self.glb_wpnts[1].x_m
            y1 = self.glb_wpnts[1].y_m
            # compute normal vector of 125% length of trackboundary but to the left of the trajectory
            xy_norm = (
                -np.array([y1 - y0, x0 - x1]) / np.linalg.norm([y1 - y0, x0 - x1]) * 1.25 * self.glb_wpnts[0].d_left
            )

            self.x_viz = x0 + xy_norm[0]
            self.y_viz = y0 + xy_norm[1]

        mrk = Marker()
        mrk.type = mrk.SPHERE
        mrk.id = int(1)
        mrk.header.frame_id = "map"
        mrk.header.stamp = self.get_clock().now().to_msg()
        mrk.color.a = 1.0
        mrk.color.g = 1.0
        mrk.pose.position.x = float(self.x_viz)
        mrk.pose.position.y = float(self.y_viz)
        mrk.pose.position.z = 0.0
        mrk.pose.orientation.w = 1.0
        mrk.scale.x = 1.0
        mrk.scale.y = 1.0
        mrk.scale.z = 1.0

        # Set color and log info based on the state of the car
        if state == StateType.GB_TRACK:
            mrk.color.g = 1.0
        elif state == StateType.OVERTAKE:
            mrk.color.r = 1.0
            mrk.color.g = 1.0
            mrk.color.b = 1.0
        elif state == StateType.TRAILING:
            mrk.color.r = 0.0
            mrk.color.g = 0.0
            mrk.color.b = 1.0
        elif state == StateType.FTGONLY:
            mrk.color.r = 1.0
            mrk.color.g = 0.0
            mrk.color.b = 0.0
        elif state in [StateType.RECOVERY_STUCK, StateType.RECOVERY_REVERSE, StateType.RECOVERY_ALIGN]:
            mrk.color.r = 1.0
            mrk.color.g = 0.5  # Orange for recovery
            mrk.color.b = 0.0
        self.state_marker_pub.publish(mrk)

    def pub_telemetry_marker(self, qual_phase: str):
        mrk = Marker()
        mrk.type = Marker.TEXT_VIEW_FACING
        mrk.id = int(2)
        mrk.header.frame_id = "map"
        mrk.header.stamp = self.get_clock().now().to_msg()
        mrk.scale.z = 0.5  # Text height
        mrk.color.a = 1.0
        mrk.color.r = 1.0
        mrk.color.g = 1.0
        mrk.color.b = 1.0

        if hasattr(self, 'current_position') and self.current_position is not None:
            mrk.pose.position.x = float(self.current_position[0])
            mrk.pose.position.y = float(self.current_position[1])
            mrk.pose.position.z = 1.2
        else:
            mrk.pose.position.x = float(self.x_viz)
            mrk.pose.position.y = float(self.y_viz)
            mrk.pose.position.z = 1.5

        mrk.pose.orientation.w = 1.0
        mode_str = self.top_level_mode.value if hasattr(self.top_level_mode, 'value') else str(self.top_level_mode)
        state_str = self.state.value if hasattr(self.state, 'value') else str(self.state)

        if self.params.mode == "qualifying":
            mrk.text = f"MODE: {mode_str}\nSTATE: {state_str}\nQUAL: {qual_phase} (Lap {self.lap_count})"
        else:
            mrk.text = f"MODE: {mode_str}\nSTATE: {state_str}"
        
        self.telemetry_marker_pub.publish(mrk)


    #############
    # MAIN LOOP #
    #############    
    def main_loop_callback(self):
        self.get_logger().debug(f"Current state: {self.state}")
        # transition logic
        if self.cur_s is not None:
            if self.params.mode == "qualifying":
                self._handle_qualifying_logic()

        now = self.get_clock().now()
        if not hasattr(self, 'state_start_time') or self.state_start_time is None:
            self.state_start_time = now

        self.state_dwell_time = (now - self.state_start_time).nanoseconds / 1e9

        if self.params.force_state:
            new_state = self.params.force_state_choice
        else:
            new_state = self.state_transition(self)

        if new_state != self.state:
            if self.state == StateType.OVERTAKE:
                self.last_ot_exit_time = now
            self.state_start_time = now
            self.state_dwell_time = 0.0
            self.state = new_state

        state_str = self.state.value if hasattr(self.state, 'value') else str(self.state)
        msg = String()
        msg.data = state_str
        self.state_pub.publish(msg)
        self.visualize_state(state=self.state)

        if self.params.mode == "qualifying":
            qual_phase = get_qualifying_phase(self)
        else:
            qual_phase = "N/A"

        # Publish qualifying state and telemetry json
        q_state_msg = String()
        q_state_msg.data = qual_phase
        self.qual_state_pub.publish(q_state_msg)

        q_info_msg = String()
        q_info_msg.data = json.dumps({
            "mode": self.params.mode,
            "phase": qual_phase,
            "lap": self.lap_count,
            "speed_scale": round(float(self.qual_speed_scale), 2),
            "state": state_str
        })
        self.qual_info_pub.publish(q_info_msg)

        self.pub_telemetry_marker(qual_phase=qual_phase)

        self.local_waypoints.wpnts = self.state_logic(self)
        self._pub_local_waypoints(self.local_waypoints)

        if self.params.mode in ["head_to_head", "qualifying"] and self.params.overtake_mode == "spliner":
            self.splini_ttl_counter -= 1
            # Once ttl has reached 0 we overwrite the avoidance waypoints with the empty waypoints
            if self.splini_ttl_counter <= 0:
                self.last_valid_avoidance_wpnts = None
                self.avoidance_wpnts = WpntArray()
                self.splini_ttl_counter = -1

    def _handle_qualifying_logic(self):
        """Removes fake obstacles during Qualifying if any existed. (Now handled externally)"""
        pass

# defined as entry point in setup.py:
def main(args=None):
    rclpy.init(args=args)

    state_machine = StateMachine()

    rclpy.spin(state_machine)

    state_machine.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
