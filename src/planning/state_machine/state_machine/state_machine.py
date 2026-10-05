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
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
from f110_msgs.msg import WpntArray, OTWpntArray, ObstacleArray, Obstacle
import random
from sensor_msgs.msg import LaserScan
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
        

        # Get sim parameter from global_parameters node to automatically switch profiles
        self.global_param_client = self.create_client(GetParameters, '/global_parameters/get_parameters')
        self.global_param_client.wait_for_service()
        req = GetParameters.Request()
        req.names = ['sim']
        future = self.global_param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.sim = False
        if future.result() is not None:
            self.sim = future.result().values[0].bool_value
        self.get_logger().info(f"Is simulation: {self.sim}")

        # Automate test_on_car alignment
        self.params.test_on_car = not self.sim

        # update on parameter changes for rate
        self.add_on_set_parameters_callback(self.params.parameters_callback)
        
        # SUBSCRIPTIONS
        if self.params.test_on_car:
            self.battery_sub = self.create_subscription(VescStateStamped, "/vesc/sensors/core", self.battery_cb, 10)
        else:
            self.battery_level = self.params.volt_threshold + 1
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

        # INITIALIZATIONS        
        self.waypoints_dist = 0.1
        
        self.state = StateType(self.params.initial_state)
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
    def battery_cb(self, msg: VescStateStamped):
        self.battery_level = msg.state.voltage_input

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
        self.glb_wpnts = data.wpnts[:-1]  # exclude last point (because last point == first point)
        self.num_glb_wpnts = len(self.glb_wpnts)
        self.track_length = data.wpnts[-1].s_m
        # Get spacing between wpnts for rough approximations
        self.wpnt_dist = data.wpnts[1].s_m - data.wpnts[0].s_m
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

        # Lap counting: detect wrap-around when cur_s jumps backwards past the
        # start/finish line.  A drop of more than half the track length is
        # treated as a lap completion regardless of absolute position.
        if self._prev_s is not None and self.track_length > 0:
            drop = self._prev_s - self.cur_s
            if drop > self.track_length * 0.5:
                self.lap_count += 1
                self.get_logger().info(
                    f"[qualifying] Lap {self.lap_count} completed (s: {self._prev_s:.2f} "
                    f"-> {self.cur_s:.2f}, track_length: {self.track_length:.2f})"
                )
        self._prev_s = self.cur_s

    def avoidance_cb(self, data: OTWpntArray):
        """Subscribes to spliner waypoints"""
        if len(data.wpnts) > 0:
            self.splini_ttl_counter = int(self.params.splini_ttl * self.params.rate_hz)
            self.avoidance_wpnts = data
        else:
            # If the spliner explicitly publishes empty waypoints, we clear our cache immediately
            # to allow fast transition back to TRAILING or GB_TRACK
            self.splini_ttl_counter = 0
            self.avoidance_wpnts = data
            self.last_valid_avoidance_wpnts = None

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
        """Evaluates if car has crashed or is stuck against a wall."""
        if not self.params.enable_recovery:
            return False

        front_dist, _ = self._evaluate_scan_clearance()
        is_slow = abs(self.cur_vs) < self.params.stuck_speed_threshold
        is_front_blocked = front_dist < self.params.crash_dist_threshold

        if is_front_blocked or (is_slow and front_dist < 0.4):
            self.stuck_counter += 1
        else:
            self.stuck_counter = max(0, self.stuck_counter - 1)

        threshold_ticks = int(self.params.stuck_time_threshold * self.params.rate_hz)
        return self.stuck_counter >= threshold_ticks

    def _evaluate_scan_clearance(self) -> tuple[float, float]:
        """Calculates front clearance and rear clearance from LiDAR scans."""
        if self.latest_scan is None or len(self.latest_scan.ranges) == 0:
            return 1.0, 1.0

        ranges = np.array(self.latest_scan.ranges)
        ranges = np.nan_to_num(ranges, nan=10.0, posinf=10.0, neginf=0.0)

        n = len(ranges)
        # Front cone: ~[-20 deg, +20 deg] around index n//2
        front_window = int(n * (40.0 / 360.0))
        front_start = max(0, n // 2 - front_window // 2)
        front_end = min(n, n // 2 + front_window // 2)
        front_dist = float(np.min(ranges[front_start:front_end])) if front_end > front_start else 1.0

        # Rear cone: first 10% and last 10% of array (assuming scan starts near rear -180 deg)
        rear_window = int(n * (30.0 / 360.0))
        rear_ranges = np.concatenate((ranges[:rear_window], ranges[-rear_window:]))
        rear_dist = float(np.min(rear_ranges)) if len(rear_ranges) > 0 else 1.0

        return front_dist, rear_dist

    def _evaluate_recovery_steer_dir(self):
        """Compares left vs right side LiDAR clearance to choose reverse steering direction."""
        if self.latest_scan is None or len(self.latest_scan.ranges) == 0:
            self.recovery_steer_dir = 1.0
            return

        ranges = np.array(self.latest_scan.ranges)
        ranges = np.nan_to_num(ranges, nan=10.0, posinf=10.0, neginf=0.0)
        n = len(ranges)

        # Left arc (25% to 45% of scan points) vs Right arc (55% to 75% of scan points)
        left_arc = ranges[int(n * 0.25):int(n * 0.45)]
        right_arc = ranges[int(n * 0.55):int(n * 0.75)]

        mean_left = float(np.mean(left_arc)) if len(left_arc) > 0 else 1.0
        mean_right = float(np.mean(right_arc)) if len(right_arc) > 0 else 1.0

        # If left has more clearance, steer right when reversing (swings front left)
        if mean_left >= mean_right:
            self.recovery_steer_dir = -1.0  # right steer while reversing
        else:
            self.recovery_steer_dir = 1.0   # left steer while reversing

        self.get_logger().info(f"[RECOVERY] Side clearance - Left: {mean_left:.2f}m, Right: {mean_right:.2f}m -> Reverse steer dir: {self.recovery_steer_dir}")

    ### Properties to evaulate state transitions
    @property
    def _low_bat(self)->bool:

        if self.battery_level < self.params.volt_threshold:
            return True
        else:
            return False
    
    @property
    def _check_only_ftg_zone(self) -> bool:
        ftg_only = False
        # check if the car is in a ftg only zone, but only if there is an only ftg zone
        if len(self.only_ftg_zones) != 0:
            for sector in self.only_ftg_zones:
                if sector[0] <= self.cur_s / self.waypoints_dist <= sector[1]:
                    ftg_only = True
                    # rospy.logwarn(f"[{self.name}] IN FTG ONLY ZONE")
                    break  # cannot be in two ftg zones
        return ftg_only

    @property
    def _check_close_to_raceline(self) -> bool:
        return np.abs(self.cur_d) < self.params.gb_ego_width_m  # [m]

    @property
    def _check_ot_sector(self) -> bool:
        for sector in self.ot_sectors:
            if sector['ot_flag']:
                if (sector['start'] <= self.cur_s / self.waypoints_dist <= (sector['end']+1)):
                    return True
        return False

    @property
    def _check_ofree(self) -> bool:
        o_free = True

        if self.params.overtake_mode == "spliner":
            if self.last_valid_avoidance_wpnts is not None:
                horizon = self.params.overtaking_horizon_m  # Horizon in front of cur_s [m]

                for obs in self.obstacles:
                    obs_s = obs.s_center
                    # Wrapping madness to check if infront
                    dist_to_obj = (obs_s - self.cur_s) % self.track_length
                    if dist_to_obj < horizon and len(self.last_valid_avoidance_wpnts):
                        obs_d = obs.d_center
                        # Get d wrt to mincurv from the overtaking line
                        avoid_wpnt_idx = np.argmin(
                            np.array([abs(avoid_s.s_m - obs_s) for avoid_s in self.last_valid_avoidance_wpnts])
                        )
                        ot_d = self.last_valid_avoidance_wpnts[avoid_wpnt_idx].d_m
                        ot_obs_dist = ot_d - obs_d
                        if abs(ot_obs_dist) < self.params.lateral_width_ot_m:
                            o_free = False
                            self.get_logger().info(f"O_FREE False, obs dist to ot lane: {ot_obs_dist} m")
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
    # HELPERS #
    ###########
    def get_splini_wpts(self) -> WpntArray:
        """Obtain the waypoints by fusing those obtained by spliner with the
        global ones.
        """
        splini_glob = self.glb_wpnts.copy()

        # Handle wrapping
        if self.last_valid_avoidance_wpnts is not None:
            if len(self.last_valid_avoidance_wpnts) == 0:
                splini_idxs = [] # Avoid index error if empty
            elif self.last_valid_avoidance_wpnts[-1].s_m > self.last_valid_avoidance_wpnts[0].s_m:
                start_s = int(self.last_valid_avoidance_wpnts[0].s_m / self.waypoints_dist + 0.5)
                end_s = int(self.last_valid_avoidance_wpnts[-1].s_m / self.waypoints_dist + 0.5)
                splini_idxs = [s % self.num_glb_wpnts for s in range(start_s, end_s)]
            else:
                start_s = int(self.last_valid_avoidance_wpnts[0].s_m / self.waypoints_dist + 0.5)
                end_s = int((self.track_length + self.last_valid_avoidance_wpnts[-1].s_m) / self.waypoints_dist + 0.5)
                splini_idxs = [s % self.num_glb_wpnts for s in range(start_s, end_s)]

            # with self.lock:  # was needed in ROS1 but hopefuly in ROS2 were fine
            for i, s in enumerate(splini_idxs):
                # splini_glob[s] = self.last_valid_avoidance_wpnts[i]
                splini_glob[s] = self.last_valid_avoidance_wpnts[min(i, len(self.last_valid_avoidance_wpnts) - 1)]

        # If the last valid points have been reset, then we just pass the global waypoints
        else:
            self.get_logger().warn(f"No valid avoidance waypoints, passing global waypoints")
            pass

        return splini_glob

    def init_ot_params(self):
        """Obtain the initial overtaking parameters from the parameter server.
        Then instantiate a parameter event handler to get the updates on those parameters.
        """
        self.parameter_client = self.create_client(GetParameters, '/ot_interpolator/get_parameters')
        self.parameter_client.wait_for_service()
        self.n_ot_sectors = None
        self.ot_param_names = None
        self.ot_sectors = None
        
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
            if self.max_speed and self.max_speed > 0:
                mrk.pose.position.z = wpnt.vx_mps / self.max_speed  # Visualise speed in z dimension
            else:
                mrk.pose.position.z = 0.0
            mrk.pose.orientation.w = 1.0
            loc_markers.markers.append(mrk)

        # ...

        if len(loc_wpnts.wpnts) == 0:
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
            
        if self.params.force_state:
            self.state = self.params.force_state_choice
        else:
            self.state = self.state_transition(self)

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
