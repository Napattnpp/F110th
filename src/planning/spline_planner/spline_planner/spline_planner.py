import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor, SetParametersResult, ParameterType
from rclpy.parameter import Parameter

from f110_msgs.msg import Obstacle, ObstacleArray, OTWpntArray, Wpnt, WpntArray
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np
import time
from typing import List, Any, Tuple
from scipy.interpolate import InterpolatedUnivariateSpline as Spline
from frenet_conversion.frenet_converter import FrenetConverter

class ObstacleSpliner(Node):
    """
    This class implements a ROS node that performs splining around obstacles.

    It subscribes to the following topics:
        - `/perception/obstacles`: Subscribes to the obstacle array.
        - `/car_state/odom_frenet`: Subscribes to the car state in Frenet coordinates.
        - `/global_waypoints`: Subscribes to global waypoints.
        - `/global_waypoints_scaled`: Subscribes to the scaled global waypoints.

    The node publishes the following topics:
        - `/planner/avoidance/markers`: Publishes spline markers.
        - `/planner/avoidance/otwpnts`: Publishes splined waypoints.
        - `/planner/avoidance/considered_OBS`: Publishes markers for the closest obstacle.
        - `/planner/avoidance/propagated_obs`: Publishes markers for the propagated obstacle.
        - `/planner/avoidance/latency`: Publishes the latency of the spliner node. (only if measuring is enabled)
    """

    def __init__(self):
        super().__init__('spliner_node')

        # initialize the instance variable
        self.obs = ObstacleArray()
        self.gb_wpnts = None
        self.gb_scaled_wpnts = None
        self.gb_vmax = None
        self.gb_max_idx = None
        self.gb_max_s = None
        self.cur_s = None
        self.cur_d = None
        self.cur_vs = None
        self.lookahead_min = 4.0
        self.lookahead_max = 12.0
        self.lookahead_kv = 1.0
        self.lookahead_kkappa = 0.5
        self.lookahead = self.lookahead_min
        self.last_switch_time = self.get_clock().now().to_msg()
        self.last_ot_side = ""

        # Initialize instance variables
        self.declare_parameters(
            namespace='',
            parameters=[
                ('from_bag', False),
                ('measure', False),
            ])
        self.from_bag = self.get_parameter(
            'from_bag').get_parameter_value().bool_value
        self.measuring = self.get_parameter(
            'measure').get_parameter_value().bool_value

        # Setup subscriptions
        self.create_subscription(
            ObstacleArray, '/perception/obstacles', self.obs_cb, QoSProfile(depth=10))
        self.create_subscription(
            Odometry, '/car_state/frenet/odom', self.state_cb, QoSProfile(depth=10))
        self.create_subscription(
            WpntArray, '/global_waypoints', self.gb_cb, QoSProfile(depth=10))
        self.create_subscription(
            WpntArray, '/global_waypoints_scaled', self.gb_scaled_cb, QoSProfile(depth=10))

        # Setup publishers
        self.mrks_pub = self.create_publisher(
            MarkerArray, '/planner/avoidance/markers', QoSProfile(depth=10))
        self.evasion_pub = self.create_publisher(
            OTWpntArray, '/planner/avoidance/otwpnts', QoSProfile(depth=10))
        self.closest_obs_pub = self.create_publisher(
            Marker, '/planner/avoidance/considered_OBS', QoSProfile(depth=10))
        self.pub_propagated = self.create_publisher(
            Marker, '/planner/avoidance/propagated_obs', QoSProfile(depth=10))

        # dyn params sub
        self.pre_apex_0 = -4.0
        self.pre_apex_1 = -3.0
        self.pre_apex_2 = -1.5
        self.post_apex_0 = 2.0
        self.post_apex_1 = 3.0
        self.post_apex_2 = 4.0
        self.evasion_dist = 0.55
        self.obs_traj_tresh = 0.55
        self.spline_bound_mindist = 0.18
        self.fixed_pred_time = 0.15
        self.kd_obs_pred = 1.0
        self.ot_side_switch_cur_d_thresh = 0.60
        self.spline_max_alat = 7.0
        self.spline_abrake_max = 4.5
        self.spline_aaccel_max = 3.5
        self.prediction_mode = "curvature_aware"
        self.multi_obs_safety_margin = 0.25
        
        pd = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range=[FloatingPointRange(from_value=0.1, to_value=8.0, step=0.001)]
        )
        pd_la = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=25.0, step=0.01)]
        )
        pd_acc = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range=[FloatingPointRange(from_value=0.5, to_value=25.0, step=0.01)]
        )
        pd_str = ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        param_dicts = [
            {'name': 'pre_apex_0', 'default': abs(self.pre_apex_0), 'descriptor': pd},
            {'name': 'pre_apex_1', 'default': abs(self.pre_apex_1), 'descriptor': pd},
            {'name': 'pre_apex_2', 'default': abs(self.pre_apex_2), 'descriptor': pd},
            {'name': 'post_apex_0', 'default': self.post_apex_0, 'descriptor': pd},
            {'name': 'post_apex_1', 'default': self.post_apex_1, 'descriptor': pd},
            {'name': 'post_apex_2', 'default': self.post_apex_2, 'descriptor': pd},
            {'name': 'evasion_dist', 'default': self.evasion_dist, 'descriptor': pd},
            {'name': 'obs_traj_tresh', 'default': self.obs_traj_tresh, 'descriptor': pd},
            {'name': 'spline_bound_mindist', 'default': self.spline_bound_mindist, 'descriptor': pd},
            {'name': 'fixed_pred_time', 'default': self.fixed_pred_time, 'descriptor': pd},
            {'name': 'kd_obs_pred', 'default': self.kd_obs_pred, 'descriptor': pd},
            {'name': 'ot_side_switch_cur_d_thresh', 'default': self.ot_side_switch_cur_d_thresh, 'descriptor': pd},
            {'name': 'lookahead_min', 'default': self.lookahead_min, 'descriptor': pd_la},
            {'name': 'lookahead_max', 'default': self.lookahead_max, 'descriptor': pd_la},
            {'name': 'lookahead_kv', 'default': self.lookahead_kv, 'descriptor': pd_la},
            {'name': 'lookahead_kkappa', 'default': self.lookahead_kkappa, 'descriptor': pd_la},
            {'name': 'spline_max_alat', 'default': self.spline_max_alat, 'descriptor': pd_acc},
            {'name': 'spline_abrake_max', 'default': self.spline_abrake_max, 'descriptor': pd_acc},
            {'name': 'spline_aaccel_max', 'default': self.spline_aaccel_max, 'descriptor': pd_acc},
            {'name': 'prediction_mode', 'default': self.prediction_mode, 'descriptor': pd_str},
            {'name': 'multi_obs_safety_margin', 'default': self.multi_obs_safety_margin, 'descriptor': pd}
        ]

        self.declare_all_parameters(param_dicts=param_dicts)
        self.add_on_set_parameters_callback(self.dyn_param_cb)

        if self.measuring:
            self.latency_pub = self.create_publisher(Float32, '/planner/avoidance/latency', QoSProfile(depth=10))

        # Wait for messages
        self.wait_for_messages()

        # Initialize converter, if required
        self.converter = self.initialize_converter()

        # Create timer
        self.create_timer(1/20, self.spliner_loop)

    #################### DYNAMIC PARAMS####################
    def declare_all_parameters(self, param_dicts: List[dict]):
        params = []
        for param_dict in param_dicts:
            param = self.declare_parameter(
                param_dict['name'], param_dict['default'], param_dict['descriptor'])
            params.append(param)
            pname = param_dict['name']
            if pname.startswith('pre_apex_'):
                setattr(self, pname, -1 * abs(param.value))
            else:
                setattr(self, pname, param.value)
        return params

    def dyn_param_cb(self, params: List[Parameter]):
        """
        Notices the change in the parameters and alters the spline params accordingly
        """
        for param in params:
            param_name = param.name
            if param_name == 'pre_apex_0':
                self.pre_apex_0 = -1 * param.value
            elif param_name == 'pre_apex_1':
                self.pre_apex_1 = -1 * param.value
            elif param_name == 'pre_apex_2':
                self.pre_apex_2 = -1 * param.value
            elif param_name == 'post_apex_0':
                self.post_apex_0 = param.value
            elif param_name == 'post_apex_1':
                self.post_apex_1 = param.value
            elif param_name == 'post_apex_2':
                self.post_apex_2 = param.value
            elif param_name == 'evasion_dist':
                self.evasion_dist = param.value
            elif param_name == 'obs_traj_tresh':
                self.obs_traj_tresh = param.value
            elif param_name == 'spline_bound_mindist':
                self.spline_bound_mindist = param.value
            elif param_name == 'fixed_pred_time':
                self.fixed_pred_time = param.value
            elif param_name == 'kd_obs_pred':
                self.kd_obs_pred = param.value
            elif param_name == 'ot_side_switch_cur_d_thresh':
                self.ot_side_switch_cur_d_thresh = param.value
            elif param_name == 'lookahead_min':
                self.lookahead_min = param.value
            elif param_name == 'lookahead_max':
                self.lookahead_max = param.value
            elif param_name == 'lookahead_kv':
                self.lookahead_kv = param.value
            elif param_name == 'lookahead_kkappa':
                self.lookahead_kkappa = param.value
            elif param_name == 'spline_max_alat':
                self.spline_max_alat = param.value
            elif param_name == 'spline_abrake_max':
                self.spline_abrake_max = param.value
            elif param_name == 'spline_aaccel_max':
                self.spline_aaccel_max = param.value
            elif param_name == 'prediction_mode':
                self.prediction_mode = param.value
            elif param_name == 'multi_obs_safety_margin':
                self.multi_obs_safety_margin = param.value
        
        # Ensure ascending order for spline parameters
        if self.pre_apex_1 < self.pre_apex_0:
            self.get_logger().info("Adjusting pre_apex_1 to ensure ascending order.")
            self.pre_apex_1 = self.pre_apex_0

        if self.pre_apex_2 < self.pre_apex_1:
            self.get_logger().info("Adjusting pre_apex_2 to ensure ascending order.")
            self.pre_apex_2 = self.pre_apex_1

        if self.post_apex_0 < self.pre_apex_2:
            self.get_logger().info("Adjusting post_apex_0 to ensure ascending order.")
            self.post_apex_0 = self.pre_apex_2

        if self.post_apex_1 < self.post_apex_0:
            self.get_logger().info("Adjusting post_apex_1 to ensure ascending order.")
            self.post_apex_1 = self.post_apex_0

        if self.post_apex_2 < self.post_apex_1:
            self.get_logger().info("Adjusting post_apex_2 to ensure ascending order.")
            self.post_apex_2 = self.post_apex_1
        
        spline_params = [
            self.pre_apex_0,
            self.pre_apex_1,
            self.pre_apex_2,
            0,
            self.post_apex_0,
            self.post_apex_1,
            self.post_apex_2,
        ]
        
        self.get_logger().info(
            f" Dynamic reconf triggered new spline params: {spline_params} [m],\n"
            f" evasion apex distance: {self.evasion_dist} [m],\n"
            f" obstacle trajectory treshold: {self.obs_traj_tresh} [m]\n"
            f" obstacle prediciton k_d: {self.kd_obs_pred},    obstacle prediciton constant time: {self.fixed_pred_time} [s] "
        )
        return SetParametersResult(successful=True)

    #################### CALLBACKS####################
    def obs_cb(self, msg: ObstacleArray):
        # Callback function for obstacle data
        self.obs = msg

    def state_cb(self, msg: Odometry):
        # Callback function for car state data
        self.cur_s = msg.pose.pose.position.x
        self.cur_d = msg.pose.pose.position.y
        self.cur_vs = msg.twist.twist.linear.x

    def gb_cb(self, msg: WpntArray):
        # Callback function for global waypoints
        self.gb_wpnts = msg
        if self.gb_vmax is None:
            self.gb_vmax = np.max(
                np.array([wpnt.vx_mps for wpnt in msg.wpnts]))
            self.gb_max_idx = msg.wpnts[-1].id if msg.wpnts[-1].id > 0 else len(msg.wpnts)
            self.gb_max_s = msg.wpnts[-1].s_m

    def gb_scaled_cb(self, msg: WpntArray):
        # Callback function for scaled global waypoints
        self.gb_scaled_wpnts = msg

    #################### HELPER FUNCTIONS####################
    def wait_for_messages(self):
        self.get_logger().info('Carstate Node waiting for Odometry messages...')
        state_print = False
        gb_print = False
        scaled_gb_print = False
        waitlist = [self.cur_s, self.gb_wpnts, self.gb_scaled_wpnts]
        while None in waitlist:
            rclpy.spin_once(self)
            if self.cur_s is not None and not state_print:
                self.get_logger().info('Received State message.')
                state_print = True
            if self.gb_wpnts is not None and not gb_print:
                self.get_logger().info('Received Global Waypoints message.')
                gb_print = True
            if self.gb_scaled_wpnts is not None and not scaled_gb_print:
                self.get_logger().info('Received Scaled Global Waypoints message.')
                scaled_gb_print = True
            waitlist = [self.cur_s, self.gb_wpnts, self.gb_scaled_wpnts]
        self.get_logger().info('All required messages received. Continuing...')

    def initialize_converter(self):
        # Initialize any converters or additional components
        waypoint_array = self.gb_wpnts.wpnts
        waypoints_x = [waypoint.x_m for waypoint in waypoint_array]
        waypoints_y = [waypoint.y_m for waypoint in waypoint_array]
        waypoints_psi = [waypoint.psi_rad for waypoint in waypoint_array]
        return FrenetConverter(np.array(waypoints_x), np.array(waypoints_y), np.array(waypoints_psi))

    #################### UTILS FUNCTIONS####################
    def _predict_obs_movement(self, obs: Obstacle, mode: str = None) -> Obstacle:
        """
        Predicts the movement of an obstacle based on the current state and mode.
        Modes:
          - 'curvature_aware' (Default): Propagates opponent with curvature-bounded speed
            and apex-reverting lateral dynamics, maintaining strict track boundary containment.
          - 'constant': Propagate opponent by constant time (linear Frenet extrapolation).
          - 'adaptive': Distance-to-opponent relative speed scaling.
        """
        if mode is None:
            mode = getattr(self, 'prediction_mode', 'curvature_aware')

        if self.cur_s is None or self.gb_max_s is None:
            return obs

        # propagate opponent by time dependent on distance
        dist_to_car = (obs.s_center - self.cur_s) % self.gb_max_s
        if dist_to_car < 12.0:
            pred_time = float(self.fixed_pred_time)

            if mode == "curvature_aware":
                # 1. Query local raceline curvature at opponent's location
                kappa = 0.0
                wpnts_list = getattr(self.gb_wpnts, 'wpnts', self.gb_wpnts)
                if wpnts_list is not None and len(wpnts_list) > 0 and self.gb_max_s is not None:
                    s_diffs = [abs((w.s_m - obs.s_center) % self.gb_max_s) for w in wpnts_list]
                    closest_idx = int(np.argmin(s_diffs))
                    w_opp = wpnts_list[closest_idx]
                    if hasattr(w_opp, 'kappa_radpm'):
                        kappa = abs(float(w_opp.kappa_radpm))

                # 2. Longitudinal velocity bounded by cornering adhesion limit
                v_corner_limit = np.sqrt(self.spline_max_alat / max(kappa, 0.05))
                v_s_eff = min(max(float(obs.vs), 0.0), v_corner_limit)
                delta_s = pred_time * v_s_eff

                # 3. Lateral dynamics: natural stabilization towards raceline in corners
                decay_factor = float(np.exp(-2.0 * kappa))
                delta_d = (obs.vd * pred_time) * decay_factor - 0.25 * obs.d_center * (1.0 - decay_factor)

            elif mode == "adaptive":
                # distance in s coordinate
                cur_s = self.cur_s
                ot_distance = (obs.s_center - cur_s) % self.gb_max_s
                cur_idx = int(cur_s * 10) % len(self.gb_scaled_wpnts.wpnts)
                rel_speed = np.clip(self.gb_scaled_wpnts.wpnts[cur_idx].vx_mps - obs.vs, 0.1, 10)
                ot_time_distance = np.clip(ot_distance / rel_speed, 0, 5) * 0.5

                delta_s = ot_time_distance * obs.vs
                delta_d = ot_time_distance * obs.vd
                delta_d = -(obs.d_center + delta_d) * \
                    np.exp(-np.abs(self.kd_obs_pred * obs.d_center))

            elif mode == "adaptive_velheuristic":
                opponent_scaler = 0.7
                cur_s = self.cur_s
                cur_idx = int(cur_s * 10) % len(self.gb_scaled_wpnts.wpnts)
                ego_speed = self.gb_scaled_wpnts.wpnts[cur_idx].vx_mps

                # distance in s coordinate
                ot_distance = (obs.s_center - cur_s) % self.gb_max_s
                rel_speed = (1 - opponent_scaler) * ego_speed
                ot_time_distance = np.clip(ot_distance / rel_speed, 0, 5)

                delta_s = ot_time_distance * opponent_scaler * ego_speed
                delta_d = -(obs.d_center) * \
                    np.exp(-np.abs(self.kd_obs_pred * obs.d_center))

            # propagate opponent by constant time
            elif mode == "constant":
                delta_s = self.fixed_pred_time * obs.vs
                delta_d = self.fixed_pred_time * obs.vd

            elif mode == "heuristic":
                # distance in s coordinate
                ot_distance = (obs.s_center - self.cur_s) % self.gb_max_s
                rel_speed = 3
                ot_time_distance = ot_distance / rel_speed

                delta_s = ot_time_distance * obs.vs
                delta_d = ot_time_distance * obs.vd
                delta_d = -(obs.d_center + delta_d) * \
                    np.exp(-np.abs(self.kd_obs_pred * obs.d_center))

            else:
                delta_s = self.fixed_pred_time * obs.vs
                delta_d = self.fixed_pred_time * obs.vd

            # update
            obs.s_start += delta_s
            obs.s_center += delta_s
            obs.s_end += delta_s
            obs.s_start %= self.gb_max_s
            obs.s_center %= self.gb_max_s
            obs.s_end %= self.gb_max_s

            obs.d_left += delta_d
            obs.d_center += delta_d
            obs.d_right += delta_d

            # Soft-bound predicted lateral extents inside physical track bounds
            wpnts_list = getattr(self.gb_wpnts, 'wpnts', self.gb_wpnts)
            if wpnts_list is not None and len(wpnts_list) > 0 and self.gb_max_s is not None:
                s_diffs = [abs((w.s_m - obs.s_center) % self.gb_max_s) for w in wpnts_list]
                w_pred = wpnts_list[int(np.argmin(s_diffs))]
                obs_rad = max(0.15, abs(obs.d_left - obs.d_right) / 2.0)
                d_max_allowed = w_pred.d_left - obs_rad
                d_min_allowed = w_pred.d_right + obs_rad
                obs.d_center = float(np.clip(obs.d_center, d_min_allowed, d_max_allowed))
                obs.d_left = float(np.clip(obs.d_left, d_min_allowed, d_max_allowed))
                obs.d_right = float(np.clip(obs.d_right, d_min_allowed, d_max_allowed))

            resp = self.converter.get_cartesian([obs.s_center], [obs.d_center])

            marker = self.xy_to_point(resp[0], resp[1], opponent=True)
            self.pub_propagated.publish(marker)

        return obs

    def _check_ot_side_possible(self, more_space) -> bool:
        # Initial side choice (first detection) is always allowed
        if self.last_ot_side == "":
            return True
        if abs(self.cur_d) > self.ot_side_switch_cur_d_thresh and more_space != self.last_ot_side:
            self.get_logger().info(
                f"Can't switch sides ({more_space} vs last {self.last_ot_side}), because cur_d ({self.cur_d:.2f}) > threshold ({self.ot_side_switch_cur_d_thresh:.2f})"
            )
            return False
        return True

    def _more_space(self, obstacle: Obstacle, gb_wpnts: List[Any], gb_idxs: List[int]) -> Tuple[str, float]:
        left_gap = abs(gb_wpnts[gb_idxs[0]].d_left - obstacle.d_left)
        right_gap = abs(gb_wpnts[gb_idxs[0]].d_right + obstacle.d_right)
        min_space = self.evasion_dist + self.spline_bound_mindist

        if right_gap > min_space and left_gap < min_space:
            # Compute apex distance to the right of the opponent
            d_apex_right = obstacle.d_right - self.evasion_dist
            # If we overtake to the right of the opponent BUT the apex is to the left of the raceline, then we set the apex to 0
            if d_apex_right > 0:
                d_apex_right = 0
            return "right", d_apex_right

        elif left_gap > min_space and right_gap < min_space:
            # Compute apex distance to the left of the opponent
            d_apex_left = obstacle.d_left + self.evasion_dist
            # If we overtake to the left of the opponent BUT the apex is to the right of the raceline, then we set the apex to 0
            if d_apex_left < 0:
                d_apex_left = 0
            return "left", d_apex_left
        else:
            candidate_d_apex_left = obstacle.d_left + self.evasion_dist
            candidate_d_apex_right = obstacle.d_right - self.evasion_dist

            if abs(candidate_d_apex_left) <= abs(candidate_d_apex_right):
                # If we overtake to the left of the opponent BUT the apex is to the right of the raceline, then we set the apex to 0
                if candidate_d_apex_left < 0:
                    candidate_d_apex_left = 0
                return "left", candidate_d_apex_left
            else:
                # If we overtake to the right of the opponent BUT the apex is to the left of the raceline, then we set the apex to 0
                if candidate_d_apex_right > 0:
                    candidate_d_apex_right = 0
                return "right", candidate_d_apex_right

    #################### MAIN FUNCTIONS####################
    def spliner_loop(self):
        # Main loop for the spliner node
        if self.measuring:
            start = time.perf_counter()

        # Sample data
        obs = self.obs
        gb_scaled_wpnts = self.gb_scaled_wpnts.wpnts
        wpnts = OTWpntArray()
        mrks = MarkerArray()

        # If obs then do splining around it
        if len(obs.obstacles) > 0:
            wpnts, mrks = self.do_spline(
                obstacles=obs, gb_wpnts=gb_scaled_wpnts)
        # Else delete spline markers
        else:
            self.last_ot_side = ""
            del_mrk = Marker()
            del_mrk.header.stamp = self.get_clock().now().to_msg()
            del_mrk.action = Marker.DELETEALL
            mrks.markers.append(del_mrk)

        # Publish wpnts and markers
        if self.measuring:
            end = time.perf_counter()
            self.latency_pub.publish(end - start)
        self.evasion_pub.publish(wpnts)
        self.mrks_pub.publish(mrks)

    def do_spline(self, obstacles: ObstacleArray, gb_wpnts: WpntArray) -> Tuple[WpntArray, MarkerArray]:
        """
        Creates an evasion trajectory for the closest obstacle by splining between pre- and post-apex points.
        Features:
          1. Full-Horizon Envelope Feasibility: Verifies boundary clearances across the entire spline corridor.
          2. Multi-Obstacle Collision Verification: Validates clearance against all active tracked obstacles.
          3. Curvature-Constrained Dynamic Velocity: Re-profiles velocities dynamically adhering to tire grip limits.
        """
        mrks = MarkerArray()
        wpnts = OTWpntArray()

        # Get spacing between wpnts for rough approximations
        wpnt_dist = gb_wpnts[1].s_m - gb_wpnts[0].s_m if len(gb_wpnts) > 1 else 0.1
        if wpnt_dist <= 0:
            wpnt_dist = (self.gb_max_s / self.gb_max_idx) if (self.gb_max_idx and self.gb_max_idx > 0) else 0.1

        # Only use obstacles that are within a threshold of the raceline
        close_obs = self._obs_filtering(obstacles=obstacles)

        if len(close_obs) > 0:
            # 1. Primary obstacle: closest in front
            closest_obs = min(
                close_obs, key=lambda obs: (obs.s_center - self.cur_s) % self.gb_max_s
            )

            # 2. Compute evasion apex along s
            if closest_obs.s_end < closest_obs.s_start:
                s_apex = (closest_obs.s_end + self.gb_max_s + closest_obs.s_start) / 2.0
            else:
                s_apex = (closest_obs.s_end + closest_obs.s_start) / 2.0

            # Approximate next 20 indexes of global wpnts to identify corner outside
            gb_idxs = [int(s_apex / wpnt_dist + i) % self.gb_max_idx for i in range(20)]
            kappas = np.array([gb_wpnts[gb_idx].kappa_radpm for gb_idx in gb_idxs])
            outside = "left" if np.sum(kappas) < 0 else "right"

            # Preferred evasion side based on immediate obstacle gap
            more_space, preferred_d_apex = self._more_space(closest_obs, gb_wpnts, gb_idxs)

            # Determine candidate side order respecting side commitment hysteresis
            candidate_sides = []
            if not self._check_ot_side_possible(more_space):
                if self.last_ot_side != "":
                    candidate_sides = [self.last_ot_side]
            else:
                opposite_side = "right" if more_space == "left" else "left"
                candidate_sides = [more_space, opposite_side]

            # Publish visualization marker for the apex reference point
            mrk = self.xy_to_point(
                x=gb_wpnts[gb_idxs[0]].x_m, y=gb_wpnts[gb_idxs[0]].y_m, opponent=False
            )
            self.closest_obs_pub.publish(mrk)

            # 3. Evaluate candidate sides for Full-Horizon and Multi-Obstacle Feasibility
            selected_spline = None
            spline_res = 0.1

            for cand_side in candidate_sides:
                # Compute apex distance for this candidate side
                if cand_side == "right":
                    cand_d_apex = closest_obs.d_right - self.evasion_dist
                    if cand_d_apex > 0:
                        cand_d_apex = 0.0
                else:
                    cand_d_apex = closest_obs.d_left + self.evasion_dist
                    if cand_d_apex < 0:
                        cand_d_apex = 0.0

                # Form 7 control points along the evasion trajectory
                evasion_points = []
                spline_params = [
                    self.pre_apex_0,
                    self.pre_apex_1,
                    self.pre_apex_2,
                    0.0,
                    self.post_apex_0,
                    self.post_apex_1,
                    self.post_apex_2,
                ]
                for dst in spline_params:
                    scaled_dst = dst * np.clip(1.0 + self.cur_vs / self.gb_vmax, 1.0, 1.5)
                    if outside == cand_side:
                        si = s_apex + scaled_dst * 1.75
                    else:
                        si = s_apex + scaled_dst
                    di = cand_d_apex if dst == 0.0 else 0.0
                    evasion_points.append([si, di])
                evasion_points = np.array(evasion_points)

                # Fit smooth cubic spline
                spatial_spline = Spline(x=evasion_points[:, 0], y=evasion_points[:, 1])
                s_cand = np.arange(evasion_points[0, 0], evasion_points[-1, 0], spline_res)
                if cand_d_apex < 0:
                    d_cand = np.clip(spatial_spline(s_cand), cand_d_apex, 0.0)
                else:
                    d_cand = np.clip(spatial_spline(s_cand), 0.0, cand_d_apex)
                s_cand_wrapped = s_cand % self.gb_max_s

                # Feasibility Check A (Priority 2): Full-Horizon Track Boundary Clearance
                spatially_feasible = True
                for i in range(len(s_cand_wrapped)):
                    idx = int((s_cand_wrapped[i] / wpnt_dist) % self.gb_max_idx)
                    w = gb_wpnts[idx]
                    di = d_cand[i]
                    if di > 0:
                        clearance = w.d_left - di
                    else:
                        clearance = abs(w.d_right) - abs(di)
                    if clearance < self.spline_bound_mindist:
                        spatially_feasible = False
                        break

                if not spatially_feasible:
                    continue  # Corridor too narrow along this side, try alternative

                # Feasibility Check B (Priority 3): Multi-Obstacle Collision Verification
                collision_free = True
                half_track = self.gb_max_s / 2.0
                for obs_sec in close_obs:
                    if obs_sec is closest_obs:
                        continue
                    obs_len = max(0.30, abs(getattr(obs_sec, 's_end', obs_sec.s_center) - getattr(obs_sec, 's_start', obs_sec.s_center)))
                    for i in range(len(s_cand_wrapped)):
                        dist_s = abs((s_cand_wrapped[i] - obs_sec.s_center + half_track) % self.gb_max_s - half_track)
                        if dist_s < (obs_len / 2.0 + 0.35):
                            if d_cand[i] > obs_sec.d_center:
                                lat_gap = d_cand[i] - getattr(obs_sec, 'd_left', obs_sec.d_center)
                            else:
                                lat_gap = getattr(obs_sec, 'd_right', obs_sec.d_center) - d_cand[i]
                            if lat_gap < self.multi_obs_safety_margin:
                                collision_free = False
                                break
                    if not collision_free:
                        break

                if not collision_free:
                    continue  # Collides with secondary obstacle, try alternative

                # Both checks passed!
                selected_spline = (cand_side, s_cand_wrapped, d_cand)
                break

            # 4. Handle Infeasible Splines (Clean Rejection -> Safe Trailing)
            danger_flag = False
            if selected_spline is None:
                danger_flag = True
                wpnts.wpnts = []
                mrks.markers = []
                wpnts.side_switch = True
                self.last_switch_time = self.get_clock().now().to_msg()
                return wpnts, mrks

            chosen_side, evasion_s, evasion_d = selected_spline

            # 5. Convert to Cartesian Coordinates
            resp = self.converter.get_cartesian(evasion_s, evasion_d)
            x_coords = resp[0]
            y_coords = resp[1]
            n_pts = len(x_coords)

            # 6. Priority 1: Curvature-Constrained Dynamic Velocity Profiling
            dx = np.gradient(x_coords)
            dy = np.gradient(y_coords)
            ddx = np.gradient(dx)
            ddy = np.gradient(dy)
            curvatures = np.abs(dx * ddy - dy * ddx) / ((dx**2 + dy**2)**1.5 + 1e-9)

            # Physical lateral velocity limit under available tire friction (v_lat = sqrt(a_lat / kappa))
            v_lat_limits = np.sqrt(self.spline_max_alat / np.maximum(curvatures, 0.05))

            # Initial target bounded by raceline profile and curvature limit
            v_targets = np.zeros(n_pts)
            for i in range(n_pts):
                gb_wpnt_i = int((evasion_s[i] / wpnt_dist) % self.gb_max_idx)
                v_raceline = gb_wpnts[gb_wpnt_i].vx_mps
                v_targets[i] = min(v_raceline, v_lat_limits[i])

            # Backward pass (Euler deceleration limit for braking)
            ds = spline_res
            for i in range(n_pts - 2, -1, -1):
                v_max_brake = np.sqrt(v_targets[i+1]**2 + 2.0 * self.spline_abrake_max * ds)
                v_targets[i] = min(v_targets[i], v_max_brake)

            # Forward pass (Euler acceleration limit for traction)
            for i in range(1, n_pts):
                v_max_accel = np.sqrt(v_targets[i-1]**2 + 2.0 * self.spline_aaccel_max * ds)
                v_targets[i] = min(v_targets[i], v_max_accel)

            # 7. Construct Output Waypoints and Telemetry Markers
            for i in range(n_pts):
                vi = float(v_targets[i])
                wpnts.wpnts.append(
                    self.xyv_to_wpnts(
                        x=x_coords[i], y=y_coords[i], s=evasion_s[i], d=evasion_d[i], v=vi, wpnts=wpnts
                    )
                )
                mrks.markers.append(
                    self.xyv_to_markers(x=x_coords[i], y=y_coords[i], v=vi, mrks=mrks)
                )

            # 8. Metadata and State Machine Signals
            wpnts.header.stamp = self.get_clock().now().to_msg()
            wpnts.header.frame_id = "map"
            wpnts.ot_side = chosen_side
            wpnts.ot_line = outside
            wpnts.side_switch = True if self.last_ot_side != chosen_side else False
            wpnts.last_switch_time = self.last_switch_time

            if self.last_ot_side != chosen_side:
                self.last_switch_time = self.get_clock().now().to_msg()
            self.last_ot_side = chosen_side

        else:
            self.last_ot_side = ""

        return wpnts, mrks

    def compute_dynamic_lookahead(self) -> float:
        """
        Dynamically scale spline lookahead distance based on vehicle linear velocity
        and local raceline curvature:
            L_base = k_v * v_s + L_min
            L = L_base / (1.0 + k_kappa * |kappa|)
            L = clamp(L, L_min, L_max)
        """
        vs = max(0.0, float(self.cur_vs)) if self.cur_vs is not None else 0.0
        l_base = self.lookahead_kv * vs + self.lookahead_min

        # Local raceline curvature attenuation
        kappa = 0.0
        wpnts_list = getattr(self.gb_wpnts, 'wpnts', self.gb_wpnts)
        if wpnts_list is not None and len(wpnts_list) > 0 and self.cur_s is not None and self.gb_max_s is not None:
            s_diffs = [abs((w.s_m - self.cur_s) % self.gb_max_s) for w in wpnts_list]
            closest_idx = int(np.argmin(s_diffs))
            w = wpnts_list[closest_idx]
            if hasattr(w, 'kappa_radpm'):
                kappa = abs(float(w.kappa_radpm))

        l_scaled = l_base / (1.0 + self.lookahead_kkappa * kappa)
        return float(np.clip(l_scaled, self.lookahead_min, self.lookahead_max))

    def _obs_filtering(self, obstacles: ObstacleArray) -> List[Obstacle]:
        # Dynamically adapt lookahead distance to current speed and curvature
        self.lookahead = self.compute_dynamic_lookahead()

        # Only use obstacles that are within a threshold of the raceline, else we don't care about them
        obs_on_traj = [obs for obs in obstacles.obstacles if abs(
            obs.d_center) < self.obs_traj_tresh]

        # Only use obstacles that within self.lookahead in front of the car
        close_obs = []
        for obs in obs_on_traj:
            obs = self._predict_obs_movement(obs)
            # Handle wraparound
            dist_in_front = (obs.s_center - self.cur_s) % self.gb_max_s
            # dist_in_back = abs(dist_in_front % (-self.gb_max_s)) # distance from ego to obstacle in the back
            if dist_in_front < self.lookahead:
                close_obs.append(obs)
                # Not within lookahead
            else:
                pass
        return close_obs

    #################### VISUALIZATION FUNCTIONS####################
    def xyv_to_markers(self, x: float, y: float, v: float, mrks: MarkerArray) -> Marker:
        mrk = Marker()
        mrk.header.frame_id = "map"
        mrk.header.stamp = self.get_clock().now().to_msg()
        mrk.type = mrk.CYLINDER
        mrk.scale.x = 0.1
        mrk.scale.y = 0.1
        mrk.scale.z = float(v / self.gb_vmax)
        mrk.color.a = 1.0
        mrk.color.b = 0.75
        mrk.color.r = 0.75
        if self.from_bag:
            mrk.color.g = 0.75

        mrk.id = len(mrks.markers)
        mrk.pose.position.x = float(x)
        mrk.pose.position.y = float(y)
        mrk.pose.position.z = v / float(self.gb_vmax / 2)
        mrk.pose.orientation.w = 1.0

        return mrk

    def xy_to_point(self, x: float, y: float, opponent=True) -> Marker:
        mrk = Marker()
        mrk.header.frame_id = "map"
        mrk.header.stamp = self.get_clock().now().to_msg()
        mrk.type = mrk.SPHERE
        mrk.scale.x = 0.5
        mrk.scale.y = 0.5
        mrk.scale.z = 0.5
        mrk.color.a = 0.8
        mrk.color.b = 0.65
        mrk.color.r = 1.0 if opponent else 0.0
        mrk.color.g = 0.65

        mrk.pose.position.x = float(x)
        mrk.pose.position.y = float(y)
        mrk.pose.position.z = 0.01
        mrk.pose.orientation.w = 1.0

        return mrk

    def xyv_to_wpnts(self, s: float, d: float, x: float, y: float, v: float, wpnts: OTWpntArray) -> Wpnt:
        wpnt = Wpnt()
        wpnt.id = len(wpnts.wpnts)
        wpnt.x_m = float(x)
        wpnt.y_m = float(y)
        wpnt.s_m = float(s)
        wpnt.d_m = float(d)
        wpnt.vx_mps = float(v)
        return wpnt


def main(args=None):
    rclpy.init(args=args)
    spline_planner = ObstacleSpliner()
    rclpy.spin(spline_planner)
    spline_planner.destroy_node()
    rclpy.shutdown()
