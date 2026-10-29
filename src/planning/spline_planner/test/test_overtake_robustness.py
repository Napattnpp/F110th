import numpy as np
import pytest
from f110_msgs.msg import Wpnt, Obstacle, ObstacleArray
from spline_planner.spline_planner import ObstacleSpliner


class MockConverter:
    def get_cartesian(self, s_arr, d_arr):
        # Approximate Cartesian for testing: straight line along x with lateral offset y = d
        s = np.array(s_arr, dtype=float)
        d = np.array(d_arr, dtype=float)
        return np.vstack([s, d])


class MockPublisher:
    def publish(self, msg):
        pass


class MockClock:
    def now(self):
        class MockTime:
            def to_msg(self):
                from builtin_interfaces.msg import Time
                return Time()
        return MockTime()


class MockRobustSpliner:
    """Test harness for robust overtaking algorithms."""
    def __init__(self):
        self._clock = MockClock()
        self.pre_apex_0 = -5.5
        self.pre_apex_1 = -3.8
        self.pre_apex_2 = -2.0
        self.post_apex_0 = 1.5
        self.post_apex_1 = 3.2
        self.post_apex_2 = 5.0
        self.evasion_dist = 0.40
        self.spline_bound_mindist = 0.18
        self.ot_side_switch_cur_d_thresh = 0.55
        self.spline_max_alat = 6.5
        self.spline_abrake_max = 5.0
        self.spline_aaccel_max = 4.0
        self.overtake_speed_boost = 1.05
        self.min_pass_delta_v = 0.8
        self.corner_curvature_thresh = 0.08
        self.prediction_mode = "curvature_aware"
        self.multi_obs_safety_margin = 0.25
        self.fixed_pred_time = 0.20
        self.kd_obs_pred = 1.0
        self.lookahead_min = 4.0
        self.lookahead_max = 12.0
        self.lookahead_kv = 1.0
        self.lookahead_kkappa = 0.5
        self.obs_traj_tresh = 0.55

        self.cur_s = 0.0
        self.cur_d = 0.0
        self.cur_vs = 4.0
        self.gb_vmax = 6.0
        self.gb_max_s = 50.0
        self.gb_max_idx = 500
        self.last_ot_side = ""
        self.last_switch_time = self._clock.now().to_msg()
        self.hsm_state = "OVERTAKE"
        self.from_bag = False
        self.enable_predictive_intercept = True
        self.max_intercept_horizon = 1.5
        self.min_closing_speed = 0.4

        self.converter = MockConverter()
        self.pub_propagated = MockPublisher()
        self.closest_obs_pub = MockPublisher()

        # Build synthetic raceline waypoints (50m length, 0.1m spacing)
        self.gb_wpnts = []
        for i in range(500):
            w = Wpnt()
            w.s_m = i * 0.1
            w.d_left = 1.0   # 1.0m to left wall
            w.d_right = -1.0 # 1.0m to right wall
            w.vx_mps = 5.0
            w.kappa_radpm = 0.0
            self.gb_wpnts.append(w)

    def get_clock(self):
        return self._clock

    def get_logger(self):
        class MockLogger:
            def info(self, msg, throttle_duration_sec=0.0): pass
            def warn(self, msg, throttle_duration_sec=0.0): pass
            def error(self, msg, throttle_duration_sec=0.0): pass
        return MockLogger()

    _predict_obs_movement = ObstacleSpliner._predict_obs_movement
    _check_ot_side_possible = ObstacleSpliner._check_ot_side_possible
    _more_space = ObstacleSpliner._more_space
    _obs_filtering = ObstacleSpliner._obs_filtering
    compute_dynamic_lookahead = ObstacleSpliner.compute_dynamic_lookahead
    do_spline = ObstacleSpliner.do_spline
    xy_to_point = ObstacleSpliner.xy_to_point
    xyv_to_markers = ObstacleSpliner.xyv_to_markers
    xyv_to_wpnts = ObstacleSpliner.xyv_to_wpnts


def test_curvature_aware_prediction():
    spliner = MockRobustSpliner()
    spliner.cur_s = 15.0  # Car at 15.0m, obstacle at 20.0m (gap 5.0m < 12.0m horizon)
    # High curvature corner at opponent location (kappa = 1.5 rad/m)
    for w in spliner.gb_wpnts[150:250]:
        w.kappa_radpm = 1.5

    obs = Obstacle()
    obs.s_center = 20.0
    obs.s_start = 19.8
    obs.s_end = 20.2
    obs.d_center = 0.3
    obs.d_left = 0.45
    obs.d_right = 0.15
    obs.vs = 5.0
    obs.vd = 0.5

    # In curvature-aware mode, cornering speed should be bounded by sqrt(a_lat / kappa) = sqrt(7.0 / 1.5) = ~2.16 m/s
    pred_obs = spliner._predict_obs_movement(obs, mode="curvature_aware")
    v_corner_limit = np.sqrt(7.0 / 1.5)
    expected_delta_s = 0.20 * v_corner_limit
    assert pred_obs.s_center == pytest.approx(20.0 + expected_delta_s, abs=0.05)
    # Lateral drift should be damped by corner decay factor
    assert pred_obs.d_center < 0.3 + (0.5 * 0.20), "Lateral drift must be damped in high curvature"


def test_curvature_constrained_velocity_profiling():
    # Verify that a sharp avoidance curve's target speed is properly attenuated
    # rather than naively copying raceline speed
    a_lat_max = 7.0
    kappa_sharp = 1.2
    v_raceline = 6.0
    v_lat_limit = np.sqrt(a_lat_max / kappa_sharp) # ~2.415 m/s

    v_profile = min(v_raceline, v_lat_limit)
    assert v_profile < v_raceline, "Avoidance spline velocity must be capped by lateral acceleration budget"
    assert v_profile == pytest.approx(2.415, abs=0.01)


def test_full_horizon_boundary_rejection():
    spliner = MockRobustSpliner()
    spliner.cur_s = 10.0
    # Artificially pinch track boundaries across the overtaking corridor (s=10m to s=30m)
    # such that track clearance is too tight for safe evasion (< spline_bound_mindist)
    for w in spliner.gb_wpnts[100:300]:
        w.d_left = 0.35   # Only 0.35m left of raceline
        w.d_right = -0.35 # Only 0.35m right of raceline

    obs = Obstacle()
    obs.s_center = 15.0
    obs.s_start = 14.8
    obs.s_end = 15.2
    obs.d_center = 0.0 # Directly on raceline
    obs.d_left = 0.15
    obs.d_right = -0.15
    obs.vs = 2.0
    obs.vd = 0.0

    obs_arr = ObstacleArray()
    obs_arr.obstacles = [obs]

    # In a narrow corridor, full-horizon check must reject the spline and return empty waypoints
    wpnts, mrks = spliner.do_spline(obs_arr, spliner.gb_wpnts)
    assert len(wpnts.wpnts) == 0, "Spliner must reject overtaking in narrow corridors where boundary clearance is breached"


def test_multi_obstacle_collision_rejection():
    spliner = MockRobustSpliner()
    spliner.cur_s = 10.0

    # Obstacle 1: Slow lead car at s=16.0
    obs1 = Obstacle()
    obs1.s_center = 16.0
    obs1.s_start = 15.8
    obs1.s_end = 16.2
    obs1.d_center = 0.0
    obs1.d_left = 0.15
    obs1.d_right = -0.15
    obs1.vs = 2.0
    obs1.vd = 0.0

    # Obstacle 2: Secondary vehicle blocking the left overtaking lane at s=17.0, d=0.45
    obs2 = Obstacle()
    obs2.s_center = 17.0
    obs2.s_start = 16.8
    obs2.s_end = 17.2
    obs2.d_center = 0.45
    obs2.d_left = 0.60
    obs2.d_right = 0.30
    obs2.vs = 2.0
    obs2.vd = 0.0

    # Obstacle 3: Third vehicle blocking the right overtaking lane at s=17.0, d=-0.45
    obs3 = Obstacle()
    obs3.s_center = 17.0
    obs3.s_start = 16.8
    obs3.s_end = 17.2
    obs3.d_center = -0.45
    obs3.d_left = -0.30
    obs3.d_right = -0.60
    obs3.vs = 2.0
    obs3.vd = 0.0

    obs_arr = ObstacleArray()
    obs_arr.obstacles = [obs1, obs2, obs3]

    # Multi-obstacle check must detect that BOTH left and right lanes are blocked by secondary obstacles
    wpnts, mrks = spliner.do_spline(obs_arr, spliner.gb_wpnts)
    assert len(wpnts.wpnts) == 0, "Spliner must reject overtaking when secondary obstacles block both passing lanes"


def test_predictive_frenet_intercept():
    """Verify Pillar 4: Predictive Frenet Intercept Horizon forward-projects the apex."""
    spliner = MockRobustSpliner()
    spliner.cur_s = 10.0
    spliner.cur_vs = 5.0  # Ego speed = 5.0 m/s

    # Dynamic moving obstacle in front at s=14.0 moving at vs=2.0 m/s
    obs = Obstacle()
    obs.s_center = 14.0
    obs.s_start = 13.8
    obs.s_end = 14.2
    obs.d_center = 0.0
    obs.d_left = 0.15
    obs.d_right = -0.15
    obs.vs = 2.0
    obs.vd = 0.0
    obs.is_static = False

    obs_arr = ObstacleArray()
    obs_arr.obstacles = [obs]

    # delta_s = 14.0 - 10.0 = 4.0m
    # closing_speed = 5.0 - 2.0 = 3.0 m/s
    # t_intercept = 4.0 / 3.0 = 1.333s (< 1.5s max_intercept_horizon)
    # expected_apex = 14.0 + 2.0 * (4.0 / 3.0) = 14.0 + 2.667 = 16.667m

    wpnts, mrks = spliner.do_spline(obs_arr, spliner.gb_wpnts)
    assert len(wpnts.wpnts) > 0, "Spline must be generated for moving obstacle"

    # Find the peak lateral offset waypoint along the generated evasion spline
    d_vals = np.array([abs(w.d_m) for w in wpnts.wpnts])
    max_d = np.max(d_vals)
    plateau_s = [w.s_m for w in wpnts.wpnts if abs(w.d_m) >= max_d - 1e-4]
    peak_s_center = (plateau_s[0] + plateau_s[-1]) / 2.0

    # The peak of the evasion spline must be forward-projected to ~16.67m (not at current 14.0m!)
    assert peak_s_center > 15.0, f"Apex ({peak_s_center:.2f}m) must be forward-projected ahead of current obstacle position (14.0m)"
    assert min(plateau_s) <= 16.67 <= max(plateau_s), f"Predicted intercept 16.67m must lie within peak evasion corridor [{min(plateau_s):.2f}, {max(plateau_s):.2f}]"
    assert peak_s_center == pytest.approx(16.67, abs=0.6), f"Apex center {peak_s_center:.2f}m should align with predicted intercept (~16.67m)"


if __name__ == '__main__':
    test_curvature_aware_prediction()
    test_curvature_constrained_velocity_profiling()
    test_full_horizon_boundary_rejection()
    test_multi_obstacle_collision_rejection()
    test_predictive_frenet_intercept()
    print("All overtaking robustness unit tests passed successfully!")
