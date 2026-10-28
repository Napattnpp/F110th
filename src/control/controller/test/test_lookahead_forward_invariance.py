import pytest
import numpy as np
from controller.map import MAP_Controller
from controller.pp import PP_Controller


def make_map_controller():
    mc = MAP_Controller.__new__(MAP_Controller)
    mc.t_clip_min = 0.5
    mc.t_clip_max = 5.0
    return mc


def test_map_controller_lookahead_no_backward_jump():
    """Verify that MAP_Controller.waypoint_at_distance_before_car does not wrap to index 0 (backward)."""
    mc = make_map_controller()
    
    # Simulate open local waypoint array (80 points spanning x=0.0 to x=8.0, y=0.0)
    # This represents /state_machine/local_waypoints
    waypoints = np.zeros((80, 2))
    waypoints[:, 0] = np.linspace(0.0, 8.0, 80)
    
    # 1. Car near start of window (idx 0), lookahead 3.0m
    pt1 = mc.waypoint_at_distance_before_car(distance=3.0, waypoints=waypoints, idx_waypoint_behind_car=0)
    assert pt1[0] == pytest.approx(3.0, abs=0.05), "Should interpolate to x=3.0m"
    assert pt1[0] > 0.0, "Lookahead point must be ahead of car"

    # 2. Car near end of window (idx 70, x=7.07m), lookahead 3.0m (exceeds 8.0m limit)
    # Old implementation wrapped to index 0, pointing backwards to x=0.0!
    pt2 = mc.waypoint_at_distance_before_car(distance=3.0, waypoints=waypoints, idx_waypoint_behind_car=70)
    assert pt2[0] >= 8.0, "Must project forward along tangent beyond window, never wrap back to 0"
    assert pt2[0] > waypoints[70, 0], "Lookahead point must strictly remain ahead of car position"


def test_pure_pursuit_lookahead_no_backward_jump():
    """Verify that PP_Controller.waypoint_at_distance_before_car does not wrap backwards."""
    pp = PP_Controller(
        t_clip_min=0.5,
        t_clip_max=5.0,
        m_l1=0.2,
        q_l1=1.0,
        speed_lookahead=1.0,
        lat_err_coeff=1.0,
        acc_scaler_for_steer=1.0,
        dec_scaler_for_steer=1.0,
        start_scale_speed=1.0,
        end_scale_speed=1.0,
        downscale_factor=1.0,
        speed_lookahead_for_steer=1.0,
        prioritize_dyn=False,
        trailing_gap=1.0,
        trailing_p_gain=1.0,
        trailing_i_gain=0.1,
        trailing_d_gain=0.1,
        blind_trailing_speed=2.0,
        loop_rate=40,
        wheelbase=0.33
    )
    
    waypoints = np.zeros((80, 2))
    waypoints[:, 0] = np.linspace(0.0, 8.0, 80)
    
    pt = pp.waypoint_at_distance_before_car(distance=4.0, waypoints=waypoints, idx_waypoint_behind_car=75)
    assert pt[0] > waypoints[75, 0], "PurePursuit lookahead must strictly project forward"

