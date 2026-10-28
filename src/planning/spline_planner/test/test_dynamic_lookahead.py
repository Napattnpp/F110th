import pytest
import rclpy
from f110_msgs.msg import Wpnt
from spline_planner.spline_planner import ObstacleSpliner


class MockSpliner:
    """Lightweight test harness for ObstacleSpliner dynamic lookahead calculation."""
    def __init__(self):
        self.lookahead_min = 4.0
        self.lookahead_max = 12.0
        self.lookahead_kv = 1.0
        self.lookahead_kkappa = 0.5
        self.cur_vs = 0.0
        self.cur_s = 0.0
        self.gb_max_s = 50.0
        self.gb_wpnts = []

    compute_dynamic_lookahead = ObstacleSpliner.compute_dynamic_lookahead


def test_dynamic_lookahead_speed_scaling():
    planner = MockSpliner()

    # 1. Standstill / low speed
    planner.cur_vs = 0.0
    la_low = planner.compute_dynamic_lookahead()
    assert la_low == pytest.approx(4.0), "Standstill lookahead should equal lookahead_min"

    # 2. Medium speed (4.0 m/s) on straight line
    planner.cur_vs = 4.0
    la_med = planner.compute_dynamic_lookahead()
    assert la_med == pytest.approx(8.0), "4 m/s should yield 1.0 * 4.0 + 4.0 = 8.0m"

    # 3. High speed (12.0 m/s): should clamp at lookahead_max
    planner.cur_vs = 12.0
    la_high = planner.compute_dynamic_lookahead()
    assert la_high == pytest.approx(12.0), "High speed lookahead should clamp at lookahead_max (12.0m)"


def test_dynamic_lookahead_curvature_attenuation():
    planner = MockSpliner()
    planner.cur_vs = 6.0  # L_base = 1.0 * 6.0 + 4.0 = 10.0m

    # Create waypoint at s=10 with curvature kappa = 1.0 rad/m
    w = Wpnt()
    w.s_m = 10.0
    w.kappa_radpm = 1.0
    planner.gb_wpnts = [w]
    planner.cur_s = 10.0

    la_curv = planner.compute_dynamic_lookahead()
    # Expected: 10.0 / (1.0 + 0.5 * 1.0) = 10.0 / 1.5 = 6.6667m
    assert la_curv == pytest.approx(10.0 / 1.5, abs=0.01)
    assert la_curv < 10.0, "Curvature should contract the lookahead distance"
