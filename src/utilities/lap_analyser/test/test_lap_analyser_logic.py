import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry
from f110_msgs.msg import Wpnt, WpntArray
from lap_analyser.lap_analyser import LapAnalyser


@pytest.fixture(scope="module")
def ros_init():
    rclpy.init()
    yield
    rclpy.shutdown()


def create_mock_waypoints(track_length=50.0, num_points=100):
    wp_array = WpntArray()
    for i in range(num_points):
        w = Wpnt()
        w.id = i
        w.s_m = float(i * (track_length / num_points))
        w.x_m = float(i)
        w.y_m = 0.0
        w.psi_rad = 0.0
        w.d_right = 1.5
        w.d_left = 1.5
        wp_array.wpnts.append(w)
    return wp_array


def test_lap_counting_hysteresis(ros_init):
    node = LapAnalyser()
    wp_msg = create_mock_waypoints(track_length=50.0)
    node.waypoints_cb(wp_msg)

    assert node.track_length == pytest.approx(49.5, abs=1.0)
    assert node.lap_armed is True  # Armed on startup for grid start

    # 1. Startup crossing from grid (s=48.0 -> s=0.5, moving forward)
    node.last_s = 48.0
    crossed = node.check_for_finish_line_pass(current_s=0.5, vs=2.0)
    assert crossed is True, "First crossing should start Lap 0"
    node.lap_count = 0

    # After crossing, trigger should be disarmed until car reaches > 60% of track
    assert node.lap_armed is False

    # 2. Moving along track at 20% progress (s=10.0) -> should not trigger
    node.last_s = 9.0
    assert node.check_for_finish_line_pass(current_s=10.0, vs=3.0) is False
    assert node.lap_armed is False

    # 3. Odometry glitch / reverse near starting line: s=0.5 to s=0.1
    node.last_s = 0.5
    assert node.check_for_finish_line_pass(current_s=0.1, vs=1.0) is False, "Small s drop without progress must NOT trigger lap"

    # 4. Car progresses past 60% (s=35.0, where 35 > 0.6 * 50 = 30) -> should arm
    node.last_s = 34.0
    assert node.check_for_finish_line_pass(current_s=35.0, vs=4.0) is False
    assert node.lap_armed is True, "Reaching >60% track progress must arm lap trigger"

    # 5. Reverse rollback test: car rolls backwards across finish line (vs = -1.0)
    node.last_s = 48.0
    assert node.check_for_finish_line_pass(current_s=0.5, vs=-1.0) is False, "Backwards motion must NEVER count as completed lap"

    # 6. Valid forward crossing (vs = 3.5, s=48.5 -> s=0.2)
    # Simulate elapsed time > min_lap_duration_sec
    node.lap_start_time = node.get_clock().now() - rclpy.duration.Duration(seconds=5.0)
    node.last_s = 48.5
    assert node.check_for_finish_line_pass(current_s=0.2, vs=3.5) is True, "Valid forward crossing with progress must trigger lap"
    assert node.lap_armed is False

    node.destroy_node()

