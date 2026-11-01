import pytest
import numpy as np
from types import SimpleNamespace
from state_machine.state_types import StateType, TopLevelMode
from state_machine.transitions import SpliniOvertakingTransition
from f110_msgs.msg import Obstacle, Wpnt, WpntArray


class MockParams:
    def __init__(self):
        self.enable_recovery = True
        self.mode = 'head_to_head'
        self.overtake_mode = 'spliner'
        self.gb_horizon_m = 4.5
        self.lateral_width_gb_m = 0.35
        self.lateral_width_ot_m = 0.18
        self.overtaking_horizon_m = 3.5
        self.splini_hyst_timer_sec = 0.75
        self.n_loc_wpnts = 80
        self.rate_hz = 40.0


class MockStateMachine:
    def __init__(self):
        self.params = MockParams()
        self.state = StateType.OVERTAKE
        self.top_level_mode = TopLevelMode.NORMAL_RACING
        self.cur_s = 25.0
        self.cur_d = 0.35
        self.cur_vs = 4.0
        self.track_length = 41.93
        self.state_dwell_time = 1.0
        self.last_valid_avoidance_wpnts = [Wpnt(s_m=25.0 + i*0.1, d_m=0.35) for i in range(30)]
        self.avoidance_wpnts = SimpleNamespace(
            wpnts=self.last_valid_avoidance_wpnts,
            header=SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=0)),
            last_switch_time=SimpleNamespace(sec=90, nanosec=0)
        )
        self.obstacles = []
        self.ot_sectors = [{'start': 0, 'end': 422, 'ot_flag': True}]
        self.num_glb_wpnts = 422
        self.glb_wpnts = [Wpnt(s_m=i*0.1, vx_mps=4.5, kappa_radpm=0.05, d_left=1.2, d_right=1.2) for i in range(422)]

    @property
    def _check_only_ftg_zone(self):
        return False

    @property
    def _check_ot_sector(self):
        return True

    @property
    def _check_availability_splini_wpts(self):
        return True

    @property
    def _check_ofree(self):
        return True

    @property
    def _check_opponent_behind(self):
        if not self.obstacles or self.cur_s is None or self.track_length is None:
            return False
        half_track = self.track_length / 2.0
        for obs in self.obstacles:
            rel_s = (obs.s_center - self.cur_s + half_track) % self.track_length - half_track
            if -5.5 <= rel_s <= -1.2:
                return True
        return False

    @property
    def _check_alongside_obstacle(self):
        if not self.obstacles or self.cur_s is None or self.track_length is None:
            return False
        half_track = self.track_length / 2.0
        for obs in self.obstacles:
            rel_s = (obs.s_center - self.cur_s + half_track) % self.track_length - half_track
            if -1.2 <= rel_s <= 1.2:
                return True
        return False

    @property
    def _check_gbfree(self):
        horizon = self.params.gb_horizon_m
        for obs in self.obstacles:
            gap = (obs.s_center - self.cur_s) % self.track_length
            if gap < horizon:
                if abs(obs.d_center) < self.params.lateral_width_gb_m:
                    return False
        return True

    @property
    def _check_enemy_in_front(self):
        horizon = self.params.gb_horizon_m
        for obs in self.obstacles:
            gap = (obs.s_start - self.cur_s) % self.track_length
            if gap < horizon:
                return True
        return False


def test_overtake_must_not_abort_midway_alongside_opponent():
    """Verify that when the car's center is alongside or slightly ahead of the opponent
    (e.g. ego at s=25.05m, opponent at s=25.00m), it DOES NOT early-exit to GB_TRACK.
    """
    sm = MockStateMachine()
    
    # Opponent centered at s=25.00m, length 0.50m (span 24.75 to 25.25m)
    obs = Obstacle()
    obs.id = 5725
    obs.s_center = 25.00
    obs.s_start = 24.75
    obs.s_end = 25.25
    obs.d_center = 0.0
    obs.vs = 2.0
    sm.obstacles = [obs]
    
    # Ego is at s=25.05m (only 5 cm ahead of obstacle center - physically alongside!)
    sm.cur_s = 25.05
    sm.cur_d = 0.35  # offline on the avoidance corridor
    
    # Ego is physically alongside the obstacle
    assert sm._check_alongside_obstacle is True
    # Opponent is NOT cleanly behind yet (requires at least 1.2m behind)
    assert sm._check_opponent_behind is False
    
    # Under current code, what does SpliniOvertakingTransition return?
    next_state = SpliniOvertakingTransition(sm)
    
    # CRITICAL ASSERTION: The car MUST STAY in OVERTAKE while alongside!
    assert next_state == StateType.OVERTAKE, (
        f"BUG #2 REPRODUCED: SpliniOvertakingTransition returned {next_state} mid-way through "
        f"the maneuver while still alongside the opponent (rel_s = {25.00 - 25.05:.2f}m)!"
    )


def test_overtake_must_not_abort_when_nose_clears_opponent():
    """Verify that when ego nose has passed opponent but opponent is still
    within trailing active pass window (e.g. ego at 25.60m, opp at 25.00m, rel_s = -0.60m),
    it remains in OVERTAKE rather than prematurely returning to raceline.
    """
    sm = MockStateMachine()
    obs = Obstacle()
    obs.id = 5725
    obs.s_center = 25.00
    obs.s_start = 24.75
    obs.s_end = 25.25
    obs.d_center = 0.0
    obs.vs = 2.0
    sm.obstacles = [obs]

    sm.cur_s = 25.60
    sm.cur_d = 0.35

    next_state = SpliniOvertakingTransition(sm)
    assert next_state == StateType.OVERTAKE, (
        f"Premature exit: Expected OVERTAKE when rel_s=-0.60m, got {next_state}"
    )


def test_overtake_exits_to_gb_track_only_when_opponent_cleanly_behind():
    """Verify that once the opponent is cleanly behind (>1.2m behind, rel_s = -1.6m),
    SpliniOvertakingTransition cleanly transitions back to GB_TRACK.
    """
    sm = MockStateMachine()
    obs = Obstacle()
    obs.id = 5725
    obs.s_center = 25.00
    obs.s_start = 24.75
    obs.s_end = 25.25
    obs.d_center = 0.0
    obs.vs = 2.0
    sm.obstacles = [obs]

    # Ego is 1.6m ahead (opponent is at rel_s = -1.6m behind)
    sm.cur_s = 26.60
    sm.cur_d = 0.35

    assert sm._check_opponent_behind is True
    assert sm._check_alongside_obstacle is False

    next_state = SpliniOvertakingTransition(sm)
    assert next_state == StateType.GB_TRACK, (
        f"Expected GB_TRACK when opponent is cleanly behind, got {next_state}"
    )


def test_overtake_does_not_abort_on_sector_boundary_while_alongside():
    """Verify that reaching a non-overtaking sector boundary while physically alongside
    the opponent DOES NOT trigger an immediate sector abort into the opponent.
    """
    sm = MockStateMachine()
    obs = Obstacle()
    obs.id = 5725
    obs.s_center = 25.00
    obs.s_start = 24.75
    obs.s_end = 25.25
    obs.d_center = 0.0
    obs.vs = 2.0
    sm.obstacles = [obs]

    sm.cur_s = 25.10
    sm.cur_d = 0.35
    # Simulate entering a non-overtake sector
    sm.ot_sectors = [{'start': 0, 'end': 20, 'ot_flag': True}, {'start': 21, 'end': 422, 'ot_flag': False}]

    class SectorGatedStateMachine(MockStateMachine):
        @property
        def _check_ot_sector(self):
            return False

    sm_gated = SectorGatedStateMachine()
    sm_gated.obstacles = [obs]
    sm_gated.cur_s = 25.10
    sm_gated.cur_d = 0.35

    assert sm_gated._check_ot_sector is False
    assert sm_gated._check_alongside_obstacle is True

    next_state = SpliniOvertakingTransition(sm_gated)
    assert next_state == StateType.OVERTAKE, (
        f"Safety violation: Car aborted to {next_state} while alongside on sector boundary!"
    )
