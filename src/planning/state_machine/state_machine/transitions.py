from __future__ import annotations

from typing import TYPE_CHECKING

from state_machine.state_types import StateType, TopLevelMode

if TYPE_CHECKING:
    from state_machine.state_machine import StateMachine


def hierarchical_transition(state_machine: StateMachine) -> StateType:
    """Top-Level Hierarchical State Transition Dispatcher."""
    p = state_machine.params

    # 1. Top-Level Mode Safety Check
    if state_machine._low_bat:
        state_machine.top_level_mode = TopLevelMode.SAFETY
        return StateType.LOW_BAT

    # 1b. Force FTG manual override (from joy_combo_node or operator)
    if getattr(state_machine, "force_ftg", False):
        return StateType.FTGONLY

    # 2. Check for Recovery Trigger
    if p.enable_recovery and state_machine.top_level_mode != TopLevelMode.RECOVERY:
        if state_machine._check_stuck:
            state_machine.get_logger().warn("[HSM] Crash / Stuck condition detected! Switching to RECOVERY mode.")
            state_machine.top_level_mode = TopLevelMode.RECOVERY
            state_machine.recovery_start_time = state_machine.get_clock().now().nanoseconds * 1e-9
            state_machine.recovery_substate_start = state_machine.recovery_start_time
            state_machine._evaluate_recovery_steer_dir()
            return StateType.RECOVERY_STUCK

    # 3. Handle Active Top-Level Mode
    if state_machine.top_level_mode == TopLevelMode.RECOVERY:
        return recovery_transition(state_machine)
    elif state_machine.top_level_mode == TopLevelMode.SAFETY:
        return StateType.LOW_BAT
    else:
        # TopLevelMode.NORMAL_RACING
        if p.mode == 'dummy':
            return dummy_transition(state_machine)
        elif p.mode == 'timetrials':
            return timetrials_transition(state_machine)
        elif p.mode == 'head_to_head':
            return head_to_head_transition(state_machine)
        elif p.mode == 'qualifying':
            return qualifying_transition(state_machine)
        else:
            return StateType.GB_TRACK


def recovery_transition(state_machine: StateMachine) -> StateType:
    """Sub-state transitions within RECOVERY mode."""
    p = state_machine.params
    now_sec = state_machine.get_clock().now().nanoseconds * 1e-9
    substate_elapsed = now_sec - state_machine.recovery_substate_start

    front_clear, rear_clear = state_machine._evaluate_scan_clearance()

    if state_machine.state == StateType.RECOVERY_STUCK:
        # 1. Bring car to a complete stop and settle before initiating reverse
        stop_time = getattr(p, "recovery_stop_time", 0.5)
        if substate_elapsed >= stop_time:
            state_machine._evaluate_recovery_steer_dir()
            state_machine.recovery_substate_start = now_sec
            state_machine.get_logger().info(f"[RECOVERY] Car stopped and settled ({substate_elapsed:.2f}s). Beginning reverse maneuver.")
            return StateType.RECOVERY_REVERSE
        return StateType.RECOVERY_STUCK

    elif state_machine.state == StateType.RECOVERY_REVERSE:
        # Exit conditions to RECOVERY_ALIGN:
        # 1. Front distance is clear (after at least 0.4s of reversing)
        # 2. Rear distance is blocked (avoid crashing into opposite wall)
        # 3. Reverse timeout reached
        if front_clear > p.recovery_front_safe_dist and substate_elapsed >= 0.4:
            state_machine.get_logger().info(f"[RECOVERY] Front clearance restored ({front_clear:.2f}m). Moving to ALIGN.")
            state_machine.recovery_substate_start = now_sec
            return StateType.RECOVERY_ALIGN

        if substate_elapsed >= 0.3 and rear_clear < p.recovery_rear_min_dist:
            state_machine.get_logger().warn(f"[RECOVERY] Rear obstruction detected ({rear_clear:.2f}m). Stopping reverse, moving to ALIGN.")
            state_machine.recovery_substate_start = now_sec
            return StateType.RECOVERY_ALIGN

        if substate_elapsed >= p.recovery_max_reverse_time:
            state_machine.get_logger().info(f"[RECOVERY] Reverse timer expired ({substate_elapsed:.2f}s). Moving to ALIGN.")
            state_machine.recovery_substate_start = now_sec
            return StateType.RECOVERY_ALIGN

        return StateType.RECOVERY_REVERSE

    elif state_machine.state == StateType.RECOVERY_ALIGN:
        # Stay in ALIGN (FTG mode stabilization) for recovery_align_time
        if substate_elapsed >= p.recovery_align_time:
            # Localization Health Gate: verify particle filter covariance has converged
            if state_machine._check_localization_healthy:
                state_machine.get_logger().info("[RECOVERY] Recovery sequence complete and localization converged! Resuming NORMAL_RACING.")
                state_machine.top_level_mode = TopLevelMode.NORMAL_RACING
                state_machine.stuck_counter = 0
                return StateType.GB_TRACK
            else:
                state_machine.get_logger().warn(
                    f"[RECOVERY] Localization trace ({state_machine.loc_cov_trace:.3f} m^2) exceeds safe limit ({p.max_loc_cov_trace} m^2). Maintaining reactive FTG alignment...",
                    throttle_duration_sec=1.0
                )
                if substate_elapsed >= getattr(p, "relocalization_timeout", 3.0):
                    state_machine.trigger_relocalization()
                return StateType.RECOVERY_ALIGN

        return StateType.RECOVERY_ALIGN

    else:
        # Default fallback
        state_machine.top_level_mode = TopLevelMode.NORMAL_RACING
        return StateType.GB_TRACK



def dummy_transition(state_machine: StateMachine)->str:
    match state_machine.state:
        case StateType.GB_TRACK:
            if state_machine._low_bat:
                return StateType.LOW_BAT
            else:
                return StateType.GB_TRACK
        case StateType.LOW_BAT:
            return StateType.LOW_BAT
        case default:
            return StateType.GB_TRACK
        
        
def timetrials_transition(state_machine: StateMachine)->str:
    return StateType.GB_TRACK


def qualifying_transition(state_machine: StateMachine) -> str:
    """Transition for qualifying mode.

    Divides the run into 5 phases based on lap count:
      Q1        : laps 1..q1_laps                   → GB_TRACK, full speed
      Buffer 1  : laps q1_laps+1..q1_laps+buf_laps  → GB_TRACK, slow speed
      Q2        : next q2_laps laps                 → head_to_head logic, full speed
      Buffer 2  : next buf_laps laps                → GB_TRACK, slow speed
      Q3        : remaining laps                    → GB_TRACK, full speed
    """
    phase = get_qualifying_phase(state_machine)
    p = state_machine.params

    if phase == "Q1":
        # Q1 — full speed, no avoidance
        state_machine.qual_speed_scale = 1.0
        return StateType.GB_TRACK
    elif phase == "Buffer 1":
        # Buffer 1 — slow down, no avoidance
        state_machine.qual_speed_scale = p.buffer_speed_scale
        return StateType.GB_TRACK
    elif phase == "Q2":
        # Q2 — obstacle avoidance active at q2_speed_scale
        state_machine.qual_speed_scale = getattr(p, 'q2_speed_scale', 1.0)
        return head_to_head_transition(state_machine)
    elif phase == "Buffer 2":
        # Buffer 2 — slow down, no avoidance
        state_machine.qual_speed_scale = p.buffer_speed_scale
        return StateType.GB_TRACK
    else:
        # Q3 — full speed, no avoidance
        state_machine.qual_speed_scale = 1.0
        return StateType.GB_TRACK


def get_qualifying_phase(state_machine: StateMachine) -> str:
    """Helper function to return the active qualifying phase string."""
    p = state_machine.params
    lap = state_machine.lap_count

    q1_end    = p.q1_laps
    buf1_end  = q1_end + p.buf_laps
    q2_end    = buf1_end + p.q2_laps
    buf2_end  = q2_end + p.buf_laps

    if lap <= q1_end:
        return "Q1"
    elif lap <= buf1_end:
        return "Buffer 1"
    elif lap <= q2_end:
        return "Q2"
    elif lap <= buf2_end:
        return "Buffer 2"
    else:
        return "Q3"


def head_to_head_transition(state_machine: StateMachine)->str:
    match state_machine.state:
        case StateType.GB_TRACK:
            return SpliniGlobalTrackingTransition(state_machine)
        case StateType.TRAILING:
            return SpliniTrailingTransition(state_machine)
        case StateType.OVERTAKE:
            return SpliniOvertakingTransition(state_machine)
        case StateType.FTGONLY:
            return SpliniFTGOnlyTransition(state_machine)
        case default:
            raise ValueError(f"Invalid state {state_machine.state}")


def SpliniGlobalTrackingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.GB_TRACK`"""
    if not state_machine._check_only_ftg_zone:
        if state_machine._check_gbfree:
            return StateType.GB_TRACK
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY


def SpliniTrailingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.TRAILING`"""
    gb_free = state_machine._check_gbfree
    ot_sector = state_machine._check_ot_sector

    if not state_machine._check_only_ftg_zone:
        # If we have been sitting around in TRAILING for a while then FTG
        if state_machine._check_ftg:
            return StateType.FTGONLY
        if gb_free:
            return StateType.GB_TRACK
        elif (
            ot_sector
            and state_machine._check_availability_splini_wpts
            and state_machine._check_ofree
        ):
            return StateType.OVERTAKE
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY


def SpliniOvertakingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.OVERTAKE`"""
    if not state_machine._check_only_ftg_zone:
        in_ot_sector = state_machine._check_ot_sector
        spline_valid = state_machine._check_availability_splini_wpts
        o_free = state_machine._check_ofree
        dwell_time = getattr(state_machine, 'state_dwell_time', 1.0)

        # If raceline is free and the ego car has passed the obstacle or rejoined the raceline: return to GB_TRACK
        if state_machine._check_gbfree and (not state_machine._check_enemy_in_front or state_machine._check_close_to_raceline):
            return StateType.GB_TRACK

        # Minimum dwell time hysteresis: prevent 30Hz flutter back to TRAILING during active overtake
        if dwell_time < 0.4 and (spline_valid or state_machine.last_valid_avoidance_wpnts is not None):
            return StateType.OVERTAKE

        # If spline is on an obstacle we trail
        if not o_free:
            return StateType.TRAILING
        if in_ot_sector and o_free and (spline_valid or state_machine.last_valid_avoidance_wpnts is not None):
            return StateType.OVERTAKE
        # If spline becomes invalid while overtaking, we trail
        elif in_ot_sector and not spline_valid and not o_free:
            return StateType.TRAILING
        # go to GB_TRACK if not in ot_sector and the GB is free
        elif not in_ot_sector and state_machine._check_gbfree:
            return StateType.GB_TRACK
        # go to Trailing if not in ot_sector and the GB is not free
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY


def SpliniFTGOnlyTransition(state_machine: StateMachine) -> StateType:
    if state_machine._check_only_ftg_zone:
        return StateType.FTGONLY
    else:
        if state_machine._check_close_to_raceline and state_machine._check_gbfree:
            return StateType.GB_TRACK
        else:
            return StateType.FTGONLY
        