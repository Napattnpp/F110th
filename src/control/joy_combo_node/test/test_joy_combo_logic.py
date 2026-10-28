import json
import pytest
import rclpy
from sensor_msgs.msg import Joy
from joy_combo_node.joy_combo_node import JoyComboNode


@pytest.fixture(scope="module")
def ros_init():
    rclpy.init()
    yield
    rclpy.shutdown()


def create_joy_msg(axes=None, buttons=None):
    msg = Joy()
    # 8 axes: [ls_x, ls_y, lt, rs_x, rs_y, rt, dpad_x, dpad_y]
    # Logitech F710: unpressed triggers are 1.0; pressed is -1.0
    default_axes = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    if axes:
        for i, val in axes.items():
            default_axes[i] = val
    msg.axes = default_axes
    # 12 buttons: [A, B, X, Y, LB, RB, Back, Start, Guide, LS, RS, ...]
    default_buttons = [0] * 12
    if buttons:
        for i, val in buttons.items():
            default_buttons[i] = val
    msg.buttons = default_buttons
    return msg


def test_base_layer_safe(ros_init):
    node = JoyComboNode()
    estop_msgs = []
    initialpose_msgs = []
    step_msgs = []
    node.e_stop_pub.publish = lambda msg: estop_msgs.append(msg)
    node.initialpose_pub.publish = lambda msg: initialpose_msgs.append(msg)
    node.dpad_step_pub.publish = lambda msg: step_msgs.append(msg)

    # Press Button A without any triggers (Base layer)
    msg = create_joy_msg(buttons={0: 1})
    node.joy_callback(msg)
    assert len(initialpose_msgs) == 0, "Base layer Button A should do nothing"

    # Press Button B without any triggers
    msg = create_joy_msg(buttons={1: 1})
    node.joy_callback(msg)
    assert len(estop_msgs) == 0, "Base layer Button B should do nothing"

    # Press D-pad Up without any triggers
    msg = create_joy_msg(axes={7: 1.0})
    node.joy_callback(msg)
    assert len(step_msgs) == 0, "Base layer D-pad Up should do nothing"

    # Test RB (Button 5) toggles autonomy
    pause_msgs = []
    node.pause_nav_pub.publish = lambda msg: pause_msgs.append(msg)
    # RB press: toggles from True (initial) to False (running)
    node.joy_callback(create_joy_msg(buttons={5: 1}))
    assert len(pause_msgs) == 1 and pause_msgs[-1].data is False
    # Release RB
    node.joy_callback(create_joy_msg(buttons={5: 0}))
    # RB press again: toggles from False to True (paused)
    node.joy_callback(create_joy_msg(buttons={5: 1}))
    assert len(pause_msgs) == 2 and pause_msgs[-1].data is True

    node.destroy_node()


def test_lt_combos(ros_init):
    node = JoyComboNode()
    initialpose_msgs = []
    estop_msgs = []
    pause_msgs = []
    ftg_msgs = []
    bookmark_events = []
    bookmark_markers = []

    node.initialpose_pub.publish = lambda msg: initialpose_msgs.append(msg)
    node.e_stop_pub.publish = lambda msg: estop_msgs.append(msg)
    node.pause_nav_pub.publish = lambda msg: pause_msgs.append(msg)
    node.force_ftg_pub.publish = lambda msg: ftg_msgs.append(msg)
    node.foxglove_event_pub.publish = lambda msg: bookmark_events.append(msg)
    node.bookmark_marker_pub.publish = lambda msg: bookmark_markers.append(msg)

    # 1. LT + A -> reset_localizer
    msg1 = create_joy_msg(axes={2: -1.0}, buttons={0: 1})
    node.joy_callback(msg1)
    assert len(initialpose_msgs) == 1, "LT + A should publish /initialpose"

    # Debounce check: sending same message again shouldn't duplicate
    node.joy_callback(msg1)
    assert len(initialpose_msgs) == 1, "Debounce: holding button should not spam /initialpose"

    # Release Button A
    msg_release = create_joy_msg(axes={2: -1.0}, buttons={0: 0})
    node.joy_callback(msg_release)

    # 2. LT + B -> emergency_stop
    msg2 = create_joy_msg(axes={2: -1.0}, buttons={1: 1})
    node.joy_callback(msg2)
    assert len(estop_msgs) == 1, "LT + B should publish /e_stop"
    assert estop_msgs[0].drive.speed == 0.0
    assert len(pause_msgs) == 1 and pause_msgs[0].data is True

    # 3. LT + X -> toggle_ftg_mode
    msg3 = create_joy_msg(axes={2: -1.0}, buttons={2: 1})
    node.joy_callback(msg3)
    assert len(ftg_msgs) == 1 and ftg_msgs[0].data is True

    # Release and press again -> toggles to False
    node.joy_callback(create_joy_msg(axes={2: -1.0}, buttons={2: 0}))
    node.joy_callback(create_joy_msg(axes={2: -1.0}, buttons={2: 1}))
    assert len(ftg_msgs) == 2 and ftg_msgs[1].data is False

    # 4. LT + Y -> foxglove_bookmark
    msg4 = create_joy_msg(axes={2: -1.0}, buttons={3: 1})
    node.joy_callback(msg4)
    assert len(bookmark_events) == 1
    assert len(bookmark_markers) == 1
    event_data = json.loads(bookmark_events[0].data)
    assert event_data["type"] == "manual_bookmark"
    node.destroy_node()


def test_rt_dpad_stepping(ros_init):
    node = JoyComboNode()
    step_msgs = []
    node.dpad_step_pub.publish = lambda msg: step_msgs.append(msg)

    # 1. RT + D-pad Up (axis 5 = -1.0, axis 7 = 1.0)
    msg_up = create_joy_msg(axes={5: -1.0, 7: 1.0})
    node.joy_callback(msg_up)
    assert len(step_msgs) == 1
    payload = json.loads(step_msgs[-1].data)
    assert payload["action"] == "step_value_up"

    # Debounce check: continuous hold
    node.joy_callback(msg_up)
    assert len(step_msgs) == 1

    # Return D-pad to neutral
    node.joy_callback(create_joy_msg(axes={5: -1.0, 7: 0.0}))

    # 2. RT + D-pad Down (axis 7 = -1.0)
    msg_down = create_joy_msg(axes={5: -1.0, 7: -1.0})
    node.joy_callback(msg_down)
    assert len(step_msgs) == 2
    assert json.loads(step_msgs[-1].data)["action"] == "step_value_down"

    # Return to neutral
    node.joy_callback(create_joy_msg(axes={5: -1.0, 7: 0.0}))

    # 3. RT + D-pad Left (axis 6 = 1.0)
    msg_left = create_joy_msg(axes={5: -1.0, 6: 1.0})
    node.joy_callback(msg_left)
    assert len(step_msgs) == 3
    assert json.loads(step_msgs[-1].data)["action"] == "step_param_prev"

    # Return to neutral
    node.joy_callback(create_joy_msg(axes={5: -1.0, 6: 0.0}))

    # 4. RT + D-pad Right (axis 6 = -1.0)
    msg_right = create_joy_msg(axes={5: -1.0, 6: -1.0})
    node.joy_callback(msg_right)
    assert len(step_msgs) == 4
    assert json.loads(step_msgs[-1].data)["action"] == "step_param_next"
    node.destroy_node()

