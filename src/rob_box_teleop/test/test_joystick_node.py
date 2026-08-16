"""Node behavior tests for joystick_control_node with mocked rclpy.

Covers the issue #1344 fixes:
- /joystick_lock published True while armed (blocks web/voice/nav2 in twist_mux)
- /joystick_lock published False while disarmed (lower sources work)
- cmd_vel_joy mapping (gas/yaw from the incident packets)
- single stop message on armed→disarmed transition
"""

import pytest

from rob_box_teleop.joystick_control_node import JoystickControlNode


def _msg(axes, buttons):
    joy = type("Joy", (), {})()
    joy.axes = axes
    joy.buttons = buttons
    return joy


def _axes(ch2=992, ch4=992, ch5=191):
    """16 SBUS-normalized axes; gas on Ch2 (index 1), yaw on Ch4 (index 3), ARM Ch5 (index 4)."""
    axes = [0.0] * 16
    axes[1] = (ch2 - 992.0) / 819.5
    axes[3] = (ch4 - 992.0) / 819.5
    axes[4] = (ch5 - 992.0) / 819.5
    return axes


def _buttons(ch5=191):
    buttons = [0] * 16
    buttons[4] = 1 if ch5 > 1500 else 0
    return buttons


def test_cmd_vel_joy_published_with_gas(node_params):
    node = JoystickControlNode()
    node.max_angular = 1.0
    # armed + full gas (Ch2=1811), yaw neutral (Ch4=992)
    node.joy_callback_sbus(_msg(_axes(ch2=1811, ch4=992, ch5=1792), _buttons(ch5=1792)))
    cmd_topic, cmd_pub = [p for p in node.publishers if p[0] == "cmd_vel_joy"][0]
    assert cmd_pub.messages, "cmd_vel_joy must be published while armed"
    twist = cmd_pub.messages[-1]
    assert twist.linear.x == pytest.approx(0.9993, abs=1e-3)
    assert twist.angular.z == pytest.approx(0.0, abs=1e-3)


def test_cmd_vel_joy_not_published_while_disarmed(node_params):
    node = JoystickControlNode()
    # disarmed → no cmd_vel_joy at all (twist_mux falls through by timeout)
    node.joy_callback_sbus(_msg(_axes(ch5=191), _buttons(ch5=191)))
    cmd_topic, cmd_pub = [p for p in node.publishers if p[0] == "cmd_vel_joy"][0]
    assert cmd_pub.messages == []


def test_stop_published_once_on_disarm_transition(node_params):
    node = JoystickControlNode()
    node.max_angular = 1.0
    # armed first (publishes normal commands)
    node.joy_callback_sbus(_msg(_axes(ch2=1811, ch4=992, ch5=1792), _buttons(ch5=1792)))
    # now disarm → exactly one zero-twist stop message
    node.joy_callback_sbus(_msg(_axes(ch5=191), _buttons(ch5=191)))
    cmd_topic, cmd_pub = [p for p in node.publishers if p[0] == "cmd_vel_joy"][0]
    stop_msgs = [m for m in cmd_pub.messages if m.linear.x == 0.0 and m.angular.z == 0.0]
    assert len(stop_msgs) == 1


def test_yaw_mapping_matches_incident(node_params):
    node = JoystickControlNode()
    node.max_angular = 1.0
    # incident packet 1786876408: Ch4=178 (yaw hard right) → angular.z ≈ -0.99
    node.joy_callback_sbus(_msg(_axes(ch2=992, ch4=178, ch5=1792), _buttons(ch5=1792)))
    cmd_topic, cmd_pub = [p for p in node.publishers if p[0] == "cmd_vel_joy"][0]
    twist = cmd_pub.messages[-1]
    assert twist.angular.z == pytest.approx(-0.9925, abs=1e-3)


# ─────────────────────────────────────────────────────────────────────────────
# SBUS timer path (publish_joy_from_sbus): lock + heartbeat diagnostics
# ─────────────────────────────────────────────────────────────────────────────

def _sbus_node(node_params):
    node = JoystickControlNode()
    node.device_connected = True
    node.max_angular = 1.0
    return node


def test_sbus_timer_armed_publishes_lock_and_cmd_vel(node_params):
    node = _sbus_node(node_params)
    # armed + full gas
    node.sbus_channels = [992] * 16
    node.sbus_channels[1] = 1811  # Ch2 gas
    node.sbus_channels[4] = 1792  # Ch5 ARM
    node.publish_joy_from_sbus()

    lock_topic, lock_pub = [p for p in node.publishers if p[0] == "/joystick_lock"][0]
    assert lock_pub.messages and lock_pub.messages[-1].data is True

    cmd_topic, cmd_pub = [p for p in node.publishers if p[0] == "cmd_vel_joy"][0]
    assert cmd_pub.messages
    assert cmd_pub.messages[-1].linear.x == pytest.approx(0.9993, abs=1e-3)


def test_sbus_timer_disarmed_publishes_lock_false_only(node_params):
    node = _sbus_node(node_params)
    # disarmed, neutral
    node.sbus_channels = [992] * 16
    node.sbus_channels[4] = 191  # Ch5 ARM low
    node.publish_joy_from_sbus()

    lock_topic, lock_pub = [p for p in node.publishers if p[0] == "/joystick_lock"][0]
    assert lock_pub.messages and lock_pub.messages[-1].data is False

    cmd_topic, cmd_pub = [p for p in node.publishers if p[0] == "cmd_vel_joy"][0]
    assert cmd_pub.messages == []


def test_sbus_timer_armed_logs_heartbeat(node_params):
    node = _sbus_node(node_params)
    node.sbus_channels = [992] * 16
    node.sbus_channels[4] = 1792  # armed
    node._last_cmd_log_time = 0.0
    node.publish_joy_from_sbus()

    logged = [str(c.args[0]) for c in node.get_logger().info.call_args_list]
    assert any("cmd_vel_joy" in line for line in logged), logged


def test_sbus_timer_disarmed_no_heartbeat(node_params):
    node = _sbus_node(node_params)
    node.sbus_channels = [992] * 16
    node.sbus_channels[4] = 191  # disarmed
    node._last_cmd_log_time = 0.0
    node.publish_joy_from_sbus()

    logged = [str(c.args[0]) for c in node.get_logger().info.call_args_list]
    assert not any("cmd_vel_joy" in line for line in logged), logged
