"""Pure-logic tests for rob_box_teleop.joystick_logic (no ROS required)."""

import pytest

from rob_box_teleop.joystick_logic import (
    apply_deadzone,
    compute_axes,
    compute_buttons,
    compute_twist,
    is_armed,
    normalize_sbus_channel,
)


# ─────────────────────────────────────────────────────────────────────────────
# normalize_sbus_channel
# ─────────────────────────────────────────────────────────────────────────────

def test_normalize_sbus_center_is_zero():
    assert normalize_sbus_channel(992) == pytest.approx(0.0, abs=1e-9)


def test_normalize_sbus_max_is_one():
    # SBUS range is asymmetric: (1811-992)=819 vs (992-172)=820, so the
    # raw max maps to ~0.9994 (not clamped to 1.0). This matches the
    # incident log where full gas produced axes[1]=0.9994.
    assert normalize_sbus_channel(1811) == pytest.approx(0.9994, abs=1e-3)


def test_normalize_sbus_min_is_minus_one():
    assert normalize_sbus_channel(172) == pytest.approx(-1.0, abs=1e-6)


def test_normalize_sbus_clamps_out_of_range():
    assert normalize_sbus_channel(2000) == 1.0
    assert normalize_sbus_channel(0) == -1.0


# ─────────────────────────────────────────────────────────────────────────────
# compute_axes — the actual incident packets from issue #1344
# ─────────────────────────────────────────────────────────────────────────────

def test_compute_axes_incident_gas_packet():
    # SBUS #302800 from the 16.08 incident: Ch1=1048 Ch2=1811 Ch3=1012 Ch4=992 ARM=1792
    channels = [1048, 1811, 1012, 992, 1792] + [992] * 11
    axes = compute_axes(channels)
    assert axes[1] == pytest.approx(0.9994, abs=1e-3)  # gas → full forward
    assert axes[3] == pytest.approx(0.0, abs=1e-3)     # yaw neutral
    assert axes[4] == pytest.approx(0.9762, abs=1e-3)  # ARM high


def test_compute_axes_incident_yaw_hard_right():
    # Ch4=178 → yaw full right (negative in ROS convention)
    channels = [992, 992, 1007, 178, 1792] + [992] * 11
    axes = compute_axes(channels)
    assert axes[3] == pytest.approx(-0.9933, abs=1e-3)


# ─────────────────────────────────────────────────────────────────────────────
# compute_buttons / is_armed
# ─────────────────────────────────────────────────────────────────────────────

def test_compute_buttons_threshold():
    channels = [992, 1811, 191, 1792] + [992] * 12
    buttons = compute_buttons(channels)
    assert buttons[0] == 0  # 992 ≤ 1500
    assert buttons[1] == 1  # 1811 > 1500
    assert buttons[2] == 0  # 191 ≤ 1500
    assert buttons[3] == 1  # 1792 > 1500


def test_is_armed_true_when_arm_high():
    buttons = [0] * 16
    buttons[4] = 1
    assert is_armed(buttons, 4) is True


def test_is_armed_false_when_arm_low():
    buttons = [0] * 16
    assert is_armed(buttons, 4) is False


def test_is_armed_false_when_buttons_short():
    assert is_armed([0, 0], 4) is False


def test_is_armed_false_when_channel_invalid():
    buttons = [1] * 16
    assert is_armed(buttons, -1) is False
    assert is_armed(buttons, 99) is False


# ─────────────────────────────────────────────────────────────────────────────
# apply_deadzone
# ─────────────────────────────────────────────────────────────────────────────

def test_apply_deadzone_zero():
    assert apply_deadzone(0.0, 0.1) == 0.0


def test_apply_deadzone_inside():
    assert apply_deadzone(0.05, 0.1) == 0.0
    assert apply_deadzone(-0.05, 0.1) == 0.0


def test_apply_deadzone_full_scale():
    assert apply_deadzone(0.9994, 0.1) == pytest.approx(0.9993, abs=1e-3)
    assert apply_deadzone(-0.9933, 0.1) == pytest.approx(-0.9925, abs=1e-3)


def test_apply_deadzone_edge():
    assert apply_deadzone(1.0, 0.1) == pytest.approx(1.0)
    assert apply_deadzone(-1.0, 0.1) == pytest.approx(-1.0)


# ─────────────────────────────────────────────────────────────────────────────
# compute_twist — issue #1344 mapping
# ─────────────────────────────────────────────────────────────────────────────

def test_compute_twist_gas_packet():
    # Ch2=1811 (gas full), Ch4=992 (yaw neutral), max_linear=1.0, max_angular=1.0
    axes = compute_axes([1048, 1811, 1012, 992, 1792] + [992] * 11)
    linear_x, angular_z = compute_twist(axes, ch_pitch=1, ch_yaw=3,
                                        max_linear=1.0, max_angular=1.0,
                                        deadzone=0.1)
    assert linear_x == pytest.approx(0.9993, abs=1e-3)
    assert angular_z == pytest.approx(0.0, abs=1e-3)


def test_compute_twist_yaw_hard_right():
    # Ch4=178 (yaw full right), no gas
    axes = compute_axes([992, 992, 1007, 178, 1792] + [992] * 11)
    linear_x, angular_z = compute_twist(axes, ch_pitch=1, ch_yaw=3,
                                        max_linear=1.0, max_angular=1.0,
                                        deadzone=0.1)
    assert linear_x == pytest.approx(0.0, abs=1e-3)
    # full right yaw → angular.z ≈ -0.99 (rotation right)
    assert angular_z == pytest.approx(-0.9925, abs=1e-3)


def test_compute_twist_neutral_is_zero():
    axes = compute_axes([992] * 16)
    linear_x, angular_z = compute_twist(axes, ch_pitch=1, ch_yaw=3,
                                        max_linear=1.0, max_angular=1.0,
                                        deadzone=0.1)
    assert linear_x == 0.0
    assert angular_z == 0.0


def test_compute_twist_short_axes_does_not_crash():
    linear_x, angular_z = compute_twist([], ch_pitch=1, ch_yaw=3,
                                        max_linear=1.0, max_angular=1.0,
                                        deadzone=0.1)
    assert linear_x == 0.0
    assert angular_z == 0.0
