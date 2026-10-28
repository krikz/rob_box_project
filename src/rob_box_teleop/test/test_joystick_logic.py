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


# ─────────────────────────────────────────────────────────────────────────────
# ArmAnnouncer (issue #4003): chatter of the ARM channel must not be spoken
# ─────────────────────────────────────────────────────────────────────────────

from rob_box_teleop.joystick_logic import (  # noqa: E402
    ARM_ANNOUNCE_HOLD_S,
    ARM_ANNOUNCE_MIN_INTERVAL_S,
    ArmAnnouncer,
)


def _run(announcer, samples, start=0.0, step=0.05):
    """Feed ``(armed, duration_s)`` segments as samples every ``step`` s; return [(t, announced)]."""
    now = start
    heard = []
    for armed, duration in samples:
        for _ in range(max(1, round(duration / step))):
            said = announcer.update(armed, now)
            if said is not None:
                heard.append((round(now, 3), said))
            now += step
    return heard


def test_defaults_cover_the_observed_chatter():
    # robot log 2026-10-24: LOW glitches lasted 48..100 ms
    assert ARM_ANNOUNCE_HOLD_S >= 0.1 * 2
    assert ARM_ANNOUNCE_MIN_INTERVAL_S >= 4.0  # one phrase = ~4 s of tts_node queue


def test_boot_with_switch_down_says_nothing():
    assert _run(ArmAnnouncer(), [(False, 10.0)]) == []


def test_real_arm_is_announced_once_after_the_hold():
    heard = _run(ArmAnnouncer(hold_s=0.3), [(True, 5.0)])
    assert [state for _, state in heard] == [True]
    assert heard[0][0] == pytest.approx(0.3, abs=0.051)


def test_real_arm_then_real_disarm_one_announcement_each():
    heard = _run(ArmAnnouncer(), [(True, 3.0), (False, 8.0)])
    assert [state for _, state in heard] == [True, False]


def test_chatter_10_glitches_of_50ms_after_arming_is_silent():
    announcer = ArmAnnouncer()
    heard = _run(announcer, [(True, 2.0)] + [(False, 0.05), (True, 1.0)] * 10)
    assert [state for _, state in heard] == [True]  # only the real arming
    assert announcer.stable is True


def test_chatter_10_glitches_of_50ms_while_disarmed_is_silent():
    heard = _run(ArmAnnouncer(), [(True, 0.05), (False, 1.0)] * 10)
    assert heard == []


def test_continuous_50ms_flapping_is_silent():
    heard = _run(ArmAnnouncer(), [(True, 0.05), (False, 0.05)] * 50)
    assert heard == []


def test_glitch_of_100ms_the_longest_observed_is_silent():
    heard = _run(ArmAnnouncer(), [(True, 2.0), (False, 0.1), (True, 2.0)])
    assert [state for _, state in heard] == [True]


def test_fast_on_off_on_gives_only_the_final_state():
    # ON settles and is spoken; OFF settles but waits for the interval; ON again
    # before the interval ends -> the stable state equals the announced one -> silent.
    announcer = ArmAnnouncer(hold_s=0.3, min_interval_s=5.0)
    heard = _run(announcer, [(True, 0.5), (False, 0.5), (True, 3.0)])
    assert [state for _, state in heard] == [True]
    assert announcer.stable is True


def test_fast_on_off_on_before_the_first_one_settles_is_one_announcement():
    heard = _run(ArmAnnouncer(hold_s=0.3), [(True, 0.2), (False, 0.2), (True, 2.0)])
    assert [state for _, state in heard] == [True]


def test_on_off_that_ends_disarmed_is_announced_with_the_final_state_after_the_interval():
    announcer = ArmAnnouncer(hold_s=0.3, min_interval_s=5.0)
    heard = _run(announcer, [(True, 0.5), (False, 10.0)])
    assert [state for _, state in heard] == [True, False]
    # the OFF phrase is held back until 5 s after the ON phrase
    assert heard[1][0] >= heard[0][0] + 5.0 - 1e-9


def test_announcements_are_never_closer_than_the_interval():
    # a person really flips the switch every 1 s for a minute
    announcer = ArmAnnouncer(hold_s=0.3, min_interval_s=5.0)
    heard = _run(announcer, [(i % 2 == 0, 1.0) for i in range(60)])
    times = [t for t, _ in heard]
    assert len(times) >= 2
    assert all(b - a >= 5.0 - 1e-9 for a, b in zip(times, times[1:]))
    # nothing queues up: at most one phrase per interval
    assert len(times) <= 60 / 5.0 + 1


def test_zero_hold_and_interval_degrade_to_every_change_and_negative_is_clamped():
    announcer = ArmAnnouncer(hold_s=-1.0, min_interval_s=-1.0)
    assert (announcer.hold_s, announcer.min_interval_s) == (0.0, 0.0)
    heard = _run(announcer, [(True, 0.05), (False, 0.05), (True, 0.05)])
    assert [state for _, state in heard] == [True, False, True]
