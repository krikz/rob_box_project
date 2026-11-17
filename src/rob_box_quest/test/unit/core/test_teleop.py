"""Unit-тесты TeleopController (без rclpy)."""

from rob_box_quest.core.teleop import (
    DEADMAN_TIMEOUT_S,
    MAX_ANGULAR_RAD_S,
    MAX_LINEAR_M_S,
    TeleopController,
)


def test_consume_with_deadman_returns_twist():
    c = TeleopController()
    twist = c.consume(linear=0.5, angular=0.2, deadman=True, now_monotonic=100.0)
    assert twist is not None
    assert twist.linear_x == 0.5
    assert twist.angular_z == 0.2


def test_consume_without_deadman_returns_none_and_clears_last():
    c = TeleopController()
    c.consume(linear=0.5, angular=0.0, deadman=True, now_monotonic=100.0)
    twist = c.consume(linear=0.5, angular=0.0, deadman=False, now_monotonic=100.1)
    assert twist is None
    # Tick сразу после release → тоже None (last_input сброшен).
    assert c.tick(now_monotonic=100.2) is None


def test_tick_repeats_twist_within_deadman_window():
    c = TeleopController()
    c.consume(linear=0.5, angular=0.1, deadman=True, now_monotonic=100.0)
    # В пределах окна — повторяем.
    twist = c.tick(now_monotonic=100.0 + DEADMAN_TIMEOUT_S - 0.05)
    assert twist is not None
    assert twist.linear_x == 0.5
    assert twist.angular_z == 0.1


def test_tick_returns_none_after_deadman_timeout():
    c = TeleopController()
    c.consume(linear=0.5, angular=0.1, deadman=True, now_monotonic=100.0)
    # За пределами окна — None.
    assert c.tick(now_monotonic=100.0 + DEADMAN_TIMEOUT_S + 0.01) is None


def test_emergency_stop_locks_consume_and_tick():
    c = TeleopController()
    c.consume(linear=0.5, angular=0.1, deadman=True, now_monotonic=100.0)
    c.emergency_stop()
    # consume → None
    assert c.consume(linear=0.5, angular=0.1, deadman=True, now_monotonic=101.0) is None
    # tick → None
    assert c.tick(now_monotonic=101.0) is None
    assert c.is_emergency is True


def test_reset_after_emergency_allows_normal_flow():
    c = TeleopController()
    c.consume(linear=0.5, angular=0.1, deadman=True, now_monotonic=100.0)
    c.emergency_stop()
    c.reset()
    assert c.is_emergency is False
    twist = c.consume(linear=0.2, angular=0.0, deadman=True, now_monotonic=102.0)
    assert twist is not None
    assert twist.linear_x == 0.2


def test_linear_clamped_to_max():
    c = TeleopController()
    twist = c.consume(linear=10.0, angular=0.0, deadman=True, now_monotonic=0.0)
    assert twist.linear_x == MAX_LINEAR_M_S


def test_linear_clamped_to_min():
    c = TeleopController()
    twist = c.consume(linear=-10.0, angular=0.0, deadman=True, now_monotonic=0.0)
    assert twist.linear_x == -MAX_LINEAR_M_S


def test_angular_clamped_to_max():
    c = TeleopController()
    twist = c.consume(linear=0.0, angular=10.0, deadman=True, now_monotonic=0.0)
    assert twist.angular_z == MAX_ANGULAR_RAD_S


def test_angular_clamped_to_min():
    c = TeleopController()
    twist = c.consume(linear=0.0, angular=-10.0, deadman=True, now_monotonic=0.0)
    assert twist.angular_z == -MAX_ANGULAR_RAD_S


def test_last_twist_property_returns_copy():
    c = TeleopController()
    c.consume(linear=0.5, angular=0.1, deadman=True, now_monotonic=0.0)
    t = c.last_twist
    assert t.linear_x == 0.5
    assert t.angular_z == 0.1
    # Изменение копии не должно влиять на state.
    t.linear_x = 99.0
    assert c.last_twist.linear_x == 0.5


def test_constants_are_safe():
    """Guard: safety belt не уехал в небезопасное значение."""
    assert 0.1 <= DEADMAN_TIMEOUT_S <= 1.0
    assert 0.5 <= MAX_LINEAR_M_S <= 3.0
    assert 1.0 <= MAX_ANGULAR_RAD_S <= 5.0
