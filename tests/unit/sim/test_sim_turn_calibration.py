"""Замер верности привода в Gazebo (rob_box_sim.turn_calibration, #4495) и колея сима.

Чистые функции без ROS: накопитель курса, вердикты разворота и прямой. И то, что
src/rob_box_sim/config/sim_drive.yaml правит только колею: остальное — из боевого
controller_manager.yaml, калибровка пола там не трогается.
"""

from __future__ import annotations

import math

import pytest
import yaml

from conftest import REPO_ROOT
from rob_box_sim import turn_calibration as tc

SIM_DRIVE = REPO_ROOT / "src" / "rob_box_sim" / "config" / "sim_drive.yaml"


def _accumulate(yaws):
    acc = tc.YawAccumulator()
    for y in yaws:
        acc.add(y)
    return acc.total


def test_accumulator_unwraps_full_turn_left():
    steps = [tc.wrap_pi(i * 0.1) for i in range(64)]  # 6.3 рад по шагу 0.1
    assert _accumulate(steps) == pytest.approx(6.3, abs=1e-9)


def test_accumulator_unwraps_two_turns_right():
    steps = [tc.wrap_pi(-i * 0.2) for i in range(64)]
    assert _accumulate(steps) == pytest.approx(-12.6, abs=1e-9)


def test_turn_verdict_ok_within_5_percent():
    v = tc.turn_verdict(0.6, odom_rad=2 * math.pi, truth_rad=2 * math.pi * 1.04)
    assert v.ok and v.ratio == pytest.approx(1.04)


def test_turn_verdict_fails_on_sim_before_fix():
    """Замер 06.11 до правки: одометрия 360°, правда 577° — в 1.6 раза больше."""
    v = tc.turn_verdict(0.3, odom_rad=math.radians(360.6), truth_rad=math.radians(577.3))
    assert not v.ok and v.ratio == pytest.approx(1.6, abs=0.01)


def test_turn_verdict_sign_does_not_matter():
    v = tc.turn_verdict(-1.0, odom_rad=-2 * math.pi, truth_rad=-2 * math.pi * 0.97)
    assert v.ok and v.truth_err == pytest.approx(-0.03)


def test_turn_verdict_fails_when_odom_overshoots():
    v = tc.turn_verdict(1.0, odom_rad=2 * math.pi * 1.06, truth_rad=2 * math.pi)
    assert not v.ok


def test_line_verdict_tolerance_is_3_percent():
    assert tc.line_verdict(0.3, odom_m=1.0, truth_m=1.029).ok
    assert not tc.line_verdict(0.3, odom_m=1.0, truth_m=1.031).ok


def test_format_row_shows_degrees_and_status():
    row = tc.format_row(tc.turn_verdict(0.3, math.pi * 2, math.pi * 2))
    assert "+360.00°" in row and row.endswith("OK")


def test_sim_drive_overrides_only_wheel_separation():
    sim = yaml.safe_load(SIM_DRIVE.read_text(encoding="utf-8"))
    assert sim == {"diff_drive_controller": {"ros__parameters": {"wheel_separation": pytest.approx(0.540)}}}


# ── Центр вращения и ошибка одометрии по позиции (#4377, #4475) ──────────────


def _spin_about(cx_body: float, cy_body: float, turn: float = 2 * math.pi, n: int = 120):
    """Правда разворота вокруг точки корпуса (cx, cy): base_footprint стартует в (1, 2), курс 0.3."""
    x0, y0, yaw0 = 1.0, 2.0, 0.3
    # центр в мире неподвижен
    wx = x0 + math.cos(yaw0) * cx_body - math.sin(yaw0) * cy_body
    wy = y0 + math.sin(yaw0) * cx_body + math.cos(yaw0) * cy_body
    poses = []
    for k in range(n + 1):
        yaw = yaw0 + turn * k / n
        x = wx - (math.cos(yaw) * cx_body - math.sin(yaw) * cy_body)
        y = wy - (math.sin(yaw) * cx_body + math.cos(yaw) * cy_body)
        poses.append((x, y, tc.wrap_pi(yaw)))
    return poses


def test_rotation_center_rear_axle_as_measured_in_hil():
    """HIL 09.11: сим-шасси крутится вокруг задней оси (x −0.144), а не вокруг base_footprint."""
    c = tc.rotation_center(_spin_about(-0.144, 0.0))
    assert c is not None
    assert c.x == pytest.approx(-0.144, abs=1e-6) and c.y == pytest.approx(0.0, abs=1e-6)
    assert c.spread < 1e-6 and c.pairs > 50


def test_rotation_center_on_base_footprint_is_zero():
    c = tc.rotation_center(_spin_about(0.0, 0.0, turn=-2 * math.pi))
    assert c is not None and math.hypot(c.x, c.y) < 1e-6


def test_rotation_center_needs_enough_turn():
    assert tc.rotation_center(_spin_about(-0.1, 0.0, turn=0.3)) is None
    assert tc.rotation_center([]) is None


def test_odom_position_error_open_loop_misses_pivot_translation():
    """open_loop одометрия стоит на месте, а base_footprint ходит по кругу радиуса 0.144:
    наибольшее расхождение — диаметр 0.288 м (замер HIL 09.11 — 0.287 м)."""
    truth = _spin_about(-0.144, 0.0)
    odom = [(0.0, 0.0, tc.wrap_pi(t[2] - 0.3)) for t in truth]
    assert tc.odom_position_error(list(zip(truth, odom))) == pytest.approx(0.288, abs=1e-3)


def test_odom_position_error_zero_when_odom_follows_truth_in_other_frame():
    truth = _spin_about(-0.144, 0.0)
    # одометрия = та же траектория в системе, повёрнутой и сдвинутой относительно мира
    odom = []
    for x, y, yaw in truth:
        dx, dy = x - 1.0, y - 2.0
        c, s = math.cos(-0.3), math.sin(-0.3)
        odom.append((c * dx - s * dy, s * dx + c * dy, tc.wrap_pi(yaw - 0.3)))
    assert tc.odom_position_error(list(zip(truth, odom))) < 1e-9


def test_turn_row_reports_center_when_known():
    v = tc.turn_verdict(0.6, odom_rad=2 * math.pi, truth_rad=2 * math.pi)
    assert "центр" not in tc.format_row(v)
    from dataclasses import replace

    row = tc.format_row(replace(v, center_x=-0.144, center_y=0.0, center_spread=0.001, odom_pos_err=0.287))
    assert "(-0.144, +0.000)" in row and "0.287 м" in row
