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
