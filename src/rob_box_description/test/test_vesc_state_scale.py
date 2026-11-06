"""#4360: state колёс поправлен через gear_ratio, команда duty не изменилась.

Формулы повторяют ``src/vesc_nexus/src/vesc_nexus/src/vesc_handler.cpp``:

* state (``updateMotion``, стр. 117-118):
  ``rad/s = erpm / pole_pairs * 2π / 60 / gear_ratio``, ``pole_pairs = max(1, poles // 2)``;
  position — интеграл этой скорости (стр. 123);
* команда duty (``motorRpsFor``/``dutyFor``, стр. 60-68):
  ``duty = clamp(wheel_rad_s / 2π * gear_ratio / max_rps, ±1)``, затем мёртвая зона
  ``min_duty``; кадр CAN — ``int32(duty * 100000)`` (``message_translator.cpp:37``).

Duty зависит только от отношения ``gear_ratio / max_rps``: оба умножены на 1.326,
отношение то же. Параметры читаются из ``urdf/rob_box_ros2_control.xacro``
(боевой URDF; эталон — ``golden/rob_box_real.urdf``).
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1]
XACRO = PKG / "urdf" / "rob_box_ros2_control.xacro"
GOLDEN = PKG / "test" / "golden" / "rob_box_real.urdf"

# Конфиг до #4360.
OLD_GEAR = 2.17
OLD_MAX_RPS = 12.2
SCALE = 1.326  # драйвер/тахометр, 04.11
DUTY_LIMIT = 1.0  # message_translator.hpp: kDutyLimit
WHEEL_RADIUS = 0.1143

# (команда м/с, драйвер об/мин при старом конфиге, тахометр об/мин) — замер 04.11,
# робот на кирпичах, right_front_wheel_joint.
MEASURED = [
    (0.2, 22.8, 17.3),
    (0.6, 45.6, 34.4),
    (1.0, 68.3, 51.5),
]


def _params(path: Path) -> tuple[dict[str, str], dict[str, dict[str, str]]]:
    root = ET.parse(path).getroot()
    for rc in root.iter("ros2_control"):
        plugin = rc.find("hardware/plugin")
        if plugin is not None and "vesc_nexus" in (plugin.text or ""):
            hw = {p.get("name"): p.text for p in rc.find("hardware").findall("param")}
            joints = {j.get("name"): {p.get("name"): p.text for p in j.findall("param")}
                      for j in rc.findall("joint")}
            return hw, joints
    raise AssertionError(f"нет ros2_control с vesc_nexus в {path}")


HW, JOINTS = _params(XACRO)
GEAR = float(HW["gear_ratio"])
MIN_DUTY = float(HW["min_duty"])


def state_rpm(erpm: float, poles: int, gear: float) -> float:
    """updateMotion: eRPM → об/мин колеса."""
    rad_s = erpm / max(1, poles // 2) * 2 * math.pi / 60.0 / gear
    return rad_s * 60.0 / (2 * math.pi)


def erpm_from_state_rpm(rpm: float, poles: int, gear: float) -> float:
    return rpm * gear * max(1, poles // 2)


def duty_for(wheel_rad_s: float, gear: float, max_rps: float, min_duty: float) -> float:
    """dutyFor в том же порядке операций, что в C++."""
    motor_rps = wheel_rad_s / (2 * math.pi) * gear
    duty = max(-DUTY_LIMIT, min(DUTY_LIMIT, motor_rps / max_rps))
    if min_duty > 0.0 and duty != 0.0:
        duty = math.copysign(min_duty + abs(duty) * (1.0 - min_duty), duty)
    return duty


def can_duty(duty: float) -> int:
    """createSetDutyCycleFrame: static_cast<int32_t>(duty * 100000)."""
    return int(max(-DUTY_LIMIT, min(DUTY_LIMIT, duty)) * 100000.0)


def test_config_duty_mode_poles_untouched():
    assert HW["control_mode"] == "duty"
    assert len(JOINTS) == 4
    for name, jp in JOINTS.items():
        assert int(jp["poles"]) == 30, name


def test_golden_urdf_has_same_vesc_params_as_xacro():
    assert _params(GOLDEN) == (HW, JOINTS)


def test_gear_and_max_rps_scaled_by_same_factor():
    assert GEAR == pytest.approx(OLD_GEAR * SCALE, rel=1e-12)
    assert MIN_DUTY == pytest.approx(0.05)
    for name, jp in JOINTS.items():
        max_rps = float(jp["max_rps"])
        assert max_rps == pytest.approx(OLD_MAX_RPS * SCALE, rel=1e-12), name
        assert GEAR / max_rps == pytest.approx(OLD_GEAR / OLD_MAX_RPS, rel=1e-12), name


def test_can_duty_frame_identical_for_old_and_new_config():
    """Кадр SET_DUTY для каждой команды от −3 до +3 м/с (шаг 1 мм/с) тот же, что до правки."""
    mismatches = []
    for jname, jp in JOINTS.items():
        new_max_rps = float(jp["max_rps"])
        for mm_s in range(-3000, 3001):
            w = mm_s / 1000.0 / WHEEL_RADIUS
            old = can_duty(duty_for(w, OLD_GEAR, OLD_MAX_RPS, MIN_DUTY))
            new = can_duty(duty_for(w, GEAR, new_max_rps, MIN_DUTY))
            if old != new:
                mismatches.append((jname, mm_s, old, new))
    assert mismatches == []


def test_duty_values_for_report():
    for v, expected in [(0.2, 0.0970573), (0.6, 0.1911720), (1.0, 0.2852867)]:
        w = v / WHEEL_RADIUS
        assert duty_for(w, GEAR, float(JOINTS["right_front_wheel_joint"]["max_rps"]), MIN_DUTY) == \
            pytest.approx(expected, abs=1e-6)


def test_state_scale_is_one_over_1_326():
    poles = int(JOINTS["right_front_wheel_joint"]["poles"])
    for erpm in (100.0, 1484.3, 5000.0):
        assert state_rpm(erpm, poles, GEAR) / state_rpm(erpm, poles, OLD_GEAR) == pytest.approx(1 / SCALE)


@pytest.mark.parametrize("cmd_m_s,driver_rpm_old,tacho_rpm", MEASURED)
def test_measured_points_match_tachometer_after_fix(cmd_m_s, driver_rpm_old, tacho_rpm):
    poles = int(JOINTS["right_front_wheel_joint"]["poles"])
    erpm = erpm_from_state_rpm(driver_rpm_old, poles, OLD_GEAR)  # что прислал VESC
    new_rpm = state_rpm(erpm, poles, GEAR)
    # До правки драйвер завышал на 32–33 %; после — расхождение с тахометром < 1 %.
    assert abs(new_rpm / tacho_rpm - 1.0) < 0.01, (cmd_m_s, new_rpm, tacho_rpm)


def test_point_0_6_m_s_gives_about_34_rpm():
    erpm = erpm_from_state_rpm(45.6, 30, OLD_GEAR)
    assert state_rpm(erpm, 30, GEAR) == pytest.approx(34.39, abs=0.01)
