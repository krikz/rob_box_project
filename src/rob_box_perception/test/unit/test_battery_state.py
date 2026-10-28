"""Батарея из ros2_control: разбор DynamicJointState и поля BatteryState (#4001, #4002).

Чистая логика без ROS: ``rob_box_perception.battery_state``. Сообщения строятся
``SimpleNamespace`` по структуре ``control_msgs/DynamicJointState``: имя joint
в ``joint_names[i]``, а ``interface_values[i].interface_names`` хранит только имя
интерфейса (``voltage``).

Запуск:
    python -m pytest src/rob_box_perception/test/unit/test_battery_state.py -v --no-cov
"""

from __future__ import annotations

import importlib
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

_HERE = Path(__file__).resolve()
_PKG_ROOT = _HERE.parents[2]
_MCP_ROOT = _HERE.parents[3] / 'rob_box_mcp_tools'
for _root in (_PKG_ROOT, _MCP_ROOT):
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

bs = importlib.import_module('rob_box_perception.battery_state')


def _iv(names, values):
    return SimpleNamespace(interface_names=list(names), values=list(values))


def _wheel():
    return _iv(['position', 'velocity'], [1.0, 0.5])


def _battery(volts, interface='voltage'):
    return _iv([interface], [volts])


# ---------------------------------------------------------------------------
# extract_battery_voltage: сопоставление joint_names[i] <-> interface_values[i]
# ---------------------------------------------------------------------------

def test_extracts_voltage_from_battery_joint():
    joints = ['left_front_wheel_joint', 'battery']
    ivs = [_wheel(), _battery(41.0)]
    assert bs.extract_battery_voltage(joints, ivs) == 41.0


def test_joint_order_does_not_matter():
    joints = ['battery', 'left_front_wheel_joint', 'right_front_wheel_joint']
    ivs = [_battery(40.2), _wheel(), _wheel()]
    assert bs.extract_battery_voltage(joints, ivs) == pytest.approx(40.2)


def test_no_battery_joint_returns_none():
    joints = ['left_front_wheel_joint', 'right_front_wheel_joint']
    ivs = [_wheel(), _wheel()]
    assert bs.extract_battery_voltage(joints, ivs) is None


def test_voltage_of_other_joint_is_not_taken():
    """Интерфейс voltage у чужого joint - не напряжение батареи."""
    joints = ['left_front_wheel_joint', 'battery']
    ivs = [_battery(12.0), _iv(['temperature'], [30.0])]
    assert bs.extract_battery_voltage(joints, ivs) is None


def test_old_bug_string_is_not_in_interface_names():
    """Регресс #4001: строка 'battery/voltage' в interface_names не ищется и не нужна."""
    joints = ['battery']
    assert bs.extract_battery_voltage(joints, [_battery(41.0)]) == 41.0
    assert bs.extract_battery_voltage(joints, [_battery(41.0, 'battery/voltage')]) is None


def test_empty_message():
    assert bs.extract_battery_voltage([], []) is None


@pytest.mark.parametrize('bad', [0.0, -3.0, float('nan'), float('inf')])
def test_invalid_voltage_values_are_none(bad):
    assert bs.extract_battery_voltage(['battery'], [_battery(bad)]) is None


# ---------------------------------------------------------------------------
# voltage_to_fraction / battery_fields: шкала 0..1 и честное «нет данных»
# ---------------------------------------------------------------------------

def test_fraction_endpoints_and_midpoint():
    assert bs.voltage_to_fraction(42.0) == pytest.approx(1.0)
    assert bs.voltage_to_fraction(30.0) == pytest.approx(0.0)
    assert bs.voltage_to_fraction(36.0) == pytest.approx(0.5)


def test_fraction_card_example_41v():
    # Критерий приёмки #4002: 41.0 В -> ~0.92
    assert bs.voltage_to_fraction(41.0) == pytest.approx(11.0 / 12.0)


def test_fraction_is_clamped_to_unit_interval():
    assert bs.voltage_to_fraction(45.0) == 1.0
    assert bs.voltage_to_fraction(29.0) == 0.0


@pytest.mark.parametrize('bad', [None, 0.0, -1.0, float('nan'), 'abc'])
def test_fraction_unknown_is_none(bad):
    assert bs.voltage_to_fraction(bad) is None


def test_fields_fresh_voltage():
    f = bs.battery_fields(41.0, age_s=0.5)
    assert f.present is True
    assert f.voltage == 41.0
    assert 0.0 <= f.percentage <= 1.0
    assert f.percentage == pytest.approx(0.9167, abs=1e-3)


@pytest.mark.parametrize('volts, age', [
    (None, None),            # данных ещё не было
    (None, 0.1),
    (0.0, 0.1),              # ноль вольт - не «разряжена», а «нет данных»
    (float('nan'), 0.1),
    (41.0, None),            # возраст неизвестен - не верим
    (41.0, 5.01),            # устарело
])
def test_fields_without_fresh_voltage_are_not_present(volts, age):
    f = bs.battery_fields(volts, age)
    assert f.present is False
    assert math.isnan(f.voltage)
    assert math.isnan(f.percentage)


def test_fields_stale_boundary_and_custom_timeout():
    assert bs.battery_fields(41.0, 5.0).present is True
    assert bs.battery_fields(41.0, 5.01).present is False
    assert bs.battery_fields(41.0, 10.0, stale_s=15.0).present is True


def test_no_data_is_never_full_battery():
    f = bs.battery_fields(None, None)
    assert not (f.percentage == 1.0)
    assert f.present is False


# ---------------------------------------------------------------------------
# Паритет с rob_box_mcp_tools.perception_projection.voltage_to_percent
# ---------------------------------------------------------------------------

def _projection():
    try:
        return importlib.import_module('rob_box_mcp_tools.perception_projection')
    except ImportError as exc:  # pragma: no cover
        pytest.fail(f'rob_box_mcp_tools.perception_projection недоступен: {exc}')


def test_constants_match_perception_projection():
    proj = _projection()
    assert bs.BATTERY_VOLTAGE_FULL == proj.BATTERY_VOLTAGE_FULL
    assert bs.BATTERY_VOLTAGE_EMPTY == proj.BATTERY_VOLTAGE_EMPTY


@pytest.mark.parametrize('volts', [0.0, -1.0, 29.0, 30.0, 33.3, 36.0, 40.0, 41.0, 42.0, 45.0])
def test_fraction_matches_voltage_to_percent_over_100(volts):
    proj = _projection()
    pct = proj.voltage_to_percent(volts)
    frac = bs.voltage_to_fraction(volts)
    if pct is None:
        assert frac is None
    else:
        assert frac == pytest.approx(pct / 100.0)
