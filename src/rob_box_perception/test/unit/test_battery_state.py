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
_CORE_ROOT = _HERE.parents[3] / 'rob_box_core'
for _root in (_PKG_ROOT, _CORE_ROOT):
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
# battery_fields: кривая берётся из rob_box_core.battery (одно место знания)
# ---------------------------------------------------------------------------

def test_curve_is_imported_from_core_not_duplicated():
    core = importlib.import_module('rob_box_core.battery')
    assert bs.voltage_to_fraction is core.voltage_to_fraction
    assert bs.valid_voltage is core.valid_voltage
    for dup in ('BATTERY_VOLTAGE_FULL', 'BATTERY_VOLTAGE_EMPTY', 'OCV_TABLE'):
        assert not hasattr(bs, dup), dup


def test_fields_fresh_voltage_card_example_41v():
    f = bs.battery_fields(41.0, age_s=0.5)
    assert f.present is True
    assert f.voltage == 41.0
    assert f.percentage == pytest.approx(0.90)  # кривая Li-ion 10S: 4.10 В/банку = 90 %


def test_fields_percentage_from_smoothed_voltage_voltage_stays_raw():
    f = bs.battery_fields(38.0, 0.1, smoothed=41.0)
    assert f.voltage == 38.0
    assert f.percentage == pytest.approx(0.90)


def test_fields_invalid_smoothed_falls_back_to_raw():
    assert bs.battery_fields(41.0, 0.1, smoothed=float('nan')).percentage == pytest.approx(0.90)
    assert bs.battery_fields(41.0, 0.1, smoothed=None).percentage == pytest.approx(0.90)


@pytest.mark.parametrize('volts, age', [
    (None, None),            # данных ещё не было
    (None, 0.1),
    (0.0, 0.1),              # ноль вольт - не «разряжена», а «нет данных»
    (float('nan'), 0.1),
    (41.0, None),            # возраст неизвестен - не верим
    (41.0, 5.01),            # устарело
])
def test_fields_without_fresh_voltage_are_not_present(volts, age):
    f = bs.battery_fields(volts, age, smoothed=41.0)
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
# Сглаживание напряжения (EMA): провалы под нагрузкой не прыгают процентом
# ---------------------------------------------------------------------------

def test_ema_first_sample_has_no_delay():
    assert bs.ema_step(None, 41.0, dt_s=1.0, tau_s=45.0) == 41.0


def test_ema_disabled_when_tau_zero():
    assert bs.ema_step(41.0, 38.0, dt_s=1.0, tau_s=0.0) == 38.0


def test_ema_zero_dt_keeps_previous():
    assert bs.ema_step(41.0, 38.0, dt_s=0.0, tau_s=45.0) == 41.0


def test_ema_moves_toward_sample_by_time_constant():
    # за dt = tau проходит 1 - 1/e ~ 63.2 % пути
    v = bs.ema_step(41.0, 37.0, dt_s=45.0, tau_s=45.0)
    assert v == pytest.approx(41.0 + (37.0 - 41.0) * (1.0 - math.exp(-1.0)))
    assert 37.0 < v < 41.0


def test_ema_does_not_depend_on_sample_rate():
    """Одна выборка за 10 с = десять за 1 с (то же значение на постоянном входе)."""
    one = bs.ema_step(41.0, 38.0, 10.0, 45.0)
    many = 41.0
    for _ in range(10):
        many = bs.ema_step(many, 38.0, 1.0, 45.0)
    assert many == pytest.approx(one)


def test_smoother_first_sample_immediate_then_smooths_sag():
    sm = bs.VoltageSmoother(tau_s=45.0)
    assert sm.update(41.0, 100.0) == 41.0
    # просадка под нагрузкой на 3 В на 2 секунды: сглаженное почти не двигается
    sagged = sm.update(38.0, 102.0)
    assert 40.8 < sagged < 41.0
    # и процент почти не прыгает (сырой 38 В дал бы ~ 52 %, сглаженный ~ 90 %)
    raw_pct = bs.voltage_to_fraction(38.0)
    smooth_pct = bs.voltage_to_fraction(sagged)
    assert raw_pct < 0.60 < 0.88 < smooth_pct


def test_smoother_follows_a_lasting_change():
    sm = bs.VoltageSmoother(tau_s=45.0)
    sm.update(41.0, 0.0)
    v = 41.0
    for t in range(1, 301):  # 5 минут устойчиво 39 В
        v = sm.update(39.0, float(t))
    assert v == pytest.approx(39.0, abs=0.05)


def test_smoother_reset_makes_next_sample_immediate():
    sm = bs.VoltageSmoother(tau_s=45.0)
    sm.update(41.0, 0.0)
    sm.reset()
    assert sm.value is None
    assert sm.update(36.0, 1.0) == 36.0


def test_smoother_tau_zero_is_passthrough():
    sm = bs.VoltageSmoother(tau_s=0.0)
    sm.update(41.0, 0.0)
    assert sm.update(38.0, 1.0) == 38.0
