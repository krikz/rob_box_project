"""Tests for rob_box_core.battery: кривая заряда 10S Li-ion (#4001, #4002).

Таблица ориентировочная (не замерена на нашей батарее) - тесты фиксируют
контракт модуля и опорные точки, а не истину про железо.
"""

from __future__ import annotations

import math

import pytest

from rob_box_core import battery as b


@pytest.mark.parametrize(
    "volts, percent",
    [
        (42.0, 100.0),
        (41.0, 90.0),
        (40.0, 80.0),
        (39.2, 70.0),
        (37.9, 50.0),
        (36.0, 15.0),
        (33.0, 2.0),
        (32.0, 0.0),
    ],
)
def test_reference_points(volts, percent):
    assert b.voltage_to_percent(volts) == pytest.approx(percent, abs=1e-6)


def test_all_table_points_roundtrip():
    for cell_v, pct in b.OCV_TABLE:
        assert b.voltage_to_percent(cell_v * b.BATTERY_CELLS) == pytest.approx(pct, abs=1e-6)


def test_interpolates_between_points():
    # между 4.10 В (90 %) и 4.20 В (100 %) на банку: середина = 95 %
    assert b.voltage_to_percent(41.5) == pytest.approx(95.0)
    # между 3.60 (15 %) и 3.63 (20 %): 3.615 -> 17.5 %
    assert b.voltage_to_percent(36.15) == pytest.approx(17.5)


@pytest.mark.parametrize("volts", [30.0, 29.0, 1.0])
def test_below_empty_is_clamped_to_zero(volts):
    assert b.voltage_to_percent(volts) == 0.0


@pytest.mark.parametrize("volts", [42.5, 45.0, 100.0])
def test_above_full_is_clamped_to_hundred(volts):
    assert b.voltage_to_percent(volts) == 100.0


@pytest.mark.parametrize("volts", [None, 0, 0.0, -1.0, float("nan"), float("inf"), "abc"])
def test_no_data_is_none(volts):
    assert b.voltage_to_percent(volts) is None
    assert b.voltage_to_fraction(volts) is None


def test_curve_is_monotonic_non_decreasing():
    prev = -1.0
    v = 28.0
    while v <= 45.0:
        pct = b.voltage_to_percent(v)
        assert pct >= prev - 1e-9, v
        assert 0.0 <= pct <= 100.0
        prev = pct
        v += 0.01


def test_table_is_strictly_ascending_in_voltage_and_monotonic_in_percent():
    volts = [v for v, _ in b.OCV_TABLE]
    pcts = [p for _, p in b.OCV_TABLE]
    assert volts == sorted(set(volts))
    assert pcts == sorted(pcts)


def test_bus_limits_follow_table_and_cell_count():
    assert b.BATTERY_CELLS == 10
    assert b.BATTERY_VOLTAGE_FULL == pytest.approx(42.0)
    assert b.BATTERY_VOLTAGE_EMPTY == pytest.approx(32.0)


def test_fraction_is_percent_over_100():
    assert b.voltage_to_fraction(41.0) == pytest.approx(0.9)
    assert b.voltage_to_fraction(32.0) == 0.0
    assert b.voltage_to_fraction(50.0) == 1.0


def test_valid_voltage():
    assert b.valid_voltage(41.0) == 41.0
    assert b.valid_voltage("41.5") == 41.5
    for bad in (None, 0, -2.0, float("nan"), float("inf"), "x"):
        assert b.valid_voltage(bad) is None
    assert not math.isnan(b.valid_voltage(1e-3))
