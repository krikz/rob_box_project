"""Unit-тесты streams.status (StatusAggregator)."""

import msgpack

from rob_box_quest.streams.status import StatusAggregator


def test_default_payload_has_sentinel_values():
    s = StatusAggregator()
    payload = s.payload()
    decoded = msgpack.unpackb(payload, raw=False)
    # Defaults: battery_pct=-1 (no source), wifi_rssi=0, mode='idle', vel=0.
    assert decoded["battery_pct"] == -1
    assert decoded["wifi_rssi"] == 0
    assert decoded["mode"] == "idle"
    assert decoded["vel_linear"] == 0.0
    assert decoded["vel_angular"] == 0.0
    assert "ts_ms" in decoded


def test_update_velocity_changes_payload():
    s = StatusAggregator()
    s.update_velocity(0.5, 0.1)
    payload = s.payload()
    decoded = msgpack.unpackb(payload, raw=False)
    assert decoded["vel_linear"] == 0.5
    assert decoded["vel_angular"] == 0.1


def test_set_mode_changes_payload():
    s = StatusAggregator()
    s.set_mode("teleop_active")
    payload = s.payload()
    decoded = msgpack.unpackb(payload, raw=False)
    assert decoded["mode"] == "teleop_active"


def test_real_battery_and_wifi_overrides_sentinel():
    s = StatusAggregator()
    s.battery_pct = 75
    s.wifi_rssi = -55
    payload = s.payload()
    decoded = msgpack.unpackb(payload, raw=False)
    assert decoded["battery_pct"] == 75
    assert decoded["wifi_rssi"] == -55


def test_ts_ms_is_recent():
    s = StatusAggregator()
    payload = s.payload()
    decoded = msgpack.unpackb(payload, raw=False)
    # Не проверяем точное значение — только что > 2025-01-01.
    assert decoded["ts_ms"] > 1_700_000_000_000
