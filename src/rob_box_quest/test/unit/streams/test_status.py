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


def test_update_battery_accepts_pct_and_volts():
    s = StatusAggregator()
    s.update_battery(pct=77.4, volts=24.1)
    decoded = msgpack.unpackb(s.payload(), raw=False)
    assert decoded["battery_pct"] == 77
    assert decoded["battery_v"] == 24.1


def test_update_battery_none_does_not_clear_known_value():
    # Источники приходят вразнобой: VESC даёт вольты, BMS — проценты.
    s = StatusAggregator()
    s.update_battery(pct=80)
    s.update_battery(volts=23.5)
    decoded = msgpack.unpackb(s.payload(), raw=False)
    assert decoded["battery_pct"] == 80
    assert decoded["battery_v"] == 23.5


def test_battery_v_is_null_without_source():
    s = StatusAggregator()
    decoded = msgpack.unpackb(s.payload(), raw=False)
    # None, а не 0.0 — клиент должен отличать "нет источника" от "0 вольт".
    assert decoded["battery_v"] is None


def test_update_wifi_sets_and_clears():
    s = StatusAggregator()
    s.update_wifi(-61)
    assert msgpack.unpackb(s.payload(), raw=False)["wifi_rssi"] == -61
    s.update_wifi(None)
    assert msgpack.unpackb(s.payload(), raw=False)["wifi_rssi"] == 0
