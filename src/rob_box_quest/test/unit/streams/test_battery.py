"""Unit-тесты streams.battery (разбор телеметрии заряда)."""

from rob_box_quest.streams.battery import parse_battery_json, voltage_to_pct


class TestParseBatteryJson:
    def test_reads_percent_field(self):
        pct, volts = parse_battery_json({"battery_pct": 87.5})
        assert pct == 87.5
        assert volts is None

    def test_reads_alternative_percent_name(self):
        pct, _ = parse_battery_json({"battery_percentage": 42})
        assert pct == 42.0

    def test_reads_voltage_field(self):
        pct, volts = parse_battery_json({"battery_voltage": 24.3})
        assert pct is None
        assert volts == 24.3

    def test_battery_key_is_voltage(self):
        # context_aggregator_node.on_device_snapshot кладёт в 'battery'
        # именно msg.battery_voltage.
        _, volts = parse_battery_json({"battery": 23.9})
        assert volts == 23.9

    def test_reads_both(self):
        pct, volts = parse_battery_json({"battery_pct": 60, "voltage_input": 22.1})
        assert pct == 60.0
        assert volts == 22.1

    def test_numeric_strings_are_accepted(self):
        pct, volts = parse_battery_json({"battery_pct": "55", "battery_v": " 24.0 "})
        assert pct == 55.0
        assert volts == 24.0

    def test_unrelated_payload_yields_nones(self):
        assert parse_battery_json({"temperature": 41.0}) == (None, None)

    def test_bool_is_not_a_number(self):
        assert parse_battery_json({"battery_pct": True}) == (None, None)


class TestVoltageToPct:
    def test_linear_midpoint(self):
        assert voltage_to_pct(22.2, 19.2, 25.2) == 50

    def test_clamps_above_full(self):
        assert voltage_to_pct(30.0, 19.2, 25.2) == 100

    def test_clamps_below_empty(self):
        assert voltage_to_pct(10.0, 19.2, 25.2) == 0

    def test_no_bounds_means_no_guess(self):
        # Границы не заданы (0.0) — процентов не выдумываем.
        assert voltage_to_pct(22.2, 0.0, 0.0) is None

    def test_inverted_bounds_rejected(self):
        assert voltage_to_pct(22.2, 25.2, 19.2) is None

    def test_no_voltage_returns_none(self):
        assert voltage_to_pct(None, 19.2, 25.2) is None
