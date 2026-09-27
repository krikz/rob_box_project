"""Unit-тесты для StateAggregator (pure-Python, без rclpy)."""

from __future__ import annotations

import json
import unittest

from rob_box_supervisor.core.aggregator import AvatarState, StateAggregator


class TestStateAggregator(unittest.TestCase):
    def setUp(self) -> None:
        self.agg = StateAggregator()

    def test_initial_snapshot_is_empty(self) -> None:
        snap = self.agg.snapshot()
        self.assertIsInstance(snap, AvatarState)
        self.assertIsNone(snap.pose_xy)
        self.assertIsNone(snap.battery_pct)
        self.assertIsNone(snap.voice_state)
        self.assertEqual(snap.dead_man_trips_total, {})
        self.assertGreater(snap.ts_ms, 0)

    def test_update_odom_sets_pose(self) -> None:
        self.agg.update_odom(1.5, -2.25)
        snap = self.agg.snapshot()
        self.assertEqual(snap.pose_xy, (1.5, -2.25))

    def test_update_device_snapshot_battery(self) -> None:
        self.agg.update_device_snapshot(battery_pct=87.5)
        self.agg.update_device_snapshot(battery_pct=None)  # не затирает
        snap = self.agg.snapshot()
        self.assertEqual(snap.battery_pct, 87.5)

    def test_update_voice_state(self) -> None:
        self.agg.update_voice_state("listening")
        snap = self.agg.snapshot()
        self.assertEqual(snap.voice_state, "listening")

    def test_record_dead_man_trip_increments(self) -> None:
        self.assertEqual(self.agg.record_dead_man_trip("quest"), 1)
        self.assertEqual(self.agg.record_dead_man_trip("quest"), 2)
        self.assertEqual(self.agg.record_dead_man_trip("telegram"), 1)
        self.assertEqual(self.agg.dead_man_count("quest"), 2)
        self.assertEqual(self.agg.dead_man_count("telegram"), 1)
        self.assertEqual(self.agg.dead_man_count("nobody"), 0)

    def test_record_dead_man_updates_last_event(self) -> None:
        self.agg.record_dead_man_trip("quest")
        snap = self.agg.snapshot()
        self.assertEqual(snap.last_event["kind"], "dead_man_trip")
        self.assertEqual(snap.last_event["client_id"], "quest")
        self.assertEqual(snap.last_event["trip_count"], 1)

    def test_msgpack_dict_round_trip(self) -> None:
        self.agg.update_odom(3.0, 4.0)
        self.agg.update_device_snapshot(battery_pct=42.0)
        self.agg.update_voice_state("speaking")
        self.agg.record_dead_man_trip("quest")
        snap = self.agg.snapshot()
        d = snap.to_msgpack_dict()
        # Сериализуем в JSON как proxy для msgpack (msgpack умеет в bytes-like).
        text = json.dumps(d)
        parsed = json.loads(text)
        self.assertEqual(parsed["pose_xy"], [3.0, 4.0])
        self.assertEqual(parsed["battery_pct"], 42.0)
        self.assertEqual(parsed["voice_state"], "speaking")
        self.assertEqual(parsed["dead_man_trips_total"], {"quest": 1})
        self.assertEqual(parsed["last_event"]["client_id"], "quest")


if __name__ == "__main__":
    unittest.main()
