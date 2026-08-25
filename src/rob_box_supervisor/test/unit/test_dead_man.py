"""Unit-тесты для DeadManCounter (pure-Python)."""

from __future__ import annotations

import unittest

from rob_box_supervisor.core.dead_man import DeadManCounter


class TestDeadManCounter(unittest.TestCase):
    def test_initial_count_is_zero(self) -> None:
        c = DeadManCounter()
        self.assertEqual(c.count("quest"), 0)
        self.assertEqual(c.snapshot(), {})

    def test_trip_increments(self) -> None:
        c = DeadManCounter()
        self.assertEqual(c.trip("quest"), 1)
        self.assertEqual(c.trip("quest"), 2)
        self.assertEqual(c.trip("telegram"), 1)
        self.assertEqual(c.count("quest"), 2)
        self.assertEqual(c.count("telegram"), 1)

    def test_snapshot_returns_copy(self) -> None:
        c = DeadManCounter()
        c.trip("quest")
        snap = c.snapshot()
        snap["quest"] = 999  # модификация копии не должна затронуть счётчик
        self.assertEqual(c.count("quest"), 1)

    def test_reset_single_client(self) -> None:
        c = DeadManCounter()
        c.trip("quest")
        c.trip("telegram")
        c.reset("quest")
        self.assertEqual(c.count("quest"), 0)
        self.assertEqual(c.count("telegram"), 1)

    def test_reset_all(self) -> None:
        c = DeadManCounter()
        c.trip("quest")
        c.trip("telegram")
        c.reset()
        self.assertEqual(c.snapshot(), {})


if __name__ == "__main__":
    unittest.main()
