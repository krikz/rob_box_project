"""Тесты журнала ТАРС (operator_journal.py) — DoD issue #1988.

DoD-инвариант: «два „перезапустил voice-assistant“ за час схлопываются в
одну запись со счётчиком». Плюс: bounded-окно, выход за окно = новая запись,
сериализация JSONL, best-effort персист.

Модуль чистый (без ROS) — тесты без mock-циклов; conftest всё равно
поднимает mock-rclpy, это не мешает.
"""

from __future__ import annotations

import json
import os
import tempfile
import unittest

from rob_box_supervisor.operator_journal import (
    DEFAULT_COLLAPSE_WINDOW_S,
    JournalEntry,
    OperatorJournal,
)


class _FakeClock:
    """Детерминированное время: секунды epoch, двигаем вручную."""

    def __init__(self, start_s: float = 1_700_000_000.0) -> None:
        self._now_s = start_s

    def __call__(self) -> float:
        return self._now_s

    def advance(self, seconds: float) -> None:
        self._now_s += seconds


class TestCollapse(unittest.TestCase):
    """DoD: два «перезапустил voice-assistant» за час → одна запись счётчиком."""

    def setUp(self) -> None:
        self.clock = _FakeClock()
        self.journal = OperatorJournal(clock=self.clock)

    def test_two_same_actions_within_hour_collapse(self) -> None:
        """Два повтора в окне → одна запись, count=2, last_ts_ms обновлён."""
        first = self.journal.record("перезапустил voice-assistant", outcome="ok")
        self.clock.advance(60)  # +1 минута (в пределах часа)
        second = self.journal.record("перезапустил voice-assistant", outcome="ok")

        self.assertEqual(len(self.journal), 1)
        # Обе записи — один и тот же объект (схлопнулись).
        self.assertEqual(first.action, "перезапустил voice-assistant")
        self.assertEqual(second.count, 2)
        self.assertEqual(first.ts_ms, second.ts_ms)  # ts_ms = первое вхождение
        self.assertGreater(second.last_ts_ms, first.last_ts_ms)
        # last_ts_ms = ts первого + 60 000 мс.
        self.assertEqual(second.last_ts_ms - first.ts_ms, 60_000)
        self.assertEqual(second.ts_ms, first.ts_ms)

    def test_three_repeats_count_three(self) -> None:
        """Три повтора за час → count=3."""
        self.journal.record("перезапустил voice-assistant")
        self.clock.advance(600)
        self.journal.record("перезапустил voice-assistant")
        self.clock.advance(600)
        third = self.journal.record("перезапустил voice-assistant")

        self.assertEqual(len(self.journal), 1)
        self.assertEqual(third.count, 3)

    def test_outside_window_is_new_entry(self) -> None:
        """Повтор вне окна (>= 1 ч) → новая запись, счётчик не растёт."""
        self.journal.record("перезапустил voice-assistant")
        self.clock.advance(DEFAULT_COLLAPSE_WINDOW_S + 1)
        entry = self.journal.record("перезапустил voice-assistant")

        self.assertEqual(len(self.journal), 2)
        self.assertEqual(entry.count, 1)

    def test_different_actions_do_not_collapse(self) -> None:
        """Разные действия не схлопываются."""
        self.journal.record("перезапустил voice-assistant")
        self.journal.record("включил анимацию полиция")
        self.assertEqual(len(self.journal), 2)

    def test_last_outcome_wins_on_collapse(self) -> None:
        """При схлопывании outcome/detail — от последнего вхождения."""
        self.journal.record("перезапустил voice-assistant", outcome="ok")
        self.clock.advance(30)
        collapsed = self.journal.record(
            "перезапустил voice-assistant",
            outcome="та же ошибка: ALSA device busy",
        )
        self.assertEqual(collapsed.outcome, "та же ошибка: ALSA device busy")

    def test_empty_action_rejected(self) -> None:
        """Пустой action — ValueError."""
        with self.assertRaises(ValueError):
            self.journal.record("   ")


class TestPersistence(unittest.TestCase):
    """JSONL-персист: загрузка/сохранение и best-effort на битом файле."""

    def test_roundtrip_file(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "journal.jsonl")
            clock = _FakeClock()
            j1 = OperatorJournal(path=path, clock=clock)
            j1.record("перезапустил voice-assistant", outcome="ok")
            clock.advance(60)
            j1.record("перезапустил voice-assistant", outcome="ok")

            # Пересоздаём из того же файла — запись схлопнута и загружена.
            j2 = OperatorJournal(path=path, clock=_FakeClock())
            self.assertEqual(len(j2), 1)
            entry = j2.recent(limit=1)[0]
            self.assertEqual(entry.action, "перезапустил voice-assistant")
            self.assertEqual(entry.count, 2)

    def test_bad_file_is_ignored(self) -> None:
        """Битый JSONL не роняет журнал (best-effort → пусто)."""
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "journal.jsonl")
            with open(path, "w", encoding="utf-8") as fh:
                fh.write("{not-json}\n")
            journal = OperatorJournal(path=path)
            self.assertEqual(len(journal), 0)

    def test_render(self) -> None:
        journal = OperatorJournal(clock=_FakeClock())
        journal.record("перезапустил voice-assistant", outcome="ok")
        text = journal.render()
        self.assertIn("перезапустил voice-assistant", text)
        self.assertIn("[журнал оператора]", text)


class TestSerialization(unittest.TestCase):
    def test_entry_dict_roundtrip(self) -> None:
        e = JournalEntry(
            action="x", ts_ms=1, last_ts_ms=2, count=3, outcome="ok"
        )
        raw = json.dumps(e.to_dict())
        restored = JournalEntry.from_dict(json.loads(raw))
        self.assertEqual(restored, e)


if __name__ == "__main__":
    unittest.main()
