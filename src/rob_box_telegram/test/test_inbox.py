"""Строка ``[tg-in]`` — контракт с координатором (ADR-0150 PR-12a #3706, docs/process/claude-coordinator-robot-flow.md §5)."""

from __future__ import annotations

import logging
import unittest
from unittest.mock import MagicMock

from rob_box_telegram.inbox import TG_IN_MARK, log_incoming, tg_in_line


class TestTgInLine(unittest.TestCase):
    def test_format_is_stable(self):
        self.assertEqual(tg_in_line(495039871, "который час"),
                         "📨 [tg-in] chat_id=495039871 coordinator=false text=который час")

    def test_coordinator_call_is_flagged(self):
        for text in ("Клод привет", "клод, глянь лог", "Claude ping"):
            self.assertIn("coordinator=true", tg_in_line(1, text), text)
        self.assertIn("coordinator=false", tg_in_line(1, "Клодия, привет"))

    def test_multiline_message_stays_one_line(self):
        line = tg_in_line(1, "Клод\nпервая\n  вторая  ")
        self.assertNotIn("\n", line)
        self.assertTrue(line.endswith("text=Клод ⏎ первая ⏎ вторая"))
        self.assertIn("coordinator=true", line)

    def test_logged_through_node_logger(self):
        node = MagicMock()
        log_incoming(node, 5, "привет")
        node.get_logger.return_value.info.assert_called_once_with(f"{TG_IN_MARK} chat_id=5 coordinator=false text=привет")

    def test_without_node_logger_falls_back_to_module_logger(self):
        with self.assertLogs("rob_box_telegram.inbox", level=logging.INFO) as logs:
            log_incoming(object(), 5, "Клод тест")
        self.assertIn("coordinator=true", logs.output[0])


if __name__ == "__main__":
    unittest.main()
