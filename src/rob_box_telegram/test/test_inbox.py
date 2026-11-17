"""Строка ``[tg-in]`` — контракт с координатором (ADR-0150 PR-12a #3706, docs/process/claude-coordinator-robot-flow.md §5)."""

from __future__ import annotations

import asyncio
import logging
import os
import tempfile
import unittest
from unittest.mock import MagicMock

from rob_box_telegram.inbox import (KEEP_FILES, TG_IN_MARK, log_incoming, quote_of, save_attachment, stash_attachment,
                                    tg_in_line)


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

    def test_reply_quote_and_file_go_before_text(self):
        line = tg_in_line(495039871, "Клод глянь", reply_to="старое\nсообщение", file="/tmp/tg_inbox/1_2.jpg")
        self.assertEqual(line, "📨 [tg-in] chat_id=495039871 coordinator=true file=/tmp/tg_inbox/1_2.jpg "
                               "reply_to=старое ⏎ сообщение text=Клод глянь")

    def test_quote_is_cut_and_cannot_forge_the_text_field(self):
        line = tg_in_line(1, "ок", reply_to="a text=подделка " + "я" * 500)
        head, _, tail = line.partition(" text=")
        self.assertEqual(tail, "ок")
        self.assertLessEqual(len(head.split("reply_to=")[1]), 200)

    def test_line_without_extras_keeps_the_old_contract(self):
        self.assertEqual(tg_in_line(1, "привет"), "📨 [tg-in] chat_id=1 coordinator=false text=привет")

    def test_quote_of_takes_text_or_caption_of_replied_message(self):
        msg = MagicMock()
        msg.reply_to_message.text = "про розетку"
        self.assertEqual(quote_of(msg), "про розетку")
        msg.reply_to_message.text = None
        msg.reply_to_message.caption = "подпись"
        self.assertEqual(quote_of(msg), "подпись")
        msg.reply_to_message = None
        self.assertEqual(quote_of(msg), "")


class TestAttachments(unittest.TestCase):
    def test_saved_under_code_built_name_and_pruned(self):
        with tempfile.TemporaryDirectory() as root:
            path = save_attachment(b"data", 77, "J/PG", root, clock=lambda: 1000)
            self.assertEqual(path, os.path.join(root, "1000_77.jpg"))
            self.assertEqual(open(path, "rb").read(), b"data")
            for n in range(KEEP_FILES + 5):
                save_attachment(b"x", n, "bin", root, clock=lambda n=n: 2000 + n)
            self.assertEqual(len(os.listdir(root)), KEEP_FILES)

    def test_empty_or_huge_is_not_saved(self):
        with tempfile.TemporaryDirectory() as root:
            self.assertIsNone(save_attachment(b"", 1, "jpg", root))
            self.assertEqual(os.listdir(root), [])

    def test_only_owner_chat_attachments_are_downloaded(self):
        bot, message = MagicMock(), MagicMock()
        message.photo = [MagicMock(file_id="small"), MagicMock(file_id="big")]
        message.message_id = 9
        with tempfile.TemporaryDirectory() as root:
            self.assertEqual(asyncio.run(stash_attachment(bot, message, 123, root)), "")
            bot.get_file.assert_not_called()

            async def get_file(file_id):
                self.assertEqual(file_id, "big")
                tg_file = MagicMock()

                async def download():
                    return bytearray(b"jpegbytes")

                tg_file.download_as_bytearray = download
                return tg_file

            bot.get_file = get_file
            path = asyncio.run(stash_attachment(bot, message, 495039871, root))
            self.assertTrue(path.startswith(root) and path.endswith("_9.jpg"))
            self.assertEqual(open(path, "rb").read(), b"jpegbytes")


if __name__ == "__main__":
    unittest.main()
