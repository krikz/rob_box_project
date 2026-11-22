#!/usr/bin/env python3
"""Tests for Telegram command handlers (Phase 6 v2 / W7).

After W7 the Telegram node is a *thin transport*: every slash-command
that used to invoke a tool now forwards its intent to ``/voice/stt/utterance``
through ``TelegramNode.forward_to_stt``. These tests assert that the
forwarding happens and the user receives an acknowledgement.
"""

from __future__ import annotations

import importlib
import sys
import types
import unittest
from unittest.mock import AsyncMock, MagicMock


def _install_fake_dependencies() -> None:
    telegram_module = types.ModuleType("telegram")
    telegram_ext_module = types.ModuleType("telegram.ext")
    numpy_module = types.ModuleType("numpy")
    pil_module = types.ModuleType("PIL")

    class Update:
        pass

    class ContextTypes:
        DEFAULT_TYPE = object

    class InlineKeyboardButton:
        def __init__(self, text, callback_data=None):
            self.text = text
            self.callback_data = callback_data

    class InlineKeyboardMarkup:
        def __init__(self, inline_keyboard):
            self.inline_keyboard = inline_keyboard

    telegram_module.Update = Update
    telegram_module.InlineKeyboardButton = InlineKeyboardButton
    telegram_module.InlineKeyboardMarkup = InlineKeyboardMarkup
    telegram_ext_module.ContextTypes = ContextTypes
    pil_module.Image = MagicMock()

    sys.modules["telegram"] = telegram_module
    sys.modules["telegram.ext"] = telegram_ext_module
    # NOTE: don't shadow ``PIL`` with a MagicMock here — ``commands.py``
    # uses real PIL at import time (``_depth_compressed_to_jpeg`` etc.),
    # and the face_card tests that follow us would then hit the mock.
    # If a future test needs a fake PIL, mock it locally inside that test.
    sys.modules.setdefault("numpy", numpy_module)
    sys.modules.setdefault("PIL", pil_module)


def _load_commands_module():
    _install_fake_dependencies()
    sys.modules.pop("rob_box_telegram.keyboard_layouts", None)
    sys.modules.pop("rob_box_telegram.handlers.commands", None)
    return importlib.import_module("rob_box_telegram.handlers.commands")


class TestTelegramCommandForwarding(unittest.IsolatedAsyncioTestCase):
    """Verify that W7 handlers forward intents to /voice/stt/utterance."""

    def setUp(self):
        self.commands = _load_commands_module()
        import rob_box_telegram.auth as auth_module

        auth_module._allowed_users = {42}

    def _make_update_and_context(self, message_text, args=None):
        node = MagicMock()
        node.forward_to_stt = MagicMock()

        update = MagicMock()
        update.effective_chat.id = 42
        update.message.reply_text = AsyncMock()
        update.message.text = message_text

        context = MagicMock()
        context.args = args or []
        context.bot_data = {"node": node}
        context.user_data = {}
        return update, context, node

    async def test_say_handler_gives_no_premature_success_and_asks_for_a_receipt(self):
        """#3727: /say не пишет «Озвучиваю» до итога; чат получит квитанцию по SpeechEvent."""
        update, context, node = self._make_update_and_context("/say привет", args=["привет", "мир"])
        node.publish_tts_with_floor.return_value = MagicMock(granted=True)

        await self.commands.say_handler(update, context)

        node.publish_tts_with_floor.assert_called_once_with("привет мир", receipt_chat_id=42)
        update.message.reply_text.assert_not_awaited()

    async def test_say_handler_floor_denied_is_reported(self):
        update, context, node = self._make_update_and_context("/say привет", args=["привет"])
        node.publish_tts_with_floor.return_value = MagicMock(granted=False, held_by="Quest")

        await self.commands.say_handler(update, context)

        update.message.reply_text.assert_awaited_once()
        self.assertIn("Голос удерживает Quest", update.message.reply_text.await_args.args[0])

    async def test_repl_handler_requires_code(self):
        update, context, node = self._make_update_and_context("/repl")

        await self.commands.repl_handler(update, context)

        node.forward_to_stt.assert_not_called()
        update.message.reply_text.assert_awaited_once_with("Использование: /repl <Renardo/FoxDot код>")

    async def test_repl_handler_forwards_single_line_code(self):
        update, context, node = self._make_update_and_context("/repl p1 >> pluck([0,2,4])")

        await self.commands.repl_handler(update, context)

        node.forward_to_stt.assert_called_once()
        self.assertEqual(
            node.forward_to_stt.call_args.args[0],
            "/repl p1 >> pluck([0,2,4])",
        )

    async def test_repl_handler_preserves_newlines_in_multiline_code(self):
        multiline_code = "Clock.bpm = 83\np1 >> pads((2, 4, 6), amp=0.3)"
        update, context, node = self._make_update_and_context(f"/repl\n{multiline_code}")

        await self.commands.repl_handler(update, context)

        self.assertEqual(
            node.forward_to_stt.call_args.args[0],
            f"/repl {multiline_code}",
        )

    async def test_repl_handler_strips_botname_suffix(self):
        update, context, node = self._make_update_and_context("/repl@RoBBoxbot p1 >> blip([0,2])")

        await self.commands.repl_handler(update, context)

        self.assertEqual(
            node.forward_to_stt.call_args.args[0],
            "/repl p1 >> blip([0,2])",
        )

    async def test_stopmusic_handler_forwards_stop_music_intent(self):
        update, context, node = self._make_update_and_context("")

        await self.commands.stopmusic_handler(update, context)

        node.forward_to_stt.assert_called_once_with("/stopmusic", chat_id=42)

    async def test_status_handler_forwards_status_intent(self):
        update, context, node = self._make_update_and_context("/status")

        await self.commands.status_handler(update, context)

        node.forward_to_stt.assert_called_once_with("/status", chat_id=42)

    async def test_goto_handler_forwards_waypoint_name(self):
        update, context, node = self._make_update_and_context("/goto kitchen", args=["kitchen"])

        await self.commands.goto_handler(update, context)

        self.assertEqual(node.forward_to_stt.call_args.args[0], "/goto kitchen")

    async def test_clear_handler_forwards_clear_intent(self):
        update, context, node = self._make_update_and_context("/clear")

        await self.commands.clear_handler(update, context)

        node.forward_to_stt.assert_called_once_with("/clear", chat_id=42)

    async def test_volume_handler_validates_range(self):
        update, context, node = self._make_update_and_context("/volume 200", args=["200"])

        await self.commands.volume_handler(update, context)

        node.forward_to_stt.assert_not_called()
        update.message.reply_text.assert_awaited_once_with("⚠️ Уровень громкости должен быть от 0 до 100")

    async def test_volume_handler_forwards_level(self):
        update, context, node = self._make_update_and_context("/volume 75", args=["75"])

        await self.commands.volume_handler(update, context)

        self.assertEqual(node.forward_to_stt.call_args.args[0], "/volume 75")


class TestFaceHandlers(unittest.IsolatedAsyncioTestCase):
    """``/face`` берёт карточку у владельца (issue #3655), не с диска.

    ``fetch`` подставной: ``(ok, reason, data, jpeg)``. Диск ``/data/faces``
    эти тесты не создают и не читают.
    """

    def setUp(self):
        self.commands = _load_commands_module()
        import rob_box_telegram.auth as auth_module

        auth_module._allowed_users = {42}

    def _make_update_and_context(self, args=None, fetch=None):
        node = MagicMock()
        node.people_card = fetch
        update = MagicMock()
        update.effective_chat.id = 42
        update.message.reply_text = AsyncMock()
        update.message.reply_photo = AsyncMock()
        context = MagicMock()
        context.args = args or []
        context.bot_data = {"node": node}
        context.user_data = {}
        return update, context

    async def test_face_handler_no_args_shows_usage(self):
        update, context = self._make_update_and_context(args=[])
        await self.commands.face_handler(update, context)
        update.message.reply_text.assert_awaited_once()
        text = update.message.reply_text.call_args.args[0]
        self.assertIn("/face", text)
        self.assertIn("4ff0ddc5", text)
        update.message.reply_photo.assert_not_awaited()

    async def test_face_handler_unknown_id_is_owner_refusal(self):
        def fetch(query, op):
            return False, "person_not_found", {}, b""

        update, context = self._make_update_and_context(args=["nonexistent"], fetch=fetch)
        await self.commands.face_handler(update, context)
        text = update.message.reply_text.call_args.args[0]
        self.assertIn("нет в базе", text)
        update.message.reply_photo.assert_not_awaited()

    async def test_face_handler_sends_owner_jpeg_not_a_local_build(self):
        jpeg = b"\xff\xd8owner-card"

        def fetch(query, op):
            self.assertEqual(query, "4ff0ddc5")
            self.assertEqual(op, 6)
            return (
                True,
                "",
                {"id": "4ff0ddc5", "name": "Дэнчик", "encounters": 3, "last_encounter": "2023-11-14T22:13:20Z"},
                jpeg,
            )

        update, context = self._make_update_and_context(args=["4ff0ddc5"], fetch=fetch)
        await self.commands.face_handler(update, context)
        update.message.reply_photo.assert_awaited_once()
        photo = update.message.reply_photo.call_args.kwargs.get("photo")
        self.assertEqual(photo.read(), jpeg)
        caption = update.message.reply_photo.call_args.kwargs.get("caption")
        self.assertIn("Дэнчик", caption)
        self.assertIn("последняя встреча по лицу", caption)
        update.message.reply_text.assert_not_awaited()

    async def test_face_handler_without_snapshot_refuses_without_a_picture(self):
        def fetch(query, op):
            return False, "no_snapshot", {"id": "4ff0ddc5", "name": "Дэнчик"}, b""

        update, context = self._make_update_and_context(args=["4ff0ddc5"], fetch=fetch)
        await self.commands.face_handler(update, context)
        text = update.message.reply_text.call_args.args[0]
        self.assertIn("нет снимка", text)
        self.assertIn("Дэнчик", text)
        update.message.reply_photo.assert_not_awaited()


if __name__ == "__main__":
    unittest.main()
