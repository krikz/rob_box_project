#!/usr/bin/env python3
"""Tests for Telegram command handlers (Phase 6 v2 / W7).

After W7 the Telegram node is a *thin transport*: every slash-command
that used to invoke a tool now forwards its intent to ``/voice/stt/result``
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
    sys.modules.setdefault("numpy", numpy_module)
    sys.modules.setdefault("PIL", pil_module)


def _load_commands_module():
    _install_fake_dependencies()
    sys.modules.pop("rob_box_telegram.keyboard_layouts", None)
    sys.modules.pop("rob_box_telegram.handlers.commands", None)
    return importlib.import_module("rob_box_telegram.handlers.commands")


class TestTelegramCommandForwarding(unittest.IsolatedAsyncioTestCase):
    """Verify that W7 handlers forward intents to /voice/stt/result."""

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

    async def test_repl_handler_requires_code(self):
        update, context, node = self._make_update_and_context("/repl")

        await self.commands.repl_handler(update, context)

        node.forward_to_stt.assert_not_called()
        update.message.reply_text.assert_awaited_once_with(
            "Использование: /repl <Renardo/FoxDot код>"
        )

    async def test_repl_handler_forwards_single_line_code(self):
        update, context, node = self._make_update_and_context(
            "/repl p1 >> pluck([0,2,4])"
        )

        await self.commands.repl_handler(update, context)

        node.forward_to_stt.assert_called_once()
        self.assertEqual(
            node.forward_to_stt.call_args.args[0],
            "/repl p1 >> pluck([0,2,4])",
        )

    async def test_repl_handler_preserves_newlines_in_multiline_code(self):
        multiline_code = "Clock.bpm = 83\np1 >> pads((2, 4, 6), amp=0.3)"
        update, context, node = self._make_update_and_context(
            f"/repl\n{multiline_code}"
        )

        await self.commands.repl_handler(update, context)

        self.assertEqual(
            node.forward_to_stt.call_args.args[0],
            f"/repl {multiline_code}",
        )

    async def test_repl_handler_strips_botname_suffix(self):
        update, context, node = self._make_update_and_context(
            "/repl@RoBBoxbot p1 >> blip([0,2])"
        )

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
        update, context, node = self._make_update_and_context(
            "/goto kitchen", args=["kitchen"]
        )

        await self.commands.goto_handler(update, context)

        self.assertEqual(node.forward_to_stt.call_args.args[0], "/goto kitchen")

    async def test_clear_handler_forwards_clear_intent(self):
        update, context, node = self._make_update_and_context("/clear")

        await self.commands.clear_handler(update, context)

        node.forward_to_stt.assert_called_once_with("/clear", chat_id=42)

    async def test_volume_handler_validates_range(self):
        update, context, node = self._make_update_and_context(
            "/volume 200", args=["200"]
        )

        await self.commands.volume_handler(update, context)

        node.forward_to_stt.assert_not_called()
        update.message.reply_text.assert_awaited_once_with(
            "⚠️ Уровень громкости должен быть от 0 до 100"
        )

    async def test_volume_handler_forwards_level(self):
        update, context, node = self._make_update_and_context(
            "/volume 75", args=["75"]
        )

        await self.commands.volume_handler(update, context)

        self.assertEqual(node.forward_to_stt.call_args.args[0], "/volume 75")


if __name__ == "__main__":
    unittest.main()
