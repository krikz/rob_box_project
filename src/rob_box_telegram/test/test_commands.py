#!/usr/bin/env python3
"""Tests for Telegram command routing through the canonical ToolProvider."""

from __future__ import annotations

import importlib
import sys
import types
import unittest
from unittest.mock import AsyncMock, MagicMock

from rob_box_core.ports import ToolProvider, ToolResult


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


class TestTelegramMusicCommands(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.commands = _load_commands_module()
        import rob_box_telegram.auth as auth_module

        auth_module._allowed_users = {42}

    def _make_update_and_context(self, message_text, args=None):
        node = MagicMock()
        node.tool_provider = MagicMock(spec=ToolProvider)
        node.tool_provider.invoke = AsyncMock(return_value=ToolResult(value="ok"))

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

        node.tool_provider.invoke.assert_not_called()
        update.message.reply_text.assert_awaited_once_with(
            "Использование: /repl <Renardo/FoxDot код>"
        )

    async def test_repl_handler_sends_single_line_code(self):
        update, context, node = self._make_update_and_context(
            "/repl p1 >> pluck([0,2,4])"
        )
        node.tool_provider.invoke = AsyncMock(
            return_value=ToolResult(value="Код выполнен успешно")
        )

        await self.commands.repl_handler(update, context)

        node.tool_provider.invoke.assert_awaited_once_with(
            "execute_music_code",
            {"code": "p1 >> pluck([0,2,4])"},
        )
        update.message.reply_text.assert_awaited_once_with(
            "🎵 Код выполнен успешно"
        )

    async def test_repl_handler_preserves_newlines_in_multiline_code(self):
        multiline_code = "Clock.bpm = 83\np1 >> pads((2, 4, 6), amp=0.3)"
        update, context, node = self._make_update_and_context(
            f"/repl\n{multiline_code}"
        )
        node.tool_provider.invoke = AsyncMock(
            return_value=ToolResult(value="Код выполнен успешно")
        )

        await self.commands.repl_handler(update, context)

        node.tool_provider.invoke.assert_awaited_once_with(
            "execute_music_code", {"code": multiline_code}
        )

    async def test_repl_handler_strips_botname_suffix(self):
        update, context, node = self._make_update_and_context(
            "/repl@RoBBoxbot p1 >> blip([0,2])"
        )

        await self.commands.repl_handler(update, context)

        code_sent = node.tool_provider.invoke.call_args.args[1]["code"]
        self.assertEqual(code_sent, "p1 >> blip([0,2])")

    async def test_stopmusic_handler_calls_stop_music(self):
        update, context, node = self._make_update_and_context("")
        node.tool_provider.invoke = AsyncMock(
            return_value=ToolResult(value="Вся музыка остановлена")
        )

        await self.commands.stopmusic_handler(update, context)

        node.tool_provider.invoke.assert_awaited_once_with("stop_music", {})
        update.message.reply_text.assert_awaited_once_with(
            "⏹ Вся музыка остановлена"
        )


if __name__ == "__main__":
    unittest.main()
