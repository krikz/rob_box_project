#!/usr/bin/env python3
"""Tests for Telegram command handlers in rob_box_telegram.handlers.commands."""

import importlib
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock


def _install_fake_telegram_modules() -> None:
    telegram_module = types.ModuleType("telegram")
    telegram_ext_module = types.ModuleType("telegram.ext")

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

    sys.modules["telegram"] = telegram_module
    sys.modules["telegram.ext"] = telegram_ext_module


def _load_commands_module():
    _install_fake_telegram_modules()
    sys.modules.pop("rob_box_telegram.keyboard_layouts", None)
    sys.modules.pop("rob_box_telegram.handlers.commands", None)
    return importlib.import_module("rob_box_telegram.handlers.commands")


class TestTelegramMusicCommands(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.commands = _load_commands_module()
        import rob_box_telegram.auth as auth_module

        auth_module._allowed_users = {42}

    def _make_update_and_context(self, args):
        node = MagicMock()
        node.mcp_bridge.execute_simple = AsyncMock(return_value="ok")

        update = MagicMock()
        update.effective_chat.id = 42
        update.message.reply_text = AsyncMock()

        context = MagicMock()
        context.args = args
        context.bot_data = {"node": node}
        context.user_data = {}
        return update, context, node

    async def test_repl_handler_requires_code(self):
        update, context, node = self._make_update_and_context([])

        await self.commands.repl_handler(update, context)

        node.mcp_bridge.execute_simple.assert_not_called()
        update.message.reply_text.assert_awaited_once_with("Использование: /repl <Renardo/FoxDot код>")

    async def test_repl_handler_sends_code_to_execute_music_code(self):
        update, context, node = self._make_update_and_context(["p1", ">>", 'pluck([0,2,4])'])
        node.mcp_bridge.execute_simple = AsyncMock(return_value="Код выполнен успешно")

        await self.commands.repl_handler(update, context)

        node.mcp_bridge.execute_simple.assert_awaited_once_with(
            "execute_music_code",
            {"code": 'p1 >> pluck([0,2,4])'},
        )
        update.message.reply_text.assert_awaited_once_with("🎵 Код выполнен успешно")

    async def test_stopmusic_handler_calls_stop_music(self):
        update, context, node = self._make_update_and_context([])
        node.mcp_bridge.execute_simple = AsyncMock(return_value="Вся музыка остановлена")

        await self.commands.stopmusic_handler(update, context)

        node.mcp_bridge.execute_simple.assert_awaited_once_with("stop_music")
        update.message.reply_text.assert_awaited_once_with("⏹ Вся музыка остановлена")


class TestTelegramNodeCommandRegistration(unittest.TestCase):
    def test_telegram_node_registers_repl_and_stopmusic_handlers(self):
        source = Path(
            "/home/builder/rob_box_project/src/rob_box_telegram/rob_box_telegram/telegram_node.py"
        ).read_text(encoding="utf-8")

        self.assertIn('CommandHandler("repl", repl_handler)', source)
        self.assertIn('CommandHandler("stopmusic", stopmusic_handler)', source)


if __name__ == "__main__":
    unittest.main()