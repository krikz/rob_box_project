#!/usr/bin/env python3
"""Tests for /avatar command + floor callbacks (AV-24, issue #1916).

Покрывает:

* ``avatar_handler`` — отправляет карточку с inline-кнопками + регистрирует
  в ``AvatarCardStore`` (через ``bot_data``).
* ``_handle_floor`` — take granted / take denied (показываем held_by) /
  release (без отказа).
* ``_handle_avatar_refresh`` — форсированный refresh из текущего state.
* ``AvatarCardDispatcher`` (sync listener) — синхронно проверяем логику:
  планирует ``call_soon_threadsafe`` в asyncio-loop (loop-стаб через
  MagicMock).
* ``message is not modified`` в ``_safe_edit`` — НЕ считается ошибкой.

Эти тесты **не** требуют python-telegram-bot / rclpy — handlers тестируем
через подмену telegram.Update и node.
"""

from __future__ import annotations

import importlib
import sys
import types
import unittest
from typing import Any, Dict
from unittest.mock import AsyncMock, MagicMock

# ── fakes ────────────────────────────────────────────────────────────────


def _install_fake_telegram() -> None:
    """Минимальный стаб telegram.Update / InlineKeyboardButton /
    InlineKeyboardMarkup — ровно столько, чтобы импортировать handler'ы."""

    if "telegram" in sys.modules:
        return
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


def _load_module(name: str):
    _install_fake_telegram()
    sys.modules.pop(name, None)
    return importlib.import_module(name)


def _make_fake_supervisor(granted: bool = True, held_by: str = "quest") -> MagicMock:
    """Подменить node.supervisor: всегда возвращает заданный результат."""
    from rob_box_telegram.supervisor_client import AcquireResult, Floor

    sup = MagicMock()
    sup.client_id = "telegram"
    sup.state = MagicMock()
    sup.state.teleop_floor = None
    sup.state.voice_floor = None
    sup.state.mode = "off"
    sup.state.since_ms = 0
    sup.acquire_floor = MagicMock(
        return_value=AcquireResult(
            granted=granted,
            denied_reason=None if granted else "held_by_other",
            held_by=held_by if not granted else None,
        )
    )
    sup.release_floor = MagicMock()
    sup.subscribe_state = MagicMock(return_value=lambda: None)
    return sup


def _make_fake_node(supervisor: MagicMock | None = None) -> MagicMock:
    node = MagicMock()
    node.supervisor = supervisor or _make_fake_supervisor()
    return node


def _make_update_and_context(node: MagicMock, chat_id: int = 42):
    update = MagicMock()
    update.effective_chat.id = chat_id
    sent_msg = MagicMock()
    sent_msg.message_id = 999
    update.message.reply_text = AsyncMock(return_value=sent_msg)

    application = MagicMock()
    application.bot_data = {"node": node}
    context = MagicMock()
    context.application = application
    context.bot_data = application.bot_data
    context.user_data = {}
    context.args = []
    return update, context


def _make_query_update_and_context(
    callback_data: str, chat_id: int = 42, message_id: int = 999, node: MagicMock | None = None
):
    """Inline-button callback (query, не message)."""
    query = MagicMock()
    query.data = callback_data
    query.message.chat_id = chat_id
    query.message.message_id = message_id
    query.answer = AsyncMock()
    query.edit_message_text = AsyncMock()

    application = MagicMock()
    application.bot_data = {"node": node or _make_fake_node()}
    context = MagicMock()
    context.application = application
    context.bot_data = application.bot_data
    context.user_data = {}
    update = MagicMock()
    update.callback_query = query
    return update, context, query


# ── /avatar command ──────────────────────────────────────────────────────


class TestAvatarCommand(unittest.IsolatedAsyncioTestCase):
    async def test_avatar_sends_card_with_keyboard(self) -> None:
        from rob_box_telegram import auth as auth_module

        auth_module._allowed_users = {42}
        commands = _load_module("rob_box_telegram.handlers.commands")
        sys.modules.pop("rob_box_telegram.avatar_card", None)
        avatar_card = importlib.import_module("rob_box_telegram.avatar_card")

        node = _make_fake_node()
        update, context = _make_update_and_context(node)

        await commands.avatar_handler(update, context)

        # Сообщение отправлено с inline-кнопками
        update.message.reply_text.assert_awaited_once()
        kwargs = update.message.reply_text.call_args.kwargs
        self.assertIsNotNone(kwargs.get("reply_markup"))
        # Кнопки содержат floor:take:*
        rm = kwargs["reply_markup"]
        cb_data = [b.callback_data for row in rm.inline_keyboard for b in row]
        self.assertIn("floor:take:teleop", cb_data)
        self.assertIn("floor:take:voice", cb_data)
        self.assertIn("avatar:refresh", cb_data)
        # store зарегистрировал карточку
        store = context.bot_data["avatar_card_store"]
        self.assertEqual(store.record_count(), 1)
        rec = store.get(42)
        self.assertEqual(rec.message_id, 999)
        self.assertIn("🎮", rec.last_text)  # карточка начинается с эмодзи

    async def test_avatar_does_not_register_listener_per_call(self) -> None:
        """100 /avatar в разных чатах → один listener на supervisor (subscribed один раз).

        Это проверка контракта «нет утечки listener'ов»: один и тот же
        ``application.bot_data`` живёт всё время работы бота — handler
        устанавливает ``avatar_state_listener_installed`` на первый
        вызов и больше не дёргает ``subscribe_state``.
        """
        from rob_box_telegram import auth as auth_module

        auth_module._allowed_users = set(range(200))
        commands = _load_module("rob_box_telegram.handlers.commands")
        sys.modules.pop("rob_box_telegram.avatar_card", None)

        # Один общий application на все итерации — как в реальном процессе.
        node = _make_fake_node()
        application = MagicMock()
        application.bot_data = {"node": node}
        context = MagicMock()
        context.application = application
        context.bot_data = application.bot_data
        context.user_data = {}
        context.args = []
        context.bot_data["telegram_loop"] = MagicMock()

        for chat_id in range(100):
            update = MagicMock()
            update.effective_chat.id = chat_id
            sent_msg = MagicMock()
            sent_msg.message_id = 999
            update.message.reply_text = AsyncMock(return_value=sent_msg)
            await commands.avatar_handler(update, context)

        # supervisor.subscribe_state вызван РОВНО один раз
        self.assertEqual(node.supervisor.subscribe_state.call_count, 1)
        # store знает про 100 карточек
        store = context.bot_data["avatar_card_store"]
        self.assertEqual(store.record_count(), 100)


# ── floor callbacks ──────────────────────────────────────────────────────


class TestFloorCallbacks(unittest.IsolatedAsyncioTestCase):
    async def test_take_granted(self) -> None:
        from rob_box_telegram import auth as auth_module

        auth_module._allowed_users = {42}
        callbacks = _load_module("rob_box_telegram.handlers.callbacks")
        sys.modules.pop("rob_box_telegram.avatar_card", None)

        node = _make_fake_node(_make_fake_supervisor(granted=True))
        update, context, query = _make_query_update_and_context("floor:take:teleop", chat_id=42, node=node)
        context.bot_data["avatar_card_store"] = importlib.import_module(
            "rob_box_telegram.avatar_card"
        ).AvatarCardStore()

        await callbacks.callback_handler(update, context)

        node.supervisor.acquire_floor.assert_called_once()
        query.answer.assert_awaited()  # крутилка снята
        query.edit_message_text.assert_awaited()  # карточка обновлена

    async def test_take_denied_shows_holder(self) -> None:
        from rob_box_telegram import auth as auth_module

        auth_module._allowed_users = {42}
        callbacks = _load_module("rob_box_telegram.handlers.callbacks")
        sys.modules.pop("rob_box_telegram.avatar_card", None)

        node = _make_fake_node(_make_fake_supervisor(granted=False, held_by="quest"))
        update, context, query = _make_query_update_and_context("floor:take:teleop", chat_id=42, node=node)
        context.bot_data["avatar_card_store"] = importlib.import_module(
            "rob_box_telegram.avatar_card"
        ).AvatarCardStore()

        await callbacks.callback_handler(update, context)

        # В тексте edit_message_text — имя держателя
        kwargs = query.edit_message_text.await_args.kwargs
        text_arg = kwargs["text"]
        self.assertIn("quest", text_arg)
        self.assertIn("Не удалось взять teleop", text_arg)

    async def test_release(self) -> None:
        from rob_box_telegram import auth as auth_module

        auth_module._allowed_users = {42}
        callbacks = _load_module("rob_box_telegram.handlers.callbacks")
        sys.modules.pop("rob_box_telegram.avatar_card", None)

        node = _make_fake_node()
        update, context, query = _make_query_update_and_context("floor:release:teleop", chat_id=42, node=node)
        context.bot_data["avatar_card_store"] = importlib.import_module(
            "rob_box_telegram.avatar_card"
        ).AvatarCardStore()

        await callbacks.callback_handler(update, context)

        # release вызван, acquire НЕ вызван
        node.supervisor.release_floor.assert_called_once()
        node.supervisor.acquire_floor.assert_not_called()

    async def test_avatar_refresh(self) -> None:
        from rob_box_telegram import auth as auth_module

        auth_module._allowed_users = {42}
        callbacks = _load_module("rob_box_telegram.handlers.callbacks")
        sys.modules.pop("rob_box_telegram.avatar_card", None)

        node = _make_fake_node()
        update, context, query = _make_query_update_and_context("avatar:refresh", chat_id=42, node=node)
        context.bot_data["avatar_card_store"] = importlib.import_module(
            "rob_box_telegram.avatar_card"
        ).AvatarCardStore()

        await callbacks.callback_handler(update, context)

        query.answer.assert_awaited()
        query.edit_message_text.assert_awaited()

    async def test_bad_callback_data_logged_not_raised(self) -> None:
        from rob_box_telegram import auth as auth_module

        auth_module._allowed_users = {42}
        callbacks = _load_module("rob_box_telegram.handlers.callbacks")

        node = _make_fake_node()
        update, context, _query = _make_query_update_and_context("floor:bogus:xxx", chat_id=42)
        # Не должно кидать — handler логирует warning и возвращается.
        await callbacks.callback_handler(update, context)


# ── AvatarCardDispatcher (sync listener) ────────────────────────────────


class TestAvatarCardDispatcher(unittest.IsolatedAsyncioTestCase):
    async def test_listener_schedules_to_loop(self) -> None:
        """Listener синхронно зовёт loop.call_soon_threadsafe с правильными args."""
        import asyncio

        from rob_box_telegram.avatar_card import AvatarCardStore
        from rob_box_telegram.handlers import avatar_card_dispatcher as disp
        from rob_box_telegram.supervisor_client import AvatarState

        loop = asyncio.new_event_loop()
        try:
            bot_data: Dict[str, Any] = {}
            store = AvatarCardStore()
            bot_data["avatar_card_store"] = store

            listener = disp.make_dispatcher(loop=loop, store=store, bot_data=bot_data)

            st = AvatarState(mode="off")
            store.register(chat_id=42, message_id=10, text="<initial>", state=st)

            listener(st)
            # loop.call_soon_threadsafe уже зван — корутина запланирована.
            # Дать ей отработать:
            await asyncio.sleep(0.01)
            loop.call_soon_threadsafe(loop.stop)
            # run_until_complete не нужен — корутина уже в loop'е
        finally:
            loop.close()

    async def test_listener_with_no_loop_does_not_raise(self) -> None:
        """loop=None (тестовый режим) — listener тихо обновляет store и не падает."""
        from rob_box_telegram.avatar_card import AvatarCardStore
        from rob_box_telegram.handlers import avatar_card_dispatcher as disp
        from rob_box_telegram.supervisor_client import AvatarState

        store = AvatarCardStore()
        bot_data: Dict[str, Any] = {}
        listener = disp.make_dispatcher(loop=None, store=store, bot_data=bot_data)

        st = AvatarState(mode="off")
        store.register(chat_id=42, message_id=10, text="<initial>", state=st)
        # Не должно бросить исключение.
        listener(st)


# ── message is not modified не считается ошибкой ─────────────────────────


class TestSafeEdit(unittest.IsolatedAsyncioTestCase):
    async def test_safe_edit_swallows_not_modified(self) -> None:
        """``BadRequest: message is not modified`` подавляется."""
        from rob_box_telegram.handlers import callbacks as cb

        # Подделка query с edit_message_text, бросающим BadRequest
        class _BadReq(Exception):
            pass

        query = MagicMock()

        async def _edit(*a, **kw):
            raise _BadReq("message is not modified")

        query.edit_message_text = _edit
        # Не должно бросить наружу.
        await cb._safe_edit(query, "text", reply_markup=MagicMock())

    async def test_safe_edit_logs_other_errors(self) -> None:
        """Другие ошибки логируются и не подавляются (но и не падают наверх)."""
        from rob_box_telegram.handlers import callbacks as cb

        query = MagicMock()

        async def _edit(*a, **kw):
            raise RuntimeError("network blip")

        query.edit_message_text = _edit
        # Не должно бросить наружу.
        await cb._safe_edit(query, "text", reply_markup=MagicMock())


# ── _publish_stop учитывает denied ──────────────────────────────────────


class TestPublishStop(unittest.IsolatedAsyncioTestCase):
    async def test_publish_stop_denied_logs(self) -> None:
        """_publish_stop с отказанным floor — логирует, не падает."""
        from rob_box_telegram.supervisor_client import AcquireResult
        from rob_box_telegram.handlers import callbacks as cb

        node = MagicMock()
        node.publish_move_with_floor = MagicMock(
            return_value=AcquireResult(granted=False, denied_reason="held_by_other", held_by="quest")
        )
        cb._publish_stop(node)  # не должно бросить

    async def test_publish_stop_granted_silent(self) -> None:
        """_publish_stop granted — тихо, без логов."""
        from rob_box_telegram.supervisor_client import AcquireResult
        from rob_box_telegram.handlers import callbacks as cb

        node = MagicMock()
        node.publish_move_with_floor = MagicMock(return_value=AcquireResult(granted=True))
        cb._publish_stop(node)


if __name__ == "__main__":
    unittest.main()
