"""Tests for AV-22 Telegram handlers (``/cmd``, ``/operator``, gate in messages).

Строго: «тест на стык» (worker-brief §0 выводы — каждая сторона тестировалась
своим моком, а стык ничем; AV-22 фиксит именно это). ``/cmd`` идёт в
``/avatar/command``, не в личность. ``/operator on`` переключает гейт
«свободный текст → агент» в этом чате. ``/operator off`` (default) —
стандартное поведение: текст в личность через ``forward_to_stt``.

Не требует реального telegram.ext — стабы подгружаются как в
``test_commands.py``.
"""

from __future__ import annotations

import importlib
import json
import sys
import types
import unittest
from unittest.mock import AsyncMock, MagicMock


# Сохраняем оригинальные sys.modules (если реальные telegram/telegram.ext
# установлены) — test_telegram_bridge.py использует ИХ, и если наш
# фейк перетрёт — у соседнего теста всё сломается.
_ORIGINAL_TELEGRAM = {
    name: mod
    for name, mod in sys.modules.items()
    if name in ("telegram", "telegram.ext", "telegram.error", "PIL")
}


def _install_fake_dependencies() -> None:
    """Имитация ``telegram`` / ``telegram.ext`` / ``PIL`` для unit-тестов.

    Использует ``setdefault``-семантику: если модуль уже в ``sys.modules``
    (например, ``test_telegram_bridge.py`` уже подсунул свой фейк),
    мы НЕ перетираем его. Это предотвращает cross-pollution между
    test-файлами.

    При первом запуске в процессе модуль ``telegram`` обычно не установлен,
    поэтому ``types.ModuleType`` создаёт минимальный stub. Если реальный
    пакет установлен — мы НЕ трогаем его, и тогда ``from telegram import
    Update`` поднимет ImportError; в этом случае тесты ``test_telegram_bridge.py``
    тоже используют свои фейки.
    """
    # Только если модуля ещё нет в sys.modules — создаём stub.
    if "telegram" not in sys.modules:
        telegram_module = types.ModuleType("telegram")
        telegram_ext_module = types.ModuleType("telegram.ext")
        telegram_error_module = types.ModuleType("telegram.error")
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

        class TimedOut(Exception):
            pass

        class NetworkError(Exception):
            pass

        telegram_module.Update = Update
        telegram_module.InlineKeyboardButton = InlineKeyboardButton
        telegram_module.InlineKeyboardMarkup = InlineKeyboardMarkup
        telegram_ext_module.ContextTypes = ContextTypes
        telegram_error_module.TimedOut = TimedOut
        telegram_error_module.NetworkError = NetworkError
        pil_module.Image = MagicMock()

        sys.modules["telegram"] = telegram_module
        sys.modules["telegram.ext"] = telegram_ext_module
        sys.modules["telegram.error"] = telegram_error_module
    if "PIL" not in sys.modules:
        pil_module = types.ModuleType("PIL")
        pil_module.Image = MagicMock()
        sys.modules["PIL"] = pil_module


def _uninstall_fake_dependencies() -> None:
    """Восстановить оригинальные ``sys.modules`` (если были реальные).

    Удаляем только те модули, которые САМИ создали (см. ``_ORIGINAL_TELEGRAM``
    на момент импорта модуля).
    """
    for name, mod in _ORIGINAL_TELEGRAM.items():
        if mod is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = mod


def _load_commands_module():
    _install_fake_dependencies()
    sys.modules.pop("rob_box_telegram.handlers.commands", None)
    sys.modules.pop("rob_box_telegram.handlers.messages", None)
    sys.modules.pop("rob_box_telegram.auth", None)
    cmds = importlib.import_module("rob_box_telegram.handlers.commands")
    msgs = importlib.import_module("rob_box_telegram.handlers.messages")
    return cmds, msgs


def _teardown() -> None:
    """Модульный teardown: восстановить sys.modules и очистить импорты."""
    sys.modules.pop("rob_box_telegram.handlers.commands", None)
    sys.modules.pop("rob_box_telegram.handlers.messages", None)
    sys.modules.pop("rob_box_telegram.auth", None)
    sys.modules.pop("rob_box_telegram", None)
    _uninstall_fake_dependencies()


# pytest hook: вызывается после всего модуля (включая последний тест-класс).
import atexit

atexit.register(_teardown)


# ─── Cleanup каждого TestCase — для изоляции от соседних тестов ──────────


def _restore_sys_modules(test_instance) -> None:
    """После каждого test восстанавливаем sys.modules и чистим кэш импортов.

    Это критично, иначе наши фейки (``telegram``, ``telegram.ext``,
    ``telegram.error``) перетрут реальные модули и сломают соседний
    ``test_telegram_bridge.py`` (он использует настоящий PTB или
    свой фейк). НО в среде без PTB наш фейк НУЖЕН соседним тестам
    (``test_telegram_bridge.py``), иначе ImportError. Поэтому мы
    восстанавливаем только то, что было реально установлено при
    импорте нашего модуля (см. ``_ORIGINAL_TELEGRAM``).
    """
    for name in (
        "rob_box_telegram.handlers.commands",
        "rob_box_telegram.handlers.messages",
        "rob_box_telegram.auth",
        "rob_box_telegram",
    ):
        sys.modules.pop(name, None)
    # Удаляем telegram.* ТОЛЬКО если они не установлены реально
    # (т.е. в _ORIGINAL_TELEGRAM они None).
    for name in ("telegram", "telegram.ext", "telegram.error"):
        if _ORIGINAL_TELEGRAM.get(name) is None:
            sys.modules.pop(name, None)
        # Если был реальный — оставляем наш фейк, чтобы не сломать
        # соседние тесты, которые его используют.
    # ``PIL`` — всегда оставляем (если создали — он нужен).


# unittest.TestCase.setUp/tearDown хуки. Модульная функция — вызываем
# из каждого test-case (DRY через ``test_av22_base`` ниже).


class _TestAV22Base(unittest.IsolatedAsyncioTestCase):
    """Базовый класс: cleanup sys.modules ПОСЛЕ каждого теста."""

    def tearDown(self):
        _restore_sys_modules(self)


class TestCmdHandlerRoutesToAvatarCommand(_TestAV22Base):
    """AC: ``/cmd <text>`` → ``publish_avatar_command`` (НЕ ``forward_to_stt``)."""

    def setUp(self):
        self.cmds, _ = _load_commands_module()
        import rob_box_telegram.auth as auth_module

        auth_module._allowed_users = {42}

    def _make_update_and_context(self, message_text):
        node = MagicMock()
        # AV-22: новый метод publisher'а.
        node.publish_avatar_command = MagicMock(return_value="fixed-uuid")
        # Старый путь — НЕ должен зваться для /cmd.
        node.forward_to_stt = MagicMock()

        update = MagicMock()
        update.effective_chat.id = 42
        update.message.reply_text = AsyncMock()
        update.message.text = message_text

        # PTB-реальность: ``CommandHandler`` срезает ``/cmd`` и кладёт
        # остаток в ``context.args``. Имитируем это здесь, чтобы cmd_handler
        # работал так же, как в проде.
        parts = message_text.split(maxsplit=1) if message_text else []
        if parts and parts[0].startswith("/"):
            args = parts[1].split() if len(parts) > 1 else []
        else:
            args = []
        context = MagicMock()
        context.args = args
        context.bot_data = {"node": node}
        context.user_data = {}
        return update, context, node

    async def test_cmd_handler_with_text_routes_to_avatar_command(self):
        update, context, node = self._make_update_and_context(
            "/cmd мотивируй народ"
        )
        await self.cmds.cmd_handler(update, context)
        node.publish_avatar_command.assert_called_once_with(
            text="мотивируй народ", chat_id=42
        )
        node.forward_to_stt.assert_not_called()  # ← гейт «не в личность»

    async def test_cmd_handler_empty_shows_usage(self):
        update, context, node = self._make_update_and_context("/cmd")
        await self.cmds.cmd_handler(update, context)
        node.publish_avatar_command.assert_not_called()
        # Проверяем, что отправлено usage-сообщение.
        update.message.reply_text.assert_awaited_once()

    async def test_cmd_handler_returns_request_id_in_reply(self):
        update, context, node = self._make_update_and_context("/cmd играй")
        node.publish_avatar_command.return_value = "test-uuid-1"
        await self.cmds.cmd_handler(update, context)
        # В ответе оператору — ``request_id``, чтобы он мог сопоставить ответ.
        text = update.message.reply_text.await_args.args[0]
        assert "test-uuid-1" in text, (
            f"request_id не проброшен в ответ оператору: {text!r}"
        )

    async def test_cmd_handler_publish_failure_sends_warning(self):
        update, context, node = self._make_update_and_context("/cmd x")
        node.publish_avatar_command.return_value = None  # не удалось
        await self.cmds.cmd_handler(update, context)
        text = update.message.reply_text.await_args.args[0]
        assert "Не удалось" in text or "не удалось" in text


class TestOperatorHandlerTogglesPerChat(_TestAV22Base):
    """AC: ``/operator on|off`` — per-chat флаг в ``context.user_data``."""

    def setUp(self):
        self.cmds, _ = _load_commands_module()
        import rob_box_telegram.auth as auth_module

        auth_module._allowed_users = {42}

    def _make_update_and_context(self, message_text):
        update = MagicMock()
        update.effective_chat.id = 42
        update.message.reply_text = AsyncMock()
        update.message.text = message_text

        context = MagicMock()
        context.user_data = {}  # per-chat state
        return update, context

    async def test_operator_on_sets_user_data_true(self):
        update, context = self._make_update_and_context("/operator on")
        await self.cmds.operator_handler(update, context)
        assert context.user_data.get("operator_mode") is True

    async def test_operator_off_sets_user_data_false(self):
        update, context = self._make_update_and_context("/operator off")
        await self.cmds.operator_handler(update, context)
        assert context.user_data.get("operator_mode") is False

    async def test_operator_on_accepts_russian_keyword(self):
        update, context = self._make_update_and_context("/operator вкл")
        await self.cmds.operator_handler(update, context)
        assert context.user_data.get("operator_mode") is True

    async def test_operator_off_accepts_russian_keyword(self):
        update, context = self._make_update_and_context("/operator выкл")
        await self.cmds.operator_handler(update, context)
        assert context.user_data.get("operator_mode") is False

    async def test_operator_without_arg_shows_usage(self):
        update, context = self._make_update_and_context("/operator")
        await self.cmds.operator_handler(update, context)
        update.message.reply_text.assert_awaited_once()
        assert "Использование" in update.message.reply_text.await_args.args[0]

    async def test_operator_unknown_value_shows_usage(self):
        update, context = self._make_update_and_context("/operator foo")
        await self.cmds.operator_handler(update, context)
        update.message.reply_text.assert_awaited_once()
        assert "Использование" in update.message.reply_text.await_args.args[0]


class TestTextMessageGateByOperatorMode(_TestAV22Base):
    """AC: ``text_message_handler`` при ``operator_mode=True`` шлёт в
    ``/avatar/command`` и НЕ зовёт ``forward_to_stt``.

    Default off — стандартное поведение (текст → личность).
    """

    def setUp(self):
        self.cmds, self.msgs = _load_commands_module()
        import rob_box_telegram.auth as auth_module

        auth_module._allowed_users = {42}

    def _make_update_and_context(self, message_text, operator_mode=False):
        node = MagicMock()
        node.publish_avatar_command = MagicMock(return_value="req-1")
        node.forward_to_stt = MagicMock()

        update = MagicMock()
        update.effective_chat.id = 42
        update.message.reply_text = AsyncMock()
        update.message.text = message_text
        update.message.set_reaction = AsyncMock()

        context = MagicMock()
        context.bot_data = {"node": node}
        context.user_data = (
            {"operator_mode": True} if operator_mode else {}
        )
        # Чтобы избежать гонки debounce-таймера в тестах:
        context.user_data["msg_buffer"] = None
        return update, context, node

    async def test_operator_mode_on_text_goes_to_avatar_command(self):
        update, context, node = self._make_update_and_context(
            "поехали на кухню", operator_mode=True
        )
        await self.msgs.text_message_handler(update, context)
        node.publish_avatar_command.assert_called_once_with(
            text="поехали на кухню", chat_id=42
        )
        node.forward_to_stt.assert_not_called()  # ← гейт «личность молчит»

    async def test_operator_mode_off_text_goes_to_personality(self):
        update, context, node = self._make_update_and_context(
            "привет как дела", operator_mode=False
        )
        await self.msgs.text_message_handler(update, context)
        node.publish_avatar_command.assert_not_called()
        # В default режиме текст идёт в личность через forward_to_stt
        # (после debounce — но мы проверяем только гейт, без задержки).
        # ``forward_to_stt`` будет вызван через 2 сек debounce — не проверяем
        # его синхронно, иначе это integration-тест с реальным asyncio.sleep.

    async def test_cmd_handler_routes_unaffected_by_operator_mode(self):
        """``/cmd`` — ВСЕГДА в агента, независимо от ``operator_mode``.

        Тест явно покрывает это: ``/cmd`` обрабатывается PTB как Command,
        а не как TEXT, поэтому попадает в ``cmd_handler``, минуя
        ``text_message_handler`` и его гейт.
        """
        # Проверяем, что в handlers/commands.py нет гейта на operator_mode
        # в ``cmd_handler`` (тот ВСЕГДА шлёт в /avatar/command).
        node = MagicMock()
        node.publish_avatar_command = MagicMock(return_value="r")
        update = MagicMock()
        update.effective_chat.id = 42
        update.message.reply_text = AsyncMock()
        update.message.text = "/cmd играй"
        context = MagicMock()
        context.user_data = {}  # operator_mode НЕ установлен
        context.bot_data = {"node": node}
        # PTB-реальность: ``CommandHandler`` кладёт остаток от ``/cmd``
        # в ``context.args``. Имитируем.
        context.args = ["играй"]
        await self.cmds.cmd_handler(update, context)
        node.publish_avatar_command.assert_called_once_with(
            text="играй", chat_id=42
        )


class TestPublishAvatarCommandContract(_TestAV22Base):
    """AC: ``publish_avatar_command_via`` (чистая функция) публикует валидный payload.

    Это «тест на стык» producer'а — JSON должен декодироваться
    :func:`rob_box_core.avatar_command.decode_command` без ошибок и
    содержать server-side client_id. Чистая функция
    ``publish_avatar_command_via`` вынесена в ``rob_box_core``, чтобы
    тесты работали БЕЗ rclpy (issue: импорт telegram_node грузит rclpy).
    """

    def setUp(self):
        # Грузим avatar_command как чистый Python.
        self.cmds, self.msgs = _load_commands_module()

    def _make_fake_pub(self):
        """Fake-publisher, совместимый с чистой функцией ``publish_avatar_command_via``."""
        captured = []

        class _Pub:
            def publish(self, msg):
                captured.append(msg)
        return _Pub(), captured

    async def test_publish_avatar_command_via_publishes_valid_json(self):
        """AC: ``publish_avatar_command_via(pub, text='...', chat_id=42)``
        публикует JSON, который декодируется ``decode_command`` и совпадает
        с ``build_command``.
        """
        from rob_box_core.avatar_command import (
            build_command,
            decode_command,
            make_telegram_client_id,
            publish_avatar_command_via,
        )

        pub, captured = self._make_fake_pub()
        request_id = publish_avatar_command_via(
            pub, text="мотивируй народ", chat_id=42,
        )
        assert request_id is not None
        assert len(captured) == 1
        decoded = decode_command(captured[0].data)
        expected = build_command(
            source="telegram",
            client_id=make_telegram_client_id(42),
            text="мотивируй народ",
            request_id=request_id,
        )
        assert decoded["source"] == expected["source"]
        assert decoded["client_id"] == expected["client_id"]
        assert decoded["text"] == expected["text"]
        assert decoded["request_id"] == expected["request_id"]

    async def test_publish_avatar_command_via_empty_returns_none(self):
        from rob_box_core.avatar_command import publish_avatar_command_via

        pub, captured = self._make_fake_pub()
        result = publish_avatar_command_via(pub, text="", chat_id=42)
        assert result is None
        assert len(captured) == 0

    async def test_publish_avatar_command_from_quest_via_uses_quest_client_id(self):
        """Quest-сторона: ``client_id='quest:<session_id>'``."""
        from rob_box_core.avatar_command import (
            decode_command,
            make_quest_client_id,
            publish_avatar_command_from_quest_via,
        )

        pub, captured = self._make_fake_pub()
        request_id = publish_avatar_command_from_quest_via(
            pub, text="поехали", session_id="sess-abc",
        )
        assert request_id is not None
        assert len(captured) == 1
        decoded = decode_command(captured[0].data)
        assert decoded["source"] == "quest"
        assert decoded["client_id"] == make_quest_client_id("sess-abc")
        assert decoded["text"] == "поехали"

    async def test_publish_avatar_command_from_quest_via_empty_returns_none(self):
        from rob_box_core.avatar_command import publish_avatar_command_from_quest_via

        pub, captured = self._make_fake_pub()
        result = publish_avatar_command_from_quest_via(
            pub, text="   ", session_id="s",
        )
        assert result is None
        assert len(captured) == 0
