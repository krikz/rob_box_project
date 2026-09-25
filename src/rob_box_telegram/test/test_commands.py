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


class TestFaceHandlers(unittest.IsolatedAsyncioTestCase):
    """``/faces`` and ``/face`` handlers (issue #3025).

    These handlers read the live face store directly from disk. The
    tests monkeypatch ``commands.FACE_STORE_MOUNT`` to a tmp dir so
    we don't need a real ``/data/faces`` in the container.

    The two handlers are read-only — they never write to the store.
    We don't need PIL for the empty-store paths; the collage path is
    exercised by ``test_face_card.py`` (PIL has a `pytest.skip` guard
    there if missing).
    """

    def setUp(self):
        self.commands = _load_commands_module()
        import rob_box_telegram.auth as auth_module

        auth_module._allowed_users = {42}
        self._orig_root = self.commands.FACE_STORE_MOUNT

    def tearDown(self):
        # Restore the production mount so other tests / re-imports
        # keep working.
        self.commands.FACE_STORE_MOUNT = self._orig_root

    def _make_update_and_context(self, args=None):
        node = MagicMock()
        update = MagicMock()
        update.effective_chat.id = 42
        update.message.reply_text = AsyncMock()
        update.message.reply_photo = AsyncMock()

        context = MagicMock()
        context.args = args or []
        context.bot_data = {"node": node}
        context.user_data = {}
        return update, context

    async def test_faces_handler_empty_store(self):
        import tempfile

        with tempfile.TemporaryDirectory() as td:
            self.commands.FACE_STORE_MOUNT = td
            update, context = self._make_update_and_context()
            await self.commands.faces_handler(update, context)
        update.message.reply_text.assert_awaited_once()
        text = update.message.reply_text.call_args.args[0]
        self.assertIn("Лицевая база пуста", text)
        # Must NOT call reply_photo — no collage for /faces
        update.message.reply_photo.assert_not_awaited()

    async def test_faces_handler_lists_named_first(self):
        import tempfile

        with tempfile.TemporaryDirectory() as td:
            from pathlib import Path

            root = Path(td)
            (root / "4ff0ddc5").mkdir()
            (root / "4ff0ddc5" / "meta.json").write_text(
                '{"name": "Дэнчик", "person_id": "4ff0ddc5", "encounter_count": 7}',
                encoding="utf-8",
            )
            (root / "bbb22222").mkdir()
            (root / "bbb22222" / "meta.json").write_text(
                '{"person_id": "bbb22222", "encounter_count": 1}',  # no name = stranger
                encoding="utf-8",
            )

            self.commands.FACE_STORE_MOUNT = td
            update, context = self._make_update_and_context()
            await self.commands.faces_handler(update, context)

        text = update.message.reply_text.call_args.args[0]
        self.assertIn("Дэнчик", text)
        self.assertIn("4ff0ddc5", text)
        self.assertIn("«незнакомец»", text)

    async def test_face_handler_no_args_shows_usage(self):
        update, context = self._make_update_and_context(args=[])
        await self.commands.face_handler(update, context)
        update.message.reply_text.assert_awaited_once()
        text = update.message.reply_text.call_args.args[0]
        self.assertIn("/face", text)
        self.assertIn("4ff0ddc5", text)

    async def test_face_handler_unknown_id(self):
        import tempfile

        with tempfile.TemporaryDirectory() as td:
            self.commands.FACE_STORE_MOUNT = td
            update, context = self._make_update_and_context(args=["nonexistent"])
            await self.commands.face_handler(update, context)
        text = update.message.reply_text.call_args.args[0]
        self.assertIn("Не нашёл", text)
        update.message.reply_photo.assert_not_awaited()

    async def test_face_handler_sends_photo_when_collage_available(self):
        """End-to-end-ish: a tmp dir with a real JPEG reference
        triggers both ``reply_photo`` (with bytes) and ``reply_text``
        is NOT called.

        PIL is a soft dependency; the test skips if missing — the
        empty-store branch above still runs in that case.
        """
        try:
            from PIL import Image  # noqa: F401
        except Exception:
            self.skipTest("PIL not installed — collage builder unavailable")

        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            person = root / "4ff0ddc5"
            person.mkdir()
            (person / "meta.json").write_text(
                '{"name": "Дэнчик", "person_id": "4ff0ddc5", '
                '"speaker_id": "1ae4b0ac", "encounter_count": 1, '
                '"last_encounter_ts": 1700000000.0, '
                '"mode_recorded": "operator"}',
                encoding="utf-8",
            )
            ref = Image.new("RGB", (16, 16), color=(255, 255, 255))
            ref.save(person / "reference.jpg", format="JPEG")

            self.commands.FACE_STORE_MOUNT = td
            update, context = self._make_update_and_context(args=["4ff0ddc5"])
            await self.commands.face_handler(update, context)

        # Collage path: reply_photo was called with bytes
        update.message.reply_photo.assert_awaited_once()
        photo_arg = update.message.reply_photo.call_args.kwargs.get("photo")
        if photo_arg is None:
            # PTB signature: positional
            photo_arg = update.message.reply_photo.call_args.args[0]
        # BytesIO or raw bytes — both have .read()
        data = photo_arg.read()
        self.assertTrue(data.startswith(b"\xff\xd8"))  # JPEG magic
        # caption arg carries the summary text
        caption = update.message.reply_photo.call_args.kwargs.get(
            "caption"
        ) or update.message.reply_photo.call_args.args[1]
        self.assertIn("Дэнчик", caption)
        self.assertIn("последняя встреча по лицу", caption)
        # No separate reply_text on the happy path — caption is enough
        update.message.reply_text.assert_not_awaited()


if __name__ == "__main__":
    unittest.main()
