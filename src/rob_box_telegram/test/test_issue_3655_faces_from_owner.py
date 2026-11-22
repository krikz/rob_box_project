"""Issue #3655: ``/faces`` берёт список у владельца галереи, не с диска.

Два контракта:

* Vision не ответил — бот говорит это явно, а не «лиц нет» (ADR-0148);
* дисковый путь ``FACE_STORE_MOUNT`` / ``list_people`` в обработчик
  ``/faces`` не возвращается (гард по тексту модуля).
"""

from __future__ import annotations

# CI гоняет каждый файл отдельно (pytest test/<файл>.py), а setup.py исключает
# каталог test из пакетов — сосед ``test_commands`` в sys.path не попадает.
# Грузим хелперы по пути файла, как и сам модуль команд.
import importlib.util
import unittest
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock

_HELPERS = importlib.util.spec_from_file_location("_faces_test_commands", Path(__file__).with_name("test_commands.py"))
_helpers = importlib.util.module_from_spec(_HELPERS)
_HELPERS.loader.exec_module(_helpers)
_install_fake_dependencies = _helpers._install_fake_dependencies
_load_commands_module = _helpers._load_commands_module

_install_fake_dependencies()  # до импорта пакета: telegram в тестовом интерпретаторе нет

import rob_box_telegram.auth as auth_module  # noqa: E402  isort: skip
from rob_box_telegram.face_roster_view import UNAVAILABLE_TEXT, format_owner_listing  # noqa: E402  isort: skip

_REPO_ROOT = Path(__file__).resolve().parents[3]


def _update_and_context(node):
    update = MagicMock()
    update.effective_chat.id = 42
    update.message.reply_text = AsyncMock()
    context = MagicMock()
    context.args = []
    context.bot_data = {"node": node}
    context.user_data = {}
    return update, context


class TestFacesFromOwner(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.commands = _load_commands_module()
        auth_module._allowed_users = {42}

    async def test_vision_unavailable_is_honest_not_empty(self):
        """Владелец не ответил: текст отказа, и в нём нет «лиц нет»/«база пуста»."""
        node = MagicMock()
        node.people_list = lambda: (False, "owner_unavailable", {})
        update, context = _update_and_context(node)

        await self.commands.faces_handler(update, context)

        update.message.reply_text.assert_awaited_once()
        text = update.message.reply_text.call_args.args[0]
        self.assertEqual(text, UNAVAILABLE_TEXT)
        self.assertNotIn("Лицевая база пуста", text)
        self.assertNotIn("лиц нет", text)

    async def test_owner_list_is_rendered_not_read_from_disk(self):
        """Ответ владельца становится таблицей; диск бот для списка не открывает."""
        node = MagicMock()
        node.people_list = lambda: (
            True,
            "",
            {
                "people": [
                    {"id": "4ff0ddc5", "name": "Дэнчик", "encounters": 7, "last_encounter_ts": 1_700_000_000.0},
                    {"id": "bbb22222", "name": None, "encounters": 1, "last_encounter_ts": 1_600_000_000.0},
                ],
                "total": 2,
            },
        )
        update, context = _update_and_context(node)

        await self.commands.faces_handler(update, context)

        text = update.message.reply_text.call_args.args[0]
        self.assertIn("Дэнчик", text)
        self.assertIn("4ff0ddc5", text)
        self.assertIn("«незнакомец»", text)
        self.assertNotIn("/data/faces", text)


class TestFacesDiskPathCannotReturn(unittest.TestCase):
    """Гард: список лиц с диска Sound Pi в бот не возвращается."""

    def test_face_store_mount_is_gone(self):
        source = Path(self._commands_source()).read_text(encoding="utf-8")
        self.assertNotIn("FACE_STORE_MOUNT", source)

    def test_faces_handler_does_not_read_disk(self):
        source = Path(self._commands_source()).read_text(encoding="utf-8")
        handler = source.split("async def faces_handler", 1)[1].split("\nasync def ", 1)[0]
        self.assertNotIn("list_people", handler)
        self.assertNotIn("format_faces_table", handler)
        self.assertNotIn('"/data/faces"', handler)
        self.assertNotIn("FACE_CARD_ROOT", handler)
        self.assertIn("people_list", handler)

    def test_face_handler_does_not_read_disk(self):
        """Карточка ``/face`` тоже не с диска: ни корня галереи, ни читалки снимков."""
        source = Path(self._commands_source()).read_text(encoding="utf-8")
        handler = source.split("async def face_handler", 1)[1].split("\nasync def ", 1)[0]
        self.assertNotIn("FACE_CARD_ROOT", handler)
        self.assertNotIn("load_detail", handler)
        self.assertNotIn("find_summary", handler)
        self.assertNotIn("build_face_collage", handler)
        self.assertNotIn('"/data/faces"', handler)
        self.assertIn("people_card", handler)
        self.assertNotIn(
            "rob_box_telegram/face_card.py",
            "\n".join(p.name for p in (_REPO_ROOT / "src/rob_box_telegram/rob_box_telegram").glob("*.py")),
        )

    def test_bot_compose_does_not_mount_faces(self):
        text = (_REPO_ROOT / "docker/sound/docker-compose.yaml").read_text(encoding="utf-8")
        bot = text.split("  telegram-bot:", 1)[1].split("\n  # ", 1)[0]
        self.assertNotIn("./data/faces:", bot)
        self.assertNotIn(":/data/faces", bot)

    @staticmethod
    def _commands_source() -> Path:
        return _REPO_ROOT / "src/rob_box_telegram/rob_box_telegram/handlers/commands.py"


class TestOwnerListingText(unittest.TestCase):
    def test_unavailable_listing_is_not_empty_base(self):
        from rob_box_supervisor.face_roster_tool import unavailable_listing

        text = format_owner_listing(unavailable_listing())
        self.assertEqual(text, UNAVAILABLE_TEXT)
        self.assertNotIn("пуста", text)

    def test_empty_owner_list_says_empty(self):
        from rob_box_supervisor.face_roster_tool import listing_from_owner

        text = format_owner_listing(listing_from_owner({"people": [], "total": 0}))
        self.assertEqual(text, "Лицевая база пуста.")


if __name__ == "__main__":
    unittest.main()
