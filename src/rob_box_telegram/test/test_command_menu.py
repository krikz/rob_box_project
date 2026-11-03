"""Меню команд бота: таблица описаний совпадает с обработчиками и соблюдает правила Bot API (#4243)."""

from __future__ import annotations

from rob_box_telegram import command_menu as cm


def _handler_names():
    # handlers/commands.py тянет telegram: имена берём из исходника, чтобы тест не требовал библиотеки
    import pathlib
    import re

    source = (pathlib.Path(__file__).resolve().parents[1] / "rob_box_telegram" / "handlers" / "commands.py")
    return set(re.findall(r"^async def (\w+)_handler\(", source.read_text(encoding="utf-8"), re.M))


def test_every_handler_has_a_description_and_no_description_is_orphaned():
    assert _handler_names() == set(cm.MENU)


def test_names_and_descriptions_follow_bot_api_rules():
    for name, entry in cm.MENU.items():
        assert cm.NAME_RE.match(name), name
        assert 1 <= len(entry.description) <= cm.DESCRIPTION_MAX, name


def test_menu_lists_public_commands_and_hides_operator_and_service_ones():
    names = [name for name, _ in cm.menu_commands()]
    assert "help" in names and "photo" in names
    for hidden in ("repl", "cmd", "operator", "myid"):
        assert hidden not in names and hidden in cm.MENU
    assert len(names) <= 100  # потолок Bot API на меню


def test_menu_is_published_from_the_table_and_a_failure_only_warns():
    import asyncio
    from unittest.mock import MagicMock

    class Bot:
        def __init__(self, error=None):
            self.sent, self.error = None, error

        async def set_my_commands(self, commands):
            if self.error:
                raise self.error
            self.sent = commands

    ok = Bot()
    assert asyncio.run(cm.publish_menu(ok, MagicMock())) is True and ok.sent == cm.menu_commands()
    log = MagicMock()
    assert asyncio.run(cm.publish_menu(Bot(RuntimeError("x")), log)) is False
    assert "RuntimeError" in log.warning.call_args.args[0]
