"""ADR-0153 S6 — джаз выбирает грамматика роутера (код): «джаз сет на тему X» → ``dj_set(style=jazz)``, «джазовый»,
«свинг», «бибоп». Слова стиля темой не становятся; «свинья», «джаггер» — не стиль, а тема."""

from __future__ import annotations

import pytest

from rob_box_voice.core.media_command_grammar import MediaIntent, parse_media_command
from rob_box_voice.core.media_router import MediaRouter, MediaState

V2 = MediaRouter()


def _dj_args(text: str):
    plan = V2.route(text, MediaState())
    assert plan is not None and [c.name for c in plan.tool_calls] == ["dj_set"], text
    return plan.tool_calls[0].arguments


@pytest.mark.parametrize("text,theme", [
    ("Робот, включи джаз сет на тему Моцарт", "моцарт"),
    ("Робот, включи джаз сет на тему в пещере горного короля", "в пещере горного короля"),
    ("включи джазовый сет на тему осень", "осень"),
    ("включи свинг сет на тему тридцатые", "тридцатые"),
    ("давай джаз сет", None),
])
def test_jazz_words_start_a_jazz_set_and_stay_out_of_the_theme(text, theme):
    command = parse_media_command(text)
    assert command.intent is MediaIntent.DJ and command.style == "jazz", command
    args = _dj_args(text)
    assert args["action"] == "start" and args["style"] == "jazz"
    assert (args.get("theme") or "").lower() == (theme or ""), args


@pytest.mark.parametrize("text,theme", [
    ("включи сет на тему свинья", "свинья"),
    ("включи сет на тему джаггер", "джаггер"),
])
def test_words_close_to_jazz_words_stay_the_theme(text, theme):
    args = _dj_args(text)
    assert "style" not in args and args.get("theme") == theme
