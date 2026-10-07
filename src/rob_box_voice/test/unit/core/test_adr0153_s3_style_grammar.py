"""ADR-0153 S3 — breaks и dnb выбирает грамматика роутера (код): «брейкбит сет на тему X» → ``dj_set(style=breaks)``,
«драм-н-бейс сет на тему X» (STT пишет и словами врозь) → ``dj_set(style=dnb)``. Слова стиля темой не становятся.
"""

from __future__ import annotations

import pytest

from rob_box_voice.core.media_command_grammar import MediaIntent, parse_media_command
from rob_box_voice.core.media_router import MediaRouter, MediaState

V2 = MediaRouter()


def _dj_args(text: str):
    plan = V2.route(text, MediaState())
    assert plan is not None and [c.name for c in plan.tool_calls] == ["dj_set"], text
    return plan.tool_calls[0].arguments


@pytest.mark.parametrize("text,style,theme", [
    ("Робот, включи брейкбит сет на тему космос", "breaks", "космос"),
    ("включи брейк бит сет на тему город", "breaks", "город"),
    ("давай брейкс про котов", "breaks", "котов"),
    ("Робот, включи драм-н-бейс сет на тему киберпанк", "dnb", "киберпанк"),
    ("включи драм н бейс на тему лес", "dnb", "лес"),
    ("поставь драм энд бейс сет", "dnb", None),
    ("давай днб про котов", "dnb", "котов"),
    ("включи джангл", "dnb", None),
])
def test_style_words_start_a_set_of_the_style_and_stay_out_of_the_theme(text, style, theme):
    command = parse_media_command(text)
    assert command.intent is MediaIntent.DJ and command.style == style, command
    args = _dj_args(text)
    assert args["action"] == "start" and args["style"] == style and args.get("theme") == theme


@pytest.mark.parametrize("text,theme", [("включи сет на тему джунгли", "джунгли"),
                                        ("включи брейкданс сет", "брейкданс")])
def test_words_close_to_style_words_stay_the_theme(text, theme):
    args = _dj_args(text)
    assert "style" not in args and args.get("theme") == theme


@pytest.mark.parametrize("text", ["расскажи про драм-н-бейс", "что такое брейкбит"])
def test_style_word_in_another_request_is_not_a_style_set(text):
    command = parse_media_command(text)
    assert not (command.intent is MediaIntent.DJ and command.style), command
