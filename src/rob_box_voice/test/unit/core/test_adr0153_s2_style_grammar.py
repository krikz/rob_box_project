"""ADR-0153 S2 — synthwave и chiptune выбирает грамматика роутера (код): «синтвейв сет на тему X» →
``dj_set(style=synthwave)``, «8-битный/восьмибитный сет» → ``dj_set(style=chiptune)``.

«8-бит», «8битный» (#3476/#3492) по-прежнему не тема сета — и теперь выбирают стиль chiptune.
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
    ("включи синтвейв сет на тему киберпанк", "synthwave", "киберпанк"),
    ("включи синтвейв", "synthwave", None),
    ("давай ретровейв на тему ночной город", "synthwave", "ночной город"),
    ("включи аутран сэт про машины", "synthwave", "машины"),
    ("включи восьмибитный сет на тему марио", "chiptune", "марио"),
    ("включи 8-битный сет на тему марио", "chiptune", "марио"),
    ("включи 8 бит на тему тетрис", "chiptune", "тетрис"),
    ("давай чиптюн про денди", "chiptune", "денди"),
    ("включи диджей сет на тему космос в стиле синтвейв", "synthwave", "космос"),
])
def test_style_words_start_a_set_of_the_style_and_stay_out_of_the_theme(text, style, theme):
    command = parse_media_command(text)
    assert command.intent is MediaIntent.DJ and command.style == style, command
    args = _dj_args(text)
    assert args["action"] == "start" and args["style"] == style and args.get("theme") == theme


def test_eight_bit_dj_persona_phrase_picks_chiptune():
    """Живая фраза 06.10: «ты диджей 8битный …» — персона остаётся, стиль сета — chiptune."""
    command = parse_media_command("ты диджей 8битный, у нас сегодня вечеринка")
    assert command.intent is MediaIntent.DJ and command.style == "chiptune"


@pytest.mark.parametrize("text", ["что такое синтвейв", "расскажи про чиптюн", "сыграй 1812 overture"])
def test_style_word_in_another_request_is_not_a_style_set(text):
    command = parse_media_command(text)
    assert not (command.intent is MediaIntent.DJ and command.style), command
