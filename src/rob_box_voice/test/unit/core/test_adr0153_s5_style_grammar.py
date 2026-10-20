"""ADR-0153 S5 — рок выбирает грамматика роутера (код): «рок сет на тему X» → ``dj_set(style=rock)``, «гранж»,
«хард-рок», «рок-н-ролл» (STT пишет через дефис и врозь). Слова стиля темой не становятся; «роковой», «рокки»,
«барокко», «рокот» — не стиль, а тема."""

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
    ("Робот, включи рок сет на тему в пещере горного короля", "в пещере горного короля"),
    ("Робот, включи гранж сет на тему космос", "космос"),
    ("включи хард-рок сет на тему гонки", "гонки"),
    ("включи рок-н-ролл сет на тему пятидесятые", "пятидесятые"),
    ("включи рок н ролл сет", None),
])
def test_rock_words_start_a_rock_set_and_stay_out_of_the_theme(text, theme):
    command = parse_media_command(text)
    assert command.intent is MediaIntent.DJ and command.style == "rock", command
    args = _dj_args(text)
    assert args["action"] == "start" and args["style"] == "rock"
    assert (args.get("theme") or "").lower() == (theme or ""), args


@pytest.mark.parametrize("text,theme", [
    ("включи сет на тему роковая любовь", "роковая любовь"),
    ("включи сет на тему рокки", "рокки"),
    ("включи сет на тему барокко", "барокко"),
    ("включи сет на тему рокот моря", "рокот моря"),
])
def test_words_close_to_rock_words_stay_the_theme(text, theme):
    args = _dj_args(text)
    assert "style" not in args and args.get("theme") == theme
