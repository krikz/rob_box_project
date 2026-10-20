"""ADR-0153 S4 — lo-fi выбирает грамматика роутера (код): «лоуфай сет на тему X» → ``dj_set(style=lofi)``, «лоу-фай»
и «lo fi» (STT пишет и через дефис, и врозь), «чилл». Слова стиля темой не становятся; «чили», «лофт» — не стиль.
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


@pytest.mark.parametrize("text,theme", [
    ("Робот, включи лоуфай сет на тему дождливый вечер", "дождливый вечер"),
    ("Робот, включи лоуфай сет на тему Моцарт", "моцарт"),
    ("включи лоу-фай сет на тему город", "город"),
    ("включи лоу фай про котов", "котов"),
    ("давай lo-fi сет", None),
    ("включи чилл сет на тему море", "море"),
])
def test_lofi_words_start_a_lofi_set_and_stay_out_of_the_theme(text, theme):
    command = parse_media_command(text)
    assert command.intent is MediaIntent.DJ and command.style == "lofi", command
    args = _dj_args(text)
    assert args["action"] == "start" and args["style"] == "lofi"
    assert (args.get("theme") or "").lower() == (theme or ""), args


@pytest.mark.parametrize("text,theme", [("включи сет на тему чили", "чили"), ("включи сет на тему лофт", "лофт")])
def test_words_close_to_lofi_words_stay_the_theme(text, theme):
    args = _dj_args(text)
    assert "style" not in args and args.get("theme") == theme
