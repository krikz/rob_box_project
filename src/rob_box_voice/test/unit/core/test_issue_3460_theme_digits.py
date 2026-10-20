"""#3460 (приёмка 06.10): цифры в теме сета не вырезаются — «Mozart40» -> «mozart 40», «Mambo Nr 5» -> «mambo nr 5»."""

from __future__ import annotations

import pytest

from rob_box_voice.core.media_command_grammar import MediaIntent, parse_media_command
from rob_box_voice.core.media_router import MediaRouter, MediaState, dj_started_text


@pytest.mark.parametrize("text,theme", [
    ("включи диджей сет на тему Mozart40", "mozart 40"),
    ("включи диджей сет на тему Mambo Nr 5", "mambo nr 5"),
    ("включи диджей сет про Mambo Nr 5", "mambo nr 5"),
    ("включи рейв на тему Mozart40", "mozart 40"),
    ("включи рейв-сет про Mambo Nr 5", "mambo nr 5"),
    ("включи диджей сет на тему космос", "космос"),
])
def test_digits_stay_in_the_theme(text, theme):
    command = parse_media_command(text)
    assert command.intent is MediaIntent.DJ and command.set_theme == theme


@pytest.mark.parametrize("text,name", [
    ("сыграй 1812 Overture", "1812 overture"),  # 06.10: роутер извлёк name='overture'
    ("включи Mozart40", "mozart 40"),
    ("поставь горный король", "горный король"),
])
def test_digits_stay_in_the_named_title(text, name):
    command = parse_media_command(text)
    assert command.intent is MediaIntent.PLAY_NAMED and command.name == name


def test_theme_reaches_dj_set_and_tts_phrase_with_digits():
    plan = MediaRouter().route("включи диджей сет на тему Mambo Nr 5", MediaState())
    assert plan is not None and plan.tool_calls[0].name == "dj_set"
    assert plan.tool_calls[0].arguments["theme"] == "mambo nr 5"
    assert dj_started_text("", "mambo nr 5") == "Включаю диджей-сет. Тема — mambo nr 5."
