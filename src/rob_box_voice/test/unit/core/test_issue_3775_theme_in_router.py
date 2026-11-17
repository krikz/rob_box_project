"""Issue #3775 — «сочини новые треки под тему X» — сет генерации с темой.

Живой прогон 21.10: «сочини новые треки под тему …» → ``generate_music(kind=instrumental)`` — один трек вместо сета.
«Ты диджей Воландеморт и у нас сегодня сэт для …» → ``dj_set`` без темы в вызове — так задумано (07.10): тему из
свободных слов выделяет ``dj_set`` из скрытого ``heard_text`` одной функцией (``theme_grounding.heard_theme``) — это
проверяет ``rob_box_mcp_tools/test/test_issue_3775_theme_reaches_generation.py``; потеря темы была в тегах заказа.
"""

from __future__ import annotations

import pytest

from rob_box_voice.core.media_command_grammar import MediaIntent, parse_media_command
from rob_box_voice.core.media_router import MediaRouter, MediaState

DJ_PHRASE = "Ты диджей Воландеморт и у нас сегодня сэт для пленников прибывших в Эскобар"
SET_PHRASE = "сочини новые треки под тему Ты Воландеморт и сэт для пленников прибывших в Эскобар"


def _calls(text):
    plan = MediaRouter().route(text, MediaState())
    return plan, [(c.name, c.arguments) for c in plan.tool_calls]


def test_dj_phrase_leaves_theme_to_dj_set():
    """Тему из свободных слов выделяет ``dj_set``, а не шаблон команды: в вызове темы нет."""
    _plan, calls = _calls(DJ_PHRASE)
    assert calls == [("dj_set", {"action": "start", "persona": "диджей Воландеморт"})]


@pytest.mark.parametrize("text, theme", [
    (SET_PHRASE, "воландеморт и пленников прибывших эскобар"),
    ("[TG] " + SET_PHRASE, "воландеморт и пленников прибывших эскобар"),
    ("сочини сет про космос", "космос"),
    ("сгенерируй треки про дождь", "дождь"),
])
def test_compose_many_tracks_is_a_generated_set(text, theme):
    command = parse_media_command(text)
    assert command.intent is MediaIntent.GENERATE and command.kind == "set"
    _plan, calls = _calls(text)
    assert calls == [("dj_set", {"action": "start", "theme": theme})]


@pytest.mark.parametrize("text, kind", [
    ("сочини трек про стрижей и ежей", "instrumental"),
    ("сочини песню про робота", "robot_song"),
    ("сгенерируй музыку", "instrumental"),
])
def test_single_track_and_song_stay_generate_music(text, kind):
    _plan, calls = _calls(text)
    assert calls[0][0] == "generate_music" and calls[0][1]["kind"] == kind
