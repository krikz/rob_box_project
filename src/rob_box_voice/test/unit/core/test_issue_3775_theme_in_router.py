"""Issue #3775 — «сочини новые треки под тему X» — сет генерации с темой; фраза сета называет тему.

Живой прогон 21.10: «сочини новые треки под тему …» → ``generate_music(kind=instrumental)`` — один трек вместо сета.
«Ты диджей Воландеморт и у нас сегодня сэт для …» → ``dj_set`` без темы в вызове — так задумано (07.10): тему из
свободных слов выделяет ``dj_set`` из скрытого ``heard_text`` одной функцией (``theme_grounding.heard_theme``) — это
проверяет ``rob_box_mcp_tools/test/test_issue_3775_theme_reaches_generation.py``; потеря темы была в тегах заказа.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from rob_box_voice.core.media_command_grammar import MediaIntent, parse_media_command
from rob_box_voice.core.media_plan_run import _started_text
from rob_box_voice.core.media_router import MediaRouter, MediaState

pytestmark = pytest.mark.unit

DJ_PHRASE = "Ты диджей Воландеморт и у нас сегодня сэт для пленников прибывших в Эскобар"
SET_PHRASE = "сочини новые треки под тему Ты Воландеморт и сэт для пленников прибывших в Эскобар"


def _calls(text):
    plan = MediaRouter().route(text, MediaState())
    return plan, [(c.name, c.arguments) for c in plan.tool_calls]


def test_dj_phrase_leaves_theme_to_dj_set_and_phrase_names_its_theme():
    """Тему называет результат ``dj_set`` (фраза кода по факту), а не шаблон команды."""
    plan, calls = _calls(DJ_PHRASE)
    assert calls == [("dj_set", {"action": "start", "persona": "диджей Воландеморт"})]
    assert _started_text(plan, {"theme": "пленников прибывших эскобар"}) == (
        "Я диджей Воландеморт, включаю сет. Тема — пленников прибывших эскобар.")


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


def _plan(arguments):
    return SimpleNamespace(tool_calls=(SimpleNamespace(name="dj_set", arguments=arguments),),
                           say_ok="Включаю диджей-сет.")


def test_generated_set_phrase_names_theme():
    """Фраза пути сета генерации (код) — плюс тема сета из результата, если фраза её ещё не назвала."""
    message = "Сет «клуб» на 10 треков: 0 из сочинённого раньше, остальные допишу по ходу."
    gen = {"theme": "пленников прибывших эскобар", "source_message": message}
    assert _started_text(_plan({"action": "start"}), gen) == (
        gen["source_message"] + " Тема — пленников прибывших эскобар.")
    rock = {"theme": "рок", "source_message": "Рок-сет на 3 трека: из сочинённого раньше."}
    assert _started_text(_plan({"action": "start"}), rock) == rock["source_message"]  # тема уже названа
