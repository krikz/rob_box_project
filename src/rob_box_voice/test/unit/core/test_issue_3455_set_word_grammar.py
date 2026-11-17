"""Issue #3455 — «сыграй интерстеллар сэт»: слово «сет/сэт» без «диджей» — DJ-сет с темой, решает грамматика.

Живой прогон 06.10 06:56Z: «сыграй интерстеллар сэт» → ``play_named('интерстеллар сэт')`` → мимо базы мелодий →
LLM. Поведение: фраза → интент, тема и аргументы ``dj_set``; заказ по имени, смена темы посреди сета и стиль —
как были.
"""

from __future__ import annotations

import pytest

from rob_box_voice.core.media_command_grammar import MediaIntent, parse_media_command
from rob_box_voice.core.media_router import MediaRouter, MediaState

V2 = MediaRouter()


def _dj_args(text: str, media: MediaState = MediaState()):
    plan = V2.route(text, media)
    assert plan is not None and [c.name for c in plan.tool_calls] == ["dj_set"], text
    return plan.tool_calls[0].arguments


@pytest.mark.parametrize("text,theme", [
    ("сыграй интерстеллар сэт", "интерстеллар"),
    ("[TG] сыграй интерстеллар сэт", "интерстеллар"),
    ("сыграй интерстеллар сет", "интерстеллар"),
    ("поставь космический сэтик", "космический"),
    ("включи сэт на тему космос", "космос"),
    ("сэт на тему интерстеллар", "интерстеллар"),
    ("сэтик про котов", "котов"),
    ("запусти сэт про роботов", "роботов"),
    ("сыграй сет на тему космос пожалуйста", "космос"),
    ("включи диджей сэт на тему космос", "космос"),
    ("сыграй dj set про космос", "космос"),
])
def test_set_word_starts_a_dj_set_with_the_theme(text, theme):
    command = parse_media_command(text)
    assert command.intent is MediaIntent.DJ and command.set_theme == theme
    assert _dj_args(text) == {"action": "start", "theme": theme}


@pytest.mark.parametrize("text", ["включи сэт", "поставь сетик", "давай сет", "сэт"])
def test_set_word_without_theme_starts_a_set(text):
    assert _dj_args(text) == {"action": "start"}


def test_live_phrase_says_code_phrase_only_after_started():
    plan = V2.route("сыграй интерстеллар сэт", MediaState())
    assert plan.confirm_started and plan.say_ok == "Включаю диджей-сет. Тема — интерстеллар."


def test_style_word_goes_to_style_not_theme():
    """#3453: «рейв» — стиль, в тему не попадает."""
    assert _dj_args("сыграй рейв интерстеллар сэт") == {"action": "start", "theme": "интерстеллар", "style": "rave"}
    assert _dj_args("рейв на тему космос") == {"action": "start", "theme": "космос", "style": "rave"}


def test_set_word_mid_set_starts_new_themed_set():
    playing = MediaState(music_playing=True, dj_enabled=True, dj_persona="диджей Вася")
    assert _dj_args("сыграй интерстеллар сэт", playing)["theme"] == "интерстеллар"


@pytest.mark.parametrize("text,name,themed", [
    ("поставь калинку", "калинку", False),
    ("сыграй к элизе", "к элизе", False),
    ("давай тему терминатора", "терминатора", True),  # #3410/#3412: тема посреди сета
])
def test_play_named_and_theme_switch_unchanged(text, name, themed):
    command = parse_media_command(text)
    assert command.intent is MediaIntent.PLAY_NAMED and command.name == name and command.themed is themed


@pytest.mark.parametrize("text,intent", [
    ("сет был классный", MediaIntent.NONE),
    ("какой сет", MediaIntent.NONE),
    ("выключи сэт", MediaIntent.STOP),
    ("хватит сэт", MediaIntent.STOP),
])
def test_set_word_in_other_phrases_is_not_a_start(text, intent):
    assert parse_media_command(text).intent is intent
