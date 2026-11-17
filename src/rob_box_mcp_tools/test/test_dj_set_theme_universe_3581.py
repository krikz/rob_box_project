"""#3581, живой сет Шифу 12.10 (set31919).

(а) «ты диджей анакен скайвокер и у нас сегодня веселье на звезде смерте погнали» → тема «веселье звезде смерте» без
персоны; хуки Celebration/Happy/Girls Just Wanna, Star Wars на 8–9 местах.
(б) следом «продолжай сэт» → LLM ``dj_set(theme='звезда смерти вечеринка тёмная сторона силы Анакин С…')`` → гард
«тема LLM не из текущей реплики» сделал темой слово «продолжай» → хуки Carry On / Keep It Up.

Решает код (ADR-0148): персонаж/место одной вселенной — одна тема (``knowledge.UNIVERSES``), «веселье» — настроение,
не тема (ADR-0157 §2.1); «продолжай сэт» без новой темы — тема последнего сета, слово просьбы темой не бывает.
Хуки — по настоящему архиву RTTTL.
"""

from __future__ import annotations

import logging

import pytest

from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
from rob_box_mcp_tools.engine.search import theme_search
from rob_box_mcp_tools.engine.theme_grounding import continued, grounded_theme, heard_theme, set_theme

from .test_engine_request_music import _tools
from .test_engine_session import _rig

pytestmark = pytest.mark.unit

LIVE_A = "ты диджей анакен скайвокер и у нас сегодня веселье на звезде смерте погнали"
LIVE_A_PERSONA = "диджей анакен скайвокер"
LIVE_B = "продолжай сэт"
LIVE_B_LLM_THEME = "звезда смерти вечеринка тёмная сторона силы Анакин С"
STAR_WARS = "звёздные войны"


@pytest.fixture(scope="module")
def library(tmp_path_factory):
    return RtttlLibrary(db_path=str(tmp_path_factory.mktemp("u3581") / "voice_memory.db"))


def test_live_phrase_a_theme_is_the_universe_and_fun_is_mood():
    slots = set_theme(heard_theme(LIVE_A), LIVE_A_PERSONA)
    assert (slots.theme, slots.mood, slots.universe) == (STAR_WARS, "веселье", "starwars")


def test_live_phrase_a_hooks_are_star_wars_first(library):
    names = theme_search(library, set_theme(heard_theme(LIVE_A), LIVE_A_PERSONA).theme).names
    assert names and all(n.startswith("starwars") for n in names[:6])
    assert not any(n.startswith(("celebrat", "happy", "girlsjus")) for n in names)


def test_persona_alone_gives_the_theme_but_an_own_theme_wins():
    assert set_theme("", "диджей Дарт Вейдер").theme == STAR_WARS
    assert set_theme("веселье", "диджей Анакин").theme == STAR_WARS
    assert set_theme("космос", "диджей Анакин").theme == "космос"


def test_continue_word_is_not_a_theme_and_the_guard_keeps_the_llm_theme():
    assert heard_theme(LIVE_B) == ""
    assert grounded_theme(LIVE_B_LLM_THEME, LIVE_B) == (LIVE_B_LLM_THEME, False)


def test_continue_takes_the_last_set_only_without_a_new_theme():
    last = (STAR_WARS, LIVE_A_PERSONA)
    assert continued("Робот, продолжай сэт", last) == last
    assert continued("давай ещё сет", last) == last
    assert continued("давай ещё сет на тему космос", last) is None
    assert continued("включи диджей сет на тему космос", last) is None
    assert continued(LIVE_B, None) is None


@pytest.mark.parametrize("theme", ["космос", "сет на тему праздник", "денди-стиль: Тетрис, Контра", "интерстеллар"])
def test_themes_without_universe_or_mood_stay_word_for_word(theme):
    assert set_theme(theme, None).theme == theme


def test_mood_next_to_theme_words_goes_to_mood():
    slots = set_theme("праздничный космос", None)
    assert (slots.theme, slots.mood) == ("космос", "праздничный")


def test_dj_set_live_a_then_continue_keeps_star_wars_and_persona(caplog):
    dj, _req = _tools(_rig())
    first = dj.execute(action="start", persona=LIVE_A_PERSONA, heard_text=LIVE_A).data
    assert first["theme"] == STAR_WARS
    with caplog.at_level(logging.INFO):
        second = dj.execute(action="start", theme=LIVE_B_LLM_THEME, heard_text="Робот, продолжай сэт").data
    assert second["ok"] and second["theme"] == STAR_WARS
    theme, persona = dj._last_dj  # персона первого сета осталась: «продолжай» её не называл
    assert theme == STAR_WARS and "анакен" in (persona or "").lower()
    assert any("продолжить сет без новой темы" in r.getMessage() for r in caplog.records)
    assert not any("→ из реплики" in r.getMessage() and "продолжай" in r.getMessage() for r in caplog.records)


def test_dj_set_continue_with_a_new_theme_is_a_new_theme():
    dj, _req = _tools(_rig())
    dj.execute(action="start", persona=LIVE_A_PERSONA, heard_text=LIVE_A)
    assert dj.execute(action="start", heard_text="давай ещё сет на тему космос").data["theme"] == "космос"


def test_dj_set_continue_without_a_last_set_keeps_the_llm_theme_not_the_word():
    dj, _req = _tools(_rig())
    data = dj.execute(action="start", theme="космос", heard_text=LIVE_B).data
    assert data["theme"] == "космос"


def test_dj_set_ordinary_space_set_is_unchanged():
    dj, _req = _tools(_rig())
    assert dj.execute(action="start", heard_text="Робот, включи диджей сет на тему космос").data["theme"] == "космос"
