"""Тема DJ-сета из реплики — чисто и одинаково на обоих путях (живой прогон 07.10 на роботе).

* «Робот, ты диджей 8битный и нас сегодня вечеринка любителей денди и классической музыки замути сэт на 30 минут» —
  тема была ``'битный и денди и классической'`` (``не найдено: «битный»``): «8битный» делился на «8»+«битный», имя
  диджея не отрезалось.
* Та же фраза с «и у нас сегодня» шла через грамматику роутера (``_dj_head_command``): тема — сырой хвост
  ``не найдено: «классической музыки замути сэт»``.
* «Ты диджей Атлас и у нас сегодня вечеринка любителей кино, играй мелодии из топовых фильмов» (~07:04 UTC) — тема
  сырой хвостом, ``не найдено: «вечеринка любителей кино», «играй мелодии…»``, сет играл случайный пул; промпт
  реплики диджея — «Ты диджей диджей Атлас», TTS — «Тема — вечеринка любителей кино, играй мелодии и…».

Теперь тему из свободных слов реплики выделяет одна функция ``theme_grounding.heard_theme`` (dj_set по скрытому
``heard_text``), грамматика её не режет; «кино/фильмы» — жанр каталога ``movie``; персона — одним видом
(``dj_line.persona_title``). Тесты — на поведение модулей и поиск по настоящему архиву, не на текст промпта.
"""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary, tagged
from rob_box_mcp_tools.engine.search import genre_of, theme_parts, theme_search
from rob_box_mcp_tools.engine.theme_grounding import grounded_theme, heard_theme
from rob_box_music.dj_line import LineFacts, persona_title, prompt
from rob_box_voice.core.media_command_grammar import parse_media_command

from .test_engine_request_music import _tools
from .test_engine_session import _rig

pytestmark = pytest.mark.unit

LIVE_I_NAS = ("Робот, ты диджей 8битный и нас сегодня вечеринка любителей денди и классической музыки "
              "замути сэт на 30 минут")
LIVE_U_NAS = ("Робот, ты диджей 8битный и у нас сегодня вечеринка любителей денди и классической музыки "
              "замути сэт на 30 минут")
ATLAS = "Ты диджей Атлас и у нас сегодня вечеринка любителей кино, играй мелодии из топовых фильмов"
DENDY_CLASSIC = "денди и классической музыки"


@pytest.mark.parametrize("phrase,theme", [
    (LIVE_I_NAS, DENDY_CLASSIC),
    (LIVE_U_NAS, DENDY_CLASSIC),
    ("Ты диджей 8битный и нас сегодня вечеринка любителей денди и классической музыки, замути сэт на 30 минут",
     DENDY_CLASSIC),
    (ATLAS, "кино, фильмов"),
    # уже работавшие фразы тем (06.10) — не сломаны
    ("ретро 8-бит: Mario, Tetris, Aladdin, Contra замути сэт", "mario, tetris, aladdin, contra"),
    ("ретро 8-бит Mario Tetris Aladdin Contra замути сэт на 30 минут", "mario tetris aladdin contra"),
    ("сыграй интерстеллар сэт", "интерстеллар"),
    ("включи сет на 3 трека: Марио, Тетрис, Зельда", "марио, тетрис, зельда"),
    ("замути сэт про марио и тетрис", "марио и тетрис"),
    ("сыграй Mozart40 сэт", "mozart 40"),
    ("[TG] Ты диджей Анакен скайвокер и у нас сегодня имперский слет в клубе", "имперский слет клубе"),
    ("ты диджей Робокс на тему детский праздник", "детский праздник"),
    # имя диджея без запятой и стиль — не тема (#3476)
    ("ты диджей Снупдог у нас сегодня вечеринка в стиле рейв", ""),
    ("Робот, замути сэт на 30 минут", ""),
])
def test_heard_theme_keeps_only_theme_words(phrase, theme):
    assert heard_theme(phrase) == theme


@pytest.mark.parametrize("noise", ["8", "битный", "диджей", "атлас", "замути", "сэт", "вечеринка", "любителей",
                                   "играй", "топовых", "30", "минут", "сегодня", "нас"])
def test_no_persona_style_request_or_length_word_in_the_theme(noise):
    for phrase in (LIVE_I_NAS, LIVE_U_NAS, ATLAS):
        assert noise not in heard_theme(phrase).replace(",", " ").split(), phrase


def test_both_paths_give_one_theme():
    """Путь команды: грамматика отдаёт DJ без темы (тему выделит dj_set из heard_text); путь LLM — тема из истории,
    заменённая темой реплики. Обе — ``heard_theme`` реплики."""
    for phrase in (LIVE_I_NAS, LIVE_U_NAS, ATLAS):
        command = parse_media_command(phrase)
        assert command.closed and command.set_theme == "", phrase
        dj, _req = _tools(_rig())
        by_code = dj.execute(action="start", persona=command.persona, heard_text=phrase).data
        dj, _req = _tools(_rig())
        by_llm = dj.execute(action="start", theme="Увертюра 1812, мегасет: Марио, Аладдин", persona="Атлас",
                            heard_text=phrase).data
        assert by_code["theme"] == by_llm["theme"] == heard_theme(phrase), phrase


@pytest.fixture(scope="module")
def library(tmp_path_factory):
    return RtttlLibrary(db_path=str(tmp_path_factory.mktemp("clean_theme") / "voice_memory.db"))


def test_dendy_and_classical_are_two_found_parts(library):
    assert theme_parts(library, DENDY_CLASSIC) == ["денди", "классической музыки"]
    hits = theme_search(library, DENDY_CLASSIC)
    assert hits.missing == () and len(hits.parts) == 2
    classical = {r["name"] for r in tagged(library, "classical")}
    assert set(hits.parts[1]) <= classical | {"1812over", "1812over_2"}


def test_film_theme_plays_film_melodies_and_nothing_is_missing(library):
    theme = heard_theme(ATLAS)
    assert genre_of("кино") == genre_of("фильмов") == "movie"
    hits = theme_search(library, theme)
    movies = {r["name"] for r in tagged(library, "movie")}
    assert hits.missing == ()
    assert hits.names and set(hits.names) <= movies


@pytest.mark.parametrize("persona", ["диджей Атлас", "Атлас", "DJ Атлас", "диджей диджей Атлас", " Атлас "])
def test_persona_has_one_form(persona):
    assert persona_title(persona) == "диджей Атлас"


def test_line_prompt_does_not_double_the_dj_word():
    facts = LineFacts(track_no=1, tracks=10, theme="кино", hook=None, energy=3)
    system, _user = prompt(facts, "диджей Атлас")
    assert system.startswith("Ты диджей Атлас.") and "диджей диджей" not in system
    assert prompt(facts, None)[0].startswith("Ты диджей робота.")


def test_dj_set_stores_the_normalized_persona():
    rig = _rig()
    dj, _req = _tools(rig)
    assert dj.execute(action="start", persona="Атлас", heard_text=ATLAS).data["theme"] == "кино, фильмов"
    assert grounded_theme(None, ATLAS) == ("кино, фильмов", True)
    assert dj._session.dj_fields(1)["persona"] == "диджей Атлас"
