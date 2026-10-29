"""#3845: подробный DJ-промт в Telegram (Шифу 22.10, Oakenfold x Stranger Things, 3 тысячи знаков).

Было: темой сета становились ВСЕ слова промта (2154 знака: «странные дела role live coder paul oakenfold …»), числа из
«BPM: 83/4 = ~124.5 BPM» читались как длина сета (3 трека, 240 с), а спецификация (темп, тональность, ноты, таблица MIDI)
молча пропадала. Решает код по структуре реплики (``engine.brief.split_brief``): просьба — первый абзац, остальное —
спецификация; то, что движок не умеет задать, сет называет фразой кода (``source_note``).
"""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.engine.brief import HEAD_MAX_CHARS, split_brief, spec_bpm, unsupported, unsupported_note
from rob_box_mcp_tools.engine.theme_grounding import heard_theme
from rob_box_mcp_tools.engine.tools_v2 import brief_args

from .test_engine_request_music import _tools
from .test_engine_session import _rig

pytestmark = pytest.mark.unit

ASK = "Ты диджей Пауль Оакенфольд и у нас Сегодня вечеринка на тему Очень странные дела"
SPEC = """# ROLE: AI DJ & Live-Coder (Paul Oakenfold x Stranger Things)

## CONTEXT
Ты играешь гибридный сет: прогрессив-транс Пола Окенфолда,
переосмысленный через синтезаторную эстетику Stranger Things (80s analog horror).
BPM: 83/4 = ~124.5 BPM (идеально для progressive trance).
Тональность: C Major / E Phrygian (мрачная, но эйфорическая).

## FORMAT 1: RTTTL (Melodic Lines Only)
; OPENING FX ARP (C5 Major arpeggio)
opening_fx:d=4,o=5,b=124:c,e,g,a,b,a,g,e,c,e,g,a,b,a,g,e,c,e,g,a,b,a,g,e
bass_line:d=4,o=3,b=124:c,e,g,a,b,a,g,e,c,e,g,a,b,a,g,e

## FORMAT 2: MIDI EVENT SPECIFICATION (Full Arrangement)
| Track | StartBeat | Duration | Note(s) | Velocity | CC# | CC_Value | Description |
|-------|-----------|----------|---------|----------|-----|----------|-------------|
| Pad | 0 | 16 | E2,G2,B2 | 40 | 74 | 30 | Opening pad, LPF=320Hz |
| Kick | 0 | 0.5 | C1 | 100 | - | - | BD hit |

## YOUR TASK
1. Проанализируй RTTTL и MIDI таблицу выше.
4. Добавь элементы Oakenfold: filter sweep перед дропом, вокальный чоп, брейкдаун с пэдом.
"""
PROMPT = f"{ASK}\n{SPEC}"


def test_the_ask_is_the_first_paragraph_and_the_rest_is_the_spec():
    brief = split_brief(PROMPT)
    assert brief.head == ASK
    assert brief.spec.startswith("# ROLE") and "BPM" in brief.spec


def test_theme_of_a_long_prompt_is_the_essence_not_the_whole_prompt():
    theme = heard_theme(PROMPT)
    assert theme == heard_theme(ASK) == "странные дела"
    assert len(theme) < 40
    for noise in ("role", "bpm", "rtttl", "midi", "velocity", "128", "124"):
        assert noise not in theme


@pytest.mark.parametrize("phrase", [
    "Робот, ты диджей Пол Окенфолд. Включи сет на тему «Очень странные дела»: мрачный синтвейв восьмидесятых, "
    "аналоговые синтезаторы, эйфорический прогрессив-транс. Стиль синтвейв, пять треков.",
    "[TG] Ты диджей Анакен скайвокер и у нас сегодня имперский слет в клубе",
    "Ты диджей Атлас и у нас сегодня вечеринка любителей кино, играй мелодии из топовых фильмов",
])
def test_a_plain_phrase_is_not_a_brief(phrase):
    assert split_brief(phrase) == split_brief(phrase).__class__(phrase.strip())


def test_a_single_endless_line_is_cut_to_the_ask_at_a_sentence_or_word_boundary():
    sentence = "Включи сет про космос. "
    rambling = sentence + "и ещё " * 200
    brief = split_brief(rambling)
    assert len(brief.head) <= HEAD_MAX_CHARS and brief.head == sentence.strip()
    word_cut = split_brief("слово " * 200)
    assert len(word_cut.head) <= HEAD_MAX_CHARS and word_cut.head.endswith("слово") and word_cut.spec


def test_bpm_is_the_computed_value_not_the_first_number_after_the_label():
    assert spec_bpm("BPM: 83/4 = ~124.5 BPM (идеально)") == 124.5
    assert spec_bpm("BPM: 128") == 128.0
    assert spec_bpm("играй 140 bpm") == 140.0
    assert spec_bpm("темп не задан") is None


def test_unsupported_names_what_the_engine_cannot_set_and_the_note_comes_from_code():
    items = unsupported(SPEC)
    assert items == ("темп 124.5 BPM", "тональность", "ноты и MIDI")
    assert unsupported_note(items) == "Из промта не использую: темп 124.5 BPM, тональность, ноты и MIDI — сет играет по теме и стилю."
    assert unsupported("прогрессив-транс, мрачный, пять треков") == () and unsupported_note(()) == ""


def test_brief_args_take_length_and_style_from_the_ask_and_the_known_styles_from_the_spec():
    text, tracks, style, styles, note = brief_args(PROMPT, 3, "auto", None)
    assert text == ASK and tracks is None, "числа из BPM/таблицы длиной сета не становятся"
    assert "124.5" in note
    text, tracks, style, styles, note = brief_args(f"{ASK}, пять треков\n\n## STYLE\nСтиль синтвейв", None, "auto", None)
    assert tracks == 5 and style == "synthwave" and styles == ["synthwave"] and note == ""
    # стиль, названный в просьбе или самим вызовом, спецификация не перебивает
    assert brief_args(f"{ASK}\n\n# S\nстиль синтвейв", None, "rave", None)[2] == "rave"
    # короткая реплика и вызов без реплики - как были
    assert brief_args(ASK, 7, "auto", ["club"]) == (ASK, 7, "auto", ["club"], "")
    assert brief_args(None, None, None, None) == (None, None, None, None, "")


def test_dj_set_with_the_long_prompt_plays_the_short_theme_and_says_what_it_did_not_use():
    dj, _req = _tools(_rig())
    result = dj.execute(action="start", theme="Очень странные дела", persona="диджей Пауль Оакенфольд",
                        heard_text=PROMPT)
    data = result.data
    assert data["ok"] is True and data["theme"] == "Очень странные дела"
    assert data["tracks"] != 3, "числа из «83/4» и таблицы - не длина сета"
    assert "124.5" in data["source_note"] and "тональность" in data["source_note"]


def test_dj_set_without_a_theme_from_the_llm_takes_the_ask_theme_only():
    dj, _req = _tools(_rig())
    data = dj.execute(action="start", heard_text=PROMPT).data
    assert data["theme"] == "странные дела"
    dj, _req = _tools(_rig())
    assert "source_note" not in dj.execute(action="start", heard_text=ASK).data


# -- марафон-4 n605: служебные слова реплики не подменяют тему LLM -----------------------------------------------------

#: Дословно из ``.github/e2e/scenarios/night/night_marathon_act6_dj_booth_v1.json`` (n605_dj_set_plan). Было:
#: ``[dj_set] тема LLM не из текущей реплики → из реплики: LLM='ночная дискотека' → 'объявляй каждый'``.
N605 = "Робот, поставь сет из трёх треков, объявляй каждый."


def test_n605_announce_each_track_is_a_way_to_serve_the_set_not_its_theme():
    from rob_box_mcp_tools.engine.theme_grounding import grounded_theme

    assert heard_theme(N605) == ""
    assert grounded_theme("ночная дискотека", N605) == ("ночная дискотека", False)


def test_n605_dj_set_keeps_the_llm_theme_and_the_length_from_the_phrase(caplog):
    import logging

    dj, _req = _tools(_rig())
    with caplog.at_level(logging.WARNING):
        data = dj.execute(action="start", theme="ночная дискотека", heard_tracks=3, heard_text=N605).data
    assert data["theme"] == "ночная дискотека" and data["tracks"] == 3
    assert not any("тема LLM не из текущей реплики" in r.getMessage() for r in caplog.records)
