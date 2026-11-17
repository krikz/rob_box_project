"""Заказ сета словами человека запускает ``dj_set`` код, без LLM (ADR-0148; живой замер 06.10 19:08 UTC).

«Робот, ты диджей 8битный и нас сегодня вечеринка любителей денди и классической музыки замути сэт на 30 минут»:
STT 19:08:29.7 → ``process_input`` 30.2 → LLM подгрузка скилла 35.5 → LLM ``dj_set`` 44.4 → ``[music] started``
46.7 — 17 с тишины. Грамматика видела DJ-запрос, но «и нас» (STT вместо «у нас») не выделило тему, реплика была
«не закрыта» и ушла в LLM.

Теперь: просьба одна — «замути сэт», остальное — тема; роутер зовёт ``dj_set(start, persona, tracks)`` сам. Тему
из слов реплики выделяет ``dj_set`` по скрытому ``heard_text`` — той же функцией, что и на пути LLM, поэтому у
команды ТЕ ЖЕ тема и длина.
"""

from __future__ import annotations

import pytest

from rob_box_voice.core.media_command_grammar import is_spoken_theme_set, parse_media_command
from rob_box_voice.core.media_router import MediaRouter, MediaState

LIVE_1908 = ("Робот, ты диджей 8битный и нас сегодня вечеринка любителей денди и классической музыки "
             "замути сэт на 30 минут")
RETRO_1530 = "ретро 8-бит Mario Tetris Aladdin Contra замути сэт на 30 минут"


def _route(text: str, media: MediaState = MediaState()):
    return MediaRouter().route(text, media)


def test_live_phrase_is_routed_to_dj_set_by_code():
    plan = _route(LIVE_1908)
    assert plan is not None, "реплика ушла бы в LLM (подгрузка скилла + dj_set, ~14 с)"
    assert [c.name for c in plan.tool_calls] == ["dj_set"]
    # ADR-0153 S2: «8битный» — стиль chiptune (код), темой по-прежнему не становится
    assert plan.tool_calls[0].arguments == {"action": "start", "persona": "диджей 8битный", "style": "chiptune",
                                            "tracks": 24}
    assert plan.cancel_inflight and plan.confirm_started
    assert plan.say_ok == "Я диджей 8битный, включаю сет."  # фраза кода, по событию started


# Та же тема, что на пути LLM (денди + классика), — на настоящем dj_set с этими аргументами и heard_text:
# rob_box_mcp_tools/test/test_dj_set_theme_from_phrase.py (здесь node/conftest подменяет rob_box_mcp_tools моком).


def test_set_order_without_persona_is_routed_too():
    plan = _route(RETRO_1530)
    assert plan is not None
    assert plan.tool_calls[0].arguments == {"action": "start", "style": "chiptune", "tracks": 24}  # «8-бит» — стиль


@pytest.mark.parametrize("text", [
    "ты диджей вася замути сэт и поставь still dre",   # вторая просьба
    "ты диджей вася а может замути сэт?",             # вопрос
    "Робот, ты диджей Пёс, сыграй Still Dre и Next Episode",
    "Робот, расскажи анекдот",
    "ты диджей вася вечеринка у меня, я не хочу сэт",  # отрицание / ссылка на себя
])
def test_not_a_plain_set_order_still_goes_to_llm(text):
    assert _route(text) is None


@pytest.mark.parametrize("text", [
    "замути сэт и включи погромче",  # вторая просьба
    "замути сэт погромче",           # громкость
    "не замути сэт",                 # отрицание
    "замути сэт?",                   # вопрос
    "расскажи про сэт",              # не просьба запустить
])
def test_spoken_theme_set_refuses_anything_beyond_one_set_order(text):
    assert is_spoken_theme_set(text) is False


def test_spoken_theme_set_accepts_one_set_order_with_theme_words():
    assert is_spoken_theme_set("давай замути мне сэт про космос и котов") is True
    assert parse_media_command(LIVE_1908).closed is True


def test_command_over_the_llm_turn_does_not_cancel_it():
    """Ответ поверх хода LLM (без отмены: «что играет»). Громкость, которая была таким примером, теперь Tier-1
    ``rob_box_dialog.grammar`` (ADR-0150 PR-6) и плана роутера не имеет."""
    plan = _route("что сейчас играет", MediaState(music_playing=True, track_name="Still Dre"))
    assert plan is not None and not plan.cancel_inflight


# ── 07.10: тема «у нас сегодня …» — не грамматикой ────────────────
LIVE_U_NAS = ("Робот, ты диджей 8битный и у нас сегодня вечеринка любителей денди и классической музыки "
              "замути сэт на 30 минут")
ATLAS = "Ты диджей Атлас и у нас сегодня вечеринка любителей кино, играй мелодии из топовых фильмов"


@pytest.mark.parametrize("text,args", [
    (LIVE_U_NAS, {"action": "start", "persona": "диджей 8битный", "style": "chiptune", "tracks": 24}),
    (ATLAS, {"action": "start", "persona": "диджей Атлас"}),
])
def test_spoken_theme_is_not_cut_by_the_grammar(text, args):
    """Раньше: theme='вечеринка любителей денди и классической музыки замути сэт' / '… играй мелодии из топовых
    фильмов' — сырой хвост в поиск и в TTS. Тему выделяет dj_set из heard_text (``theme_grounding.heard_theme``)."""
    plan = _route(text)
    assert plan is not None and plan.tool_calls[0].arguments == args
    assert "Тема" not in plan.say_ok and "диджей диджей" not in plan.say_ok


def test_named_play_plan_does_not_cancel_before_the_find_3525():
    """Заказ по имени (cancel_inflight=False до находки) — план роутера с ``play_name``."""
    plan = _route("Робот, поставь к Элизе")
    assert plan is not None and plan.play_name and not plan.cancel_inflight
