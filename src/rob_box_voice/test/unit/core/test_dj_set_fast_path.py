"""Заказ сета словами человека запускает ``dj_set`` код, без LLM (ADR-0148; живой замер 06.10 19:08 UTC).

«Робот, ты диджей 8битный и нас сегодня вечеринка любителей денди и классической музыки замути сэт на 30 минут»:
STT 19:08:29.7 → ``process_input`` 30.2 → LLM ``load_skill`` 35.5 → LLM ``dj_set`` 44.4 → ``[music v2] started``
46.7 — 17 с тишины. Грамматика видела DJ-запрос, но «и нас» (STT вместо «у нас») не выделило тему, реплика была
«не закрыта» и ушла в LLM.

Теперь: просьба одна — «замути сэт», остальное — тема; роутер зовёт ``dj_set(start, persona, tracks)`` сам. Тему
из слов реплики выделяет ``dj_set`` по скрытому ``heard_text`` — той же функцией, что и на пути LLM, поэтому у
команды ТЕ ЖЕ тема и длина. Контекст хода (``heard_text``/``heard_tracks``/``turn_id``) у команды свой
(:func:`command_turn_context`), не прошлого хода LLM.
"""

from __future__ import annotations

import asyncio

import pytest

from rob_box_voice.core.media_command_grammar import is_spoken_theme_set, parse_media_command
from rob_box_voice.core.media_router import MediaRouter, MediaState
from rob_box_voice.core.media_plan_run import command_turn_context, run_media_plan
from rob_box_voice.core.music_player_state import MusicEventLog, build_music_event_payload
from rob_box_voice.core.set_length_words import heard_set_length

LIVE_1908 = ("Робот, ты диджей 8битный и нас сегодня вечеринка любителей денди и классической музыки "
             "замути сэт на 30 минут")
RETRO_1530 = "ретро 8-бит Mario Tetris Aladdin Contra замути сэт на 30 минут"


def _route(text: str, media: MediaState = MediaState()):
    return MediaRouter().route(text, media)


def test_live_phrase_is_routed_to_dj_set_by_code():
    plan = _route(LIVE_1908)
    assert plan is not None, "реплика ушла бы в LLM (load_skill + dj_set, ~14 с)"
    assert [c.name for c in plan.tool_calls] == ["dj_set"]
    assert plan.tool_calls[0].arguments == {"action": "start", "persona": "диджей 8битный", "tracks": 24}
    assert plan.cancel_inflight and plan.confirm_started
    assert plan.say_ok == "Я диджей 8битный, включаю сет."  # фраза кода, по событию started


def test_live_phrase_gets_its_own_turn_context_with_length_24():
    plan = _route(LIVE_1908)
    ctx = command_turn_context(plan, LIVE_1908)
    assert ctx["_turn_heard_text"] == LIVE_1908
    assert ctx["_turn_set_tracks"] == 24 == heard_set_length(LIVE_1908)
    assert ctx["_turn_id"] and ctx["_turn_id"] != command_turn_context(plan, LIVE_1908)["_turn_id"]


# Та же тема, что на пути LLM (денди + классика), — на настоящем dj_set с этими аргументами и heard_text:
# rob_box_mcp_tools/test/test_dj_set_theme_from_phrase.py (здесь node/conftest подменяет rob_box_mcp_tools моком).


def test_set_order_without_persona_is_routed_too():
    plan = _route(RETRO_1530)
    assert plan is not None
    assert plan.tool_calls[0].arguments == {"action": "start", "tracks": 24}


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


def test_volume_command_keeps_the_llm_turn_context():
    """Громкость идёт поверх хода LLM (без отмены) — его скрытый контекст не трогаем."""
    plan = _route("громче", MediaState(music_playing=True))
    assert plan is not None and not plan.cancel_inflight
    assert command_turn_context(plan, "громче") == {}


# ── 07.10: тема «у нас сегодня …» — не грамматикой, фраза запуска — из темы результата dj_set ────────────────
LIVE_U_NAS = ("Робот, ты диджей 8битный и у нас сегодня вечеринка любителей денди и классической музыки "
              "замути сэт на 30 минут")
ATLAS = "Ты диджей Атлас и у нас сегодня вечеринка любителей кино, играй мелодии из топовых фильмов"


@pytest.mark.parametrize("text,args", [
    (LIVE_U_NAS, {"action": "start", "persona": "диджей 8битный", "tracks": 24}),
    (ATLAS, {"action": "start", "persona": "диджей Атлас"}),
])
def test_spoken_theme_is_not_cut_by_the_grammar(text, args):
    """Раньше: theme='вечеринка любителей денди и классической музыки замути сэт' / '… играй мелодии из топовых
    фильмов' — сырой хвост в поиск и в TTS. Тему выделяет dj_set из heard_text (``theme_grounding.heard_theme``)."""
    plan = _route(text)
    assert plan is not None and plan.tool_calls[0].arguments == args
    assert "Тема" not in plan.say_ok and "диджей диджей" not in plan.say_ok


def _started_phrase(plan, result):
    events = MusicEventLog()
    events.observe_json(build_music_event_payload("started", "s:01:A:x", ts=1.0, phase_in_form=0.0))

    async def call_tool(_call):
        return True, "Сет начат\n" + repr({"ok": True, "track_id": "s:01:A:x", **result})

    return asyncio.run(run_media_plan(plan, call_tool, events))[1]


def test_started_phrase_names_the_theme_dj_set_actually_took():
    """Фразу строит код из результата тула (ADR-0148): тема — та, что выделил dj_set, а не сырой хвост реплики."""
    assert _started_phrase(_route(ATLAS), {"theme": "кино, фильмов"}) == (
        "Я диджей Атлас, включаю сет. Тема — кино, фильмов.")
    assert _started_phrase(_route(ATLAS), {}) == "Я диджей Атлас, включаю сет."
    themed = _route("включи диджей сет на тему космос")
    assert _started_phrase(themed, {"theme": "что-то другое"}) == "Включаю диджей-сет. Тема — космос."
