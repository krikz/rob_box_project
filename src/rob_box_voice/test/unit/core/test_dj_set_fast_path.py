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

import pytest

from rob_box_voice.core.media_command_grammar import is_spoken_theme_set, parse_media_command
from rob_box_voice.core.media_router import MediaRouter, MediaState
from rob_box_voice.core.media_plan_run import command_turn_context
from rob_box_voice.core.set_length_words import heard_set_length

LIVE_1908 = ("Робот, ты диджей 8битный и нас сегодня вечеринка любителей денди и классической музыки "
             "замути сэт на 30 минут")
RETRO_1530 = "ретро 8-бит Mario Tetris Aladdin Contra замути сэт на 30 минут"
#: Тема, которую LLM 06.10 18:04 взяла из СТАРОЙ реплики истории.
OLD_LLM_THEME = "Увертюра 1812, мегасет для геймеров: Марио, Аладдин, Тетрис"


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


def test_command_theme_is_the_same_as_on_the_llm_path():
    """Тема команды и тема пути LLM — одна функция над одной репликой: денди и классика, без слов просьбы."""
    grounding = pytest.importorskip("rob_box_mcp_tools.engine.theme_grounding")
    ctx = command_turn_context(_route(LIVE_1908), LIVE_1908)
    command_theme, _ = grounding.grounded_theme(None, ctx["_turn_heard_text"])
    llm_theme, replaced = grounding.grounded_theme(OLD_LLM_THEME, LIVE_1908)
    assert replaced and command_theme == llm_theme
    assert "денди" in command_theme and "классической" in command_theme
    for service in ("робот", "диджей", "замути", "сэт", "минут"):
        assert service not in command_theme.lower().split()


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
