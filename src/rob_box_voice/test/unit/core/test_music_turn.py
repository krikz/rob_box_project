"""ADR-0148 — речь о музыке хода LLM строит код по результату ``dj_set`` / ``request_music``.

Живые случаи 06.10 (Vision Pi, логи у координатора):

* 14:50 UTC set98207 — ``dj_set`` ок → ``request_music('1812 Overture')`` (погасил сет, не заиграл) →
  ``speak_text('Сет запустил…')``;
* 14:38 UTC set97492 — ``dj_set`` → ``not_started``, модель: «Сет собрался и стартовал»;
* 15:30 UTC — «ретро 8-бит Mario Tetris Aladdin Contra замути сэт на 30 минут» → ``tools=[]``,
  «Сет идёт — 30 минут ретро-ностальгии!».

Тесты проверяют поведение модуля :mod:`rob_box_voice.core.music_turn` и исполнителя тулов, а не текст промпта.
"""

from __future__ import annotations

import asyncio
import json

import pytest

from rob_box_llm.provider import ToolCall, ToolResult
from rob_box_voice.core.media_phrases import (
    DJ_FAIL_TEXT,
    NOT_LAUNCHED_TEXT,
    REQUEST_FAIL_TEXT,
    SET_KEPT_TEXT,
    dj_started_text,
    play_ok_text,
    request_ok_text,
)
from rob_box_voice.core.music_turn import (
    SPEECH_REFUSAL_CODE,
    MusicTurn,
    launch_requested,
    turn_reply,
)
from rob_box_voice.core.set_length_words import heard_set_length
from rob_box_voice.scheduler.task_scheduler import TaskScheduler
from rob_box_voice.scheduler.tool_executor import SchedulerToolExecutor

DJ_ARGS = {"action": "start", "theme": "ретро 8-бит"}
DJ_OK = "{'ok': True, 'track_id': 'set98207:01:A:aa', 'set_id': 'set98207', 'started': True}"
SET_PLAYING_ERR = "музыка не заменена: set_playing"
NOT_STARTED_ERR = "сет не начался: not_started"
LIE = "Сет запустил, погнали!"
RETRO_1530 = "ретро 8-бит Mario Tetris Aladdin Contra замути сэт на 30 минут"


def _turn(*launches) -> MusicTurn:
    turn = MusicTurn()
    for tool, args, is_error, content in launches:
        turn.record(tool, args, is_error=is_error, content=content)
    return turn


# ── модуль: итог хода → фраза ─────────────────────────────────────────────


def test_set_then_request_refused_in_same_turn_says_the_set_plays_and_nothing_else():
    """14:50 set98207: сет заиграл, заказ отклонён ``set_playing`` → фраза о сете и об отказе, речь модели нельзя."""
    turn = _turn(("dj_set", DJ_ARGS, False, DJ_OK),
                 ("request_music", {"intent": "melody", "text": "1812 Overture"}, True, SET_PLAYING_ERR))

    assert turn.speech_allowed() is False
    assert turn.phrase() == f"{dj_started_text('', 'ретро 8-бит')} {SET_KEPT_TEXT}"


def test_dj_set_not_started_can_never_be_called_started():
    """14:38 set97492: ``not_started`` → речь модели не исполняется, фраза — честный отказ."""
    turn = _turn(("dj_set", DJ_ARGS, True, NOT_STARTED_ERR))

    assert turn.speech_allowed() is False
    assert turn.phrase() == DJ_FAIL_TEXT
    assert turn_reply(turn, requested=True, voiced=1) == DJ_FAIL_TEXT  # и даже если что-то успело прозвучать


def test_refusal_inside_json_body_is_a_failure_too():
    """Отказ исполнителя приходит и как ``is_error=False`` с ``{"success": false}`` (#3323)."""
    turn = _turn(("request_music", {"intent": "track"}, False, json.dumps({"success": False, "error": "x"})))
    assert turn.speech_allowed() is False and turn.phrase() == REQUEST_FAIL_TEXT


def test_successful_launch_allows_speech_rap_under_beat():
    """Скилл composer: «рэп под бит» = ``request_music`` и потом ``speak_text`` × N — успех речь не запрещает."""
    turn = _turn(("request_music", {"intent": "track", "text": "бит под рэп"}, False, "{'ok': True}"))

    assert turn.speech_allowed() is True
    assert turn_reply(turn, requested=True, voiced=3) is None  # рэп прозвучал — фраза кода не нужна
    assert turn_reply(turn, requested=True, voiced=0) == request_ok_text("")  # модель промолчала — говорит код


def test_success_phrases_come_from_the_one_table():
    melody = _turn(("request_music", {"intent": "melody", "text": "сыграй к элизе"}, False,
                    "{'ok': True, 'title': 'Fur Elise', 'found': True}"))
    assert melody.phrase() == play_ok_text("Fur Elise")
    dj = _turn(("dj_set", {"action": "start", "theme": "космос", "persona": "диджей Вася"}, False, DJ_OK))
    assert dj.phrase() == dj_started_text("диджей Вася", "космос")


def test_stop_and_other_tools_are_not_launches():
    turn = _turn(("dj_set", {"action": "stop"}, False, "{'ok': True}"), ("lookup_melody", {}, False, "{}"))
    assert turn.launches == [] and turn.speech_allowed() and turn.phrase() is None


def test_last_success_after_a_failure_wins():
    turn = _turn(("dj_set", DJ_ARGS, True, NOT_STARTED_ERR), ("dj_set", DJ_ARGS, False, DJ_OK))
    assert turn.speech_allowed() is True and turn.phrase() == dj_started_text("", "ретро 8-бит")


def test_tools_free_music_request_gets_the_honest_phrase():
    """15:30 UTC: просьба о сете по словам человека (длина сета 24), запуска нет → честная фраза кода."""
    assert heard_set_length(RETRO_1530) == 24
    assert launch_requested(RETRO_1530, heard_set_length(RETRO_1530)) is True
    assert turn_reply(MusicTurn(), requested=True, voiced=0) == NOT_LAUNCHED_TEXT


@pytest.mark.parametrize("text,expected", [
    ("включи диджей сет на тему космос", True),
    ("поставь клубный трек", True),
    ("ты диджей снупдог", True),
    ("как дела", False),
    ("что играет", False),
    ("сыграй 1812 Overture", False),  # PLAY_NAMED: промах базы уходит в LLM, «нет такой» — честно и без тула
])
def test_launch_request_is_decided_by_the_grammar_not_by_the_reply(text, expected):
    assert launch_requested(text, heard_set_length(text)) is expected


def test_non_music_turn_is_left_to_the_model():
    assert turn_reply(MusicTurn(), requested=False, voiced=0) is None


def test_honest_phrase_does_not_claim_the_player_is_silent():
    """Прошлый сет мог играть (15:30): фраза говорит о НОВОМ сете, а не «музыка не играет»."""
    assert "не играет" not in NOT_LAUNCHED_TEXT and "Новый сет" in NOT_LAUNCHED_TEXT


# ── исполнитель тулов: speak_text после неудачного запуска не исполняется ──


class _Underlying:
    def __init__(self, results):
        self.results = results
        self.executed = []

    async def discover(self):
        return ()

    async def execute(self, call: ToolCall) -> ToolResult:
        self.executed.append(call.name)
        is_error, content = self.results.get(call.name, (False, "{'ok': True}"))
        return ToolResult(tool_call_id=call.id, content=content, is_error=is_error)

    async def aclose(self):
        return None


def _run(coro):
    return asyncio.run(coro)


def test_executor_refuses_speak_text_after_set_vs_request_refusal():
    """14:50 set98207 целиком на исполнителе: ``speak_text('Сет запустил…')`` не доходит до TTS."""
    underlying = _Underlying({"dj_set": (False, DJ_OK), "request_music": (True, SET_PLAYING_ERR)})
    executor = SchedulerToolExecutor(underlying, scheduler=None)

    async def _turn_calls():
        executor.begin_turn()
        await executor.execute(ToolCall(id="1", name="dj_set", arguments=DJ_ARGS))
        await executor.execute(ToolCall(id="2", name="request_music",
                                        arguments={"intent": "melody", "text": "1812 Overture"}))
        return await executor.execute(ToolCall(id="3", name="speak_text", arguments={"text": LIE}))

    result = _run(_turn_calls())

    assert underlying.executed == ["dj_set", "request_music"]  # speak_text не исполнен
    assert result.is_error is True and json.loads(result.content)["error"] == SPEECH_REFUSAL_CODE
    assert executor.music_turn.phrase() == f"{dj_started_text('', 'ретро 8-бит')} {SET_KEPT_TEXT}"


def test_executor_lets_speech_through_after_a_started_launch_and_forgets_at_turn_boundary():
    underlying = _Underlying({"request_music": (False, "{'ok': True}")})

    async def _turns():
        sched = TaskScheduler()
        sched.start()
        executor = SchedulerToolExecutor(underlying, scheduler=sched)
        try:
            executor.begin_turn()
            await executor.execute(ToolCall(id="1", name="request_music", arguments={"intent": "track"}))
            rap = await executor.execute(ToolCall(id="2", name="speak_text", arguments={"text": "йо"}))
            assert json.loads(rap.content)["status"] == "queued"
            await asyncio.wait_for(sched.wait_all(), timeout=2.0)
            executor.begin_turn()
            return executor.music_turn.launches
        finally:
            sched.shutdown()

    assert _run(_turns()) == []
    assert underlying.executed == ["request_music", "speak_text"]
