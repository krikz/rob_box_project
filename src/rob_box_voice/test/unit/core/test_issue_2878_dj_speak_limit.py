"""Issue #2878 — DJ-ход: не больше одной (двух на старте сета) реплики
``speak_text``, и та, что проходит, обрезана до ~140 символов.

Живой прогон 23.09.2026, DJ-сет «Диджей Хопер»: в ОДНОМ DJ-ходе LLM
вызвала ``speak_text`` ШЕСТЬ раз подряд (длинные монологи поверх трека),
пока AgentCore не упёрся в ``_MAX_TOOL_ITERATIONS=8``. ``TrackStartGuard``
(issue #2859) её не останавливал — ``speak_text`` не запускает трек.

Покрытие:

* чистый :class:`TrackStartGuard` — ``is_dj_turn`` / ``speak_limit`` /
  ``should_refuse_speak`` / ``record_speak`` / сброс на ``reset()``;
* :func:`trim_dj_speech` — обрезка до ~140 символов / 1-2 предложений;
* :class:`SchedulerToolExecutor` — отказ лишних ``speak_text``, трим
  текста, не-DJ ходы не ограничены.
"""

from __future__ import annotations

import asyncio
import json

import pytest

from rob_box_llm.provider import ToolCall, ToolResult
from rob_box_voice.core.track_start_guard import (
    DEFAULT_DJ_SPEAK_LIMIT,
    DJ_SPEAK_MAX_CHARS,
    PARTY_START_DJ_SPEAK_LIMIT,
    SPEAK_REFUSAL_ERROR_CODE,
    SPEAK_REFUSAL_MESSAGE,
    TrackStartGuard,
    speak_refusal_content,
    trim_dj_speech,
)
from rob_box_voice.scheduler.task_scheduler import TaskScheduler
from rob_box_voice.scheduler.tool_executor import SchedulerToolExecutor


# ---------------------------------------------------------------------------
# trim_dj_speech
# ---------------------------------------------------------------------------


def test_trim_short_text_is_unchanged() -> None:
    assert trim_dj_speech("Йо, качаем!") == "Йо, качаем!"


def test_trim_keeps_at_most_two_sentences() -> None:
    text = "Первое предложение. Второе предложение. Третье — лишнее совсем."
    trimmed = trim_dj_speech(text)
    assert trimmed == "Первое предложение. Второе предложение."
    assert "Третье" not in trimmed


def test_trim_hard_cuts_one_long_sentence_at_word_boundary() -> None:
    long_sentence = "слово " * 40  # far past 140 chars, no sentence break
    trimmed = trim_dj_speech(long_sentence.strip())
    assert len(trimmed) <= DJ_SPEAK_MAX_CHARS + 1  # +1 for the ellipsis char
    assert trimmed.endswith("…")
    assert not trimmed[:-1].endswith(" ")


def test_trim_empty_text_is_unchanged() -> None:
    assert trim_dj_speech("   ") == ""


def test_trim_within_limit_returns_as_is() -> None:
    text = "Разгоняемся!"
    assert trim_dj_speech(text) == text


# ---------------------------------------------------------------------------
# Pure guard
# ---------------------------------------------------------------------------


def test_fresh_guard_is_not_a_dj_turn() -> None:
    guard = TrackStartGuard()
    assert guard.is_dj_turn is False
    assert guard.should_refuse_speak() is False


def test_track_start_marks_dj_turn() -> None:
    guard = TrackStartGuard()
    guard.record("compose_music", is_error=False)
    assert guard.is_dj_turn is True
    assert guard.speak_limit == DEFAULT_DJ_SPEAK_LIMIT


def test_set_dj_mode_marks_dj_turn_even_when_track_not_started_yet() -> None:
    guard = TrackStartGuard()
    guard.record("set_dj_mode", is_error=False, args={"enabled": True})
    assert guard.is_dj_turn is True


def test_set_dj_mode_failure_still_marks_dj_turn() -> None:
    """Сам факт вызова говорит о контексте хода — не о его успехе."""
    guard = TrackStartGuard()
    guard.record("set_dj_mode", is_error=True, args={"enabled": True})
    assert guard.is_dj_turn is True


def test_default_dj_speak_limit_is_one() -> None:
    guard = TrackStartGuard()
    guard.record("compose_music", is_error=False)
    assert guard.should_refuse_speak() is False
    guard.record_speak()
    assert guard.should_refuse_speak() is True


def test_party_start_dj_mode_with_plan_raises_limit_to_two() -> None:
    guard = TrackStartGuard()
    guard.record("set_dj_mode", is_error=False, args={
        "enabled": True, "plan": "Трек 1: ...\nТрек 2: ...",
    })
    assert guard.speak_limit == PARTY_START_DJ_SPEAK_LIMIT
    guard.record_speak()
    assert guard.should_refuse_speak() is False
    guard.record_speak()
    assert guard.should_refuse_speak() is True


def test_set_dj_mode_without_plan_keeps_default_limit() -> None:
    guard = TrackStartGuard()
    guard.record("set_dj_mode", is_error=False, args={
        "enabled": True, "next_transition_sec": 90,
    })
    assert guard.speak_limit == DEFAULT_DJ_SPEAK_LIMIT


def test_reset_clears_dj_turn_and_speak_count() -> None:
    guard = TrackStartGuard()
    guard.record("compose_music", is_error=False)
    guard.record_speak()
    guard.reset()
    assert guard.is_dj_turn is False
    assert guard.speak_count == 0
    assert guard.should_refuse_speak() is False


def test_speak_refusal_content_carries_counts() -> None:
    payload = json.loads(speak_refusal_content(1, 1))
    assert payload["success"] is False
    assert payload["error"] == SPEAK_REFUSAL_ERROR_CODE
    assert payload["message"] == SPEAK_REFUSAL_MESSAGE
    assert payload["speak_count_this_turn"] == 1
    assert payload["speak_limit_this_turn"] == 1


# ---------------------------------------------------------------------------
# SchedulerToolExecutor wiring
# ---------------------------------------------------------------------------


class _FakeUnderlying:
    def __init__(self) -> None:
        self.executed: list[ToolCall] = []

    async def discover(self):
        return ()

    async def execute(self, call: ToolCall) -> ToolResult:
        self.executed.append(call)
        return ToolResult(
            tool_call_id=call.id,
            content=json.dumps({"success": True, "tool": call.name}),
            is_error=False,
        )

    async def aclose(self) -> None:
        return None


def _speak(call_id: str, text: str = "Йо, народ!") -> ToolCall:
    return ToolCall(id=call_id, name="speak_text", arguments={"text": text})


async def _with_scheduler(body):
    """Run *body(executor, underlying)* against a real, shut-down-on-exit
    :class:`TaskScheduler` — ``speak_text`` is channel-routed (not bypass),
    so exercising it for real needs a live scheduler + drained pump, same
    pattern as ``test_tool_executor.py``.
    """
    underlying = _FakeUnderlying()
    sched = TaskScheduler()
    sched.start()
    executor = SchedulerToolExecutor(underlying, scheduler=sched)
    try:
        result = await body(executor)
        await asyncio.wait_for(sched.wait_all(), timeout=2.0)
        return result, underlying
    finally:
        sched.shutdown()


def test_non_dj_turn_speak_text_is_unlimited() -> None:
    """Обычный диалог (никакого DJ-тула в ходе) — лимита нет."""

    async def _body(executor: SchedulerToolExecutor) -> list[ToolResult]:
        executor.begin_turn()
        return [
            await executor.execute(_speak("s1")),
            await executor.execute(_speak("s2")),
            await executor.execute(_speak("s3")),
        ]

    results, _underlying = asyncio.run(_with_scheduler(_body))
    assert all(json.loads(r.content)["status"] == "queued" for r in results)


def test_dj_turn_refuses_second_speak_text() -> None:
    async def _body(executor: SchedulerToolExecutor):
        executor.begin_turn()
        await executor.execute(
            ToolCall(id="cm", name="compose_music", arguments={"name": "x"}))
        first = await executor.execute(_speak("s1", "Первая реплика."))
        second = await executor.execute(_speak("s2", "Вторая реплика лишняя."))
        return first, second

    (first, second), _underlying = asyncio.run(_with_scheduler(_body))
    assert json.loads(first.content)["status"] == "queued"
    payload = json.loads(second.content)
    assert second.is_error is False
    assert payload["success"] is False
    assert payload["error"] == SPEAK_REFUSAL_ERROR_CODE
    assert payload["message"] == SPEAK_REFUSAL_MESSAGE


def test_party_start_turn_allows_two_speak_text_calls() -> None:
    async def _body(executor: SchedulerToolExecutor):
        executor.begin_turn()
        await executor.execute(ToolCall(
            id="dj", name="set_dj_mode",
            arguments={"enabled": True, "plan": "Трек 1: ...\nТрек 2: ..."},
        ))
        await executor.execute(
            ToolCall(id="cm", name="compose_music", arguments={"name": "x"}))
        first = await executor.execute(_speak("s1", "Представление диджея."))
        second = await executor.execute(_speak("s2", "Анонс первого трека."))
        third = await executor.execute(_speak("s3", "Третья — лишняя."))
        return first, second, third

    (first, second, third), _underlying = asyncio.run(_with_scheduler(_body))
    assert json.loads(first.content)["status"] == "queued"
    assert json.loads(second.content)["status"] == "queued"
    payload = json.loads(third.content)
    assert payload["error"] == SPEAK_REFUSAL_ERROR_CODE


def test_dj_turn_trims_long_speak_text() -> None:
    long_text = "Первое предложение диджея. " * 10

    async def _body(executor: SchedulerToolExecutor) -> None:
        executor.begin_turn()
        await executor.execute(
            ToolCall(id="cm", name="compose_music", arguments={"name": "x"}))
        await executor.execute(_speak("s1", long_text))

    _result, underlying = asyncio.run(_with_scheduler(_body))
    spoken = next(c for c in underlying.executed if c.name == "speak_text")
    trimmed_text = spoken.arguments["text"]
    assert trimmed_text == trim_dj_speech(long_text)
    assert len(trimmed_text) <= DJ_SPEAK_MAX_CHARS + 1
    assert trimmed_text != long_text


def test_next_dj_turn_resets_speak_limit() -> None:
    async def _body(executor: SchedulerToolExecutor):
        executor.begin_turn()
        await executor.execute(
            ToolCall(id="cm1", name="compose_music", arguments={"name": "a"}))
        first = await executor.execute(_speak("s1"))

        executor.begin_turn()
        await executor.execute(
            ToolCall(id="cm2", name="compose_music", arguments={"name": "b"}))
        second = await executor.execute(_speak("s2"))
        return first, second

    (first, second), _underlying = asyncio.run(_with_scheduler(_body))
    assert json.loads(first.content)["status"] == "queued"
    assert json.loads(second.content)["status"] == "queued"


@pytest.mark.parametrize("tool_first", ["compose_music", "set_dj_mode"])
def test_either_track_start_or_dj_mode_call_enables_speak_guard(
    tool_first: str,
) -> None:
    async def _body(executor: SchedulerToolExecutor):
        executor.begin_turn()
        await executor.execute(ToolCall(id="t1", name=tool_first, arguments={}))
        first = await executor.execute(_speak("s1"))
        second = await executor.execute(_speak("s2"))
        return first, second

    (first, second), _underlying = asyncio.run(_with_scheduler(_body))
    assert json.loads(first.content)["status"] == "queued"
    assert json.loads(second.content)["error"] == SPEAK_REFUSAL_ERROR_CODE
