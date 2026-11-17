"""Issue #3265 — ретрай-коррекция внутри хода с thinking не думает.

Живой ход 01.10 11:10: первый вызов (thinking) — 76 с и пустой ответ;
ретрай с ``[SYSTEM CORRECTION]`` думал снова (+39 с). Теперь thinking
получает только вызов, читающий реплику.
"""
from __future__ import annotations

from rob_box_harness.providers.reasoning import (
    CORRECTION_PREFIX,
    TURN_REASONING,
    is_reasoning_call,
)
from rob_box_llm.provider import LLMMessage

SYSTEM = LLMMessage(role="system", content="правила")
USER = LLMMessage(role="user", content="[DJ_AUTO переход #2] Вызови compose_music.")
ASSISTANT = LLMMessage(role="assistant", content="")
CORRECTION = LLMMessage(
    role="user", content=CORRECTION_PREFIX + " Твой предыдущий ответ был пустым."
)
TOOL = LLMMessage(role="tool", content="ok")


def _call(flag, messages):
    token = TURN_REASONING.set(flag)
    try:
        return is_reasoning_call(messages)
    finally:
        TURN_REASONING.reset(token)


def test_first_call_of_thinking_turn_thinks():
    assert _call(True, [SYSTEM, USER]) is True


def test_turn_without_flag_never_thinks():
    assert _call(False, [SYSTEM, USER]) is False


def test_correction_retry_does_not_think():
    assert _call(True, [SYSTEM, USER, ASSISTANT, CORRECTION]) is False


def test_correction_with_leading_whitespace_does_not_think():
    msg = LLMMessage(role="user", content="\n  " + CORRECTION.content)
    assert _call(True, [SYSTEM, USER, ASSISTANT, msg]) is False


def test_call_after_tool_result_does_not_think():
    assert _call(True, [SYSTEM, USER, ASSISTANT, TOOL]) is False
