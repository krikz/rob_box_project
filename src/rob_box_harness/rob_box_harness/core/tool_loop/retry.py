"""Corrective-retry helpers extracted from ``AgentCore._run_with_tools``.

Issue #2630 PR-B — выделить ~8 CC из основного метода.

Два single-shot retry, каждый ровно один раз на ход:

1. **Truncated tool-call arguments** (issue #1899). Модель обрезала
   JSON-аргументы по ``max_tokens``. Без retry выполнение упало бы
   на валидации, цикл потерял бы ~6 секунд на пустой повтор. С
   retry мы заранее просим модель переделать с более короткими
   аргументами.

2. **Silent / pseudo-call response** (issue #1217). Модель вернула
   пустой ответ или написала вызов инструмента текстом, не
   сформировав function-calling payload. Пользователь ничего не
   услышал; retry объясняет, что текстовое описание вызова — это
   не вызов, и просит дать реальный tool-call.

Оба retry добавляют ``assistant`` echo (если есть content) + ``user``
correction к ``messages`` и заново вызывают :meth:`AgentCore._stream_response`.
Никакой рекурсии — флаг ``_truncated_tool_args_retried`` /
``_silent_retried`` гарантирует single-shot.

Helpers:
- :func:`is_truncated_args_candidate` — стоит ли вообще пробовать retry
- :func:`build_truncated_args_correction` — текст correction-сообщения
- :func:`build_silent_response_correction` — то же для silent/pseudo
- :func:`request_truncated_args_retry` — append messages + re-stream
- :func:`request_silent_response_retry` — то же
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from rob_box_harness.core.agent_core import (
    _is_pseudo_tool_call,
)
from rob_box_llm.provider import LLMMessage, LLMResponse

if TYPE_CHECKING:
    from rob_box_harness.core.agent_core import AgentCore


_LOG = logging.getLogger(__name__)


def is_truncated_args_candidate(response: LLMResponse) -> bool:
    """True iff a single-shot truncated-args retry is worth attempting.

    Truncated JSON tool-call arguments (``finish_reason='length'``)
    would fail validation and burn ~6 s on a wasted executor call
    (issue #1899). We retry BEFORE execution.
    """
    return bool(response.tool_calls) and bool(response.truncated_tool_args)


def build_truncated_args_correction(response: LLMResponse) -> LLMMessage:
    """Build the user-role correction message for the truncated-args retry.

    Lists the tool names we observed truncated and asks the model to
    re-emit the same call with shorter argument payloads, or to split
    the work across multiple turns.
    """
    names = sorted({c.name for c in response.tool_calls})
    return LLMMessage(
        role="user",
        content=(
            "[SYSTEM CORRECTION] Твой предыдущий tool-call "
            "был ОБРЕЗАН: ответ не поместился в max_tokens "
            "и JSON-аргументы НЕ ЗАКРЫЛИСЬ. Инструмент "
            f"{names!r} НЕ БЫЛ вызван (валидация бы упала "
            "на пустых/обрезанных аргументах). "
            "Повтори ход и вызови нужный tool снова, но "
            "с БОЛЕЕ КОРОТКИМИ значениями аргументов — "
            "короткие строки, никаких многострочных "
            "описаний. Если аргументов слишком много для "
            "бюджета токенов, разбей на несколько ходов: "
            "сначала вызови основной tool с минимальным "
            "набором полей, остальное — следующим ходом."
        ),
    )


def build_silent_response_correction(response: LLMResponse) -> LLMMessage:
    """Build the user-role correction for a silent / pseudo-call reply.

    Two failure shapes share the same retry path (issue #1217):

    * Empty payload — model returned ``content=""`` AND no
      tool_calls. Nothing happened.
    * Pseudo-call — model DESCRIBED a tool call as text instead of
      using function-calling. The user heard the description, but
      no tool ran; the robot stays silent.

    The correction text is shaped per failure mode so the model sees
    exactly what went wrong (a generic «ответ был пустым» does NOT
    help when the model wrote the call as text — it would just keep
    writing it again).
    """
    if _is_pseudo_tool_call(response.content or ""):
        return LLMMessage(
            role="user",
            content=(
                "[SYSTEM CORRECTION] Ты НАПИСАЛ вызов инструмента "
                "текстом: "
                + (response.content or "").strip()[:120]
                + ". Это не вызов — это строка, её никто не "
                "выполнил, и пользователь ничего не услышал. "
                "Инструменты вызываются механизмом function "
                "calling, а не текстом ответа. Повтори ход и "
                "вызови нужный tool ПО-НАСТОЯЩЕМУ, со всеми "
                "аргументами."
            ),
        )
    return LLMMessage(
        role="user",
        content=(
            "[SYSTEM CORRECTION] Твой предыдущий ответ "
            "был пустым: ни текста, ни tool-вызова. "
            "Пользователь ничего не услышал, ничего не "
            "произошло. ОБЯЗАТЕЛЬНО в ЭТОМ ответе вызови "
            "нужный tool (speak_text — для речи) или дай "
            "содержательный текстовый ответ."
        ),
    )


async def request_truncated_args_retry(
    agent: "AgentCore",
    messages: list[LLMMessage],
    response: LLMResponse,
    tools: list[dict],
) -> LLMResponse:
    """Append correction message, re-stream, return the new response.

    Records the assistant turn we received so the OpenAI-style history
    stays valid (``assistant`` message must precede the next ``user``
    when tool_calls were emitted).
    """
    names = sorted({c.name for c in response.tool_calls})
    _LOG.warning(
        "AgentCore [issue 1899]: tool-call arguments JSON cut "
        "off mid-stream (finish_reason=%r). Asking model to "
        "retry with shorter args. tools=%s",
        response.finish_reason,
        names,
    )
    if response.content or response.tool_calls:
        messages.append(
            LLMMessage(
                role="assistant",
                content=response.content,
                tool_calls=response.tool_calls,
            )
        )
    messages.append(build_truncated_args_correction(response))
    return await agent._stream_response(messages, tools=tools)


async def request_silent_response_retry(
    agent: "AgentCore",
    messages: list[LLMMessage],
    response: LLMResponse,
    tools: list[dict],
) -> LLMResponse:
    """Append the silent-response correction, re-stream, return the result."""
    if response.content:
        messages.append(
            LLMMessage(role="assistant", content=response.content)
        )
    messages.append(build_silent_response_correction(response))
    return await agent._stream_response(messages, tools=tools)