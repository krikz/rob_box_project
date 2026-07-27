"""Shared helpers for the built-in dummy harnesses.

The two built-in harnesses (``EchoHarness`` and ``UpperHarness``)
share a near-identical ``step`` method; the only difference is
the final post-processing of the assistant text. The shared
loop is implemented once in :func:`run_request_response_loop`
and both harnesses call it from their own ``step``.

Why a helper instead of a base class: keeping the harnesses
inherit only from :class:`Harness` makes the relationship
between "framework contract" and "harness code" explicit in
each file. There's no temptation to read an inherited method
in isolation.
"""

from __future__ import annotations

from typing import Any, Awaitable, Callable, Mapping

from rob_box_harness.effects import EchoEffect
from rob_box_harness.harness import Harness
from rob_box_llm.provider import LLMMessage


def ensure_user_text(value: Any) -> str:
    """Coerce ``value`` into a string for the LLM.

    The contract is intentionally loose: callers can pass a plain
    string, a mapping with a ``text`` / ``message`` / ``prompt`` /
    ``input`` key, or anything else (which is ``str()``-ed as a
    last resort). Keeps the harness's step() simple.
    """
    if isinstance(value, str):
        return value
    if isinstance(value, Mapping):
        for key in ("text", "message", "prompt", "input"):
            if key in value:
                return str(value[key])
    return str(value)


PostProcessor = Callable[[str], str]
"""Signature of a final-step text post-processor. Receives the LLM's
final response text and returns the text the harness will return
and dispatch."""


async def run_request_response_loop(
    harness: Harness[Mapping[str, Any]],
    input_data: Any,
    *,
    post_process: PostProcessor | None = None,
) -> str:
    """Execute the canonical "user input → LLM → tool loop → final text" path.

    This is the helper the two built-in harnesses delegate to. The
    steps are:

      1. Coerce the input into a user message.
      2. Call ``self.llm.complete`` and capture the response.
      3. If the response includes ``tool_calls``, execute them via
         ``self.tools``, feed the tool results back to the LLM,
         and use the follow-up response as the final text.
      4. Persist user / tool / assistant turns in ``self.memory``.
      5. Dispatch an :class:`EchoEffect` carrying the final text
         (after any ``post_process``).
      6. Return the final text to the caller.

    The ``post_process`` callable is applied to the final text
    before it is persisted / dispatched / returned. The default
    is the identity function (no transformation).
    """
    if harness.llm is None or harness.memory is None or harness.tools is None:
        raise RuntimeError("harness.init() must be called first")
    if post_process is None:
        post_process = lambda text: text  # noqa: E731 — local identity

    user_text = ensure_user_text(input_data)
    messages: list[LLMMessage] = [LLMMessage(role="user", content=user_text)]
    response = await harness.llm.complete(messages)
    assistant_text = (
        response.content if isinstance(response.content, str) else str(response.content)
    )

    await harness.memory.append_turn(
        harness.config.name,
        LLMMessage(role="user", content=user_text),
    )

    if response.tool_calls:
        await harness.hooks.invoke("on_tool_call", harness.name, response.tool_calls)
        tool_messages: list[LLMMessage] = []
        for call in response.tool_calls:
            result = await harness.tools.execute(call)
            await harness.hooks.invoke("on_tool_result", harness.name, result)
            tool_messages.append(
                LLMMessage(
                    role="tool",
                    content=result.content,
                    tool_call_id=result.tool_call_id,
                )
            )
        messages.extend(
            [
                LLMMessage(
                    role="assistant",
                    content=assistant_text,
                    tool_calls=response.tool_calls,
                ),
                *tool_messages,
            ]
        )
        follow_up = await harness.llm.complete(messages)
        assistant_text = (
            follow_up.content
            if isinstance(follow_up.content, str)
            else str(follow_up.content)
        )
        for tool_msg in tool_messages:
            await harness.memory.append_turn(harness.config.name, tool_msg)

    final_text = post_process(assistant_text)

    await harness.memory.append_turn(
        harness.config.name,
        LLMMessage(role="assistant", content=final_text),
    )

    await harness.effects.dispatch(EchoEffect(text=final_text))
    harness._record("last_assistant_text", final_text)
    return final_text


__all__ = ["ensure_user_text", "run_request_response_loop", "PostProcessor"]
