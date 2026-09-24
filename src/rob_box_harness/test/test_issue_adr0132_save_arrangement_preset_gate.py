"""test_issue_adr0132_save_arrangement_preset_gate.py — ADR-0132 PR-7 hard gate.

``save_arrangement_preset`` (composer skill, ``rob_box_mcp_tools/tools/
music.py``) persists the knobs of the last-played track as the DEFAULT for
every future ``compose_music`` call of that melody. The owner's decision
(ADR-0132 §7): the model must never decide on its own that an arrangement
is good enough to become that default — only an EXPLICIT praise or
explicit save-request in the user's own words unlocks it.

The gate cannot live inside the MCP tool itself — that process never sees
conversation history. It lives in :mod:`rob_box_harness.core.agent_core`,
inside :meth:`AgentCore._execute_tool_batch`
(``_gate_save_arrangement_preset``), which is the ONE place that already
computes ``current_user_input`` for every LLM tool-call batch (the same
point issue #1708's hallucinated-lyrics guard hooks into — see
``test_issue_1708_hallucinated_lyrics.py``, whose fakes this file mirrors).

Three contracts pinned here:

1. No praise/save-request in the last user turn → the tool is NEVER
   executed (verified via ``fail_on_execute_names`` — the executor raises
   if it's reached) and the model gets a refusal it can act on.
2. Explicit praise/save-request → the tool DOES execute, and
   ``approved_by_user_quote`` in the arguments that reach the executor is
   overwritten with the REAL user utterance — even if the model's own
   tool-call arguments carried a different (possibly fabricated) quote.
3. Explicit negative feedback («не нравится») must not be misread as
   praise (the ``contains_praise`` negation guard, see
   ``test_praise_gate.py``) — refused too.

Run with::

    python3 -m pytest src/rob_box_harness/test/test_issue_adr0132_save_arrangement_preset_gate.py -v
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Any

import pytest

from rob_box_harness.core import agent_core
from rob_box_harness.core.agent_core import AgentCore
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateMachine,
)
from rob_box_harness.tools import ToolProvider, ToolSpec
from rob_box_llm.provider import LLMMessage, LLMResponse, ToolCall, ToolResult

# ---------------------------------------------------------------------
# Fakes — copied from test_issue_1708_hallucinated_lyrics.py (that
# module's own docstring says to copy from it rather than share
# fixtures across regression modules).
# ---------------------------------------------------------------------


@dataclass
class _FakeLLMMessage:
    role: str = "user"
    content: str = ""


class _ScriptedLLM:
    """Pops a pre-recorded response on each complete() call."""

    name = "scripted_llm"

    def __init__(self, responses: list[Any]) -> None:
        self._responses = list(responses)
        self.calls: list[Any] = []
        self.messages_seen: list[list[LLMMessage]] = []

    async def complete(
        self,
        messages: Any = None,
        *,
        tools: Any = (),
        settings: Any = None,
        **_kwargs: Any,
    ) -> Any:
        materialised = list(messages) if messages is not None else []
        self.calls.append((materialised, tools))
        self.messages_seen.append(
            [LLMMessage(role=m.role, content=m.content) for m in materialised]
        )
        if self._responses:
            item = self._responses.pop(0)
            if isinstance(item, BaseException):
                raise item
            return item
        return LLMResponse(content="done", tool_calls=())

    async def aclose(self) -> None:
        return None


class _RecordingToolProvider(ToolProvider):
    """Executes via handler_map; remembers every call that reached it.

    ``fail_on_execute_names`` mirrors the #1708 test file: if a call for
    one of these names ever reaches :meth:`execute`, that IS the bug
    (the gate should have refused it before the executor was touched).
    """

    name = "recording_tools"

    def __init__(self, manifest: tuple[Any, ...], handler_map: dict[str, Any]) -> None:
        self._manifest: tuple[Any, ...] = manifest
        self._handler_map: dict[str, Any] = handler_map
        self.executed: list[Any] = []
        self.fail_on_execute_names: set[str] = set()

    async def discover(self) -> tuple[Any, ...]:
        return self._manifest

    async def execute(self, call: Any) -> Any:
        if call.name in self.fail_on_execute_names:
            raise AssertionError(
                f"tool call reached executor that should have been "
                f"gated: name={call.name!r} args={call.arguments!r}"
            )
        self.executed.append(call)
        handler = self._handler_map.get(call.name)
        if handler is None:
            return ToolResult(
                tool_call_id=call.id, content=f"unknown tool: {call.name}", is_error=True,
            )
        result = handler(dict(call.arguments))
        if hasattr(result, "__await__"):
            result = await result
        if isinstance(result, ToolResult):
            return result
        return ToolResult(tool_call_id=call.id, content=str(result), is_error=False)

    async def aclose(self) -> None:
        return None


class _FakeMemoryStore:
    def __init__(self) -> None:
        self.turns: list[Any] = []
        self.facts: list[tuple[str, str]] = []

    async def append_turn(self, scope: str, turn: Any) -> None:
        self.turns.append(turn)

    async def load_recent(self, scope: str, limit: int = 10) -> list[Any]:
        return list(self.turns[-limit:])

    async def save_fact(self, scope: str, fact: Any) -> None:
        self.facts.append((fact.key, fact.value))

    async def search_facts(self, scope: str, query: str, limit: int = 5) -> list[Any]:
        return []

    async def aclose(self) -> None:
        return None


def _wake(core: AgentCore) -> None:
    core._dsm.on_event(DialogueEvent.WAKE_WORD)


def _build_manifest() -> tuple[Any, ...]:
    return (
        ToolSpec(
            name="save_arrangement_preset",
            description="Сохранить ручки последнего сыгранного трека как пресет.",
            parameters={
                "type": "object",
                "properties": {
                    "note": {"type": "string"},
                    "approved_by_user_quote": {"type": "string"},
                },
            },
        ),
    )


def _build_core(
    responses: list[LLMResponse],
    *,
    handler_map: dict[str, Any],
    fail_on_execute_names: set[str] | None = None,
) -> tuple[AgentCore, _RecordingToolProvider, _ScriptedLLM]:
    llm = _ScriptedLLM(responses)
    tools = _RecordingToolProvider(_build_manifest(), handler_map)
    if fail_on_execute_names:
        tools.fail_on_execute_names.update(fail_on_execute_names)
    memory = _FakeMemoryStore()
    dsm = DialogueStateMachine()
    core = AgentCore(llm=llm, tools=tools, memory=memory, dsm=dsm)
    _wake(core)
    return core, tools, llm


def _run(core: AgentCore, text: str, *, dynamic_system: str | None = None) -> Any:
    return asyncio.run(
        core.process_input(text, history=[], dynamic_system=dynamic_system)
    )


def _save_call(quote: str | None = None) -> ToolCall:
    args: dict[str, Any] = {"note": "звучит собранно"}
    if quote is not None:
        args["approved_by_user_quote"] = quote
    return ToolCall(id="s1", name="save_arrangement_preset", arguments=args)


# ---------------------------------------------------------------------
# 1) No praise → the tool must NEVER reach the executor.
# ---------------------------------------------------------------------


def test_no_praise_refuses_without_executing() -> None:
    """«сыграй ещё раз» carries no praise/save-request — gated, not executed."""
    scripted = [
        LLMResponse(
            content="",
            tool_calls=(_save_call(quote="модель придумала цитату"),),
            finish_reason="tool_calls",
        ),
        LLMResponse(content="done", tool_calls=()),
    ]

    async def save_handler(_args: dict[str, object]) -> str:
        raise AssertionError("save_arrangement_preset reached the executor without praise")

    core, tools, llm = _build_core(
        scripted,
        handler_map={"save_arrangement_preset": save_handler},
        fail_on_execute_names={"save_arrangement_preset"},
    )

    _run(core, "сыграй ещё раз")

    assert tools.executed == [], f"tool must not execute; got {tools.executed!r}"

    # The model sees a refusal it can act on (finishes the turn with speech,
    # not a silent failure) — mirrors the #1708 sentinel-visibility check.
    second_iter = llm.messages_seen[1]
    tool_msgs = [m for m in second_iter if m.role == "tool"]
    assert len(tool_msgs) == 1
    assert "save_arrangement_preset" in tool_msgs[0].content
    assert "отклонён" in tool_msgs[0].content or "похвал" in tool_msgs[0].content.lower()


# ---------------------------------------------------------------------
# 2) Explicit praise/save-request → executes, quote is the REAL one.
# ---------------------------------------------------------------------


def test_praise_executes_and_quote_is_overwritten_with_the_real_utterance() -> None:
    """«вот это кайф, сохрани» unlocks the call; the model's own fabricated
    ``approved_by_user_quote`` must be discarded and replaced by the actual
    user turn, not trusted verbatim (the model cannot self-certify)."""
    fabricated_quote = "юзер сказал что это гениально (выдумано моделью)"
    real_user_text = "вот это кайф, сохрани"

    scripted = [
        LLMResponse(
            content="",
            tool_calls=(_save_call(quote=fabricated_quote),),
            finish_reason="tool_calls",
        ),
        LLMResponse(content="done", tool_calls=()),
    ]

    seen_args: dict[str, Any] = {}

    async def save_handler(args: dict[str, object]) -> str:
        seen_args.update(args)
        return '{"success": true, "melody_key": "fifth"}'

    core, tools, _llm = _build_core(
        scripted,
        handler_map={"save_arrangement_preset": save_handler},
    )

    _run(core, real_user_text)

    executed_names = [c.name for c in tools.executed]
    assert executed_names == ["save_arrangement_preset"], (
        f"tool must execute on explicit praise; got {executed_names!r}"
    )
    assert seen_args.get("approved_by_user_quote") == real_user_text, (
        f"approved_by_user_quote must be the REAL user turn, not the "
        f"model's own quote; got {seen_args.get('approved_by_user_quote')!r}"
    )
    assert seen_args.get("approved_by_user_quote") != fabricated_quote


def test_explicit_save_request_without_praise_word_also_executes() -> None:
    """«сохрани этот вариант» — a direct save request, no praise adjective."""
    scripted = [
        LLMResponse(
            content="",
            tool_calls=(_save_call(quote=None),),
            finish_reason="tool_calls",
        ),
        LLMResponse(content="done", tool_calls=()),
    ]

    async def save_handler(args: dict[str, object]) -> str:
        return '{"success": true}'

    core, tools, _llm = _build_core(
        scripted,
        handler_map={"save_arrangement_preset": save_handler},
    )

    _run(core, "сохрани этот вариант")

    assert [c.name for c in tools.executed] == ["save_arrangement_preset"]
    saved_call = tools.executed[0]
    assert saved_call.arguments["approved_by_user_quote"] == "сохрани этот вариант"


# ---------------------------------------------------------------------
# 3) Negative feedback must not be misread as praise.
# ---------------------------------------------------------------------


def test_negative_feedback_is_refused_not_misread_as_praise() -> None:
    """«не нравится» — the ``нравится`` substring alone must NOT unlock
    the gate (praise_gate negation guard, see test_praise_gate.py)."""
    scripted = [
        LLMResponse(
            content="",
            tool_calls=(_save_call(quote="модель решила сохранить всё равно"),),
            finish_reason="tool_calls",
        ),
        LLMResponse(content="done", tool_calls=()),
    ]

    async def save_handler(_args: dict[str, object]) -> str:
        raise AssertionError("save_arrangement_preset executed on negative feedback")

    core, tools, _llm = _build_core(
        scripted,
        handler_map={"save_arrangement_preset": save_handler},
        fail_on_execute_names={"save_arrangement_preset"},
    )

    _run(core, "не нравится, сделай по-другому")

    assert tools.executed == []


# ---------------------------------------------------------------------
# 4) Issue #2955 -- the gate must read the RAW utterance, not the
#    composed ``<system_context>...`` + text turn dialogue_node builds
#    (issue #2817/#2822). A word like «нравится» sitting in a speaker's
#    PROFILE inside the snapshot must NOT unlock the gate; conversely, a
#    real praise/save-request in the utterance must still work even
#    with a system_context glued in front of it, and the saved quote
#    must be the utterance alone.
# ---------------------------------------------------------------------

_SYSTEM_CONTEXT_WITH_PRAISE_WORD = (
    "<system_context>\n"
    "  <speaker>Антон, профиль: обычно говорит быстро, "
    "  дружелюбный, ему нравится джаз</speaker>\n"
    "  <memory>Хранит факт: нравится вечерний сет</memory>\n"
    "</system_context>"
)


def test_praise_word_in_system_context_does_not_unlock_gate() -> None:
    """Live 24.09.2026 (issue #2955): «нравится» in the SPEAKER PROFILE
    inside ``<system_context>`` must not read as the user's own praise.
    The utterance itself («сыграй ещё раз») carries no praise/save-
    request -> refused, exactly like ``test_no_praise_refuses_without_executing``."""
    scripted = [
        LLMResponse(
            content="",
            tool_calls=(_save_call(quote="модель придумала цитату"),),
            finish_reason="tool_calls",
        ),
        LLMResponse(content="done", tool_calls=()),
    ]

    async def save_handler(_args: dict[str, object]) -> str:
        raise AssertionError(
            "save_arrangement_preset reached the executor -- system_context "
            "praise word must not unlock the gate (issue #2955)"
        )

    core, tools, _llm = _build_core(
        scripted,
        handler_map={"save_arrangement_preset": save_handler},
        fail_on_execute_names={"save_arrangement_preset"},
    )

    _run(
        core,
        "сыграй ещё раз",
        dynamic_system=_SYSTEM_CONTEXT_WITH_PRAISE_WORD,
    )

    assert tools.executed == [], f"tool must not execute; got {tools.executed!r}"


def test_real_praise_with_system_context_executes_with_clean_quote() -> None:
    """Live 24.09.2026 (issue #2955): «вот это кайф, огонь! сохрани этот
    вариант» IS real praise + a save-request -- gate must still unlock
    with a ``<system_context>`` snapshot glued in front of it by
    ``_compose_current_turn_message``, and ``approved_by_user_quote``
    must be EXACTLY the utterance, with no system_context bleed
    (previously observed live: ``'<system_conte...'``)."""
    real_user_text = "вот это кайф, огонь! сохрани этот вариант"
    scripted = [
        LLMResponse(
            content="",
            tool_calls=(_save_call(quote="выдуманная моделью цитата"),),
            finish_reason="tool_calls",
        ),
        LLMResponse(content="done", tool_calls=()),
    ]

    seen_args: dict[str, Any] = {}

    async def save_handler(args: dict[str, object]) -> str:
        seen_args.update(args)
        return '{"success": true, "melody_key": "fifth"}'

    core, tools, _llm = _build_core(
        scripted,
        handler_map={"save_arrangement_preset": save_handler},
    )

    _run(
        core,
        real_user_text,
        dynamic_system=_SYSTEM_CONTEXT_WITH_PRAISE_WORD,
    )

    assert [c.name for c in tools.executed] == ["save_arrangement_preset"]
    quote = seen_args.get("approved_by_user_quote")
    assert quote == real_user_text, (
        f"approved_by_user_quote must be the RAW utterance alone, no "
        f"system_context bleed; got {quote!r}"
    )
    assert "system_context" not in (quote or "")


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(pytest.main([__file__, "-v"]))
