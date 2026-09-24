"""test_issue_2942_preset_save_real_validator.py — ADR-0132 PR-7 gate x2942.

Live regression (Vision Pi, 24.09.2026, session logged in issue #2942):

    03:46:40 tool_calls save_arrangement_preset {...}
    03:46:40 tool: 'ToolValidationError: unexpected argument: approved_by_user_quote'

``_gate_save_arrangement_preset`` (``rob_box_harness/core/agent_core.py``)
unconditionally injects/overwrites ``approved_by_user_quote`` into the tool
call arguments once praise is detected. ``test_issue_adr0132_save_arrangement_
preset_gate.py`` proved the GATE's own logic (praise → call rewritten,
no praise → refused) but ran it against a **fake** ``_RecordingToolProvider``
that never validates a JSON Schema at all — so it could not have caught the
live bug: the real executor (``ROSMCPToolProvider._validate_json_schema``,
``additionalProperties: False``) rejects any argument the tool's declared
schema doesn't list, and the schema (``SaveArrangementPresetTool.parameters``
in ``rob_box_mcp_tools/tools/music.py``) only declared ``note`` — the gate's
own injected argument was "unexpected" to the validator it never went
through in tests.

Fix (issue #2942): ``approved_by_user_quote`` is now declared in the tool's
own parameter list (see ``music.py``, with a description telling the model
NOT to fill it — the gate overwrites it unconditionally regardless of what
the model sends) and the generated catalog
(``rob_box_core/_tool_catalog_data.py``, via ``tools/gen_tool_catalog.py``)
picked it up. This file exercises the REAL pipeline end to end:

    real catalog entry (``rob_box_core.tool_catalog.get_tool``)
      -> ``ROSMCPToolProvider.update_tools`` (real JSON-Schema validator)
      -> ``LegacyToolProviderAdapter`` (real harness/core bridge, the same
         one ``ToolValidationError`` surfaced through live)
      -> ``AgentCore._execute_tool_batch`` / ``_gate_save_arrangement_preset``

No fake registry, no hand-rolled schema — the exact schema the robot ships.

Run with::

    python3 -m pytest src/rob_box_harness/test/test_issue_2942_preset_save_real_validator.py -v
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Any, Mapping

import pytest

from rob_box_core.ports import ToolContext
from rob_box_core.tool_catalog import get_tool
from rob_box_harness.core.agent_core import AgentCore
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateMachine,
)
from rob_box_harness.executors import ROSMCPToolProvider, adapt_tool_provider
from rob_box_llm.provider import LLMMessage, LLMResponse, ToolCall, ToolResult


# ---------------------------------------------------------------------
# Fakes — mirrors test_issue_adr0132_save_arrangement_preset_gate.py's
# own docstring convention: copy fixtures rather than share them across
# regression modules. The ONLY thing that must NOT be fake here is the
# tool registry/validator, which is why this file builds its provider
# from the real ``ROSMCPToolProvider`` + the real generated catalog
# instead of reusing that file's ``_RecordingToolProvider``.
# ---------------------------------------------------------------------


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


@dataclass
class _RecordedInvocation:
    name: str
    parameters: dict[str, Any]
    timeout: float | None


class _RecordingBridge:
    """Stands in for the legacy ROS2 MCP bridge (``execute_tool_call_sync``).

    Only the transport is fake — everything ABOVE it (schema validation in
    ``ROSMCPToolProvider``, the ``ToolValidationError`` -> ``is_error`` bridge
    in ``LegacyToolProviderAdapter``, and the gate in ``AgentCore``) is the
    real production code path.
    """

    def __init__(self) -> None:
        self.invocations: list[_RecordedInvocation] = []

    def execute_tool_call_sync(
        self,
        tool_name: str,
        parameters: dict[str, Any],
        timeout: float | None = None,
    ) -> Mapping[str, Any]:
        self.invocations.append(
            _RecordedInvocation(name=tool_name, parameters=dict(parameters), timeout=timeout)
        )
        return {
            "success": True,
            "message": f"Пресет «В пещере горного короля» сохранён: bass_style=root.",
            "data": {"melody_key": "in_the_hall_of_the_mountain_king"},
        }


def _real_save_preset_provider() -> tuple[ROSMCPToolProvider, _RecordingBridge]:
    """Build the SAME provider stack production uses for one tool.

    ``get_tool(...).to_openai_tool()`` is the exact dict the LLM is shown and
    that ``ROSMCPToolProvider.update_tools`` parses into a
    :class:`ToolDescriptor` — no hand-written schema, so a drift between
    ``music.py``'s ``parameters`` and what the validator enforces cannot
    hide here the way it did in the fake-registry gate tests.
    """
    bridge = _RecordingBridge()
    provider = ROSMCPToolProvider(bridge)
    entry = get_tool("save_arrangement_preset")
    provider.update_tools([entry.to_openai_tool()])
    return provider, bridge


def _wake(core: AgentCore) -> None:
    core._dsm.on_event(DialogueEvent.WAKE_WORD)


def _build_core(responses: list[LLMResponse]) -> tuple[AgentCore, ROSMCPToolProvider, _RecordingBridge, _ScriptedLLM]:
    llm = _ScriptedLLM(responses)
    provider, bridge = _real_save_preset_provider()
    tools = adapt_tool_provider(provider)
    memory = _FakeMemoryStore()
    dsm = DialogueStateMachine()
    core = AgentCore(llm=llm, tools=tools, memory=memory, dsm=dsm)
    _wake(core)
    return core, provider, bridge, llm


def _run(core: AgentCore, text: str) -> Any:
    return asyncio.run(core.process_input(text, history=[]))


def _save_call(quote: str | None = None) -> ToolCall:
    args: dict[str, Any] = {"note": "звучит собранно"}
    if quote is not None:
        args["approved_by_user_quote"] = quote
    return ToolCall(id="s1", name="save_arrangement_preset", arguments=args)


# ---------------------------------------------------------------------
# 0) Sanity — the REAL catalog schema round-trips through the REAL
#    validator directly (no AgentCore/gate involved). Pins the actual
#    live bug: before the fix, this call alone raised
#    ``ToolValidationError: unexpected argument: approved_by_user_quote``.
# ---------------------------------------------------------------------


def test_real_schema_accepts_gate_injected_argument_directly() -> None:
    provider, bridge = _real_save_preset_provider()

    validation = provider.validate_args(
        "save_arrangement_preset",
        {"note": "звучит собранно", "approved_by_user_quote": "вот это кайф, сохрани"},
    )

    assert validation.valid, f"real catalog schema rejected the gate's argument: {validation.errors!r}"

    result = asyncio.run(
        provider.invoke(
            "save_arrangement_preset",
            {"note": "звучит собранно", "approved_by_user_quote": "вот это кайф, сохрани"},
            ToolContext(),
        )
    )
    assert result.is_error is False
    assert bridge.invocations[0].parameters["approved_by_user_quote"] == "вот это кайф, сохрани"


# ---------------------------------------------------------------------
# 1) Praise -> tool executes through the REAL validator, preset stored
#    with the REAL quote.
# ---------------------------------------------------------------------


def test_praise_executes_through_real_validator_with_real_quote() -> None:
    fabricated_quote = "юзер сказал что это гениально (выдумано моделью)"
    real_user_text = "вот это кайф, огонь! сохрани этот вариант"

    scripted = [
        LLMResponse(
            content="",
            tool_calls=(_save_call(quote=fabricated_quote),),
            finish_reason="tool_calls",
        ),
        LLMResponse(content="done", tool_calls=()),
    ]

    core, _provider, bridge, _llm = _build_core(scripted)

    result = _run(core, real_user_text)

    assert len(bridge.invocations) == 1, (
        f"save_arrangement_preset must reach the real bridge exactly once "
        f"through the real validator; got {bridge.invocations!r}"
    )
    invocation = bridge.invocations[0]
    assert invocation.name == "save_arrangement_preset"
    assert invocation.parameters["approved_by_user_quote"] == real_user_text, (
        "approved_by_user_quote reaching the REAL executor must be the "
        "actual user turn, not the model's own (possibly fabricated) quote"
    )
    assert invocation.parameters["approved_by_user_quote"] != fabricated_quote

    # No ToolValidationError leaked into the model-facing turn as an
    # is_error tool message (the live bug's user-visible symptom).
    assert result is not None


# ---------------------------------------------------------------------
# 2) No praise -> refused BEFORE validation/execution — the real
#    validator/bridge must never even see the call.
# ---------------------------------------------------------------------


def test_no_praise_refuses_before_reaching_real_validator() -> None:
    scripted = [
        LLMResponse(
            content="",
            tool_calls=(_save_call(quote="модель придумала цитату"),),
            finish_reason="tool_calls",
        ),
        LLMResponse(content="done", tool_calls=()),
    ]

    core, _provider, bridge, llm = _build_core(scripted)

    _run(core, "сыграй ещё раз")

    assert bridge.invocations == [], (
        f"save_arrangement_preset must never reach the real bridge without "
        f"praise; got {bridge.invocations!r}"
    )

    second_iter = llm.messages_seen[1]
    tool_msgs = [m for m in second_iter if m.role == "tool"]
    assert len(tool_msgs) == 1
    assert "save_arrangement_preset" in tool_msgs[0].content
    assert "отклонён" in tool_msgs[0].content or "похвал" in tool_msgs[0].content.lower()


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(pytest.main([__file__, "-v"]))
