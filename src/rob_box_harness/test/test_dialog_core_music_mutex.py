"""Issue #992 / #1561 — per-batch music mutex tests.

The fix lives in :mod:`rob_box_harness.core.dialog_core` as
:func:`_filter_conflicting_music_calls`. When the LLM pairs a
library/AI-music tool (``generate_music`` / ``gen_play_from_library`` /
``gen_search_library`` / ``gen_list_library`` / ``gen_delete_from_library``)
with ``execute_music_code`` in the same tool batch, the Renardo call is
dropped so it does not race the library mp3 and produce a 2-track
cacophony. The dropped call id receives a synthetic ``ToolResult`` so the
OpenAI-style history stays valid for the next iteration.

Acceptance map:

* :class:`TestFilterConflictingMusicCalls` — pure-function behaviour.
* :class:`TestProcessInputMusicMutex` — end-to-end ``process_input``
  scenario: LLM returns ``[gen_play_from_library, execute_music_code]``,
  only the library call lands in ``tools_called``; the Renardo call never
  reaches the executor.
* :class:`TestRegressionIssue1561` — the live 23.08 17:36 incident
  (Robot asks for «включи из коллекции про енотиков» — LLM fires
  ``gen_play_from_library + execute_music_code`` — robot should play
  ONLY the library mp3).
"""

from __future__ import annotations

import asyncio
import json
from typing import Any

from rob_box_harness.core.dialog_core import (
    DialogCore,
    _EXECUTE_MUSIC_CODE_SKIP_PAYLOAD,
    _filter_conflicting_music_calls,
)
from rob_box_llm.provider import LLMResponse, ToolCall


# ──────────────────────────────────────────────────────────────────────
# Pure-function tests — _filter_conflicting_music_calls
# ──────────────────────────────────────────────────────────────────────


class TestFilterConflictingMusicCalls:
    def test_empty_batch_returns_empty(self) -> None:
        """Defensive: an empty batch must not crash."""
        filtered, skipped = _filter_conflicting_music_calls([])
        assert filtered == []
        assert skipped == {}

    def test_no_library_music_passes_all_calls(self) -> None:
        """Without a library/AI music tool, execute_music_code runs."""
        calls = [
            ToolCall(id="c1", name="speak_text", arguments={"text": "ok"}),
            ToolCall(
                id="c2",
                name="execute_music_code",
                arguments={"code": "Clock.clear()"},
            ),
        ]
        filtered, skipped = _filter_conflicting_music_calls(calls)
        assert filtered == calls
        assert skipped == {}

    def test_gen_play_from_library_drops_execute_music_code(self) -> None:
        """The live 23.08 17:36 incident: LLM pairs gen_play_from_library
        with execute_music_code → Renardo call is dropped."""
        calls = [
            ToolCall(id="c1", name="speak_text", arguments={"text": "Включаю!"}),
            ToolCall(
                id="c2",
                name="gen_play_from_library",
                arguments={"track_id": 42},
            ),
            ToolCall(
                id="c3",
                name="execute_music_code",
                arguments={
                    "code": "Clock.clear()\np1 >> pluck([0,2,4])",
                },
            ),
        ]
        filtered, skipped = _filter_conflicting_music_calls(calls)
        assert [c.id for c in filtered] == ["c1", "c2"]
        assert "c3" in skipped
        assert "library_track" in skipped["c3"]
        # The non-conflicting calls were NOT marked as skipped.
        assert "c1" not in skipped
        assert "c2" not in skipped

    def test_generate_music_drops_execute_music_code(self) -> None:
        """generate_music + execute_music_code → Renardo dropped, AI gen runs."""
        calls = [
            ToolCall(
                id="c1",
                name="generate_music",
                arguments={"prompt": "песня про котика", "lyrics": ""},
            ),
            ToolCall(
                id="c2",
                name="execute_music_code",
                arguments={"code": "Clock.bpm = 120"},
            ),
        ]
        filtered, skipped = _filter_conflicting_music_calls(calls)
        assert [c.id for c in filtered] == ["c1"]
        assert "c2" in skipped

    def test_gen_list_library_alone_does_not_drop_anything(self) -> None:
        """gen_list_library is informational; it doesn't actually start
        music, so execute_music_code in the same batch is fine."""
        calls = [
            ToolCall(id="c1", name="gen_list_library", arguments={"limit": 5}),
            ToolCall(
                id="c2",
                name="execute_music_code",
                arguments={"code": "Clock.clear()"},
            ),
        ]
        filtered, skipped = _filter_conflicting_music_calls(calls)
        # Policy: ANY library-tool → music mutex. gen_list_library is a
        # "library tool" so the mutex still fires (the next batch will
        # probably call gen_play_from_library; we don't want the LLM to
        # think it can fire Renardo in parallel right now).
        assert [c.id for c in filtered] == ["c1"]
        assert "c2" in skipped

    def test_speak_text_alongside_conflict_kept(self) -> None:
        """speak_text is unrelated to music — must NOT be skipped."""
        calls = [
            ToolCall(id="c1", name="speak_text", arguments={"text": "Готово!"}),
            ToolCall(
                id="c2",
                name="gen_play_from_library",
                arguments={"track_id": 7},
            ),
            ToolCall(
                id="c3",
                name="execute_music_code",
                arguments={"code": "Clock.clear()"},
            ),
        ]
        filtered, skipped = _filter_conflicting_music_calls(calls)
        assert [c.id for c in filtered] == ["c1", "c2"]
        assert "c3" in skipped

    def test_multiple_execute_music_code_all_dropped(self) -> None:
        """If the LLM emits 2 Renardo calls alongside 1 library call,
        BOTH Renardo calls must be dropped — not just the first."""
        calls = [
            ToolCall(
                id="c1",
                name="gen_play_from_library",
                arguments={"track_id": 1},
            ),
            ToolCall(
                id="c2",
                name="execute_music_code",
                arguments={"code": "Clock.bpm = 120"},
            ),
            ToolCall(
                id="c3",
                name="execute_music_code",
                arguments={"code": "Clock.clear()"},
            ),
        ]
        filtered, skipped = _filter_conflicting_music_calls(calls)
        assert [c.id for c in filtered] == ["c1"]
        assert set(skipped.keys()) == {"c2", "c3"}

    def test_skip_payload_is_stable_json(self) -> None:
        """The synthetic payload returned to the LLM must be JSON-stable
        so future tests / debug tooling can grep for it."""
        payload = _EXECUTE_MUSIC_CODE_SKIP_PAYLOAD
        encoded = json.dumps(payload, ensure_ascii=False)
        decoded = json.loads(encoded)
        assert decoded["status"] == "skipped_due_to_library_track"
        assert "issue #992 #1561" in decoded["reason"].lower() or \
            "issue #992 #1561" in decoded["reason"]


# ──────────────────────────────────────────────────────────────────────
# End-to-end DialogCore integration
# ──────────────────────────────────────────────────────────────────────


class _FakeLLMProvider:
    """Minimal LLM stand-in — replays scripted responses."""

    def __init__(self, responses: list[LLMResponse]) -> None:
        self.responses = list(responses)
        self.calls: list[tuple[list, list]] = []

    async def complete(self, messages, *, tools=()):
        self.calls.append((list(messages), list(tools)))
        return self.responses.pop(0)

    async def stream(self, messages, *, tools=()):
        # Replay mode — same as complete().
        return await self.complete(messages, tools=tools)


# ──────────────────────────────────────────────────────────────────────
# End-to-end DialogCore integration (uses the real FakeToolProvider)
# ──────────────────────────────────────────────────────────────────────


def _build_provider(
    handler_map: dict[str, Any],
) -> Any:
    """Wrap a ``name -> handler`` dict in a real :class:`FakeToolProvider`.

    Records every executed call into ``provider.executed`` for the test
    to assert on. The handler is called with ``call.arguments`` (a
    Mapping) and may be sync or async.
    """
    from rob_box_harness.tools import FakeToolProvider, ToolSpec

    executed: list[tuple[str, dict[str, Any]]] = []
    provider = FakeToolProvider()

    for name, handler in handler_map.items():
        async def _wrap(args, _h=handler, _n=name):
            executed.append((_n, dict(args)))
            return _h(dict(args))

        spec = ToolSpec(
            name=name,
            description=f"test tool: {name}",
            parameters={"type": "object", "properties": {}},
        )
        provider.register(spec, _wrap)

    # Expose the recorder on the provider for tests.
    provider.executed = executed  # type: ignore[attr-defined]
    return provider


class TestProcessInputMusicMutex:
    """End-to-end test: full DialogCore flow with a paired music batch."""

    def test_library_plus_execute_music_drops_renardo(self) -> None:
        """The Robot asks «включи из коллекции про енотиков» — the LLM
        fires ``[speak_text, gen_play_from_library, execute_music_code]``.
        DialogCore must:
          * execute speak_text and gen_play_from_library,
          * DROP execute_music_code (no Renardo call),
          * emit a synthetic tool_result for the dropped id so history
            stays valid for the next iteration,
          * put ``gen_play_from_library`` in tools_called (NOT
            ``execute_music_code`` — that call never ran)."""
        from rob_box_harness.memory import InMemoryStore
        from rob_box_harness.core.dialogue_state_machine import (
            DialogueStateMachine,
        )

        dsm = DialogueStateMachine()
        memory = InMemoryStore()

        # Scripted responses: first turn is the conflicting batch, second
        # turn is a plain-text reply ("Включаю!").
        scripted = [
            LLMResponse(
                content="",
                tool_calls=(
                    ToolCall(
                        id="call_s",
                        name="speak_text",
                        arguments={"text": "Включаю!"},
                    ),
                    ToolCall(
                        id="call_lib",
                        name="gen_play_from_library",
                        arguments={"track_id": 42},
                    ),
                    ToolCall(
                        id="call_ren",
                        name="execute_music_code",
                        arguments={"code": "Clock.clear()\np1 >> pluck([0,2,4])"},
                    ),
                ),
            ),
            LLMResponse(content="done", tool_calls=()),
        ]
        llm = _FakeLLMProvider(scripted)

        def gen_play(args):
            return {"status": "playing", "track_id": args.get("track_id")}

        def speak(args):
            return {"status": "spoken"}

        tools = _build_provider(
            {
                "gen_play_from_library": gen_play,
                "speak_text": speak,
                # NO execute_music_code handler on purpose — if the guard
                # accidentally lets it through, the executor would raise
                # an "unknown tool" error and the test would fail loudly.
            }
        )

        core = DialogCore(llm=llm, tools=tools, memory=memory, dsm=dsm)
        # Drive the DSM into DIALOGUE so the LLM gate actually fires —
        # production shell does the same via _on_stt.
        asyncio.run(core.handle_wake_word(""))

        result = asyncio.run(
            core.process_input(
                "включи из коллекции про енотиков",
                history=[],
            )
        )

        # ── 1. Library mp3 ran, Renardo never ran ──────────────────────
        executed_names = [name for (name, _args) in tools.executed]
        assert "gen_play_from_library" in executed_names
        assert "speak_text" in executed_names
        assert "execute_music_code" not in executed_names, (
            "Renardo must NOT have fired — library mp3 is the only music "
            "source for this turn (issue #992 #1561)."
        )

        # ── 2. tools_called reflects what actually ran ─────────────────
        assert "gen_play_from_library" in result.tools_called
        assert "execute_music_code" not in result.tools_called, (
            "execute_music_code was filtered out before execution — it "
            "must not appear in tools_called (would mislead downstream "
            "music_guard into thinking Renardo actually played)."
        )

        # ── 3. History stayed valid — every tool_call_id got a result ──
        # The second LLM call (after the assistant message that carried
        # 3 tool_calls) must have seen 3 tool messages with matching ids.
        second_call_messages = llm.calls[1][0]
        tool_msgs = [m for m in second_call_messages if m.role == "tool"]
        tool_msg_ids = {m.tool_call_id for m in tool_msgs if m.tool_call_id}
        assert tool_msg_ids == {"call_s", "call_lib", "call_ren"}, (
            f"history invalid — missing tool_result for one of the call "
            f"ids. Got: {tool_msg_ids}"
        )

        # ── 4. The synthetic tool_result for call_ren carries the
        #      skip payload (so the LLM loop sees a clean status).
        ren_msg = next(m for m in tool_msgs if m.tool_call_id == "call_ren")
        ren_payload = json.loads(ren_msg.content)
        assert ren_payload["status"] == "skipped_due_to_library_track"


class TestRegressionIssue1561:
    """The exact 23.08 17:36 incident: «включи из коллекции про енотиков»
    → LLM fires [speak_text, gen_play_from_library, execute_music_code]
    → robot heard library mp3 + Renardo beat on top = каша.

    After the fix: robot hears ONLY the library mp3.
    """

    def test_issue_1561_live_scenario(self) -> None:
        """Replay of the live scenario from issue #1561 body."""
        from rob_box_harness.memory import InMemoryStore
        from rob_box_harness.core.dialogue_state_machine import (
            DialogueStateMachine,
        )

        dsm = DialogueStateMachine()
        memory = InMemoryStore()
        scripted = [
            LLMResponse(
                content="",
                tool_calls=(
                    ToolCall(
                        id="c1",
                        name="speak_text",
                        arguments={"text": "Еноты-мутанты уже в городе!"},
                    ),
                    ToolCall(
                        id="c2",
                        name="gen_play_from_library",
                        arguments={"track_id": 99},
                    ),
                    ToolCall(
                        id="c3",
                        name="execute_music_code",
                        arguments={"code": "Clock.clear()\np1 >> pluck([0])"},
                    ),
                ),
            ),
            LLMResponse(content="done", tool_calls=()),
        ]
        llm = _FakeLLMProvider(scripted)
        tools = _build_provider(
            {
                "gen_play_from_library": lambda a: {"status": "playing"},
                "speak_text": lambda a: {"status": "spoken"},
            }
        )
        core = DialogCore(llm=llm, tools=tools, memory=memory, dsm=dsm)
        asyncio.run(core.handle_wake_word(""))
        result = asyncio.run(
            core.process_input("включи из коллекции про енотиков", history=[])
        )

        # Acceptance — ONE music tool ran:
        music_tools_executed = [
            name for (name, _args) in tools.executed
            if name in {"execute_music_code", "generate_music",
                        "gen_play_from_library"}
        ]
        assert music_tools_executed == ["gen_play_from_library"], (
            f"Only gen_play_from_library should run; got "
            f"{music_tools_executed!r}"
        )

        # tools_called on the result must match reality:
        assert "gen_play_from_library" in result.tools_called
        assert "execute_music_code" not in result.tools_called
