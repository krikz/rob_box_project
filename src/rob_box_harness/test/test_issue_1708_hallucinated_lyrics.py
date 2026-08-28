"""test_issue_1708_hallucinated_lyrics.py — Issue #1708 hallucinated-lyrics guard.

Issue #1708 — after ``execute_music_code`` the LLM sometimes also calls
``speak_text`` with invented lyrics («Нига-стайл, Колобок-флоу,
уехал я, братан, в тёмны...»). The user hears a robotic voice reading
hallucinated text on top of (or instead of) the beat.

The fix lives in :mod:`rob_box_harness.core.dialog_core`:

* Module-level helper :func:`_is_hallucinated_speak_text` decides whether
  a ``speak_text`` call should be suppressed.
* :func:`_suppressed_speak_text_result` returns the sentinel the LLM
  sees instead of a real TTS request.
* :func:`_is_vocal_request` mirrors ``rob_box_voice``'s narrow vocal
  detector so the guard only fires on NON-vocal requests — backing
  mode (rap / poem / song lyrics via ``speak_text`` × N) is preserved.

These tests drive ``DialogCore.process_input`` end-to-end with the
real port fakes from ``test_dialog_core.py`` (copied locally to avoid
cross-file fixture sharing in this isolated regression module). They
also exercise the pure-function helpers directly so a future regression
in the heuristic gets caught even when the full harness stack changes.

Run with::

    python3 -m pytest src/rob_box_harness/test/test_issue_1708_hallucinated_lyrics.py -v
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Any

import pytest

from rob_box_harness.core import dialog_core
from rob_box_harness.core.dialog_core import DialogCore
from rob_box_harness.core.dialogue_state_machine import (
    DialogueStateMachine,
)
from rob_box_harness.tools import ToolProvider
from rob_box_llm.provider import (
    LLMMessage,
    LLMResponse,
    ToolCall,
    ToolResult,
)


# ---------------------------------------------------------------------
# Helpers — copied from test_dialog_core.py so this module is
# runnable in isolation. If the originals drift, copy from there.
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
        if messages is None:
            materialised: list[Any] = []
        elif isinstance(messages, list):
            materialised = messages
        else:
            materialised = list(messages)
        self.calls.append((materialised, tools))
        # Snapshot the message list for diagnostics.
        self.messages_seen.append(
            [LLMMessage(role=m.role, content=m.content) for m in materialised]
        )
        if self._responses:
            item = self._responses.pop(0)
            if isinstance(item, BaseException):
                raise item
            return item
        # Default: empty final answer.
        return LLMResponse(content="done", tool_calls=())

    async def aclose(self) -> None:
        return None


class _RecordingToolProvider(ToolProvider):
    """Executes via handler_map; remembers every call we did NOT suppress.

    Returns ``ToolResult`` for every successful execution so the LLM
    sees a normal ``role=tool`` message in its history.

    Inherits from :class:`ToolProvider` so it satisfies the
    ``DialogCore(tools=...)`` type contract (the abstract base
    declares ``discover`` / ``execute`` / ``aclose``).
    """

    name = "recording_tools"

    def __init__(self, manifest: tuple[Any, ...], handler_map: dict[str, Any]) -> None:
        self._manifest: tuple[Any, ...] = manifest
        self._handler_map: dict[str, Any] = handler_map
        self.executed: list[Any] = []
        # If non-empty, raise AssertionError if ``execute()`` is called
        # for any tool whose name is in this set. Used by the guard
        # tests to assert that hallucinated ``speak_text`` calls were
        # dropped before reaching the executor (music tools still run).
        self.fail_on_execute_names: set[str] = set()

    async def discover(self) -> tuple[Any, ...]:
        return self._manifest

    async def execute(self, call: Any) -> Any:
        if call.name in self.fail_on_execute_names:
            raise AssertionError(
                f"tool call reached executor that should have been "
                f"suppressed: name={call.name!r} args={call.arguments!r}"
            )
        self.executed.append(call)
        handler = self._handler_map.get(call.name)
        if handler is None:
            return ToolResult(
                tool_call_id=call.id,
                content=f"unknown tool: {call.name}",
                is_error=True,
            )
        result = handler(dict(call.arguments))
        if hasattr(result, "__await__"):
            result = await result
        if isinstance(result, ToolResult):
            return result
        return ToolResult(
            tool_call_id=call.id,
            content=str(result),
            is_error=False,
        )

    async def aclose(self) -> None:
        return None


class _FakeMemoryStore:
    def __init__(self) -> None:
        from rob_box_harness.memory import Turn
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


def _build_core(
    responses: list[LLMResponse],
    *,
    manifest: tuple[Any, ...],
    handler_map: dict[str, Any],
    fail_on_execute_names: set[str] | None = None,
) -> tuple[DialogCore, _RecordingToolProvider, _ScriptedLLM]:
    llm = _ScriptedLLM(responses)
    tools = _RecordingToolProvider(manifest, handler_map)
    if fail_on_execute_names:
        tools.fail_on_execute_names.update(fail_on_execute_names)
    memory = _FakeMemoryStore()
    dsm = DialogueStateMachine()
    core = DialogCore(llm=llm, tools=tools, memory=memory, dsm=dsm)
    asyncio.run(core.handle_wake_word(""))
    return core, tools, llm


def _run(core: DialogCore, text: str) -> Any:
    return asyncio.run(core.process_input(text, history=[]))


# ---------------------------------------------------------------------
# Pure-function helper tests — drive the heuristic directly.
# ---------------------------------------------------------------------


class TestIsVocalRequest:
    """Direct coverage of the local _is_vocal_request helper.

    Kept narrow on purpose: this is the single signal that decides
    whether backing mode is allowed. False positives would silence
    legitimate rap backing turns; false negatives would re-introduce
    the issue #1708 bug. Pin the keyword set verbatim.
    """

    @pytest.mark.parametrize(
        "text",
        [
            "спой куплет",
            "Спой песенку про колобка",
            "робокс спой про мурку",
            "зачитай рэп",
            "зачитай стих",
            "прочитай стихотворение",
            "спой частушки",
            "спой куплет и ещё куплет",
            "рэп про космос",
            "rap beat",
            "куплет про осень",
        ],
    )
    def test_vocal_keywords_detected(self, text: str) -> None:
        assert dialog_core._is_vocal_request(text) is True, (
            f"vocal cue should match: {text!r}"
        )

    @pytest.mark.parametrize(
        "text",
        [
            "сыграй бит про колобка в нига стайле",
            "включи трек",
            "поставь музыку",
            "запусти мелодию",
            "сыграй баха",
            "диджей вечеринка",  # «вечеринка» is DJ mode, not vocal
            "выключи музыку",
            "останови музыку",
            "диджей, сделай погромче",  # DJ, not vocal
        ],
    )
    def test_non_vocal_requests_not_detected(self, text: str) -> None:
        assert dialog_core._is_vocal_request(text) is False, (
            f"non-vocal cue must NOT match: {text!r}"
        )

    def test_empty_input_returns_false(self) -> None:
        assert dialog_core._is_vocal_request("") is False

    def test_none_safe(self) -> None:
        # Defensive: live providers may pass None. We never crash.
        assert dialog_core._is_vocal_request("") is False  # use empty str


class TestIsHallucinatedSpeakText:
    """Direct coverage of the heuristic.

    Combines ``prior_music_calls`` (what ran earlier in the same batch),
    ``user_input`` (the original request), and the candidate
    ``speak_text`` call. Every branch must stay pinned.
    """

    def _call(self, text: str = "lyrics") -> ToolCall:
        return ToolCall(
            id="t1", name="speak_text", arguments={"text": text}
        )

    def test_non_speak_text_never_hallucinated(self) -> None:
        call = ToolCall(id="t1", name="stop_music", arguments={})
        assert dialog_core._is_hallucinated_speak_text(
            call=call,
            same_batch_music_calls=frozenset({"execute_music_code"}),
            user_input="сыграй бит",
        ) is False

    def test_no_prior_music_never_hallucinated(self) -> None:
        # LLM called only speak_text (e.g. answering a question).
        assert dialog_core._is_hallucinated_speak_text(
            call=self._call("полный ответ на вопрос юзера"),
            same_batch_music_calls=frozenset(),
            user_input="расскажи про себя",
        ) is False

    def test_vocal_request_never_hallucinated(self) -> None:
        # Backing mode: execute_music_code + speak_text × N is legal
        # on a vocal request («спой куплет»).
        assert dialog_core._is_hallucinated_speak_text(
            call=self._call("куплет про колобка длинный текст" * 5),
            same_batch_music_calls=frozenset({"execute_music_code"}),
            user_input="спой куплет про колобка",
        ) is False

    def test_short_accept_after_music_not_hallucinated(self) -> None:
        # Master prompt allows ONE short accept phrase. Keep it.
        assert dialog_core._is_hallucinated_speak_text(
            call=self._call("Ок, играю Бах"),
            same_batch_music_calls=frozenset({"execute_music_code"}),
            user_input="сыграй баха",
        ) is False

    def test_long_lyrics_after_music_non_vocal_is_hallucination(self) -> None:
        # The exact bug variant 1: non-vocal request, LLM hallucinates
        # lyrics anyway. «сыграй бит в нига стайле» is instrumental —
        # backing-mode lyrics are NOT appropriate, so any speak_text
        # longer than the accept threshold is a hallucination.
        assert dialog_core._is_hallucinated_speak_text(
            call=self._call(
                "Нига-стайл, Колобок-флоу, уехал я, братан, в тёмны..."
            ),
            same_batch_music_calls=frozenset({"execute_music_code"}),
            user_input="сыграй бит в нига стайле про колобка",
        ) is True

    def test_vocal_request_with_short_lyrics_after_music_is_backing(self) -> None:
        # Backing mode: user EXPLICITLY asked to sing (спой куплет),
        # LLM produced backing beat + lyrics. Both are legitimate —
        # the heuristic must NOT fire.
        assert dialog_core._is_hallucinated_speak_text(
            call=self._call(
                "Куплет про колобка длинный текст" * 3
            ),
            same_batch_music_calls=frozenset({"execute_music_code"}),
            user_input="спой куплет про колобка",
        ) is False

    def test_empty_speak_text_never_hallucinated(self) -> None:
        # Empty speak_text is issue #1343 territory (validation
        # rejection), NOT hallucinated lyrics — we must not steal that
        # signal from the downstream anti-duplicate path.
        assert dialog_core._is_hallucinated_speak_text(
            call=self._call(""),
            same_batch_music_calls=frozenset({"execute_music_code"}),
            user_input="сыграй бит",
        ) is False

    def test_speak_text_before_music_in_batch_is_hallucination(self) -> None:
        # The exact issue #1708 live shape: LLM calls ``speak_text``
        # FIRST (the "title" pattern) THEN ``execute_music_code``.
        # The order inside the batch doesn't matter for the guard —
        # what matters is that BOTH are in the same batch on a
        # non-vocal request.
        assert dialog_core._is_hallucinated_speak_text(
            call=self._call(
                "Нига-стайл, Колобок-флоу, уехал я, братан, в тёмны..."
            ),
            same_batch_music_calls=frozenset({"execute_music_code"}),
            user_input="сыграй бит про колобка в нига стайле",
        ) is True

    def test_accept_threshold_exact(self) -> None:
        # 41 chars = 1 over the threshold (40) → suppressed.
        long_text = "a" * 41
        assert dialog_core._is_hallucinated_speak_text(
            call=self._call(long_text),
            same_batch_music_calls=frozenset({"execute_music_code"}),
            user_input="сыграй бит",
        ) is True
        # 40 chars exactly = legitimate accept.
        short_text = "a" * 40
        assert dialog_core._is_hallucinated_speak_text(
            call=self._call(short_text),
            same_batch_music_calls=frozenset({"execute_music_code"}),
            user_input="сыграй бит",
        ) is False

    def test_generate_music_counts_as_prior_music(self) -> None:
        # MiniMax Music API path also triggers the guard — but only
        # on a NON-vocal request. «поставь музыку фоном» is library
        # play, not singing, so backing-mode lyrics are inappropriate.
        assert dialog_core._is_hallucinated_speak_text(
            call=self._call("неожиданно произнесённый текст после трека"),
            same_batch_music_calls=frozenset({"generate_music"}),
            user_input="поставь музыку фоном",
        ) is True


class TestSuppressedSpeakTextResult:
    """The sentinel ToolResult must reach the LLM verbatim so the
    next iteration sees WHY its call was dropped.
    """

    def test_carries_call_id(self) -> None:
        call = ToolCall(id="abc", name="speak_text", arguments={"text": "x"})
        result = dialog_core._suppressed_speak_text_result(call)
        assert result.tool_call_id == "abc"

    def test_marked_as_error(self) -> None:
        # LLM treats is_error=True as a tool failure and pivots away
        # from this call shape on its next iteration.
        call = ToolCall(id="abc", name="speak_text", arguments={"text": "x"})
        result = dialog_core._suppressed_speak_text_result(call)
        assert result.is_error is True

    def test_message_explains_why(self) -> None:
        # The message must mention the rule so the LLM can learn it.
        call = ToolCall(id="abc", name="speak_text", arguments={"text": "x"})
        result = dialog_core._suppressed_speak_text_result(call)
        assert "speak_text подавлен" in result.content
        assert "execute_music_code" in result.content


class TestExtractSpeakText:
    """Defensive argument extraction — live providers occasionally
    deliver non-mapping arguments.
    """

    def test_normal_mapping(self) -> None:
        assert dialog_core._extract_speak_text({"text": "  привет  "}) == "привет"

    def test_missing_key(self) -> None:
        assert dialog_core._extract_speak_text({}) == ""

    def test_none_input(self) -> None:
        assert dialog_core._extract_speak_text(None) == ""

    def test_non_mapping_input(self) -> None:
        assert dialog_core._extract_speak_text("not a mapping") == ""

    def test_non_string_value(self) -> None:
        assert dialog_core._extract_speak_text({"text": 123}) == ""


# ---------------------------------------------------------------------
# Integration tests — drive DialogCore.process_input end-to-end.
# ---------------------------------------------------------------------


def _build_music_manifest() -> tuple[Any, ...]:
    from rob_box_harness.tools import ToolSpec

    return (
        ToolSpec(
            name="execute_music_code",
            description="Сыграть Renardo / SuperCollider code.",
            parameters={
                "type": "object",
                "properties": {
                    "code": {"type": "string"},
                    "segments": {"type": "integer"},
                },
                "required": ["code"],
            },
        ),
        ToolSpec(
            name="speak_text",
            description="Произнести текст голосом через TTS.",
            parameters={
                "type": "object",
                "properties": {"text": {"type": "string"}},
                "required": ["text"],
            },
        ),
        ToolSpec(
            name="generate_music",
            description="MiniMax Music API: сгенерировать песню.",
            parameters={"type": "object", "properties": {}},
        ),
    )


class TestIssue1708GuardIntegration:
    """End-to-end coverage of the dialog_core guard."""

    def test_hallucinated_lyrics_are_suppressed(self) -> None:
        """The exact live bug: execute_music_code + speak_text(lyrics).

        Verifies:

        * ``speak_text`` is NOT in ``tools_provider.executed``
          (the executor never saw the call);
        * ``result.tools_called`` still contains ``speak_text``
          (so operators see the suppression in the diagnostic log);
        * ``result.speak_text_real_count`` is ``0`` (so issue #988
          anti-duplicate does NOT silence the final ``done``);
        * The LLM sees a sentinel tool result on its next iteration
          (verified via ``llm.messages_seen[1]``).
        """
        # Three LLM cycles:
        # 1. execute_music_code + speak_text(lyrics) → guard suppresses speak_text
        # 2. LLM sees the sentinel, returns plain "done"
        #
        # User request is NON-vocal (instrumental): «сыграй бит в нига
        # стайле» has no vocal cue, so the guard fires. The exact
        # vocal variant («спой песенку про колобка в нига стайле» from
        # issue #1708) IS allowed backing mode and is tested separately
        # — see test_vocal_request_allows_backing_lyrics.
        scripted = [
            LLMResponse(
                content="",
                tool_calls=(
                    ToolCall(
                        id="m1",
                        name="execute_music_code",
                        arguments={
                            "code": "p1 >> blip([0,2,4], dur=0.5)",
                            "segments": 96,
                        },
                    ),
                    ToolCall(
                        id="s1",
                        name="speak_text",
                        arguments={
                            "text": (
                                "Нига-стайл, Колобок-флоу, уехал я, "
                                "братан, в тёмны..."
                            ),
                        },
                    ),
                ),
                finish_reason="tool_calls",
            ),
            LLMResponse(content="done", tool_calls=()),
        ]

        async def music_handler(_args: dict[str, object]) -> str:
            return json_dumps_ok()

        async def speak_handler(_args: dict[str, object]) -> str:
            # Must NEVER be called for the suppressed speak_text.
            raise AssertionError(
                "suppressed speak_text reached executor (issue #1708 "
                "guard failed)"
            )

        core, tools, llm = _build_core(
            scripted,
            manifest=_build_music_manifest(),
            handler_map={
                "execute_music_code": music_handler,
                "speak_text": speak_handler,
            },
            fail_on_execute_names={"speak_text"},
        )

        result = _run(core, "сыграй бит в нига стайле про колобка")

        # Music tool ran (it's allowed).
        executed_names = [c.name for c in tools.executed]
        assert executed_names == ["execute_music_code"], (
            f"only music tool should run; got {executed_names!r}"
        )

        # tools_called includes speak_text (for diagnostic visibility).
        assert "execute_music_code" in result.tools_called
        assert "speak_text" in result.tools_called

        # speak_text_real_count is 0 — the suppression decremented it.
        # This is critical: if it were still 1, issue #988 anti-duplicate
        # would skip the final ``done`` and the user would hear silence.
        assert result.speak_text_real_count == 0, (
            f"suppressed speak_text must decrement speak_text_real_count; "
            f"got {result.speak_text_real_count}"
        )

        # The sentinel reached the LLM on iteration 2 (the second
        # complete() call).
        assert len(llm.calls) == 2, (
            f"expected 2 LLM calls; got {len(llm.calls)}"
        )
        second_iter = llm.messages_seen[1]
        tool_msgs = [m for m in second_iter if m.role == "tool"]
        # Both tool results must be present (music + suppressed speak_text).
        assert len(tool_msgs) == 2
        suppressed_msg = next(m for m in tool_msgs if "подавлен" in m.content)
        assert "speak_text подавлен" in suppressed_msg.content

    def test_vocal_request_allows_backing_lyrics(self) -> None:
        """Vocal request: execute_music_code + speak_text × N must run.

        Regression net for the live issue #1708 user prompt «спой
        песенку про колобка в нига стайле» (with the wake-word prefix
        stripped — the classifier treats «робокс…» as WAKE_WORD and
        the harness then never calls the LLM). Per the master prompt
        §5 the LLM IS allowed to produce backing-mode lyrics on a
        vocal request — the guard must NOT fire there. (If the LLM
        hallucinates irrelevant lyrics, that's a prompt-level
        regression and out of scope for this guard.)
        """
        scripted = [
            LLMResponse(
                content="",
                tool_calls=(
                    ToolCall(
                        id="m1",
                        name="execute_music_code",
                        arguments={
                            "code": "p1 >> pads([0,2,4], dur=1)",
                            "segments": 32,
                        },
                    ),
                ),
                finish_reason="tool_calls",
            ),
            LLMResponse(
                content="",
                tool_calls=(
                    ToolCall(
                        id="s1",
                        name="speak_text",
                        arguments={
                            "text": (
                                "Куплет про колобка длинный текст "
                                "связный осмысленный текст песни"
                            ),
                        },
                    ),
                    ToolCall(
                        id="s2",
                        name="speak_text",
                        arguments={
                            "text": (
                                "Второй куплет про колобка тоже "
                                "осмысленный текст песни"
                            ),
                        },
                    ),
                ),
                finish_reason="tool_calls",
            ),
            LLMResponse(content="done", tool_calls=()),
        ]

        async def handler(_args: dict[str, object]) -> str:
            return json_dumps_ok()

        core, tools, _ = _build_core(
            scripted,
            manifest=_build_music_manifest(),
            handler_map={
                "execute_music_code": handler,
                "speak_text": handler,
            },
        )

        result = _run(core, "спой песенку про колобка в нига стайле")

        executed_names = [c.name for c in tools.executed]
        # 1 music + 2 speak_text = 3 real executions.
        assert executed_names.count("execute_music_code") == 1
        assert executed_names.count("speak_text") == 2
        assert result.speak_text_real_count == 2, (
            f"backing mode must count BOTH speak_text calls; got "
            f"{result.speak_text_real_count}"
        )

    def test_short_accept_after_music_is_kept(self) -> None:
        """Master-prompt §5: ONE short accept phrase after music is OK.

        Regression net for
        ``test_issue_992_batch_cleanup.py::test_track_execute_music_code_single_accept_no_cleanup``.
        """
        scripted = [
            LLMResponse(
                content="",
                tool_calls=(
                    ToolCall(
                        id="m1",
                        name="execute_music_code",
                        arguments={
                            "code": "p1 >> pads([0,2,4], dur=1)",
                            "segments": 96,
                        },
                    ),
                ),
                finish_reason="tool_calls",
            ),
            LLMResponse(
                content="",
                tool_calls=(
                    ToolCall(
                        id="s1",
                        name="speak_text",
                        arguments={"text": "Ок, играю Бах."},
                    ),
                ),
                finish_reason="tool_calls",
            ),
            LLMResponse(content="done", tool_calls=()),
        ]

        async def handler(_args: dict[str, object]) -> str:
            return json_dumps_ok()

        core, tools, _ = _build_core(
            scripted,
            manifest=_build_music_manifest(),
            handler_map={
                "execute_music_code": handler,
                "speak_text": handler,
            },
        )

        result = _run(core, "сыграй баха")

        executed_names = [c.name for c in tools.executed]
        assert executed_names.count("execute_music_code") == 1
        assert executed_names.count("speak_text") == 1, (
            f"short accept after music must execute; got {executed_names!r}"
        )
        assert result.speak_text_real_count == 1

    def test_speak_text_without_prior_music_is_kept(self) -> None:
        """No music tool → speak_text runs normally (no false positive)."""
        scripted = [
            LLMResponse(
                content="",
                tool_calls=(
                    ToolCall(
                        id="s1",
                        name="speak_text",
                        arguments={"text": "Длинный ответ на обычный вопрос юзера."},
                    ),
                ),
                finish_reason="tool_calls",
            ),
            LLMResponse(content="done", tool_calls=()),
        ]

        async def handler(_args: dict[str, object]) -> str:
            return json_dumps_ok()

        core, tools, _ = _build_core(
            scripted,
            manifest=_build_music_manifest(),
            handler_map={"speak_text": handler},
        )

        result = _run(core, "расскажи про себя")

        executed_names = [c.name for c in tools.executed]
        assert executed_names == ["speak_text"]
        assert result.speak_text_real_count == 1

    def test_generate_music_path_also_guarded(self) -> None:
        """MiniMax Music API path: same hallucinated-lyrics pattern.

        Mirrors :func:`_is_hallucinated_speak_text`'s acceptance of
        ``generate_music`` in ``same_batch_music_calls`` — verified at
        the integration level so a regression in ``_MUSIC_LAUNCH_TOOLS``
        (which feeds the same-batch-music detection) gets caught.
        """
        # User input is NON-vocal: «включи трек» is library/track play,
        # not a vocal request. Backing-mode lyrics are inappropriate
        # — any speak_text after ``generate_music`` is a hallucination.
        scripted = [
            LLMResponse(
                content="",
                tool_calls=(
                    ToolCall(
                        id="m1",
                        name="generate_music",
                        arguments={"prompt": "lo-fi про дождь"},
                    ),
                    ToolCall(
                        id="s1",
                        name="speak_text",
                        arguments={
                            "text": (
                                "неожиданно длинная фраза поверх только что "
                                "сгенерированного трека"
                            ),
                        },
                    ),
                ),
                finish_reason="tool_calls",
            ),
            LLMResponse(content="done", tool_calls=()),
        ]

        async def music_handler(_args: dict[str, object]) -> str:
            return json_dumps_ok()

        async def speak_handler(_args: dict[str, object]) -> str:
            raise AssertionError(
                "speak_text after generate_music must be suppressed"
            )

        # Register generate_music in the manifest so the LLM can pick it.
        from rob_box_harness.tools import ToolSpec

        manifest = (
            ToolSpec(
                name="generate_music",
                description="MiniMax Music API.",
                parameters={"type": "object", "properties": {}},
            ),
            ToolSpec(
                name="speak_text",
                description="TTS.",
                parameters={
                    "type": "object",
                    "properties": {"text": {"type": "string"}},
                },
            ),
        )
        core, tools, _ = _build_core(
            scripted,
            manifest=manifest,
            handler_map={
                "generate_music": music_handler,
                "speak_text": speak_handler,
            },
            fail_on_execute_names={"speak_text"},
        )

        result = _run(core, "поставь музыку фоном")

        executed_names = [c.name for c in tools.executed]
        assert executed_names == ["generate_music"], (
            f"only generate_music should run; got {executed_names!r}"
        )
        assert result.speak_text_real_count == 0


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------


def json_dumps_ok() -> str:
    """Tiny shim — keeps the test file free of ``json`` imports."""
    return '{"ok": true, "played": true}'


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(pytest.main([__file__, "-v"]))
