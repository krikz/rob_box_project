"""``DialogCore`` — high-level facade over the four dialogue ports.

Composes:

* :class:`rob_box_llm.LLMProvider`       — LLM client (``complete`` / ``stream``)
* :class:`rob_box_harness.tools.ToolProvider` — tool dispatch
* :class:`rob_box_harness.memory.MemoryStore` — conversation + facts persistence
* :class:`DialogueStateMachine`          — pure lifecycle state machine

into a single async ``process_input`` entry point so the dialogue
shell (a thin ROS2 node) can stay ≤350 LOC.

The class itself is pure Python — no ROS2, no ``rclpy``. The shell
calls ``process_input`` from its STT callback and publishes the
returned :class:`DialogResult` to ``/voice/dialogue/response``.

Per ``.planning/06-01-PLAN.md`` §W3, the class composes the ports
without absorbing their responsibilities. Tool dispatch and state
transitions still go through the existing port APIs — DialogCore is
the *orchestrator*, not a duplicate of the ports.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Iterable

from rob_box_harness.core.confirmation_policy import ConfirmationKind
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateKind,
    DialogueStateMachine,
)
from rob_box_harness.memory import MemoryStore
from rob_box_harness.tools import ToolProvider, ToolSpec
from rob_box_llm.provider import LLMMessage, LLMProvider, LLMResponse, ToolCall, ToolResult

if TYPE_CHECKING:
    # Forward import — AcceptanceGate is in rob_box_harness.core.acceptance
    # and importing it eagerly here would create a circular dependency
    # with the package's __init__ exports.
    from rob_box_harness.core.acceptance import AcceptanceGate


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------


#: Hard cap on tool-loop iterations inside ``process_input``. Mirrors the
#: legacy OpenAI Agents SDK ``max_turns`` ceiling so a runaway tool loop
#: can't pin the dialogue thread forever. Five is enough for memory_context
#: + a follow-up explanation, and short enough that a misbehaving tool
#: fails loudly rather than running away.
_MAX_TOOL_ITERATIONS: int = 8


# ---------------------------------------------------------------------------
# Result type
# ---------------------------------------------------------------------------


@dataclass
class DialogResult:
    """Outcome of a single :meth:`DialogCore.process_input` call.

    The shell consumes these fields to decide what to publish on
    ``/voice/dialogue/response`` / ``/voice/dialogue/state``.

    Fields
    ------
    spoken_text:
        The assistant's reply (already through the LLM). Empty string
        when the LLM errored before producing output.
    new_state:
        The DSM state after processing (typically ``IDLE`` for a
        successful turn).
    tools_called:
        Names of tools the LLM invoked during this turn — populated
        by the in-core tool loop (see :meth:`DialogCore.process_input`
        for the full contract). Empty when the model answered in
        plain text or the tool loop did not run.
    error:
        ``None`` on success, otherwise the exception that aborted the
        turn. The shell logs this but does not raise it further.
    """

    spoken_text: str = ""
    new_state: DialogueStateKind = DialogueStateKind.IDLE
    tools_called: list[str] = field(default_factory=list)
    error: BaseException | None = None
    # ── LLM diagnostics (live 16:58) ────────────────────────────────────
    # When the LLM returns empty content (MiniMax M3 Interleaved Thinking
    # compaction, timeouts, finish_reason='length'), we want to know WHY
    # in the debug log. Populated by DialogCore from the last LLMResponse
    # and consumed by dialogue_node when spoken_text is empty.
    finish_reason: str | None = None
    raw_response: Any | None = None


# ---------------------------------------------------------------------------
# DialogCore
# ---------------------------------------------------------------------------


class DialogCore:
    """Orchestrator for a single conversation turn.

    The shell drives the loop:

    1. STT callback receives text → ``process_input(text, history)``
    2. Core persists the user turn, calls the LLM, persists the
       assistant turn, and returns a :class:`DialogResult`.
    3. Shell publishes the result.

    The core is intentionally stateless across turns — every turn is a
    self-contained ``process_input`` call. The DSM keeps the lifecycle
    state, the MemoryStore keeps the conversation, the LLM keeps no
    state at all (the shell passes the history explicitly).
    """

    def __init__(
        self,
        *,
        llm: LLMProvider,
        tools: ToolProvider,
        memory: MemoryStore,
        dsm: DialogueStateMachine,
        user_id: str = "default",
        history_trim_limit: int | None = None,
        inactivity_timeout: float | None = None,
        acceptance_gate: "AcceptanceGate | None" = None,
        system_prompt: str | None = None,
    ) -> None:
        """Compose the four dialogue ports into a single facade.

        Args:
            llm: LLM client — required.
            tools: Tool dispatch port — required. DialogCore uses
                ``tools.discover()`` to fetch the OpenAI-style schema
                for ``llm.complete(messages, tools=...)`` and runs
                the tool loop in-process when the LLM asks for a
                tool call. Required even when the LLM never invokes
                a tool, because the loop is the only way the model
                can discover ``memory_context`` and friends.
            memory: Conversation + facts store — required.
            dsm: Dialogue state machine — required.
            user_id: Scope key used when calling the memory store.
                Defaults to ``"default"``.
            history_trim_limit: When set and the caller passes
                ``history=None`` to :meth:`process_input`, ask
                ``memory.load_recent(user_id, limit=history_trim_limit)``
                instead of running with an empty history. Lets the
                shell trim history purely through the memory port
                without re-implementing slicing.
            inactivity_timeout: When set, ``check_timeout()`` drops
                out of ``LISTENING`` after this many seconds of
                silence (forwarded to
                :meth:`DialogueStateMachine.check_inactivity_timeout`).
                ``None`` disables the inactivity check; the shell is
                then expected to drive ``TIMEOUT`` events manually.
            acceptance_gate: Optional :class:`AcceptanceGate` (issue
                #968 §8 / §11.2). When provided, every tool call
                emitted by the LLM is first classified: ``require``
                segments enter ``AWAITING_CONFIRMATION`` and the
                LLM receives an ``"awaiting_user_confirmation"``
                tool result; ``pass_through`` / ``notify`` segments
                are executed as before. When ``None`` the core runs
                the legacy un-gated path — backward-compatible with
                every existing test that doesn't construct a gate.
        """
        if llm is None:
            raise TypeError("DialogCore: llm is required")
        if tools is None:
            raise TypeError("DialogCore: tools is required")
        if memory is None:
            raise TypeError("DialogCore: memory is required")
        if dsm is None:
            raise TypeError("DialogCore: dsm is required")
        self._llm = llm
        self._tools = tools
        self._memory = memory
        self._dsm = dsm
        self._user_id = user_id
        self._system_prompt = system_prompt
        self._history_trim_limit = history_trim_limit
        self._inactivity_timeout = inactivity_timeout
        self._acceptance_gate = acceptance_gate

    # ---- main entry point -----------------------------------------------

    async def process_input(
        self,
        text: str,
        *,
        history: Iterable[LLMMessage] | None = None,
        is_dj_auto: bool = False,
    ) -> DialogResult:
        """Process a single user turn.

        Returns a :class:`DialogResult` describing the assistant's
        reply, the new DSM state, and any error. Never raises — LLM
        exceptions are wrapped in ``result.error`` so the shell can
        log them without aborting the conversation loop.

        ``is_dj_auto`` (issue #992) — when True the input is treated
        as an autonomous DJ auto-transition: the wake-word classifier
        is bypassed and the DSM is force-transitioned to DIALOGUE so
        the LLM actually runs. Without this the DJ prompt
        ``"[DJ_AUTO] ... Роббокс ..."`` matches the "роббокс" wake
        trigger, ``on_user_input`` returns ``WAKE_WORD`` (instead of
        ``STT_RESULT``), the DSM transition is a no-op on LISTENING,
        and ``process_input`` silently skips the LLM call — which is
        exactly the production symptom in issue #992 ("DJ cycle never
        produces music, robot says 'задумался'").

        History trimming: if the caller passes ``history=None`` AND
        ``history_trim_limit`` was set at construction, the memory
        port's ``load_recent`` is queried for the trimmed window.
        The shell therefore never has to slice the history itself —
        it just decides whether to pass an explicit list or trust
        the memory port.

        Steps:
        1. Classify the input via ``dsm.on_user_input``.
        2. Drive the DSM via ``dsm.on_event``.
        3. If we're in ``DIALOGUE`` state (i.e. the input was real
           speech), persist the user turn, call the LLM **with the
           tool schemas discovered from** ``self._tools``, persist
           the assistant turn, and run any follow-up tool loop the
           LLM triggers until either the model returns a plain-text
           reply or ``_MAX_TOOL_ITERATIONS`` is reached.
        4. Return the result.

        Tool-loop contract
        ------------------

        We pass ``tools.discover()`` (translated to the OpenAI
        Chat-Completions ``{"type": "function", ...}`` shape) on
        every ``complete()`` call. When the LLM responds with
        ``LLMResponse.tool_calls``:

        * Each :class:`ToolCall` is executed via
          ``self._tools.execute(call)``.
        * The :class:`ToolResult` (success or function-level error)
          is fed back as an ``LLMMessage(role="tool", ...)`` and
          ``complete()`` is re-issued.
        * ``result.tools_called`` accumulates the *unique* names of
          tools that actually ran.

        The loop is capped at ``_MAX_TOOL_ITERATIONS`` to keep a
        runaway tool from pinning the dialogue thread. Tool
        transport failures (raised :class:`ToolExecutionError`) are
        wrapped into ``result.error`` the same way LLM errors are.
        """
        result = DialogResult()

        # 1. classify — DJ auto-transitions bypass the wake-word
        #    classifier because their prompt intentionally mentions
        #    "роббокс" / "диджей" (the persona), which would otherwise
        #    short-circuit into a WAKE_WORD event and skip the LLM
        #    call entirely (issue #992 root cause).
        if is_dj_auto:
            event = DialogueEvent.STT_RESULT
        else:
            event = self._dsm.on_user_input(text)

        # 2. transition
        self._dsm.on_event(event)
        # DJ auto-turns: if DSM is LISTENING (no wake word yet) but the
        # shell already drove the cycle, force-transition into DIALOGUE
        # so the LLM gate fires. ``STT_RESULT`` from LISTENING
        # normally does this — but a previous turn may have left the
        # DSM in IDLE (e.g. silence timeout between transitions).
        if is_dj_auto and self._dsm.current_state == DialogueStateKind.IDLE:
            self._dsm.on_event(DialogueEvent.WAKE_WORD)
            self._dsm.on_event(DialogueEvent.STT_RESULT)
        result.new_state = self._dsm.current_state

        # 3. if we're now in DIALOGUE state, run the LLM
        if (
            self._dsm.current_state == DialogueStateKind.DIALOGUE
            and event == DialogueEvent.STT_RESULT
        ):
            from rob_box_harness.memory import Turn
            try:
                # Resolve the trimmed history BEFORE we append the
                # new turn — otherwise ``load_recent`` would echo
                # the just-stored user message back into the prompt.
                messages = await self._resolve_history(history)
                messages.append(LLMMessage(role="user", content=text))
                # 🔴 FIX (live 11:19 DJ): DJ-переходы (is_dj_auto=True) НЕ
                # пишутся в долгую память — иначе каждый переход (#1..#N)
                # копит user+assistant пары в SQLite, history_max_turns
                # (20 пар) заполняется DJ-промптами, LLM тонет в 20+
                # одинаковых «[DJ_AUTO переход #N]» + [CRITICAL] ретраях
                # → пустые ответы → «Что-то я задумался». Системный
                # DJ-триггер — не реплика юзера, он не должен загрязнять
                # контекст диалога.
                if not is_dj_auto:
                    await self._memory.append_turn(
                        self._user_id, Turn(role="user", content=text)
                    )
                spoken, tools_called, finish_reason, raw_response = (
                    await self._run_with_tools(messages)
                )
                result.spoken_text = spoken
                result.tools_called = list(tools_called)
                result.finish_reason = finish_reason
                result.raw_response = raw_response
                if not is_dj_auto:
                    await self._memory.append_turn(
                        self._user_id, Turn(role="assistant", content=spoken)
                    )
            except Exception as exc:  # noqa: BLE001 — wrap into result
                result.error = exc
                # The user turn was already appended BEFORE the LLM call
                # (line above). On error, do NOT append again — that
                # would produce a duplicate row in the conversation
                # history. SQLiteVoiceMemory also has its own 5-second
                # dedup window as a safety net.
            # End-of-dialogue: drive the state machine back to IDLE.
            self._dsm.on_event(DialogueEvent.DIALOGUE_END)
            result.new_state = self._dsm.current_state

        return result

    async def _run_with_tools(
        self,
        messages: list[LLMMessage],
    ) -> tuple[str, list[str]]:
        """Run the LLM tool loop and return ``(spoken_text, tools_called)``.

        ``messages`` is the live message list — tool-result messages
        are appended in-place so the LLM sees a coherent conversation
        history on every follow-up turn. The list is intentionally
        mutated rather than rebuilt: the upstream providers expect
        an ordered list and rebuilding from scratch would lose the
        interleaved tool/assistant ordering the wire format requires.

        The loop terminates when:

        * The model returns an empty ``tool_calls`` tuple (i.e. a
          plain-text reply) — that's the final answer.
        * ``_MAX_TOOL_ITERATIONS`` is reached — the last spoken
          text is used verbatim, and a warning is logged so
          operators can spot runaway loops in the wild.
        * The provider raises — the exception propagates to the
          ``except`` in :meth:`process_input`.

        Tool-level errors (i.e. the handler returned ``is_error=True``)
        are NOT loop-terminating: the LLM gets the error string as
        a ``tool`` message and can correct itself. Only a
        :class:`ToolExecutionError` (transport-level) aborts the
        turn because that's a wiring problem, not a tool problem.
        """
        tool_schemas = await self._tools.discover()
        openai_tools = [_tool_spec_to_openai(spec) for spec in tool_schemas]

        tools_called: list[str] = []
        seen: set[str] = set()
        response: LLMResponse = await self._llm.complete(
            messages, tools=openai_tools
        )

        for _ in range(_MAX_TOOL_ITERATIONS):
            if not response.tool_calls:
                break

            # Record unique tool names actually invoked.
            for call in response.tool_calls:
                if call.name not in seen:
                    seen.add(call.name)
                    tools_called.append(call.name)

            # Append the assistant turn that contained the tool_calls
            # (required by OpenAI Chat-Completions ordering rules).
            messages.append(
                LLMMessage(
                    role="assistant",
                    content=response.content,
                    tool_calls=response.tool_calls,
                )
            )

            # Execute every requested tool call and feed results back.
            # When an AcceptanceGate is wired in, require-classified calls
            # enter AWAITING_CONFIRMATION and the LLM receives a sentinel
            # tool result instead of an executor result — the LLM cycle
            # is NOT blocked (§4.4: MERGE/QUEUE/AWAITING don't cancel the
            # LLM cycle), and the user gets to confirm/reject before the
            # call ever reaches the executor.
            for call in response.tool_calls:
                if self._acceptance_gate is not None:
                    segment, decision = self._acceptance_gate.submit(
                        tool=call.name,
                        args=dict(call.arguments or {}),
                        call_id=call.id,
                    )
                    if decision.kind is ConfirmationKind.REQUIRE:
                        # Don't call the executor — gate will dispatch it
                        # later via the future scheduler (Фаза 2).
                        tool_result = ToolResult(
                            tool_call_id=call.id,
                            content=json.dumps(
                                {
                                    "status": "awaiting_user_confirmation",
                                    "segment_id": segment.segment_id,
                                    "tool": call.name,
                                    "plan_text": segment.decision.plan_text,
                                    "confirmation_timeout_ms": int(
                                        self._acceptance_gate.config.confirmation_timeout_ms
                                    ),
                                },
                                ensure_ascii=False,
                            ),
                            is_error=False,
                        )
                        messages.append(
                            LLMMessage(
                                role="tool",
                                content=tool_result.content,
                                tool_call_id=tool_result.tool_call_id,
                                tool_result=tool_result,
                            )
                        )
                        continue

                tool_result = await self._tools.execute(call)
                messages.append(
                    LLMMessage(
                        role="tool",
                        content=tool_result.content,
                        tool_call_id=tool_result.tool_call_id,
                        tool_result=tool_result,
                    )
                )

            response = await self._llm.complete(messages, tools=openai_tools)

        else:  # for-else: loop exhausted without breaking
            logging.getLogger(__name__).warning(
                "DialogCore: tool loop hit _MAX_TOOL_ITERATIONS=%d; "
                "returning the last spoken text as-is.",
                _MAX_TOOL_ITERATIONS,
            )

        return response.content, tools_called, response.finish_reason, response.raw

    async def _resolve_history(
        self,
        history: Iterable[LLMMessage] | None,
    ) -> list[LLMMessage]:
        """Build the LLM message list — either from ``history`` or.
        via ``memory.load_recent``.

        * When the caller passes an explicit iterable, that wins —
          the shell owns the trim there.
        * When ``history`` is ``None`` and ``history_trim_limit`` is
          set, the memory port provides the trimmed window.
        * When ``history`` is ``None`` and no trim limit is set, we
          return an empty list (the shell can fall back to whatever
          it has in mind).
        """
        if history is not None:
            return list(history)
        if self._history_trim_limit is None:
            out: list[LLMMessage] = []
            if self._system_prompt:
                out.append(LLMMessage(role="system", content=self._system_prompt))
            return out
        recent_turns = await self._memory.load_recent(
            self._user_id, limit=self._history_trim_limit
        )
        out: list[LLMMessage] = []
        # 🔴 FIX: system prompt обязателен первым сообщением — раньше
        # dialogue_node грузил _system_prompt, но никогда не передавал
        # его в LLM: deepseek работала БЕЗ системного промпта (все
        # правила «ALWAYS execute_music_code», NO METALANGUAGE и т.д.
        # не доходили до модели). Найдено через verbose-трейс 09:08:
        # в messages не было [0] system.
        if self._system_prompt:
            out.append(LLMMessage(role="system", content=self._system_prompt))
        for turn in recent_turns:
            out.append(LLMMessage(role=turn.role, content=turn.content))
        return out

    # ---- handle_* shortcuts used by the shell --------------------------

    def is_wake_word(self, text: str) -> bool:
        """Return ``True`` if ``text`` looks like a wake-word utterance.

        Convenience for the shell's STT callback — the bool return
        matches the W3 plan §3 signature for ``handle_wake_word``
        even though the facade-level ``handle_wake_word`` below
        still returns a full :class:`DialogResult` for callers
        that want the new state too.
        """
        return self._dsm.on_user_input(text) == DialogueEvent.WAKE_WORD

    async def handle_silence(self) -> DialogResult:
        """Apply a ``SILENCE_COMMAND`` event to the DSM.

        The DSM now tracks both the silence deadline and the
        activity timestamp; this method just drives the
        ``SILENCE_COMMAND`` event and reports the resulting state.
        """
        self._dsm.on_event(DialogueEvent.SILENCE_COMMAND)
        return DialogResult(new_state=self._dsm.current_state)

    async def handle_wake_word(self, text: str) -> DialogResult:
        """Apply a ``WAKE_WORD`` event to the DSM.

        The text parameter is the raw STT transcript that triggered
        the wake word — useful for callers that want to log it.
        Returns a :class:`DialogResult` reporting the new state.

        Callers that only need the bool from the W3 plan §3
        signature should use :meth:`is_wake_word` instead.
        """
        del text  # currently unused; reserved for logging
        self._dsm.on_event(DialogueEvent.WAKE_WORD)
        return DialogResult(new_state=self._dsm.current_state)

    def check_timeout(self) -> bool:
        """Drive a ``TIMEOUT`` event into the DSM if appropriate.

        Two paths:

        * If ``inactivity_timeout`` was set at construction,
          forward to :meth:`DialogueStateMachine.check_inactivity_timeout`
          so ``LISTENING → IDLE`` after the configured silence.
        * Otherwise, fall back to the legacy ``TIMEOUT`` event
          — keeps the original behaviour for callers that don't
          opt in to the inactivity-tracking API.

        Returns:
            ``True`` if the state changed (i.e. a timeout fired).
        """
        if self._inactivity_timeout is not None:
            return self._dsm.check_inactivity_timeout(
                self._inactivity_timeout,
            )
        before = self._dsm.current_state
        self._dsm.on_event(DialogueEvent.TIMEOUT)
        return self._dsm.current_state != before


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _tool_spec_to_openai(spec: ToolSpec) -> dict[str, Any]:
    """Translate a :class:`ToolSpec` into the OpenAI Chat-Completions shape.

    The upstream DeepSeek / OpenAI provider expects a list of
    ``{"type": "function", "function": {"name", "description", "parameters"}}``
    dicts. We pass :class:`ToolSpec.parameters` through unchanged — it
    is already a JSON Schema fragment by contract.

    Specs without a description or schema still serialise cleanly:
    the fields are optional in the wire format.
    """
    return {
        "type": "function",
        "function": {
            "name": spec.name,
            "description": spec.description,
            "parameters": dict(spec.parameters),
        },
    }


__all__ = ["DialogCore", "DialogResult"]
