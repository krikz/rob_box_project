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

from dataclasses import dataclass, field
from typing import Any, Iterable

from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateKind,
    DialogueStateMachine,
)
from rob_box_harness.memory import MemoryStore
from rob_box_harness.tools import ToolProvider
from rob_box_llm.provider import LLMMessage, LLMProvider, LLMResponse


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
        Names of tools the LLM invoked (currently empty — the actual
        tool loop is run by the upstream LLM provider, not by
        DialogCore; reserved for future expansion).
    error:
        ``None`` on success, otherwise the exception that aborted the
        turn. The shell logs this but does not raise it further.
    """

    spoken_text: str = ""
    new_state: DialogueStateKind = DialogueStateKind.IDLE
    tools_called: list[str] = field(default_factory=list)
    error: BaseException | None = None


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
    ) -> None:
        """Compose the four dialogue ports into a single facade.

        Args:
            llm: LLM client — required.
            tools: Tool dispatch port — required (kept for symmetry
                with the W3 plan even though the actual tool loop is
                run by the upstream LLM provider for now).
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
        self._history_trim_limit = history_trim_limit
        self._inactivity_timeout = inactivity_timeout

    # ---- main entry point -----------------------------------------------

    async def process_input(
        self,
        text: str,
        *,
        history: Iterable[LLMMessage] | None = None,
    ) -> DialogResult:
        """Process a single user turn.

        Returns a :class:`DialogResult` describing the assistant's
        reply, the new DSM state, and any error. Never raises — LLM
        exceptions are wrapped in ``result.error`` so the shell can
        log them without aborting the conversation loop.

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
           speech), persist the user turn, call the LLM, persist the
           assistant turn.
        4. Return the result.
        """
        result = DialogResult()

        # 1. classify
        event = self._dsm.on_user_input(text)

        # 2. transition
        self._dsm.on_event(event)
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
                await self._memory.append_turn(
                    self._user_id, Turn(role="user", content=text)
                )
                response = await self._llm.complete(messages)
                spoken = _extract_text(response)
                result.spoken_text = spoken
                await self._memory.append_turn(
                    self._user_id, Turn(role="assistant", content=spoken)
                )
            except Exception as exc:  # noqa: BLE001 — wrap into result
                result.error = exc
                # Still persist the user turn so the conversation
                # history isn't lost when the LLM errors.
                # ``append_turn`` is documented idempotent (SQLiteVoiceMemory
                # enforces a 5-second dedup window; InMemoryStore is append-only
                # but the user turn was NOT appended yet on this path because
                # the append above happens after the LLM call).
                await self._memory.append_turn(
                    self._user_id, Turn(role="user", content=text)
                )
            # End-of-dialogue: drive the state machine back to IDLE.
            self._dsm.on_event(DialogueEvent.DIALOGUE_END)
            result.new_state = self._dsm.current_state

        return result

    async def _resolve_history(
        self,
        history: Iterable[LLMMessage] | None,
    ) -> list[LLMMessage]:
        """Build the LLM message list — either from ``history`` or
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
            return []
        recent_turns = await self._memory.load_recent(
            self._user_id, limit=self._history_trim_limit
        )
        out: list[LLMMessage] = []
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


def _extract_text(response: Any) -> str:
    """Best-effort extraction of the assistant's reply text.

    Different ``LLMProvider`` implementations may return either an
    ``LLMResponse`` dataclass, an OpenAI-style dict, or a Pydantic
    model. We handle the common cases so the shell can stay thin.
    """
    # 1. dataclass / object with `.content`
    content = getattr(response, "content", None)
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        # OpenAI-style content parts — concatenate text parts.
        parts: list[str] = []
        for part in content:
            if isinstance(part, dict):
                text = part.get("text")
                if isinstance(text, str):
                    parts.append(text)
            elif isinstance(part, str):
                parts.append(part)
        if parts:
            return "".join(parts)

    # 2. dict-like
    if isinstance(response, dict):
        choices = response.get("choices") or []
        if choices:
            message = choices[0].get("message") or {}
            text = message.get("content")
            if isinstance(text, str):
                return text
            if isinstance(text, list):
                parts = []
                for part in text:
                    if isinstance(part, dict) and isinstance(part.get("text"), str):
                        parts.append(part["text"])
                    elif isinstance(part, str):
                        parts.append(part)
                if parts:
                    return "".join(parts)

    # 3. fallback: stringify
    return str(response)


__all__ = ["DialogCore", "DialogResult"]