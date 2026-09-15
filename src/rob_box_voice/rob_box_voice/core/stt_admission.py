"""stt_admission.py — admission pipeline for STT text (issue #2628 / ADR-0021 R1).

Pure helpers + ``SttAdmission`` orchestrator extracted from
:meth:`DialogueNode._on_stt` so each barrier lives in ``core/`` and is
unit-testable without ROS2.

The original ``_on_stt`` was a 12-branch monolith (CC=54) that mixed:

* pure text parsing (Telegram prefix, speaker-tag event);
* policy decisions (rejected / silence / wake-word / backlog / barge-in);
* ROS-bound side effects (cancel_run, dispatch_turn, DSM transition).

This module owns the **pure** half: helpers and orchestrator + per-step
classes that call back into a small :class:`SttAdmissionHost` protocol.
The dialogue node adapter wires the host callbacks to its real ROS
surfaces (:meth:`_cancel_run`, :meth:`_dispatch_turn`, DSM transition, ...).

Why the split matters:

* The orchestrator + helpers are 100% unit-testable in plain pytest —
  no rclpy import, no logger fixture, no ROS spin.
* The order + retry budget + "first verdict wins" rule is owned by
  ``SttAdmission.evaluate`` — one place to read, one place to change.
* A future regression ("robot ignored phrase X") logs the *step name*
  that fired the DROP/HANDLED, so the operator can grep one line
  instead of guessing which of the 12 inline ``return`` statements
  closed the call.

See ``docs/adr/0021-cc-budget.md`` (R1) for the threshold (CC<=15) and
``docs/adr/0080-dialogue-fsm.md`` §2.4 for the orchestrator pattern
mirrored from :mod:`rob_box_voice.core.turn`.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import Enum
from typing import (
    Any,
    Dict,
    List,
    Optional,
    Protocol,
    Sequence,
    Tuple,
    runtime_checkable,
)

from rob_box_voice.core.dialogue_text import (
    has_wake_word,
    is_silence_command,
    is_unsilence_command,
    strip_wake_word,
)


# ---------------------------------------------------------------------------
# Pure helpers — text/dict parsing, no ROS
# ---------------------------------------------------------------------------


def parse_tg_prefix(text: str) -> Tuple[str, Optional[int]]:
    """Strip ``[TG:<chat_id>]`` prefix from STT text and return the chat id.

    Issue #1195 / ADR-0066 §6.3 — telegram_node prepends ``[TG:<id>]`` to
    every text it forwards. The chat id is later used for the echo path
    (see ``DialogueNode._active_tg_chat_id``). When the marker is absent
    or malformed, the original text is returned unchanged and the chat
    id is ``None``.

    Pure: no logging, no side effects, no exceptions raised on bad
    input. ``TypeError`` (e.g. ``None`` passed in) returns ``(str(text), None)``
    so the caller doesn't need to wrap it in a ``try/except``.

    Args:
        text: Raw ``/voice/stt/result`` payload string.

    Returns:
        ``(cleaned_text, tg_chat_id)`` — ``cleaned_text`` is the original
        minus the ``[TG:...]`` prefix (already ``.strip()``-ed); when the
        prefix is absent the original ``text`` is returned with its
        outer whitespace stripped.

    Examples:
        >>> parse_tg_prefix("[TG:42] привет")
        ('привет', 42)
        >>> parse_tg_prefix("просто текст")
        ('просто текст', None)
        >>> parse_tg_prefix("[TG:not_a_number] привет")
        ('[TG:not_a_number] привет', None)
        >>> parse_tg_prefix("")
        ('', None)
    """
    if not isinstance(text, str):
        try:
            text = str(text or "")
        except (TypeError, ValueError):
            return "", None
    if not text:
        return "", None
    if not text.startswith("[TG:"):
        return text.strip(), None
    marker_end = text.find("]")
    if marker_end == -1:
        return text.strip(), None
    raw = text[4:marker_end].strip()
    try:
        chat_id = int(raw)
    except (TypeError, ValueError):
        return text.strip(), None
    cleaned = text[marker_end + 1:].strip()
    return cleaned, chat_id


def parse_speaker_event(
    speaker_event: Optional[Dict[str, Any]],
) -> Tuple[Optional[str], float]:
    """Pull ``speaker_tag`` / ``duration_s`` out of a stt_node speaker event.

    Issue #2628 — replaces the inline ``try/except`` block at the top of
    the old ``_on_stt``. ``speaker_event`` is a dict that stt_node
    publishes via :data:`/voice/speaker/by_text`; ``None`` means "no
    speaker info for this turn".

    Pure: no logging, no side effects. Missing keys fall back to
    ``(None, 0.0)`` — matches the legacy behaviour byte-for-byte.

    Args:
        speaker_event: ``dict`` from stt_node, or ``None``.

    Returns:
        ``(speaker_tag, speaker_duration_s)`` — ``speaker_tag`` is
        ``str``-coerced and empty-string-normalised to ``None`` so the
        downstream consumer can use ``speaker_tag or "anonymous"``
        without extra branches.
    """
    if not speaker_event:
        return None, 0.0
    if not isinstance(speaker_event, dict):
        return None, 0.0
    speaker_tag = str(speaker_event.get("speaker_tag") or "")
    try:
        duration = float(speaker_event.get("duration_s") or 0.0)
    except (TypeError, ValueError):
        duration = 0.0
    return (speaker_tag or None), duration


# ---------------------------------------------------------------------------
# Value types — what the steps see and what they return
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SttContext:
    """Inputs to the STT admission pipeline (post parsing).

    Attributes:
        raw: Original STT text (before any normalisation).
        text: Current text — mutated by ``apply`` calls that strip
            a wake-word, a Telegram prefix, etc.
        text_lower: ``text.lower()`` — pre-computed so steps don't
            recompute the lowercased copy on every call.
        tg_chat_id: Telegram chat id (set by :func:`parse_tg_prefix`),
            or ``None`` for mic input.
        speaker_tag: Active speaker tag, or ``None``.
        speaker_duration_s: Duration of the speaker segment, in seconds.
        state_name: ``DialogueStateKind.name`` — string snapshot to
            avoid leaking the FSM enum into ``core/``.
        is_silenced: Convenience flag — ``True`` when state==SILENCED.
        wake_words: Frozen tuple of wake-word strings (snapshot).
        backlog_pending: ``True`` when the no-wake-word backlog
            accumulator has buffered phrases to inject as a hint.
            Computed **once** before admission (the caller owns the
            accumulator); :class:`BacklogFlushStep` sets the matching
            flag on the host, :class:`StripWakeWordStep` reads the
            snapshot to decide whether a bare wake-word preserves the
            original text instead of dropping with ``empty_after_strip``.
        skip_counter: Mutable dict of skip-reason counters (e.g.
            ``{"no_wake_word": 3}``). Steps increment the counter that
            matches the reason they drop on; orchestrator does not
            touch it.
    """

    raw: str
    text: str
    text_lower: str
    tg_chat_id: Optional[int] = None
    speaker_tag: Optional[str] = None
    speaker_duration_s: float = 0.0
    state_name: str = "IDLE"
    is_silenced: bool = False
    wake_words: Tuple[str, ...] = ()
    backlog_pending: bool = False
    skip_counter: Dict[str, int] = field(default_factory=dict)

    def with_text(self, new_text: str) -> "SttContext":
        """Return a fresh context with ``text`` replaced and ``text_lower``
        recomputed. Pure — never mutates the input.
        """
        return replace(
            self,
            text=new_text,
            text_lower=new_text.lower(),
        )


class SttOutcomeKind(str, Enum):
    """Verdict an :class:`SttStep` may return.

    * ``DROP(reason)`` — pipeline stops, the STT text did NOT reach LLM.
      The orchestrator increments the matching counter and logs the
      step name. Use for "this text is not for us" (rejected / no wake
      word / empty after strip / silence command / backlog hold /
      new-session reset / quick_decide IGNORE).
    * ``HANDLED(reason)`` — pipeline stops, but the step ALREADY did
      the side effect (silence FSM transition, reset session, dispatch
      a follow-up). The orchestrator must NOT re-run the side effect.
      Use for state-machine transitions that already speak a line.
    * ``PASS`` — pipeline continues with the (possibly mutated)
      context. Most steps return ``PASS``; the orchestrator walks the
      list until a DROP/HANDLED/PASS-at-end verdict is reached.
    """

    DROP = "drop"
    HANDLED = "handled"
    PASS = "pass"


@dataclass(frozen=True)
class SttOutcome:
    """Result of one :meth:`SttStep.apply` (or the orchestrator).

    Attributes:
        kind: Action the dialogue_node adapter must take.
        step_name: Which step decided. ``""`` for ``PASS`` (no step
            raised).
        reason: Short human-readable tag for the log line. Also
            doubles as the key in ``SttContext.skip_counter`` when
            ``kind`` is ``DROP`` — the orchestrator increments the
            counter automatically.
        new_context: For ``PASS`` — the (possibly mutated) context to
            feed into the next step. ``None`` for ``DROP`` / ``HANDLED``.
    """

    kind: SttOutcomeKind
    step_name: str = ""
    reason: str = ""
    new_context: Optional[SttContext] = None


#: Convenience constructor for "nothing raised".
PASS: SttOutcome = SttOutcome(kind=SttOutcomeKind.PASS)


def drop(step_name: str, reason: str) -> SttOutcome:
    """Build a ``DROP`` outcome with the standard shape."""
    return SttOutcome(kind=SttOutcomeKind.DROP, step_name=step_name, reason=reason)


def handled(step_name: str, reason: str) -> SttOutcome:
    """Build a ``HANDLED`` outcome with the standard shape."""
    return SttOutcome(
        kind=SttOutcomeKind.HANDLED,
        step_name=step_name,
        reason=reason,
    )


# ---------------------------------------------------------------------------
# Host — what the steps call back into
# ---------------------------------------------------------------------------


@runtime_checkable
class SttAdmissionHost(Protocol):
    """Side-effect surface the steps use.

    The dialogue node adapter implements these callbacks against its
    real ROS surfaces (locks, counters, dispatch_turn). ``core/`` is
    free of rclpy imports — this Protocol is the seam.

    All methods are best-effort: a ``None`` return is treated as
    "feature disabled" and the step degrades to ``PASS`` (e.g. when
    the backlog accumulator is not configured).
    """

    def unsilence(self, text_lower: str) -> bool:
        """Issue #1101 — if ``text_lower`` matches an unsilence phrase,
        transition out of SILENCED and return ``True``. Otherwise return
        ``False`` so the step can DROP the text with the
        ``silenced`` reason.
        """
        ...

    def accumulate_without_wake(
        self,
        speaker_tag: Optional[str],
        text: str,
    ) -> bool:
        """Push the text into the no-wake backlog accumulator. Returns
        ``True`` when the accumulator accepted the text (i.e. backlog
        is enabled and the feature is configured), ``False`` when the
        feature is off — the caller drops with ``no_wake_word`` reason.
        """
        ...

    def handle_silence_command(self) -> bool:
        """Run the «замолчи» command path. The dialogue node adapter
        publishes the «Хорошо, молчу.» line, mutates the FSM, and
        cancels the in-flight turn. Returns ``True`` so the step can
        return ``HANDLED`` (not DROP — the side effect already ran).
        """
        ...

    def is_music_stop_command(self, text_lower: str) -> bool:
        """Issue #1279 — ``True`` when the silence-shaped phrase is
        actually a music-stop request («хватит диджеить», «стоп
        музыку»). The adapter wraps
        :func:`rob_box_voice.core.dialogue_text.is_music_stop_command`
        so the rule lives next to the silence matcher and the
        :data:`MUSIC_STOP_OVERRIDES` list. ``SilenceCommandStep``
        consults this to decide whether to drop (genuine silence)
        or fall through to LLM (music-stop).
        """
        ...

    def handle_command_intent(self, text: str, text_lower: str) -> bool:
        """Issue #1279 — when ``command_parser`` recognises a non-LLM
        intent (NAVIGATE/STOP/STATUS/MAP/...), run it via command_node
        and cancel the LLM turn. Returns ``True`` when handled. The
        adapter owns the ``_command_intent_gate_enabled`` flag and the
        ``_command_parser.parse(...)`` call; ``text_lower`` is needed
        to honour :data:`MUSIC_STOP_OVERRIDES` (music-stop phrases must
        reach LLM, not the command gate).
        """
        ...

    def reset_session(
        self,
        text: str,
        text_lower: str,
        tg_chat_id: Optional[int],
    ) -> bool:
        """Issue #XXXX — «новая сессия» / «сбрось всё» / TG «/clear».
        The adapter wraps :meth:`DialogueNode._is_new_session_command`
        (which depends on ``_new_session_phrases`` — not a core/
        concern). Returns ``True`` when reset ran.
        """
        ...

    def flush_pending_backlog(self) -> None:
        """Set ``_pending_backlog_flush = True`` so the next dispatch
        prepends the backlog hint. No-op when the backlog is empty.
        """
        ...

    def quick_decide_verdict(self, clean: str) -> Tuple[str, bool, bool]:
        """Run ``quick_decide`` against the cleaned text and return
        ``(verdict_value, ignored, pending_llm)``.

        ``ignored=True`` means the verdict was ``IGNORE`` — drop the
        text entirely. ``pending_llm=True`` means ``PENDING_LLM`` —
        queue the text into ``_pending_user_messages`` and stop. The
        orchestrator then calls back into :meth:`enqueue_pending` or
        :meth:`cancel_inflight` depending on the verdict.
        """
        ...

    def enqueue_pending(self, clean: str) -> bool:
        """Append ``(clean, time.monotonic())`` to ``_pending_user_messages``,
        dropping the oldest entry if the queue is at the limit. Returns
        ``True`` when the text was queued (orchestrator should DROP).
        """
        ...

    def cancel_inflight(self, stop_tts: bool) -> None:
        """Cancel the in-flight LLM turn (and optionally stop TTS) for
        a fresh input. Mirrors the legacy ``_cancel_run(reason, stop_tts=...)``
        path.
        """
        ...

    def transition_idle_to_wake(self) -> bool:
        """Wake-word path: IDLE → LISTENING via ``DialogueEvent.WAKE_WORD``.
        Returns ``True`` when the transition fired (orchestrator proceeds
        to ``STT_RESULT`` regardless — see the legacy code).
        """
        ...

    def transition_stt_result(self) -> None:
        """LISTENING → DIALOGUE via ``DialogueEvent.STT_RESULT``."""
        ...

    def publish_state(self) -> None:
        """Push the new FSM state to ``/dialogue/state``."""
        ...

    def trigger_thinking_sfx(self) -> None:
        """Publish the «thinking» cue to ``/sound/trigger``."""
        ...


# ---------------------------------------------------------------------------
# Step protocol + orchestrator
# ---------------------------------------------------------------------------


@runtime_checkable
class SttStep(Protocol):
    """One barrier in the admission pipeline.

    Each step takes the :class:`SttContext`, decides whether the text
    should pass / drop / be handled, and returns the verdict. The
    orchestrator owns the order + the "first non-PASS wins" rule.

    Implementations MUST be pure — no rclpy import, no global state
    mutation. Side effects go through the :class:`SttAdmissionHost`
    callbacks.

    Note: ``name`` is read off the instance (``step.name``) but is NOT
    a protocol member — Python's structural typing doesn't model class
    attributes as protocol members when implementations are
    ``@dataclass(frozen=True)`` (mutable ``name`` attribute against an
    immutable protocol member). The orchestrator reads ``name`` via
    :func:`getattr` defensively and falls back to the class name.
    """

    def apply(
        self,
        ctx: SttContext,
        host: SttAdmissionHost,
    ) -> SttOutcome:
        ...


@dataclass
class SttAdmission:
    """Owns the step order + the "first verdict wins" rule.

    Construction takes the ordered list of steps. ``evaluate`` walks the
    list, returns the FIRST non-``PASS`` verdict, and threads the
    (possibly mutated) context between steps. Mirrors the orchestrator
    pattern from :class:`rob_box_voice.core.turn.TurnGuards` (issue
    #2241 / ADR-0080 §2.4).

    For ``DROP`` verdicts the orchestrator auto-increments
    ``ctx.skip_counter[reason]`` so the dialogue node adapter does not
    need to repeat the bookkeeping 12 times.

    Args:
        steps: Ordered sequence. Iteration order = priority. The first
            step that returns ``DROP`` or ``HANDLED`` wins; the rest
            are not consulted for this turn.
        logger: Optional logger for the diagnostic lines the
            orchestrator emits on DROP / HANDLED. ``None`` silences
            the orchestrator — handy for unit tests.
    """

    steps: Sequence[SttStep]
    logger: Optional[Any] = field(default=None, repr=False)

    def __post_init__(self) -> None:
        if not self.steps:
            raise ValueError(
                "SttAdmission: steps must be a non-empty sequence "
                "(got empty — an empty order means 'always pass')."
            )

    def evaluate(
        self,
        ctx: SttContext,
        host: SttAdmissionHost,
    ) -> SttOutcome:
        """Run the ordered steps; return the first non-PASS verdict.

        Returns :data:`PASS` if every step defers. The final
        ``new_context`` from the last PASS wins (so the dialogue node
        adapter sees the fully-mutated text).
        """
        current = ctx
        for step in self.steps:
            outcome = step.apply(current, host)
            if outcome.kind is SttOutcomeKind.PASS:
                if outcome.new_context is not None:
                    current = outcome.new_context
                continue
            # DROP / HANDLED — auto-increment counter + log
            if outcome.kind is SttOutcomeKind.DROP:
                key = outcome.reason or outcome.step_name
                ctx.skip_counter[key] = ctx.skip_counter.get(key, 0) + 1
            label = f"{outcome.step_name} ({outcome.reason})" if outcome.reason else outcome.step_name
            if self.logger is not None:
                log_fn = self.logger.warning if outcome.kind is SttOutcomeKind.HANDLED else self.logger.info
                log_fn(
                    f"🚦 [stt-admission] {outcome.kind.value}: "
                    f"step={label} text={ctx.text[:60]!r}"
                )
            return outcome
        return SttOutcome(
            kind=SttOutcomeKind.PASS,
            new_context=current,
        )


# ---------------------------------------------------------------------------
# Default steps — one per legacy barrier
# ---------------------------------------------------------------------------


#: Default ``barge_in_policy`` value (matches the legacy
#: ``getattr(self, "_barge_in_policy", "replace")`` default).
DEFAULT_BARGE_IN_POLICY: str = "replace"


@dataclass(frozen=True)
class EmptyTextStep:
    """Step 1 — empty STT text. Drops with ``empty_text``."""

    name: str = "empty_text"

    def apply(self, ctx: SttContext, host: SttAdmissionHost) -> SttOutcome:
        if not ctx.text:
            return drop(self.name, "empty_text")
        return PASS


@dataclass(frozen=True)
class RejectedMarkerStep:
    """Step 4 — STT echoed a ``rejected`` / ``empty`` / «тишина» marker.

    Issue #989 — ``rejected`` and friends are produced by stt_node when
    it threw away the audio (empty buffer, VAD timeout). They must NOT
    reach LLM.
    """

    name: str = "rejected_marker"
    _PREFIXES: Tuple[str, ...] = (
        "rejected", "«rejected", "empty", "«пусто", "тишина",
    )

    def apply(self, ctx: SttContext, host: SttAdmissionHost) -> SttOutcome:
        if ctx.text_lower.startswith(self._PREFIXES):
            return drop(self.name, "stt_rejected")
        return PASS


@dataclass(frozen=True)
class SilencedStateStep:
    """Step 5 — DSM in SILENCED state.

    Issue #1101 — when the operator has paused the dialogue, only an
    explicit unsilence command may pass. Everything else is dropped
    with ``silenced`` reason. ``host.unsilence(...)`` runs the FSM
    transition if the text matches; we return ``HANDLED`` either way
    so the orchestrator does not increment a counter for the «unsilence»
    path (the FSM already logged it).
    """

    name: str = "silenced_state"

    def apply(self, ctx: SttContext, host: SttAdmissionHost) -> SttOutcome:
        if not ctx.is_silenced:
            return PASS
        if is_unsilence_command(ctx.text_lower):
            host.unsilence(ctx.text_lower)
            return handled(self.name, "unsilence")
        return drop(self.name, "silenced")


@dataclass(frozen=True)
class WakeWordStep:
    """Step 6 — wake-word gate (mic path only; TG bypasses).

    ADR-0066 §6.3 — Telegram input (``tg_chat_id is not None``)
    bypasses the gate: the user is already in a chat, no wake word
    needed. Mic input without a wake word either accumulates into the
    backlog (``host.accumulate_without_wake`` returns True) or is
    dropped with ``no_wake_word``.
    """

    name: str = "wake_word"

    def apply(self, ctx: SttContext, host: SttAdmissionHost) -> SttOutcome:
        if ctx.tg_chat_id is not None:
            return PASS
        if has_wake_word(ctx.text_lower, list(ctx.wake_words)):
            return PASS
        if host.accumulate_without_wake(ctx.speaker_tag, ctx.text):
            return handled(self.name, "backlog_accumulated")
        return drop(self.name, "no_wake_word")


@dataclass(frozen=True)
class StripWakeWordStep:
    """Step 7 — strip the wake word from the text.

    Issue #989 fix + #2346 — the LLM input must not include the wake
    word itself; the bare wake word with no follow-up drops with
    ``empty_after_strip`` **unless** a backlog hint is pending (in
    which case the bare wake word is preserved as a signal — the
    original ``_on_stt`` behaviour: ``clean = text`` when
    ``backlog_pending``).
    """

    name: str = "strip_wake_word"

    def apply(self, ctx: SttContext, host: SttAdmissionHost) -> SttOutcome:
        # Issue #2628 fixture evidence — TG path also strips the wake
        # word (e.g. ``[TG:42] робот привет`` → ``"привет"``). Legacy
        # ``_on_stt`` ran ``clean = strip_wake_word(text)`` unconditionally
        # after parse_tg_prefix; the TG-bypass was a refactor regression.
        clean = strip_wake_word(ctx.text, list(ctx.wake_words))
        if not clean:
            # Bare wake-word OR no input: drop unless backlog is
            # pending. The caller-owned snapshot on ``ctx.backlog_pending``
            # preserves the legacy byte-for-byte fallback.
            if ctx.backlog_pending:
                return SttOutcome(
                    kind=SttOutcomeKind.PASS,
                    new_context=ctx.with_text(ctx.raw),
                )
            return drop(self.name, "empty_after_strip")
        return SttOutcome(
            kind=SttOutcomeKind.PASS,
            new_context=ctx.with_text(clean),
        )


@dataclass(frozen=True)
class SilenceCommandStep:
    """Step 8 — «замолчи» / «хватит» command.

    Issue #1279 follow-up — «хватит диджеить» is NOT a silence, it's
    a music-stop command that must reach LLM. This step only handles
    the genuine silence phrase; the music-stop detection is delegated
    to ``host.is_music_stop_command(text_lower)`` — the dialogue node
    adapter imports ``is_music_stop_command`` from
    :mod:`rob_box_voice.core.dialogue_text` so the rule stays in core/.
    """

    name: str = "silence_command"

    def apply(self, ctx: SttContext, host: SttAdmissionHost) -> SttOutcome:
        if not is_silence_command(ctx.text_lower):
            return PASS
        if host.is_music_stop_command(ctx.text_lower):
            # Music-stop override — phrase must reach LLM.
            return PASS
        if host.handle_silence_command():
            return handled(self.name, "silence_command")
        return PASS


@dataclass(frozen=True)
class CommandIntentGateStep:
    """Step 9 — command-intent gate (issue #1279).

    When ``command_parser`` recognises a non-LLM intent (NAVIGATE /
    STOP / STATUS / MAP / ...), route it to ``command_node`` and skip
    LLM. ``host.handle_command_intent(text)`` is the seam — the
    dialogue node adapter owns the ``_command_intent_gate_enabled``
    flag and the ``_command_parser.parse(...)`` call.
    """

    name: str = "command_intent_gate"

    def apply(self, ctx: SttContext, host: SttAdmissionHost) -> SttOutcome:
        if ctx.tg_chat_id is not None:
            # TG path bypasses the gate — chat input is conversational
            # by default.
            return PASS
        if host.handle_command_intent(ctx.text, ctx.text_lower):
            return handled(self.name, "command_intent")
        return PASS


@dataclass(frozen=True)
class NewSessionStep:
    """Step 10 — «новая сессия» / «сбрось всё» / TG «/clear».

    Issue #XXXX — when the user asks for a fresh session, reset the
    dialogue context and do NOT dispatch to LLM. ``host.reset_session``
    runs ``DialogueNode._reset_dialogue_session``.
    """

    name: str = "new_session"

    def apply(self, ctx: SttContext, host: SttAdmissionHost) -> SttOutcome:
        # The host owns the ``_is_new_session_command(...)`` check
        # (it depends on the ``_new_session_phrases`` parameter, which
        # is not a core/ concern).
        if host.reset_session(ctx.text, ctx.text_lower, ctx.tg_chat_id):
            return handled(self.name, "new_session")
        return PASS


@dataclass(frozen=True)
class BacklogFlushStep:
    """Step 11 — backlog hint injection.

    When the backlog accumulator has buffered phrases (snapshot on
    ``ctx.backlog_pending`` is True), the next user-initiated turn
    prepends them as a hint. ``host.flush_pending_backlog`` sets the
    ``_pending_backlog_flush`` flag; the dialogue node adapter reads
    it inside ``_dispatch_turn``. When ``backlog_pending`` is False
    the step is a no-op — matches legacy L2350-2351 (``if
    backlog_pending: self._pending_backlog_flush = True``).
    """

    name: str = "backlog_flush"

    def apply(self, ctx: SttContext, host: SttAdmissionHost) -> SttOutcome:
        # Issue #1766 — only set the flush flag when the caller-owned
        # snapshot on ``ctx.backlog_pending`` is True. Mirrors the
        # legacy ``if backlog_pending: self._pending_backlog_flush = True``
        # at L2350-2351.
        if ctx.backlog_pending:
            host.flush_pending_backlog()
        return PASS


@dataclass(frozen=True)
class BargeInClassifyStep:
    """Step 12 — barge-in classifier (issue #968 / S4.2).

    When ``barge_in_policy == "classify"``, route the new input through
    :func:`quick_decide`:

    * ``IGNORE`` → DROP with ``quick_decide_ignore``.
    * ``PENDING_LLM`` + live turn → ``host.enqueue_pending(...)``
      then DROP with ``pending_llm`` (the text reaches LLM as a
      follow-up turn, not a fresh dispatch).
    * otherwise (REPLACE / PENDING_LLM with no live turn) → cancel
      the in-flight turn via ``host.cancel_inflight(stop_tts=...)``
      and PASS.
    """

    name: str = "barge_in_classify"
    policy: str = DEFAULT_BARGE_IN_POLICY

    def apply(self, ctx: SttContext, host: SttAdmissionHost) -> SttOutcome:
        if self.policy != "classify":
            # REPLACE / OFF — legacy unconditional cancel. The host
            # owns the cancel call; we just signal PASS.
            host.cancel_inflight(stop_tts=True)
            return PASS
        verdict, ignored, pending_llm = host.quick_decide_verdict(ctx.text)
        if ignored:
            return drop(self.name, "quick_decide_ignore")
        if pending_llm:
            if host.enqueue_pending(ctx.text):
                return handled(self.name, "pending_llm")
            # No slot to queue → fall through to REPLACE-style cancel.
        # REPLACE-style: cancel the in-flight turn (stop_tts depends
        # on the verdict — only REPLACE stops TTS; PENDING_LLM with
        # no queue should not cut an already-playing segment).
        # ``quick_decide_verdict`` returns the uppercase string form
        # (``verdict.value`` — see :class:`QuickVerdict`) — case-fold
        # here so the comparison is unambiguous.
        host.cancel_inflight(stop_tts=(verdict.lower() == "replace"))
        return PASS


@dataclass(frozen=True)
class DispatchTriggerStep:
    """Final step — FSM transition + LLM dispatch.

    Mirrors the tail of the legacy ``_on_stt``:

    * IDLE → WAKE_WORD transition (handled via host).
    * STT_RESULT transition.
    * Publish state.
    * Publish the «thinking» SFX.
    """

    name: str = "dispatch_trigger"

    def apply(self, ctx: SttContext, host: SttAdmissionHost) -> SttOutcome:
        host.transition_idle_to_wake()
        host.transition_stt_result()
        host.publish_state()
        host.trigger_thinking_sfx()
        return PASS


# ---------------------------------------------------------------------------
# Default step list — preserves the legacy order byte-for-byte
# ---------------------------------------------------------------------------


def default_steps(
    *,
    barge_in_policy: str = DEFAULT_BARGE_IN_POLICY,
) -> List[Any]:
    """Return the canonical step order for ``DialogueNode._on_stt``.

    Order matches the inline if-chain at
    ``dialogue_node.py:2159-2445`` (issue #2628 refactor):

    1. :class:`EmptyTextStep` — short-circuit empty payload.
    2. TG prefix parsing — handled **before** admission by the caller
       (``parse_tg_prefix`` is a pure helper, not a step). Skipped here.
    3. speaker event parsing — same: handled by the caller via
       :func:`parse_speaker_event` before admission. Skipped here.
    4. :class:`RejectedMarkerStep` — STT echoed a rejected marker.
    5. :class:`SilencedStateStep` — DSM in SILENCED.
    6. :class:`WakeWordStep` — wake-word gate (mic path).
    7. :class:`StripWakeWordStep` — strip the wake word.
    8. :class:`SilenceCommandStep` — «замолчи».
    9. :class:`CommandIntentGateStep` — command_intent gate (issue #1279).
    10. :class:`NewSessionStep` — «новая сессия» / «/clear».
    11. :class:`BacklogFlushStep` — backlog hint injection.
    12. :class:`BargeInClassifyStep` — barge_in_policy="classify".
    13. :class:`DispatchTriggerStep` — FSM transition + dispatch.
    """
    return [
        EmptyTextStep(),
        RejectedMarkerStep(),
        SilencedStateStep(),
        WakeWordStep(),
        StripWakeWordStep(),
        SilenceCommandStep(),
        CommandIntentGateStep(),
        NewSessionStep(),
        BacklogFlushStep(),
        BargeInClassifyStep(policy=barge_in_policy),
        DispatchTriggerStep(),
    ]


__all__ = [
    "BargeInClassifyStep",
    "BacklogFlushStep",
    "CommandIntentGateStep",
    "DEFAULT_BARGE_IN_POLICY",
    "DefaultSttAdmission",
    "DispatchTriggerStep",
    "EmptyTextStep",
    "NewSessionStep",
    "PASS",
    "RejectedMarkerStep",
    "SilenceCommandStep",
    "SilencedStateStep",
    "SttAdmission",
    "SttAdmissionHost",
    "SttContext",
    "SttOutcome",
    "SttOutcomeKind",
    "SttStep",
    "StripWakeWordStep",
    "WakeWordStep",
    "default_steps",
    "drop",
    "handled",
    "parse_speaker_event",
    "parse_tg_prefix",
]


def DefaultSttAdmission(
    *,
    barge_in_policy: str = DEFAULT_BARGE_IN_POLICY,
    logger: Optional[Any] = None,
) -> SttAdmission:
    """Convenience constructor: the canonical pipeline for ``_on_stt``.

    Equivalent to ``SttAdmission(steps=default_steps(...), logger=...)``.
    Kept as a function (not a dataclass) so the constructor can take a
    default ``logger`` and so callers don't have to import the steps
    list by hand.
    """
    return SttAdmission(
        steps=default_steps(barge_in_policy=barge_in_policy),
        logger=logger,
    )