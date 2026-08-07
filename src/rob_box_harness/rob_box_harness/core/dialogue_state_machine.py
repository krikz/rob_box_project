"""DialogState dataclass and DialogueStateMachine — pure state machine for dialog states.

The state machine tracks voice-dialogue lifecycle:

    IDLE → LISTENING → DIALOGUE → IDLE
    any state → SILENCED → IDLE (after timeout)

It is **pure** — no I/O, no ROS2, no LLM, no async. Immediately
testable with plain pytest. Designed as a composable building block
that :class:`DialogHarness` owns and drives via ``on_event`` /
``on_user_input``.

The existing ``rob_box_voice.core.dialogue_manager.DialogueManager``
is NOT modified — this is a parallel implementation for the harness
framework (ADR-0001 §2.7.1, Phase 06-05 --wave 2).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any


def _now() -> float:
    """Return the current unix timestamp.

    Indirected through a helper so tests can monkey-patch the clock
    in one place. ``DialogueStateMachine`` calls this for every
    timestamp update (silence-start, last-activity, etc.).
    """
    return time.time()


# ---------------------------------------------------------------------------
# State / Event enums
# ---------------------------------------------------------------------------


class DialogueStateKind(Enum):
    """Dialog lifecycle states.

    The order mirrors the voice-assistant flow:
    IDLE (waiting for wake word) → LISTENING (wake word detected)
    → DIALOGUE (processing user speech) → back to IDLE.
    SILENCED is a transient suppression state reachable from any
    other state (user said "тихо"/"молчи").
    """

    IDLE = auto()
    LISTENING = auto()
    DIALOGUE = auto()
    SILENCED = auto()


class DialogueEvent(Enum):
    """Events that drive state transitions.

    These are the ONLY inputs the state machine accepts.
    ``on_user_input`` is the classification function that maps raw
    text to one of these events.
    """

    WAKE_WORD = auto()        # "роббокс" / wake-word detected
    STT_RESULT = auto()        # speech recognized (normal input)
    DIALOGUE_END = auto()      # LLM response complete
    SILENCE_COMMAND = auto()   # "тихо" / "молчи" detected
    TIMEOUT = auto()           # no activity — return to IDLE
    UNSILENCE = auto()         # silenced period expired / explicit resume


# ---------------------------------------------------------------------------
# DialogState
# ---------------------------------------------------------------------------


@dataclass
class DialogState:
    """Runtime state bag for the dialog harness.

    This is the ``StateT`` parameter passed to ``Harness[DialogState]``.
    It holds all the mutable fields that ``DialogHarness.step()``
    reads and writes during a single conversation turn.

    The fields are intentionally flat — the harness can snapshot
    and restore these easily. Do NOT store ROS2 handles, LLM clients,
    or other I/O objects here.
    """

    wake_active: bool = True
    """Whether the wake-word detector is active (True after init)."""

    silenced: bool = False
    """True when the user has asked the robot to stay quiet."""

    silenced_until: float = 0.0
    """Unix timestamp when the silence period expires (0 = not silenced)."""

    dialogue_id: str = ""
    """Unique identifier for the current dialogue session."""

    user_id: str = "default"
    """Identifier for the current user / chat session."""

    last_stt_text: str = ""
    """Most recent STT transcript received."""

    last_response: str = ""
    """Most recent assistant response text."""

    turn_count: int = 0
    """Monotonic counter of dialog turns since init."""

    metadata: dict[str, Any] = field(default_factory=dict)
    """Extension point for harness-specific telemetry or flags."""


# ---------------------------------------------------------------------------
# DialogueStateMachine
# ---------------------------------------------------------------------------


class DialogueStateMachine:
    """Pure state machine for dialog lifecycle.

    States::

        IDLE ──WAKE_WORD──▶ LISTENING ──STT_RESULT──▶ DIALOGUE
         ▲                      │                        │
         │                      │ TIMEOUT                │ DIALOGUE_END
         │                      ▼                        │
         │                    IDLE  ◀────────────────────┘
         │
         │    ┌──────────────────────┐
         └────┤  SILENCED            │
              │  (from any state via │
              │   SILENCE_COMMAND)   │
              └──────┬───────────────┘
                     │ TIMEOUT / UNSILENCE
                     ▼
                   IDLE

    The machine is **stateless except for the current state enum**
    and the silence timeout. All conversation metadata lives in
    :class:`DialogState`, which is passed as a parameter rather
    than stored inside the DSM. This keeps the DSM testable with
    plain ``pytest`` and no mocking.
    """

    # Allowed transitions between DialogueStateKind values, used by
    # ``transition()`` for validation. The same graph is implemented
    # event-driven in ``on_event``; ``transition()`` is an alternative
    # entry point for callers (e.g. shell rescue paths) that want to
    # bypass event classification and force a specific target state.
    _ALLOWED_TRANSITIONS: dict[DialogueStateKind, frozenset[DialogueStateKind]] = {
        DialogueStateKind.IDLE: frozenset({
            DialogueStateKind.LISTENING,
            DialogueStateKind.SILENCED,
        }),
        DialogueStateKind.LISTENING: frozenset({
            DialogueStateKind.DIALOGUE,
            DialogueStateKind.IDLE,
            DialogueStateKind.SILENCED,
        }),
        DialogueStateKind.DIALOGUE: frozenset({
            DialogueStateKind.IDLE,
            DialogueStateKind.LISTENING,
            DialogueStateKind.SILENCED,
        }),
        DialogueStateKind.SILENCED: frozenset({
            DialogueStateKind.IDLE,
        }),
    }

    def __init__(
        self,
        initial: DialogueStateKind = DialogueStateKind.IDLE,
        silence_timeout: float = 300.0,
    ) -> None:
        """Create a new state machine.

        Args:
            initial: Starting state (default: IDLE).
            silence_timeout: Seconds before auto-return from SILENCED
                to IDLE (default: 300 s = 5 min). The harness is
                responsible for calling ``check_silence_timeout``
                periodically; the DSM itself has no timer loop.
        """
        self._state: DialogueStateKind = initial
        self._silence_timeout: float = silence_timeout
        self._silenced_at: float = 0.0
        self._last_activity_at: float = _now()

    # -- properties ---------------------------------------------------------

    @property
    def state(self) -> DialogueStateKind:
        """Current state of the machine."""
        return self._state

    @property
    def current_state(self) -> DialogueStateKind:
        """Alias for :attr:`state` — preferred name per W3 plan §3.

        Both names read the same internal field. The shell should
        use ``current_state`` in new code; ``state`` is kept for
        backward compatibility with existing callers.
        """
        return self._state

    @property
    def silence_timeout(self) -> float:
        """Configured silence timeout in seconds."""
        return self._silence_timeout

    @property
    def last_activity_at(self) -> float:
        """Unix timestamp of the most recent activity mark.

        Used by :meth:`check_inactivity_timeout` to decide whether
        ``LISTENING`` has been idle long enough to drop back to
        ``IDLE``. The clock is updated automatically by ``on_event``
        on ``STT_RESULT`` and ``WAKE_WORD``; callers can also call
        :meth:`mark_activity` explicitly.
        """
        return self._last_activity_at

    # -- direct transition API ---------------------------------------------

    def transition(
        self,
        new_state: DialogueStateKind,
        *,
        force: bool = False,
    ) -> DialogueStateKind:
        """Validate and apply a direct state transition.

        This is the explicit ``transition()`` entry point requested
        by the W3 plan §3. Unlike :meth:`on_event` (which infers the
        target from a :class:`DialogueEvent`), this method takes the
        target state directly and validates it against the allowed
        transition graph.

        Allowed transitions (mirrors :attr:`on_event` semantics):

        * ``IDLE``       → ``LISTENING``, ``SILENCED``
        * ``LISTENING``  → ``DIALOGUE``, ``IDLE``, ``SILENCED``
        * ``DIALOGUE``   → ``IDLE``, ``LISTENING``, ``SILENCED``
        * ``SILENCED``   → ``IDLE``

        Args:
            new_state: Target state.
            force: When ``True``, skip validation and apply the
                transition unconditionally. Use for shell rescue
                paths only — default is strict validation.

        Returns:
            The new state (== ``new_state``).

        Raises:
            TypeError: If ``new_state`` is not a
                :class:`DialogueStateKind`.
            ValueError: If the transition is not allowed and
                ``force`` is ``False``.
        """
        if not isinstance(new_state, DialogueStateKind):
            raise TypeError(
                f"DialogueStateMachine.transition: expected "
                f"DialogueStateKind, got {type(new_state).__name__}"
            )
        if not force and new_state != self._state:
            allowed = self._ALLOWED_TRANSITIONS.get(self._state, frozenset())
            if new_state not in allowed:
                raise ValueError(
                    f"Illegal dialogue transition: "
                    f"{self._state.name} → {new_state.name}. "
                    f"Allowed targets from {self._state.name}: "
                    f"{sorted(s.name for s in allowed) or '(none — use reset)'}"
                )
        self._state = new_state
        if new_state == DialogueStateKind.SILENCED:
            self._silenced_at = _now()
        elif new_state == DialogueStateKind.IDLE:
            self._silenced_at = 0.0
        self._last_activity_at = _now()
        return self._state

    def mark_activity(self) -> None:
        """Record that some user/system activity just happened.

        Updates :attr:`last_activity_at` to ``time.time()``. The
        :meth:`on_event` method calls this automatically on every
        event, so most callers don't need to invoke it directly.
        External timers (e.g. ROS2 STT callback) may call it when
        they observe activity outside the state machine's event
        surface.
        """
        self._last_activity_at = _now()

    def check_inactivity_timeout(self, timeout: float) -> bool:
        """Drop out of ``LISTENING`` after ``timeout`` seconds of silence.

        Implements the "LISTENING → IDLE after inactivity" rule
        from W3 plan §3. The shell is expected to call this
        periodically (e.g. on a ROS2 timer); the DSM itself does
        not run a background loop.

        Args:
            timeout: Inactivity window in seconds. Pass the
                shell-configured timeout (typically 5-10 s for
                wake-word follow-up listening).

        Returns:
            ``True`` if the state changed (i.e. ``LISTENING`` was
            dropped back to ``IDLE``); ``False`` otherwise.
        """
        if self._state != DialogueStateKind.LISTENING:
            return False
        if _now() - self._last_activity_at >= timeout:
            self._state = DialogueStateKind.IDLE
            self._last_activity_at = _now()
            return True
        return False

    # -- event processing ---------------------------------------------------

    def on_event(
        self,
        event: DialogueEvent,
        dialog_state: DialogState | None = None,
    ) -> DialogueStateKind:
        """Process an event and transition to the appropriate state.

        Args:
            event: The event driving the transition.
            dialog_state: Optional DialogState for context (e.g.
                checking silenced_until timestamp). May be None for
                simple transition tests.

        Returns:
            The new state after the transition.
        """
        # Every event counts as activity — even a no-op like
        # ``STT_RESULT`` in ``IDLE`` proves the user/ASR pipe is alive.
        self.mark_activity()
        current = self._state

        # ── SILENCE_COMMAND: any state → SILENCED ──
        if event == DialogueEvent.SILENCE_COMMAND:
            self._state = DialogueStateKind.SILENCED
            self._silenced_at = _now()
            return self._state

        # ── While in SILENCED ──
        if current == DialogueStateKind.SILENCED:
            if event in (DialogueEvent.TIMEOUT, DialogueEvent.UNSILENCE):
                self._state = DialogueStateKind.IDLE
                self._silenced_at = 0.0
                return self._state
            # All other events are ignored while SILENCED
            return self._state

        # ── IDLE ──
        if current == DialogueStateKind.IDLE:
            if event == DialogueEvent.WAKE_WORD:
                self._state = DialogueStateKind.LISTENING
            # STT_RESULT, DIALOGUE_END, TIMEOUT in IDLE are no-ops
            return self._state

        # ── LISTENING ──
        if current == DialogueStateKind.LISTENING:
            if event == DialogueEvent.STT_RESULT:
                self._state = DialogueStateKind.DIALOGUE
            elif event == DialogueEvent.TIMEOUT:
                self._state = DialogueStateKind.IDLE
            # WAKE_WORD in LISTENING is a no-op (already listening)
            return self._state

        # ── DIALOGUE ──
        if current == DialogueStateKind.DIALOGUE:
            if event == DialogueEvent.DIALOGUE_END:
                self._state = DialogueStateKind.IDLE
            elif event == DialogueEvent.WAKE_WORD:
                # Barge-in: wake word during dialogue → restart listen
                self._state = DialogueStateKind.LISTENING
            # STT_RESULT in DIALOGUE is a no-op (already processing)
            return self._state

        # Fallback: unrecognized transition — stay in current state
        return self._state

    def on_user_input(
        self,
        text: str,
        dialog_state: DialogState | None = None,
    ) -> DialogueEvent:
        """Classify raw user text into a :class:`DialogueEvent`.

        Pure function — no side effects, no I/O.

        Classification rules (order matters — silence commands
        are checked before wake words to prevent "роббокс замолчи"
        from being treated as a wake word):

        1. If text matches a silence command → ``SILENCE_COMMAND``
        2. If text matches a wake word → ``WAKE_WORD``
        3. Otherwise → ``STT_RESULT`` (normal speech input)

        Args:
            text: Raw STT transcript (already lowercased by the
                harness before calling).
            dialog_state: Optional state context (currently unused
                but accepted for future extension, e.g. per-user
                wake words).

        Returns:
            The classified event.
        """
        text_lower = text.lower().strip()
        if not text_lower:
            return DialogueEvent.TIMEOUT

        # Silence commands — check FIRST so "роббокс замолчи" is
        # treated as SILENCE_COMMAND, not WAKE_WORD.
        # 🔴 FIX (live 09:32): «молчи» — ТОЛЬКО как отдельная команда
        # (точное совпадение). Подстрока «молчи» в составе фразы
        # («сыграй баха и молчи просто играй» = играй без комментариев)
        # НЕ должна уводить DSM в SILENCED — иначе LLM не вызывается
        # и робот отвечает «Что-то я задумался».
        silence_triggers = (
            "тихо", "замолчи", "хватит", "стоп",
            "помолчи", "заткнись", "умолкни",
        )
        # Точные команды (вся фраза = команда замолчать)
        exact_silence = ("молчи", "молчать", "тишина", "тише")
        if any(trigger in text_lower for trigger in silence_triggers):
            return DialogueEvent.SILENCE_COMMAND
        if text_lower.strip() in exact_silence:
            return DialogueEvent.SILENCE_COMMAND

        # Wake words
        wake_triggers = (
            "роббокс", "роб бокс", "робокс", "робок", "робот",
            "robbox", "rob box",
        )
        if any(trigger in text_lower for trigger in wake_triggers):
            return DialogueEvent.WAKE_WORD

        return DialogueEvent.STT_RESULT

    def check_silence_timeout(self) -> DialogueEvent | None:
        """Check whether the silence period has expired.

        Returns:
            ``TIMEOUT`` if the silence duration has elapsed since
            ``SILENCE_COMMAND`` was received, ``None`` otherwise.
        """
        if self._state != DialogueStateKind.SILENCED:
            return None
        if _now() - self._silenced_at >= self._silence_timeout:
            return DialogueEvent.TIMEOUT
        return None

    # -- lifecycle ----------------------------------------------------------

    def reset(self, initial: DialogueStateKind = DialogueStateKind.IDLE) -> None:
        """Reset the machine to a known state.

        Args:
            initial: State to reset to (default: IDLE).
        """
        self._state = initial
        self._silenced_at = 0.0
        self._last_activity_at = _now()


__all__ = [
    "DialogueStateKind",
    "DialogueEvent",
    "DialogState",
    "DialogueStateMachine",
]
