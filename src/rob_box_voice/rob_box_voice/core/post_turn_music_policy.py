"""post_turn_music_policy.py — pure post-turn music state machine (Issue #2627).

Extracted from ``DialogueNode._run_turn.finally`` (legacy CC=59 → target
≤15, see ADR-0021 R1 / issue #2627). The 200-line music decision block
in the ``finally`` is now a single :func:`decide` call: the dialogue node
hands over the turn outcome and the mutable music state, the policy returns
a :class:`PostTurnActions` dataclass with the side-effect decisions, and the
node just executes them.

Why this lives in ``core/``:

* **No ROS2 / rclpy imports.** The policy takes a small typed snapshot
  (the ``DialogResult`` subset + 6 booleans) and returns a typed decision
  dataclass. ``core/`` is the agreed-upon home for testable business logic
  (see ``core/dialogue_guards.py`` / ``core/music_guard.py``).
* **Direct unit-test surface.** The policy is now testable without ROS2,
  without ``DialogueNode``, and without a Yandex STT pipeline. Tests for
  the BACKING-vs-TRACK discriminator (issue #992 Bug C), the double-stop
  deferral (issue #992 Bug B), the catch-up cleanup, and the DJ-auto
  short-circuit all live in ``test/unit/core/test_post_turn_music_policy.py``.
* **State SSoT (ADR-0021 R2).** Six flags is one too many sources of truth.
  :class:`PostTurnMusicState` is a frozen dataclass — the policy reads it,
  the dialogue node owns the canonical instance, and the next PR can move
  the owned instance into ``core/`` too without changing this module.

Scope (this module):

* :class:`TurnOutcome` — the slice of ``DialogResult`` we need
  (``tools_called``, ``spoken_text``, ``speak_text_count``).
* :class:`PostTurnMusicState` — the six mutable booleans the legacy
  ``finally`` consulted, plus ``active_batches``.
* :class:`PostTurnActions` — the four-decision dataclass.
* :func:`decide` — the pure state-machine function.
* :func:`is_singing_intent` — the BACKING detector's predicate, exposed so
  tests can probe it without rebuilding the policy.

Out of scope:

* Actually publishing / mutating state. The dialogue node adapter
  executes :class:`PostTurnActions` — this module never touches ``self``.
* Logging. Side-effect-rich log lines stay on the node, behind a single
  ``self.get_logger().info(...)`` per decision branch.
* Telemetry. ``PostTurnActions`` is plain Python; no metrics imports.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Iterable, Optional, Sequence, Tuple


# ---------------------------------------------------------------------------
# Inputs — pure data, no ROS
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TurnOutcome:
    """The minimum slice of :class:`DialogResult` the policy needs.

    Keeping it as a small dataclass (not the full ``DialogResult``) lets
    unit tests construct fixtures with one line and lets the policy stay
    decoupled from upstream churn (issue #2556 / #2547 added fields to
    ``DialogResult`` without touching the music state machine).

    Attributes:
        tools_called: Tool names invoked this turn. Empty tuple when the
            LLM returned no tool calls (frequent trigger for the
            catch-up cleanup branch).
        spoken_text: The LLM's user-facing reply, post-strip. ``""`` when
            the LLM returned nothing — same contract as
            ``DialogResult.spoken_text``.
        speak_text_count: Real ``speak_text`` invocations (issue #988
            phantom calls excluded). ``>= 2`` is the BACKING discriminator.
        user_input_for_intent: The original user command, used only by
            the singing-intent detector (BACKING vs TRACK split). Defaults
            to ``""`` when the dialogue node has no command context.
    """

    tools_called: Tuple[str, ...] = ()
    spoken_text: str = ""
    speak_text_count: int = 0
    user_input_for_intent: str = ""


@dataclass
class PostTurnMusicState:
    """Mutable music state owned by the dialogue node.

    The policy reads this; it never mutates the instance. The dialogue
    node writes back the chosen fields from :class:`PostTurnActions`
    after :func:`decide` returns. This matches the pattern used by
    :class:`rob_box_voice.core.turn.TurnState` — frozen dataclasses +
    ``replace`` for the ``TurnState`` half, mutable ``dataclass`` for the
    music state because the legacy code path set individual booleans in
    independent lines.

    Attributes:
        pending_cleanup: ``True`` when a music cleanup has been armed for
            the next ``tts_batch_complete``. Set by issue #935 v3 (the
            ``stop_music`` deferral).
        track_mode_active: ``True`` when the currently-playing music is a
            TRACK — it lives until an explicit ``stop_music`` or the
            watchdog, NOT until the next dialogue (issue #992 Bug C,
            live 30.08). Set by the BACKING-vs-TRACK discriminator below.
        active_batches: Count of TTS batches still in flight. Used by the
            catch-up cleanup branch (issue #992 prelude-deferral catch-up).
        stop_music_already_pending: ``True`` when a previous
            ``stop_music`` already armed ``pending_cleanup``; a second
            one must be ignored (issue #992 duplicate-stop deferral).
        last_tools_called: The most recent ``DialogResult.tools_called``
            (or ``()`` if the previous turn had no tools). Executor reads
            this to render the legacy log message
            ("stop_music deferred — will cleanup after TTS finishes"
            vs. "music_cleanup deferred — waiting for TTS or 10s fallback")
            so the diagnostic surface stays 1:1 with the pre-PR-C code.
    """

    pending_cleanup: bool = False
    track_mode_active: bool = False
    active_batches: int = 0
    stop_music_already_pending: bool = False
    last_tools_called: Tuple[str, ...] = ()


# ---------------------------------------------------------------------------
# Output — what the dialogue node must execute
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PostTurnActions:
    """Side-effect decisions the dialogue node must execute.

    The dialogue node hands :class:`PostTurnActions` to a small executor
    block — three ``if`` statements, no extra branches — keeping the
    legacy semantics 1:1 while collapsing the original 200-line decision
    block into a single function call.

    All four fields describe the **final post-turn state** the executor
    must reach (delta from current is computed in the executor, not in
    the policy). See :meth:`decide` for the merge of
    ``track_mode_active`` / ``pending_cleanup`` overrides with the
    inherited :class:`PostTurnMusicState` snapshot.

    Attributes:
        track_mode_active: Final value for ``state.track_mode_active``.
            ``True`` when the LLM started a TRACK (live 30.08) or a track
            from a previous turn survives. ``False`` after ``stop_music``
            or any non-music turn where no track survives.
        pending_cleanup: Final value for ``state.pending_cleanup``.
            ``True`` when the next ``tts_batch_complete`` must fire
            ``_publish_music_cleanup`` (BACKING + 2+ speak_text, default
            deferral, or stop_music-deferred arm).
        fire_cleanup_now: ``True`` when the catch-up branch
            (no active batches + cleanup pending) wants to fire
            ``_publish_music_cleanup(reason='tts_batch_complete')``
            IMMEDIATELY at the end of the turn. Mirrors the legacy
            ``_publish_music_cleanup`` call at ``dialogue_node.py:3911-3913``.
        close_session: ``True`` when the post-turn state machine wants
            the dialogue node to publish ``DIALOGUE_END`` at the bottom
            of this turn. ``False`` when a guard already dispatched a
            retry — the dialogue node keeps ``DIALOGUE`` open so the
            retry turn's LLM gate fires (issue #1204, #992 Bug D).
        skip_log: ``True`` when the executor should skip the redundant
            re-arm log line (issue #992 "already pending — ignoring
            redundant re-arm"). Lets the executor render that one-line
            branch inline without re-deriving the predicate.
    """

    track_mode_active: bool = False
    pending_cleanup: bool = False
    fire_cleanup_now: bool = False
    close_session: bool = True
    skip_log: bool = False


# ---------------------------------------------------------------------------
# Pure decision function
# ---------------------------------------------------------------------------


def decide(
    *,
    outcome: TurnOutcome,
    state: PostTurnMusicState,
    was_dj_auto: bool,
    user_input: str,
    guard_retry_pending: bool = False,
    music_retry_dispatched: bool = False,
    tool_retry_dispatched: bool = False,
    pending_queue_dispatched: bool = False,
    singing_intent_detector: Optional[Callable[[str], bool]] = None,
) -> PostTurnActions:
    """Pure post-turn music state machine.

    Maps 1:1 onto the legacy ``finally`` block at
    ``dialogue_node.py:3797-3985``. The policy is:

    1. If the LLM called ``stop_music``: arm (or keep armed)
       ``pending_cleanup`` unless already pending.
    2. Else, if the LLM called any ``MUSIC_STARTING_TOOLS``:
       * BACKING (2+ ``speak_text`` AND singing intent): set
         ``track_mode_active=False`` and arm ``pending_cleanup``.
       * TRACK (everything else): set ``track_mode_active=True`` and
         clear any pending cleanup (the new track supersedes it).
    3. Else, if a TRACK is already playing: leave state alone — the
       track lives through this turn (live 30.08).
    4. Else: arm ``pending_cleanup`` for the next ``tts_batch_complete``.

    The catch-up cleanup (step 5) and the session-close gate (step 6) are
    derived separately:

    5. If ``pending_cleanup`` is set AND ``active_batches == 0``:
       fire ``_publish_music_cleanup`` RIGHT NOW. Mirrors the legacy
       catch-up at ``dialogue_node.py:3911-3917``.
    6. Close the session (``DIALOGUE_END``) when none of the retry
       dispatchers are pending. Mirrors the legacy guard at
       ``dialogue_node.py:3970-3980``.

    Args:
        outcome: The LLM turn outcome (tools, spoken, count, intent input).
        state: The current mutable music state owned by the dialogue node.
            Read-only here.
        was_dj_auto: ``True`` for DJ-mode tick transitions. Skips the
            auto-arm branch (DJ transitions live by their own rules).
        user_input: The original user command (fallback for the singing
            intent detector when ``outcome.user_input_for_intent`` is empty).
        guard_retry_pending: ``True`` when one of the guards
            (``_apply_music_guard``, ``_apply_tool_skipped_guard``, …) already
            dispatched a retry this turn.
        music_retry_dispatched: ``True`` when ``_apply_music_guard`` fired
            a follow-up DJ turn. From the legacy executor — kept as a
            separate flag for symmetry with the legacy code shape.
        tool_retry_dispatched: ``True`` when ``_apply_tool_skipped_guard``
            fired a non-music tool retry.
        pending_queue_dispatched: ``True`` when ``_drain_pending_user_messages``
            merged queued phrases into a follow-up turn (issue #968 / S7).
        singing_intent_detector: Optional callable
            ``(text: str) -> bool`` for the BACKING detector. When ``None``,
            we use the inline :func:`is_singing_intent` predicate. Tests
            pass a stub to pin the discriminator without re-typing the
            keyword list.

    Returns:
        A :class:`PostTurnActions` the dialogue node must apply.

    Notes:
        * Pure — no ``self``, no I/O, no logging. The dialogue node
          adapter renders the log lines.
        * Stateless across calls — every input combination maps to exactly
          one :class:`PostTurnActions`. The Python ``@dataclass(frozen=True)``
          equality lets tests assert directly.
    """
    tools_called = set(outcome.tools_called or ())

    # Lazy import — the keyword constants live in ``dialogue_guards`` to
    # avoid a top-level cycle when this module is imported from a test.
    from .dialogue_guards import (
        MUSIC_MODE_TOOLS,
        MUSIC_STARTING_TOOLS,
        MUSIC_STOP_TOOLS,
    )

    music_starters = MUSIC_STARTING_TOOLS | MUSIC_MODE_TOOLS

    track_mode_active: Optional[bool] = None
    pending_cleanup: Optional[bool] = None
    skip_log = False

    if "stop_music" in tools_called:
        # Issue #935 v3 — defer cleanup until TTS finishes. Issue #992
        # duplicate-stop: a second ``stop_music`` (already armed) is a
        # no-op. The node renders the matching log line via ``skip_log``.
        # Issue #3108 live 31.08 — stop_music also DROPS the
        # ``track_mode_active`` flag so the next Bug-C retry does not
        # see «музыка играет» against an empty play queue. The legacy
        # code did this in a separate ``if tools_now & MUSIC_STOP_TOOLS``
        # branch (see ``dialogue_node.py:3874-3875`` pre-PR-C); here we
        # fuse it into the ``stop_music`` arm so the executor cannot
        # accidentally drop the reset when refactoring the dispatch.
        track_mode_active = False
        if state.stop_music_already_pending:
            skip_log = True
        else:
            pending_cleanup = True
            skip_log = False
    elif tools_called & music_starters:
        # Issue #992 TWO MUSIC MODES — BACKING (sing-along) vs TRACK.
        # BACKING = 2+ real speak_text AND singing intent in the user
        # command. Without the intent (live 13.08, «наполни комнату
        # музыкой» + greeting + comment) it's TRACK — no cleanup armed.
        backing_singing = (
            outcome.speak_text_count >= 2
            and _detect_singing_intent(
                outcome.user_input_for_intent or user_input or "",
                detector=singing_intent_detector,
            )
        )
        track_mode_active = not backing_singing
        if backing_singing:
            # Backup the cleanup arm — duplicate arms stay no-ops (issue
            # #992 «already pending»).
            if state.pending_cleanup:
                skip_log = True  # already pending — redundant arm
            else:
                pending_cleanup = True
        elif state.pending_cleanup:
            # TRACK supersedes BACKING's pending cleanup. LLM restarted
            # music — cancel the deferred stop (issue #992 live 02.09).
            pending_cleanup = False
        else:
            skip_log = True  # TRACK, nothing pending — no cleanup armed
    elif state.track_mode_active:
        # Live 30.08 — track survives this turn. No-op on the cleanup flag.
        track_mode_active = True  # explicit write so the adapter logs "kept"
        skip_log = True
    elif was_dj_auto or state.pending_cleanup:
        # DJ transitions live by their own rules — never arm cleanup here.
        # Same for «already pending» — the legacy branch returns the
        # redundant-arm debug line.
        track_mode_active = False
        skip_log = True
    else:
        # Default: arm the deferred cleanup for the next tts_batch_complete.
        pending_cleanup = True
        skip_log = False

    # Re-derive the final cleanup flag from ``state`` + the local override.
    final_pending = (
        pending_cleanup
        if pending_cleanup is not None
        else state.pending_cleanup
    )
    final_track_mode = (
        track_mode_active
        if track_mode_active is not None
        else state.track_mode_active
    )

    # Catch-up cleanup (issue #992 prelude-deferral catch-up).
    fire_cleanup_now = (
        final_pending
        and state.active_batches == 0
        and not was_dj_auto
    )

    # Session-close gate (issue #1204, #992 Bug D, S7).
    close_session = not (
        guard_retry_pending
        or music_retry_dispatched
        or tool_retry_dispatched
        or pending_queue_dispatched
    )

    return PostTurnActions(
        track_mode_active=final_track_mode,
        pending_cleanup=final_pending,
        fire_cleanup_now=fire_cleanup_now,
        close_session=close_session,
        skip_log=skip_log,
    )


# ---------------------------------------------------------------------------
# BACKING detector (predicate only)
# ---------------------------------------------------------------------------


def is_singing_intent(text: str) -> bool:
    """BACKING discriminator — does ``text`` look like a sing-along request?

    Mirrors the legacy inline detector at
    ``dialogue_node.py:3859-3861`` (``_has_singing_intent``). Kept as a
    thin wrapper so the policy can pass an explicit detector and tests can
    substitute a stub. The actual regex lives in
    :mod:`rob_box_voice.core.dialogue_guards` (``has_singing_intent``)
    so both the dialogue node and the policy agree on the BACKING-vs-TRACK
    split without drift.

    Empty / ``None`` text returns ``False`` (matches the legacy behaviour
    where ``_has_singing_intent("")`` was falsy).
    """
    return _detect_singing_intent(text)


def _detect_singing_intent(
    text: str,
    *,
    detector: Optional[object] = None,
) -> bool:
    if not text:
        return False
    if detector is not None:
        return bool(detector(text))
    from .dialogue_guards import has_singing_intent
    # Issue #2627 — delegate to the public, narrow ``has_singing_intent``
    # in :mod:`rob_box_voice.core.dialogue_guards` (mirrors the legacy
    # ``DialogueNode._has_singing_intent`` regex). Tests can substitute
    # the ``detector`` callable above to pin the discriminator without
    # rebuilding the keyword regex.
    return has_singing_intent(text)


#: Re-export the deterministic call signature for tests that want to
#: patch ``is_singing_intent`` without going through the adapter.
__all__ = [
    "PostTurnActions",
    "PostTurnMusicState",
    "TurnOutcome",
    "decide",
    "is_singing_intent",
]
