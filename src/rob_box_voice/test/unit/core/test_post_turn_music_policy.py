"""test_post_turn_music_policy.py — Issue #2627 pure-decision unit tests.

Covers the BACKING-vs-TRACK discriminator (issue #992 Bug C), the
double-stop deferral (issue #992 Bug B), the catch-up cleanup when no
TTS batches are active, and the DJ-auto short-circuit. The dialogue
node adapter stays thin — every state machine branch lives here and
gets exercised without ROS2, without ``DialogueNode``, and without a
running voice pipeline.

Issue #2627 acceptance: ``PostTurnMusicPolicy`` (or equivalent) lives in
``src/rob_box_voice/rob_box_voice/core/``, pure, no ROS dependencies.
This test file is the executable proof.

Run from ``src/rob_box_voice``::

    PYTHONPATH=src/rob_box_voice python -m pytest \\
        test/unit/core/test_post_turn_music_policy.py -v
"""

from __future__ import annotations

import sys
from pathlib import Path

# Make ``rob_box_voice.core`` importable from this layout (the repo's
# pytest configures ``src/rob_box_voice`` on sys.path, but the unit tests
# sometimes run from a different cwd).
_REPO_SRC = Path(__file__).resolve().parents[3] / "src" / "rob_box_voice"
if str(_REPO_SRC) not in sys.path:
    sys.path.insert(0, str(_REPO_SRC))

from rob_box_voice.core.post_turn_music_policy import (  # noqa: E402
    PostTurnActions,
    PostTurnMusicState,
    TurnOutcome,
    decide,
    is_singing_intent,
)
from rob_box_voice.core.dialogue_guards import (  # noqa: E402
    has_singing_intent,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _outcome(
    tools=(),
    spoken="ok",
    speak_text_count=1,
    user_input="",
):
    return TurnOutcome(
        tools_called=tuple(tools),
        spoken_text=spoken,
        speak_text_count=speak_text_count,
        user_input_for_intent=user_input,
    )


def _state(
    pending=False,
    track=False,
    active_batches=1,
    stop_pending=False,
):
    return PostTurnMusicState(
        pending_cleanup=pending,
        track_mode_active=track,
        active_batches=active_batches,
        stop_music_already_pending=stop_pending,
    )


# ---------------------------------------------------------------------------
# 1. BACKING vs TRACK discriminator (issue #992 Bug C — two music modes)
# ---------------------------------------------------------------------------


def test_backing_mode_when_two_speak_text_and_singing_intent():
    """2+ speak_text + «спой» → BACKING → cleanup armed, track_mode off."""
    actions = decide(
        outcome=_outcome(
            tools=("execute_music_code",),
            speak_text_count=2,
            user_input="робот, спой песенку",
        ),
        state=_state(),
        was_dj_auto=False,
        user_input="робот, спой песенку",
    )
    assert actions.track_mode_active is False
    assert actions.pending_cleanup is True
    assert actions.skip_log is False


def test_track_mode_when_only_one_speak_text():
    """1 speak_text + execute_music_code → TRACK → no cleanup armed."""
    actions = decide(
        outcome=_outcome(
            tools=("execute_music_code",),
            speak_text_count=1,
            user_input="сыграй баха",
        ),
        state=_state(),
        was_dj_auto=False,
        user_input="сыграй баха",
    )
    assert actions.track_mode_active is True
    # TRACK supersedes a pending cleanup (legacy live 02.09).
    assert actions.pending_cleanup is False


def test_track_mode_when_no_singing_intent():
    """execute_music_code + greeting (no intent) → TRACK (live 13.08).

    The classic live failure mode: «наполни комнату музыкой» → execute_music_code
    fires, but only a greeting is voiced (speak_text_count=1) and there is
    no singing intent in the user command. The policy must NOT arm cleanup.
    """
    actions = decide(
        outcome=_outcome(
            tools=("execute_music_code",),
            speak_text_count=1,
            user_input="наполни комнату музыкой",
        ),
        state=_state(),
        was_dj_auto=False,
        user_input="наполни комнату музыкой",
    )
    assert actions.track_mode_active is True
    assert actions.pending_cleanup is False


def test_backing_arms_cleanup_and_drops_pending_when_overlap():
    """When a previous turn armed cleanup AND this is BACKING → no double-arm.

    Mirrors legacy «already pending» debug branch at
    ``dialogue_node.py:3873-3877``.
    """
    actions = decide(
        outcome=_outcome(
            tools=("execute_music_code",),
            speak_text_count=2,
            user_input="спой про весну",
        ),
        state=_state(pending=True),
        was_dj_auto=False,
        user_input="спой про весну",
    )
    assert actions.track_mode_active is False
    assert actions.pending_cleanup is True  # still pending (idempotent)
    assert actions.skip_log is True  # «already pending» redundant arm


# ---------------------------------------------------------------------------
# 2. Stop-music deferral (issue #935 v3 + #992 Bug B)
# ---------------------------------------------------------------------------


def test_stop_music_arms_pending_cleanup():
    actions = decide(
        outcome=_outcome(tools=("stop_music",), spoken="выключаю"),
        state=_state(),
        was_dj_auto=False,
        user_input="выключи музыку",
    )
    assert actions.pending_cleanup is True
    assert actions.skip_log is False


def test_duplicate_stop_music_is_a_noop():
    """Second ``stop_music`` while one is already pending → ignore (issue #992)."""
    actions = decide(
        outcome=_outcome(tools=("stop_music",), spoken="выключаю"),
        state=_state(pending=True, stop_pending=True),
        was_dj_auto=False,
        user_input="выключи музыку",
    )
    # The flag stays ``True`` (it was already True) and the executor renders
    # the «ignoring duplicate» debug line.
    assert actions.pending_cleanup is True
    assert actions.skip_log is True


# ---------------------------------------------------------------------------
# 3. Catch-up cleanup (issue #992 prelude-deferral catch-up)
# ---------------------------------------------------------------------------


def test_catch_up_fires_cleanup_when_no_active_batches():
    """Armed cleanup + 0 active batches + non-DJ → fire RIGHT NOW."""
    actions = decide(
        outcome=_outcome(tools=("speak_text",), speak_text_count=1),
        state=_state(pending=True, active_batches=0),
        was_dj_auto=False,
        user_input="привет",
    )
    assert actions.pending_cleanup is True
    assert actions.fire_cleanup_now is True


def test_catch_up_skipped_when_active_batches_present():
    """Armed cleanup but batches still in flight → defer (no fire)."""
    actions = decide(
        outcome=_outcome(tools=("speak_text",), speak_text_count=1),
        state=_state(pending=True, active_batches=2),
        was_dj_auto=False,
        user_input="привет",
    )
    assert actions.fire_cleanup_now is False


def test_catch_up_skipped_for_dj_auto():
    """DJ transitions never fire cleanup directly (DJ lives by its own rules)."""
    actions = decide(
        outcome=_outcome(tools=("speak_text",), speak_text_count=1),
        state=_state(pending=True, active_batches=0),
        was_dj_auto=True,
        user_input="",  # DJ auto-turns have no user input
    )
    assert actions.fire_cleanup_now is False


# ---------------------------------------------------------------------------
# 4. Track survival (live 30.08 — track lives through non-music turns)
# ---------------------------------------------------------------------------


def test_track_survives_non_music_turn():
    """A TRACK playing from a previous turn must NOT be cleaned up here."""
    actions = decide(
        outcome=_outcome(tools=("speak_text",), speak_text_count=1),
        state=_state(track=True, active_batches=1),
        was_dj_auto=False,
        user_input="продолжай играть",
    )
    # Track-mode stays True — no cleanup armed.
    assert actions.track_mode_active is True
    assert actions.pending_cleanup is False


# ---------------------------------------------------------------------------
# 5. Session-close gate (issue #1204 / #992 Bug D / S7)
# ---------------------------------------------------------------------------


def test_session_closes_when_no_retries_pending():
    actions = decide(
        outcome=_outcome(tools=("speak_text",), speak_text_count=1),
        state=_state(active_batches=0),
        was_dj_auto=False,
        user_input="привет",
    )
    assert actions.close_session is True


def test_session_stays_open_when_guard_dispatched_retry():
    """Issue #1204 / #992 Bug D — guard dispatched retry → keep DIALOGUE open.

    The retry's ``_run_turn`` re-entry needs the DSM in DIALOGUE so the LLM
    gate fires. Without this gate the retry's synthetic prompt would hit
    IDLE and short-circuit to a no-op.
    """
    actions = decide(
        outcome=_outcome(tools=("speak_text",), speak_text_count=1),
        state=_state(active_batches=0),
        was_dj_auto=False,
        user_input="привет",
        music_retry_dispatched=True,
    )
    assert actions.close_session is False


def test_session_stays_open_when_pending_queue_dispatched():
    """S7 — drained pending phrases are a continuation, not the end."""
    actions = decide(
        outcome=_outcome(tools=("speak_text",), speak_text_count=1),
        state=_state(active_batches=0),
        was_dj_auto=False,
        user_input="привет",
        pending_queue_dispatched=True,
    )
    assert actions.close_session is False


def test_session_stays_open_when_tool_retry_dispatched():
    actions = decide(
        outcome=_outcome(tools=(), speak_text_count=0),
        state=_state(active_batches=0),
        was_dj_auto=False,
        user_input="который час",
        tool_retry_dispatched=True,
    )
    assert actions.close_session is False


# ---------------------------------------------------------------------------
# 6. Singing-intent predicate — direct probe
# ---------------------------------------------------------------------------


def test_is_singing_intent_sing_keyword():
    assert is_singing_intent("спой песенку") is True


def test_is_singing_intent_track_keyword_is_false():
    """«сыграй баха» has no singing intent — TRACK, not BACKING."""
    assert is_singing_intent("сыграй баха") is False


def test_is_singing_intent_empty_is_false():
    assert is_singing_intent("") is False


# ---------------------------------------------------------------------------
# 7. ``has_singing_intent`` — direct detector probe (issue #2627 SSoT)
# ---------------------------------------------------------------------------


def test_has_singing_intent_positive_cases():
    """Every vocal stem in the legacy regex must match."""
    for phrase in ("спой песенку", "Спой мне", "пой что-нибудь", "рэп", "реп", "песенку", "куплет", "частушки", "напев"):
        assert has_singing_intent(phrase) is True, phrase


def test_has_singing_intent_negative_cases():
    """Plain music / track commands must NOT trigger BACKING.

    This is the canonical live 13.08 case: «наполни комнату музыкой» →
    ``execute_music_code`` fires, but the user did NOT ask for a
    sing-along, so the policy must mark it TRACK (no deferred cleanup).
    """
    for phrase in ("сыграй баха", "включи трек", "наполни комнату музыкой", "поставь музыку"):
        assert has_singing_intent(phrase) is False, phrase


def test_has_singing_intent_handles_empty_and_none():
    """Empty / ``None`` text returns ``False`` (matches legacy)."""
    assert has_singing_intent("") is False
    assert has_singing_intent(None) is False


# ---------------------------------------------------------------------------
# 8. ``decide`` accepts an external ``singing_intent_detector`` (issue #2627)
# ---------------------------------------------------------------------------


def test_decide_uses_external_singing_intent_detector():
    """A stub detector forces BACKING/track_mode off without re-typing the regex.

    This is the integration point for future policy tuning (live A/B
    experiments can swap the detector without touching the policy).
    """
    forced_backing = lambda _text: True  # noqa: E731 — explicit stub
    actions = decide(
        outcome=_outcome(
            tools=("execute_music_code",),
            speak_text_count=2,
            user_input="абсолютно не певческий ввод",
        ),
        state=_state(),
        was_dj_auto=False,
        user_input="абсолютно не певческий ввод",
        singing_intent_detector=forced_backing,
    )
    assert actions.track_mode_active is False
    assert actions.pending_cleanup is True


def test_decide_external_detector_negative_yields_track():
    forced_track = lambda _text: False  # noqa: E731
    actions = decide(
        outcome=_outcome(
            tools=("execute_music_code",),
            speak_text_count=2,
            user_input="спой песенку",  # legacy would say True
        ),
        state=_state(),
        was_dj_auto=False,
        user_input="спой песенку",
        singing_intent_detector=forced_track,
    )
    assert actions.track_mode_active is True  # stub forced TRACK
    assert actions.pending_cleanup is False
