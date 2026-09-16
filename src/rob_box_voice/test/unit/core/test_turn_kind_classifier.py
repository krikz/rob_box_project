"""test_turn_kind_classifier.py — issue #2627 PR-D unit tests.

Covers :func:`_classify_turn_kind` (and :class:`TurnKind`) which is the
dispatch-boundary validator for ``DialogueNode._run_turn``. The legacy
public signature keeps the 5 mutually-exclusive bool-flags
(``is_dj_auto``, ``is_babble_retry``, ``is_action_claim_retry``,
``is_code_retry``, ``is_synthetic``); this classifier maps them to a
single :class:`TurnKind` enum value and refuses to start a turn when
the caller sets more than one ``is_X_retry`` flag (a bug-class the
legacy code silently swallowed).

Run from ``src/rob_box_voice``::

    PYTHONPATH=src/rob_box_voice python -m pytest \\
        test/unit/core/test_turn_kind_classifier.py -v
"""

from __future__ import annotations

import sys
from pathlib import Path

_REPO_SRC = Path(__file__).resolve().parents[3] / "src" / "rob_box_voice"
if str(_REPO_SRC) not in sys.path:
    sys.path.insert(0, str(_REPO_SRC))

import pytest

# Import via the module — the enum and helper live at module scope in
# ``rob_box_voice.dialogue_node``, not behind a ``core/`` re-export.
from rob_box_voice.core.turn_kind import TurnKind, _classify_turn_kind  # noqa: E402


# ---------------------------------------------------------------------------
# Happy-path precedence — first match wins (matches the legacy ordering).
# ---------------------------------------------------------------------------


def test_classify_user_when_all_flags_false() -> None:
    """No flag set → TurnKind.USER (user-initiated)."""
    assert (
        _classify_turn_kind(
            is_dj_auto=False,
            is_babble_retry=False,
            is_action_claim_retry=False,
            is_code_retry=False,
            is_synthetic=False,
        )
        is TurnKind.USER
    )


def test_classify_dj_auto_wins_over_everything() -> None:
    """``is_dj_auto=True`` wins even when ``is_synthetic`` is also set.

    Legacy behavior: ``_run_turn`` checks ``was_dj_auto`` first when
    resetting music-guard budgets. The classifier mirrors that order.
    """
    assert (
        _classify_turn_kind(
            is_dj_auto=True,
            is_babble_retry=False,
            is_action_claim_retry=False,
            is_code_retry=False,
            is_synthetic=True,  # OK — DJ_AUTO outranks
        )
        is TurnKind.DJ_AUTO
    )


def test_classify_retry_babble() -> None:
    assert (
        _classify_turn_kind(
            is_dj_auto=False,
            is_babble_retry=True,
            is_action_claim_retry=False,
            is_code_retry=False,
            is_synthetic=True,  # legacy passes both — that's fine
        )
        is TurnKind.RETRY_BABBLE
    )


def test_classify_retry_action_claim() -> None:
    assert (
        _classify_turn_kind(
            is_dj_auto=False,
            is_babble_retry=False,
            is_action_claim_retry=True,
            is_code_retry=False,
            is_synthetic=True,
        )
        is TurnKind.RETRY_ACTION_CLAIM
    )


def test_classify_retry_code() -> None:
    assert (
        _classify_turn_kind(
            is_dj_auto=False,
            is_babble_retry=False,
            is_action_claim_retry=False,
            is_code_retry=True,
            is_synthetic=True,
        )
        is TurnKind.RETRY_CODE
    )


def test_classify_synthetic_other_catchall() -> None:
    """``is_synthetic=True`` with no retry → TurnKind.SYNTHETIC_OTHER.

    Catches the hallucinated-MIDI / phantom-action /
    universal-action-claim / unknown-melody guards that don't have a
    named retry variant yet.
    """
    assert (
        _classify_turn_kind(
            is_dj_auto=False,
            is_babble_retry=False,
            is_action_claim_retry=False,
            is_code_retry=False,
            is_synthetic=True,
        )
        is TurnKind.SYNTHETIC_OTHER
    )


# ---------------------------------------------------------------------------
# Bug-class — refuse to start when multiple retry flags are set.
# ---------------------------------------------------------------------------


def test_raises_when_two_retry_flags_set() -> None:
    """Two ``is_X_retry=True`` → ValueError (silent-drop bug-class)."""
    with pytest.raises(ValueError, match="More than one is_X_retry flag"):
        _classify_turn_kind(
            is_dj_auto=False,
            is_babble_retry=True,
            is_action_claim_retry=True,  # ← collides with babble
            is_code_retry=False,
            is_synthetic=True,
        )


def test_raises_when_three_retry_flags_set() -> None:
    with pytest.raises(ValueError, match="More than one is_X_retry flag"):
        _classify_turn_kind(
            is_dj_auto=False,
            is_babble_retry=True,
            is_action_claim_retry=True,
            is_code_retry=True,  # ← triple collision
            is_synthetic=True,
        )


def test_does_not_raise_when_dj_auto_collides_with_retry() -> None:
    """``is_dj_auto=True`` is allowed to coexist with retry flags.

    The bug-class check is ``is_X_retry`` only — DJ_AUTO is a different
    runtime, not a retry. ``is_dj_auto=True, is_babble_retry=True``
    means «DJ tick that happens to look like babble», which is legal.
    """
    # Should NOT raise:
    kind = _classify_turn_kind(
        is_dj_auto=True,
        is_babble_retry=True,
        is_action_claim_retry=False,
        is_code_retry=False,
        is_synthetic=True,
    )
    assert kind is TurnKind.DJ_AUTO


# ---------------------------------------------------------------------------
# TurnKind enum — sanity.
# ---------------------------------------------------------------------------


def test_turn_kind_values_are_distinct() -> None:
    values = {member.value for member in TurnKind}
    assert len(values) == len(TurnKind), (
        "TurnKind members must have unique string values"
    )


def test_turn_kind_user_is_default() -> None:
    """USER is the value at the bottom of the precedence list."""
    assert TurnKind.USER.value == "user"