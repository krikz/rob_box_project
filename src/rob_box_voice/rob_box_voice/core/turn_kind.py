"""turn_kind.py — Issue #2627 PR-D single source of truth for a turn's origin.

Pre-PR-D, :meth:`DialogueNode._run_turn` took 5 mutually-exclusive
bool-flags (``is_dj_auto``, ``is_babble_retry``,
``is_action_claim_retry``, ``is_code_retry``, ``is_synthetic``) that
combinatorially exploded into 32 possible invocations — and the
contract that exactly one of them is True was never enforced. Every
branch in ``_run_turn`` checked ``is_X`` separately, so the CC-budget
measured each one as its own branch even though they represent a
single decision (what kind of turn is this?).

After PR-D the five flags are still accepted on the public signature
(backwards compatible — guard callers like ``_check_babble_and_retry``
keep their ``is_babble_retry=True, is_synthetic=True`` shapes), but
:meth:`_run_turn` derives a single :class:`TurnKind` from them at the
top and routes on the enum value. Invalid combinations (two retries +
synthetic, etc.) raise :class:`ValueError` immediately, so the
bug-class — "guard chain set two flags and the second was silently
ignored" — is closed at the dispatch boundary.

This module lives in ``core/`` because:

* **No ROS2 / rclpy imports.** The enum and the pure classifier are
  testable without spinning up the dialogue node (rclpy pulls in
  ``pyyaml``, ``numpy``, ``opencv``, etc. — too heavy for a unit
  test).
* **Pure function on plain primitives.** ``_classify_turn_kind`` takes
  5 booleans and returns an enum value, no ``self``, no I/O.

Variants (ordered by precedence, first match wins in
:func:`_classify_turn_kind`):

* ``DJ_AUTO`` — DJ auto-tick transition. Skips the wake-word
  classifier and bypasses the speaker pipeline (issue #992).
* ``RETRY_BABBLE`` — ``_check_babble_and_retry`` follow-up turn.
  Budget reset is suppressed (issue #992 Bug D).
* ``RETRY_ACTION_CLAIM`` — unbacked-action-claim guard
  follow-up. Mirrors the babble case.
* ``RETRY_CODE`` — Renardo code-speech guard follow-up.
* ``SYNTHETIC_OTHER`` — generic synthetic turn (hallucinated-MIDI,
  phantom-action, universal-action-claim, unknown-melody, etc.)
  that does not fit the three named retries above.
* ``USER`` — user-initiated turn. Resets all budgets.

Notes:
    ADR-0021 R2 (State SSoT) — single source of truth for "what kind
    of turn is this?". Future guard catch-sites should pick the
    closest matching variant instead of inventing yet another
    ``is_X_retry`` flag.
"""

from __future__ import annotations

from enum import Enum


class TurnKind(Enum):
    """Dispatch-boundary enum for :meth:`DialogueNode._run_turn`.

    See module docstring for the full rationale. Enum values are
    lowercase strings so :func:`repr` is grep-friendly in production
    logs (``turn_kind=user``, ``turn_kind=dj_auto``).
    """

    USER = "user"
    DJ_AUTO = "dj_auto"
    RETRY_BABBLE = "retry_babble"
    RETRY_ACTION_CLAIM = "retry_action_claim"
    RETRY_CODE = "retry_code"
    SYNTHETIC_OTHER = "synthetic_other"


def _classify_turn_kind(
    *,
    is_dj_auto: bool,
    is_babble_retry: bool,
    is_action_claim_retry: bool,
    is_code_retry: bool,
    is_synthetic: bool,
) -> TurnKind:
    """Issue #2627 PR-D — derive :class:`TurnKind` from the 5 bool-flags.

    Precedence (first match wins, matches the legacy ordering):

    1. ``is_dj_auto`` — DJ auto-tick
    2. ``is_babble_retry`` — babble-detector follow-up
    3. ``is_action_claim_retry`` — unbacked-action-claim follow-up
    4. ``is_code_retry`` — Renardo code-speech follow-up
    5. ``is_synthetic`` — generic synthetic turn (catch-all for
       hallucinated-MIDI / phantom-action / universal-action-claim /
       unknown-melody / etc.)
    6. otherwise — :attr:`TurnKind.USER`

    Raises:
        ValueError: when more than one ``is_X_retry`` flag is set
            (the legacy code silently dropped the second one — a bug
            class). The dispatch path catches this via
            :meth:`DialogueNode._run_turn` and refuses to start the
            turn.
    """
    retry_count = sum(
        (is_babble_retry, is_action_claim_retry, is_code_retry)
    )
    if retry_count > 1:
        raise ValueError(
            "More than one is_X_retry flag set — bug-class. "
            f"is_babble_retry={is_babble_retry}, "
            f"is_action_claim_retry={is_action_claim_retry}, "
            f"is_code_retry={is_code_retry}"
        )
    if is_dj_auto:
        return TurnKind.DJ_AUTO
    if is_babble_retry:
        return TurnKind.RETRY_BABBLE
    if is_action_claim_retry:
        return TurnKind.RETRY_ACTION_CLAIM
    if is_code_retry:
        return TurnKind.RETRY_CODE
    if is_synthetic:
        return TurnKind.SYNTHETIC_OTHER
    return TurnKind.USER


__all__ = ["TurnKind", "_classify_turn_kind"]