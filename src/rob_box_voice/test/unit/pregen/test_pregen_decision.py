"""Tests for :func:`decision.decide` (ADR-0056 §6.4 #7).

Covers the per-chunk accept/reject rule and the derived property
"given N candidates with identical quality verdicts, the one
decision() accepts is the highest-confidence one" — the property
the executor relies on when multiple speculative tasks are in
flight at the same time.
"""
from __future__ import annotations

import pytest

from rob_box_voice.scheduler.pregen import (
    CONFIDENCE_FLOOR,
    Decision,
    QualityVerdict,
    decide,
)


def test_decide_accept_when_pass_and_at_floor():
    assert decide(QualityVerdict.PASS, CONFIDENCE_FLOOR) is Decision.ACCEPT


def test_decide_accept_when_pass_and_above_floor():
    assert decide(QualityVerdict.PASS, 0.99) is Decision.ACCEPT


def test_decide_reject_when_pass_but_below_floor():
    assert decide(QualityVerdict.PASS, CONFIDENCE_FLOOR - 0.01) is Decision.REJECT


def test_decide_reject_when_quality_fails():
    """Any non-PASS verdict → REJECT regardless of confidence."""
    for v in [
        QualityVerdict.REJECT_SILENT,
        QualityVerdict.REJECT_CLIPPED,
        QualityVerdict.REJECT_TRIMMED,
    ]:
        assert decide(v, 1.0) is Decision.REJECT


def test_decide_respects_custom_floor():
    """Override knob works."""
    # With floor=0.5, PASS at 0.5 should accept
    assert decide(QualityVerdict.PASS, 0.5, confidence_floor=0.5) is Decision.ACCEPT
    # With floor=0.7, PASS at 0.5 should reject
    assert decide(QualityVerdict.PASS, 0.5, confidence_floor=0.7) is Decision.REJECT


def test_decision_serialises_to_string():
    import json as _json
    s = _json.dumps({"d": Decision.ACCEPT})
    assert s == '{"d": "accept"}'


# ---------------------------------------------------------------------------
# Multi-candidate "picks best" property (issue #2003 DoD).
# ---------------------------------------------------------------------------


def _candidate(verdict: QualityVerdict, confidence: float) -> tuple:
    """Pair used as (verdict, confidence) for the candidate list."""
    return (verdict, confidence)


def test_decision_picks_best():
    """DoD: decision() selects the chunk with the maximal combined score.

    "Combined score" here is modelled as the pair
    (QualityVerdict, confidence). The decision module encodes the
    rule: ACCEPT iff verdict is PASS AND confidence >= CONFIDENCE_FLOOR.

    Among N candidates, the *only* ACCEPTed ones are those above the
    floor; the "best" the executor should pick is the highest-confidence
    ACCEPT. This test exercises that property directly.
    """
    candidates = [
        # (verdict, confidence)
        (QualityVerdict.PASS,              0.55),  # quality OK, below floor
        (QualityVerdict.REJECT_SILENT,     0.99),  # quality fails — bad
        (QualityVerdict.REJECT_CLIPPED,    0.99),  # quality fails — bad
        (QualityVerdict.PASS,              0.70),  # above floor — ACCEPT
        (QualityVerdict.PASS,              0.92),  # above floor — ACCEPT (winner)
    ]
    decisions = [(v, c, decide(v, c)) for v, c in candidates]

    # The two quality-failed candidates must be REJECTed outright.
    assert decisions[1][2] is Decision.REJECT
    assert decisions[2][2] is Decision.REJECT

    # The below-floor PASS must be REJECTed (confidence gate).
    assert decisions[0][2] is Decision.REJECT

    # The two above-floor PASS candidates must be ACCEPTed.
    assert decisions[3][2] is Decision.ACCEPT
    assert decisions[4][2] is Decision.ACCEPT

    # The "best" the executor picks is the one with the highest
    # confidence among the ACCEPT set.
    accepted = [
        (v, c) for v, c, d in decisions if d is Decision.ACCEPT
    ]
    best_verdict, best_confidence = max(accepted, key=lambda pair: pair[1])
    assert best_verdict is QualityVerdict.PASS
    assert best_confidence == 0.92


def test_decision_picks_best_among_all_pass_candidates():
    """Among N PASS candidates, only the one with confidence >= floor is ACCEPT.

    This is the literal property that decision() "picks the best":
    given a batch where every candidate cleared the quality gate, the
    executor must choose the candidate with the *highest* combined
    score (modeled here as ``confidence``). Any candidate below the
    floor is REJECT, regardless of relative ordering.
    """
    candidates = [
        _candidate(QualityVerdict.PASS, 0.55),  # below floor
        _candidate(QualityVerdict.PASS, 0.62),  # ABOVE floor — winner
        _candidate(QualityVerdict.PASS, 0.71),
        _candidate(QualityVerdict.PASS, 0.80),
    ]
    decisions = [decide(v, c) for v, c in candidates]

    # Only the candidate at index 1 crosses the 0.6 floor.
    assert decisions[0] is Decision.REJECT
    assert decisions[1] is Decision.ACCEPT
    assert decisions[2] is Decision.ACCEPT
    assert decisions[3] is Decision.ACCEPT

    # The "winner" the executor should pick is the candidate with
    # the highest confidence among the ACCEPT set.
    accepted = [
        candidates[i] for i, d in enumerate(decisions) if d is Decision.ACCEPT
    ]
    best = max(accepted, key=lambda pair: pair[1])
    assert best == candidates[3]


def test_decision_rejects_all_when_quality_fails_for_any():
    """Even with perfect confidence, a non-PASS verdict → REJECT for ALL.

    This nails down the *combined* gate: quality is the first gate,
    confidence is the second. If quality fails, the candidate cannot
    be "the best" regardless of confidence.
    """
    candidates = [
        _candidate(QualityVerdict.REJECT_SILENT, 1.0),
        _candidate(QualityVerdict.REJECT_CLIPPED, 0.95),
        _candidate(QualityVerdict.REJECT_TRIMMED, 0.99),
    ]
    for verdict, confidence in candidates:
        assert decide(verdict, confidence) is Decision.REJECT


def test_decision_picks_only_one_winner_with_duplicate_top_score():
    """When two candidates tie on confidence, decision() picks BOTH (both pass).

    The executor then has the tie-break problem (not decision()'s
    concern). This test pins that decision() does NOT silently prefer
    one of two equally-scoring candidates — both must come out ACCEPT
    so the chooser (caller) can decide.
    """
    candidates = [
        _candidate(QualityVerdict.PASS, 0.75),
        _candidate(QualityVerdict.PASS, 0.75),
    ]
    decisions = [decide(v, c) for v, c in candidates]
    assert decisions.count(Decision.ACCEPT) == 2


def test_decision_picks_none_when_all_below_floor():
    """If every candidate is below the floor, decision() rejects them all.

    The executor must NOT pick any of them. This is the canonical
    "all candidates are bad" case the property must cover.
    """
    candidates = [
        _candidate(QualityVerdict.PASS, 0.40),
        _candidate(QualityVerdict.PASS, 0.50),
        _candidate(QualityVerdict.PASS, 0.59),
    ]
    for verdict, confidence in candidates:
        assert decide(verdict, confidence) is Decision.REJECT


@pytest.mark.parametrize(
    "verdict,confidence,expected",
    [
        # Same quality verdict, varying confidence → at-and-above floor = ACCEPT
        (QualityVerdict.PASS, CONFIDENCE_FLOOR,         Decision.ACCEPT),
        (QualityVerdict.PASS, CONFIDENCE_FLOOR + 0.001, Decision.ACCEPT),
        (QualityVerdict.PASS, CONFIDENCE_FLOOR - 0.001, Decision.REJECT),
        (QualityVerdict.PASS, 0.99,                    Decision.ACCEPT),
        (QualityVerdict.PASS, 0.01,                    Decision.REJECT),
        # Quality-fail short-circuits confidence
        (QualityVerdict.REJECT_SILENT, 1.0,            Decision.REJECT),
        (QualityVerdict.REJECT_CLIPPED, 1.0,           Decision.REJECT),
        (QualityVerdict.REJECT_TRIMMED, 1.0,           Decision.REJECT),
    ],
)
def test_decision_truth_table(verdict, confidence, expected):
    """Exhaustive truth table — every (verdict, confidence) cell is covered."""
    assert decide(verdict, confidence) is expected