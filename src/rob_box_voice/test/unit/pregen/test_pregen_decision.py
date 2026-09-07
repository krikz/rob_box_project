"""Tests for :func:`decision.decide` (ADR-0056 §6.4 #7)."""
from __future__ import annotations

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