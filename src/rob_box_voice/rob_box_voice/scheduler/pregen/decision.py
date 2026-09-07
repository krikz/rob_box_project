"""Final accept/reject decision for a speculative chunk (ADR-0056 §3.2).

This is the *last* gate before the speculative audio lands in
the pre-fetch cache. It composes:

* the audio-quality verdict from :mod:`quality`, and
* the confidence estimate from :mod:`estimator`

into a single :class:`Decision`. The rule is deliberately boring:

    ACCEPT  iff verdict == PASS AND confidence >= CONFIDENCE_FLOOR
    REJECT  otherwise

The ``basis`` string of the resulting :class:`Decision` carries
enough info for the executor to log "why" without re-running the
heuristics. The executor still owns the metric counter for the
respective :attr:`Decision.REJECT` reason — this module is pure.
"""

from __future__ import annotations

from enum import Enum

from .estimator import CONFIDENCE_FLOOR
from .quality import QualityVerdict


#: ``Decision.basis`` used when the verdict is :attr:`Decision.ACCEPT`.
#: Carried verbatim into :class:`PreGenResult.basis` for the
#: ``/voice/tts/metrics`` topic.
ACCEPT_BASIS_DEFAULT: str = "accept"


class Decision(str, Enum):
    """Outcome of the accept/reject gate.

    Subclassing :class:`str` so it serialises cleanly into logs
    and JSON metrics.
    """

    ACCEPT = "accept"
    REJECT = "reject"


def decide(
    verdict: QualityVerdict,
    confidence: float,
    *,
    confidence_floor: float = CONFIDENCE_FLOOR,
) -> Decision:
    """Apply the accept/reject rule.

    Parameters
    ----------
    verdict
        Outcome of :func:`quality.check_audio_quality` on the
        already-synthesised audio.
    confidence
        Outcome of :func:`estimator.estimate_confidence` for the
        same speculative task.
    confidence_floor
        Override knob for tests. Defaults to
        :data:`estimator.CONFIDENCE_FLOOR`.

    Returns
    -------
    Decision
        :attr:`Decision.ACCEPT` iff ``verdict is QualityVerdict.PASS``
        *and* ``confidence >= confidence_floor``. :attr:`Decision.REJECT`
        otherwise.

    Notes
    -----
    Pure: no side effects, no logging. The executor logs the
    reason when the decision is :attr:`Decision.REJECT` so the
    metrics topic records a precise counter (``pregens_rejected_quality``
    vs ``pregens_rejected_confidence`` vs ``pregens_rejected_both``).
    """
    if verdict is not QualityVerdict.PASS:
        return Decision.REJECT
    if confidence < confidence_floor:
        return Decision.REJECT
    return Decision.ACCEPT