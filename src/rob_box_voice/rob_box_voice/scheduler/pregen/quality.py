"""Audio-quality gate for speculative chunks (ADR-0056 §3.6).

Three cheap heuristics, applied to the **already-synthesised**
audio before it lands in the pre-fetch cache. They are not
pretending to be a full audio QA system — that's a future ADR.
They are the first barrier that catches the obvious failure modes:

1. **silent** — the synthesizer produced near-silence (rms below
   the floor). Typical for Silero on a non-cyrillic script that
   we mis-routed, or for a Yandex ``UtteranceSynthesis`` that
   returned an empty buffer.
2. **clipped** — the actual duration is way longer than the
   estimator's prediction. Typical for Yandex after a long SSML
   silently got truncated to a single short chunk, leaving the
   audio shorter than the cache key implies — or for the opposite
   case where Yandex inserted extra silence to fill a ``<break>``
   that LLM hallucinated.
3. **trimmed** — the audio has too much silence at the start
   and/or end (sentinel of "the model produced audio but the
   meaningful part is microscopic").

Each verdict carries enough info to log a meaningful reason
without an extra string-formatting dance in the executor.
"""

from __future__ import annotations

from enum import Enum
from typing import Optional

import numpy as np


#: Maximum allowed ratio of actual-vs-estimated duration.
#: 1.25 = 25% tolerance. Generous enough not to flag normal
#: prosody drift, tight enough to catch a half-truncated chunk.
DURATION_RATIO_FLOOR: float = 1.25


#: Silence threshold (0.0..1.0) — fraction of audio that can be
#: "near-silence" at head/tail before we treat it as trimmed.
#: 0.15 = 15% — a normal chunk has < 5% head/tail silence.
SILENCE_RATIO_FLOOR: float = 0.15


#: RMS threshold in dBFS below which the chunk is considered
#: silent. -40 dBFS is roughly the noise floor of a working
#: synthesizer on a quiet room mic; below that the user hears
#: nothing.
RMS_DB_FLOOR: float = -40.0


#: Sample-rate threshold (Hz) below which the heuristics refuse to
#: run and the gate returns PASS. Realistic speech is 8 kHz+;
#: anything lower is an obvious misconfiguration.
_MIN_SAMPLE_RATE_HZ: int = 8000


class QualityVerdict(str, Enum):
    """Outcome of the audio quality gate.

    Subclassing :class:`str` so it serialises cleanly into logs
    and JSON metrics (``str(QualityVerdict.PASS) == "pass"``).
    """

    PASS = "pass"
    REJECT_SILENT = "reject_silent"
    REJECT_CLIPPED = "reject_clipped"
    REJECT_TRIMMED = "reject_trimmed"


def _rms_dbfs(audio: np.ndarray) -> float:
    """Root-mean-square of ``audio`` in dBFS (full-scale sine = 0 dB).

    Returns ``-inf`` for silent input. Handles empty arrays
    safely (returns ``-inf``).
    """
    if audio.size == 0:
        return float("-inf")
    # Guard against pathological inputs (NaN/inf) that would
    # propagate into dBFS.
    audio_f = np.asarray(audio, dtype=np.float32)
    if not np.all(np.isfinite(audio_f)):
        audio_f = np.nan_to_num(audio_f, nan=0.0, posinf=0.0, neginf=0.0)
    rms = float(np.sqrt(np.mean(audio_f ** 2)))
    if rms <= 0.0:
        return float("-inf")
    # dBFS = 20 * log10(rms / 1.0); full-scale sine = 0 dB.
    return 20.0 * float(np.log10(rms))


def _silence_ratio_head_tail(
    audio: np.ndarray,
    *,
    threshold_dbfs: float = -40.0,
) -> float:
    """Fraction of samples in the head/tail that are "silent".

    "Silent" = amplitude below the dBFS threshold (linear value
    derived from ``threshold_dbfs``). We only check the first and
    last 20% of the audio — a chunk that is silent in the
    *middle* but speaks at head/tail is still meaningful (and a
    different bug class that the audio gate is not designed to
    catch).

    Returns
    -------
    float
        ``[0.0, 1.0]``. ``0.0`` = no silence at head/tail.
        ``1.0`` = entire head+tail is silent.
    """
    if audio.size == 0:
        return 1.0
    n = audio.size
    window = max(1, n // 5)  # 20% of audio at each end
    head = audio[:window]
    tail = audio[-window:] if window > 0 else audio[:0]
    if head.size == 0 and tail.size == 0:
        return 0.0

    linear_th = 10.0 ** (threshold_dbfs / 20.0)
    combined = np.concatenate([head, tail]) if tail.size else head
    silent_samples = int(np.sum(np.abs(combined) <= linear_th))
    return float(silent_samples) / float(combined.size)


def check_audio_quality(
    audio: np.ndarray,
    sample_rate: int,
    estimated_duration_ms: float,
    *,
    duration_ratio_floor: float = DURATION_RATIO_FLOOR,
    silence_ratio_floor: float = SILENCE_RATIO_FLOOR,
    rms_db_floor: float = RMS_DB_FLOOR,
) -> QualityVerdict:
    """Run the three heuristics and return the verdict.

    The order is deliberate: cheap first. ``duration_ratio`` is a
    single multiplication; ``rms_low`` is one ``sqrt(mean)``; the
    silence ratio is the most expensive (two slices + a
    ``np.sum``).

    Parameters
    ----------
    audio
        ``np.ndarray`` of float32 mono samples in ``[-1.0, 1.0]``.
        Empty arrays short-circuit to :attr:`REJECT_SILENT`.
    sample_rate
        Hz. Must be ``>= _MIN_SAMPLE_RATE_HZ``; below that we
        return :attr:`PASS` (the gate refuses to police a
        misconfiguration it can't reason about).
    estimated_duration_ms
        Duration predicted by :func:`estimator.estimate_confidence`.
        Required for the ``duration_ratio`` heuristic.
    duration_ratio_floor, silence_ratio_floor, rms_db_floor
        Override knobs for the unit tests. Defaults come from the
        module-level constants.

    Returns
    -------
    QualityVerdict
        One of :attr:`PASS` / :attr:`REJECT_SILENT` /
        :attr:`REJECT_CLIPPED` / :attr:`REJECT_TRIMMED`.
        ``PASS`` is returned only when **all three** heuristics
        passed.
    """
    if audio is None or audio.size == 0:
        return QualityVerdict.REJECT_SILENT

    if sample_rate < _MIN_SAMPLE_RATE_HZ:
        # Refuse to police — sample rate is not a quality concern.
        return QualityVerdict.PASS

    # --- heuristic 1: duration_ratio ---
    if estimated_duration_ms > 0:
        actual_ms = (audio.size / float(sample_rate)) * 1000.0
        ratio = actual_ms / estimated_duration_ms
        # Symmetric in the sense we accept under-prediction too
        # (silence_ratio catches the "too short" case separately).
        if ratio > duration_ratio_floor or ratio < (1.0 / duration_ratio_floor):
            return QualityVerdict.REJECT_CLIPPED

    # --- heuristic 2: rms_low ---
    rms_db = _rms_dbfs(audio)
    if rms_db < rms_db_floor:
        return QualityVerdict.REJECT_SILENT

    # --- heuristic 3: silence_ratio ---
    silence_ratio = _silence_ratio_head_tail(audio, threshold_dbfs=rms_db_floor)
    if silence_ratio > silence_ratio_floor:
        return QualityVerdict.REJECT_TRIMMED

    return QualityVerdict.PASS