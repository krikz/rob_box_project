"""Speculative-execution orchestrator for chunk-level pre-generation (ADR-0056 §3.2).

This module is the **only** async-aware component in the package.
It owns:

* the lifecycle of in-flight speculative tasks (``asyncio.Task``),
* the dictionary of completed-but-not-yet-claimed results,
* the cancellation contract (``cancel(reason)`` clears results,
  per ADR-0056 §3.5 — distinct from
  ``scheduler.SpeculativePreGenerator.cancel`` which *keeps* them),
* the per-process :class:`PreGenMetrics` counters that the
  ``/voice/tts/metrics`` topic drains.

Why ``asyncio`` here and not threads
-------------------------------------

The ``TTSNode`` already runs synthesis on a bounded
``ThreadPoolExecutor`` (``self._synthesis_executor``). Adding a
second thread pool for speculation would double the thread
budget under bursty input — exactly the bug class that the
BLK-9 / OWASP-A04 fix was designed to prevent.

Instead, we run the speculative pipeline in the asyncio loop
that ``rclpy`` already spins (``self.get_loop()`` in the caller)
and bridge the synchronous TTS providers via ``asyncio.to_thread``.
For MiniMax (the only provider whose current implementation is
already coroutine-based) we use the native coroutine directly.
The dispatch logic is encapsulated in :func:`_dispatch_synthesis`
below so the caller never has to know which branch we took.

Cancellation contract
---------------------

``cancel(reason)`` does three things:

1. Cancel every in-flight :class:`asyncio.Task` (best-effort —
   already-running TTS calls cannot be interrupted).
2. **Clear** the ``_results`` cache (this is the *key* difference
   from ``SpeculativePreGenerator`` — a cached result with a
   stale ``dialogue_id`` would be played by mistake on REPLACE
   / barge-in).
3. Bump ``cancelled_count`` for metrics.

The caller (``TTSNode.cancel_pregen``) invokes this from any of
the four sites enumerated in ADR-0056 §3.5 (dialogue switch,
``_interrupt_playback``, STOP control msg, REPLACE).
"""

from __future__ import annotations

import asyncio
import inspect
import logging
import time
from dataclasses import dataclass, field
from typing import (
    Any,
    Awaitable,
    Callable,
    Dict,
    List,
    Mapping,
    Optional,
    Sequence,
    Tuple,
)

import numpy as np

from .decision import ACCEPT_BASIS_DEFAULT, Decision, decide
from .estimator import (
    CONFIDENCE_FLOOR,
    SegmentEstimate,
    estimate_confidence,
)
from .pre_gen import DEFAULT_NEXT_PRIORITY, PreGenTask, build_pregen_task
from .quality import check_audio_quality


_LOG = logging.getLogger(__name__)


# ---------------------------------------------------------------------
# Public data classes
# ---------------------------------------------------------------------


@dataclass(frozen=True)
class PreGenResult:
    """Final outcome of one speculative chunk, ready for claim.

    Lives in :attr:`SpeculativeExecutor._results` until
    :meth:`SpeculativeExecutor.claim` pulls it out and feeds it
    into :func:`TTSNode._synthesize_and_play` as ``prebaked_audio``.
    """

    speech_id: str
    audio: np.ndarray
    sample_rate: int
    decision: Decision
    confidence: float
    basis: str
    elapsed_ms: float


@dataclass
class PreGenMetrics:
    """Counters surfaced through the ``/voice/tts/metrics`` topic.

    Plain dataclass (mutable). The executor updates counters from
    the asyncio worker that completes each speculative task; the
    publisher reads them on a low-frequency cadence. We avoid
    ``threading.Lock`` here because asyncio single-threaded
    mutation is enough — the executor never yields between
    counter bumps.
    """

    kickoffs_total: int = 0
    pregens_scheduled: int = 0
    pregens_completed: int = 0
    pregens_rejected_quality: int = 0
    pregens_rejected_confidence: int = 0
    pregens_stale: int = 0
    pregens_claimed: int = 0
    pregens_bypassed: int = 0
    cancelled_count: int = 0
    latency_chunk_to_chunk_ms_total: float = 0.0
    latency_chunk_to_chunk_ms_count: int = 0

    def as_dict(self) -> dict:
        mean_latency = (
            self.latency_chunk_to_chunk_ms_total
            / self.latency_chunk_to_chunk_ms_count
            if self.latency_chunk_to_chunk_ms_count > 0
            else 0.0
        )
        return {
            "kickoffs_total": self.kickoffs_total,
            "pregens_scheduled": self.pregens_scheduled,
            "pregens_completed": self.pregens_completed,
            "pregens_rejected_quality": self.pregens_rejected_quality,
            "pregens_rejected_confidence": self.pregens_rejected_confidence,
            "pregens_stale": self.pregens_stale,
            "pregens_claimed": self.pregens_claimed,
            "pregens_bypassed": self.pregens_bypassed,
            "cancelled_count": self.cancelled_count,
            "latency_chunk_to_chunk_ms_mean": mean_latency,
            "latency_chunk_to_chunk_ms_count": self.latency_chunk_to_chunk_ms_count,
        }


# ---------------------------------------------------------------------
# Synthesis dispatch — bridge sync / async TTS providers
# ---------------------------------------------------------------------


#: Returned by a sync ``synth_callable`` shape.
#: ``np.ndarray`` is the audio; ``sample_rate`` is in Hz.
_SynthOutput = Tuple[np.ndarray, int]


#: Type of the synth callback the executor accepts. The callback
#: takes ``PreGenTask``-derived arguments and returns
#: ``(audio, sample_rate)``. If the callable is a coroutine
#: function, the executor ``await``s it directly; if it is a
#: plain function, the executor wraps it with ``asyncio.to_thread``.
SynthCallable = Callable[..., Any]


async def _dispatch_synthesis(
    synth_callable: SynthCallable,
    *,
    ssml: str,
    text: str,
    voice: str,
    language: Optional[str],
    ssml_attributes: Mapping[str, Any],
) -> _SynthOutput:
    """Run the underlying TTS call and return ``(audio, sample_rate)``.

    The dispatch accepts the **same** kwargs that
    :meth:`TTSNode._synthesize_and_play` already uses for its
    primary path — by design, so the executor reuses the
    *exact* production providers (Yandex gRPC v3 / Silero v5 /
    MiniMax T2A v2) without a separate testing surface.
    """
    if inspect.iscoroutinefunction(synth_callable):
        # Native coroutine provider (MiniMax async path).
        result = await synth_callable(
            ssml=ssml,
            text=text,
            voice=voice,
            language=language,
            ssml_attributes=dict(ssml_attributes),
        )
    else:
        # Sync provider — bridge via the default thread pool.
        result = await asyncio.to_thread(
            synth_callable,
            ssml,
            text,
            dict(ssml_attributes),
            voice,
            language,
        )

    # Normalise the return shape. The provider chain in
    # ``_synthesize_and_play`` returns ``{"audio_np": ..., "sample_rate": ...}``
    # for MiniMax and a bare ``np.ndarray`` (with implicit
    # ``self.audio_output_sample_rate``) for Yandex/Silero.
    # Callers MUST adapt their ``synth_callable`` to one of the
    # two shapes documented here.
    if isinstance(result, Mapping):
        audio = result.get("audio_np")
        sample_rate = result.get("sample_rate")
    else:
        audio = result
        sample_rate = None
    if audio is None:
        raise RuntimeError(
            "synth_callable returned no audio (result={!r})".format(result)
        )
    if sample_rate is None:
        raise RuntimeError(
            "synth_callable returned no sample_rate; the executor "
            "needs it to compute duration_ratio. Adapt your "
            "callable to return {'audio_np': ..., 'sample_rate': ...}"
        )
    return audio, int(sample_rate)


# ---------------------------------------------------------------------
# Orchestrator
# ---------------------------------------------------------------------


class SpeculativeExecutor:
    """Owns the lifecycle of speculative chunks for one ``TTSNode``.

    Constructed once in :meth:`TTSNode.__init__` and reused for the
    entire node lifetime. The executor does **not** know about
    rclpy — it accepts a plain ``synth_callable`` that performs
    the actual TTS work, exactly like the existing ``SpeculativePreGenerator``
    does for the scheduler layer.
    """

    def __init__(
        self,
        *,
        synth_callable: SynthCallable,
        confidence_floor: float = CONFIDENCE_FLOOR,
        history_window: int = 10,
    ) -> None:
        if synth_callable is None:
            raise ValueError("synth_callable must not be None")
        self._synth = synth_callable
        self._confidence_floor = float(confidence_floor)
        self._history_window = max(1, int(history_window))

        # Active in-flight tasks, keyed by ``PreGenTask.next_speech_id``.
        self._active: Dict[str, asyncio.Task] = {}
        # Completed-but-not-yet-claimed results, keyed the same way.
        self._results: Dict[str, PreGenResult] = {}
        # Recent actual durations for the active voice (ms).
        self._actual_durations_ms: List[float] = []
        # Recent estimated durations (ms) — parallel to actuals.
        self._estimated_durations_ms: List[float] = []

        self.metrics = PreGenMetrics()
        self._cancelled = False

    # --- public API --------------------------------------------------

    def is_cancelled(self) -> bool:
        """``True`` after :meth:`cancel` ran; reset by :meth:`kickoff`."""
        return self._cancelled

    def snapshot(self) -> dict:
        """Diagnostic snapshot (no rclpy)."""
        return {
            "active": len(self._active),
            "cached": len(self._results),
            "history_size": len(self._actual_durations_ms),
            "cancelled": self._cancelled,
            "metrics": self.metrics.as_dict(),
        }

    def record_synthesis_actual(
        self,
        *,
        actual_duration_ms: float,
        estimated_duration_ms: float,
    ) -> None:
        """Feed the calibration window after each finished chunk.

        Called by :meth:`TTSNode._synthesize_and_play` once per
        chunk (canonical or pre-gen) — the executor doesn't care
        which, both contribute to confidence.
        """
        if actual_duration_ms <= 0 or estimated_duration_ms <= 0:
            return
        self._actual_durations_ms.append(float(actual_duration_ms))
        self._estimated_durations_ms.append(float(estimated_duration_ms))
        # Trim to the window.
        if len(self._actual_durations_ms) > self._history_window:
            self._actual_durations_ms = self._actual_durations_ms[
                -self._history_window:
            ]
            self._estimated_durations_ms = self._estimated_durations_ms[
                -self._history_window:
            ]

    def observe_chunk_to_chunk_latency(self, elapsed_ms: float) -> None:
        """Record wall-clock latency between consecutive chunks.

        Called by :meth:`TTSNode._synthesize_and_play` after each
        successful playback (canonical or pre-baked). The mean of
        this stream is the ``latency_chunk_to_chunk`` metric
        surfaced in DoD #2 (ADR-0056 §3.7).
        """
        if elapsed_ms <= 0:
            return
        self.metrics.latency_chunk_to_chunk_ms_total += float(elapsed_ms)
        self.metrics.latency_chunk_to_chunk_ms_count += 1

    async def kickoff(
        self,
        current_chunk: Mapping[str, Any],
        *,
        fallback_voice: Optional[str] = None,
        fallback_language: Optional[str] = None,
        text: Optional[str] = None,
        fallback_dialogue_id: Optional[str] = None,
    ) -> Optional[str]:
        """Maybe-launch one speculative task. Returns ``speech_id`` or ``None``.

        Parameters
        ----------
        current_chunk
            The JSON-decoded chunk payload (same shape
            :func:`pre_gen.build_pregen_task` accepts).
        fallback_voice, fallback_language
            Passed through to :func:`pre_gen.build_pregen_task`
            when the publisher did not specify voice/language.
        text
            Pre-extracted plain text for the **next** chunk
            (already computed by ``dialogue_callback`` for the
            metrics logger). ``None`` is allowed — the executor
            extracts it from the SSML on demand.
        fallback_dialogue_id
            Used to set :attr:`PreGenTask.dialogue_id` if the
            caller didn't provide one.

        Returns
        -------
        Optional[str]
            ``speech_id`` of the launched speculative task, or
            ``None`` if no task was launched (publisher opt-out,
            last chunk, low confidence, etc.). The ``speech_id``
            lets the caller correlate :meth:`kickoff` with the
            eventual :meth:`claim`.
        """
        self.metrics.kickoffs_total += 1
        self._cancelled = False

        task = build_pregen_task(
            current_chunk,
            fallback_voice=fallback_voice,
            fallback_language=fallback_language,
        )
        if task is None:
            return None

        if task.dialogue_id is None and fallback_dialogue_id is not None:
            # Re-build with explicit dialogue_id by passing the
            # current_chunk mapping — the helper already filled
            # ``dialogue_id`` from ``current_chunk["dialogue_id"]``
            # when available, but the caller may have a stronger
            # source (e.g. ``self.current_dialogue_id``).
            object.__setattr__(task, "dialogue_id", fallback_dialogue_id)

        estimate: SegmentEstimate = estimate_confidence(
            task,
            self._actual_durations_ms,
            recent_estimated_durations_ms=self._estimated_durations_ms,
        )
        if estimate.confidence < self._confidence_floor:
            self.metrics.pregens_rejected_confidence += 1
            _LOG.debug(
                "pregenerate rejected: confidence=%.2f < floor=%.2f "
                "(basis=%s, speech_id=%s)",
                estimate.confidence,
                self._confidence_floor,
                estimate.basis,
                task.next_speech_id,
            )
            return None

        # Already running for this speech_id? Skip (idempotency
        # under rapid resubmits).
        if task.next_speech_id in self._active:
            return task.next_speech_id
        # Already cached? Don't re-run.
        if task.next_speech_id in self._results:
            return task.next_speech_id

        coro = self._run_pregen_one(
            task=task,
            confidence=estimate.confidence,
            basis=estimate.basis,
            text=text,
        )
        asyncio_task = asyncio.create_task(coro)
        self._active[task.next_speech_id] = asyncio_task
        self.metrics.pregens_scheduled += 1
        return task.next_speech_id

    async def cancel(self, reason: str) -> int:
        """Cancel every in-flight task and clear the cache.

        Returns the count of cancelled tasks (for metrics).
        Per ADR-0056 §3.5 the cache is **cleared** — distinct from
        ``SpeculativePreGenerator.cancel`` which keeps cached
        results because the scheduler layer's invariant is
        different.
        """
        if not self._active and not self._results:
            self._cancelled = True
            return 0

        cancelled = 0
        for speech_id, t in list(self._active.items()):
            if not t.done():
                t.cancel()
                cancelled += 1
            self._active.pop(speech_id, None)
        # Wait for cancellation to settle — best-effort.
        if self._active:
            await asyncio.gather(
                *self._active.values(),
                return_exceptions=True,
            )
            self._active.clear()
        self._results.clear()
        self.metrics.cancelled_count += cancelled
        self._cancelled = True
        _LOG.info(
            "pregenerate cancelled (reason=%s, in_flight=%d)",
            reason,
            cancelled,
        )
        return cancelled

    def claim(self, speech_id: str) -> Optional[PreGenResult]:
        """Ownership-transfer: pop and return the cached result.

        Returns ``None`` if no cached result exists for
        ``speech_id``. The caller (``TTSNode._synthesize_and_play``)
        treats ``None`` as "fall back to the canonical path".
        """
        result = self._results.pop(speech_id, None)
        if result is not None:
            self.metrics.pregens_claimed += 1
        return result

    async def shutdown(self) -> None:
        """Cancel everything (for node teardown). Idempotent."""
        await self.cancel(reason="shutdown")
        self._actual_durations_ms.clear()
        self._estimated_durations_ms.clear()

    # --- internals ---------------------------------------------------

    async def _run_pregen_one(
        self,
        *,
        task: PreGenTask,
        confidence: float,
        basis: str,
        text: Optional[str],
    ) -> None:
        """Run one speculative chunk through synth → quality → decision.

        Errors here are *expected* (provider flake, network blip,
        cancellation) and MUST NOT propagate — they are logged
        and treated as if the pre-gen never happened.
        """
        speech_id = task.next_speech_id
        start = time.monotonic()
        try:
            if text is None:
                text = _extract_text_from_ssml(task.next_ssml)

            audio, sample_rate = await _dispatch_synthesis(
                self._synth,
                ssml=task.next_ssml,
                text=text,
                voice=task.voice,
                language=task.language,
                ssml_attributes=task.ssml_attributes,
            )

            # Estimate duration from char-rate heuristic (cheap
            # proxy; the executor doesn't have access to the
            # scheduler-level :class:`SegmentEstimate`).
            estimated_ms = max(
                1.0,
                float(len(text)) * 60.0,  # 60 ms / char — typical Russian
            )

            verdict = check_audio_quality(
                audio=audio,
                sample_rate=sample_rate,
                estimated_duration_ms=estimated_ms,
            )
            d = decide(verdict, confidence, confidence_floor=self._confidence_floor)

            if d is Decision.ACCEPT:
                elapsed_ms = (time.monotonic() - start) * 1000.0
                self._results[speech_id] = PreGenResult(
                    speech_id=speech_id,
                    audio=audio,
                    sample_rate=sample_rate,
                    decision=d,
                    confidence=confidence,
                    basis=basis or ACCEPT_BASIS_DEFAULT,
                    elapsed_ms=elapsed_ms,
                )
                self.metrics.pregens_completed += 1
                # Also feed the calibration histogram so future
                # confidence estimates improve.
                actual_ms = (audio.size / float(sample_rate)) * 1000.0
                self.record_synthesis_actual(
                    actual_duration_ms=actual_ms,
                    estimated_duration_ms=estimated_ms,
                )
            else:
                self.metrics.pregens_rejected_quality += 1
                _LOG.debug(
                    "pregenerate rejected by quality gate "
                    "(verdict=%s, confidence=%.2f, speech_id=%s)",
                    verdict,
                    confidence,
                    speech_id,
                )
        except asyncio.CancelledError:
            # Expected on dialogue switch / barge-in. Don't
            # count as quality rejection.
            raise
        except Exception as exc:  # noqa: BLE001
            self.metrics.pregens_rejected_quality += 1
            _LOG.warning(
                "pregenerate failed (speech_id=%s, err=%r) — "
                "treating as no-pregen for this chunk",
                speech_id,
                exc,
            )
        finally:
            self._active.pop(speech_id, None)


def _extract_text_from_ssml(ssml: str) -> str:
    """Strip XML tags from SSML — best-effort, no lxml dependency."""
    import re

    return re.sub(r"<[^>]+>", "", ssml).strip()