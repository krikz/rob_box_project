"""utterance_speaker.py — single source of truth for "who said THIS phrase"
(issue #2829, ADR-0131).

Before this module, ``dialogue_node`` decided "who is speaking" by reading
whatever the *last* ``/voice/speaker/result`` message happened to leave in
a shared mutable field (``_current_speaker``), after a blind
``asyncio.sleep(0.30)`` meant to let biometry "usually" finish first. STT
latency (1-4s) and biometry latency (0.6-1.9s, occasionally ~50s right
after startup) are not ordered relative to each other, so whenever STT won
the race the robot attributed the CURRENT phrase to whoever the PREVIOUS
phrase's biometry result belonged to.

:class:`UtteranceSpeakerRegistry` fixes this by keying every biometry
result by ``utterance_id`` (see ``utterance_id.py``) and making callers
``await`` the result for the SPECIFIC utterance they care about, with a
timeout. Late or out-of-order results are joined by id, not by arrival
order:

* biometry result arrives before the caller asks -- served instantly from
  the small ring buffer;
* biometry result arrives after the caller starts waiting -- the waiting
  coroutine is woken as soon as ``submit()`` stores it;
* biometry never arrives within ``timeout`` -- caller gets ``None``
  (treated as "unknown"), never a stale different utterance's result.

This is the ONLY module allowed to answer "who said utterance X" from
biometry. Other historical signals (``speaker_context``/``speaker_tag``
from ``/voice/stt/speaker``, ``_speaker_by_text``, ``_speaker_tracker``)
are diagnostic-only after this change -- see ADR-0131 §"Источники: было →
стало" for the per-source disposition table.
"""

from __future__ import annotations

import asyncio
import threading
import time
from typing import Any, Dict, Optional

# How long a resolved-or-unclaimed entry survives in the ring buffer before
# being evicted. Generous relative to the expected 0.6-1.9s biometry
# latency (issue #2829) so a slightly late resolve() poll still finds it,
# but short enough that a stale entry can never be mistaken for a *later*
# utterance that happens to reuse... it cannot reuse an id (hash of
# distinct audio), so this bound only exists to cap memory, not to avoid
# collisions.
_ENTRY_TTL_SEC = 30.0
# Ring buffer cap -- generous multiple of plausible in-flight utterances
# (normally 1, at most a handful during rapid back-and-forth barge-in).
_MAX_ENTRIES = 64


class UtteranceSpeakerRegistry:
    """Join point for "which biometry result belongs to which utterance".

    Thread-safety: :meth:`submit` is called from the ROS subscription
    callback thread (``/voice/speaker/result``). :meth:`resolve` is
    awaited from the dialogue node's asyncio task. Both take the same
    ``threading.Lock``; the async side polls at a short interval instead
    of blocking the lock across an await (keeps the critical section
    trivial and avoids needing a cross-thread asyncio primitive).
    """

    def __init__(self, poll_interval_sec: float = 0.02) -> None:
        self._lock = threading.Lock()
        self._results: Dict[str, Dict[str, Any]] = {}
        self._timestamps: Dict[str, float] = {}
        self._poll_interval_sec = poll_interval_sec

    def submit(self, utterance_id: str, result: Dict[str, Any]) -> None:
        """Record the biometry result for ``utterance_id``.

        Called once per utterance from the ``/voice/speaker/result``
        subscription callback (any thread). Overwrites a previous entry
        for the same id only in the (impossible under sha1) case of a
        collision -- never a design path we rely on.
        """
        if not utterance_id:
            return
        with self._lock:
            self._results[utterance_id] = dict(result)
            self._timestamps[utterance_id] = time.monotonic()
            self._evict_stale_locked()

    def _evict_stale_locked(self) -> None:
        """Drop entries older than ``_ENTRY_TTL_SEC`` or beyond the cap.

        Must be called with ``self._lock`` held.
        """
        now = time.monotonic()
        stale = [
            uid
            for uid, ts in self._timestamps.items()
            if now - ts > _ENTRY_TTL_SEC
        ]
        for uid in stale:
            self._timestamps.pop(uid, None)
            self._results.pop(uid, None)
        if len(self._timestamps) > _MAX_ENTRIES:
            # Drop oldest first -- FIFO overflow, not LRU: an entry nobody
            # resolved in 64 utterances' worth of time is not coming back.
            oldest = sorted(self._timestamps.items(), key=lambda kv: kv[1])
            for uid, _ts in oldest[: len(self._timestamps) - _MAX_ENTRIES]:
                self._timestamps.pop(uid, None)
                self._results.pop(uid, None)

    async def resolve(
        self, utterance_id: str, timeout_sec: float
    ) -> Optional[Dict[str, Any]]:
        """Wait up to ``timeout_sec`` for the biometry result of one utterance.

        Returns the stored result dict (whatever :meth:`submit` was given
        -- typically the parsed ``/voice/speaker/result`` payload), or
        ``None`` if it never showed up in time. ``None`` MUST be treated
        as "unknown speaker for this phrase", never as "keep whatever we
        had before" -- that silent carry-over is exactly the bug this
        module exists to remove.
        """
        if not utterance_id:
            return None
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while True:
            with self._lock:
                result = self._results.get(utterance_id)
            if result is not None:
                return result
            if time.monotonic() >= deadline:
                return None
            await asyncio.sleep(self._poll_interval_sec)

    def peek(self, utterance_id: str) -> Optional[Dict[str, Any]]:
        """Non-blocking lookup -- for diagnostics/tests only, not the
        turn-taking hot path (use :meth:`resolve` there)."""
        with self._lock:
            return self._results.get(utterance_id)
