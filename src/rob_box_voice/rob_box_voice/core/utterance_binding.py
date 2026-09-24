"""utterance_binding.py — bind ``utterance_id`` to the TEXT of its phrase
(issue #2862, ADR-0131 §2.3a).

``stt_node`` publishes a phrase on two topics: ``/voice/stt/utterance``
(JSON ``{"utterance_id", "text"}``) and ``/voice/stt/result`` (plain text,
contract frozen — telegram/perception/GUI/harness read it). DDS does not
order delivery across topics and ``dialogue_node`` runs both callbacks in a
``ReentrantCallbackGroup`` on a ``MultiThreadedExecutor``, so the id may be
processed AFTER the text. The previous single "pending slot" then handed
the turn ``None`` or — worse — the id of the PREVIOUS phrase (a backlog
phrase whose id landed after its own text), see issue #2862 evidence.

:class:`UtteranceIdBinder` keys ids by phrase text (same idea as
``_speaker_by_text``, #1077) instead of arrival order:

* id arrives first — stored under its text, :meth:`claim` returns it at once;
* text arrives first — :meth:`claim` waits (bounded) for the id of THIS text;
* id never arrives in time — :meth:`claim` returns ``None`` and leaves a
  tombstone so the late id is dropped on arrival instead of lingering;
* an id is only ever returned to a claim with the SAME text, so it can never
  be attached to a different (next) phrase.

Pure Python, no ROS — unit-tested in ``test/unit/core``.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Callable, Deque, Dict, Optional, Tuple

# Unclaimed ids / tombstones older than this are evicted (memory cap only;
# correctness comes from the text match, not from the TTL).
_ENTRY_TTL_SEC = 30.0
# Hard cap on distinct texts held; normally 0-1 in flight.
_MAX_KEYS = 64


def _key(text: str) -> str:
    return (text or "").strip()


class UtteranceIdBinder:
    """Thread-safe text -> ``utterance_id`` join between two ROS topics.

    :meth:`offer` is called from the ``/voice/stt/utterance`` callback,
    :meth:`claim` from the ``/voice/stt/result`` callback — different
    executor threads, hence the ``threading.Condition``.
    """

    def __init__(
        self,
        ttl_sec: float = _ENTRY_TTL_SEC,
        max_keys: int = _MAX_KEYS,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._ttl = float(ttl_sec)
        self._max_keys = int(max_keys)
        self._clock = clock
        self._cond = threading.Condition()
        # text -> FIFO of (utterance_id, stored_at); FIFO covers the same
        # text said twice in a row.
        self._ids: Dict[str, Deque[Tuple[str, float]]] = {}
        # text -> FIFO of give-up timestamps: claims that timed out and
        # whose id, if it ever arrives, must be discarded.
        self._orphans: Dict[str, Deque[float]] = {}

    def offer(self, text: str, utterance_id: str) -> None:
        """Store ``utterance_id`` for the phrase ``text`` (or drop it if a
        claim for that text already gave up)."""
        key = _key(text)
        if not key or not utterance_id:
            return
        with self._cond:
            now = self._clock()
            self._prune(now)
            tomb = self._orphans.get(key)
            if tomb:
                tomb.popleft()
                if not tomb:
                    del self._orphans[key]
                return
            self._ids.setdefault(key, deque()).append((str(utterance_id), now))
            self._cond.notify_all()

    def claim(self, text: str, timeout_sec: float) -> Optional[str]:
        """Return (and consume) the id published for ``text``.

        Waits up to ``timeout_sec`` if the id has not arrived yet. On
        timeout returns ``None`` and tombstones the text so the late id
        cannot be claimed by a later phrase.
        """
        key = _key(text)
        if not key:
            return None
        deadline = self._clock() + max(0.0, float(timeout_sec))
        with self._cond:
            while True:
                uid = self._take(key)
                if uid is not None:
                    return uid
                remaining = deadline - self._clock()
                if remaining <= 0:
                    self._orphans.setdefault(key, deque()).append(self._clock())
                    return None
                self._cond.wait(remaining)

    def _take(self, key: str) -> Optional[str]:
        queue = self._ids.get(key)
        if not queue:
            return None
        uid, _ = queue.popleft()
        if not queue:
            del self._ids[key]
        return uid

    def _prune(self, now: float) -> None:
        for table in (self._ids, self._orphans):
            for key in list(table):
                queue = table[key]
                while queue and now - _stamp(queue[0]) > self._ttl:
                    queue.popleft()
                if not queue:
                    del table[key]
            while len(table) > self._max_keys:
                del table[next(iter(table))]


def _stamp(item) -> float:
    return item[1] if isinstance(item, tuple) else item
