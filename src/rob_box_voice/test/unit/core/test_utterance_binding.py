"""test_utterance_binding.py -- issue #2862, ``UtteranceIdBinder``.

Text -> utterance_id join between ``/voice/stt/utterance`` and
``/voice/stt/result`` that does not depend on delivery order.
"""

from __future__ import annotations

import threading

from rob_box_voice.core.utterance_binding import UtteranceIdBinder


class _Clock:
    def __init__(self) -> None:
        self.t = 1000.0

    def __call__(self) -> float:
        return self.t


def test_id_first_then_claim():
    b = UtteranceIdBinder()
    b.offer("привет", "u1")
    assert b.claim("привет", 0.0) == "u1"
    assert b.claim("привет", 0.0) is None


def test_claim_waits_for_late_id():
    b = UtteranceIdBinder()
    t = threading.Timer(0.05, b.offer, args=("привет", "u1"))
    t.start()
    assert b.claim("привет", 1.0) == "u1"
    t.join()


def test_timeout_tombstones_late_id():
    b = UtteranceIdBinder()
    assert b.claim("привет", 0.01) is None
    b.offer("привет", "late")  # dropped: its claim already gave up
    b.offer("привет", "u2")  # the same text said again, id first
    assert b.claim("привет", 0.0) == "u2"


def test_different_text_never_gets_id():
    b = UtteranceIdBinder()
    b.offer("первая", "u1")
    assert b.claim("вторая", 0.0) is None
    assert b.claim("первая", 0.0) == "u1"


def test_same_text_twice_fifo():
    b = UtteranceIdBinder()
    b.offer("да", "u1")
    b.offer("да", "u2")
    assert b.claim("да", 0.0) == "u1"
    assert b.claim("да", 0.0) == "u2"


def test_whitespace_normalised():
    b = UtteranceIdBinder()
    b.offer(" привет \n", "u1")
    assert b.claim("привет", 0.0) == "u1"


def test_empty_inputs_ignored():
    b = UtteranceIdBinder()
    b.offer("", "u1")
    b.offer("привет", "")
    assert b.claim("", 0.0) is None
    assert b.claim("привет", 0.0) is None


def test_ttl_evicts_unclaimed_ids_and_tombstones():
    clock = _Clock()
    b = UtteranceIdBinder(ttl_sec=30.0, clock=clock)
    b.offer("старое", "u1")
    assert b.claim("сирота", 0.0) is None
    clock.t += 31.0
    b.offer("новое", "u2")  # triggers pruning
    assert b.claim("старое", 0.0) is None
    b.offer("сирота", "u3")  # tombstone expired -> id kept
    assert b.claim("сирота", 0.0) == "u3"


def test_max_keys_cap():
    b = UtteranceIdBinder(max_keys=2)
    for i in range(5):
        b.offer(f"t{i}", f"u{i}")
    assert b.claim("t0", 0.0) is None
    assert b.claim("t4", 0.0) == "u4"
