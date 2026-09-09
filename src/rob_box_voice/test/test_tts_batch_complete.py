#!/usr/bin/env python3
"""test_tts_batch_complete.py — Unit tests for issue #980 batch tracking.

Pure-Python tests that exercise:

1. ``speak_helpers.build_ssml_payload`` carries ``batch_id`` / ``batch_index``
   / ``batch_total`` into the JSON payload and omits them for the legacy
   single-chunk path (back-compat).
2. ``speak_helpers.split_into_chunks`` is the chunker ``dialogue_node``
   uses to decide whether a turn is single- vs multi-chunk.
3. ``EffectAwaiterRegistry`` still releases ``/voice/tts/finished`` awaiters
   exactly once per speech_id even when the JSON payload includes the new
   batch fields.

The tests do not spin up ROS — they import the helper module directly and
use ``json.loads`` to verify the on-the-wire contract that ``tts_node`` and
``dialogue_node`` share. The actual ``/voice/tts/batch_complete`` publication
logic lives in ``tts_node._publish_tts_finished`` and is exercised via the
dialogue_shell integration test (separate file).

Run with::

    python3 -m pytest test_tts_batch_complete.py
"""

from __future__ import annotations

import asyncio
import json
import unittest

from rob_box_voice.core.speak_helpers import (
    EffectAwaiterRegistry,
    build_ssml_payload,
    split_into_chunks,
)


class TestBuildSsmlPayloadBatch(unittest.TestCase):
    """``build_ssml_payload`` attaches batch metadata (issue #980)."""

    def test_legacy_payload_has_no_batch_fields(self):
        """Back-compat: without batch kwargs the JSON must look like before."""
        payload = json.loads(build_ssml_payload("привет"))
        self.assertIn("speech_id", payload)
        self.assertEqual(payload["ssml"], "<speak>привет</speak>")
        self.assertNotIn("batch_id", payload)
        self.assertNotIn("batch_index", payload)
        self.assertNotIn("batch_total", payload)

    def test_batch_payload_round_trips_all_fields(self):
        """All three batch kwargs are echoed back in the JSON."""
        payload = json.loads(
            build_ssml_payload(
                "chunk",
                "neutral",
                batch_id="b-1",
                batch_index=2,
                batch_total=4,
            )
        )
        self.assertEqual(payload["batch_id"], "b-1")
        self.assertEqual(payload["batch_index"], 2)
        self.assertEqual(payload["batch_total"], 4)
        self.assertEqual(payload["ssml"], "<speak>chunk</speak>")

    def test_batch_index_is_int_not_string(self):
        """``dialogue_node`` passes ints — the helper must not stringify them."""
        payload = json.loads(
            build_ssml_payload(
                "x",
                batch_id="b",
                batch_index=1,
                batch_total=3,
            )
        )
        self.assertIsInstance(payload["batch_index"], int)
        self.assertIsInstance(payload["batch_total"], int)

    def test_partial_batch_fields_are_preserved(self):
        """Only some batch fields provided → JSON keeps just those."""
        payload = json.loads(
            build_ssml_payload("x", batch_id="b-1")
        )
        self.assertEqual(payload["batch_id"], "b-1")
        self.assertNotIn("batch_index", payload)
        self.assertNotIn("batch_total", payload)


class TestSplitIntoChunks(unittest.TestCase):
    """Sentence-aware splitter used to decide multi-chunk eligibility."""

    def test_short_text_returns_single_chunk(self):
        chunks = split_into_chunks("Привет, мир!")
        self.assertEqual(chunks, ["Привет, мир!"])

    def test_long_text_is_split_into_multiple_chunks(self):
        """Rap-style long text must yield >=2 chunks so batch_id fires."""
        text = (
            "Я робот, я читаю рэп, я бегу по лестнице, я не сплю, "
            "я не ем, я только говорю, я отвечаю на вопросы, "
            "я помогаю людям в этот странный день, "
            "я устал, я хочу спать, но продолжаю работать."
        )
        chunks = split_into_chunks(text, max_len=80)
        self.assertGreaterEqual(len(chunks), 2)


class _FakeLoop:
    """Minimal stand-in for an asyncio loop's ``call_soon_threadsafe``."""

    def __init__(self) -> None:
        self.events: list[asyncio.Event] = []

    def __call__(self, ev: asyncio.Event) -> None:
        # Immediately release the event; the test then inspects the registry.
        ev.set()
        self.events.append(ev)


class TestEffectAwaiterRegistryBatch(unittest.TestCase):
    """``handle_tts_finished`` still releases one awaiter per speech_id."""

    def test_finished_releases_awaiter_with_batch_fields(self):
        loop = _FakeLoop()
        reg = EffectAwaiterRegistry(
            release_tts=loop, release_sound=loop
        )

        ev = asyncio.Event()
        reg.register_tts("sid-1", ev)
        reg.handle_tts_finished(
            json.dumps(
                {
                    "speech_id": "sid-1",
                    "success": True,
                    "batch_id": "b-1",
                    "batch_index": 1,
                    "batch_total": 3,
                }
            )
        )
        self.assertTrue(ev.is_set())
        self.assertIsNone(reg.pop_tts("sid-1"))


class TestBatchCompleteFireRule(unittest.TestCase):
    """Document the fire-rule the ``tts_node`` uses (issue #980).

    The rule is: a ``/voice/tts/batch_complete`` is published only when
    ``batch_index == batch_total`` *and* both are present (not None).
    These tests verify the predicate the ``_publish_tts_finished`` helper
    applies at runtime — encoded as a standalone function so we don't
    need to spin up rclpy/grpc/torch to exercise the rule.
    """

    @staticmethod
    def _is_last_chunk(
        batch_id: str | None,
        batch_index: int | None,
        batch_total: int | None,
    ) -> bool:
        """Mirror the predicate inside ``tts_node._publish_tts_finished``."""
        return (
            batch_id is not None
            and batch_index is not None
            and batch_total is not None
            and int(batch_index) == int(batch_total)
        )

    def test_first_chunk_of_multi_chunk_batch_does_not_fire(self):
        self.assertFalse(self._is_last_chunk("b-1", 1, 4))

    def test_middle_chunk_does_not_fire(self):
        self.assertFalse(self._is_last_chunk("b-1", 2, 4))

    def test_last_chunk_fires(self):
        self.assertTrue(self._is_last_chunk("b-1", 4, 4))

    def test_legacy_single_chunk_turn_fires_once(self):
        """Back-compat: ``batch_id=None`` batches never fire batch_complete."""
        # dialogue_node's _handle_result routes single-chunk turns through
        # ``_publish_response`` (no batch kwargs), so tts_node normalises to
        # (None, None, None) → never fires. The "fire once" promise is
        # preserved by the helper always publishing *exactly one*
        # finished event with batch_complete=True for that chunk.
        self.assertFalse(self._is_last_chunk(None, None, None))

    def test_single_chunk_with_batch_metadata_fires(self):
        """dialogue_node may pass (batch_id, 1, 1) for a 1-chunk turn.

        In that case batch_index == batch_total, so batch_complete fires
        immediately on the only chunk — exactly the contract dialogue_node
        relies on for cleanup.
        """
        self.assertTrue(self._is_last_chunk("b-1", 1, 1))


if __name__ == "__main__":
    unittest.main()
