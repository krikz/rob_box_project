"""test_utterance_speaker.py -- issue #2829 / ADR-0131.

Covers the acceptance criteria listed in the issue's PR-1 task:

(a) race -- STT arrives before biometry -> caller waits for ITS OWN
    utterance, not whatever the previous one left behind;
(b) biometry never arrives -> unknown, not the previous speaker;
(c) speaker switch A -> B between two utterances -> the second utterance
    resolves to B, not A;
(d) out-of-order results (utterance 2's biometry submitted before
    utterance 1's) -> each utterance still resolves to its own result;
(e) ``compute_utterance_id`` gives the same id for the same bytes,
    regardless of which "node" (stt_node vs speaker_id_node) calls it.
"""

from __future__ import annotations

import asyncio
import time

import pytest

from rob_box_voice.core.utterance_id import compute_utterance_id
from rob_box_voice.core.utterance_speaker import UtteranceSpeakerRegistry


def _run(coro):
    return asyncio.run(coro)


class TestComputeUtteranceId:
    def test_same_bytes_same_id(self):
        pcm = b"\x01\x02\x03\x04" * 100
        # Two independent calls -- stands in for "stt_node computes it"
        # and "speaker_id_node computes it" from the same /audio/speech_audio
        # message, with zero coordination between them.
        assert compute_utterance_id(pcm) == compute_utterance_id(pcm)

    def test_different_bytes_different_id(self):
        assert compute_utterance_id(b"aaaa") != compute_utterance_id(b"bbbb")

    def test_empty_bytes_do_not_raise(self):
        # Degenerate but valid utterance -- must not crash the caller.
        assert compute_utterance_id(b"") != ""

    def test_id_is_stable_short_hex(self):
        uid = compute_utterance_id(b"some pcm bytes")
        assert len(uid) == 12
        int(uid, 16)  # raises ValueError if not hex


class TestUtteranceSpeakerRegistryRace:
    """(a) STT wins the race against biometry for THIS utterance."""

    def test_resolve_waits_for_late_submit_of_same_utterance(self):
        registry = UtteranceSpeakerRegistry(poll_interval_sec=0.01)

        async def scenario():
            resolve_task = asyncio.create_task(
                registry.resolve("utt-1", timeout_sec=1.0)
            )
            await asyncio.sleep(0.05)
            # Biometry for THIS utterance lands late, after resolve()
            # already started waiting.
            registry.submit("utt-1", {"is_known": True, "name": "Anton"})
            return await resolve_task

        result = _run(scenario())
        assert result == {"is_known": True, "name": "Anton"}

    def test_resolve_returns_immediately_if_already_submitted(self):
        registry = UtteranceSpeakerRegistry()
        registry.submit("utt-1", {"is_known": True, "name": "Anton"})
        result = _run(registry.resolve("utt-1", timeout_sec=1.0))
        assert result == {"is_known": True, "name": "Anton"}


class TestUtteranceSpeakerRegistryTimeout:
    """(b) biometry never arrives -> unknown, never the previous value."""

    def test_timeout_returns_none_not_stale_value(self):
        registry = UtteranceSpeakerRegistry(poll_interval_sec=0.01)
        # A PREVIOUS utterance's result is sitting in the registry --
        # resolving a DIFFERENT (never-submitted) id must never pick it
        # up. This is exactly the bug in issue #2829: reading "whatever
        # is there" instead of "the result for THIS id".
        registry.submit("utt-old", {"is_known": True, "name": "Anton"})

        t0 = time.monotonic()
        result = _run(registry.resolve("utt-new", timeout_sec=0.05))
        elapsed = time.monotonic() - t0

        assert result is None
        assert elapsed >= 0.05


class TestUtteranceSpeakerRegistrySwitch:
    """(c) speaker switch A -> B between two utterances."""

    def test_second_utterance_resolves_to_new_speaker(self):
        registry = UtteranceSpeakerRegistry()
        registry.submit("utt-a", {"is_known": True, "name": "Anton"})
        registry.submit("utt-b", {"is_known": True, "name": "Boris"})

        result_a = _run(registry.resolve("utt-a", timeout_sec=1.0))
        result_b = _run(registry.resolve("utt-b", timeout_sec=1.0))

        assert result_a["name"] == "Anton"
        assert result_b["name"] == "Boris"


class TestUtteranceSpeakerRegistryOutOfOrder:
    """(d) biometry for utterance 2 lands before utterance 1's."""

    def test_out_of_order_submits_resolve_to_their_own_utterance(self):
        registry = UtteranceSpeakerRegistry()
        # Utterance 2's (faster) biometry result arrives FIRST.
        registry.submit("utt-2", {"is_known": True, "name": "Boris"})
        # Utterance 1's (slower) biometry result arrives SECOND.
        registry.submit("utt-1", {"is_known": True, "name": "Anton"})

        result_1 = _run(registry.resolve("utt-1", timeout_sec=1.0))
        result_2 = _run(registry.resolve("utt-2", timeout_sec=1.0))

        assert result_1["name"] == "Anton"
        assert result_2["name"] == "Boris"


class TestUtteranceSpeakerRegistryHousekeeping:
    def test_empty_utterance_id_is_ignored(self):
        registry = UtteranceSpeakerRegistry()
        registry.submit("", {"is_known": True, "name": "Anton"})
        assert registry.peek("") is None

    def test_resolve_with_empty_utterance_id_returns_none_immediately(self):
        registry = UtteranceSpeakerRegistry()
        result = _run(registry.resolve("", timeout_sec=1.0))
        assert result is None

    def test_ring_buffer_caps_entry_count(self):
        registry = UtteranceSpeakerRegistry()
        for i in range(200):
            registry.submit(f"utt-{i}", {"is_known": False})
        # Internal cap is 64 -- this is a whitebox check on the
        # implementation detail, acceptable here since it is the whole
        # point of the eviction test.
        assert len(registry._results) <= 64  # noqa: SLF001
