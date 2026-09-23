"""test_issue_2829_apply_speaker_identity_ssot.py

Issue #2829 / ADR-0131 -- ``dialogue_node._apply_speaker_identity`` must
join STT and voice-biometry by ``utterance_id`` instead of reading
whatever ``_current_speaker`` happens to hold after a blind
``asyncio.sleep(0.30)``.

Same node-construction trick as
``test_issue_2809_low_confidence_speaker_does_not_leak_name.py``
(``object.__new__`` -- no ROS2 spin needed).
"""

from __future__ import annotations

import asyncio
import sys
import threading
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

_audio_common = types.ModuleType("audio_common_msgs")
_audio_common_msg = types.ModuleType("audio_common_msgs.msg")
_audio_common_msg.AudioData = MagicMock
sys.modules.setdefault("audio_common_msgs", _audio_common)
sys.modules.setdefault("audio_common_msgs.msg", _audio_common_msg)

for _hw in ("pyaudio", "usb", "usb.core", "usb.util", "sounddevice"):
    sys.modules.setdefault(_hw, MagicMock())

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from rob_box_voice.dialogue_node import DialogueNode  # noqa: E402
from rob_box_voice.core.utterance_speaker import UtteranceSpeakerRegistry  # noqa: E402


def _run(coro):
    return asyncio.run(coro)


@pytest.fixture()
def node():
    n = object.__new__(DialogueNode)
    n.get_logger = MagicMock(return_value=MagicMock())
    n._speaker_lock = threading.Lock()
    n._publish_speaker_observation = MagicMock()
    n._utterance_speaker = UtteranceSpeakerRegistry(poll_interval_sec=0.005)
    n._speaker_resolve_timeout_sec = 0.2
    n._current_speaker = {"is_known": False}
    return n


class TestRaceSttBeforeBiometry:
    """(a) issue #2829 §1: STT wins the race -- must wait for THIS
    utterance's biometry, not read the previous speaker's leftover
    value that happens to sit in ``_current_speaker``."""

    def test_waits_for_its_own_utterance_not_previous_speakers_value(self, node):
        # Stale value from the PREVIOUS phrase, still sitting in
        # _current_speaker when STT for the NEXT phrase comes in first.
        node._current_speaker = {
            "is_known": True,
            "speaker_id": "anton-1",
            "name": "Anton",
            "confidence": 0.93,
        }

        async def scenario():
            task = asyncio.create_task(
                node._apply_speaker_identity(
                    "privet eto boris", speaker_context=None, utterance_id="utt-2"
                )
            )
            await asyncio.sleep(0.02)
            # Biometry for THIS (new) utterance lands a bit late, and it
            # is a DIFFERENT person than what _current_speaker held.
            node._utterance_speaker.submit(
                "utt-2",
                {
                    "is_known": True,
                    "speaker_id": "boris-1",
                    "name": "Boris",
                    "confidence": 0.88,
                },
            )
            return await task

        result = _run(scenario())
        assert "[Spkr:Boris]" in result
        assert "Anton" not in result


class TestTimeoutIsHonestUnknown:
    """(b) issue #2829 §1: biometry never arrives -> unknown, not the
    previous speaker."""

    def test_timeout_yields_unknown_not_stale_previous_speaker(self, node):
        node._current_speaker = {
            "is_known": True,
            "speaker_id": "anton-1",
            "name": "Anton",
            "confidence": 0.93,
        }

        result = _run(
            node._apply_speaker_identity(
                "kto-to novyj govorit", speaker_context=None, utterance_id="utt-never"
            )
        )

        assert "Anton" not in result
        assert "[Speaker:unknown]" in result
        assert node._current_speaker == {"is_known": False}


class TestSpeakerSwitchAcrossUtterances:
    """(c) A -> B between two consecutive utterances."""

    def test_two_utterances_resolve_to_their_own_speaker(self, node):
        node._utterance_speaker.submit(
            "utt-a",
            {"is_known": True, "speaker_id": "a", "name": "Anton", "confidence": 0.9},
        )
        node._utterance_speaker.submit(
            "utt-b",
            {"is_known": True, "speaker_id": "b", "name": "Boris", "confidence": 0.9},
        )

        result_a = _run(
            node._apply_speaker_identity("fraza a", speaker_context=None, utterance_id="utt-a")
        )
        result_b = _run(
            node._apply_speaker_identity("fraza b", speaker_context=None, utterance_id="utt-b")
        )

        assert "[Spkr:Anton]" in result_a
        assert "[Spkr:Boris]" in result_b


class TestOutOfOrderResults:
    """(d) biometry for utterance 2 submitted before utterance 1's."""

    def test_each_utterance_gets_its_own_result_regardless_of_arrival_order(self, node):
        node._utterance_speaker.submit(
            "utt-2",
            {"is_known": True, "speaker_id": "b", "name": "Boris", "confidence": 0.9},
        )
        node._utterance_speaker.submit(
            "utt-1",
            {"is_known": True, "speaker_id": "a", "name": "Anton", "confidence": 0.9},
        )

        result_1 = _run(
            node._apply_speaker_identity("fraza 1", speaker_context=None, utterance_id="utt-1")
        )
        result_2 = _run(
            node._apply_speaker_identity("fraza 2", speaker_context=None, utterance_id="utt-2")
        )

        assert "[Spkr:Anton]" in result_1
        assert "[Spkr:Boris]" in result_2


class TestNoUtteranceIdFallsBackToLegacyRead:
    """Synthetic/retry turns (babble-retry, DJ auto, ...) carry no fresh
    audio utterance -- ``utterance_id=None`` must skip resolve() entirely
    and just read the last resolved snapshot, exactly like before this
    change (no new race to protect against, nothing to wait for)."""

    def test_none_utterance_id_reads_current_speaker_without_waiting(self, node):
        node._current_speaker = {
            "is_known": True,
            "speaker_id": "anton-1",
            "name": "Anton",
            "confidence": 0.93,
        }

        result = _run(
            node._apply_speaker_identity("prodolzhenie razgovora", speaker_context=None)
        )

        assert "[Spkr:Anton]" in result
