"""test_issue_2829_identity_ack_speaker_gate.py

Issue #2829 (ADR-0131), follow-up requested by the coordinator after
PR #2830 (#2828) and PR #2831 (#2809) landed on develop:

Both the #2828 identity-ack question ("вы разные люди или это ты?")
and the #2809 tentative-speaker question read "the first live reply"
as the answer, REGARDLESS of who said it (``IdentityAckQuestion.consume``
docstring: "снимается на ПЕРВОЙ же реплике после вопроса"). Now that
dialogue_node has a real single source of truth for "who said this
reply" (``UtteranceSpeakerRegistry`` + ``_resolve_speaker_for_utterance``),
a "да" from an unrelated third voice (or from an unknown speaker) must
NOT be able to confirm someone else's identity merge.

This file covers the #2828 path specifically
(``_resolve_identity_ack_answer`` / ``_reply_speaker_matches_identity_plan``).
The #2809 tentative-speaker path does not need a separate gate: its
session state is keyed by the CURRENT reply's own resolved
``speaker_id`` (see ``_handle_tentative_speaker`` -- ``full_sid`` comes
from ``sp``, which ``_apply_speaker_identity`` has already resolved for
THIS utterance before calling it), so it is already correctly scoped to
whoever is speaking now -- no extra gate needed there.

Same ``object.__new__`` node-construction trick as
``test_issue_2828_identity_question_collision.py``.
"""

from __future__ import annotations

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

from rob_box_voice.core.identity_ack import identity_ack_plan  # noqa: E402
from rob_box_voice.dialogue_node import DialogueNode  # noqa: E402

BORIS_ID = "f65d3050-0000-0000-0000-000000000000"
SASHA_ID = "5d99f474-0000-0000-0000-000000000000"
STRANGER_ID = "aaaaaaaa-0000-0000-0000-000000000000"

CONFLICT_ACK = {
    "event": "registered",
    "name": "Борис",
    "speaker_id": BORIS_ID,
    "reused_profile": False,
    "voice_conflict": {"name": "Саша", "speaker_id": SASHA_ID, "score": 0.907},
}


def _asked_node(current_speaker: dict):
    """Node with the #2828 question already armed (bypassing
    ``_ask_identity_if_ambiguous`` -- that plumbing is already covered by
    ``test_issue_2828_identity_question_collision.py``; here we only
    care about ``_resolve_identity_ack_answer``'s speaker gate), and a
    given resolved ``_current_speaker`` (as if
    ``_resolve_speaker_for_utterance`` for THIS reply's utterance_id had
    already run)."""
    n = object.__new__(DialogueNode)
    n.get_logger = MagicMock(return_value=MagicMock())
    n._speaker_lock = threading.Lock()
    n._speaker_merge_pub = MagicMock()
    n._current_speaker = current_speaker
    plan = identity_ack_plan(CONFLICT_ACK, "Вы разные люди или это ты?")
    n._identity_ack_state().arm(plan)
    return n


class TestUnknownReplyDoesNotConfirmMerge:
    def test_unknown_speaker_saying_yes_does_not_merge(self):
        n = _asked_node({"is_known": False})

        n._resolve_identity_ack_answer("да, это я")

        n._speaker_merge_pub.publish.assert_not_called()

    def test_third_party_speaker_saying_yes_does_not_merge(self):
        """A completely unrelated known voice answers "yes" -- must not
        be able to confirm someone ELSE's identity merge."""
        n = _asked_node(
            {"is_known": True, "speaker_id": STRANGER_ID, "name": "Кто-то"}
        )

        n._resolve_identity_ack_answer("да, это я")

        n._speaker_merge_pub.publish.assert_not_called()


class TestMatchingReplySpeakerStillMerges:
    def test_new_id_speaker_saying_yes_merges(self):
        """The person being asked about (new_id == Boris) is the one
        answering -- unchanged happy path from PR #2830."""
        n = _asked_node(
            {"is_known": True, "speaker_id": BORIS_ID, "name": "Борис"}
        )

        n._resolve_identity_ack_answer("да, это я")

        n._speaker_merge_pub.publish.assert_called_once()

    def test_known_id_speaker_saying_yes_merges(self):
        """The OTHER profile in the question (known_id == Sasha) answers
        -- also a legitimate confirmation, e.g. Sasha grabbing the mic
        to say "yes, that's me under another name"."""
        n = _asked_node(
            {"is_known": True, "speaker_id": SASHA_ID, "name": "Саша"}
        )

        n._resolve_identity_ack_answer("да, это я")

        n._speaker_merge_pub.publish.assert_called_once()


class TestReplySpeakerMatchesIdentityPlanHelper:
    def _node(self, current_speaker):
        n = object.__new__(DialogueNode)
        n._speaker_lock = threading.Lock()
        n._current_speaker = current_speaker
        return n

    def test_unknown_never_matches(self):
        n = self._node({"is_known": False})
        assert n._reply_speaker_matches_identity_plan(
            {"new_id": BORIS_ID, "known_id": SASHA_ID}
        ) is False

    def test_stranger_does_not_match(self):
        n = self._node({"is_known": True, "speaker_id": STRANGER_ID})
        assert n._reply_speaker_matches_identity_plan(
            {"new_id": BORIS_ID, "known_id": SASHA_ID}
        ) is False

    def test_new_id_matches(self):
        n = self._node({"is_known": True, "speaker_id": BORIS_ID})
        assert n._reply_speaker_matches_identity_plan(
            {"new_id": BORIS_ID, "known_id": SASHA_ID}
        ) is True

    def test_known_id_matches(self):
        n = self._node({"is_known": True, "speaker_id": SASHA_ID})
        assert n._reply_speaker_matches_identity_plan(
            {"new_id": BORIS_ID, "known_id": SASHA_ID}
        ) is True
