"""Кадр ``Identity(ask)`` и ``/dialog/session`` в v1 (ADR-0150 §7.1, §7.2, §7.5 п.2; #3695 ч.2c)."""

from __future__ import annotations

import json
import math
import time
import types
from unittest.mock import MagicMock

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub

install_dialog_msgs_stub()

from rob_box_dialog.session import SpeakerRef  # noqa: E402
from rob_box_dialog.speaker_adapter import AskCandidate  # noqa: E402
from rob_box_voice.core import identity_question as iq  # noqa: E402


def _node(azimuth=None):
    node = types.SimpleNamespace(_session_pub=MagicMock(), _speaker_ref=SpeakerRef(azimuth_deg=azimuth))
    logger = MagicMock()
    node.get_logger = lambda: logger
    return node


def _heard(t, doa=None, source="respeaker"):
    return types.SimpleNamespace(t=t, doa_deg=doa, source=source)


def test_question_opens_identity_frame_with_azimuth_and_publishes_session():
    node = _node(azimuth=80.0)
    iq.set_speaker(node, SpeakerRef(person_ref=None, azimuth_deg=80.0))
    iq.open_question(node, AskCandidate("stranger", "unknown"))
    frame = iq.question(node)
    assert (frame.kind, frame.set_by, frame.payload["kind"], frame.payload["azimuth_deg"]) == (
        "Identity", "code", "unknown", 80.0)
    msg = node._session_pub.publish.call_args.args[0]
    assert msg.speaker_azimuth_deg == 80.0 and msg.speaker_name_state == "unknown"
    (scene,) = msg.scenes
    assert scene.kind == "Identity" and json.loads(scene.payload_json)["kind"] == "unknown"
    assert scene.ttl_until.sec > 0


def test_session_state_without_speaker_azimuth_is_nan_and_name_only_from_ref():
    node = _node()
    iq.set_speaker(node, SpeakerRef(person_ref="p1", name="Саша", name_state="confirmed"))
    msg = node._session_pub.publish.call_args.args[0]
    assert math.isnan(msg.speaker_azimuth_deg)
    assert (msg.speaker_id, msg.speaker_name, msg.agent) == ("p1", "Саша", "personality")


def test_first_addressed_phrase_takes_the_question_and_keeps_it_for_the_turn():
    node = _node(azimuth=80.0)
    iq.open_question(node, AskCandidate("stranger", "unknown"))
    iq.take_answer(node, "identity_answer", _heard(time.time() + 3, doa=85.0))
    assert iq.question(node) is None
    assert node._identity_answer["kind"] == "unknown"
    iq.take_answer(node, "voice", _heard(time.time() + 5))  # следующая фраза — уже не ответ
    assert node._identity_answer is None


def test_telegram_message_does_not_take_the_voice_question():
    node = _node(azimuth=80.0)
    iq.open_question(node, AskCandidate("stranger", "unknown"))
    iq.take_answer(node, "telegram", _heard(time.time(), source="telegram"))
    assert iq.question(node) is not None and node._identity_answer is None


@pytest.mark.parametrize("kind, text, name", [
    ("unknown", "Гриша", "Гриша"),
    ("unknown", "Гриша.", "Гриша"),
    ("contested", "борис", "Борис"),
    ("unknown", "Гриша пришёл", None),      # не голое имя — решает обычное «меня зовут …» (#2925) или LLM
    ("unknown", "Кракозябра", None),        # не имя из словаря
    ("single", "Саша", None),               # на «Саша, это ты?» ответ — да/нет, не представление
])
def test_bare_name_is_an_introduction_only_as_answer_to_ask_name(kind, text, name):
    node = _node()
    node._identity_answer = {"kind": kind}
    assert iq.answer_name(node, text) == name


def test_bare_name_without_question_is_nothing():
    node = _node()
    node._identity_answer = None
    assert iq.answer_name(node, "Гриша") is None
