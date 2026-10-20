"""Tests for ``rob_box_core.speaker_wire`` и ``rob_box_core.msg_stub`` (ADR-0150 PR-2).

Класс сообщения строится из НАСТОЯЩЕГО ``rob_box_dialog_msgs/msg/*.msg``: если
поле переименуют в ``.msg``, а кодек не поправят, тест упадёт.
"""

from __future__ import annotations

import math

import pytest

from rob_box_core.msg_stub import build_dialog_msgs_module, install_dialog_msgs_stub
from rob_box_core.speaker_wire import fill_speaker_result, speaker_result_to_payload

M = install_dialog_msgs_stub()


def _roundtrip(payload):
    return speaker_result_to_payload(fill_speaker_result(M.SpeakerResult(), payload))


def test_stub_built_from_msg_files():
    S = build_dialog_msgs_module()
    utt = S.Utterance(text="привет", source=S.Utterance.SOURCE_TELEGRAM, chat_id=42)
    assert (utt.utterance_id, utt.text, utt.chat_id, utt.source) == ("", "привет", 42, "telegram")
    assert utt.started_at.sec == 0 and S.SessionState().scenes == []
    with pytest.raises(AttributeError):
        S.Utterance(no_such_field=1)


def test_match_known_roundtrip():
    payload = {
        "is_known": True, "speaker_id": "sid-1", "name": "Денис",
        "confidence": 0.9312, "epithet": "Болтун", "utterance_id": "a5d8a1212530",
        "source": "register",
    }
    assert _roundtrip(payload) == payload


def test_match_known_name_suppressed_keeps_tentative():
    payload = {
        "is_known": True, "speaker_id": "sid-2", "name": None, "confidence": 0.61,
        "epithet": None, "tentative_conf": 0.61, "tentative_kind": "tentative_single",
        "tentative_name": "Денис", "utterance_id": "u1",
    }
    assert _roundtrip(payload) == payload


def test_match_unknown_and_inconclusive():
    assert _roundtrip({"is_known": False}) == {"is_known": False}
    assert _roundtrip({"is_known": False, "utterance_id": "u"}) == {"is_known": False, "utterance_id": "u"}
    inc = {"is_known": False, "inconclusive": True, "reason": "too_short", "utterance_id": "u"}
    assert _roundtrip(inc) == inc


def test_registered_with_conflict_and_twin():
    payload = {
        "event": "registered", "name": "Борис", "speaker_id": "s", "reused_profile": False,
        "utterance_id": "u9",
        "voice_conflict": {"name": "Денис", "speaker_id": "s0", "score": 0.8123},
        "name_twin": {"name": "Борис", "speaker_id": "s7", "score": None},
    }
    assert _roundtrip(payload) == payload


def test_registered_already_known_without_utterance():
    payload = {"event": "registered", "name": "Д", "speaker_id": "s", "reused_profile": True,
               "already_known": True, "utterance_id": None}
    assert _roundtrip(payload) == payload


def test_register_error_too_short_and_other():
    short = {"event": "register_error", "error": "too_short", "name": "Д",
             "duration_s": 0.41, "min_required_s": 1.5, "utterance_id": "u"}
    assert _roundtrip(short) == short
    other = {"event": "register_error", "error": "utterance_not_found", "name": "Д",
             "utterance_id": "u"}
    assert _roundtrip(other) == other


def test_renamed_and_merged_acks():
    renamed = {"event": "renamed", "ok": True, "speaker_id": "s", "new_name": "Ден"}
    merged = {"event": "merged", "ok": False, "src_speaker_id": "a", "dst_speaker_id": "b",
              "embeddings_moved": 3, "facts_moved": 5}
    assert _roundtrip(renamed) == renamed
    assert _roundtrip(merged) == merged


def test_unknown_event_is_an_error_not_a_silent_drop():
    with pytest.raises(ValueError):
        fill_speaker_result(M.SpeakerResult(), {"event": "teleported"})


def test_twin_without_score_is_nan_on_the_wire():
    msg = fill_speaker_result(
        M.SpeakerResult(),
        {"event": "registered", "name": "Б", "speaker_id": "s", "name_twin": {"name": "Б", "speaker_id": "t", "score": None}},
    )
    assert math.isnan(msg.twin_score)
