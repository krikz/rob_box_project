"""Tests for ``rob_box_core.speaker_wire`` и ``rob_box_core.msg_stub`` (ADR-0150 PR-2).

Класс сообщения строится из НАСТОЯЩЕГО ``rob_box_dialog_msgs/msg/*.msg``: если
поле переименуют в ``.msg``, а кодек не поправят, тест упадёт.
"""

from __future__ import annotations

import math

import pytest

from rob_box_core.msg_stub import build_dialog_msgs_module, install_dialog_msgs_stub
from rob_box_core.speaker_wire import (
    NAME_CONTESTED,
    TENTATIVE_CONTESTED,
    fill_speaker_result,
    name_state,
    speaker_result_to_payload,
)

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


def test_unknown_voice_carries_its_best_candidate_as_a_hypothesis():
    """#3767: лучший кандидат неузнанной фразы — гипотеза для метки бэклога, у узнанной не передаётся."""
    unknown = {"is_known": False, "candidate_name": "Борис", "candidate_score": 0.4123, "utterance_id": "u"}
    assert _roundtrip(unknown) == unknown
    inc = {"is_known": False, "inconclusive": True, "reason": "too_short", "candidate_name": "Саша",
           "candidate_score": 0.3}
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


def test_registered_by_renaming_the_voice_profile_carries_new_name_4054():
    """#4054: регистрация переименовала уверенно узнанный профиль голоса фразы — ``new_name`` доезжает до читателя."""
    payload = {"event": "registered", "name": "Тамара", "speaker_id": "e35517b2", "reused_profile": True,
               "utterance_id": "u722", "new_name": "Тамара"}
    assert _roundtrip(payload) == payload


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


def test_e2e_db_flag_roundtrips_and_is_absent_by_default():
    """#3876: метка «результат из E2E-БД» доезжает до потребителя; боевой результат её не несёт."""
    payload = {"is_known": True, "speaker_id": "sid-1", "name": "Саша", "confidence": 0.99,
               "epithet": None, "e2e_db": True}
    assert _roundtrip(payload) == payload
    prod = {"is_known": True, "speaker_id": "sid-1", "name": "Денис", "confidence": 0.9,
            "epithet": None}
    assert "e2e_db" not in _roundtrip(prod)
    assert M.SpeakerResult().e2e_db is False
    # метка не зависит от вида события
    ack = {"event": "registered", "name": "Саша", "speaker_id": "s", "reused_profile": False,
           "utterance_id": None, "e2e_db": True}
    assert _roundtrip(ack)["e2e_db"] is True


# -- name_state: что результат фразы говорит об имени (#4023, #3993) ---------------------------------------


@pytest.mark.parametrize("payload, state", [
    ({"is_known": True, "speaker_id": "s", "name": "Саша", "confidence": 0.95}, "confirmed"),
    ({"is_known": True, "speaker_id": "s", "name": "Саша", "source": "register"}, "confirmed"),
    ({"is_known": True, "speaker_id": "s", "name": None, "confidence": 0.77, "tentative_conf": 0.77,
      "tentative_kind": "single", "tentative_name": "Дэнчик"}, "tentative"),
    # акт 2b 23.10: «Саша» 0.944, «Борис» 0.661, band_high 0.99 — имя скрыто, кандидатов двое
    ({"is_known": True, "speaker_id": "s", "name": None, "confidence": 0.944, "tentative_conf": 0.944,
      "tentative_kind": "contested"}, "contested"),
    ({"is_known": True, "speaker_id": "s", "name": None, "confidence": 0.0, "epithet": "Гость",
      "source": "enroll"}, "nameless"),
    ({"is_known": False}, "unknown"),
    ({"is_known": False, "inconclusive": True, "reason": "too_short"}, "unknown"),
    ({"is_known": False, "candidate_name": "Борис", "candidate_score": 0.41}, "unknown"),
    ({"event": "registered", "name": "Саша", "speaker_id": "s", "reused_profile": False}, "unknown"),
])
def test_name_state_of_phrase_result_survives_the_wire(payload, state):
    """Состояние выводится из полей сообщения — до провода и после одно и то же."""
    assert name_state(payload) == state
    assert name_state(_roundtrip(payload)) == state


def test_contested_result_keeps_only_the_candidate_id_and_no_name():
    """contested: ``speaker_id`` — лишь ближайший кандидат (приписывать его голосу не будут потребители), имени нет."""
    out = _roundtrip({"is_known": True, "speaker_id": "sasha-id", "name": None, "confidence": 0.944,
                      "tentative_conf": 0.944, "tentative_kind": TENTATIVE_CONTESTED, "utterance_id": "u1"})
    assert name_state(out) == NAME_CONTESTED
    assert out["name"] is None and "tentative_name" not in out and out["speaker_id"] == "sasha-id"
