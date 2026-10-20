"""ADR-0150 PR-2: фраза ReSpeaker -> поля ``Utterance`` (тип из настоящего ``.msg``)."""

from __future__ import annotations

from rob_box_core.msg_stub import install_dialog_msgs_stub
from rob_box_voice.core.stt_utterance import SOURCE_RESPEAKER, fill_respeaker_utterance
from rob_box_voice.core.utterance_id import compute_utterance_id

M = install_dialog_msgs_stub()


def test_source_constant_matches_msg_definition():
    assert SOURCE_RESPEAKER == M.Utterance.SOURCE_RESPEAKER


def test_fields_of_a_respeaker_phrase():
    audio = b"\x01\x02" * 8000
    msg = fill_respeaker_utterance(
        M.Utterance(),
        text="робот привет",
        utterance_id=compute_utterance_id(audio),
        duration_s=2.5,
        now_s=1_800_000_010.25,
        speaker_tag="1",
        provider="yandex",
    )
    assert msg.text == "робот привет"
    assert msg.utterance_id == compute_utterance_id(audio) != ""
    assert (msg.source, msg.chat_id, msg.speaker_tag, msg.provider) == ("respeaker", 0, "1", "yandex")
    assert (msg.ended_at.sec, msg.ended_at.nanosec) == (1_800_000_010, 250_000_000)
    assert (msg.started_at.sec, msg.started_at.nanosec) == (1_800_000_007, 750_000_000)


def test_missing_tag_and_provider_become_empty_strings():
    msg = fill_respeaker_utterance(
        M.Utterance(), text="а", utterance_id="u", duration_s=-1.0, now_s=5.0,
        speaker_tag=None, provider=None,
    )
    assert (msg.speaker_tag, msg.provider) == ("", "")
    assert msg.started_at.sec == msg.ended_at.sec == 5  # отрицательная длительность не уводит назад
