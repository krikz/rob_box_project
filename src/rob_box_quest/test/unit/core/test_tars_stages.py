"""Unit-тесты TarsStageTracker (issue #3253 Ш2, ADR-0078 §3.6).

Время — аргументом (монотонные секунды), часов нет. PCM — int16 моно:
16000 Гц × 2 байта = 32000 байт на секунду звука.
"""

from __future__ import annotations

from rob_box_quest.core.tars_stages import (
    DEFAULT_SILENCE_TIMEOUT_S,
    DEFAULT_SYNTH_TIMEOUT_S,
    DEFAULT_TAIL_MARGIN_S,
    TarsStageTracker,
    pcm_duration_s,
)

SR = 16000
ONE_SECOND = 2 * SR  # байт int16-моно на 1 с звука


def _stages(events):
    return [e["stage"] for e in events if e["kind"] == "stage"]


def _kinds(events):
    return [e["kind"] for e in events]


def test_pcm_duration_int16_mono():
    assert pcm_duration_s(ONE_SECOND, SR) == 1.0
    assert pcm_duration_s(0, SR) == 0.0
    assert pcm_duration_s(100, 0) == 0.0


def test_first_chunk_emits_speaking_once():
    t = TarsStageTracker()
    assert t.on_request("r1", "tars-r1", 0.0) == []
    first = t.on_chunk("r1", ONE_SECOND, SR, 0.1)
    assert _stages(first) == ["speaking"]
    assert first[0]["request_id"] == "r1"
    assert t.on_chunk("r1", ONE_SECOND, SR, 0.2) == []


def test_idle_and_done_only_after_audio_played_out():
    """finished приходит раньше конца звука (стрим быстрее реального времени):
    idle + done — только после оценки конца звука + запас."""
    t = TarsStageTracker()
    t.on_request("r1", "tars-r1", 0.0)
    t.on_chunk("r1", ONE_SECOND, SR, 1.0)  # звук до 2.0
    t.on_chunk("r1", ONE_SECOND, SR, 1.1)  # очередь: звук до 3.0
    assert t.on_finished("tars-r1", True, "", 1.2) == []
    assert t.tick(3.0) == []
    assert t.tick(3.0 + DEFAULT_TAIL_MARGIN_S - 0.01) == []
    events = t.tick(3.0 + DEFAULT_TAIL_MARGIN_S)
    assert _stages(events) == ["idle"]
    assert events[0]["reason"] == "done"
    assert _kinds(events) == ["stage", "done"]
    assert events[1]["request_id"] == "r1"
    # Реплика закрыта — повторных idle нет.
    assert t.tick(100.0) == []
    assert t.active_request_id is None


def test_finished_after_audio_end_closes_immediately():
    t = TarsStageTracker()
    t.on_request("r1", "tars-r1", 0.0)
    t.on_chunk("r1", ONE_SECOND, SR, 0.0)
    events = t.on_finished("tars-r1", True, "", 5.0)
    assert _stages(events) == ["idle"]
    assert _kinds(events) == ["stage", "done"]


def test_full_sequence_speaking_then_idle():
    t = TarsStageTracker()
    seq = []
    seq += t.on_request("r1", "tars-r1", 0.0)
    seq += t.on_chunk("r1", ONE_SECOND // 2, SR, 0.5)
    seq += t.on_chunk("r1", ONE_SECOND // 2, SR, 0.6)
    seq += t.on_finished("tars-r1", True, "", 0.7)
    seq += t.tick(10.0)
    assert _stages(seq) == ["speaking", "idle"]
    assert _kinds(seq)[-1] == "done"


def test_foreign_speech_id_is_ignored():
    """/voice/tts/finished общий с голосом личности — чужой speech_id не наш."""
    t = TarsStageTracker()
    t.on_request("r1", "tars-r1", 0.0)
    t.on_chunk("r1", ONE_SECOND, SR, 0.0)
    assert t.on_finished("personality-xyz", True, "", 5.0) == []
    assert t.on_finished("", True, "", 5.0) == []
    assert t.active_request_id == "r1"


def test_chunk_of_other_request_is_ignored():
    t = TarsStageTracker()
    t.on_request("r1", "tars-r1", 0.0)
    assert t.on_chunk("r-old", ONE_SECOND, SR, 0.1) == []


def test_synthesis_error_without_audio_goes_idle_with_error():
    t = TarsStageTracker()
    t.on_request("r1", "tars-r1", 0.0)
    events = t.on_finished("tars-r1", False, "minimax_down", 1.0)
    assert _stages(events) == ["idle"]
    assert events[0]["reason"] == "minimax_down"
    assert _kinds(events) == ["stage", "error"]
    assert events[1]["reason"] == "minimax_down"


def test_synthesis_error_after_partial_audio_waits_for_played_out():
    t = TarsStageTracker()
    t.on_request("r1", "tars-r1", 0.0)
    t.on_chunk("r1", ONE_SECOND, SR, 0.0)
    assert t.on_finished("tars-r1", False, "", 0.2) == []
    events = t.tick(1.0 + DEFAULT_TAIL_MARGIN_S)
    assert _kinds(events) == ["stage", "error"]
    assert events[0]["reason"] == "tts_failed"


def test_barge_in_closes_speaking_reply_immediately():
    t = TarsStageTracker()
    t.on_request("r1", "tars-r1", 0.0)
    t.on_chunk("r1", 10 * ONE_SECOND, SR, 0.0)
    events = t.on_cancel("barge_in", 1.0)
    assert _stages(events) == ["idle"]
    assert events[0]["reason"] == "barge_in"
    assert _kinds(events) == ["stage", "done"]
    # Поздние чанки отменённой реплики speaking не возвращают.
    assert t.on_chunk("r1", ONE_SECOND, SR, 1.1) == []
    assert t.tick(100.0) == []


def test_cancel_without_reply_is_noop():
    assert TarsStageTracker().on_cancel("barge_in", 0.0) == []


def test_lost_finished_falls_back_to_silence_timeout():
    t = TarsStageTracker()
    t.on_request("r1", "tars-r1", 0.0)
    t.on_chunk("r1", ONE_SECOND, SR, 0.0)
    assert t.tick(1.0 + DEFAULT_SILENCE_TIMEOUT_S - 0.01) == []
    events = t.tick(1.0 + DEFAULT_SILENCE_TIMEOUT_S)
    assert events[0]["reason"] == "silence_timeout"
    assert _kinds(events) == ["stage", "done"]


def test_no_audio_and_no_finished_falls_back_to_synth_timeout():
    t = TarsStageTracker()
    t.on_request("r1", None, 0.0)
    assert t.tick(DEFAULT_SYNTH_TIMEOUT_S - 0.01) == []
    events = t.tick(DEFAULT_SYNTH_TIMEOUT_S)
    assert events[0]["reason"] == "synth_timeout"


def test_non_string_speech_id_is_not_matched():
    t = TarsStageTracker()
    t.on_request("r1", 123, 0.0)
    assert t.on_finished("123", True, "", 1.0) == []


def test_new_request_replaces_previous():
    t = TarsStageTracker()
    t.on_request("r1", "tars-r1", 0.0)
    t.on_chunk("r1", ONE_SECOND, SR, 0.0)
    t.on_request("r2", "tars-r2", 0.5)
    assert t.active_request_id == "r2"
    assert _stages(t.on_chunk("r2", ONE_SECOND, SR, 0.6)) == ["speaking"]
