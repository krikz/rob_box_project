"""Детекторы сбоя хода кодом (#4512, шаг 3; ADR-0148): «робот глух» и «ход без речи» на синтетических событиях.

Тесты проверяют поведение ядра ``scripts/dialog/turn_failures.py`` (вердикт и его причина), адаптер строк лога и сверку
списка «Tier-1 без фразы» с самим ``turn.tier1_phrase`` — знание о фразах Tier-1 живёт там.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
for _pkg in ("rob_box_core", "rob_box_dialog"):
    sys.path.insert(0, str(REPO / "src" / _pkg))
sys.path.insert(0, str(REPO / "scripts" / "dialog"))

import turn_failures as tf  # noqa: E402

from rob_box_dialog.grammar import Tier1Command  # noqa: E402
from rob_box_dialog.respond import ActionResult  # noqa: E402
from rob_box_dialog.turn import tier1_phrase  # noqa: E402

E = tf.Event


def rejected(ts):
    return E(ts, tf.STT_REJECTED)


def turn(ts, turn_id="t1", stage="done", status="finished", agent="personality", extra=()):
    """Ход: открылся в ``ts``, события ``extra`` (уже с id), конец через 1 с."""
    return [E(ts, tf.TURN_OPEN, turn_id), *extra, E(ts + 1.0, tf.TURN_END, turn_id, stage, status, agent=agent)]


def speech(ts, kind="queued", speech_id="s1"):
    return E(ts, tf.SPEECH, value=kind, extra=speech_id)


def kinds(verdicts):
    return [(v.kind, v.reason) for v in verdicts]


# --- «робот глух» ---


def test_three_rejected_phrases_without_turn_is_deaf():
    out = tf.detect([rejected(10.0), rejected(25.0), rejected(40.0)])
    assert kinds(out) == [(tf.DEAF, "stt_rejected=3")]
    assert out[0].since == 10.0 and out[0].ts == 40.0


def test_two_rejected_phrases_are_noise():
    assert tf.detect([rejected(10.0), rejected(25.0)]) == []


def test_turn_between_rejected_phrases_resets_the_series():
    events = [rejected(10.0), rejected(20.0), *turn(30.0), rejected(40.0), rejected(50.0)]
    assert [v for v in tf.detect(events) if v.kind == tf.DEAF] == []


def test_series_must_fit_the_window():
    gap = tf.DEAF_WINDOW_S / 2 + 1.0
    assert tf.detect([rejected(0.0), rejected(gap), rejected(2 * gap)]) == []


def test_series_fires_once_until_a_turn():
    events = [rejected(float(t)) for t in range(0, 50, 10)]
    assert kinds(tf.detect(events)) == [(tf.DEAF, "stt_rejected=3")]


def test_empty_stt_under_music_is_the_music():
    events = [
        E(0.0, tf.MUSIC, value=tf.MUSIC_ON),
        rejected(10.0),
        rejected(20.0),
        rejected(30.0),
        E(35.0, tf.MUSIC, value=tf.MUSIC_OFF),
        rejected(40.0),
    ]
    assert tf.detect(events) == []


def test_stt_provider_failure_is_counted_even_under_music():
    failed = [E(float(t), tf.STT_REJECTED, value=tf.STT_FAILED) for t in (10, 20)]
    events = [E(0.0, tf.MUSIC, value=tf.MUSIC_ON), *failed, rejected(25.0), rejected(30.0)]
    assert tf.detect(events) == []  # под музыкой пустые не идут: отказов только два
    out = tf.detect([*failed, rejected(30.0)])
    assert kinds(out) == [(tf.DEAF, "stt_rejected=3,stt_error=2")]


def test_echo_is_not_a_human_phrase():
    events = [rejected(10.0), E(15.0, tf.ECHO), E(16.0, tf.ECHO), rejected(20.0)]
    assert tf.detect(events) == []


def test_no_wake_series_goes_to_reason_but_does_not_fire_by_default():
    no_wake = [E(float(t), tf.NO_WAKE) for t in range(0, 40, 5)]
    assert tf.detect(no_wake) == []
    out = tf.detect([E(1.0, tf.NO_WAKE), rejected(10.0), rejected(20.0), rejected(30.0)])
    assert kinds(out) == [(tf.DEAF, "stt_rejected=3,no_wake=1")]


def test_no_wake_threshold_is_a_knob(monkeypatch):
    monkeypatch.setattr(tf, "DEAF_NO_WAKE_MIN", 2)
    out = tf.detect([E(1.0, tf.NO_WAKE), E(5.0, tf.NO_WAKE)])
    assert kinds(out) == [(tf.DEAF, "no_wake=2")]


# --- «ход без речи» ---


def test_turn_with_queued_speech_is_fine():
    assert tf.detect(turn(0.0, extra=[speech(0.5)])) == []


def test_turn_without_speech_is_no_speech():
    out = tf.detect(turn(0.0, extra=[E(0.5, tf.EXECUTED, "t1", "navigate_to_waypoint", "done")]))
    assert [(v.kind, v.reason, v.turn_id) for v in out] == [(tf.NO_SPEECH, "actions_only", "t1")]


def test_empty_turn_reason():
    assert kinds(tf.detect(turn(0.0))) == [(tf.NO_SPEECH, "empty")]


def test_speech_right_after_turn_end_counts_within_grace():
    events = [*turn(0.0), speech(1.0 + tf.SPEECH_GRACE_S - 1.0)]
    assert tf.detect(events) == []


def test_speech_after_grace_belongs_to_no_turn():
    events = [*turn(0.0), speech(1.0 + tf.SPEECH_GRACE_S + 1.0)]
    assert kinds(tf.detect(events)) == [(tf.NO_SPEECH, "empty")]


def test_verdict_waits_for_grace_while_streaming():
    watch = tf.TurnWatch()
    for event in turn(0.0):
        assert watch.feed(event) == []
    assert watch.feed(rejected(1.0 + tf.SPEECH_GRACE_S - 0.5)) == []
    assert kinds(watch.feed(rejected(1.0 + tf.SPEECH_GRACE_S + 0.5))) == [(tf.NO_SPEECH, "empty")]
    assert watch.flush() == []


def test_queued_then_rejected_speech_is_dropped():
    events = turn(0.0, extra=[speech(0.5), speech(0.6, "rejected")])
    assert kinds(tf.detect(events)) == [(tf.NO_SPEECH, "speech_dropped")]


def test_started_speech_owner_is_the_turn_it_was_queued_in():
    events = [*turn(0.0, "t1", extra=[speech(0.5, "queued", "s1")]), *turn(2.0, "t2"), speech(2.5, "started", "s1")]
    assert [(v.turn_id, v.reason) for v in tf.detect(events)] == [("t2", "empty")]


@pytest.mark.parametrize("stage", [tf.kn.SILENCED_TURN_STAGE, "cancelled"])
def test_silenced_and_interrupted_turns_are_silent_by_right(stage):
    assert tf.detect(turn(0.0, stage=stage)) == []


def test_failed_turn_without_speech():
    assert kinds(tf.detect(turn(0.0, stage="failed", status="failed"))) == [(tf.NO_SPEECH, "failed")]


def test_text_channel_turn_answers_in_chat():
    assert tf.detect(turn(0.0, agent=tf.kn.TEXT_CHANNEL_AGENT)) == []


def test_model_pass_on_backlog_pickup_is_silent_by_right():
    assert tf.detect(turn(0.0, extra=[E(0.5, tf.PASSED)])) == []


def test_perform_speaks_through_its_owner():
    assert tf.detect(turn(0.0, extra=[E(0.5, tf.PERFORM, "t1")])) == []


def test_sound_or_face_on_request_is_an_answer():
    events = turn(0.0, extra=[E(0.5, tf.EXECUTED, "t1", "play_animation", "done")])
    assert tf.detect(events) == []


def test_sound_after_reading_data_owes_words():
    """М11 n109: прочитал список звуков и только пискнул (``turn.sound_owed``)."""
    events = turn(
        0.0,
        extra=[E(0.3, tf.EXECUTED, "t1", "get_sound_info", "done"), E(0.6, tf.EXECUTED, "t1", "play_sound", "done")],
    )
    assert kinds(tf.detect(events)) == [(tf.NO_SPEECH, "sound_after_query")]


@pytest.mark.parametrize("kind", tf.TIER1_SILENT_KINDS)
def test_tier1_kinds_without_phrase_are_silent_by_right(kind):
    assert tf.detect(turn(0.0, extra=[E(0.1, tf.TIER1, "t1", kind)])) == []


def test_stop_all_without_music_is_silent_with_music_owes_a_phrase():
    """Решение Шифу 05.11 (М12 n511): стоп, погасивший музыку, говорит фразу кода."""
    quiet = turn(0.0, extra=[E(0.1, tf.TIER1, "t1", tf.STOP_ALL_KIND)])
    assert tf.detect(quiet) == []
    music = turn(0.0, extra=[E(0.1, tf.TIER1, "t1", tf.STOP_ALL_KIND), E(0.2, tf.EXECUTED, "t1", "stop_music", "done")])
    assert kinds(tf.detect(music)) == [(tf.NO_SPEECH, "tier1:stop_all")]


def test_voiced_tier1_kind_without_speech_is_no_speech():
    events = turn(0.0, extra=[E(0.1, tf.TIER1, "t1", "volume"), E(0.2, tf.EXECUTED, "t1", "set_volume", "done")])
    assert kinds(tf.detect(events)) == [(tf.NO_SPEECH, "tier1:volume")]


# --- сверка «Tier-1 без фразы» с turn.tier1_phrase (знание — там) ---


def _result(tool, status="done"):
    return ActionResult("t1", 0, "personality", tool, "action.music", status)


@pytest.mark.parametrize("kind", tf.TIER1_SILENT_KINDS)
def test_silent_kinds_have_no_phrase_in_turn(kind):
    assert tier1_phrase(Tier1Command(kind), [], pick=lambda key: key) is None


def test_stop_all_phrase_follows_music_tools_in_turn():
    assert tier1_phrase(Tier1Command(tf.STOP_ALL_KIND), [], pick=lambda key: key) is None
    assert tier1_phrase(Tier1Command(tf.STOP_ALL_KIND), [_result("stop_music")], pick=lambda key: key)
    assert tf.tier1_silent(tf.STOP_ALL_KIND, []) and not tf.tier1_silent(tf.STOP_ALL_KIND, [("stop_music", "done")])


def test_show_queue_phrase_only_on_failure_in_turn():
    assert tier1_phrase(Tier1Command(tf.kn.SHOW_QUEUE_KIND), [_result("play_animation")]) is None
    assert tier1_phrase(Tier1Command(tf.kn.SHOW_QUEUE_KIND), [_result("play_animation", "failed")])
    assert tf.tier1_silent(tf.kn.SHOW_QUEUE_KIND, [("play_animation", "done")])
    assert not tf.tier1_silent(tf.kn.SHOW_QUEUE_KIND, [("play_animation", "failed")])


# --- адаптер строк лога (синтетические строки, без реплик людей) ---

_P = "[dialog_node-3] [INFO] [1793743448.500000000] [dialog_node]: "
_STT = "[stt_node-5] [WARN] [1793743448.500000000] [stt_node]: [stt_attempt] "
_TTS = "[tts_node-4] [INFO] [1793743448.500000000] [tts_node]: "


@pytest.mark.parametrize(
    "line, expected",
    [
        (
            _STT + "yandex:empty(1ms)->vosk:empty(1ms) -> rejected",
            E(1793743448.5, tf.STT_REJECTED),
        ),
        (
            _STT + "minimax:dead(0ms)->yandex:timeout(9ms) " "-> rejected",
            E(1793743448.5, tf.STT_REJECTED, value=tf.STT_FAILED),
        ),
        (
            _STT + "yandex:ok(1ms '<t>') -> accepted '<t>'",
            None,
        ),
        (_P + "[backlog] accumulated reason=no_wake entries=1", E(1793743448.5, tf.NO_WAKE)),
        (
            _P + "🧭 [turn-v2] форма команды: закрыты=[] пение=- turn=0123456789ab",
            E(1793743448.5, tf.TURN_OPEN, "0123456789ab"),
        ),
        (
            _P + "🧭 [turn-v2] tier1 kind=stop_all args={} turn=0123456789ab",
            E(1793743448.5, tf.TIER1, "0123456789ab", "stop_all"),
        ),
        (
            _P + "📨 [execute] событие stop_music 0123456789ab/0 done - ход_ждёт=True",
            E(1793743448.5, tf.EXECUTED, "0123456789ab", "stop_music", "done"),
        ),
        (
            _P + "🧭 [turn-v2] выступление старт kind=tale backing=none → {} turn=0123456789ab",
            E(1793743448.5, tf.PERFORM, "0123456789ab"),
        ),
        (_P + "🧭 [turn-v2] passed=unclear addressed=False", E(1793743448.5, tf.PASSED)),
        (_P + "🧭 [turn-v2] passed=unclear addressed=True", None),
        (
            _P + "📊 [turn] turn_id=0123456789ab epoch=3 agent=telegram stage=done status=finished latency_ms=1",
            E(1793743448.5, tf.TURN_END, "0123456789ab", "done", "finished", agent="telegram"),
        ),
        (
            _TTS + "🗣️ SpeechEvent queued speech_id=769a9f3f agent=personality",
            E(1793743448.5, tf.SPEECH, value="queued", extra="769a9f3f"),
        ),
        (
            "[mcp_server-9] [INFO] [1793743448.500000000] [mcp_server]: 🎵 [music] started track_id=x clock_beat=1",
            E(1793743448.5, tf.MUSIC, value=tf.MUSIC_ON),
        ),
        (
            "[mcp_server-9] [INFO] [1793743448.500000000] [mcp_server]: 🎵 [music] stop track_id=x reason=user_stop",
            E(1793743448.5, tf.MUSIC, value=tf.MUSIC_OFF),
        ),
        ("[dialog_node-3]   tools(10): command, memory_save", None),
    ],
)
def test_line_event(line, expected):
    assert tf.line_event(line) == expected


def test_events_from_log_sorts_and_dedupes_overlapping_acts():
    a = "[stt_node-5] [WARN] [1793743450.000000000] [stt_node]: [stt_attempt] x -> rejected"
    b = "[stt_node-5] [WARN] [1793743449.000000000] [stt_node]: [stt_attempt] x -> rejected"
    assert [e.ts for e in tf.events_from_log([a, b, a])] == [1793743449.0, 1793743450.0]


def test_cli_prints_verdicts_without_texts(tmp_path, capsys):
    log = tmp_path / "voice.log"
    log.write_text(
        "\n".join(
            f"[stt_node-5] [WARN] [{1793743400 + 10 * i}.000000000] [stt_node]: [stt_attempt] y:empty(1ms) -> rejected"
            for i in range(3)
        ),
        encoding="utf-8",
    )
    assert tf.main([str(log)]) == 0
    assert capsys.readouterr().out.strip() == (
        "voice.log 🚨 [turn-fail] deaf ts=1793743420.000 since=1793743400.000 reason=stt_rejected=3"
    )
