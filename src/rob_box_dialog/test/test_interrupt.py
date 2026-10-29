"""Перебивание по эпохе и классу действия (ADR-0150 §6.2, §5.1; PR-9 #3703): решение кода, без ROS и LLM."""

import concurrent.futures
from dataclasses import replace

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import ToolCall
from rob_box_dialog.grammar import Tier1Command
from rob_box_dialog.interrupt import (HALT, NO_INTERRUPT, STOP, TurnsInFlight, on_interrupt, stop_music_calls,
                                      stops_music, survives)
from rob_box_dialog.session import MusicStateRef, SceneFrame, Session

SNAP = Session("personality").snapshot
DJ = replace(SNAP, music=MusicStateRef("playing"), scenes=(SceneFrame("DJ", "code"),))
TRACK = replace(SNAP, music=MusicStateRef("playing", track_id="t1"))


def test_halt_cancels_everything_now_including_motion():
    stop = on_interrupt("Робот, стой", DJ)
    assert stop.interrupts and stop.policy == HALT and stop.kind == "stop_all"
    assert stop.speech_boundary == "now" and stop.perform == HALT and stop.stop_motion
    assert stop.music == (ToolCall("dj_set", {"action": "stop"}), ToolCall("stop_music", {}))
    assert "action.motion" in stop.cancel_actions
    assert stop.keep == {"action.local", "action.identity"}


def test_enough_replaces_on_sentence_boundary_and_keeps_motion_and_identity():
    stop = on_interrupt("Робот, хватит", TRACK)
    assert stop.policy == STOP and stop.speech_boundary == "as_requested" and stop.perform == STOP
    assert stop.music == (ToolCall("stop_music", {}),)
    assert {"action.motion", "action.identity", "action.local"} == stop.keep
    assert {"query", "action.perform", "action.speak_aloud", "action.media"} == stop.cancel_actions
    # В20 (Шифу 22.10): «хватит» — стоп звуков, не тишина; stop_motion «хватит» шлёт по-прежнему (вопрос открыт)
    assert stop.stop_motion


def test_silence_word_interrupts_speech_but_not_music_or_motion():
    stop = on_interrupt("Робот, замолчи", DJ)
    assert stop.policy == STOP and stop.kind == "silence"
    assert stop.music == () and not stop.stop_motion and stop.perform == STOP


@pytest.mark.parametrize("snap", [TRACK, DJ])
def test_3994_stop_speech_cuts_speech_and_performance_but_not_music_or_motion(snap):
    """n811: «хватит читать, а музыку оставь» под играющий бит — речь и выступление гаснут, плеер и движение — нет;
    бит самого выступления доигрывает (``keep_backing``)."""
    stop = on_interrupt("Робот, хватит читать, а музыку оставь", snap)
    assert stop.interrupts and stop.policy == STOP and stop.kind == "stop_speech"
    assert stop.perform == STOP and stop.speech_boundary == "as_requested" and stop.keep_backing
    assert stop.music == () and not stop.stop_motion


@pytest.mark.parametrize("snap", [TRACK, DJ])
@pytest.mark.parametrize("text", ["Робот, стой, музыку оставь", "Робот, стой, а музыку оставь",
                                  "Робот, стой, музыку не выключай"])
def test_3994_halt_keeping_music_still_stops_motion_but_not_music(text, snap):
    """Ревью PR #4035: «стой, а музыку оставь» не должен оставлять робота ехать. ``halt``: речь сразу, выступление —
    halt, ``stop_motion``; плеер музыки не гасится, бит выступления доигрывает."""
    stop = on_interrupt(text, snap)
    assert stop.interrupts and stop.policy == HALT and stop.kind == "stop_all"
    assert stop.stop_motion and "action.motion" in stop.cancel_actions
    assert stop.speech_boundary == "now" and stop.perform == HALT and stop.keep_backing
    assert stop.music == ()


@pytest.mark.parametrize("text", ["Робот, хватит, музыку оставь", "Робот, стоп, а бит не трогай"])
def test_3994_bare_stop_keeping_music_stops_motion_like_enough(text):
    stop = on_interrupt(text, TRACK)
    assert stop.policy == STOP and stop.kind == "stop_all" and stop.stop_motion
    assert stop.music == () and stop.keep_backing


def test_3994_shut_up_keeping_music_keeps_the_performance_beat():
    stop = on_interrupt("Робот, замолчи, а музыку оставь", TRACK)
    assert stop.kind == "silence" and stop.keep_backing and stop.music == () and not stop.stop_motion


def test_3994_stops_music_only_for_stop_all_without_keep():
    assert stops_music(Tier1Command("stop_all"))
    assert not stops_music(Tier1Command("stop_all", {"keep_music": True}))
    assert not stops_music(Tier1Command("stop_speech", {"keep_music": False}))
    assert not stops_music(Tier1Command("silence"))


def test_3994_stop_speech_without_keep_lets_the_performance_outro_its_beat():
    stop = on_interrupt("Робот, хватит читать", TRACK)
    assert stop.kind == "stop_speech" and stop.music == () and not stop.stop_motion and not stop.keep_backing


def test_3994_stt_partial_with_stop_speech_already_interrupts():
    """Сегмент фразы с вейком (``/dialog/interrupt_request``): «робот хватит читать» рвёт речь, как «хватит»."""
    assert on_interrupt("робот хватит читать", TRACK).interrupts


def test_4020_stop_form_stops_everything_like_enough():
    stop = on_interrupt("Робот, останови, Борис уже уснул под неё", TRACK)
    assert stop.kind == "stop_all" and stop.music == (ToolCall("stop_music", {}),) and stop.stop_motion
    assert not stop.keep_backing


@pytest.mark.parametrize("text", ["Робот, а который час", "Робот, спой песню про стоп", "Робот, расскажи сказку",
                                  "стоит ли нам ехать", "Робот, хватит ли нам бензина", "",
                                  "Робот, где остановка автобуса"])
def test_not_a_clear_stop_does_not_interrupt(text):
    assert on_interrupt(text, DJ) is NO_INTERRUPT


def test_stop_word_without_wake_still_decides_by_words():
    """Адресацию (вейк) решает ``address.py`` до перебивания; здесь — только слова команды."""
    assert on_interrupt("стой", SNAP).policy == HALT


def test_survives_follows_action_classes_and_sound_rule():
    for cls, spec in kn.ACTION_CLASSES.items():
        expected = not spec.interruptible and cls not in kn.INTERRUPT_STOPS_CLASSES
        assert survives(cls) is expected, cls
    assert not survives("action.media")  # старт музыки старой эпохи не заиграет после «хватит» (В20)
    assert survives("action.motion") and survives("action.identity")


def test_stop_music_calls_by_snapshot():
    assert stop_music_calls(SNAP) == ()
    assert stop_music_calls(TRACK) == (ToolCall("stop_music", {}),)
    assert [c.tool for c in stop_music_calls(DJ)] == ["dj_set", "stop_music"]


def test_turns_in_flight_cancel_all_but_own_phrase():
    turns = TurnsInFlight()
    old, waiting, own, done = (concurrent.futures.Future() for _ in range(4))
    done.set_result(None)
    for fut, key in ((old, "u1"), (waiting, "u2"), (own, "u3"), (done, "u0")):
        turns.add(fut, key)
    assert len(turns) == 3
    assert turns.cancel(keep_key="u3") == 2
    assert old.cancelled() and waiting.cancelled() and not own.cancelled()
    assert len(turns) == 1 and turns.cancel() == 1 and own.cancelled()


def test_session_replace_epoch_keeps_scenes_unlike_reset():
    session = Session("personality")
    session.push_scene("OperatorHold", "operator")
    assert session.replace_epoch() == 1
    assert session.snapshot.epoch == 1 and session.snapshot.top("OperatorHold") is not None
    session.reset()
    assert session.snapshot.epoch == 2 and session.snapshot.scenes == ()
