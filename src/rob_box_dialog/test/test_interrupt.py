"""Перебивание по эпохе и классу действия (ADR-0150 §6.2, §5.1; PR-9 #3703): решение кода, без ROS и LLM."""

import concurrent.futures
from dataclasses import replace

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import ToolCall
from rob_box_dialog.interrupt import (HALT, NO_INTERRUPT, STOP, TurnsInFlight, on_interrupt, stop_music_calls,
                                      survives)
from rob_box_dialog.session import MusicStateRef, Session

SNAP = Session("personality").snapshot
DJ = replace(SNAP, music=MusicStateRef("playing", dj=True))
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


@pytest.mark.parametrize("text", ["Робот, а который час", "Робот, спой песню про стоп", "Робот, расскажи сказку",
                                  "стоит ли нам ехать", "Робот, хватит ли нам бензина", ""])
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
