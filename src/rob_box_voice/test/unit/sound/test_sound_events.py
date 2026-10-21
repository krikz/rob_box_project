"""ADR-0150 PR-3b (#3690): звук отвечает событием на каждый запрос - молчаливого пропуска нет (V33, A23)."""

import types
from unittest.mock import MagicMock

import pytest
from rob_box_dialog import knowledge as kn
from rob_box_dialog_msgs.msg import ExpressionEvent, SoundEvent

import rob_box_voice.sound_node as sn_module
from rob_box_voice.sound_node import EARCON_SOUNDS, SoundNode


class _Thread:
    """Поток-заглушка: запоминает, что запустили, и не исполняет (исполняем сами)."""

    started = []

    def __init__(self, target=None, args=(), daemon=None):
        self.target, self.args = target, args

    def start(self):
        _Thread.started.append(self)


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(sn_module, "sd", MagicMock())
    monkeypatch.setattr(sn_module.threading, "Thread", _Thread)
    _Thread.started = []
    node = SoundNode()
    node.sounds = {"button_click": _segment(), "ui_confirm": _segment(), "error": _segment()}
    node.playback_manager = MagicMock()
    node.playback_manager.play_audio.return_value = True
    node.cleanup_playback_noise = lambda: None
    return node


def _segment():
    return types.SimpleNamespace(frame_rate=16000, channels=1, get_array_of_samples=lambda: [0, 1, 2])


def _request(earcon, turn_id="turn-1"):
    return types.SimpleNamespace(earcon=earcon, turn_id=turn_id)


def _sound_events(node):
    return [c.args[0] for c in node.sound_event_pub.publish.call_args_list]


def _expression_events(node):
    return [c.args[0] for c in node.expression_event_pub.publish.call_args_list]


def test_every_earcon_of_the_table_has_a_sound_file():
    assert set(EARCON_SOUNDS) == set(kn.EARCONS)


def test_earcon_starts_the_sound_and_answers_started(node):
    node.expression_callback(_request("heard"))
    (thread,) = _Thread.started
    assert thread.args[:2] == ("button_click", "button_click") and thread.args[2] == "turn-1"
    (answer,) = _expression_events(node)
    assert (answer.executor, answer.turn_id, answer.kind, answer.reason) == ("sound", "turn-1", ExpressionEvent.KIND_STARTED, "")


def test_second_earcon_while_the_first_plays_is_skipped_busy_not_silently_dropped(node):
    node.expression_callback(_request("heard", "t1"))
    node.expression_callback(_request("done", "t2"))
    assert len(_Thread.started) == 1
    (skipped,) = _sound_events(node)
    assert (skipped.kind, skipped.reason, skipped.request_id, skipped.sound) == (
        SoundEvent.KIND_SKIPPED, "busy", "t2", "ui_confirm")
    first, second = _expression_events(node)
    assert (first.kind, second.kind, second.reason) == (ExpressionEvent.KIND_STARTED, ExpressionEvent.KIND_SKIPPED, "busy")


def test_voice_stream_wins_over_effects_and_says_so(node):
    node._voice_stream = object()
    node.expression_callback(_request("fail"))
    (skipped,) = _sound_events(node)
    assert (skipped.kind, skipped.reason) == (SoundEvent.KIND_SKIPPED, "voice_stream")
    assert _Thread.started == []


def test_sound_missing_from_the_pack_is_skipped_not_found(node):
    node.sounds.pop("ui_confirm")
    node.expression_callback(_request("done"))
    (skipped,) = _sound_events(node)
    assert (skipped.kind, skipped.reason) == (SoundEvent.KIND_SKIPPED, "not_found")


def test_request_without_earcon_is_not_a_sound_request(node):
    node.expression_callback(_request(""))
    assert _sound_events(node) == [] and _expression_events(node) == []


def test_play_sound_trigger_gets_the_same_events(node):
    node.trigger_callback(types.SimpleNamespace(data="button_click"))
    node.trigger_callback(types.SimpleNamespace(data="error"))
    (skipped,) = _sound_events(node)
    assert (skipped.kind, skipped.reason) == (SoundEvent.KIND_SKIPPED, "busy")
    assert _expression_events(node) == []


def test_thread_reports_started_then_finished(node):
    node.play_sound_thread("button_click", "button_click", "r1")
    started, finished = _sound_events(node)
    assert (started.kind, started.request_id, started.sound) == (SoundEvent.KIND_STARTED, "r1", "button_click")
    assert (finished.kind, finished.reason) == (SoundEvent.KIND_FINISHED, "")
    assert node.is_playing is False


def test_thread_reports_skipped_when_the_audio_device_refuses(node):
    node.playback_manager.play_audio.return_value = False
    node.play_sound_thread("error", "error", "r2")
    last = _sound_events(node)[-1]
    assert (last.kind, last.reason) == (SoundEvent.KIND_SKIPPED, "busy")


def test_thread_reports_skipped_error_on_exception_and_frees_the_channel(node):
    node.playback_manager.play_audio.side_effect = RuntimeError("boom")
    node.play_sound_thread("error", "error", "r3")
    last = _sound_events(node)[-1]
    assert (last.kind, last.reason) == (SoundEvent.KIND_SKIPPED, "error")
    assert node.is_playing is False
