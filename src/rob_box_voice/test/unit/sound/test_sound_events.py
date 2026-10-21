"""ADR-0150 PR-3b (#3690): звук отвечает событием на каждый запрос - молчаливого пропуска нет (V33, A23)."""

import json
import types
from unittest.mock import MagicMock

import pytest
from rob_box_dialog import knowledge as kn
from rob_box_dialog_msgs.msg import ExpressionEvent, SoundEvent

import rob_box_voice.sound_node as sn_module
from rob_box_voice.core.sound_requests import parse_trigger
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


def _segment(seconds=None):
    seg = types.SimpleNamespace(frame_rate=16000, channels=1, get_array_of_samples=lambda: [0, 1, 2])
    if seconds is not None:
        seg.duration_seconds = seconds
    return seg


def _trigger(node, sound, rid):
    node.trigger_callback(types.SimpleNamespace(data=json.dumps({"sound": sound, "request_id": rid})))


# ── очередь play_sound при коротком текущем звуке (#3698, просьба Шифу 21.10) ─────────────────


def test_play_sound_waits_out_a_short_earcon_and_plays_right_after(node):
    """Живой прогон 21.10 16:25Z: earcon «error» (~1 с) занял канал, play_sound получил busy, а на слух звук
    будто прозвучал. Теперь запрос ждёт короткий текущий звук и играет следом с тем же request_id."""
    node.sounds.update(error=_segment(1.0), robot_happy=_segment(2.2))
    node.expression_callback(_request("fail"))
    _trigger(node, "robot_happy", "rq-happy")
    assert [t.args[0] for t in _Thread.started] == ["error"]
    assert _sound_events(node) == []  # не skipped: ждёт
    node.play_sound_thread(*_Thread.started[0].args)  # earcon доиграл → следующий из очереди
    assert _Thread.started[-1].args == ("robot_happy", "robot_happy", "rq-happy")
    node.play_sound_thread(*_Thread.started[-1].args)
    kinds = [(e.request_id, e.kind) for e in _sound_events(node)]
    assert ("rq-happy", SoundEvent.KIND_STARTED) in kinds and ("rq-happy", SoundEvent.KIND_FINISHED) in kinds
    assert node.is_playing is False


def test_long_current_sound_is_an_honest_busy(node):
    node.sounds.update(robot_stinger=_segment(12.4), robot_happy=_segment(2.2))
    _trigger(node, "robot_stinger", "rq-stinger")
    _trigger(node, "robot_happy", "rq-happy")
    (skipped,) = _sound_events(node)
    assert (skipped.request_id, skipped.kind, skipped.reason) == ("rq-happy", SoundEvent.KIND_SKIPPED, "busy")
    assert kn.SOUND_QUEUE_MAX_WAIT_S < 12.4


def test_newer_request_supersedes_the_waiting_one(node):
    node.sounds.update(error=_segment(1.0), robot_happy=_segment(2.2), ui_confirm=_segment(0.7))
    node.expression_callback(_request("fail"))
    _trigger(node, "robot_happy", "rq-1")
    _trigger(node, "ui_confirm", "rq-2")
    (skipped,) = _sound_events(node)
    assert (skipped.request_id, skipped.reason) == ("rq-1", "superseded")
    node.play_sound_thread(*_Thread.started[0].args)
    assert _Thread.started[-1].args == ("ui_confirm", "ui_confirm", "rq-2")


def test_earcon_does_not_queue(node):
    node.sounds.update(button_click=_segment(0.4), ui_confirm=_segment(0.7))
    node.expression_callback(_request("heard", "t1"))
    node.expression_callback(_request("done", "t2"))
    (skipped,) = _sound_events(node)
    assert (skipped.request_id, skipped.reason) == ("t2", "busy")


def test_queued_request_yields_to_the_operator_voice(node):
    node.sounds.update(error=_segment(1.0), robot_happy=_segment(2.2))
    node.expression_callback(_request("fail"))
    _trigger(node, "robot_happy", "rq-happy")
    node._voice_stream = object()
    node.play_sound_thread(*_Thread.started[0].args)
    assert len(_Thread.started) == 1
    last = _sound_events(node)[-1]
    assert (last.request_id, last.reason) == ("rq-happy", "voice_stream")
    assert node.is_playing is False


def test_queue_limit_is_below_the_play_sound_deadline():
    assert 0 < kn.SOUND_QUEUE_MAX_WAIT_S < kn.ACTION_CLASSES[kn.TOOL_CLASSES["play_sound"]].timeout_s


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


def test_play_sound_request_id_reaches_the_event(node):
    """Тул ``play_sound`` ждёт ``SoundEvent`` со своим ``request_id`` (PR-7 ч.7b, #3698)."""
    node.trigger_callback(types.SimpleNamespace(data='{"sound": "button_click", "request_id": "rq-1"}'))
    node.trigger_callback(types.SimpleNamespace(data='{"sound": "error", "request_id": "rq-2"}'))
    (thread,) = _Thread.started
    assert thread.args == ("button_click", "button_click", "rq-1")
    (skipped,) = _sound_events(node)
    assert (skipped.kind, skipped.reason, skipped.request_id, skipped.sound) == (
        SoundEvent.KIND_SKIPPED, "busy", "rq-2", "error")


def test_trigger_forms():
    assert parse_trigger('{"sound": "ui_confirm", "request_id": "r9"}') == ("ui_confirm", "r9")
    sound, rid = parse_trigger(" thinking ")  # ручной ros2 topic pub / сценарный раннер: id выдаёт владелец
    assert sound == "thinking" and rid
    sound, rid = parse_trigger('{"sound": "error"}')
    assert sound == "error" and rid
    assert parse_trigger("{broken")[0] == "{broken"


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
