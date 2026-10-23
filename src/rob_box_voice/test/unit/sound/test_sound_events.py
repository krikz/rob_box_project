"""ADR-0150 PR-3b (#3690): звук отвечает событием на каждый запрос - молчаливого пропуска нет (V33, A23)."""

import json
import shutil
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
    assert set(EARCON_SOUNDS) == set(kn.EARCONS) | {v for v in kn.SILENCED_EARCONS.values() if v}


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


# ── тишина (#3984, решение Шифу 23.10): вместо пик-пик - тихий приглушённый звук ──────────────────────────────


def _session(*scene_kinds, agent="personality"):
    return types.SimpleNamespace(agent=agent, scenes=[types.SimpleNamespace(kind=k) for k in scene_kinds])


def _sound_pack():
    import pathlib

    return next(p for p in pathlib.Path(__file__).resolve().parents if (p / "sound_pack").is_dir()) / "sound_pack"


def test_muffled_sound_file_lies_next_to_the_other_earcons_and_is_a_distinct_sound():
    pack = _sound_pack()
    assert (pack / "muffled.mp3").is_file() and (pack / "make_muffled.py").is_file()
    assert kn.SILENCED_EARCONS["heard"] == "muffled" and EARCON_SOUNDS["muffled"] == "muffled"
    assert EARCON_SOUNDS["muffled"] != EARCON_SOUNDS["heard"]
    assert "muffled" not in kn.EARCONS                      # не выбор LLM (enum схемы команды)
    assert set(kn.SILENCED_EARCONS) <= set(kn.EARCONS)


@pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="ffmpeg нужен только чтобы измерить файл")
def test_muffled_sound_is_short_and_quieter_than_the_heard_click():
    import re
    import subprocess

    def probe(name):
        return subprocess.run(["ffmpeg", "-hide_banner", "-i", str(_sound_pack() / name), "-af", "volumedetect", "-f",
                               "null", "-"], capture_output=True, text=True).stderr

    def peak(text):
        return float(re.search(r"max_volume: (-?[\d.]+) dB", text).group(1))

    def seconds(text):
        h, m, sec = re.search(r"Duration: (\d+):(\d+):([\d.]+)", text).groups()
        return int(h) * 3600 + int(m) * 60 + float(sec)

    muffled, click = probe("muffled.mp3"), probe("button_click.mp3")
    assert seconds(muffled) <= 0.3 and peak(muffled) < peak(click) - 6.0


def test_in_silence_heard_plays_the_muffled_sound_not_the_click_and_unsilence_brings_the_click_back(node):
    node.sounds["muffled"] = _segment()
    node.session_callback(_session("Silence"))
    node.expression_callback(_request("heard", "t1"))
    (thread,) = _Thread.started
    assert thread.args[:2] == ("muffled", "muffled") and thread.args[2] == "t1"
    (answer,) = _expression_events(node)
    assert (answer.kind, answer.reason) == (ExpressionEvent.KIND_STARTED, "")
    node.release_channel()
    node.session_callback(_session())                       # «Робот, говори»: сцены Silence больше нет
    node.expression_callback(_request("heard", "t2"))
    assert _Thread.started[-1].args[:2] == ("button_click", "button_click")


def test_in_silence_done_earcon_is_skipped_with_a_reason_and_fail_is_muffled(node):
    node.sounds["muffled"] = _segment()
    node.session_callback(_session("Silence"))
    node.expression_callback(_request("done", "t1"))
    assert _Thread.started == []
    (answer,) = _expression_events(node)
    assert (answer.kind, answer.reason) == (ExpressionEvent.KIND_SKIPPED, "silenced")
    node.expression_callback(_request("fail", "t2"))
    assert _Thread.started[-1].args[:2] == ("muffled", "muffled")


def test_foreign_agent_session_does_not_change_the_silence(node):
    node.session_callback(_session("Silence"))
    node.session_callback(_session(agent="telegram"))
    assert node._silenced is True


def test_scenario_silence_then_a_phrase_then_robot_speak_ring_and_sound_follow_the_session(node):
    """Сквозной сценарий #3984 на реальных контрактах: один и тот же ``SessionState`` ведёт кольцо и звук.

    вход в тишину -> анимация кольца «тишина»; реплика («Робби, как дела?» -> ``addressed``) -> звук ``muffled`` и
    кольцо стадию не показывает; «Робот, говори» (сцена снята) -> кольцо в покое, «услышал» снова обычный."""
    from rob_box_voice.core import ring_reflex as rr

    class _Ring:
        calls = []

        def __getattr__(self, name):
            return lambda *args: (self.calls.append((name, *args)) or True)

    timers = []
    ring_reflex = rr.RingReflex(_Ring(), lambda delay, fn: timers.append(fn) or types.SimpleNamespace(cancel=lambda: None),
                                lambda *answer: None)

    def session(msg):
        node.session_callback(msg)
        ring_reflex.on_state("silenced" if node._silenced else "idle")

    node.sounds["muffled"] = _segment()
    session(_session("Silence"))
    assert _Ring.calls == [("mono", *rr.SILENCE_FRAMES[0])] and ring_reflex.mode == "silence"
    assert ring_reflex.on_turn_event("t1", "addressed", "started") is None   # стадия кольцо не трогает
    node.expression_callback(_request("heard", "t1"))
    assert _Thread.started[-1].args[0] == "muffled" and _Thread.started[-1].args[0] != "button_click"
    node.release_channel()
    session(_session())
    assert _Ring.calls[-1] == ("trace",) and ring_reflex.mode == rr.REST
    assert ring_reflex.on_turn_event("t2", "addressed", "started") == "doa_sector"
    node.expression_callback(_request("heard", "t2"))
    assert _Thread.started[-1].args[0] == "button_click"
