"""ADR-0150 PR-3 (#3688): tts_node - владелец речи (журнал, события, огибающая, отмена по эпохе)."""

from __future__ import annotations

import json
import types
from unittest.mock import MagicMock

import numpy as np
import pytest

from rob_box_core import speech_client
from rob_box_core.msg_stub import install_dialog_msgs_stub
from rob_box_voice.core import speech_compat
from rob_box_voice.core.speech_owner import (
    FRAME_MS,
    SpeechLedger,
    SpeechMeta,
    SpeechOwner,
    envelope_levels,
    plain_text,
    sentence_ends,
)

M = install_dialog_msgs_stub()
_KIND_NAMES = ("queued", "synthesized", "started", "progress", "finished", "cancelled", "voice_changed", "rejected")
KIND = {getattr(M.SpeechEvent, f"KIND_{name.upper()}"): name for name in _KIND_NAMES}
#: Тип ``builtin_interfaces/Time`` той сборки сообщений, что под рукой (rosidl в CI, заглушка без ROS).
TIME = type(M.SpeechEvent().stamp)


class _Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class _Node:
    """Нода-владелец: ровно то, чем ``SpeechOwner`` пользуется у ``tts_node``."""

    def __init__(self):
        self.pubs, self.subs, self.timers = {}, {}, []
        self.chunks, self.finished, self.states = [], [], []
        self.interrupts = 0
        self.ros_now_s = 1_000.0
        self.owner = None

    def create_publisher(self, msg_type, topic, depth):
        self.pubs[topic] = _Pub()
        return self.pubs[topic]

    def create_subscription(self, msg_type, topic, callback, depth):
        self.subs[topic] = callback

    def create_timer(self, period, callback):
        self.timers.append((period, callback))

    def get_clock(self):
        ns = int(self.ros_now_s * 1e9)
        stamp = TIME(sec=ns // 10**9, nanosec=ns % 10**9)
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=ns, to_msg=lambda: stamp))

    def get_logger(self):
        return MagicMock()

    def dialogue_callback(self, msg):
        self.chunks.append(json.loads(msg.data))

    def _interrupt_playback(self):
        self.interrupts += 1

    def publish_state(self, state):
        self.states.append(state)

    def _publish_tts_finished(self, speech_id, *, success, error=None, queued=False, **_):
        """Как в tts_node: финал идёт в журнал владельца и в /voice/tts/finished."""
        self.owner.terminal(speech_id, success, error, queued)
        self.finished.append((speech_id, success, error))


@pytest.fixture
def setup():
    clock = _Clock()
    node = _Node()
    owner = SpeechOwner(node, output_latency_ms=120, ledger=SpeechLedger(clock=clock))
    node.owner = owner
    owner.attach()
    return node, owner, clock


def _events(node):
    return [(KIND[e.kind], e) for e in node.pubs["/voice/speech/event"].sent]


def _kinds(node):
    return [k for k, _ in _events(node)]


def _chunk(text="Раз два. Три четыре.", **meta):
    return {"ssml": f"<speak>{text}</speak>", "_speech": dict(meta)}


def _request(**fields):
    req = M.SpeechRequest()
    req.speech_id, req.text, req.agent, req.commit = "sid-1", "Привет", "personality", True
    for key, value in fields.items():
        setattr(req, key, value)
    return req


# -- чистые функции -----------------------------------------------------------


def test_plain_text_and_sentence_ends():
    assert plain_text("<speak>x &lt; y  <break/> z</speak>") == "x < y z"
    assert sentence_ends("Раз два. Три четыре!") == (8, 20)
    assert sentence_ends("без точки") == (9,)


def test_envelope_levels_rms_by_40ms_windows():
    sr = 16000
    tone = np.full((sr, 2), 0.5, dtype=np.float32)  # 1 с стерео, |x| = 0.5
    levels = envelope_levels(tone, sr)
    assert len(levels) * FRAME_MS == 1000
    assert levels == [0.5] * 25
    assert envelope_levels(np.zeros(sr // 2), sr) == [0.0] * 13  # 0.5 с → ceil(12.5)


# -- вход ---------------------------------------------------------------------


def test_request_goes_into_tts_pipeline_with_meta(setup):
    node, owner, _ = setup
    node.subs["/voice/speech/request"](_request(epoch=4, turn_id="t", voice="ermil", priority=10))
    (chunk,) = node.chunks
    assert chunk["ssml"] == "<speak>Привет</speak>" and chunk["speech_id"] == "sid-1"
    assert (chunk["sink"], chunk["priority"], chunk["voice"]) == ("speaker", "operator", "ermil")
    assert chunk["_speech"]["epoch"] == 4 and chunk["_speech"]["turn_id"] == "t"


def test_request_without_speech_id_gets_one(setup):
    node, _, _ = setup
    node.subs["/voice/speech/request"](_request(speech_id=""))
    assert len(node.chunks[0]["speech_id"]) == 36


@pytest.mark.parametrize(
    "fields, reason",
    [
        ({"sink": "headset"}, "sink_unsupported"),
        ({"at": 1234.5}, "timing_unsupported"),
        ({"boundary": 1}, "timing_unsupported"),
        ({"commit": False}, "timing_unsupported"),
    ],
)
def test_unsupported_request_is_rejected_loudly(setup, fields, reason):
    """Чего владелец ещё не исполняет - честный rejected, а не тихое «сыграл как смог»."""
    node, _, _ = setup
    node.subs["/voice/speech/request"](_request(**fields))
    assert node.chunks == []
    ((kind, ev),) = _events(node)
    assert (kind, ev.reason, ev.speech_id) == ("rejected", reason, "sid-1")


# -- жизнь реплики ---------------------------------------------------------------


def test_lifecycle_events_and_envelope(setup):
    node, owner, clock = setup
    owner.admit(_chunk(agent="personality", epoch=2), "s1")
    owner.synthesized("s1", 1.0, {"used_provider": "yandex", "used_voice": "anton"}, "ermil")
    assert owner.started("s1", np.full((16000, 2), 0.25, dtype=np.float32), 16000)
    clock.t += 1.0
    node._publish_tts_finished("s1", success=True)

    kinds = _kinds(node)
    assert kinds == ["queued", "synthesized", "voice_changed", "started", "finished"]
    evs = dict(_events(node))
    assert evs["queued"].text_len == 20 and evs["queued"].epoch == 2
    assert (evs["synthesized"].duration_ms, evs["synthesized"].provider) == (1000, "yandex")
    assert (evs["voice_changed"].voice_requested, evs["voice_changed"].voice_applied) == ("ermil", "anton")
    assert evs["finished"].spoken_chars == evs["finished"].text_len == 20
    (env,) = node.pubs["/voice/speech/envelope"].sent
    assert env.speech_id == "s1" and len(env.levels) * env.frame_ms == evs["started"].duration_ms
    assert (env.t0.sec, env.t0.nanosec) == (1000, 120_000_000)  # now + output_latency_ms
    assert owner.ledger.get("s1") is None


def test_cancel_by_epoch_spares_newer_epoch_and_other_agents(setup):
    node, owner, _ = setup
    owner.admit(_chunk(agent="personality", epoch=1), "old")
    owner.admit(_chunk(agent="personality", epoch=2), "new-session-ack")
    owner.admit(_chunk(agent="operator", epoch=1), "tars")
    owner.admit(_chunk(agent="personality", epoch=0, interruptible=False), "keep")

    node.subs["/voice/speech/cancel"](M.SpeechCancel(epoch=1, agent="personality", reason="barge_in"))

    assert owner.is_cancelled("old")
    assert not owner.is_cancelled("new-session-ack")
    assert not owner.is_cancelled("tars")
    assert not owner.is_cancelled("keep")  # непрерываемую отмена по эпохе не трогает
    assert node.interrupts == 0  # ничего не играет - глушить нечего


def test_cancel_playing_stops_now_and_reports_spoken_chars(setup):
    node, owner, clock = setup
    owner.admit(_chunk(agent="personality", epoch=3), "s")
    owner.started("s", np.zeros((32000, 2), dtype=np.float32), 16000)  # 2 с
    clock.t += 1.0
    node.subs["/voice/speech/cancel"](M.SpeechCancel(epoch=3, agent="personality"))
    assert node.interrupts == 1
    node._publish_tts_finished("s", success=False, error="stopped")
    kind, ev = _events(node)[-1]
    assert kind == "cancelled" and ev.spoken_chars == 10 < ev.text_len == 20


def test_cancel_by_speech_id_hits_even_non_interruptible(setup):
    node, owner, _ = setup
    owner.admit(_chunk(interruptible=False), "s")
    node.subs["/voice/speech/cancel"](M.SpeechCancel(speech_id="s", epoch=0, agent="personality"))
    assert owner.is_cancelled("s")


def test_progress_once_per_sentence(setup):
    node, owner, clock = setup
    tick = node.timers[0][1]
    owner.admit(_chunk("Раз. Два. Три."), "s")
    owner.started("s", np.zeros((48000, 2), dtype=np.float32), 16000)  # 3 с на 14 символов
    for _ in range(3):
        clock.t += 1.2
        tick()
    progress = [ev.spoken_chars for kind, ev in _events(node) if kind == "progress"]
    assert progress == [4, 9]  # концы первых двух предложений; последнее - finished


# -- адаптер v1 -------------------------------------------------------------------


def test_legacy_meta_reads_agent_and_epoch():
    assert SpeechMeta.from_chunk({"ssml": "x", "epoch": 7}) == SpeechMeta(agent="personality", epoch=7)
    assert SpeechMeta.from_chunk({"priority": "operator"}).agent == "operator"
    assert SpeechMeta.from_chunk({"agent": "telegram", "epoch": "bad"}) == SpeechMeta(agent="telegram")


def test_legacy_stop_cancels_everything_and_projects_state(setup):
    node, owner, _ = setup
    node._speech = owner
    owner.admit(_chunk(agent="operator", epoch=9), "s")
    speech_compat.on_control(node, types.SimpleNamespace(data="STOP"))
    assert owner.is_cancelled("s") and node.interrupts == 1 and node.states == ["stopped"]


def test_legacy_ignore_stop_ms_is_not_accepted(setup):
    node, owner, _ = setup
    node._speech = owner
    speech_compat.on_control(node, types.SimpleNamespace(data="IGNORE_" + "STOP_MS:700"))
    assert node.interrupts == 0 and node.states == []


# -- проводка в tts_node --------------------------------------------------------------


def _bare_tts(owner):
    from rob_box_voice.tts_node import TTSNode

    n = object.__new__(TTSNode)
    n._speech = owner
    n.finished_pub = MagicMock()
    n.get_logger = lambda: MagicMock()
    return n


def test_tts_finished_projection_and_event_come_from_one_call(setup):
    node, owner, _ = setup
    owner.admit(_chunk(), "s")
    n = _bare_tts(owner)
    n._publish_tts_finished("s", success=True, duration_sec=1.0)
    assert json.loads(n.finished_pub.publish.call_args.args[0].data)["success"] is True
    assert _kinds(node)[-1] == "finished"


def test_tts_fifo_gate_drops_cancelled_speech_without_sound(setup):
    node, owner, _ = setup
    owner.admit(_chunk(), "s")
    node.subs["/voice/speech/cancel"](M.SpeechCancel(speech_id="s", agent="personality"))
    n = _bare_tts(owner)
    assert n._sap_wait_fifo_and_dialogue(None, "s", None) is False
    assert json.loads(n.finished_pub.publish.call_args.args[0].data)["error"] == "cancelled"
    assert _kinds(node)[-1] == "cancelled"


def test_epoch_watch_feeds_barge_in_cancel(setup):
    """stt_node: отмена «того, что звучит» по эпохе из событий владельца."""
    node, owner, _ = setup
    watch = speech_client.EpochWatch("personality")
    owner.admit(_chunk(agent="personality", epoch=4), "s")
    for ev in node.pubs["/voice/speech/event"].sent:
        watch.on_event(ev)
    assert watch.epoch == 4
