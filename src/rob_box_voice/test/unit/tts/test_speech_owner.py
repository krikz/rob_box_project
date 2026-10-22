"""ADR-0150 PR-3 (#3688): tts_node - владелец речи (журнал, события, огибающая, отмена по эпохе)."""

from __future__ import annotations

import json
import threading
import time
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


def test_request_to_headset_is_rejected_loudly(setup):
    """Чего владелец не исполняет - честный rejected, а не тихое «сыграл как смог»."""
    node, _, _ = setup
    node.subs["/voice/speech/request"](_request(sink="headset"))
    assert node.chunks == []
    ((kind, ev),) = _events(node)
    assert (kind, ev.reason, ev.speech_id) == ("rejected", "sink_unsupported", "sid-1")


def test_timing_fields_travel_with_the_chunk(setup):
    node, _, _ = setup
    node.subs["/voice/speech/request"](_request(at=1234.5, boundary=1))
    meta = node.chunks[0]["_speech"]
    assert (meta["at"], meta["boundary"], meta["commit"]) == (1234.5, 1, True)


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


def test_boundary_sentence_cancel_waits_for_sentence_end(setup):
    node, owner, clock = setup
    tick = node.timers[0][1]
    owner.admit(_chunk(agent="personality", boundary=1), "s")
    owner.started("s", np.zeros((32000, 2), dtype=np.float32), 16000)  # 2 с на 20 символов
    clock.t += 0.2  # 2 символа сказано
    node.subs["/voice/speech/cancel"](M.SpeechCancel(epoch=0, agent="personality"))
    tick()
    assert node.interrupts == 0  # до конца первого предложения (8 символов = 0.8 с) ещё звучит
    clock.t += 0.7
    tick()
    assert node.interrupts == 1
    node._publish_tts_finished("s", success=False, error="stopped")
    kind, ev = _events(node)[-1]
    assert (kind, ev.spoken_chars) == ("cancelled", 8)


def test_output_latency_has_one_source():
    """Задержка вывода — одно число (knowledge): tts_node берёт из него дефолт, yaml его не переписывает (PR-7c)."""
    import pathlib

    root = pathlib.Path(__file__).resolve().parents[5]
    tts_node = (root / "src/rob_box_voice/rob_box_voice/tts_node.py").read_text(encoding="utf-8")
    assert 'declare_parameter("output_latency_ms", SPEECH_OUTPUT_LATENCY_MS)' in tts_node
    for yaml in root.glob("**/config/**/tts_node.yaml"):
        if "node_modules" not in yaml.parts:
            assert "output_latency_ms:" not in yaml.read_text(encoding="utf-8"), yaml


def test_cancel_now_overrides_sentence_boundary(setup):
    """«Стой» (ADR-0150 PR-7c): реплика просила границу предложения, отмена с ``boundary=now`` режет сразу."""
    node, owner, clock = setup
    owner.admit(_chunk(agent="personality", boundary=1), "s")
    owner.started("s", np.zeros((32000, 2), dtype=np.float32), 16000)
    clock.t += 0.2
    node.subs["/voice/speech/cancel"](M.SpeechCancel(speech_id="s", epoch=0, agent="personality",
                                                     boundary=M.SpeechCancel.BOUNDARY_NOW))
    assert node.interrupts == 1  # не ждём конца предложения


def test_cancel_as_requested_keeps_the_sentence_boundary(setup):
    node, owner, clock = setup
    owner.admit(_chunk(agent="personality", boundary=1), "s")
    owner.started("s", np.zeros((32000, 2), dtype=np.float32), 16000)
    node.subs["/voice/speech/cancel"](M.SpeechCancel(speech_id="s", epoch=0, agent="personality",
                                                     boundary=M.SpeechCancel.BOUNDARY_AS_REQUESTED))
    assert node.interrupts == 0


def test_client_cancel_carries_the_boundary(setup):
    node, _, _ = setup
    speech_client.cancel(node, agent="personality", speech_id="s", boundary="now")
    speech_client.cancel(node, agent="personality", speech_id="s")
    sent = node.pubs["/voice/speech/cancel"].sent
    assert [m.boundary for m in sent] == [M.SpeechCancel.BOUNDARY_NOW, M.SpeechCancel.BOUNDARY_AS_REQUESTED]
    with pytest.raises(ValueError):
        speech_client.cancel(node, agent="personality", speech_id="s", boundary="soon")


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


def test_at_waits_until_moment_minus_output_latency(setup):
    node, owner, _ = setup  # output_latency_ms=120 в фикстуре
    owner.admit(_chunk(at=node.ros_now_s + 0.42), "s")
    t0 = time.monotonic()
    assert owner.started("s", np.zeros((160, 2)), 16000)
    assert 0.25 <= time.monotonic() - t0 < 0.45  # 0.42 - 0.12 = 0.30 с
    assert "started" in _kinds(node)


def test_cancel_during_wait_for_at_returns_false(setup):
    node, owner, _ = setup
    owner.admit(_chunk(at=node.ros_now_s + 5.0), "s")
    result = []
    worker = threading.Thread(target=lambda: result.append(owner.started("s", np.zeros((160, 2)), 16000)))
    worker.start()
    time.sleep(0.05)
    node.subs["/voice/speech/cancel"](M.SpeechCancel(speech_id="s", agent="personality"))
    worker.join(timeout=2.0)
    assert result == [False]
    assert "started" not in _kinds(node)


def test_at_in_the_past_plays_at_once(setup):
    _, owner, _ = setup
    owner.admit(_chunk(at=1.0), "s")
    t0 = time.monotonic()
    assert owner.started("s", np.zeros((160, 2)), 16000)
    assert time.monotonic() - t0 < 0.2


def _presynth_node(node):
    node._synthesis_executor = types.SimpleNamespace(submit=lambda fn, *a: fn(*a))
    node._extract_text_from_ssml = plain_text
    node._parse_ssml_attributes = lambda ssml: {}

    def chain(voice, language, attrs, text, sink, ctx):
        ctx.update(audio_np=np.zeros(8000, dtype=np.float32), sample_rate=16000, used_provider="minimax")

    node._sap_run_provider_chain = chain


def test_presynth_commit_false_holds_audio_until_commit(setup):
    node, owner, _ = setup
    _presynth_node(node)
    node.subs["/voice/speech/request"](_request(commit=False))

    assert node.chunks == []  # не играем
    assert _kinds(node) == ["queued", "synthesized"]
    assert dict(_events(node))["synthesized"].duration_ms == 500
    pregen = []
    held = owner.claim_presynth("sid-1", pregen.append)
    assert held["sample_rate"] == 16000 and held["audio_np"].size == 8000 and pregen == []
    assert owner.claim_presynth("sid-1", lambda sid: "pregen") == "pregen"  # дальше - обычный pregen


def test_cancel_drops_held_presynth(setup):
    """Сегмент выступления заменён правкой: отмена по ``speech_id`` выбрасывает его готовый PCM (не держим вечно)."""
    node, owner, _ = setup
    _presynth_node(node)
    node.subs["/voice/speech/request"](_request(commit=False))
    node.subs["/voice/speech/cancel"](M.SpeechCancel(speech_id="sid-1", epoch=0, agent="personality"))
    assert owner.claim_presynth("sid-1", lambda sid: "pregen") == "pregen"


def test_cancel_during_presynth_drops_the_result_and_says_cancelled(setup):
    node, owner, _ = setup
    _presynth_node(node)
    started = []
    node._synthesis_executor = types.SimpleNamespace(submit=lambda fn, *a: started.append((fn, a)))
    node.subs["/voice/speech/request"](_request(commit=False))
    node.subs["/voice/speech/cancel"](M.SpeechCancel(speech_id="sid-1", epoch=0, agent="personality"))
    fn, args = started[0]
    fn(*args)  # синтез закончился уже после отмены
    assert _kinds(node) == ["queued", "cancelled"]
    assert owner.claim_presynth("sid-1", lambda sid: "pregen") == "pregen"


def test_presynth_then_commit_true_goes_to_pipeline(setup):
    node, owner, _ = setup
    _presynth_node(node)
    node.subs["/voice/speech/request"](_request(commit=False))
    node.subs["/voice/speech/request"](_request(commit=True, at=node.ros_now_s + 1.0))
    (chunk,) = node.chunks
    assert chunk["speech_id"] == "sid-1" and chunk["_speech"]["at"] == node.ros_now_s + 1.0
    assert owner.claim_presynth("sid-1", lambda sid: None)["audio_np"].size == 8000


def test_presynth_in_streaming_mode_is_rejected(setup):
    node, owner, _ = setup
    _presynth_node(node)
    node.minimax_streaming = True
    node.subs["/voice/speech/request"](_request(commit=False))
    assert _kinds(node) == ["queued", "rejected"]
    assert _events(node)[-1][1].reason == "presynth_unavailable"


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


def test_tts_worker_skips_synthesis_of_cancelled_speech(setup):
    """Отменённая в очереди реплика не идёт в (платный) синтез."""
    node, owner, _ = setup
    owner.admit(_chunk(), "s")
    node.subs["/voice/speech/cancel"](M.SpeechCancel(speech_id="s", agent="personality"))
    n = _bare_tts(owner)
    n._synthesize_and_play = MagicMock()
    n._run_synthesis_worker("<speak>x</speak>", "x", None, {}, "s", None, None, None, play_seq=None)
    n._synthesize_and_play.assert_not_called()
    assert _kinds(node)[-1] == "cancelled"


def test_epoch_watch_feeds_barge_in_cancel(setup):
    """stt_node: отмена «того, что звучит» по эпохе из событий владельца."""
    node, owner, _ = setup
    watch = speech_client.EpochWatch("personality")
    owner.admit(_chunk(agent="personality", epoch=4), "s")
    for ev in node.pubs["/voice/speech/event"].sent:
        watch.on_event(ev)
    assert watch.epoch == 4
