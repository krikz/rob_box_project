"""Приглушение музыки под речь (ADR-0150 О8 (а), В16, PR-7d #3701): счётчик реплик, ручки мастер-шины, проводка."""

import pathlib
import re
from types import SimpleNamespace

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub
from rob_box_mcp_tools.engine.speech_duck import SpeechDuck, duck_controls
from rob_box_music import knowledge as kn

from .test_mcp_server import _FakeLogger, _load_mcp_server_module

pytestmark = pytest.mark.unit

REPO = next(p for p in pathlib.Path(__file__).resolve().parents if (p / "docker").is_dir())
SCD = REPO / "docker/sound/voice_assistant/custom_synthdefs/masterfilter.scd"


class _Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def _duck(**kw):
    sent, clock = [], _Clock()
    duck = SpeechDuck(lambda *a: sent.append(a), logger=_FakeLogger(), clock=clock, **kw)
    return duck, sent, clock


def _states(sent):
    return [dict(zip(a[2::2], a[3::2]))["duck"] for a in sent]


def test_speech_ducks_music_and_finish_releases_it_with_the_envelope_knobs():
    duck, sent, _ = _duck()
    duck.on_event("queued", "a")
    duck.on_event("synthesized", "a", 1500)
    assert sent == [], "до started музыка не трогается"
    duck.on_event("started", "a", 1500)
    assert sent == [("/n_set", 999, "duck", 1.0, "duckDb", -9.0, "duckAttack", 0.08, "duckRelease", 0.4)]
    duck.on_event("progress", "a")
    duck.on_event("finished", "a")
    assert _states(sent) == [1.0, 0.0] and not duck.ducked


@pytest.mark.parametrize("end", ["cancelled", "rejected"])
def test_cancelled_or_rejected_speech_releases_too(end):
    duck, sent, _ = _duck()
    duck.on_event("started", "a", 900)
    duck.on_event(end, "a")
    assert _states(sent) == [1.0, 0.0]


def test_overlapping_speech_keeps_music_down_until_the_last_one_ends():
    """Личность и Telegram say_aloud подряд/внахлёст: duck 0 — только когда не звучит ни одна реплика."""
    duck, sent, _ = _duck()
    duck.on_event("started", "a", 2000)
    duck.on_event("started", "b", 2000)
    duck.on_event("finished", "a")
    assert _states(sent) == [1.0] and duck.ducked
    duck.on_event("finished", "b")
    duck.on_event("finished", "b")  # повтор финала — без второго /n_set
    assert _states(sent) == [1.0, 0.0]


def test_final_of_unknown_speech_and_empty_id_do_nothing():
    duck, sent, _ = _duck()
    duck.on_event("finished", "ghost")
    duck.on_event("started", "")
    assert sent == []


def test_lost_final_releases_after_duration_plus_grace_loudly():
    duck, sent, clock = _duck(grace_s=3.0)
    duck.on_event("started", "a", 2000)
    clock.now += 4.9
    duck.tick()
    assert _states(sent) == [1.0], "срок ещё не вышел"
    clock.now += 0.2
    duck.tick()
    assert _states(sent) == [1.0, 0.0]
    assert any("не пришёл" in m for m in duck._log.warning_messages)


def test_unknown_duration_uses_the_long_deadline():
    duck, sent, clock = _duck(grace_s=0.0)
    duck.on_event("started", "a", 0)
    clock.now += 59.0
    duck.tick()
    assert duck.ducked
    clock.now += 2.0
    duck.tick()
    assert not duck.ducked


def test_send_failure_is_logged_and_retried_on_next_transition():
    calls = []

    def flaky(*a):
        calls.append(a)
        if len(calls) == 1:
            raise OSError("scsynth down")

    duck = SpeechDuck(flaky, logger=_FakeLogger())
    duck.on_event("started", "a", 1000)
    assert not duck.ducked
    duck.on_event("started", "b", 1000)
    assert duck.ducked and len(calls) == 2


def test_controls_clamp_depth_down_only_and_convert_ms():
    assert duck_controls(-9, 80, 400) == kn.SPEECH_DUCK
    assert duck_controls(6, 0, -5) == {"duckDb": 0.0, "duckAttack": 0.001, "duckRelease": 0.001}


def _synthdef():
    code = "\n".join(line.split("//", 1)[0] for line in SCD.read_text(encoding="utf-8").splitlines())
    head = code.index("|", code.index("SynthDef.new(\\masterfilter"))
    args = dict(item.split("=") for item in "".join(code[head + 1:code.index("|", head + 1)].split()).split(","))
    return code, args


def test_synthdef_defaults_are_knowledge_and_duck_is_off_by_default():
    _, args = _synthdef()
    assert {k: float(args[k]) for k in kn.SPEECH_DUCK} == dict(kn.SPEECH_DUCK)
    assert float(args["duck"]) == 0.0
    assert "duck" not in kn.MASTER_DEFAULTS, "старт трека не должен снимать приглушение под речью"


def test_duck_is_a_separate_multiplier_after_limiter_gain_and_trim():
    """gain пользователя и trim сета не перезаписываются: duck умножает выход после них (О8 (а))."""
    code, _ = _synthdef()
    gain = code.index("out = out * Lag.kr(gain, lag);")
    trim = code.index("Lag.kr(trimDb.dbamp, trimLag)")
    duck = code.index("duckAmp = Slew.ar(")
    assert gain < trim < duck < code.index("ReplaceOut.ar(")
    assert re.search(r"out = out \* \(duckAmp \* .*duckDb.*\.clip\(-40, 0\)\)\.dbamp;", code)
    assert code.count("gain =") == 1 and "duck" not in code[gain:trim]


class _Node:
    def __init__(self):
        self.params = {"music_duck_db": -6.0, "music_duck_attack_ms": 50.0, "music_duck_release_ms": 300.0}
        self.subs, self.timers = [], []
        self._logger = _FakeLogger()

    def get_parameter(self, name):
        return SimpleNamespace(value=self.params[name])

    def get_logger(self):
        return self._logger

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subs.append((msg_type, topic, callback))

    def create_timer(self, period, callback):
        self.timers.append((period, callback))


def test_mcp_server_wires_speech_events_to_the_master_bus(monkeypatch):
    msgs = install_dialog_msgs_stub()
    module = _load_mcp_server_module(monkeypatch)
    node, sent = _Node(), []
    duck = module._attach_speech_duck(node, SimpleNamespace(send_osc=lambda *a: sent.append(a)))
    assert duck is not None and [s[1] for s in node.subs] == ["/voice/speech/event"]
    assert node.timers[0][1] == duck.tick
    callback = node.subs[0][2]
    callback(SimpleNamespace(kind=msgs.SpeechEvent.KIND_STARTED, speech_id="s1", duration_ms=1200))
    callback(SimpleNamespace(kind=msgs.SpeechEvent.KIND_VOICE_CHANGED, speech_id="s1", duration_ms=0))
    assert duck.ducked
    callback(SimpleNamespace(kind=msgs.SpeechEvent.KIND_FINISHED, speech_id="s1", duration_ms=0))
    assert sent[0] == ("/n_set", 999, "duck", 1.0, "duckDb", -6.0, "duckAttack", 0.05, "duckRelease", 0.3)
    assert _states(sent) == [1.0, 0.0]


def test_no_sound_server_means_no_subscription(monkeypatch):
    module = _load_mcp_server_module(monkeypatch)
    node = _Node()
    assert module._attach_speech_duck(node, None) is None and node.subs == []


def test_node_declares_duck_parameters_from_knowledge():
    src = (REPO / "src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py").read_text(encoding="utf-8")
    for name, key in (("music_duck_db", "duckDb"), ("music_duck_attack_ms", "duckAttack"),
                      ("music_duck_release_ms", "duckRelease")):
        assert re.search(rf'declare_parameter\("{name}", music_kn\.SPEECH_DUCK\["{key}"\]', src), name
