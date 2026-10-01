"""Владелец плеера v2 и адаптер Renardo на фейках (ADR-0149 PR-4, эпик #3312).

Детерминированно, без Renardo и ROS: события ``started``/``rejected``, фаза старта,
привязка ``/fail`` к ``track_id``, единственный писатель снимка.
"""

import json
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from rob_box_mcp_tools.engine.player_owner import PlayerOwner
from rob_box_mcp_tools.engine.renardo_adapter import RenardoAdapter, osc_fail_detail
from rob_box_voice.core.music_player_state import parse_music_state

from ._ros_stubs import RosStubs

_ros = RosStubs()
with _ros:
    from rob_box_mcp_tools.tools.music import MusicManager
_ros_stubs = _ros.fixture()

pytestmark = pytest.mark.unit

SLOTS = {"kick": "d1", "bass": "p1"}


def _program(track_id="v2:1:A:aaaa", synths=("bass",), samples=("X",), form_beats=128.0, files=()):
    code = 'd1 >> play("X...")\np1 >> bass([40, None])\n'
    return SimpleNamespace(code=code, track_id=track_id, deck="A", bpm=132, form_beats=form_beats,
                           slots=dict(SLOTS), synths=frozenset(synths), samples=frozenset(samples),
                           sample_files=frozenset(files))


class FakeAdapter:
    def __init__(self, problem=None, error=None):
        self.problem, self.error = problem, error
        self.started, self.stopped = [], 0

    def check(self, program):
        return self.problem

    def start(self, program, on_started):
        if self.error:
            raise self.error
        self.started.append((program, on_started))

    def stop(self):
        self.stopped += 1

    def fire(self, i=-1, phase=0.0, aligned=True):
        program, on_started = self.started[i]
        on_started({"track_id": program.track_id, "deck": "A", "form_beats": program.form_beats,
                    "start_beat": 256.0, "clock_beat": 256.01, "late_beats": 0.01, "bpm": 132.0,
                    "phase_in_form": phase, "players_aligned": aligned, "latency_s": 0.5})


def _owner(adapter):
    states, events = [], []
    now = [1000.0]
    owner = PlayerOwner(adapter, states.append, lambda e: events.append(json.loads(e)),
                        clock=lambda: now[0])
    return owner, states, events, now


def test_started_event_and_playing_state_come_only_from_the_clock_callback():
    adapter = FakeAdapter()
    owner, states, events, _ = _owner(adapter)
    assert owner.play(_program(), dj={"enabled": True, "set_id": "s1"}) == {"ok": True, "track_id": "v2:1:A:aaaa"}
    assert states == [] and events == []  # exec прошёл, но звука ещё нет
    adapter.fire()
    assert [e["event"] for e in events] == ["started"]
    assert events[0]["track_id"] == "v2:1:A:aaaa" and events[0]["phase_in_form"] == 0.0
    assert events[0]["bpm"] == 132.0 and events[0]["deck"] == "A"
    snap = parse_music_state(states[-1])
    assert snap.state == "playing" and snap.track_id == "v2:1:A:aaaa" and snap.dj is True
    assert json.loads(states[-1])["dj"] == {"enabled": True, "set_id": "s1"}


def test_form_ends_at_counts_passes_of_the_looping_form():
    adapter = FakeAdapter()
    owner, states, _, now = _owner(adapter)
    owner.play(_program())
    adapter.fire()
    pass_s = 128 * 60.0 / 132
    assert parse_music_state(states[-1]).form_ends_at == pytest.approx(1000.0 + pass_s)
    now[0] += pass_s * 1.5
    owner.publish_state()
    assert parse_music_state(states[-1]).form_ends_at == pytest.approx(1000.0 + 2 * pass_s)


@pytest.mark.parametrize("problem", [("unknown_synth", "синтов нет на сервере: nosuchsynth"),
                                     ("missing_sample", "нет сэмплов для символов: Q")])
def test_failed_resource_check_is_rejected_loudly_before_exec(problem):
    adapter = FakeAdapter(problem=problem)
    owner, states, events, _ = _owner(adapter)
    result = owner.play(_program())
    assert result["ok"] is False and result["reason"] == problem[0]
    assert adapter.started == []  # до exec не дошло
    assert events == [{"event": "rejected", "track_id": "v2:1:A:aaaa", "ts": 1000.0,
                       "reason": problem[0], "detail": problem[1]}]
    assert states == []  # снимок не врёт «играет»


def test_exec_error_is_rejected():
    owner, _, events, _ = _owner(FakeAdapter(error=NameError("name 'nosuchsynth' is not defined")))
    result = owner.play(_program())
    assert result["reason"] == "exec_error" and "nosuchsynth" in result["detail"]
    assert [e["event"] for e in events] == ["rejected"]


def test_started_of_a_replaced_track_is_ignored():
    adapter = FakeAdapter()
    owner, states, events, _ = _owner(adapter)
    owner.play(_program("t1"))
    owner.play(_program("t2"))
    adapter.fire(0)  # колбэк t1 пришёл после того, как его заменили
    assert events == [] and states == []
    adapter.fire(1)
    assert [e["track_id"] for e in events] == ["t2"]
    assert parse_music_state(states[-1]).track_id == "t2"


def test_server_fail_is_bound_to_the_track_and_rejects_missing_synth_once():
    adapter = FakeAdapter()
    owner, _, events, _ = _owner(adapter)
    owner.play(_program("t1"))
    owner.on_server_fail("/n_set Node 1042 not found")  # служебный отказ — только лог
    assert events == []
    owner.on_server_fail("/s_new SynthDef not found")
    owner.on_server_fail("/s_new SynthDef not found")
    assert [(e["event"], e["track_id"], e["reason"]) for e in events] == [("rejected", "t1", "server_fail")]


def test_stop_publishes_idle_and_resets_dj():
    adapter = FakeAdapter()
    owner, states, _, _ = _owner(adapter)
    owner.play(_program(), dj={"enabled": True})
    adapter.fire()
    assert owner.stop() == {"ok": True, "track_id": "v2:1:A:aaaa"}
    snap = parse_music_state(states[-1])
    assert adapter.stopped == 1 and snap.state == "idle" and snap.dj is False


# --- RenardoAdapter на фейковом клоке --------------------------------------------------------

class FakeClock:
    meter = (4, 4)

    def __init__(self, beat=37.3):
        self.beat, self.bpm, self.latency, self.now_flag = beat, 120.0, 0.25, True
        self.calls, self.scheduled = [], []

    def now(self):
        return self.beat

    def next_bar(self):
        return self.beat + (4 - self.beat % 4)

    def update_tempo_now(self, bpm):
        self.calls.append(("tempo", bpm, self.latency))
        self.bpm = float(bpm)

    def set_time(self, beat):
        self.calls.append(("set_time", beat))
        self.beat = beat

    def schedule(self, fn, beat):
        self.scheduled.append((fn, beat))

    def get_bpm(self):
        return self.bpm


class FakePlayer:
    def __init__(self, clock):
        self.clock, self.event_index, self.stops = clock, None, 0

    def __rshift__(self, _other):
        self.event_index = self.clock.next_bar()
        return self

    def stop(self):
        self.stops += 1


class FakeSamples:
    def getBufferFromSymbol(self, symbol, spack, index=0):
        return SimpleNamespace(bufnum=0 if symbol == "Q" else 7)


def _adapter(known=frozenset({"bass", "pluck"})):
    clock = FakeClock()
    ns = {"Clock": clock, "Samples": FakeSamples(), "play": lambda *a: None, "bass": lambda *a: None,
          "d1": FakePlayer(clock), "p1": FakePlayer(clock)}
    sent = []
    adapter = RenardoAdapter(lambda: ns, lambda: known, lambda *a: sent.append(a))
    return adapter, ns, clock, sent


@pytest.mark.parametrize("program,known,reason", [
    (_program(synths=("nosuchsynth",)), frozenset({"bass"}), "unknown_synth"),
    (_program(samples=("X", "Q", ".")), frozenset({"bass"}), "missing_sample"),
    (_program(), None, "synth_palette_unknown"),
])
def test_adapter_check_rejects_missing_resources(program, known, reason):
    adapter, *_ = _adapter(known)
    assert adapter.check(program)[0] == reason


def test_adapter_check_passes_when_synths_and_buffers_exist():
    adapter, *_ = _adapter()
    assert adapter.check(_program(samples=("X", "-", "."))) is None


def test_adapter_preloads_the_sample_index_the_track_plays():
    """PR-3c: бочка v2 — ``X:12``; грузится буфер файла 12, а не нулевой (иначе первый удар — без буфера)."""
    loaded = []

    class Recording:
        def getBufferFromSymbol(self, symbol, spack, index=0):
            loaded.append((symbol, spack, index))
            return SimpleNamespace(bufnum=0 if (symbol, index) == ("X", 99) else 7)

    adapter, ns, *_ = _adapter()
    ns["Samples"] = Recording()
    assert adapter.check(_program(samples=("X:12", "-"))) is None
    assert sorted(loaded) == [("-", 0, 0), ("X", 0, 12)]
    assert adapter.check(_program(samples=("X:99",))) == ("missing_sample", "нет сэмплов для символов: X:99")


def test_adapter_preloads_loop_files_of_the_program():
    """PR-3d: файлы ``loop()`` (каталог DJ_Dave) грузятся до exec тем же путём, что в программе; нет файла — отказ."""
    loaded = []

    class Recording(FakeSamples):
        def loadBuffer(self, filename, index=0, force=False):
            loaded.append(filename)
            return 0 if "nope" in filename else 12

    adapter, ns, *_ = _adapter()
    ns["Samples"] = Recording()
    assert adapter.check(_program(files=("dj_dave/dirt/psr/009_10.wav",))) is None
    assert loaded == ["../../dj_dave/dirt/psr/009_10.wav"]
    assert adapter.check(_program(files=("dj_dave/nope.wav",)))[0] == "missing_sample"


def test_adapter_starts_form_on_a_multiple_of_form_beats():
    adapter, ns, clock, _ = _adapter()
    got = []
    info = adapter.start(_program(), got.append)
    assert clock.calls == [("tempo", 132, 0.5), ("set_time", 126.0)]  # latency 0.5 до темпа
    assert clock.latency == 0.5 and clock.now_flag is False
    assert info["start_beat"] == 128.0 and info["phase_in_form"] == 0.0 and info["players_aligned"]
    fn, beat = clock.scheduled[-1]
    assert beat == 128.0
    clock.beat = 128.02
    fn()
    assert got[0]["phase_in_form"] == 0.0 and got[0]["late_beats"] == pytest.approx(0.02)
    assert got[0]["bpm"] == 132.0 and got[0]["latency_s"] == 0.5


def test_adapter_restarts_players_of_the_new_program_and_stop_ramps_group():
    adapter, ns, clock, sent = _adapter()
    adapter.start(_program(), lambda s: None)  # слоты могли играть и до v2 — снимаются всегда
    assert ns["d1"].stops == 1 and ns["p1"].stops == 1
    adapter.start(_program("t2"), lambda s: None)
    assert ns["d1"].stops == 2 and ns["p1"].stops == 2  # по разу на слот, без дублей
    with patch("rob_box_mcp_tools.engine.renardo_adapter.time.sleep"):
        adapter.stop()
    assert ns["d1"].stops == 3
    assert [a[0] for a in sent] == ["/n_set", "/g_freeAll"]


def _osc(address, *strings):
    def pad(b):
        return b + b"\x00" * (4 - len(b) % 4)
    return pad(address.encode()) + pad(("," + "s" * len(strings)).encode()) + b"".join(
        pad(s.encode()) for s in strings)


def test_osc_fail_reaches_the_listener_through_music_manager():
    assert osc_fail_detail(_osc("/done", "/b_allocRead")) is None
    assert osc_fail_detail(_osc("/fail", "/s_new", "SynthDef not found")) == "/s_new SynthDef not found"
    mgr = MusicManager.__new__(MusicManager)
    mgr._logger = SimpleNamespace(warning=lambda m: None)
    got = []
    mgr.osc_fail_listener = got.append
    mgr._log_osc_reply(_osc("/fail", "/s_new", "SynthDef not found"))
    assert got == ["/s_new SynthDef not found"]
