"""Голос человека как второй источник приглушения музыки (#4155): ``/audio/vad`` → тот же ``SpeechDuck``.

Просьба Шифу 28.10: «когда кто-то говорит, музыку сразу чуть тише». Счётчик реплик робота и ручка ``duck`` — те же, что
в ``test_speech_duck.py``; здесь — подъём/спад VAD, удержание, эхо робота, «музыка играет», проводка в ``mcp_server``.
"""

import re
import sys
from types import SimpleNamespace

import pytest

from rob_box_core.msg_stub import install_dialog_msgs_stub
from rob_box_dialog import knowledge as dialog_kn
from rob_box_mcp_tools.engine.speech_duck import TICK_S, SpeechDuck, VoiceGate, VoiceSource, duck_controls
from rob_box_music import knowledge as kn

from .test_mcp_server import _FakeLogger, _load_mcp_server_module
from .test_speech_duck import REPO, _Clock, _Node, _states

pytestmark = pytest.mark.unit


class _Music:
    """``music_playing`` источника: играет ли дека."""

    def __init__(self, playing=True):
        self.playing = playing

    def __call__(self):
        return self.playing


def _vduck(playing=True, **kw):
    sent, clock, music = [], _Clock(), _Music(playing)
    duck = SpeechDuck(lambda *a: sent.append(a), logger=_FakeLogger(), clock=clock,
                      voice=VoiceSource(music_playing=music), **kw)
    return duck, sent, clock, music


def _run(duck, clock, seconds):
    """Таймер ноды: ``tick`` каждые ``TICK_S``."""
    for _ in range(round(seconds / TICK_S)):
        clock.now += TICK_S
        duck.tick()


def _msg(on, controls):
    args = []
    for name, value in controls.items():
        args += [name, float(value)]
    return ("/n_set", 999, "duck", 1.0 if on else 0.0, *args)


VOICE_ON, VOICE_OFF = _msg(True, kn.VOICE_DUCK), _msg(False, kn.VOICE_DUCK)
SPEECH_ON, SPEECH_OFF = _msg(True, kn.SPEECH_DUCK), _msg(False, kn.SPEECH_DUCK)
VOICE_LINES = ("голос человека — музыка тише", "голос человека — возврат")


def _voice_logs(duck):
    return [m for m in duck._log.info_messages if "голос человека" in m]


def test_voice_rise_ducks_the_music_at_once_and_the_fall_releases_it_after_the_hold():
    duck, sent, clock, _ = _vduck()
    duck.on_vad(True)
    assert sent == [VOICE_ON] and duck.ducked, "подъём VAD — duck 1 сразу, не по тику"
    clock.now += 0.4
    duck.on_vad(False)
    assert sent == [VOICE_ON], "спад ещё не отпускает: впереди удержание"
    _run(duck, clock, kn.VOICE_DUCK_HOLD_S - 0.25)
    assert sent == [VOICE_ON] and duck.ducked
    _run(duck, clock, 0.5)
    assert sent == [VOICE_ON, VOICE_OFF] and not duck.ducked
    _run(duck, clock, 3.0)
    assert sent == [VOICE_ON, VOICE_OFF], "отпущено один раз, дальше тики молчат"


def test_voice_duck_is_softer_than_robot_speech_and_has_envelope_knobs_of_its_own():
    assert kn.SPEECH_DUCK["duckDb"] < kn.VOICE_DUCK["duckDb"] < 0.0
    assert duck_controls(kn.VOICE_DUCK["duckDb"], kn.VOICE_DUCK["duckAttack"] * 1000,
                         kn.VOICE_DUCK["duckRelease"] * 1000) == kn.VOICE_DUCK
    assert VOICE_ON[4:] != SPEECH_ON[4:]


def test_vad_chatter_inside_the_hold_does_not_touch_the_knob_and_logs_only_the_edges():
    """На сплошной речи 54–75 фронтов за 20 с (ADR-0137): паузы короче удержания ручку не дёргают."""
    duck, sent, clock, _ = _vduck()
    duck.on_vad(True)
    for _ in range(40):
        clock.now += 0.1
        duck.on_vad(False)
        clock.now += 0.3
        duck.tick()
        duck.on_vad(True)
    assert sent == [VOICE_ON]
    duck.on_vad(False)
    _run(duck, clock, kn.VOICE_DUCK_HOLD_S + 0.5)
    assert sent == [VOICE_ON, VOICE_OFF]
    logs = _voice_logs(duck)
    assert len(logs) == 2 and VOICE_LINES[0] in logs[0] and logs[1].endswith(VOICE_LINES[1])


def test_a_new_burst_inside_the_hold_cancels_the_release_and_the_hold_restarts_after_it():
    duck, sent, clock, _ = _vduck()
    duck.on_vad(True)
    duck.on_vad(False)
    _run(duck, clock, 1.0)
    duck.on_vad(True)  # подъём внутри удержания
    _run(duck, clock, 5.0)  # всплеск идёт — музыка тише, сколько бы он ни длился (в пределах VOICE_DUCK_MAX_S)
    assert sent == [VOICE_ON] and duck.ducked
    duck.on_vad(False)
    _run(duck, clock, kn.VOICE_DUCK_HOLD_S + 0.5)
    assert sent == [VOICE_ON, VOICE_OFF]


def test_robot_speech_and_human_voice_together_release_only_when_both_are_over():
    duck, sent, clock, _ = _vduck()
    duck.on_vad(True)  # человек начал
    duck.on_event("started", "a", 6000)  # робот заговорил поверх: глубже, ручки речи
    assert sent == [VOICE_ON, SPEECH_ON]
    duck.on_vad(False)
    _run(duck, clock, kn.VOICE_DUCK_HOLD_S + 0.5)  # человек замолчал и удержание вышло, а робот говорит
    assert sent == [VOICE_ON, SPEECH_ON] and duck.ducked
    duck.on_event("finished", "a")
    assert sent == [VOICE_ON, SPEECH_ON, SPEECH_OFF] and not duck.ducked


def test_robot_ends_while_the_human_still_talks_music_stays_down_at_the_voice_depth():
    duck, sent, clock, _ = _vduck()
    duck.on_vad(True)
    duck.on_event("started", "a", 3000)
    duck.on_event("finished", "a")
    assert sent == [VOICE_ON, SPEECH_ON, VOICE_ON], "речь кончилась, человек говорит: duck 0 нет, глубина голоса"
    duck.on_vad(False)
    _run(duck, clock, kn.VOICE_DUCK_HOLD_S + 0.5)
    assert sent[-1] == VOICE_OFF and not duck.ducked


def test_vad_while_the_robot_speaks_is_its_own_echo_not_a_voice():
    duck, sent, clock, _ = _vduck()
    duck.on_event("started", "a", 3000)
    duck.on_vad(True)
    clock.now += 0.5
    duck.on_vad(False)
    duck.on_event("finished", "a")  # сразу после эха: удержание голоса не должно было начаться
    assert sent == [SPEECH_ON, SPEECH_OFF], "эхо не продлевает приглушение речи робота на удержание"
    _run(duck, clock, kn.VOICE_DUCK_HOLD_S + 1.0)
    assert sent == [SPEECH_ON, SPEECH_OFF] and _voice_logs(duck) == []


def test_vad_in_the_echo_window_after_the_robot_speech_is_echo_too():
    duck, sent, clock, _ = _vduck()
    duck.on_event("started", "a", 3000)
    duck.on_event("finished", "a")
    clock.now += dialog_kn.IDENTITY_MIC_GRACE_S - 0.5
    duck.on_vad(True)
    clock.now += 0.3
    duck.on_vad(False)
    _run(duck, clock, 5.0)
    assert sent == [SPEECH_ON, SPEECH_OFF], "хвост эха после реплики музыку под голос не глушит"


def test_rejected_speech_that_never_sounded_opens_no_echo_window():
    duck, sent, clock, _ = _vduck()
    duck.on_event("rejected", "ghost")
    duck.on_vad(True)
    assert sent == [VOICE_ON]


def test_voice_that_goes_on_after_the_echo_window_is_picked_up_by_tick():
    duck, sent, clock, _ = _vduck()
    duck.on_event("started", "a", 3000)
    duck.on_event("finished", "a")
    clock.now += 1.0
    duck.on_vad(True)  # человек заговорил в эхо-окне и не умолк
    _run(duck, clock, dialog_kn.IDENTITY_MIC_GRACE_S - 1.0 - 0.25)
    assert sent == [SPEECH_ON, SPEECH_OFF]
    _run(duck, clock, 0.5)  # окно вышло, VAD всё ещё поднят — это уже человек
    assert sent == [SPEECH_ON, SPEECH_OFF, VOICE_ON]


def test_vad_without_playing_music_is_a_noop():
    duck, sent, clock, _ = _vduck(playing=False)
    duck.on_vad(True)
    _run(duck, clock, 2.0)
    duck.on_vad(False)
    _run(duck, clock, 3.0)
    assert sent == [] and not duck.ducked and _voice_logs(duck) == []


def test_music_that_starts_during_a_burst_is_ducked_by_the_next_tick():
    duck, sent, clock, music = _vduck(playing=False)
    duck.on_vad(True)
    music.playing = True
    _run(duck, clock, TICK_S)
    assert sent == [VOICE_ON]


def test_music_that_stops_during_the_duck_still_gets_the_release():
    duck, sent, clock, music = _vduck()
    duck.on_vad(True)
    music.playing = False
    duck.on_vad(False)
    _run(duck, clock, kn.VOICE_DUCK_HOLD_S + 0.5)
    assert sent == [VOICE_ON, VOICE_OFF], "ручку не оставляем опущенной, если музыку сняли посреди голоса"


def test_lost_vad_fall_does_not_keep_the_music_down_forever_and_says_so():
    duck, sent, clock, _ = _vduck()
    duck.on_vad(True)
    _run(duck, clock, kn.VOICE_DUCK_MAX_S - 1.0)
    assert sent == [VOICE_ON] and duck._log.warning_messages == []
    _run(duck, clock, 1.5)
    assert sent == [VOICE_ON, VOICE_OFF] and any("спад /audio/vad не пришёл" in m for m in duck._log.warning_messages)
    duck.on_vad(False)  # запоздалый спад — не событие
    assert sent == [VOICE_ON, VOICE_OFF]


def test_without_a_voice_source_vad_does_nothing():
    sent = []
    duck = SpeechDuck(lambda *a: sent.append(a), logger=_FakeLogger(), clock=_Clock())
    duck.on_vad(True)
    duck.tick()
    assert sent == [] and not duck.ducked


def test_voice_send_failure_is_logged_and_the_next_burst_tries_again():
    calls = []

    def flaky(*a):
        calls.append(a)
        if len(calls) == 1:
            raise OSError("scsynth down")

    clock = _Clock()
    duck = SpeechDuck(flaky, logger=_FakeLogger(), clock=clock, voice=VoiceSource(music_playing=lambda: True))
    duck.on_vad(True)
    assert not duck.ducked and any("не отправлен" in m for m in duck._log.warning_messages)
    duck.on_vad(False)
    _run(duck, clock, kn.VOICE_DUCK_HOLD_S + 0.5)
    duck.on_vad(True)
    assert duck.ducked and len(calls) == 2


def test_voice_source_defaults_are_knowledge_and_the_gate_decides_by_time_and_permission():
    source = VoiceSource(music_playing=lambda: True)
    assert (dict(source.controls), source.hold_s, source.max_s, source.echo_grace_s) == (
        dict(kn.VOICE_DUCK), kn.VOICE_DUCK_HOLD_S, kn.VOICE_DUCK_MAX_S, dialog_kn.IDENTITY_MIC_GRACE_S)
    gate = VoiceGate(0.5, 5.0)
    gate.edge(True, 0.0)
    assert gate.pending and not gate.step(0.0, allowed=False)
    assert gate.step(0.1, allowed=True) and gate.voiced
    gate.edge(False, 1.0)
    assert not gate.step(1.4, allowed=True) and gate.step(1.5, allowed=True) and not gate.voiced


# замер 2 с робота, 28.10 (#4155): кавер «Staying Alive», вокал английский; (начало всплеска /audio/vad, длина), с от 13:41:00
COVER_BURSTS = ((32.833, 0.10), (60.226, 0.09), (60.434, 0.09), (73.921, 0.40))


def test_measured_false_bursts_under_the_vocal_cover_duck_music_for_seconds_not_for_minutes():
    """Три всплеска за 2 мин (два соседних склеиваются удержанием) — три приглушения по ~1.5 с, а не «всё время»."""
    stamps, clock = [], _Clock()
    duck = SpeechDuck(lambda *a: stamps.append((clock.now, a[3])), logger=_FakeLogger(), clock=clock,
                      voice=VoiceSource(music_playing=lambda: True))
    timeline = sorted([(t, 0, True) for t, _ in COVER_BURSTS] + [(t + d, 0, False) for t, d in COVER_BURSTS]
                      + [(30.0 + i * TICK_S, 1, None) for i in range(round(60.0 / TICK_S))])
    for now, kind, high in timeline:
        clock.now = now
        if kind == 0:
            duck.on_vad(high)
        else:
            duck.tick()
    downs = [t for t, state in stamps if state == 1.0]
    ups = [t for t, state in stamps if state == 0.0]
    assert len(downs) == len(ups) == 3
    ducked_s = sum(up - down for down, up in zip(downs, ups))
    assert ducked_s < 6.0, f"приглушено {ducked_s:.1f} с за 128 с кавера"
    assert not duck.ducked


def test_knowledge_matches_the_segmenter_of_audio_node():
    """Удержание короче ``speech_continuation`` (конец фразы для STT), предел всплеска = ``speech_max_duration``."""
    src = (REPO / "src/rob_box_voice/rob_box_voice/audio_node.py").read_text(encoding="utf-8")
    continuation = float(re.search(r"declare_parameter\('speech_continuation', ([\d.]+)\)", src).group(1))
    max_duration = float(re.search(r"declare_parameter\('speech_max_duration', ([\d.]+)\)", src).group(1))
    assert 1.0 <= kn.VOICE_DUCK_HOLD_S < continuation
    assert kn.VOICE_DUCK_MAX_S == max_duration


class _Bool:
    pass


def _voice_node(monkeypatch, *, voice=True, owner=True, playing=True):
    install_dialog_msgs_stub()
    module = _load_mcp_server_module(monkeypatch)
    monkeypatch.setattr(sys.modules["std_msgs.msg"], "Bool", _Bool, raising=False)
    node, sent = _Node(), []
    node.params["music_voice_duck"] = voice
    state = {"playing": playing}
    if owner:
        node._player_owner = SimpleNamespace(is_playing=lambda: state["playing"])
    duck = module._attach_speech_duck(node, SimpleNamespace(send_osc=lambda *a: sent.append(a)))
    return node, duck, sent, state


def test_mcp_server_wires_vad_to_the_voice_source_with_the_deck_as_the_music_predicate(monkeypatch):
    node, duck, sent, state = _voice_node(monkeypatch)
    assert [s[1] for s in node.subs] == ["/voice/speech/event", "/audio/vad"] and node.subs[1][0] is _Bool
    assert node.timers == [(TICK_S, duck.tick)]
    vad = node.subs[1][2]
    vad(SimpleNamespace(data=True))
    assert sent == [("/n_set", 999, "duck", 1.0, "duckDb", -4.0, "duckAttack", 0.04, "duckRelease", 0.25)]
    vad(SimpleNamespace(data=False))
    state["playing"] = False
    assert duck.ducked, "удержание ещё идёт, дека для отпуска не нужна"


def test_mcp_server_voice_source_is_silent_while_no_music_plays_or_there_is_no_player(monkeypatch):
    node, duck, sent, state = _voice_node(monkeypatch, playing=False)
    node.subs[1][2](SimpleNamespace(data=True))
    assert sent == [] and not duck.ducked
    state["playing"] = True
    node.timers[0][1]()  # музыка пошла посреди всплеска — тик подхватывает
    assert _states(sent) == [1.0]
    node, duck, sent, _ = _voice_node(monkeypatch, owner=False)
    node.subs[1][2](SimpleNamespace(data=True))
    node.timers[0][1]()
    assert sent == []


def test_mcp_server_parameter_switches_the_voice_source_off_with_its_subscription(monkeypatch):
    node, duck, sent, _ = _voice_node(monkeypatch, voice=False)
    assert [s[1] for s in node.subs] == ["/voice/speech/event"]
    duck.on_vad(True)
    assert sent == []


def test_node_declares_voice_duck_parameters_from_knowledge():
    src = (REPO / "src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py").read_text(encoding="utf-8")
    assert re.search(r'declare_parameter\("music_voice_duck", True\)', src)
    for name, key in (("music_voice_duck_db", "duckDb"), ("music_voice_duck_attack_ms", "duckAttack"),
                      ("music_voice_duck_release_ms", "duckRelease")):
        assert re.search(rf'declare_parameter\("{name}", music_kn\.VOICE_DUCK\["{key}"\]', src), name
    assert re.search(r'declare_parameter\("music_voice_duck_hold_s", music_kn\.VOICE_DUCK_HOLD_S\)', src)
