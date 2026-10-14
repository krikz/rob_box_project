"""Хвосты ADR-0158 PR-8: мост песни из трека кэша — ``BridgeHold`` (петля, а не один проход и тишина); результат поверх
синтеза-моста входит на ближайшем такте кроссфейдом, а не на границе формы моста (до ~2 мин после готовности).

Настоящие ``BridgeHold`` + ``SetSession`` + ``PlayerOwner`` + ``RenardoAdapter`` на симуляторе клока (``_rig``),
настоящий ``play_library_track`` и кэш ``GeneratedMusicLibrary``, фейковый ffmpeg. Без сети и денег.
"""

from __future__ import annotations

import time
from types import SimpleNamespace

import pytest

from rob_box_mcp_tools.engine.gen import gen_session as gs
from rob_box_mcp_tools.engine.gen.library_play import play_library_track
from rob_box_mcp_tools.engine.session import NEARLY_LEAD_BEATS, compose_source
from rob_box_mcp_tools.engine.source_route import cache_tracks
from rob_box_mcp_tools.engine.search import ThemeHits
from rob_box_mcp_tools.engine.tools_v2 import DjSetTool, RequestMusicTool
from rob_box_music import knowledge as kn
from rob_box_music.model import AudioTrack
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import seeded_plan

from .test_adr0158_pr7_song import PHRASE, FakeStudio
from .test_adr0158_pr8_gen_set import _lib, fake_ffmpeg
from .test_engine_session import PROFILE, _forms, _rig

pytestmark = pytest.mark.unit

BAR = 4.0
XFADE = kn.GEN_XFADE_BARS * BAR


def _events(rig, kind):
    return [e for e in rig.events if e["event"] == kind]


def _wait(cond, timeout=5.0):
    """Фон ``BridgeHold`` (поток подготовки результата) — дождаться его шага, клок стоит."""
    end = time.monotonic() + timeout
    while not cond() and time.monotonic() < end:
        time.sleep(0.01)
    assert cond(), "фон моста не дошёл до шага"


# ── доля входа ──

def test_cut_in_beat_is_the_next_bar_the_cue_still_makes():
    form = 256.0  # 64 такта синтеза
    assert gs.cut_in_beat(None) is None  # мост не играет
    at = gs.cut_in_beat((137.3, 100.0, form))
    assert at % BAR == 0 and 137.3 + gs.CUT_IN_CUE_BEATS < at <= 137.3 + gs.CUT_IN_CUE_BEATS + BAR
    # последние ``lead`` долей прохода — тоже ближайший такт: nearly_finished моста видит поставленный результат
    assert gs.cut_in_beat((100.0 + form - NEARLY_LEAD_BEATS - 2.0, 100.0, form)) == 100.0 + form - NEARLY_LEAD_BEATS + 4.0
    # nearly_finished этого прохода уже был (результата не было — мост продлён): вход на такте, не через проход
    late = 100.0 + form - 10.0
    assert gs.cut_in_beat((late, 100.0, form)) == 100.0 + form - 4.0


# ── синтез-мост с длинной формой ──

def _synth_hold(tmp_path, monkeypatch):
    """Синтез-мост с формой 64 такта на деке A (как мост песни после клуба 13.10), результат — файл кэша 60 с."""
    monkeypatch.setattr("rob_box_mcp_tools.engine.gen.gen_session.audio_track",
                        lambda info, track_id, run=None: AudioTrack(track_id, str(tmp_path / "song.wav"), 60.0))
    lib = _lib(tmp_path, cached=1, seconds=60.0)
    rig = _rig(tracks=1)
    model = compose_source(_forms(seeded_plan(PROFILE, 7, set_id="s7"), "long64"))(2, "A")
    program = render(model, "A")
    hold = gs.BridgeHold(rig.owner, model, program, lambda: lib, facts={"title": "клуб"}, logger=rig.log)
    assert hold.start()["ok"]
    rig.clock.run_until(rig.clock.beat + 2)
    bridge = _events(rig, "started")[0]
    assert bridge["form_beats"] == 256.0
    return rig, hold, lib, bridge


@pytest.mark.parametrize("into", [37.3, 130.6])  # готово на 9-м и на 33-м такте 64-тактовой формы моста
def test_song_over_a_long_synth_bridge_enters_on_the_next_bar_with_a_crossfade(tmp_path, monkeypatch, into):
    rig, hold, lib, bridge = _synth_hold(tmp_path, monkeypatch)
    ready = bridge["start_beat"] + into
    rig.clock.run_until(ready)
    assert hold.deliver(cache_tracks(lib)[0].id)
    _wait(lambda: rig.owner._pending is not None)  # noqa: SLF001 — cue поставлен из фона
    rig.clock.run_until(ready + 3 * BAR)
    live = _events(rig, "started")[-1]
    assert live["track_id"] == hold.live_id and live["deck"] == "B"
    assert live["start_beat"] % BAR == 0 and live["start_beat"] - ready <= BAR + gs.CUT_IN_CUE_BEATS  # ≤ 1 такт
    assert live["start_beat"] < bridge["start_beat"] + bridge["form_beats"] - XFADE  # не на границе формы моста
    rig.clock.run_until(ready + bridge["form_beats"])
    free = [m for _, m in rig.log.lines if "deck A free" in m]
    assert free, "мост не ушёл"
    assert not _events(rig, "finished") or _events(rig, "finished")[0]["track_id"] == hold.live_id
    assert len(_events(rig, "queued")) == 1  # nearly_finished моста не готовил результат второй раз
    assert any("входит на доле" in m for _, m in rig.log.lines)


def test_ready_inside_the_last_lead_of_a_pass_cuts_in_on_the_next_bar(tmp_path, monkeypatch):
    """Готово перед ``nearly_finished`` прохода: вход всё равно на ближайшем такте, ``nearly_finished`` между
    постановкой и входом мост не продлевает и результат второй раз не готовит."""
    rig, hold, lib, bridge = _synth_hold(tmp_path, monkeypatch)
    end = bridge["start_beat"] + bridge["form_beats"]
    ready = end - NEARLY_LEAD_BEATS - 2.0
    rig.clock.run_until(ready)
    assert hold.deliver(cache_tracks(lib)[0].id)
    _wait(lambda: rig.owner._pending is not None)  # noqa: SLF001
    rig.clock.run_until(end + 2 * BAR)
    live = _events(rig, "started")[-1]
    assert live["track_id"] == hold.live_id and live["start_beat"] - ready <= BAR + gs.CUT_IN_CUE_BEATS
    assert len(_events(rig, "queued")) == 1 and not [m for m in rig.log.warnings() if "не готов" in m]


def test_bridge_failure_over_a_synth_bridge_is_a_finale_at_the_boundary_not_a_cut(tmp_path, monkeypatch):
    rig, hold, _, bridge = _synth_hold(tmp_path, monkeypatch)
    rig.clock.run_until(bridge["start_beat"] + 40.0)
    assert hold.fail()
    _wait(lambda: rig.owner._queued is not None)  # noqa: SLF001
    assert rig.owner._pending is None  # noqa: SLF001 — финал моста не врезается посреди формы


# ── мост песни из трека кэша ──

def _song_request(tmp_path, seconds):
    """``request_music`` с настоящим ``play_library_track`` (шов ``hold``) и студией-фейком; в кэше — инструментал
    гранжа ``seconds`` с (мост)."""
    lib = _lib(tmp_path, cached=1, seconds=seconds)
    rig = _rig(tracks=1)
    dj = DjSetTool(None, rig.owner, melodies=lambda ids: {}, finder=lambda theme: ThemeHits(), seed=lambda: 4242)
    node = SimpleNamespace(_player_owner=rig.owner, _dj_set_tool=dj, _music_confirm=None)

    def play(track_id, hold=None):
        return play_library_track(node, lib, track_id, clock=lambda: 7.0, run=fake_ffmpeg, hold=hold)

    studio = FakeStudio()
    req = RequestMusicTool(None, rig.owner, dj, melodies=lambda ids: {}, seed=lambda: 777, records=lambda ids: {},
                           words=lambda word: ThemeHits(), gen_library=lambda: lib, play_cached=play, songs=studio)
    return rig, req, studio, lib


@pytest.mark.parametrize("song_after_passes", [0.5, 3.5])  # песня пишется быстрее и дольше трека кэша
def test_cache_bridge_of_a_song_loops_until_the_song_then_crossfades(tmp_path, monkeypatch, song_after_passes):
    """Правило 13.10: мост держит звук до результата. Раньше трек кэша играл один проход (``play_library_track``,
    ``once``) — песня дольше трека, и между ними тишина."""
    real = gs.audio_track
    monkeypatch.setattr(gs, "audio_track", lambda info, track_id, run=None: real(info, track_id, run=fake_ffmpeg))
    rig, req, studio, lib = _song_request(tmp_path, seconds=30.0)
    result = req.execute(intent="track", text=PHRASE, heard_text=PHRASE)
    assert result.success and result.data["pending"] == "song" and result.data["source"] == "cache"
    hold = req.hold
    assert hold is not None and result.data["track_id"] == hold.bridge_id and studio.play is not None
    rig.clock.run_until(rig.clock.beat + 2)
    bridge = _events(rig, "started")[0]
    # 30 с при 60 BPM: файл вниз до такта (28), проход короче на кроссфейд (2 такта — не больше трети файла)
    assert bridge["track_id"] == hold.bridge_id and bridge["form_beats"] == 20.0
    ready = bridge["start_beat"] + song_after_passes * bridge["form_beats"]
    rig.clock.run_until(ready)
    assert not _events(rig, "finished")  # мост не кончился: петля, а не один проход
    song = lib.save_track(b"MP3:60.0", prompt="song", title="Песня про космос", is_instrumental=False,
                          kind="robot_song", style_key="grunge", genre="grunge", task_id="S1", variant=0,
                          duration_ms=60000)["id"]
    assert studio.play(song).success  # SongStudio играет песню через held_play → BridgeHold.deliver
    _wait(lambda: rig.owner._pending is not None)  # noqa: SLF001
    rig.clock.run_until(ready + 3 * bridge["form_beats"])
    live = _events(rig, "started")[-1]
    assert live["track_id"] == hold.live_id and live["deck"] == "B"
    # мост — файл: вход на ближайшем такте, мост гаснет огибающей деки (14.10: ждал конца прохода файла, 52 с)
    assert live["start_beat"] - ready <= BAR + gs.CUT_IN_CUE_BEATS
    finished = _events(rig, "finished")
    assert not finished or finished[0]["track_id"] == hold.live_id
