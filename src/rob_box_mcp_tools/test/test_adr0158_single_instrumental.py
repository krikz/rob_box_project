"""ADR-0158 — одиночный инструментал: «Робот, сгенерируй рок» → мост держит звук → новый трек врезается на такте.

Живой прогон 14.10 02:27 UTC (develop ``362a6d3f9``): ``[source] asked=[generated] genre=rock кандидаты=[только
cache:…]`` → «Сгенерировать сейчас не могу: генерация у меня ещё не подключена» — одиночный инструментал оставался
``not_wired``. Настоящие ``request_music`` + ``SourceRouter`` + ``BridgeHold`` + ``SetSession`` + ``PlayerOwner`` на
симуляторе клока, настоящий кэш и ``gen_ledger``, фейковый KIE (без сети и денег) и ffmpeg.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from rob_box_mcp_tools.engine.gen import gen_session as gs
from rob_box_mcp_tools.engine.gen.instrumental import InstrumentalMaker
from rob_box_mcp_tools.engine.gen.library_play import play_library_track
from rob_box_mcp_tools.engine.gen.provider import GenUnavailable
from rob_box_mcp_tools.engine.search import ThemeHits
from rob_box_mcp_tools.engine.tools_v2 import SOURCE_DECLINED, DjSetTool, GenerateMusicTool, RequestMusicTool

from .test_adr0158_pr7_song import FakeStudio
from .test_adr0158_pr8_gen_set import Kie, _lib, fake_ffmpeg
from .test_adr0158_song_bridge_hold import BAR, _events, _wait
from .test_engine_session import _rig

pytestmark = pytest.mark.unit

ROCK = "Робот, сгенерируй рок"


def _request(tmp_path, monkeypatch, *, cached=1, style="rock", kie=None, songs=None):
    """``request_music`` с исполнителем инструментала; заказ KIE ждёт в ``jobs`` (фон запускает тест)."""
    real = gs.audio_track
    monkeypatch.setattr(gs, "audio_track", lambda info, track_id, run=None: real(info, track_id, run=fake_ffmpeg))
    lib = _lib(tmp_path, cached=cached, seconds=30.0, style=style)
    rig = _rig(tracks=1)
    dj = DjSetTool(None, rig.owner, melodies=lambda ids: {}, finder=lambda theme: ThemeHits(), seed=lambda: 4242)
    node = SimpleNamespace(_player_owner=rig.owner, _dj_set_tool=dj, _music_confirm=None)

    def play(track_id, hold=None):
        return play_library_track(node, lib, track_id, clock=lambda: 7.0, run=fake_ffmpeg, hold=hold)

    kie = kie or Kie(seconds=150.0)
    jobs, said = [], []
    maker = InstrumentalMaker(kie, lambda: lib, balance=lambda: 900.0, spawn=jobs.append)
    req = RequestMusicTool(None, rig.owner, dj, melodies=lambda ids: {}, seed=lambda: 777, records=lambda ids: {},
                           words=lambda word: ThemeHits(), gen_library=lambda: lib, play_cached=play,
                           speak=said.append, songs=songs, instrumental=maker)
    return SimpleNamespace(rig=rig, req=req, lib=lib, kie=kie, jobs=jobs, said=said, maker=maker)


def test_generate_rock_bridges_with_cache_then_new_track_cuts_in_on_a_bar(tmp_path, monkeypatch):
    t = _request(tmp_path, monkeypatch)
    result = t.req.execute(intent="track", text=ROCK, heard_text=ROCK)
    assert result.success and result.data["pending"] == "instrumental", result
    assert result.data["route"] == "generate:instrumental:V6" and result.data["source"] == "cache"
    hold = t.req.hold
    assert hold is not None and result.data["track_id"] == hold.bridge_id
    assert result.data["source_message"] == ("Сочиняю новый трек, рок — секунд 50. "
                                             "Пока играет сочинённое раньше «Grunge 0».")
    assert len(t.jobs) == 1 and not t.kie.jobs  # заказ — в фоне, ответ тула его не ждёт
    t.rig.clock.run_until(t.rig.clock.beat + 2)
    bridge = _events(t.rig, "started")[0]
    assert bridge["track_id"] == hold.bridge_id
    t.rig.clock.run_until(bridge["start_beat"] + 2.5 * bridge["form_beats"])
    assert not _events(t.rig, "finished")  # мост — петля: держит звук, пока пишется трек
    ready = t.rig.clock.beat
    t.jobs[0]()  # KIE ответил
    job = t.kie.jobs[0]
    assert (job.kind, job.model, job.style_key) == ("instrumental", "V6", "rock")
    assert "distorted guitars" in job.style_tags  # теги — из GEN_STYLES, не от LLM
    _wait(lambda: t.rig.owner._pending is not None)  # noqa: SLF001 — cue поставлен из фона
    t.rig.clock.run_until(ready + 3 * bridge["form_beats"])
    live = _events(t.rig, "started")[-1]
    assert live["track_id"] == hold.live_id and live["deck"] == "B"
    assert live["start_beat"] - ready <= BAR + gs.CUT_IN_CUE_BEATS  # вход на ближайшем такте, не в конце формы
    assert t.said == ["А вот новый трек «Rock T1.0», рок, 2 минуты 30 секунд."]  # название и длина — из кэша
    assert t.lib.ledger.spent_since("2000-01-01") == 12.0  # трата — по факту ответа, в gen_ledger


def test_rock_without_cache_bridges_with_the_nearest_strong_synth_style(tmp_path, monkeypatch):
    t = _request(tmp_path, monkeypatch, cached=0)
    result = t.req.execute(intent="track", text=ROCK, heard_text=ROCK)
    assert result.success and result.data["pending"] == "instrumental" and result.data["style"] == "breaks"
    assert result.data["source_message"].endswith("Пока играет своё — брейкбит.")
    assert t.req.hold is not None and result.data["track_id"] == t.req.hold.bridge_id and len(t.jobs) == 1


def test_generation_failure_keeps_the_bridge_and_names_the_reason(tmp_path, monkeypatch):
    """В3: провал после выбора — мост доигрывает финалом, второго круга генерации нет, фраза причины — от кода."""
    t = _request(tmp_path, monkeypatch, kie=Kie(fail=GenUnavailable("no_credits", "402")))
    assert t.req.execute(intent="track", text=ROCK, heard_text=ROCK).success
    t.rig.clock.run_until(t.rig.clock.beat + 2)
    t.jobs[0]()
    assert t.said == ["Сгенерировать не вышло: на счёте генерации нет кредитов — доиграю то, что играет."]
    assert len(t.jobs) == 1 and t.lib.ledger.spent_since("2000-01-01") == 0.0
    _wait(lambda: t.rig.owner._queued is not None)  # noqa: SLF001 — финал моста, не тишина


def test_day_limit_is_a_hard_filter_cache_plays_with_the_reason(tmp_path, monkeypatch):
    t = _request(tmp_path, monkeypatch)
    t.lib.ledger.record_spend("X1", 120.0, kind="instrumental", request_key="k", model="V6", gen_provider="kie_suno")
    result = t.req.execute(intent="track", text=ROCK, heard_text=ROCK)
    assert result.success and result.data["source"] == "cache" and not t.jobs
    assert result.data["source_message"] == ("Сгенерировать сейчас не могу: лимит генерации на сегодня вышел. "
                                             "Ставлю сочинённое раньше: «Grunge 0».")


def test_play_grunge_takes_the_cache_and_spends_nothing(tmp_path, monkeypatch):
    t = _request(tmp_path, monkeypatch, style="grunge")
    text = "Робот, включи гранж"
    result = t.req.execute(intent="track", text=text, heard_text=text)
    assert result.success and result.data["source"] == "cache" and result.data["route"].startswith("cache:")
    assert "pending" not in result.data and not t.jobs and not t.kie.jobs


def test_generate_music_tool_same_turn_is_one_order_and_busy_declines(tmp_path, monkeypatch):
    t = _request(tmp_path, monkeypatch, songs=FakeStudio())  # студия песни есть — инструментал всё равно свой
    tool = GenerateMusicTool(None, t.req)
    first = tool.execute(kind="instrumental", style="rock", turn_id="T1", heard_text=ROCK)
    assert first.success and first.data["pending"] == "instrumental"
    again = t.req.execute(intent="track", text=ROCK, turn_id="T1", heard_text=ROCK)
    assert again.success and again.data["pending"] == "instrumental" and len(t.jobs) == 1
    other = t.req.execute(intent="track", text=ROCK, turn_id="T2", heard_text=ROCK)
    assert not other.success and other.data["reason"] == SOURCE_DECLINED and len(t.jobs) == 1
