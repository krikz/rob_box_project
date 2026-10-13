"""ADR-0158 PR-4 (G10) — кавер пьесы: наш рендер мелодии → Suno cover → кэш; мост синтезом; повтор — из кэша.

Решение Шифу В7 (13.10): «валькирии на ситаре» → кавер с ситаром, пока ждём — наша версия той же пьесы, потом
кавер с деки; повторный заказ играет из кэша без генерации. Здесь — без сети и денег: фейковый провайдер KIE,
настоящий кэш ``GeneratedMusicLibrary`` и настоящий владелец плеера (``_rig``).
"""

from __future__ import annotations

import wave
from types import SimpleNamespace

import pytest

from rob_box_mcp_tools.base import MCPToolResult
from rob_box_mcp_tools.engine.classic import pick_record
from rob_box_mcp_tools.engine.gen.cover import (CoverMaker, CoverResult, cover_job, melody_notes, render_wav,
                                                source_notes)
from rob_box_mcp_tools.engine.gen.library import GeneratedMusicLibrary
from rob_box_mcp_tools.engine.gen.provider import GenUnavailable
from rob_box_mcp_tools.engine.piece_cover import PieceCover
from rob_box_mcp_tools.engine.source_route import SourceRouter, cache_tracks
from rob_box_mcp_tools.engine.tools_v2 import DjSetTool, RequestMusicTool
from rob_box_music import knowledge as kn
from rob_box_music.choice import DeterministicChooser
from rob_box_music.source import CoverWork, SourceAsk, source_ask
from rob_box_voice.core.media_phrases import launch_text

from .test_engine_session import _rig
from .test_gen_library_cache import FakeProvider
from .test_issue_3604_composer_filter import HITS, _both
from .test_piece_fail_open import Scores
from melodies import LONG, SHORT  # noqa: E402  (путь добавлен test_piece_fail_open)

pytestmark = pytest.mark.unit

RIDE = {"name": "rideofth", "title": "Ride Of The Valkyries", "artist": "Wagner", "rtttl": LONG,
        "tags": ["classical", "picaxe:mixed3"]}
TABLE = {"rideofth": RIDE, "valkyrie": {"name": "valkyrie", "title": "Valkyrie", "artist": "In Development",
                                        "rtttl": SHORT, "tags": []}}
SITAR = CoverWork("Ride Of The Valkyries", True, "sitar", "Ride Of The Valkyries")


class Kie(FakeProvider):
    """Фейковый KIE с ключом: заказ запоминается, ответ — два варианта."""

    configured = True


def _records(ids):
    return {i: (TABLE[i], TABLE[i]["title"]) for i in ids if i in TABLE}


def _track():
    pick = pick_record(RIDE, RIDE["title"], "валькирии", seed=5)
    assert pick.found and pick.public_domain and pick.work == "Ride Of The Valkyries"
    return pick.track


def test_melody_render_is_twenty_to_forty_seconds_of_our_lead(tmp_path):
    notes = source_notes(melody_notes(_track()))
    assert notes and kn.COVER_SOURCE_MIN_S <= max(s + d for s, d, _ in notes) <= kn.COVER_SOURCE_MAX_S
    seconds = render_wav(notes, str(tmp_path / "src.wav"))
    with wave.open(str(tmp_path / "src.wav"), "rb") as wav:
        assert (wav.getnchannels(), wav.getframerate()) == (1, kn.COVER_SOURCE_RATE_HZ)
        assert abs(wav.getnframes() / wav.getframerate() - seconds) < 0.01
    assert source_notes([(0.0, 7.0, 69)]) == [(0.0, 7.0, 69), (7.0, 7.0, 69), (14.0, 7.0, 69)]  # мотив повторяется


def test_cover_job_styles_come_from_tables():
    sitar = cover_job(SITAR, SourceAsk(), "/tmp/x.wav")
    assert sitar.kind == "cover" and sitar.style_tags == kn.COVER_STYLES["sitar"] and sitar.style_key == ""
    assert sitar.audio_weight == kn.COVER_AUDIO_WEIGHT and "electronic" in sitar.negative_tags
    plain = cover_job(CoverWork("Ride Of The Valkyries", True), SourceAsk(), "/tmp/x.wav")
    assert plain.style_key == "orchestral" and plain.style_tags == kn.GEN_STYLES["orchestral"].tags
    club = cover_job(CoverWork("X", True), SourceAsk(genre="club"), "/tmp/x.wav")
    assert "electronic" not in club.negative_tags


def test_maker_caches_both_variants_under_the_cover_key_and_the_repeat_routes_to_cache(tmp_path):
    """Заказ → оба варианта в кэше с ``work_id``/``instrument``; повтор того же заказа — ``cache:``, не генерация."""
    lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
    kie = Kie()
    maker = CoverMaker(kie, lambda: lib, clock=iter(range(100)).__next__, workdir=str(tmp_path))
    result = maker.make(_track(), SITAR, source_ask("сыграй валькирии на ситаре"))
    assert result.track_id and result.task_id == "TASK9" and result.credits == 12.0 and not result.reason
    assert kie.submitted[0].kind == "cover" and kie.submitted[0].audio_path.endswith(".wav")
    rows = [lib.get_track_info(t.id) for t in cache_tracks(lib)]
    assert {(r["work_id"], r["instrument"], r["kind"]) for r in rows} == {("Ride Of The Valkyries", "sitar", "cover")}
    assert len(rows) == 2 and lib.ledger.spent_since("2000-01-01") == 12.0
    assert not list(tmp_path.glob("cover_src_*.wav"))  # исходник после загрузки не копится
    router = SourceRouter(DeterministicChooser(), lambda: lib, cover=maker)
    route = router.route(source_ask("сыграй валькирии на ситаре"), piece=True, work=SITAR)
    assert route.kind == "cache" and route.track.kind == "cover"


def test_maker_failure_is_a_reason_not_silence(tmp_path):
    lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
    maker = CoverMaker(Kie(fail=GenUnavailable("no_credits", "402")), lambda: lib, workdir=str(tmp_path))
    assert maker.make(_track(), SITAR, SourceAsk()).reason == "no_credits"
    assert cache_tracks(lib) == ()


class FakeMaker:
    """Исполнитель кавера без потока: ``start`` запоминает заказ, результат отдаёт тест."""

    def __init__(self, ready=True, balance=900.0):
        self._ready, self._balance, self.orders = ready, balance, []

    def ready(self):
        return self._ready

    def balance(self):
        return self._balance

    def start(self, track, work, ask, done):
        self.orders.append(SimpleNamespace(track=track, work=work, ask=ask, done=done))


def _request(rig, maker, played, said, lib=None):
    dj = DjSetTool(None, rig.owner, melodies=lambda ids: {}, finder=lambda theme: _both(HITS, False),
                   seed=lambda: 4242, scores=Scores())

    def play(track_id):
        played.append(track_id)
        return MCPToolResult(success=True, data={"ok": True, "track_id": f"gen_{track_id[:8]}_1"})

    return RequestMusicTool(None, rig.owner, dj, melodies=lambda ids: {}, seed=lambda: 5, records=_records,
                            words=lambda word: _both(HITS, False), gen_library=lambda: lib, play_cached=play,
                            cover=maker, speak=said.append)


def _cached(lib, seconds=60.0):
    """Трек кэша «кавер» (для ``BridgeHold``: результат входит с деки моста)."""
    return lib.save_track(f"MP3:{seconds}".encode(), prompt="cover", title="Ride (cover)", is_instrumental=True,
                          kind="cover", task_id="T1", variant=0, duration_ms=int(seconds * 1000))["id"]


def _fake_audio(info, track_id, run=None):
    from rob_box_music.model import AudioTrack

    return AudioTrack(track_id=track_id, path=str(info["path"]).replace("track.mp3", "t.wav"), duration_s=60.0)


def _queued(rig, timeout=5.0):
    """Фон ``BridgeHold`` поставил результат (или финал) в очередь владельца."""
    import time

    end = time.monotonic() + timeout
    while rig.owner._queued is None and time.monotonic() < end:  # noqa: SLF001
        time.sleep(0.01)
    return rig.owner._queued  # noqa: SLF001


def test_instrument_order_bridges_with_our_synth_then_the_cover_takes_the_deck(tmp_path, monkeypatch):
    """Гитара, а не ситар: у подставного сервера стенда (``_rig``) синта sitar нет; путь тот же. Мост держит звук,
    пока кавер не готов (ADR-0158 PR-8, живой прогон 13.10: 44 с тишины), кавер входит кроссфейдом на деке B."""
    monkeypatch.setattr("rob_box_mcp_tools.engine.gen.gen_session.audio_track", _fake_audio)
    rig, maker, played, said = _rig(), FakeMaker(), [], []
    lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
    req = _request(rig, maker, played, said, lib)
    result = req.execute(intent="melody", text="сыграй валькирии на гитаре", heard_text="сыграй валькирии на гитаре")
    assert result.success, result.error
    data = result.data
    assert data["melody_id"] == "rideofth" and data["instrument"] == "guitar" and rig.owner.is_playing()
    assert launch_text("", data).startswith("Живую версию на гитаре готовлю — секунд через 50, а пока своя. Ставлю")
    order = maker.orders[0]
    assert order.work == CoverWork("Ride Of The Valkyries", True, "guitar", data["title"]) and order.track is not None
    rig.clock.run_until(rig.clock.beat + 2)  # мост встал
    bridge = [e for e in rig.events if e["event"] == "started"][0]
    rig.clock.run_until(bridge["start_beat"] + 3 * bridge["form_beats"])  # кавер пишется — мост повторяет форму
    assert rig.owner.now_playing()["track_id"] == bridge["track_id"]
    order.done(CoverResult(_cached(lib), task_id="T1", credits=12.0))  # кавер готов, на деке всё ещё мост
    program, _prev = _queued(rig)
    assert program.deck == "B" and "loop(" in program.code and played == []
    rig.clock.run_until(rig.clock.beat + 2 * bridge["form_beats"])
    live = [e for e in rig.events if e["event"] == "started"][-1]
    assert live["track_id"] == program.track_id and live["deck"] == "B"
    assert said == [f"А вот живая версия «{data['title']}» на гитаре."]


def test_cover_does_not_barge_in_after_stop_and_failure_is_spoken_only_over_the_bridge(tmp_path):
    rig, maker, played, said = _rig(), FakeMaker(), [], []
    req = _request(rig, maker, played, said, GeneratedMusicLibrary(str(tmp_path / "lib")))
    req.execute(intent="melody", text="сыграй валькирии на гитаре", heard_text="сыграй валькирии на гитаре")
    rig.clock.run_until(rig.clock.beat + 2)
    first = maker.orders[0]
    first.done(CoverResult(reason="timeout"))  # мост звучит — фраза о провале, мост доиграет финальным проходом
    assert said == [f"Живой версии «{first.work.spoken}» не будет: генерация не уложилась по времени."]
    finale, _prev = _queued(rig)
    assert finale.deck == "B" and "loop(" not in finale.code  # мост ещё раз, не файл
    rig.owner.stop("user_stop")
    req.execute(intent="melody", text="сыграй валькирии на гитаре", heard_text="сыграй валькирии на гитаре")
    rig.owner.stop("user_stop")
    maker.orders[1].done(CoverResult("cov2"))  # человек остановил музыку — кавер остаётся в кэше, дека пустая
    assert played == [] and len(said) == 1 and not rig.owner.is_playing()


def test_without_money_the_piece_plays_our_synth_and_says_why(tmp_path):
    rig, maker, played, said = _rig(), FakeMaker(balance=0.0), [], []
    req = _request(rig, maker, played, said, GeneratedMusicLibrary(str(tmp_path / "lib")))
    result = req.execute(intent="melody", text="сыграй валькирии на гитаре", heard_text="сыграй валькирии на гитаре")
    assert result.success and maker.orders == []
    assert launch_text("", result.data).startswith(
        "Живую версию на гитаре сейчас не сделаю: на счёте генерации нет кредитов. Ставлю")


def test_bridge_not_public_domain_takes_the_cover_from_the_public_domain_record_of_the_same_order():
    """Живой прогон 13.10: свежесть выбрала мостом `valkyrie` («In Development»), а кавер — по Вагнеру `rideofth`."""
    from rob_box_music.works import RTTTL, WorkCandidate

    class Pieces:
        def candidates(self, query):
            return None, [WorkCandidate("valkyrie", RTTTL, "Valkyrie", "In Development", 0.5, 0.0),
                          WorkCandidate("rideofth", RTTTL, "Ride Of The Valkyries", "Wagner", 0.5, 0.0)], \
                _records(TABLE)

    bridge = {"program": object(), "work": "Valkyrie", "public_domain": False, "title": "Valkyrie",
              "instrument": "", "played": {"work": "валькирий"}}
    work, track = PieceCover(FakeMaker()).source(bridge, Pieces(), seed=5)
    assert work == CoverWork("Ride Of The Valkyries", True, "", work.spoken) and track is not None
    pd = dict(bridge, work="Ride Of The Valkyries", public_domain=True)
    assert PieceCover(FakeMaker()).source(pd, Pieces(), seed=5)[1] is None  # мост сам — источник
