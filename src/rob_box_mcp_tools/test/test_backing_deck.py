"""Подложка выступления (ADR-0150 §5.6.3, §9.1; PR-7c ч.2 #3700) на настоящих ``PlayerOwner`` + ``RenardoAdapter`` и
симуляторе клока Renardo: ``started{beat_at, bar_beats}``, ``request_music(intent=backing)`` — проходы формы по длине
речи, продление, ``nearly_finished``, outro на такте (``stop_music(boundary=bar)``), отказ ``busy`` поверх трека."""

import math
from types import SimpleNamespace

import pytest

from rob_box_dialog import knowledge as dkn
from rob_box_dialog.plan.sync import beat_heard_at
from rob_box_mcp_tools.engine.backing_deck import BACKING_LEAD_BEATS, BackingDeck, backing_order
from rob_box_mcp_tools.engine.player_owner import beat_fields
from rob_box_mcp_tools.engine.search import ThemeHits
from rob_box_mcp_tools.engine.tools_v2 import DjSetTool, RequestMusicTool, music_busy
from rob_box_music import knowledge as kn
from rob_box_music.model import BEATS_PER_BAR

from ._ros_stubs import RosStubs
from .test_engine_session import _rig

_ros = RosStubs()
with _ros:  # пакет tools тянет навигацию (rclpy.action) — без ROS нужны заглушки
    from rob_box_mcp_tools.tools.music import StopMusicTool
_ros_stubs = _ros.fixture()

pytestmark = pytest.mark.unit


def _confirm(rig):
    """``MusicEventLog.wait`` на симуляторе: крутить клок, пока у трека нет ``started``/``rejected``."""
    def confirm(track_id):
        for _ in range(400):
            hit = [e for e in rig.events if e.get("track_id") == track_id and e["event"] in ("started", "rejected")]
            if hit:
                fields = {k: v for k, v in hit[-1].items() if k not in ("event", "track_id", "ts")}
                return SimpleNamespace(event=hit[-1]["event"], fields=fields)
            rig.clock.run_until(rig.clock.beat + 1)
        return None
    return confirm


def _tools(rig):
    dj = DjSetTool(None, rig.owner, melodies=lambda ids: {}, finder=lambda theme: ThemeHits(), seed=lambda: 4242)
    req = RequestMusicTool(None, rig.owner, dj, melodies=lambda ids: {}, seed=lambda: 777, records=lambda ids: {},
                           words=lambda word: ThemeHits())
    confirm = _confirm(rig)
    req.backing = BackingDeck(rig.owner, rig.adapter, lambda mood: req._stage_club("", mood), confirm,
                              lambda: music_busy(rig.owner, dj), log=rig.log)
    return req, dj


def _events(rig, kind):
    return [e for e in rig.events if e["event"] == kind]


# --- started: сетка тактов ------------------------------------------------------------------------------------------

def test_started_carries_beat_at_by_the_one_formula_and_bar_length():
    snap = {"bpm": 120.0, "late_beats": 0.5, "latency_s": 0.25}
    assert beat_fields(snap, 1000.0) == {"beat_at": pytest.approx(beat_heard_at(1000.0, 0.5, 120.0, 0.25)),
                                         "bar_beats": float(BEATS_PER_BAR)}
    assert beat_fields({"bpm": 0}, 1000.0) == {}  # темпа нет — выступление читает без тактов


def test_every_started_event_now_has_the_grid():
    rig = _rig()
    req, _ = _tools(rig)
    assert req.execute(intent="track", text="поставь трек").success
    rig.clock.run_until(rig.clock.beat + 200)
    (started,) = _events(rig, "started")
    late_s = started["late_beats"] * 60.0 / started["bpm"]
    assert started["beat_at"] == pytest.approx(1000.0 - late_s + started["latency_s"], abs=1e-3)
    assert started["bar_beats"] == BEATS_PER_BAR


# --- request_music(intent=backing) ----------------------------------------------------------------------------------

def test_backing_plays_as_many_form_passes_as_the_speech_needs_then_finishes():
    rig = _rig()
    req, _ = _tools(rig)
    result = req.execute(intent="backing", text="", mood="groove", min_seconds=40.0)
    assert result.success, result
    data = result.data
    assert data["started"] is True and data["mood"] == "groove"
    for key in ("bpm", "beat_at", "bar_beats", "start_beat", "form_beats", "end_beat"):
        assert key in data
    start, form, bpm = data["start_beat"], data["form_beats"], data["bpm"]
    passes = (data["end_beat"] - start) / form
    assert passes == int(passes) and passes >= 1
    assert (data["end_beat"] - start) * 60.0 / bpm >= 40.0 - form * 60.0 / bpm  # покрывает речь (с точностью формы)
    assert not _events(rig, "finished")
    rig.clock.run_until(data["end_beat"] - BACKING_LEAD_BEATS + 0.01)
    (nearly,) = _events(rig, "nearly_finished")
    assert nearly["form_end_beat"] == data["end_beat"]
    rig.clock.run_until(data["end_beat"] + 0.01)
    assert [e["track_id"] for e in _events(rig, "finished")] == [data["track_id"]]
    assert not rig.owner.is_playing()


def test_second_order_extends_the_playing_backing_and_the_old_end_is_idle():
    rig = _rig()
    req, _ = _tools(rig)
    first = req.execute(intent="backing", text="", mood="groove", min_seconds=10.0).data
    longer = req.execute(intent="backing", text="", mood="groove", min_seconds=120.0)
    assert longer.success and longer.data["extended"] is True and longer.data["track_id"] == first["track_id"]
    assert longer.data["end_beat"] > first["end_beat"]
    rig.clock.run_until(first["end_beat"] + 0.01)
    assert not _events(rig, "finished") and rig.owner.is_playing()  # прежний конец холостой
    rig.clock.run_until(longer.data["end_beat"] + 0.01)
    assert len(_events(rig, "finished")) == 1


def test_shorter_order_never_cuts_the_backing():
    rig = _rig()
    req, _ = _tools(rig)
    first = req.execute(intent="backing", text="", mood="groove", min_seconds=60.0).data
    again = req.execute(intent="backing", text="", mood="groove", min_seconds=1.0).data
    assert again["end_beat"] == first["end_beat"]


def test_outro_stops_on_the_next_bar_without_nearly_finished():
    rig = _rig()
    req, _ = _tools(rig)
    data = req.execute(intent="backing", text="", mood="groove", min_seconds=60.0).data
    rig.clock.run_until(data["start_beat"] + 5.3)
    node = SimpleNamespace(_backing_deck=req.backing, _player_owner=rig.owner,
                           get_logger=lambda: SimpleNamespace(info=lambda *a, **k: None))
    stopped = StopMusicTool(node).execute(boundary="bar")
    assert stopped.success and stopped.data["was_playing"] is True
    bar_end = stopped.data["end_beat"]
    assert bar_end == data["start_beat"] + 2 * BEATS_PER_BAR  # 5.3 доли от старта → граница такта на 8-й
    rig.clock.run_until(bar_end + 0.01)
    assert len(_events(rig, "finished")) == 1 and not _events(rig, "nearly_finished")


def test_stop_bar_without_backing_is_the_plain_stop():
    rig = _rig()
    req, _ = _tools(rig)
    assert req.execute(intent="track", text="поставь трек").success
    node = SimpleNamespace(_backing_deck=req.backing, _player_owner=rig.owner, _stop_deck=rig.owner.stop,
                           get_logger=lambda: SimpleNamespace(info=lambda *a, **k: None))
    result = StopMusicTool(node).execute(boundary="bar")
    assert result.data == {"was_playing": True} and not rig.owner.is_playing()


def test_backing_over_a_playing_track_is_refused_not_replaced():
    """О9 (б): подложка молча не вытесняет трек — спросить человека дело диалога."""
    rig = _rig()
    req, _ = _tools(rig)
    track = req.execute(intent="track", text="поставь трек")
    rig.clock.run_until(rig.clock.beat + 200)
    result = req.execute(intent="backing", text="", mood="groove", min_seconds=30.0)
    assert not result.success and result.data["reason"] == "busy"
    assert rig.owner.now_playing()["track_id"] == track.data["track_id"]


def test_without_deck_backing_is_an_honest_refusal():
    assert backing_order(None, "groove", 10.0)["reason"] == "backing_unavailable"


def test_dialog_backing_moods_exist_in_the_music_engine():
    """Настроения подложки знает диалог (PERFORM_BACKING_MOODS), играет — движок: одна таблица настроений."""
    moods = {m for chain in dkn.PERFORM_BACKING_MOODS.values() for m in chain}
    assert moods <= set(kn.MOOD_ENERGY)


def test_passes_cover_need_exactly_by_form_boundaries():
    rig = _rig()
    req, _ = _tools(rig)
    data = req.execute(intent="backing", text="", mood="calm", min_seconds=25.0).data
    need_beats = 25.0 * data["bpm"] / 60.0
    expected = data["start_beat"] + max(1, math.ceil((rig.clock.beat + need_beats - data["start_beat"])
                                                     / data["form_beats"])) * data["form_beats"]
    assert data["end_beat"] == pytest.approx(expected)
