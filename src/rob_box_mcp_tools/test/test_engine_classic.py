"""ADR-0149 PR-11 — classic-песня на движке v2: «поставь Калинку» (эпик #3312).

Поведение на событиях нот (``render.events``), а не на тексте программы: мелодия целиком по куплетам,
аккомпанемент ``harmonize`` в тональности мелодии, темп — из RTTTL, один проход формы → ``finished``,
``request_music(intent=melody)`` играет песню v2, промах поиска — честный ``found: false``.
"""

from __future__ import annotations

import sys
from collections import defaultdict
from pathlib import Path
from unittest.mock import Mock

import pytest

from rob_box_mcp_tools.core.rtttl_compose import melody_to_compose_params, rtttl_to_melody
from rob_box_mcp_tools.engine.classic import ClassicPick, find_record, pick_classic, pick_score, song_material
from rob_box_mcp_tools.engine.search import ThemeHits
from rob_box_mcp_tools.engine.tools_v2 import DjSetTool, RequestMusicTool
from rob_box_music import knowledge as kn
from rob_box_music.arrange.song import song_track, verse_count
from rob_box_music.model import BEATS_PER_BAR
from rob_box_music.render.events import program_events
from rob_box_music.render.renardo import render
from rob_box_music.tonality import key_fit

from .test_engine_session import _rig, _started

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "rob_box_music" / "test"))
from melodies import CHROMATIC, LONG, SHORT, SLOW  # noqa: E402

pytestmark = pytest.mark.unit

RTTTLS = {"long": LONG, "short": SHORT, "chroma": CHROMATIC, "slow": SLOW}
#: «Калинка» из архива (``kalinkav_2``, дословно): поиск по «калинка» находит её (``RtttlLibrary.get``).
KALINKA = ("KalinkaV:d=4,o=5,b=100:8g#6,8f#6,16d#6,16e#6,8f#6,16d#6,16e#6,16f#6,16e#6,16d#6,16c#6,16p,16g#6,"
           "16d#6,16e#6,8f#6,16d#6,16e#6,16f#6,16f#6,16e#6,16d#6,16c#6,16p,8g#6,16d#6,16e#6,8f#6,16d#6,16e#6,8f#6,"
           "16e#6,16d#6,8c#6")


def _record(name):
    rtttl = KALINKA if name == "kalinka" else RTTTLS[name]
    return {"name": name, "title": name.title(), "rtttl": rtttl}


def _song(name, seed=1, deck="A"):
    track = song_track(song_material(_record(name), name), seed=seed, deck=deck)
    program = render(track, deck)
    _parsed, events = program_events(program.code, program.form_beats)
    role_of = {slot: role for role, slot in program.slots.items()}
    by_role = defaultdict(list)
    for ev in events:
        by_role[role_of[ev.slot]].append(ev)
    return track, program, by_role


def _params(name):
    return melody_to_compose_params(rtttl_to_melody(_record(name)["rtttl"]))


@pytest.mark.parametrize("name", ["kalinka", *RTTTLS])
def test_melody_plays_whole_in_every_verse(name):
    """Каждый куплет — вся мелодия как её подготовил ``rtttl_compose``: те же ноты, те же доли."""
    track, _program, by_role = _song(name)
    lead = _params(name)["harmony"].lead
    theme_beats = sum(d for _m, d in lead)
    verse, beat = [], 0.0
    for midi, dur in lead:
        if midi is not None:
            verse.append((beat, midi))
        beat += dur
    heard = sorted((round(e.beat, 6), int(e.midi)) for e in by_role["lead"])
    expected = sorted((round(v * theme_beats + b, 6), m) for v in range(len(track.form.sections)) for b, m in verse)
    assert heard == expected
    assert len(track.form.sections) == verse_count(int(theme_beats // BEATS_PER_BAR))
    assert all(sec.bars * BEATS_PER_BAR == theme_beats for sec in track.form.sections)
    assert track.hook.source == name and track.hook.bars * BEATS_PER_BAR == theme_beats


@pytest.mark.parametrize("name", ["kalinka", "long", "short", "slow"])
def test_accompaniment_is_in_the_key_of_the_melody(name):
    track, _program, by_role = _song(name)
    params = _params(name)
    assert (kn.ROOTS[track.key.root], track.key.mode) == (params["root"], params["scale"])
    for role in ("bass", "pad"):
        notes = [(int(e.midi), e.sus_beats) for e in by_role[role] if e.detune == 0]
        assert notes and key_fit(notes, params["root"], params["scale"]) >= 0.8, role


@pytest.mark.parametrize("name,bpm", [("kalinka", 100), ("long", 130), ("short", 100)])
def test_tempo_comes_from_the_rtttl_not_the_club_window(name, bpm):
    track, program, _by_role = _song(name)
    assert track.bpm == program.bpm == bpm == _params(name)["bpm"]


def test_first_verse_is_the_theme_over_pad_and_bass_last_is_full():
    track, _program, by_role = _song("long")
    first, last = track.form.sections[0], track.form.sections[-1]
    assert {"lead", "pad", "bass"} <= first.roles and "kick" not in first.roles
    assert {"kick", "lead", "pad", "bass"} <= last.roles
    assert not [e for e in by_role["kick"] if e.beat < first.bars * BEATS_PER_BAR]


def test_song_stereo_goes_through_role_stereo():
    """Ширина — та же таблица ``Style.stereo``, что у club: пэд в два голоса, бас и бочка в центре."""
    track, _program, by_role = _song("long")
    assert dict(track.mix.stereo) == {r: s for r, s in track.mix.stereo.items() if r in kn.STYLES["club"].stereo}
    assert {e.pan for e in by_role["pad"]} == {-kn.PAD_SPREAD, kn.PAD_SPREAD}
    assert {e.pan for e in by_role["bass"]} == {0.0} and track.mix.duck == () and not track.mix.duck_roles


def _library(record):
    library = Mock()
    library.get.side_effect = lambda q: record
    library.ru_alias_for.return_value = None
    library.search.return_value = []
    library.vocabulary.return_value = frozenset(" ".join(str(v) for k, v in (record or {}).items()
                                                         if k in ("name", "title")).lower().replace("_", " ").split())
    return library


def test_find_record_uses_the_named_play_rule():
    kalinka = {**_record("kalinka"), "name": "kalinkav_2", "title": "Kalinka V2.0"}
    assert find_record(_library(kalinka), "калинка")[0] is kalinka
    anthem = {"name": "national", "title": "Soviet Anthem", "rtttl": LONG}
    record, reason = find_record(_library(anthem), "гимн германии")
    assert record is None and "вне поиска" in reason
    assert find_record(_library(None), "абвгд") == (None, "lookup: не найдена")


def test_pick_classic_renders_the_found_song():
    pick = pick_classic(_library({**_record("kalinka"), "name": "kalinkav_2"}), "калинка", seed=3)
    assert pick.found and pick.melody_id == "kalinkav_2" and pick.bpm == 100
    assert pick.program.track_id.startswith("classic:kalinkav_2:A:")


def test_pick_score_takes_the_first_material_that_makes_a_song():
    """ADR-0154 PR-7B: песня из партитуры — первый материал поиска, из которого она складывается; ни одного —
    ``found=False`` с причиной по каждому (заказ уходит в RTTTL)."""
    from dataclasses import replace

    from rob_box_music import material as mt
    from rob_box_music.model import Key, PitchEvent

    melody = tuple(PitchEvent(60 + (i % 4) * 2, float(i), 1.0, 2) for i in range(32))
    chords = tuple(mt.ChordSpan(float(b * 4), 4.0, 0, "maj", 0) for b in range(8))
    good = mt.ScoreMaterial("local:good", "Good", "t", "s", "PD", (4, 4), 120, Key(0, "major"), melody, chords)
    bare = replace(good, material_id="local:bare", chords=())
    ids = ["local:missing", "local:bare", "local:good"]
    pick = pick_score({"local:bare": bare, "local:good": good}, ids, "q", seed=1)
    assert pick.found and pick.melody_id == "local:good" and pick.title == "Good" and pick.bpm == 120
    assert pick.program.track_id.startswith("classic:local:good:A:")
    miss = pick_score({"local:bare": bare}, ids[:2], "q", seed=1)
    assert not miss.found and "local:missing" in miss.reason and "аккорд" in miss.reason
    other = "ref:d=8,o=5,b=132:" + ",".join(["c,c#,c,c#,c,c#,c,c#"] * 2)  # мотива эталона в материале нет
    refused = pick_score({"local:good": good}, ["local:good"], "q", seed=1, references=[other])
    assert not refused.found and "главного мотива" in refused.reason


def _tools(rig, classic):
    dj = DjSetTool(None, rig.owner, melodies=lambda ids: {}, finder=lambda theme: ThemeHits(), seed=lambda: 4242)
    return dj, RequestMusicTool(None, rig.owner, dj, melodies=lambda ids: {}, seed=lambda: 5, classic=classic)


def _picker(asked):
    def pick(query, *, seed, deck="A"):
        asked.append(query)
        track = song_track(song_material(_record("kalinka"), "Калинка"), seed=seed, deck=deck)
        return ClassicPick(query, True, melody_id="kalinka", title="Калинка", program=render(track, deck), bpm=100)
    return pick


@pytest.mark.parametrize("args", [{"intent": "melody", "text": "поставь калинку"},
                                  {"intent": "track", "text": "калинку", "genre": "folk"}])
def test_request_music_melody_plays_the_v2_song_once(args):
    rig = _rig()
    asked = []
    _dj, req = _tools(rig, _picker(asked))
    result = req.execute(**args)
    assert result.success and result.data["found"] and asked == ["калинку"]
    rig.clock.run_until(rig.clock.beat + 2)
    started = _started(rig)
    assert [e["track_id"] for e in started] == [result.data["track_id"]] and started[0]["phase_in_form"] == 0.0
    end = started[0]["start_beat"] + started[0]["form_beats"]
    rig.clock.run_until(end + 1)
    assert [e["event"] for e in rig.events][-1] == "finished"
    assert '"state": "idle"' in rig.states[-1] and result.data["track_id"] in rig.states[-1]


def test_melody_closes_a_running_set():
    rig = _rig()
    dj, req = _tools(rig, _picker([]))
    assert dj.execute(action="start", theme="космос").success
    assert req.execute(intent="melody", text="поставь калинку").success
    assert dj._session is None


def test_not_found_is_honest_and_does_not_touch_the_deck():
    rig = _rig()
    _dj, req = _tools(rig, lambda query, *, seed, deck="A": ClassicPick(query, False, "lookup: не найдена"))
    result = req.execute(intent="melody", text="поставь абвгд")
    assert result.success is False and result.data["found"] is False and result.data["reason"] == "not_found"
    assert rig.events == []


def test_song_that_does_not_fit_the_model_is_rejected_loudly():
    rig = _rig()

    def broken(query, *, seed, deck="A"):
        raise ValueError("мелодия 3.5 долей — не целое число тактов")

    _dj, req = _tools(rig, broken)
    result = req.execute(intent="melody", text="поставь калинку")
    assert result.success is False and result.data["reason"] == "compose_error"
    assert [e["event"] for e in rig.events] == ["rejected"]
