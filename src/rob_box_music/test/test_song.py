"""PR-11 ADR-0149: песня (``Form.kind == song``) в модели v2 — валидатор и рендер на событиях нот.

Материал здесь собран руками (без ``harmonize``: пакет чистый); песня по настоящему ``harmonize`` —
``rob_box_mcp_tools/test/test_engine_classic.py``.
"""

from __future__ import annotations

from dataclasses import replace

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.arrange.song import SongMaterial, song_track, verse_count
from rob_box_music.model import TrackError, validate
from rob_box_music.render.events import program_events
from rob_box_music.render.renardo import render

#: Ля минор, 2 такта, ритм не по сетке 16-х (пунктирная 16-я + 32-я), пауза внутри.
LEAD = ((69, 0.75), (72, 0.375), (71, 0.125), (None, 0.75), (76, 2.0), (74, 1.0), (72, 1.0), (69, 2.0))
BASS = ((45, 2.0), (45, 2.0), (40, 2.0), (45, 2.0))
PAD = (((57, 60, 64), 2.0), ((57, 60, 64), 2.0), ((56, 59, 64), 2.0), ((57, 60, 64), 2.0))


def material(**kw):
    base = dict(melody_id="am", title="Ля", bpm=96, root=9, mode="harmonicMinor", lead=LEAD, bass=BASS, pad=PAD,
                pad_sus=0.4, drums="X...o...X...o...", hats="-.-.-.-.-.-.-.-.")
    return SongMaterial(**{**base, **kw})


def test_song_form_is_verses_of_the_whole_melody():
    track = song_track(material(), seed=1)
    assert track.form.song and track.bpm == 96 and len(track.form.sections) == verse_count(2) == 3
    assert [s.bars for s in track.form.sections] == [2, 2, 2] and track.hook.bars == 2


def test_song_has_no_club_look_no_sidechain_and_no_sweep():
    """PR-7 × PR-11: у песни нет клубного вида секции — ни сайдчейна (``amplify`` по огибающей), ни LPF-свипа;
    уровень по куплетам меняет только состав ролей. ``trim`` — дефолт: песня вне сета (``Program.master`` пуст)."""
    track = song_track(material(), seed=1)
    assert track.mix.duck == () and track.mix.duck_roles == frozenset() and track.mix.lpf == {}
    program = render(track, "A")
    _p, events = program_events(program.code, program.form_beats)
    assert events and all("lpf" not in e.fx for e in events)
    assert "lpf=" not in program.code and program.master == {}
    for role in ("bass", "pad"):
        assert {round(e.amp / e.gate, 3) for e in events if e.slot == program.slots[role]} <= {1.0} | set(
            kn.ACCENT_AMPLIFY), role


def test_off_grid_melody_is_heard_note_for_note():
    track = song_track(material(), seed=1)
    program = render(track, "B")
    _p, events = program_events(program.code, program.form_beats)
    lead = sorted((e.beat, int(e.midi), e.sus_beats) for e in events if e.slot == program.slots["lead"])
    assert lead == sorted((p.beat, p.midi, p.dur_beats) for p in track.parts["lead"].pitches)
    assert program.form_beats == 24.0


def test_long_melody_is_one_full_verse():
    long_lead = LEAD * 20  # 40 тактов
    track = song_track(material(lead=long_lead, bass=BASS * 20, pad=PAD * 20), seed=2)
    assert len(track.form.sections) == 1 and track.form.sections[0].roles >= {"kick", "lead", "bass"}


def test_no_drums_means_no_drum_roles():
    track = song_track(material(drums="", hats=""), seed=1)
    assert set(track.parts) == set(kn.TONAL_ROLES)


def test_accompaniment_in_a_foreign_key_is_rejected():
    foreign = tuple(((61, 63, 66), d) for _c, d in PAD)  # C# F# — чужой лад
    with pytest.raises(TrackError) as err:
        song_track(material(pad=foreign, bass=tuple((42, d) for _m, d in BASS)), seed=1)
    assert err.value.path.startswith("parts.") and "в ладу" in err.value.reason


def test_parts_of_different_length_are_refused():
    with pytest.raises(ValueError, match="bass"):
        song_track(material(bass=BASS[:-1]), seed=1)


def test_club_rules_still_hold_for_club_form():
    track = song_track(material(), seed=1)
    with pytest.raises(TrackError) as err:
        validate(replace(track, form=replace(track.form, kind=kn.FORM_CLUB)))
    assert err.value.path == "form.bars_total"
