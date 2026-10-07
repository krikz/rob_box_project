"""ADR-0154 PR-7B: песня из материала партитуры («сыграй <произведение>», ``song.score_song``).

Куплеты = секции материала (без меток — фразы подряд до ``knowledge.SONG_SCORE_VERSE_BARS``), мелодия автора один
раз, пэд — аккорды автора арпеджио (``pad.arp_events``, ``Style.arp_order``), бас — голос автора. Материал —
синтетический (``test_harmony_material.synthetic``): ноты и аккорды известны заранее.
"""

from __future__ import annotations

from dataclasses import replace

import pytest

from rob_box_music import knowledge as kn
from rob_box_music import material as mt
from rob_box_music.arrange import song
from rob_box_music.model import BEATS_PER_BAR, PitchEvent, validate
from test_harmony_material import synthetic

DEGREES = (0, 0, 5, 5, 3, 3, 4, 4, 5, 5, 3, 3, 0, 0, 4, 4)
SECTIONS = (mt.ScoreSection("A", 0, 8, "text"), mt.ScoreSection("B", 8, 8, "text"))


def scored(degrees=DEGREES, sections=SECTIONS, **kw) -> mt.ScoreMaterial:
    m = synthetic(degrees, **kw)
    bass = tuple(PitchEvent(c.root_pc + 36, c.beat, c.dur_beats, 2) for c in m.chords)
    return replace(m, sections=sections, bass=bass, bpm=120)


def test_verses_are_the_sections_and_the_melody_plays_once():
    m = scored()
    track = song.score_song(m, seed=1)
    validate(track)
    assert [(s.name, s.bars) for s in track.form.sections] == [("A", 8), ("B", 8)]
    assert track.form.song and track.bpm == 120 and track.key == m.key
    lead = track.parts["lead"].pitches
    assert [(e.midi, e.beat, e.dur_beats) for e in lead] == [(e.midi, e.beat, e.dur_beats) for e in m.melody]
    assert track.hook.source == m.material_id and track.history_key.progression == "score"


def test_pad_arpeggiates_the_author_chords_and_bass_is_the_author_bass():
    m = scored()
    track = song.score_song(m, seed=1)
    order = song.SONG_STYLE.arp_order
    step = BEATS_PER_BAR / 16
    for e in track.parts["pad"].pitches:
        chord = next(c for c in m.chords if c.beat <= e.beat < c.beat + c.dur_beats)
        assert e.midi % 12 in {(chord.root_pc + i) % 12 for i in kn.CHORD_INTERVALS[chord.quality]}
        assert kn.SONG_SCORE_PAD[0] <= e.midi <= kn.SONG_SCORE_PAD[1] and e.dur_beats == step
    first_bar = sorted(track.parts["pad"].pitches, key=lambda e: e.beat)[:len(order)]
    ranks = [sorted({x.midi for x in first_bar}).index(x.midi) for x in first_bar]
    assert ranks == [o % 3 for o in order]  # голоса по Style.arp_order
    lo, hi = kn.SONG_REGISTERS["bass"]
    bass = track.parts["bass"].pitches
    assert [(e.midi % 12, e.beat) for e in bass] == [(c.root_pc, c.beat) for c in m.chords]
    assert all(lo <= e.midi <= hi for e in bass)


def test_without_sections_verses_group_phrases():
    phrases = tuple(mt.Phrase(b, 4, "new") for b in range(0, 16, 4))
    m = replace(scored(sections=()), phrases=phrases)
    assert song.score_verses(m) == [("verse", 0, 8), ("verse", 8, 8)]
    track = song.score_song(m, seed=2)
    assert track.form.bars_total == 16 and len(track.form.sections) == 2


def test_material_without_chords_or_meter_is_refused():
    with pytest.raises(ValueError, match="аккорд"):
        song.score_song(replace(scored(), chords=()), seed=1)
    waltz = scored(DEGREES[:8], sections=(), meter=(3, 4))
    if kn.TRIPLE_METER_MODE == "unfit":
        with pytest.raises(ValueError, match="3/4"):
            song.score_song(waltz, seed=1)


_NAMES = ("c", "c#", "d", "d#", "e", "f", "f#", "g", "g#", "a", "a#", "b")


def _rtttl(pitches) -> str:
    return "ref:d=4,o=5,b=120:" + ",".join(f"{_NAMES[m % 12]}{m // 12 - 1}" for m in pitches)


def test_reference_rtttl_starts_the_song_at_the_main_motif_or_refuses_the_material():
    """Узнаваемость — тот же ``hook.for_theme``, что у DJ-трека: контур RTTTL-эталона нашёлся с такта 12 — песня с него
    (мелодия = мелодия автора с такта 12); не нашёлся — материал не годится (``HookError``, заказ уходит в RTTTL)."""
    from rob_box_music.arrange import hook as hooks

    m = scored()
    ref = _rtttl([e.midi for e in m.melody[48:60]])  # такты 12–14: ступени 0 0 4 — только там
    track = song.score_song(m, seed=1, references=[ref])
    lead = track.parts["lead"].pitches
    tail = [e for e in m.melody if e.beat >= 48]
    assert [(e.midi, e.beat) for e in lead] == [(e.midi, e.beat - 48) for e in tail]
    assert track.form.bars_total == 4
    scales = "ref:d=8,o=5,b=132:" + ",".join(["c,d,e,f,g,a,b,c6,b,a,g,f,e,d,c,d"] * 2)
    with pytest.raises(hooks.HookError, match="главного мотива"):
        song.score_song(m, seed=1, references=[scales])
    assert song.score_song(m, seed=1).form.bars_total == 16  # без эталона — вся пьеса, как раньше
