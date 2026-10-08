"""Аудит 07.10 Ф1 (#3529): гармония под ЗВУЧАЩУЮ мелодию каждой секции — одна гармонизация Витерби по выученной
таблице переходов с эмиссией «доля мелодии в тонах аккорда, сильные доли с весом» (``knowledge.HOOK_HARMONY``).

Проверяется поведение (какие аккорды под какой мелодией), а не числа таблиц: аккорд на такт, петля длиной в хук,
build — педаль, break — аккорды вдвое длиннее, b9 на сильной доле штрафуется, уменьшённое — только vii° → I, бас не
стоит на тритоне. Матрица — маленькая (тесты быстрые); полная матрица 1200 треков — скрипт аудита
``scripts/music/research/arranger_theory_audit.py notes``.
"""

from __future__ import annotations

import random
import statistics
from dataclasses import replace

import pytest

from melodies import LONG, MELODIES, profile
from rob_box_music import knowledge as kn
from rob_box_music.arrange import bass, compose as cp, harmony
from rob_box_music.model import BEATS_PER_BAR, Hook, Key, PitchEvent, validate
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import seeded_profile

C = Key(0, "major")
A_MINOR = Key(9, "minor")


def bar(*notes, start=0.0):
    """Ноты такта четвертями (``None`` — пауза) от доли ``start``."""
    return [PitchEvent(m, start + i, 1.0, 2) for i, m in enumerate(notes) if m is not None]


# ── Витерби: эмиссия под звучащую мелодию ──────────────────────────────────────────────────────────────────────

def test_strong_beats_weigh_more_than_weak_ones():
    """Такт E F G A: ноты сильных долей (E, G) — тоны I, слабых (F, A) — тоны IV; при равной длительности побеждает
    аккорд сильных долей."""
    got = harmony.viterbi(C, bar(64, 65, 67, 69), 4.0, 1)
    assert got == (0,)


def test_b9_on_a_strong_beat_is_avoided():
    """Минор, вводный тон G# на сильных долях: натуральная v (E G B) даёт малую нону G#/G — слот берёт мажорную V
    (E G# B, ``knowledge.DEGREE_QUALITIES``, аудит Ф2), в ней вводный тон — тон аккорда."""
    notes = bar(71, 76, 80, 76)
    got = harmony.qualify(A_MINOR, notes, 4.0, harmony.viterbi(A_MINOR, notes, 4.0, 1))
    assert got == (harmony.ChordSym(4, "maj"),)
    assert harmony.strong_b9(A_MINOR, notes, 4) and not harmony.strong_b9(A_MINOR, notes, got[0])


def test_natural_minor_melody_keeps_the_minor_dominant():
    """Мелодия натурального минора (G, не G#) над ступенью 4 — диатоническая v: гармонический V только там, где его
    просит мелодия (ничья — диатоническое качество)."""
    notes = bar(71, 76, 79, 76)
    assert harmony.qualify(A_MINOR, notes, 4.0, (4,)) == (harmony.ChordSym(4, "min"),)
    assert harmony.qualify(A_MINOR, bar(71, 76, 71, 76), 4.0, (4,)) == (harmony.ChordSym(4, "min"),)


def test_chord_per_bar_follows_a_melody_that_changes_every_bar():
    """Слот — такт (``HOOK_HARMONY.slot_bars``): I–IV–V–I по такту узнаются по такту."""
    notes = bar(72, 76, 79, 76) + bar(77, 81, 72, 81, start=4) + bar(79, 83, 74, 83, start=8) + bar(72, 76, 79, 72,
                                                                                                    start=12)
    assert kn.HOOK_HARMONY.slot_bars == 1
    assert harmony.viterbi(C, notes, 4.0, 4, ring=True) == (0, 3, 4, 0)


def test_unresolved_diminished_triads_are_never_chosen():
    """ii° минора, vi° дорийского, v° фригийского — не берутся даже под их собственное трезвучие."""
    for key, dim in ((A_MINOR, 1), (Key(2, "dorian"), 5), (Key(4, "phrygian"), 4)):
        assert dim in harmony.unresolved_dims(key)
        pcs = sorted(harmony.chord_pcs(kn.STYLES["club"], key, dim))
        notes = bar(*(60 + pc for pc in pcs), 60 + pcs[0])
        assert dim not in harmony.viterbi(key, notes * 1, 4.0, 1)


def test_leading_tone_triad_only_resolves_to_the_tonic():
    """vii° мажора — только перед I; последним в незамкнутой цепочке — нет."""
    vii = bar(71, 74, 77, 71)
    tonic = bar(72, 76, 79, 72, start=4)
    other = bar(69, 72, 77, 69, start=4)  # F A C — IV после vii°
    assert harmony.viterbi(C, vii, 4.0, 1) != (6,)
    with_i = harmony.viterbi(C, vii + tonic, 4.0, 2)
    assert with_i[0] != 6 or with_i[1] == 0
    assert harmony.viterbi(C, vii + other, 4.0, 2)[0] != 6


def test_ring_path_scores_the_way_back_to_the_first_chord():
    paths = harmony.paths(C, bar(72, 76, 79, 76) + bar(79, 83, 74, 83, start=4), 4.0, 2, ring=True)
    assert [p for _s, p in paths][0] == (0, 4)
    assert len({p[0] for _s, p in paths}) == len(paths), "по одному пути на каждую первую ступень"


def test_a13_cap_takes_the_next_path_when_the_best_is_played_out():
    notes = bar(72, 76, 79, 76) + bar(79, 83, 74, 83, start=4)
    best = harmony.melody_progression(C, notes, 4.0, 2, ring=True)
    recent = [harmony.progression_name(best)] * harmony.PROGRESSION_CAP
    assert harmony.melody_progression(C, notes, 4.0, 2, ring=True, recent=recent) != best


# ── трек: секции развития и петля ─────────────────────────────────────────────────────────────────────────────

def _sections(track):
    start = 0
    for sec in track.form.sections:
        yield start, sec
        start += sec.bars


def _degrees(track, name):
    return [c.degree for c in track.harmony.progression[name]]


@pytest.fixture(scope="module")
def tracks():
    out = []
    for style in ("club", "rave", "synthwave"):
        for seed in range(2):
            prof = replace(profile(hooks=tuple(MELODIES)), style=style)
            plan = seeded_plan(prof, seed)
            history = []
            for no in (1, 2):
                track = cp.compose(plan, no, melodies=MELODIES, history=history)
                validate(track)
                out.append(track)
    return out


def test_harmony_is_a_chord_per_bar_of_every_section(tracks):
    for track in tracks:
        for _start, sec in _sections(track):
            assert len(track.harmony.progression[sec.name]) == sec.bars


def test_build_is_a_pedal_on_tonic_or_dominant(tracks):
    for track in tracks:
        for name in ("build", "build2"):
            if name in track.harmony.progression:
                degrees = set(_degrees(track, name))
                assert len(degrees) == 1 and degrees <= set(kn.PEDAL_DEGREES), (name, degrees)


def test_break_chords_last_twice_as_long(tracks):
    for track in tracks:
        got = _degrees(track, "break") if "break" in track.harmony.progression else []
        step = kn.HOOK_HARMONY.slot_bars * kn.SECTION_HARMONY["break"]
        assert all(got[i] == got[i - i % step] for i in range(len(got)))


def test_loop_is_as_long_as_the_hook_so_both_halves_of_the_drop_match(tracks):
    """Аудит П2: хук в 4 такта над петлёй в 8 тактов шёл во второй половине на чужие аккорды. Петля = хук."""
    for track in tracks:
        loop = [int(d) for d in track.history_key.progression.split("-")]
        assert len(loop) * kn.HOOK_HARMONY.slot_bars == track.hook.bars if track.hook else len(loop) >= 1
        drop = _degrees(track, "drop")[track.hook.theme_bars if track.hook else 0:]
        assert drop == [loop[i % len(loop)] for i in range(len(drop))]


def test_drop2_thirds_get_a_chord_without_b9_where_one_exists(tracks):
    """drop2 — хук и терции над ним: такт, где терция дала малую нону на сильной доле, получает ступень без b9 ни в
    одном голосе, если такая есть (аудит Ф1: b9 на сильной доле в 82 % треков)."""
    checked = 0
    for track in tracks:
        start = next((b for b, sec in _sections(track) if sec.name == "drop2"), None)
        if start is None:  # форма без drop2 (short32)
            continue
        for at, chord in enumerate(track.harmony.progression["drop2"]):
            lo = (start + at) * BEATS_PER_BAR
            voices = [replace(e, beat=e.beat - lo) for e in track.parts["lead"].pitches if lo <= e.beat < lo + 4]
            clean = [d for d in range(7) if d not in harmony.diminished(track.key)
                     and not harmony.strong_b9(track.key, voices, d)]
            if clean:
                checked += 1
                assert not harmony.strong_b9(track.key, voices, chord.degree), (start + at, chord.degree, clean)
    assert checked


def test_pad_voice_leading_stays_close_across_the_whole_track(tracks):
    for track in tracks:
        chords = [c for name in [s.name for _b, s in _sections(track)] for c in track.harmony.progression[name]]
        leaps = [max(abs(x - y) for x, y in zip(sorted(a.voicing), sorted(b.voicing))) for a, b in zip(chords, chords[1:])]
        assert max(leaps) <= 7


def _strong_ct(notes, degrees, slot_bars, style, key):
    """Доля нот сильных долей (1 и 3) в тонах аккорда петли ``degrees`` (аккорд держится ``slot_bars`` тактов)."""
    hits = [e.midi % 12 in harmony.chord_pcs(style, key, degrees[int(e.beat // (slot_bars * BEATS_PER_BAR)) % len(degrees)])
            for e in notes if e.beat % 2 == 0]
    return sum(hits) / len(hits)


def test_melody_on_strong_beats_is_in_the_chord_more_often_than_under_style_templates(tracks):
    """Аудит П1: шаблон стиля на 2 такта (как было, ``fit_progression``) против гармонии под хук — на каждом хуке
    не хуже на 0.08, в среднем лучше на 0.1 (матрица аудита: 0.51 → цель ≥ 0.70 — скриптом аудита)."""
    pairs = []
    for track in tracks:
        style = kn.STYLES[track.style]
        loop = [int(d) for d in track.history_key.progression.split("-")]
        old = harmony.fit_progression(style, track.key, track.hook.notes, 2 * BEATS_PER_BAR, random.Random(0))
        pairs.append((_strong_ct(track.hook.notes, loop, kn.HOOK_HARMONY.slot_bars, style, track.key),
                      _strong_ct(track.hook.notes, old, 2, style, track.key)))
    assert all(new >= old + 0.08 for new, old in pairs), pairs
    assert statistics.mean(new - old for new, old in pairs) >= 0.1, pairs


def test_no_tritone_in_the_bass_on_a_diminished_triad():
    style = kn.STYLES["club"]
    pcs = harmony.chord_pcs(style, C, 6)  # B D F
    notes = bass.bar_notes(pcs[0], pcs[2], style.registers["bass"])
    assert {n % 12 for n in notes} == {pcs[0]}
    figure = kn.BASS_FIGURES["offbeat"]
    tones = {0: bass.BassTone(bass.ANCHORS["fifth"])}
    got = bass.figure_bass(figure, [(0, pcs)], style.registers["bass"], tones)
    assert all((e.midi - pcs[0]) % 12 != bass.TRITONE for e in got)


def test_rtttl_theme_track_has_no_unresolved_diminished_chord():
    prof = seeded_profile("космос", "rave")
    plan = seeded_plan(prof, 0)
    track = cp.compose(plan, 1, melodies={"long": LONG, **MELODIES})
    banned = harmony.unresolved_dims(track.key)
    assert not any(c.degree in banned for chords in track.harmony.progression.values() for c in chords)


def test_hook_without_theme_keeps_working_when_the_section_melody_is_silent():
    """Секция развития без нот (хук из одной ноты в конце) — гармония по таблице, не падение."""
    hook = Hook(tuple(bar(None, None, None, 72, start=12)), 4, None)
    spec = cp.form_spec(kn.STYLES["club"], "club48")
    fixed = cp.melody_harmonizer(kn.STYLES["club"], C, (), random.Random(0))
    arranged = cp._arrange(kn.STYLES["club"], spec, hook, C, "blip", fixed)
    assert len(arranged.chords) == sum(bars for _n, bars, _e, _r in spec)
