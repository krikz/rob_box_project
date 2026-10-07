"""ADR-0154 PR-3: гармония из материала партитуры (``harmony.from_material``), выученная таблица переходов
(``knowledge.PROGRESSION_TRANSITIONS``), запасной Витерби и ``compose`` по ``TrackPlan.material``.

Материал — синтетический (наш, не чужая партитура): прогрессия известна по построению, M3 на синтетике — точное
совпадение ступеней слотов с аккордами «оригинала», кроме случаев, где адаптация обязана их поменять.
"""

from __future__ import annotations

import math
from dataclasses import replace
from typing import Sequence, Tuple

import pytest

from rob_box_music import knowledge as kn
from rob_box_music import material as mt
from rob_box_music.arrange import compose as cp
from rob_box_music.arrange import harmony
from rob_box_music.model import Key, PitchEvent
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import ThemeProfile

#: Ноты такта (четверти) на трезвучии ступени до мажора / ля минора, в коридоре лида 62..84.
MAJOR_BAR = {0: (72, 76, 79, 76), 1: (74, 77, 81, 77), 3: (77, 81, 84, 81), 4: (71, 74, 79, 74), 5: (69, 72, 76, 72)}
MINOR_BAR = {0: (69, 72, 76, 72), 3: (74, 77, 81, 77), 4: (71, 76, 80, 76), 5: (65, 69, 72, 69)}
CHORD_BEATS = cp.CHORD_BARS * 4


def synthetic(degrees_per_bar: Sequence[int], *, meter: Tuple[int, int] = (4, 4), key: Key = Key(0, "major"),
              chords=None, bpm=None, lead_in: float = 0.0) -> mt.ScoreMaterial:
    """Материал: такт на ступень ``degrees_per_bar``; аккорд такта — трезвучие ступени (диатоническое)."""
    bar = mt.bar_beats(meter)
    table, home = (MAJOR_BAR, 0) if key.mode == "major" else (MINOR_BAR, 9)
    scale = kn.SCALES[key.mode]
    melody, spans = [], []
    for i, d in enumerate(degrees_per_bar):
        for k, midi in enumerate(table[d][:meter[0]]):
            beat = i * bar + k * bar / meter[0]
            if beat >= lead_in:
                melody.append(PitchEvent(midi + key.root - home, beat, bar / meter[0], 3 if k == 0 else 2))
        quality = "maj" if (scale[(d + 2) % 7] - scale[d]) % 12 == 4 else "min"
        spans.append(mt.ChordSpan(i * bar, bar, (key.root + scale[d]) % 12, quality, d))
    return mt.ScoreMaterial(material_id="local:synth01", title="Синтетика PR-3", composer="test", source="synthetic",
                            license="PD", meter=meter, bpm=bpm, key=key, melody=tuple(melody),
                            chords=tuple(spans) if chords is None else chords)


def hook_notes(material: mt.ScoreMaterial) -> Tuple[PitchEvent, ...]:
    return tuple(e for e in material.melody if e.beat < 32)


def from_material(material, slots=4, scale=1.0, key=None):
    phrase = mt.Phrase(0, 8, "new", 1)
    key = key or material.key
    return harmony.from_material(material, phrase, key, hook_notes(material), CHORD_BEATS, slots, scale)


# ── таблица переходов (данные knowledge) ─────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("mode", ["major", "minor"])
def test_transition_table_rows_are_distributions(mode):
    table = kn.PROGRESSION_TRANSITIONS[mode]
    assert len(table["start"]) == 7 and len(table["next"]) == 7
    assert math.isclose(sum(table["start"]), 1.0, abs_tol=1e-3)
    for row in table["next"]:
        assert len(row) == 7 and all(p > 0 for p in row)
        assert math.isclose(sum(row), 1.0, abs_tol=1e-3)


@pytest.mark.parametrize("mode", ["major", "minor"])
def test_dominant_resolves_to_tonic_in_corpus_table(mode):
    """Свойства корпуса (Н5), а не числа побайтно: из V чаще всего в I; из I — в V и IV (топ-2)."""
    nxt = kn.PROGRESSION_TRANSITIONS[mode]["next"]
    assert max(range(7), key=lambda d: nxt[4][d]) == 0
    assert set(sorted(range(1, 7), key=lambda d: -nxt[0][d])[:2]) == {3, 4}


@pytest.mark.parametrize("mode", ["major", "minor"])
def test_piece_starts_on_tonic_far_more_often_than_on_any_other_degree(mode):
    """Свойство корпуса: пьеса открывается тоникой (PDMX, ADR-0154 PR-6) — с заметным отрывом от второй ступени."""
    start = kn.PROGRESSION_TRANSITIONS[mode]["start"]
    ranked = sorted(range(7), key=lambda d: -start[d])
    assert ranked[0] == 0 and start[0] > 2 * start[ranked[1]]


def test_table_has_provenance():
    prov = kn.PROGRESSION_TRANSITIONS_PROVENANCE
    assert prov["scores"] >= 70 and len(prov["corpus_sha256"]) == 64 and "score_markov_harmony" in prov["script"]


@pytest.mark.parametrize("mode, table", [("major", "major"), ("lydian", "major"), ("mixolydian", "major"),
                                         ("minor", "minor"), ("dorian", "minor"), ("harmonicMinor", "minor")])
def test_transition_table_by_mode_family(mode, table):
    assert harmony.transition_table(mode) is kn.PROGRESSION_TRANSITIONS[table]


@pytest.mark.parametrize("mode", ["majorPentatonic", "minorPentatonic", kn.CHROMATIC])
def test_transition_table_refuses_non_heptatonic(mode):
    with pytest.raises(ValueError, match="семиступенный"):
        harmony.transition_table(mode)


# ── гармония из материала: M3 на синтетике ────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("bars, expected", [
    ((0, 0, 3, 3, 4, 4, 0, 0), (0, 3, 4, 0)),
    ((0, 0, 5, 5, 3, 3, 4, 4), (0, 5, 3, 4)),
    ((5, 5, 3, 3, 0, 0, 4, 4), (5, 3, 0, 4)),
])
def test_material_chords_become_slot_degrees(bars, expected):
    assert from_material(synthetic(bars)) == expected


def test_minor_material_keeps_author_degrees():
    assert from_material(synthetic((0, 0, 3, 3, 4, 4, 0, 0), key=Key(9, "minor"))) == (0, 3, 4, 0)


def test_degrees_do_not_depend_on_track_tonic():
    m = synthetic((0, 0, 5, 5, 3, 3, 4, 4))
    assert from_material(m, key=Key(7, "major")) == from_material(m) == (0, 5, 3, 4)


def test_three_four_is_padded_bar_by_bar():
    """3/4: такт материала → такт 4/4 с паузой на 4-й доле (В5 (б)) — слот = 2 такта материала, как у 4/4."""
    assert from_material(synthetic((0, 0, 3, 3, 4, 4, 0, 0), meter=(3, 4))) == (0, 3, 4, 0)


def test_time_scale_stretches_material_slots():
    """Множитель темпа 2 (тема вдвое медленнее клуба): 8 тактов трека = 4 такта материала, слот = такт."""
    assert from_material(synthetic((0, 3, 4, 0, 5, 5, 5, 5)), scale=2.0) == (0, 3, 4, 0)


def test_slot_chord_is_the_longest_one():
    chords = (mt.ChordSpan(0.0, 3.0, 0, "maj", 0), mt.ChordSpan(3.0, 5.0, 5, "maj", 3),
              mt.ChordSpan(8.0, 8.0, 7, "maj", 4), mt.ChordSpan(16.0, 16.0, 0, "maj", 0))
    m = synthetic((0, 0, 4, 4, 0, 0, 0, 0), chords=chords)
    assert from_material(m) == (3, 4, 0, 0)


def test_lead_in_rest_shifts_slots_with_the_hook():
    """Хук срезает начальную паузу (``hook._onsets``): слоты гармонии считаются от первой ноты фразы."""
    m = synthetic((0, 0, 3, 3, 4, 4, 0, 0), lead_in=1.0)
    slots = harmony.material_slots(m, mt.Phrase(0, 8, "new", 1), CHORD_BEATS, 4)
    assert [c.degree for c in slots] == [0, 3, 4, 0]


@pytest.mark.parametrize("root_pc, quality, expected", [
    (4, "maj", 4),    # E мажор в ля миноре (гармонический V) → v: E-G общие
    (10, "maj", 1),   # B♭ мажор (неаполитанский): D-F общие и с ii°, и с iv — ничья → меньшая ступень (ii°)
    (4, "dom7", 4),   # E7 → v
])
def test_non_diatonic_chord_adapts_to_common_tones(root_pc, quality, expected):
    chord = mt.ChordSpan(0.0, 4.0, root_pc, quality, None)
    assert harmony._adapt(chord, Key(9, "minor")) == expected


def test_non_diatonic_without_common_tones_is_left_to_viterbi():
    assert harmony._adapt(mt.ChordSpan(0.0, 4.0, 1, "other", None), Key(0, "major")) is None


def test_material_without_chords_falls_back_to_viterbi():
    m = synthetic((0, 0, 3, 3, 4, 4, 0, 0), chords=())
    assert from_material(m) == harmony.viterbi(m.key, hook_notes(m), CHORD_BEATS, 4)


def test_single_chord_phrase_keeps_first_slot_and_moves_by_table():
    """Фраза на одном аккорде (Н8, sustained) — Витерби от авторского первого слота, а не петля без движения."""
    m = synthetic((0,) * 8, chords=(mt.ChordSpan(0.0, 32.0, 0, "maj", 0),))
    moved = list(harmony.viterbi(m.key, hook_notes(m), CHORD_BEATS, 4, [0]))
    got = from_material(m)
    assert got[0] == 0 and got == tuple(harmony._cadence(moved, kn.PROGRESSION_TRANSITIONS["major"]))


def test_loop_without_way_back_gets_a_cadence():
    table = kn.PROGRESSION_TRANSITIONS["major"]
    pairs = ((a, b) for a in range(7) for b in range(7) if a != b)
    last, first = min(pairs, key=lambda ab: table["next"][ab[0]][ab[1]])
    assert table["next"][last][first] < harmony.CADENCE_MIN_P
    middle = [d for d in range(7) if d not in (first, last)][:2]
    got = harmony._cadence([first, *middle, last], table)
    assert got[:3] == [first, *middle] and got[3] != last
    assert table["next"][middle[1]][got[3]] * table["next"][got[3]][first] > (
        table["next"][middle[1]][last] * table["next"][last][first])


def test_loop_with_way_back_keeps_author_chords():
    """В мажоре любая ступень ходит в I с P ≥ порога — петля, начатая с I, авторская целиком."""
    table = kn.PROGRESSION_TRANSITIONS["major"]
    assert all(table["next"][d][0] >= harmony.CADENCE_MIN_P for d in range(1, 7))
    assert harmony._cadence([0, 3, 1, 6], table) == [0, 3, 1, 6]


def test_mode_mismatch_is_refused():
    with pytest.raises(ValueError, match="лад"):
        from_material(synthetic((0, 0, 3, 3, 4, 4, 0, 0)), key=Key(0, "minor"))


# ── Витерби (запасной путь, альтернатива F) ───────────────────────────────────────────────────────────────────

def test_viterbi_follows_melody_triads():
    m = synthetic((0, 0, 3, 3, 4, 4, 0, 0))
    assert harmony.viterbi(m.key, hook_notes(m), CHORD_BEATS, 4) == (0, 3, 4, 0)


def test_viterbi_respects_fixed_slots_and_is_deterministic():
    m = synthetic((0, 0, 3, 3, 4, 4, 0, 0))
    a = harmony.viterbi(m.key, hook_notes(m), CHORD_BEATS, 4, [None, 5, None, None])
    assert a[1] == 5 and a == harmony.viterbi(m.key, hook_notes(m), CHORD_BEATS, 4, [None, 5, None, None])


def test_viterbi_without_melody_is_the_markov_chain():
    """Без нот выбор — только таблица: старт — самая частая первая ступень, дальше — ходы цепи."""
    table = kn.PROGRESSION_TRANSITIONS["major"]
    got = harmony.viterbi(Key(0, "major"), (), CHORD_BEATS, 1)
    assert got == (max(range(7), key=lambda d: (table["start"][d], -d)),)


# ── compose по TrackPlan.material ─────────────────────────────────────────────────────────────────────────────

def _plan(material_id=None, seed=7):
    profile = ThemeProfile("", kn.DEFAULT_STYLE, 132, 0, "major", (), None)
    plan = seeded_plan(profile, seed)
    return replace(plan, tracks=(replace(plan.track(1), material=material_id),))


def test_compose_takes_hook_and_harmony_from_material():
    m = synthetic((0, 0, 5, 5, 3, 3, 4, 4))
    track = cp.compose(_plan(m.material_id), 1, materials={m.material_id: m})
    assert track.hook is not None and track.hook.source == m.material_id
    assert track.history_key.progression == "0-5-3-4"
    assert {tuple(c.degree for c in chords) for chords in track.harmony.progression.values()} == {(0, 5, 3, 4)}


def test_compose_without_material_is_unchanged():
    """Гард: None и неизвестный/негодный материал — тот же трек, что до PR-3 (материал не входит в sha)."""
    base = cp.compose(_plan(None), 1)
    assert cp.compose(_plan(None), 1, materials={"local:synth01": synthetic((0,) * 8)}) == base
    assert cp.compose(_plan("local:missing"), 1, materials={}) == base
    pentatonic = replace(synthetic((0, 0, 3, 3, 4, 4, 0, 0)), key=Key(0, "majorPentatonic"))
    assert cp.compose(_plan("local:synth01"), 1, materials={"local:synth01": pentatonic}) == base
