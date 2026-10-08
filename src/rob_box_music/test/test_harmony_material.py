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
from rob_box_music.model import Key, PitchEvent, chord_tones, validate
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import ThemeProfile

#: Ноты такта (четверти) на трезвучии ступени до мажора / ля минора, в коридоре лида 62..84.
MAJOR_BAR = {0: (72, 76, 79, 76), 1: (74, 77, 81, 77), 3: (77, 81, 84, 81), 4: (71, 74, 79, 74), 5: (69, 72, 76, 72)}
MINOR_BAR = {0: (69, 72, 76, 72), 3: (74, 77, 81, 77), 4: (71, 76, 80, 76), 5: (65, 69, 72, 69)}
CHORD_BEATS = 8  # слот функций гармонии в этих тестах — 2 такта (синтетика: аккорд на 2 такта, как M3)


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


def from_material(material, slots=4, scale=1.0, key=None, notes=None):
    """``harmony.from_material`` по фразе 0..8 тактов; мелодия — хук материала в тональности ``key`` (как в треке:
    хук перенесён в тонику трека); ``notes=()`` — без мелодии (проверка чтения слотов, а не мелодии)."""
    phrase = mt.Phrase(0, 8, "new", 1)
    key = key or material.key
    if notes is None:
        shift = key.root - material.key.root
        notes = tuple(replace(e, midi=e.midi + shift) for e in hook_notes(material))
    return harmony.from_material(material, phrase, key, notes, CHORD_BEATS, slots, scale)


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
    """Свойства корпуса (Н5), а не числа побайтно: из V чаще всего в I; из I — чаще всего в V, а IV в топ-3
    (на полном PDMX ii обогнал IV на волосок — 0.194 против 0.186 в major, поэтому «топ-2» было свойством малого корпуса)."""
    nxt = kn.PROGRESSION_TRANSITIONS[mode]["next"]
    assert max(range(7), key=lambda d: nxt[4][d]) == 0
    top3 = sorted(range(1, 7), key=lambda d: -nxt[0][d])[:3]
    assert top3[0] == 4 and 3 in top3


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
    """Мелодия натурального минора — ступени автора как есть."""
    m = synthetic((0, 0, 3, 3, 4, 4, 0, 0), key=Key(9, "minor"))
    natural = tuple(replace(e, midi=e.midi - 1) if e.midi % 12 == 8 else e for e in hook_notes(m))  # G# → G
    assert from_material(m, notes=natural) == (0, 3, 4, 0)


def _with_major_dominant(m: mt.ScoreMaterial) -> mt.ScoreMaterial:
    """Аккорды автора — как у ``synthetic``, но ступень 4 минора — мажорная V (E G# B), как у классиков."""
    return replace(m, chords=tuple(replace(c, quality="maj") if c.degree == 4 else c for c in m.chords))


def test_author_dominant_keeps_its_quality_and_the_leading_tone():
    """Аудит Ф2/П4 (#3530): V автора (E G# B) с вводным тоном в мелодии (G# на сильной доле) переносится как есть —
    ступень 4 и мажорное качество (гармонический минор), а не натуральная v (E G B) с малой ноной G#/G. Ступени слотов
    — автора целиком."""
    m = _with_major_dominant(synthetic((0, 0, 3, 3, 4, 4, 0, 0), key=Key(9, "minor")))
    assert any(e.midi % 12 == 8 and e.beat % 2 == 0 for e in hook_notes(m)), "G# на сильной доле — условие теста"
    got = harmony.material_chords(m, mt.Phrase(0, 8, "new", 1), m.key, hook_notes(m), CHORD_BEATS, 4)
    assert got == (harmony.ChordSym(0, "min"), harmony.ChordSym(3, "min"), harmony.ChordSym(4, "maj"),
                   harmony.ChordSym(0, "min"))
    assert from_material(m) == (0, 3, 4, 0)
    slot = [e for e in hook_notes(m) if 16 <= e.beat < 24]
    assert not harmony.strong_b9(m.key, [replace(e, beat=e.beat - 16) for e in slot], got[2])


@pytest.mark.parametrize("root, quality, degree, track_root", [
    (4, "maj", 4, 2),     # V в ля миноре → V в ре миноре: A C# E
    (11, "maj", 1, 5),    # II# (V/V) в ля миноре — мажорная, не ii°
    (2, "maj", 3, 9),     # IV (мелодический минор) — мажорная, не iv
    (9, "maj", 0, 0),     # пикардийская I
    (4, "dom7", 4, 9),    # V7
])
def test_author_quality_survives_the_transfer_to_the_track_key(root, quality, degree, track_root):
    """Перенос аккорда автора в тональность трека: та же ступень и то же качество (аудит Ф2, #3530)."""
    author = mt.ChordSpan(0.0, 4.0, root, quality, degree)
    got = harmony.author_chord(author, Key(9, "minor"))
    assert got == harmony.ChordSym(degree, quality)
    track = Key(track_root, "minor")
    shift = track_root - 9
    pcs = harmony.chord_pcs(kn.STYLES["club"], track, got.degree, got.quality)
    assert set(pcs) == {(root + shift + i) % 12 for i in kn.CHORD_INTERVALS[quality][:3]}


def test_leading_tone_chord_of_the_author_becomes_the_major_dominant():
    """vii° автора в миноре (G# B D в ля миноре) — прима вне натурального лада: приводится к аккорду лада с наибольшим
    числом общих звуков среди диатонических и гармонического V — мажорная V (E G# B), а не VII (G B D) с натуральной
    VII под вводным тоном."""
    assert harmony.author_chord(mt.ChordSpan(0.0, 4.0, 8, "dim", None), Key(9, "minor")) == harmony.ChordSym(4, "maj")


def test_author_own_non_chord_tone_keeps_the_author_chord():
    """Тот же G# над минорной v, которую написал сам автор (E G B): неаккордовый тон автора — его замысел (хроматика
    Грига — не баг, аудит П7), а не перенос; ступень автора остаётся."""
    m = synthetic((0, 0, 3, 3, 4, 4, 0, 0), key=Key(9, "minor"))
    assert from_material(m) == (0, 3, 4, 0)


def test_degrees_do_not_depend_on_track_tonic():
    m = synthetic((0, 0, 5, 5, 3, 3, 4, 4))
    assert from_material(m, key=Key(7, "major")) == from_material(m) == (0, 5, 3, 4)


@pytest.mark.parametrize("mode", ["pause", "stretch", "long", "lift"])
def test_three_four_reads_bar_by_bar_in_every_mode(monkeypatch, mode):
    """3/4: такт материала → такт 4/4 (пауза, растяжение, долгая доля — #3517) — слот = 2 такта материала, как у 4/4."""
    monkeypatch.setattr(kn, "TRIPLE_METER_MODE", mode)
    assert from_material(synthetic((0, 0, 3, 3, 4, 4, 0, 0), meter=(3, 4)), notes=()) == (0, 3, 4, 0)


def test_time_scale_stretches_material_slots():
    """Множитель темпа 2 (тема вдвое медленнее клуба): 8 тактов трека = 4 такта материала, слот = такт."""
    assert from_material(synthetic((0, 3, 4, 0, 5, 5, 5, 5)), scale=2.0, notes=()) == (0, 3, 4, 0)


def test_slot_chord_is_the_longest_one():
    chords = (mt.ChordSpan(0.0, 3.0, 0, "maj", 0), mt.ChordSpan(3.0, 5.0, 5, "maj", 3),
              mt.ChordSpan(8.0, 8.0, 7, "maj", 4), mt.ChordSpan(16.0, 16.0, 0, "maj", 0))
    m = synthetic((0, 0, 4, 4, 0, 0, 0, 0), chords=chords)
    assert from_material(m) == (3, 4, 0, 0)


def test_lead_in_rest_shifts_slots_with_the_hook():
    """Слоты гармонии считаются от начала такта фразы, как и хук материала (#3531)."""
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
    loop = (0, 0, 5, 5, 3, 3, 4, 4)  # аккорд на такт (#3529): петля — 8 тактов хука
    assert track.history_key.progression == "-".join(map(str, loop))
    for name in ("drop", "intro", "outro"):
        assert tuple(c.degree for c in track.harmony.progression[name]) == (loop * 2)[:len(
            track.harmony.progression[name])], name


def test_minor_track_plays_the_author_major_dominant():
    """Аудит Ф2 (#3530) на треке целиком: V автора в миноре — мажорный аккорд такта (``Chord.quality``), пэд играет
    вводный тон (вне натурального лада), трек проходит валидатор; вводный тон мелодии не звучит над натуральной VII."""
    m = _with_major_dominant(synthetic((0, 0, 3, 3, 4, 4, 0, 0), key=Key(9, "minor")))
    plan = _plan(m.material_id)
    plan = replace(plan, profile=replace(plan.profile, mode="minor"))
    track = cp.compose(plan, 1, materials={m.material_id: m})
    assert track.hook is not None and track.hook.source == m.material_id
    validate(track)
    leading, b7 = (track.key.root + 11) % 12, (track.key.root + 10) % 12
    by_bar = dict(_form_bars(track))
    dominants = [c for c in by_bar.values() if c.degree == 4]
    assert dominants and all(c.quality == "maj" and leading in {v % 12 for v in c.voicing} for c in dominants)
    assert any(e.midi % 12 == leading for e in track.parts["pad"].pitches)
    size = kn.STYLES[track.style].chord_size
    clashes = [e for e in track.parts["lead"].pitches if e.midi % 12 == leading
               and b7 in chord_tones(track.key, by_bar[int(e.beat // 4)], size)]
    assert not clashes


def _form_bars(track):
    bar = 0
    for sec in track.form.sections:
        for i, chord in enumerate(track.harmony.progression.get(sec.name, ())):
            yield bar + i, chord
        bar += sec.bars


def test_compose_without_material_is_unchanged():
    """Гард: None и неизвестный/негодный материал — тот же трек, что до PR-3 (материал не входит в sha)."""
    base = cp.compose(_plan(None), 1)
    assert cp.compose(_plan(None), 1, materials={"local:synth01": synthetic((0,) * 8)}) == base
    assert cp.compose(_plan("local:missing"), 1, materials={}) == base
    pentatonic = replace(synthetic((0, 0, 3, 3, 4, 4, 0, 0)), key=Key(0, "majorPentatonic"))
    assert cp.compose(_plan("local:synth01"), 1, materials={"local:synth01": pentatonic}) == base
