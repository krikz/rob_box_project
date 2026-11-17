"""ADR-0154 PR-4: бас из материала партитуры (``bass.material_tones``, ``knowledge.BASS_TONES``), развитие ``rhythm``
(``Hook.answer``) и 3/4 → 4/4 паузой на 4-й доле для баса.

Материал — синтетический (наш, ``test_harmony_material.synthetic``): бас автора задан по построению, поэтому
ожидаемый тон такта (прима/терция/квинта/подход) известен заранее. Гард «без материала ни байта» — здесь на уровне
генератора и ``compose``; на 50 сидах × 5 тем — ``test_style_same_tracks``.
"""

from __future__ import annotations

import math
from dataclasses import replace

import pytest

from rob_box_music import knowledge as kn
from rob_box_music import material as mt
from rob_box_music.arrange import bass, harmony
from rob_box_music.arrange import compose as cp
from rob_box_music.arrange import hook as hooks
from rob_box_music.model import APPROACH_MAX_BEATS, BEATS_PER_BAR, Key, PitchEvent, validate
from test_harmony_material import _plan, synthetic

STYLE = kn.STYLES[kn.DEFAULT_STYLE]
REGISTER = STYLE.registers["bass"]
PHRASE = mt.Phrase(0, 8, "new", 1)
CHORD_BARS = 2  # слот синтетики — 2 такта (по такту на ступень, два такта на аккорд)
CHORD_BEATS = CHORD_BARS * BEATS_PER_BAR
LOOP = (0, 5, 3, 4)  # I vi IV V — синтетика «по такту на ступень, два такта на аккорд»
ROOTS = {0: 36, 5: 45, 3: 41, 4: 43}  # прима ступени до мажора в басу автора


def with_bass(m: mt.ScoreMaterial, notes) -> mt.ScoreMaterial:
    """Басовый голос автора: ``(midi, доля, длительность)``."""
    return replace(m, bass=tuple(PitchEvent(midi, beat, dur, 2) for midi, beat, dur in notes))


def root_bass(m: mt.ScoreMaterial, degrees) -> mt.ScoreMaterial:
    bar = mt.bar_beats(m.meter)
    return with_bass(m, [(ROOTS[d], i * bar, bar) for i, d in enumerate(degrees)])


def tones(m: mt.ScoreMaterial, degrees=LOOP, scale=1.0):
    return bass.material_tones(STYLE, m, PHRASE, degrees, CHORD_BEATS, scale)


BARS = tuple(d for d in LOOP for _ in range(CHORD_BARS))


# ── таблица (данные knowledge, выучены на корпусе) ──────────────────────────────────────────────────────────────

@pytest.mark.parametrize("mode", ["major", "minor"])
def test_bass_tones_are_a_distribution_with_root_on_top(mode):
    """Свойства корпуса (Н7), а не числа побайтно: доли в сумме 1, прима — самый частый тон, подход — редкость."""
    t = kn.BASS_TONES[mode]
    assert math.isclose(sum(t[r] for r in kn.BASS_TONE_RELATIONS), 1.0, abs_tol=1e-3)
    assert max(kn.BASS_TONE_RELATIONS, key=t.__getitem__) == "root"
    assert min(t["third"], t["fifth"]) > 0.1  # обращения и квинта — не шум (Н7: ≈ 20 % каждая)
    assert 0 < t["approach_per_bar"] < 0.5


def test_bass_tones_have_provenance():
    prov = kn.BASS_TONES_PROVENANCE
    assert prov["script"].startswith("scripts/music/research/score_bass_tones.py")
    assert prov["scores"] >= 70 and len(prov["corpus_sha256"]) == 64


# ── тон такта по басу автора ────────────────────────────────────────────────────────────────────────────────────

def test_root_bass_gives_root_tones_without_approaches():
    assert tones(root_bass(synthetic(BARS), BARS)) == (bass.ROOT_TONE,) * 8


def test_material_without_bass_takes_the_corpus_default():
    default = bass.ANCHORS[max(bass.ANCHORS, key=kn.BASS_TONES["major"].__getitem__)]
    assert tones(synthetic(BARS)) == (bass.BassTone(default, 0),) * 8


def test_inversion_of_the_author_is_kept():
    """Такт 2 (vi = ля минор) у автора стоит на до — терция аккорда; такт 4 (IV = фа) на до — квинта."""
    notes = [(ROOTS[d], i * 4.0, 4.0) for i, d in enumerate(BARS)]
    notes[2] = (48, 8.0, 4.0)
    notes[4] = (48, 16.0, 4.0)
    got = tones(with_bass(synthetic(BARS), notes))
    assert [t.anchor for t in got] == [0, 0, 1, 0, 2, 0, 0, 0]


def test_foreign_note_falls_back_to_the_default_tone():
    notes = [(ROOTS[d], i * 4.0, 4.0) for i, d in enumerate(BARS)]
    notes[6] = (42, 24.0, 4.0)  # фа-диез на V (соль) — чужая нота
    assert tones(with_bass(synthetic(BARS), notes))[6] == bass.ROOT_TONE


def test_longest_note_of_the_bar_decides():
    notes = [(ROOTS[d], i * 4.0, 4.0) for i, d in enumerate(BARS) if i != 2]
    notes += [(45, 8.0, 1.0), (48, 9.0, 3.0)]  # ля четверть, до — три четверти
    got = tones(with_bass(synthetic(BARS), sorted(notes, key=lambda n: n[1])))
    assert got[2].anchor == 1


def test_author_approach_by_semitone_is_taken_with_its_side():
    """Такт 3 (ля): ля, затем ми на последней доле → фа такта 4 — подход снизу; сверху — фа-диез."""
    base = [(ROOTS[d], i * 4.0, 4.0) for i, d in enumerate(BARS) if i != 3]
    below = sorted(base + [(45, 12.0, 3.0), (40, 15.0, 1.0)], key=lambda n: n[1])
    above = sorted(base + [(45, 12.0, 3.0), (42, 15.0, 1.0)], key=lambda n: n[1])
    assert tones(with_bass(synthetic(BARS), below))[3] == bass.BassTone(0, -1)
    assert tones(with_bass(synthetic(BARS), above))[3] == bass.BassTone(0, 1)


def test_approaches_are_capped_by_the_corpus_rate():
    """Подход на каждом стыке у автора — в петле остаётся ``round(approach_per_bar × 8)``, самые ранние."""
    notes = []
    for i, d in enumerate(BARS):
        target = ROOTS[BARS[(i + 1) % 8]]
        notes += [(ROOTS[d], i * 4.0, 3.0), (target - 1, i * 4.0 + 3, 1.0)]
    got = tones(with_bass(synthetic(BARS), notes))
    cap = round(kn.BASS_TONES["major"]["approach_per_bar"] * 8)
    assert [i for i, t in enumerate(got) if t.approach] == list(range(cap))


def test_three_four_bass_is_read_with_a_rest_on_the_fourth_beat(monkeypatch):
    """3/4 → 4/4 (В5 (б)): такт материала встаёт в такт клуба, 4-я доля — пауза; тон и подход читаются по тактам."""
    monkeypatch.setattr(kn, "TRIPLE_METER_MODE", "pause")
    m = synthetic(BARS, meter=(3, 4))
    notes = [(ROOTS[d], i * 3.0, 3.0) for i, d in enumerate(BARS)]
    notes[2] = (48, 6.0, 3.0)
    notes[3:4] = [(45, 9.0, 2.0), (40, 11.0, 1.0)]  # ля, ми на 3-й доле → фа такта 4
    got = tones(with_bass(m, notes))
    assert got[2].anchor == 1 and got[3] == bass.BassTone(0, -1)


def test_time_scale_reads_the_author_bar_under_two_track_bars():
    """Множитель темпа 2: такт автора — два такта трека; до в такте 1 автора на vi трека (такты 2–3) — терция."""
    m = with_bass(synthetic(BARS), [(48 if i == 1 else ROOTS[d], i * 4.0, 4.0) for i, d in enumerate(BARS)])
    assert [t.anchor for t in tones(m, scale=2.0)[:4]] == [0, 0, 1, 1]


# ── генератор: регистр, комплементарность бочке, подход ──────────────────────────────────────────────────────────

def _bars(degrees, key=Key(0, "major")):
    return [(i, harmony.chord_pcs(STYLE, key, d)) for i, d in enumerate(degrees)]


@pytest.mark.parametrize("figure", sorted(kn.BASS_FIGURES))
def test_tones_keep_steps_register_and_end_by_the_beat(figure):
    fig = kn.BASS_FIGURES[figure]
    bars = _bars(BARS)
    by_bar = {i: bass.BassTone(i % 3, (-1, 1, 0)[i % 3]) for i in range(len(bars))}
    events = bass.figure_bass(fig, bars, REGISTER, by_bar)
    plain = bass.figure_bass(fig, bars, REGISTER)
    assert [e.beat for e in events] == [e.beat for e in plain]  # шаги рисунка те же — мимо бочки, как было
    assert all(round(e.beat * 4) % 4 != 0 for e in events)
    assert all(REGISTER[0] <= e.midi <= REGISTER[1] for e in events)
    assert all(e.beat + e.dur_beats <= int(e.beat) + 1 + 1e-9 for e in events)


@pytest.mark.parametrize("figure", sorted(kn.BASS_FIGURES))
def test_root_tones_are_byte_identical_to_no_tones(figure):
    fig = kn.BASS_FIGURES[figure]
    bars = _bars(BARS)
    plain = bass.figure_bass(fig, bars, REGISTER)
    assert bass.figure_bass(fig, bars, REGISTER, {i: bass.ROOT_TONE for i in range(8)}) == plain
    assert bass.figure_bass(fig, bars, REGISTER, None) == plain


def test_anchor_puts_the_bar_on_the_chord_tone():
    pcs = harmony.chord_pcs(STYLE, Key(0, "major"), 5)
    events = bass.figure_bass(kn.BASS_FIGURES["offbeat"], [(0, pcs)], REGISTER, {0: bass.BassTone(1)})
    assert [e.midi % 12 for e in events] == [pcs[1]] * 3 + [pcs[2]]


def test_approach_is_a_short_semitone_into_the_next_bar():
    fig = kn.BASS_FIGURES["offbeat"]
    events = bass.figure_bass(fig, _bars((0, 3)), REGISTER, {0: bass.BassTone(0, -1)})
    last, nxt = events[len(fig.steps) - 1], events[len(fig.steps)]
    assert last.midi == nxt.midi - 1 and last.dur_beats <= APPROACH_MAX_BEATS


def test_approach_flips_side_at_the_register_edge():
    fig = kn.BASS_FIGURES["offbeat"]
    events = bass.figure_bass(fig, _bars((4, 0)), (36, 52), {0: bass.BassTone(0, -1)})
    nxt = events[len(fig.steps)]
    assert nxt.midi == 36 and events[len(fig.steps) - 1].midi == 37


def test_no_approach_into_a_bar_without_bass():
    fig = kn.BASS_FIGURES["offbeat"]
    bars = [(0, harmony.chord_pcs(STYLE, Key(0, "major"), 0)), (2, harmony.chord_pcs(STYLE, Key(0, "major"), 3))]
    assert bass.figure_bass(fig, bars, REGISTER, {0: bass.BassTone(0, -1)}) == bass.figure_bass(fig, bars, REGISTER)


# ── развитие rhythm: ритм хука, контур следующей фразы (Н10) ─────────────────────────────────────────────────────

ANSWERED = BARS + (4, 4, 3, 3, 5, 5, 0, 0)


def test_answer_has_the_hook_rhythm_and_the_next_phrase_contour():
    hook, key = hooks.from_material(synthetic(ANSWERED), 132, 0, "major", cp.hook_register(STYLE))
    assert hook.answer
    assert [(e.beat, e.dur_beats) for e in hook.answer] == [(e.beat, e.dur_beats) for e in hook.notes]
    assert [e.midi for e in hook.answer] != [e.midi for e in hook.notes]
    lo, hi = cp.hook_register(STYLE)
    assert all(lo <= e.midi <= hi for e in hook.answer)
    assert min(e.midi for e in hook.answer) >= min(e.midi for e in hook.notes)  # пэду под лидом место не меньше


def test_drop2_plays_the_answer_and_hook_without_answer_keeps_thirds():
    hook, key = hooks.from_material(synthetic(ANSWERED), 132, 0, "major", cp.hook_register(STYLE))
    assert hooks.develop(hook, "drop2", 8, key) == tuple(sorted(hook.answer, key=lambda e: (e.beat, e.midi)))
    plain = replace(hook, answer=())
    assert hooks.develop(plain, "drop2", 8, key) != hooks.develop(plain, "drop", 8, key)  # терции, как до PR-4
    assert hooks.develop(hook, "drop", 8, key) == hooks.develop(plain, "drop", 8, key)


@pytest.mark.parametrize("degrees", [BARS, BARS + BARS])
def test_no_answer_without_a_new_next_phrase(degrees):
    """Нет следующей фразы (8 тактов) или она — буквальный повтор: ответа нет, drop2 — свой."""
    hook, _key = hooks.from_material(synthetic(degrees), 132, 0, "major", cp.hook_register(STYLE))
    assert hook.answer == ()


# ── compose: бас материала в треке ──────────────────────────────────────────────────────────────────────────────

def _loop_bars(track) -> set:
    """Такты формы, где звучит петля хука (#3529): секции без лида и секция темы после темы целиком (хук по кругу);
    у секций развития (build — педаль, break, ответ drop2) гармония своя, под их мелодию."""
    out, start = set(), 0
    span = track.hook.theme_bars if track.hook else 0
    for sec in track.form.sections:
        if "lead" not in sec.roles:
            out |= set(range(start, start + sec.bars))
        elif sec.name == kn.THEME_SECTION:
            out |= set(range(start + span, start + sec.bars))
        start += sec.bars
    return out


def _bass_by_loop_bar(track):
    out = {}
    loop = _loop_bars(track)
    for e in track.parts["bass"].pitches:
        if int(e.beat // BEATS_PER_BAR) in loop:
            out.setdefault(int(e.beat // BEATS_PER_BAR) % 8, []).append(e)
    return out


def test_compose_puts_the_author_inversion_and_approach_into_the_bass():
    notes = [(ROOTS[d], i * 4.0, 4.0) for i, d in enumerate(BARS)]
    notes[2] = (48, 8.0, 4.0)  # vi на до — терция
    notes[3:4] = [(45, 12.0, 3.0), (42, 15.0, 1.0)]  # подход фа-диез → фа (сверху; квинта ля минора — ми, не он)
    m = with_bass(synthetic(ANSWERED), notes)
    track = cp.compose(_plan(m.material_id), 1, materials={m.material_id: m})
    validate(track)
    assert track.history_key.progression == "-".join(map(str, BARS))  # аккорд на такт (#3529)
    by_bar = _bass_by_loop_bar(track)
    third = harmony.chord_pcs(STYLE, track.key, 5)[1]
    assert by_bar[2][0].midi % 12 == third
    lo, hi = STYLE.registers["bass"]
    assert all(lo <= e.midi <= hi for e in track.parts["bass"].pitches)
    assert all(round(e.beat * 4) % 4 != 0 for e in track.parts["bass"].pitches)  # мимо долей прямой бочки
    bars = {}
    for e in track.parts["bass"].pitches:
        bars.setdefault(int(e.beat // BEATS_PER_BAR), []).append(e)
    loop = _loop_bars(track)  # петля хука по такту формы; тема целиком и секции развития — своя гармония
    joints = [b for b in bars if b % 8 == 3 and b + 1 in bars and {b, b + 1} <= loop]
    assert joints and all(bars[b][-1].midi == bars[b + 1][0].midi + 1 for b in joints)  # подход сверху, как у автора
    assert track.hook.answer  # drop2 — ритм хука с контуром следующей фразы


def test_compose_with_root_bass_has_the_same_bass_as_without_bass():
    """Опора на материал не выдумывает: бас автора на приме без подходов — тот же бас, что без баса в материале."""
    m = synthetic(BARS)
    with_root = root_bass(m, BARS)
    a = cp.compose(_plan(m.material_id), 1, materials={m.material_id: m})
    b = cp.compose(_plan(m.material_id), 1, materials={m.material_id: with_root})
    assert a.parts["bass"] == b.parts["bass"]


def test_compose_without_material_ignores_bass_tones():
    base = cp.compose(_plan(None), 1)
    m = with_bass(synthetic(ANSWERED), [(48, i * 4.0, 4.0) for i in range(16)])
    assert cp.compose(_plan(None), 1, materials={m.material_id: m}) == base
