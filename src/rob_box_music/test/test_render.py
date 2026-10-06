"""PR-2/PR-3a ADR-0149: ``render(track, deck)`` и club-трек ``compose`` — свойства на событиях нот, без снапшотов.

Чётные сиды — трек с хуком из тестовых мелодий, нечётные — без мелодий (мотив лида PR-2).
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import replace

import pytest

from melodies import MELODIES, compose_p, profile
from rob_box_music import knowledge as kn
from rob_box_music.arrange.compose import club_track
from rob_box_music.model import BEATS_PER_BAR
from rob_box_music.render.events import program_events
from rob_box_music.render.renardo import FORM_START, ROLE_SLOT, RenderError, render

SEEDS = range(40)


def track_for(seed, deck="A", hooked=None):
    hooked = seed % 2 == 0 if hooked is None else hooked
    return compose_p(profile(root=seed % 12, mode=("minor", "major", "dorian")[seed % 3]), 1, set_seed=seed,
                     melodies=MELODIES if hooked else None, deck=deck)


def _events(track, deck="A"):
    program = render(track, deck)
    _parsed, events = program_events(program.code, program.form_beats)
    slot_role = {slot: role for role, slot in program.slots.items()}
    by_role = defaultdict(list)
    for ev in events:
        by_role[slot_role[ev.slot]].append(ev)
    return program, by_role


def _section_of(track, beat):
    start = 0.0
    for sec in track.form.sections:
        end = start + sec.bars * BEATS_PER_BAR
        if start <= beat < end:
            return sec
        start = end
    raise AssertionError(beat)


@pytest.mark.parametrize("seed", SEEDS)
def test_events_equal_the_model(seed):
    """Свёрнутая программа звучит ровно как модель: каждая нота, доля и sus (первый голос; второй — PR-9 стерео)."""
    track = track_for(seed)
    _program, by_role = _events(track)
    for role in kn.TONAL_ROLES:
        heard = sorted((e.beat, e.midi, e.sus_beats) for e in by_role[role] if e.detune == 0 and e.pan <= 0)
        model = sorted((p.beat, p.midi, p.dur_beats) for p in track.parts[role].pitches)
        assert heard == model, role


@pytest.mark.parametrize("seed", SEEDS)
def test_key_registers_and_order(seed):
    track = track_for(seed)
    _program, by_role = _events(track)
    pcs = kn.scale_pitch_classes(track.key.root, track.key.mode)
    span = {}
    for role in kn.TONAL_ROLES:
        midis = [e.midi for e in by_role[role]]
        assert {m % 12 for m in midis} <= pcs, role
        lo, hi = kn.REGISTERS[role]
        assert lo <= min(midis) and max(midis) <= hi, role
        span[role] = (min(midis), max(midis))
    assert span["bass"][1] < span["pad"][1] < span["lead"][1] <= kn.LEAD_MAX_MIDI
    assert span["pad"][1] <= span["lead"][0] - 3, "верх пэда ниже лида на ≥ 3 полутона"


@pytest.mark.parametrize("seed", SEEDS)
def test_kick_four_on_floor_and_bass_off_the_kick(seed):
    track = track_for(seed)
    _program, by_role = _events(track)
    kicks = {e.beat for e in by_role["kick"]}
    beats = range(int(track.form.bars_total * BEATS_PER_BAR))

    fill_beats, end = set(), 0  # PR-3b: последняя доля такта fill-а (перед дропом, конец трека) без бочки
    for sec in track.form.sections:
        end += sec.bars * BEATS_PER_BAR
        if sec.fill_last_bar:
            fill_beats.add(end - 1)

    def fill_beat(b):
        return b in fill_beats

    expected = {float(b) for b in beats if "kick" in _section_of(track, b).roles and not fill_beat(b)}
    assert kicks == expected, "бочка на каждой доле, кроме последней доли fill-а"
    bass = [e.beat for e in by_role["bass"]]
    assert bass and not kicks & set(bass), "бас не на шагах бочки"
    steps = kn.BASS_FIGURES[track.history_key.bass_figure].steps  # рисунок баса трека (ADR-0152 PR-6)
    assert all(round(b % BEATS_PER_BAR * 4) in steps for b in bass), "бас на шагах своего рисунка (не на доле)"
    per_bar = defaultdict(int)
    for b in bass:
        per_bar[int(b // BEATS_PER_BAR)] += 1
    assert set(per_bar.values()) <= {len(steps) - 1, len(steps)}


@pytest.mark.parametrize("seed", SEEDS)
def test_lead_is_a_motif_with_rests(seed):
    """Без мелодии лид — мотив PR-2: ≤ 4 атак в такте, пауза в половине тактов, пул ≤ 4 нот (без терций drop2)."""
    track = track_for(seed, hooked=False)
    _program, by_role = _events(track)
    per_bar = defaultdict(int)
    for beat in {ev.beat for ev in by_role["lead"]}:  # атаки, а не голоса (drop2 — терции)
        assert "lead" in _section_of(track, beat).roles
        per_bar[int(beat // BEATS_PER_BAR)] += 1
    assert per_bar and max(per_bar.values()) <= 4, "лид ≤ 4 нот на такт"
    lead_bars = [b for b in range(track.form.bars_total) if "lead" in _section_of(track, b * 4).roles]
    assert sum(1 for b in lead_bars if per_bar[b] == 0) >= len(lead_bars) // 2, "второй такт фразы — пауза"
    drop = [e.midi for e in by_role["lead"] if _section_of(track, e.beat).name != "drop2"]
    assert len(set(drop)) <= 4, "пул ≤ 4 нот"


@pytest.mark.parametrize("seed", SEEDS)
def test_pad_voice_leading_moves_little(seed):
    """Ни один голос пэда не прыгает дальше большой терции, включая стык петли (старый вид — до 11)."""
    track = track_for(seed)
    chords = track.harmony.progression["drop"]
    for prev, nxt in zip(chords, chords[1:] + chords[:1]):
        moves = [abs(a - b) for a, b in zip(prev.voicing, nxt.voicing)]
        assert max(moves) <= 4, (prev, nxt)


def test_pad_voice_leading_on_average_is_a_step():
    ring = []
    for seed in range(200):
        chords = track_for(seed).harmony.progression["drop"]
        ring += [sum(abs(a - b) for a, b in zip(p.voicing, n.voicing)) for p, n in zip(chords, chords[1:] + chords[:1])]
    assert sum(ring) / len(ring) <= 4.5


@pytest.mark.parametrize("seed", SEEDS)
def test_tempo_window_and_sections_gate(seed):
    track = track_for(seed)
    lo, hi = kn.STYLES["club"].bpm
    assert lo <= track.bpm <= hi
    _program, by_role = _events(track)
    for role, events in by_role.items():
        assert all(role in _section_of(track, e.beat).roles for e in events), role


def test_render_is_deterministic_and_deck_only_changes_slots():
    track = track_for(7)
    a, b = render(track, "A"), render(track, "B")
    assert a == render(track, "A")
    for deck, prog in (("A", a), ("B", b)):  # трек 1 сета — энергия 2, без клэпа (PR-3b)
        assert set(prog.slots.values()) == {kn.DECK_SLOTS[deck][ROLE_SLOT[r]] for r in track.parts}
    swap = dict(zip(kn.DECK_SLOTS["A"], kn.DECK_SLOTS["B"]))
    lines_a = a.code.splitlines()[1:]
    # строка ``c1.buf = [...]`` (psr-пул, PR-3d) — тоже атрибут плеера деки
    assert [swap[ln[:2]] + ln[2:] for ln in lines_a] == b.code.splitlines()[1:]
    # темп и клок — у владельца плеера: программа только читает долю старта плееров (``var`` секций, PR-8)
    assert a.code.count("Clock") == a.code.count(f"start={FORM_START}") > 0 and FORM_START == "Clock.next_bar()"
    drums = {kn.DRUM_SYMBOLS[r] + (f":{p.sample}" if p.sample else "")  # бочка — с номером файла (X:12)
             for r, p in track.parts.items() if r in kn.DRUM_SYMBOLS}
    assert a.synths == {track.parts[r].synth_or_sample for r in kn.TONAL_ROLES} and a.samples == drums
    assert a.form_beats == track.form.bars_total * BEATS_PER_BAR


def test_render_refuses_what_it_cannot_express():
    track = track_for(3)
    with pytest.raises(RenderError, match="деки"):
        render(track, "C")
    off_grid = replace(track.parts["lead"], pitches=tuple(
        replace(p, beat=p.beat + 0.1) if i == 0 else p for i, p in enumerate(track.parts["lead"].pitches)))
    with pytest.raises(RenderError, match="вне сетки"):
        render(replace(track, parts={**track.parts, "lead": off_grid}), "A")
    assert set(ROLE_SLOT) >= set(track.parts)


def test_club_track_wrapper_is_compose_without_theme():
    """``club_track(seed)`` (проверки плеера PR-4) — тот же ``compose`` без темы: мотив лида, окно club."""
    track = club_track(11, deck="B")
    assert track.hook is None and track.track_id.split(":")[2] == "B"
    assert track == club_track(11, deck="B") and render(track, "B").code
