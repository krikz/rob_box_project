"""PR-2 ADR-0149: ``render(track, deck)`` и первый club-трек — свойства на событиях нот, без снапшотов."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import replace

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.arrange.compose import club_track
from rob_box_music.model import BEATS_PER_BAR, Grid
from rob_box_music.render.events import program_events
from rob_box_music.render.renardo import ROLE_SLOT, RenderError, render

SEEDS = range(40)


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
    """Свёрнутая программа звучит ровно как модель: каждая нота, доля и sus."""
    track = club_track(seed)
    _program, by_role = _events(track)
    for role in kn.TONAL_ROLES:
        heard = sorted((e.beat, e.midi, e.sus_beats) for e in by_role[role])
        model = sorted((p.beat, p.midi, p.dur_beats) for p in track.parts[role].pitches)
        assert heard == model, role


@pytest.mark.parametrize("seed", SEEDS)
def test_key_registers_and_order(seed):
    track = club_track(seed)
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
    track = club_track(seed)
    _program, by_role = _events(track)
    kicks = {e.beat for e in by_role["kick"]}
    assert kicks == {float(b) for b in range(int(track.form.bars_total * BEATS_PER_BAR))}
    bass = [e.beat for e in by_role["bass"]]
    assert bass and not kicks & set(bass), "бас не на шагах бочки"
    assert all(b % 1 == 0.5 for b in bass), "бас на «и» доли"
    per_bar = defaultdict(int)
    for b in bass:
        per_bar[int(b // BEATS_PER_BAR)] += 1
    assert set(per_bar.values()) <= {3, 4}


@pytest.mark.parametrize("seed", SEEDS)
def test_lead_is_a_motif_with_rests(seed):
    track = club_track(seed)
    _program, by_role = _events(track)
    per_bar = defaultdict(int)
    for ev in by_role["lead"]:
        assert "lead" in _section_of(track, ev.beat).roles
        per_bar[int(ev.beat // BEATS_PER_BAR)] += 1
    assert per_bar and max(per_bar.values()) <= 4, "лид ≤ 4 нот на такт"
    lead_bars = [b for b in range(track.form.bars_total) if "lead" in _section_of(track, b * 4).roles]
    assert sum(1 for b in lead_bars if per_bar[b] == 0) >= len(lead_bars) // 2, "второй такт фразы — пауза"
    assert len({e.midi for e in by_role["lead"]}) <= 4, "пул ≤ 4 нот"


@pytest.mark.parametrize("seed", SEEDS)
def test_pad_voice_leading_moves_little(seed):
    """Ни один голос пэда не прыгает дальше большой терции, включая стык петли (старый вид — до 11)."""
    track = club_track(seed)
    chords = track.harmony.progression["drop"]
    for prev, nxt in zip(chords, chords[1:] + chords[:1]):
        moves = [abs(a - b) for a, b in zip(prev.voicing, nxt.voicing)]
        assert max(moves) <= 4, (prev, nxt)


def test_pad_voice_leading_on_average_is_a_step():
    ring = []
    for seed in range(200):
        chords = club_track(seed).harmony.progression["drop"]
        ring += [sum(abs(a - b) for a, b in zip(p.voicing, n.voicing)) for p, n in zip(chords, chords[1:] + chords[:1])]
    assert sum(ring) / len(ring) <= 4.5


@pytest.mark.parametrize("seed", SEEDS)
def test_tempo_window_and_sections_gate(seed):
    track = club_track(seed)
    lo, hi = kn.GENRE_WINDOWS["club"].bpm
    assert lo <= track.bpm <= hi
    _program, by_role = _events(track)
    for role, events in by_role.items():
        assert all(role in _section_of(track, e.beat).roles for e in events), role


def test_render_is_deterministic_and_deck_only_changes_slots():
    track = club_track(7)
    a, b = render(track, "A"), render(track, "B")
    assert a == render(track, "A")
    assert set(a.slots.values()) == set(kn.DECK_SLOTS["A"]) and set(b.slots.values()) == set(kn.DECK_SLOTS["B"])
    swap = dict(zip(kn.DECK_SLOTS["A"], kn.DECK_SLOTS["B"]))
    lines_a = a.code.splitlines()[1:]
    assert [swap[ln.split()[0]] + ln[2:] for ln in lines_a] == b.code.splitlines()[1:]
    assert "Clock" not in a.code, "темп и клок — у владельца плеера, не в программе трека"
    assert a.synths == {"bass", "sinepad", "pluck"} and a.samples == {"X", "-", "*"}
    assert a.form_beats == track.form.bars_total * BEATS_PER_BAR


def test_render_refuses_what_it_cannot_express():
    track = club_track(3)
    hats = track.parts["hats"]
    swung = replace(hats, grid=Grid(tuple(replace(s, offset_ms=8) if s.on else s for s in hats.grid.steps)))
    with pytest.raises(RenderError, match="свинг"):
        render(replace(track, parts={**track.parts, "hats": swung}), "A")
    with pytest.raises(RenderError, match="деки"):
        render(track, "C")
    off_grid = replace(track.parts["lead"], pitches=tuple(
        replace(p, beat=p.beat + 0.1) if i == 0 else p for i, p in enumerate(track.parts["lead"].pitches)))
    with pytest.raises(RenderError, match="вне сетки"):
        render(replace(track, parts={**track.parts, "lead": off_grid}), "A")
    assert set(ROLE_SLOT) >= set(track.parts)
