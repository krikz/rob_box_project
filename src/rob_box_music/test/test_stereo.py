"""PR-9 ADR-0149 §3.9: стерео на событиях нот — низ в центре, пэд двумя голосами, хэты/клэп по ударам.

Замер PR-8 на роботе: LR 1.00, Side/Mid −31 дБ (эталон 0.04 и −0.4 дБ), L−R −1.5…−1.7 дБ от статической панорамы
хэтов +0.25. Свойства — на событиях ``render.events`` (симулятор Renardo), без снапшотов строк.
"""

from __future__ import annotations

from collections import defaultdict

import pytest

from melodies import MELODIES, compose_p, profile
from rob_box_music import knowledge as kn
from rob_box_music.arrange.mix import alternate_pan
from rob_box_music.render.events import expand_voices, program_events
from rob_box_music.render.renardo import render

SEEDS = range(16)


def _events(seed, deck="A"):
    track = compose_p(profile(root=seed % 12, mode=("minor", "major")[seed % 2]), 1 + seed % 4, set_seed=seed,
                      melodies=MELODIES if seed % 2 == 0 else None, deck=deck)
    program = render(track, deck)
    _p, events = program_events(program.code, program.form_beats)
    slot_role = {slot: role for role, slot in program.slots.items()}
    by_role = defaultdict(list)
    for ev in events:
        by_role[slot_role[ev.slot]].append(ev)
    return track, by_role


@pytest.mark.parametrize("seed", SEEDS)
def test_kick_and_bass_are_strictly_centred_single_voices(seed):
    _track, by_role = _events(seed)
    for role in ("kick", "bass"):
        assert by_role[role], role
        assert all(e.pan == 0 and e.detune == 0 and not e.fx for e in by_role[role]), role


@pytest.mark.parametrize("seed", SEEDS)
def test_pad_is_two_detuned_voices_on_opposite_sides_with_haas(seed):
    """Каждая нота аккорда пэда — два голоса: ``-w`` без расстройки и ``+w`` на 1/8 тона выше и позже на Хаас."""
    track, by_role = _events(seed)
    haas = kn.PAD_HAAS_MS * track.bpm / 60000.0
    left = [e for e in by_role["pad"] if e.pan < 0]
    right = [e for e in by_role["pad"] if e.pan > 0]
    assert left and len(left) == len(right) == len(by_role["pad"]) // 2
    assert {e.pan for e in left} == {-kn.PAD_SPREAD} and {e.pan for e in right} == {kn.PAD_SPREAD}
    assert {e.detune for e in left} == {0.0} and {e.detune for e in right} == {kn.PAD_DETUNE}
    pairs = sorted((e.beat, e.midi) for e in left)
    shifted = sorted((round(e.beat - haas, 2), e.midi) for e in right)
    assert [(round(b, 2), m) for b, m in pairs] == shifted, "второй голос — та же нота, позже на Хаас"
    assert 12 <= 60000.0 * round(haas, 3) / track.bpm <= 20


@pytest.mark.parametrize("seed", SEEDS)
def test_lead_stays_on_axis_without_room2(seed):
    """Лид на оси; ``room2`` нет ни у кого — на роботе он глушит весь выход (замер 02.10)."""
    track, by_role = _events(seed)
    assert by_role["lead"] and all(e.pan == 0 and e.detune == 0 for e in by_role["lead"])
    assert "room2" not in render(track, "A").code


def _hit_pans(events):
    return [e.pan for e in sorted(events, key=lambda e: e.beat)]


@pytest.mark.parametrize("seed", SEEDS)
def test_hats_and_clap_alternate_sides_on_every_hit_with_zero_mean(seed):
    """Сторона меняется на каждом ударе, а не по номеру шага; за форму перекос ≤ одного удара (вместо +0.25 на всё)."""
    _track, by_role = _events(seed)
    first = {}
    for role in [r for r in ("hats", "clap", "perc") if by_role[r]]:
        pans = _hit_pans(by_role[role])
        assert pans and {abs(p) for p in pans} == {kn.PAN_HATS}, role
        flips = sum(1 for a, b in zip(pans, pans[1:]) if a != b)
        assert flips >= 0.9 * (len(pans) - 1), (role, flips, len(pans))  # стык свёрнутого периода — не чаще
        assert abs(sum(pans)) <= kn.PAN_HATS * 2 + 1e-9, (role, sum(pans))
        first[role] = pans[0]
    assert "hats" in first
    assert all(first[r] == -first["hats"] for r in first if r != "hats"), "клэп/перкуссия — с другой стороны"


@pytest.mark.parametrize("seed", SEEDS)
def test_energy_weighted_balance_is_centred(seed):
    """Прокси A8 |L−R| ≤ 1 дБ: Σ amp²·pan / Σ amp² по всем событиям ≈ 0 (Pan2 равной мощности)."""
    _track, by_role = _events(seed)
    events = [e for evs in by_role.values() for e in evs]
    power = sum(e.amp ** 2 for e in events)
    assert abs(sum(e.amp ** 2 * e.pan for e in events)) / power < 0.02


def test_alternate_pan_flips_on_hits_only_over_two_passes():
    hits = [True, False, True, True, False]  # три удара — нечётно, поэтому два прохода
    pans = alternate_pan(hits, 0.4, first=1)
    assert len(pans) == 10
    on_hits = [p for p, h in zip(pans, hits * 2) if h]
    assert on_hits == [0.4, -0.4, 0.4, -0.4, 0.4, -0.4] and sum(on_hits) == 0
    assert alternate_pan(hits, 0.4, first=-1)[0] == -0.4


def test_nested_groups_expand_like_renardo():
    """Как ``Player.send_osc_message`` (проба на роботе 02.10): плоские кортежи — «молнией» по модулю,
    вложенный кортеж — два голоса на каждую ноту аккорда."""
    zipped = expand_voices({"degree": (0, 4, 7), "pan": (-0.7, 0.7), "pshift": (0, 0.125), "delay": 0})
    assert [(v["degree"], v["pan"]) for v in zipped] == [(0, -0.7), (4, 0.7), (7, -0.7)]
    nested = expand_voices({"degree": (0, 4, 7), "pan": ((-0.7, 0.7),), "pshift": ((0, 0.125),),
                            "delay": ((0, 0.03),)})
    assert [(v["degree"], v["pan"], v["pshift"], v["delay"]) for v in nested] == [
        (0, -0.7, 0, 0), (0, 0.7, 0.125, 0.03), (4, -0.7, 0, 0), (4, 0.7, 0.125, 0.03),
        (7, -0.7, 0, 0), (7, 0.7, 0.125, 0.03)]
    assert len(expand_voices({"degree": 5, "pan": ((-0.7, 0.7),), "pshift": 0, "delay": 0})) == 2
