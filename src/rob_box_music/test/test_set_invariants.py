"""ADR-0157 PR-5 (#3562): инварианты сета H1–H7 по модели ``Track`` — гард на сид-серии и поведение блэнда.

Сид-серия: каждый стиль ``knowledge.STYLES`` — 4 сета × 5 треков подряд (20 треков, половина сетов — с RTTTL-хуками
темы, половина — мотив лида). Гарды: H1 пульс дропа (и рантайм в ``model.validate``), H3 конфликт ладов на стыке,
H6 сустейн не у пэда, H7 дакинг у каждого трека с бочкой и HPF входа деки (``Transition.filter_in``) в программе.
H2 (окно без темы, решение В5) и H4 (фразировка, решение В6) — отчётные: считаются, порога нет.
"""

from __future__ import annotations

import dataclasses

import pytest

from melodies import MELODIES
from parallel import pmap
from rob_box_music import knowledge as kn
from rob_box_music.arrange.compose import compose
from rob_box_music.diversity import track_composition
from rob_box_music.model import (
    BEATS_PER_BAR, Grid, Step, Track, TrackError, Transition, pulse_beats, pulse_faults, validate,
)
from rob_box_music.render.events import program_events
from rob_box_music.render.renardo import render
from rob_box_music.set_invariants import (
    ducked, junction_invariants, lead_phrasing, long_sustains, summary, track_invariants,
)
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import seeded_profile

THEMES = ("космос", "дождь", "дорога", "киберпанк")
TRACKS = 5


def _style_sets(style: str):
    out = []
    for k, theme in enumerate(THEMES):
        hooks = k % 2 == 0
        profile = seeded_profile(theme, style)
        profile = dataclasses.replace(profile, hook_ids=tuple(MELODIES)) if hooks else profile
        plan = seeded_plan(profile, k, TRACKS, set_id=f"inv{k}")
        history: list = []
        run = []
        for no in range(1, TRACKS + 1):
            track = compose(plan, no, history=history, melodies=MELODIES if hooks else None, deck="AB"[no % 2])
            history.insert(0, track_composition(track))
            run.append(track)
        out.append(run)
    return style, out


@pytest.fixture(scope="module")
def series():
    return dict(pmap(_style_sets, list(kn.STYLES)))


def test_series_covers_every_style_with_twenty_tracks(series):
    assert set(series) == set(kn.STYLES)
    assert all(sum(len(run) for run in runs) == len(THEMES) * TRACKS for runs in series.values())


def test_h1_every_drop_bar_has_pulse_and_no_long_gaps(series):
    bad = [(s, t.track_id, f) for s, runs in series.items() for run in runs for t in run for f in pulse_faults(t)]
    assert not bad, bad[:5]


def test_h3_no_chromatic_author_chords_against_the_leaving_key(series):
    joints = [(s, junction_invariants(a, b))
              for s, runs in series.items() for run in runs for a, b in zip(run, run[1:])]
    assert all(j.blend_bars == 8 for _s, j in joints), "соседи сета сводятся блэндом 8 тактов"
    assert sum(j.author_chords for _s, j in joints) > 0, "H3 проверил хоть один аккорд окна"
    bad = [(s, j.key_conflicts) for s, j in joints if j.key_conflicts]
    assert not bad, bad[:5]


def test_h6_sustains_longer_than_two_bars_only_on_the_pad(series):
    bad = [(s, t.track_id, long_sustains(t)) for s, runs in series.items() for run in runs for t in run
           if long_sustains(t)]
    assert not bad, bad[:5]


def test_h7_every_track_with_a_kick_ducks_under_it(series):
    for style, runs in series.items():
        assert kn.STYLES[style].duck_roles, style
        for track in (t for run in runs for t in run if "kick" in t.parts):
            assert ducked(track), (style, track.track_id)


def test_h2_and_h4_are_reported_not_gated(series):
    """В5: стык 8 тактов остаётся (окно без темы ≈ 32 доли + хвост фразы лида); В6: фразировку только печатаем."""
    table = summary(series)
    for style, row in table.items():
        assert row["tracks"] == len(THEMES) * TRACKS and row["h1"] == row["h3"] == row["h6"] == 0, (style, row)
        assert 24 <= row["h2_median"] <= 64, (style, row["h2_median"])
        assert 0.0 <= row["h4_rest_median"] < 1.0 and row["h4_run16_max"] >= 0, (style, row)


# ── H1: рантайм-гард и метрика на собранных нарушениях ─────────────────────────────────────────────────────────

@pytest.fixture(scope="module")
def club():
    return compose(seeded_plan(seeded_profile("космос"), 3, set_id="h1"), 2)


def _drop_index(track: Track) -> int:
    return next(i for i, s in enumerate(track.form.sections) if kn.section_kind(s.name).startswith("drop"))


def test_validate_rejects_a_drop_without_kick_snare_or_bass(club):
    i = _drop_index(club)
    sec = club.form.sections[i]
    sections = list(club.form.sections)
    sections[i] = dataclasses.replace(sec, roles=frozenset(sec.roles) - set(kn.PULSE_ROLES))
    broken = dataclasses.replace(club, form=dataclasses.replace(club.form, sections=tuple(sections)))
    with pytest.raises(TrackError, match=r"H1") as err:
        validate(broken)
    assert err.value.path == f"form.sections[{i}].roles" and "дропа без kick/clap/bass" in err.value.reason


def test_pulse_gap_outside_breaks_is_a_fault(club):
    """Бочка раз в 4 такта без малого и баса в build — пауза пульса 15 долей > 8: нарушение секции build."""
    i = next(i for i, s in enumerate(club.form.sections) if s.name == "build")
    sections = list(club.form.sections)
    sections[i] = dataclasses.replace(sections[i], roles=frozenset(sections[i].roles) - {"clap", "bass"})
    sparse = Grid((Step(True),) + (Step(False),) * 63)
    kick = dataclasses.replace(club.parts["kick"], grid=sparse)
    broken = dataclasses.replace(club, form=dataclasses.replace(club.form, sections=tuple(sections)),
                                 parts={**club.parts, "kick": kick})
    faults = dict(pulse_faults(broken))
    assert i in faults and "пауза пульса 15 долей" in faults[i], faults


def test_pulse_counts_bass_sustain_only_where_the_bass_is_ordered(club):
    pulse = pulse_beats(club)
    start = sum(s.bars for s in club.form.sections[:_drop_index(club)]) * BEATS_PER_BAR
    assert start in pulse and 0 not in pulse, "интро — хэты и пэд: пульса нет; дроп начинается с пульса"


def test_track_invariants_of_a_lead_phrase(club):
    inv = track_invariants(club)
    rest, run = lead_phrasing(club)
    assert inv.pulse == () and inv.long_sus == () and (inv.lead_rest, inv.lead_run16) == (rest, run)
    assert rest is not None and 0.0 < rest < 1.0


# ── H7: HPF входа деки (ADR-0149 §3.12) — рендер читает ``Transition.filter_in`` ───────────────────────────────

def _hpf_events(track: Track, deck: str = "B"):
    program = render(track, deck)
    _p, events = program_events(program.code, program.form_beats)
    role_of = {slot: role for role, slot in program.slots.items()}
    return program, [(role_of[e.slot], e.beat, e.fx.get("hpf")) for e in events]


def test_incoming_deck_opens_its_highpass_over_the_bars_before_the_bass_swap(club):
    swap_beats = club.transition_in.bass_swap_bar * BEATS_PER_BAR
    program, events = _hpf_events(club)
    assert "hpf=var(" in program.code
    early = [(r, b, hz) for r, b, hz in events if b < swap_beats and r in ("pad", "hats")]
    assert early and all(hz is not None for _r, _b, hz in early), "до свопа пэд и хэты под HPF"
    first = min(early, key=lambda e: e[1])
    assert first[2] == kn.HPF_IN_HZ[0] and first[1] < 1.0
    later = sorted(hz for r, b, hz in early if b >= swap_beats - 1)
    assert all(hz <= kn.HPF_IN_HZ[0] / 4 for hz in later), "к свопу срез опустился"
    assert all(hz is None for r, b, hz in events if b >= swap_beats), "после свопа фильтр снят"
    assert all(hz is None for r, _b, hz in events if r in ("kick", "bass")), "бочка и бас входят без фильтра"


def test_no_highpass_without_filter_in(club):
    plain = dataclasses.replace(club, transition_in=Transition(club.transition_in.phrase_bars,
                                                               club.transition_in.bass_swap_bar, False))
    assert "hpf=" not in render(plain, "A").code
