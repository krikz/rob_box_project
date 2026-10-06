"""ADR-0152 PR-5: рисунки пэда (``pumped16``/``held``/``stabs``), ≥ 2 пэда на семью, A9-модель на трек.

Поведение — на модели ``Track`` и событиях ``render.events`` (симулятор Renardo), а не на тексте программы.
"""

from __future__ import annotations

import itertools
from collections import defaultdict
from dataclasses import replace

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.arrange import mix
from rob_box_music.arrange.compose import BASS_GENERATORS, PAD_GENERATORS, _bar_chords, compose, form_spec
from rob_box_music.diversity import MusicHistory, track_composition, track_history
from rob_box_music.model import blend_bars
from rob_box_music.render.events import program_events
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import seeded_profile

CLUB = kn.STYLES["club"]
#: Строки таблицы тем (четыре семьи тембров) и тема не из таблицы (семья по умолчанию).
THEMES = ("космос", "киберпанк", "детский праздник", "калинка", "бухгалтерский отчёт")
SEEDS = range(30)
TRACKS = 3


@pytest.fixture(scope="module")
def sets():
    """{тема: [треки 30 сетов по 3 трека, с историей сета]}."""
    out = {}
    for theme in THEMES:
        profile = seeded_profile(theme)
        tracks = []
        for seed in SEEDS:
            plan = seeded_plan(profile, seed, set_id=f"p{seed}")
            history: list = []
            for no in range(1, TRACKS + 1):
                track = compose(plan, no, history=history)
                history.insert(0, track_history(track, plan.set_id))
                tracks.append(track)
        out[theme] = tracks
    return out


def _figure(track) -> str:
    return track.history_key.pad_figure


def test_every_theme_gets_three_pads_and_two_figures(sets):
    """A16a (ADR-0152 §6.2) внутри темы: 30 сидов дают ≥ 3 разных пэда и ≥ 2 рисунка; по всем темам — все рисунки."""
    figures_all, pads_all = set(), set()
    for theme, tracks in sets.items():
        pads = {t.parts["pad"].synth_or_sample for t in tracks}
        figures = {_figure(t) for t in tracks}
        assert len(pads) >= 3 and len(figures) >= 2, (theme, pads, figures)
        pads_all |= pads
        figures_all |= figures
    assert figures_all == set(CLUB.pad_figures) and len(pads_all) >= 5, (figures_all, pads_all)


def test_figure_does_not_freeze_within_a_set(sets):
    """Подряд один рисунок в трёх треках сета — не чаще, чем в трети сетов (штраф за недавнее, ``pad_figure``)."""
    for theme, tracks in sets.items():
        frozen = sum(len({_figure(t) for t in tracks[i:i + TRACKS]}) == 1 for i in range(0, len(tracks), TRACKS))
        assert frozen <= len(SEEDS) // 3, (theme, frozen)


def test_families_have_two_pads_and_every_figure_a_synth():
    """≥ 2 пэда на семью; у каждого рисунка есть синт семьи; синт с хвостом — только в ``held``."""
    assert all(len(family["pad"]) >= 2 for family in CLUB.timbres.values())
    for row, figure in itertools.product((*kn.THEME_TIMBRE, None), CLUB.pad_figures):
        synths = mix.pad_synths(CLUB, row, figure)
        assert synths, (row, figure)
        if not kn.PAD_FIGURES[figure].long_tails:
            assert all(mix.sustains_to_sus(s) for s in synths), (row, figure, synths)
    assert not all(mix.sustains_to_sus(s) for s in mix.pad_synths(CLUB, "slavic", "held")), "warmpad — в held"
    for synth in kn.SYNTH_PALETTE["pad"]:
        assert synth in kn.LANE_DB_AT_UNIT["pad"] and synth in kn.LAYER_BANDS["pad"], synth


def _first(sets, figure):
    return next(t for tracks in sets.values() for t in tracks if _figure(t) == figure)


def _pad_events(track):
    program = render(track, "A")
    _p, events = program_events(program.code, program.form_beats)
    return [e for e in events if e.slot == program.slots["pad"]]


def test_held_pad_holds_the_chord_without_the_pump(sets):
    """``held``: пэд не под сайдчейном (нет ``amplify``), аккорд звучит до смены — ``sus`` 2 такта, атака на такте."""
    track = _first(sets, "held")
    assert "pad" not in track.mix.duck_roles and {"bass", "sample"} <= set(track.mix.duck_roles)
    events = _pad_events(track)
    assert events and all(e.beat % 8 == 0 and e.sus_beats == 8 for e in events if e.pan < 0)
    assert len({round(e.amp / e.gate, 6) for e in events}) == 1, "громкость нот held не качается"


def test_stabs_are_short_chords_on_the_offbeats(sets):
    """``stabs``: аккорд на «и» каждой доли под сайдчейном до следующего «и»; подхват — только на первой доле формы
    (пэд звучит всю форму), последний стэб кончается с формой."""
    track = _first(sets, "stabs")
    assert "pad" in track.mix.duck_roles
    left = sorted((e for e in _pad_events(track) if e.pan < 0), key=lambda e: e.beat)
    offbeats = [e for e in left if e.beat % 1 == 0.5]
    assert offbeats and all(e.sus_beats == 1.0 for e in offbeats[:-3])
    assert {e.beat for e in left if e.beat % 1 != 0.5} == {0.0} and all(e.sus_beats == 0.5 for e in left[:3])
    assert max(e.beat + e.sus_beats for e in left) == track.form.bars_total * 4


def test_pumped16_stays_a_chord_on_every_sixteenth(sets):
    track = _first(sets, "pumped16")
    assert "pad" in track.mix.duck_roles
    beats = sorted({e.beat for e in _pad_events(track) if e.pan < 0})
    assert all(b2 - b1 == 0.25 for b1, b2 in zip(beats[:16], beats[1:16]))


def test_figure_level_is_the_role_target_plus_its_offset(sets):
    """Цель уровня рисунка — цель роли + ``level_offset_db`` (громкость как у ``pumped16``), минус поправка A9."""
    for figure in CLUB.pad_figures:
        track = _first(sets, figure)
        want = CLUB.role_level_db["pad"] + kn.PAD_FIGURES[figure].level_offset_db + track.mix.a9_trim.get("pad", 0.0)
        assert track.mix.level_db["pad"] == pytest.approx(min(want, mix._cap("pad", track.parts["pad"])), abs=0.011)


def _combinations():
    for family, figure in itertools.product(CLUB.timbres, CLUB.pad_figures):
        pads = [s for s in CLUB.timbres[family]["pad"] if kn.PAD_FIGURES[figure].long_tails or mix.sustains_to_sus(s)]
        for pad, bass, bass_figure in itertools.product(pads, CLUB.timbres[family]["bass"], CLUB.bass_figures):
            yield family, figure, pad, bass, bass_figure


@pytest.fixture(scope="module")
def base_tracks():
    """Треки-основа: темы всех семей × 4 сида × номера 1 (форма открытия) и 2 (обычная форма)."""
    out = []
    for theme in THEMES[:4]:
        for seed, no in itertools.product(range(4), (1, 2)):
            out.append((no, compose(seeded_plan(seeded_profile(theme), seed), no)))
    return out


def _remix(track, no, figure, pad, bass, bass_figure=None, lead=None):
    spec = form_spec(CLUB, track.history_key.template)
    chords = track.harmony.progression[track.form.sections[0].name]
    parts = dict(track.parts)
    parts["pad"] = PAD_GENERATORS[figure](CLUB, track.key, _bar_chords(spec, "pad", chords), pad,
                                          track.parts["pad"].register)
    parts["bass"] = (BASS_GENERATORS[bass_figure](CLUB, track.key, _bar_chords(spec, "bass", chords), bass,
                                                  CLUB.registers["bass"])
                     if bass_figure else replace(track.parts["bass"], synth_or_sample=bass))
    if lead:
        parts["lead"] = replace(track.parts["lead"], synth_or_sample=lead)
    return mix.mix_parts(CLUB, parts, track.form, figure)


@pytest.mark.parametrize("family,figure,pad,bass,bass_figure", list(_combinations()))
def test_a9_model_holds_for_every_family_figure_and_synth(base_tracks, family, figure, pad, bass, bass_figure):
    """A9-модель на трек (ADR-0152 §4 п.2): низ худшего дропа ≥ порога стиля на каждой комбинации семья × рисунок
    пэда × пэд × бас × рисунок баса × лид семьи (PR-6: басы семей держат низ — ``retrobass`` 0.72 из ``hard`` убран)."""
    for (no, track), lead in itertools.product(base_tracks, CLUB.timbres[family]["lead"]):
        _leveled, track_mix = _remix(track, no, figure, pad, bass, bass_figure, lead)
        assert track_mix.a9_trim.get("pad", 0.0) >= kn.A9_PAD_FLOOR_DB
        assert 0.0 <= track_mix.a9_trim.get("bass", 0.0) <= kn.A9_BASS_BOOST_DB
        assert track_mix.a9_model >= CLUB.a9_model_low, (family, figure, pad, bass, bass_figure, lead,
                                                         track_mix.a9_model, dict(track_mix.a9_trim))


def test_a9_trim_lowers_the_pad_first_and_records_it(base_tracks):
    """Пэд громче цели стиля на 14 дБ (``warmpad`` достаёт) опускается ступенями до порога, поправка — в
    ``Mix.a9_trim``; бас не трогается, пока хватает пэда."""
    _no, track = base_tracks[0]
    loud = {**track.parts, "pad": replace(track.parts["pad"], synth_or_sample="warmpad")}
    hot = replace(CLUB, role_level_db={**CLUB.role_level_db, "pad": CLUB.role_level_db["pad"] + 14.0})
    leveled, track_mix = mix.mix_parts(hot, loud, track.form, "pumped16")
    assert track_mix.a9_trim["pad"] < 0 and track_mix.a9_trim["pad"] % kn.A9_STEP_DB == 0
    assert "bass" not in track_mix.a9_trim and track_mix.a9_model >= CLUB.a9_model_low
    assert leveled["pad"].level_db == track_mix.level_db["pad"] == hot.role_level_db["pad"] + track_mix.a9_trim["pad"]


def test_blend_and_render_survive_every_figure_pair(sets):
    """Рисунок пэда не трогает состав ролей секций: блэнд соседних треков сета — полные 8 тактов на всех парах."""
    pairs = defaultdict(int)
    for tracks in sets.values():
        for i in range(0, len(tracks), TRACKS):
            for a, b in zip(tracks[i:i + TRACKS], tracks[i + 1:i + TRACKS]):
                assert blend_bars(a, b) == CLUB.blend[0], (_figure(a), _figure(b))
                pairs[(_figure(a), _figure(b))] += 1
    assert len(pairs) >= 6, dict(pairs)


def test_pad_figure_goes_to_history_and_composition(sets):
    track = _first(sets, "held")
    row = track_history(track, "s1")
    assert row["pad_figure"] == "held" and row["pad"] == track.parts["pad"].synth_or_sample
    store = MusicHistory(":memory:")
    assert store.record(**row) and store.recent(1)[0]["pad_figure"] == "held"
    comp = track_composition(track)
    assert comp["pad_figure"] == "held" and comp["a9_model"] == track.mix.a9_model
    assert comp["a9_trim"] == dict(track.mix.a9_trim)
