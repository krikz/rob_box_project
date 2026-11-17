"""ADR-0152 PR-9: ``acid16`` — кислотная 16-я линия ``tb303`` со срезом фильтра на каждую ноту, только семья ``hard``.

Поведение — на модели ``Track`` и событиях ``render.events`` (симулятор Renardo), а не на тексте программы.
A9-модель по всем комбинациям ``hard`` × пэд × рисунок пэда × лид — ``test_pad_figures.test_a9_model_holds_for_every_
family_figure_and_synth`` (пары рисунок ↔ синт фильтрует ``mix.bass_pair_ok``); здесь — что ``tb303``/``acid16``
в этот перебор входят.
"""

from __future__ import annotations

import dataclasses

import pytest

from parallel import pmap
from rob_box_music import knowledge as kn
from rob_box_music.arrange import mix
from rob_box_music.arrange.compose import BASS_GENERATORS, compose
from rob_box_music.model import BEATS_PER_BAR, STEPS_PER_BAR, TrackError, validate
from rob_box_music.render.events import program_events
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import seeded_profile

CLUB = kn.STYLES["club"]
FIGURE = kn.BASS_FIGURES["acid16"]
THEMES = ("космос", "киберпанк", "детский праздник", "калинка", "бухгалтерский отчёт", "")
STEP_BEATS = BEATS_PER_BAR / STEPS_PER_BAR


def _row(theme):
    return seeded_profile(theme).row


@pytest.fixture(scope="module")
def acid_tracks():
    """Треки темы ``киберпанк`` (семья ``hard``) с ``acid16``: 40 сидов × 3 трека."""
    out = []
    for seed in range(40):
        plan = seeded_plan(seeded_profile("киберпанк"), seed, set_id=f"a{seed}")
        out += [t for t in (compose(plan, no) for no in (1, 2, 3)) if t.history_key.bass_figure == "acid16"]
    return out


def test_acid16_and_tb303_exist_only_in_the_hard_family():
    """Данными, без ветвления: ``acid16`` в пуле рисунков только там, где в семье темы есть ``tb303``."""
    families = {name: fam["bass"] for name, fam in CLUB.timbres.items()}
    assert [n for n, bass in families.items() if "tb303" in bass] == ["hard"]
    for theme in THEMES:
        family = CLUB.timbres[kn.THEME_TIMBRE.get(_row(theme) or "", CLUB.default_timbre)]["bass"]
        assert ("acid16" in mix.bass_figures(CLUB, kn.family_of(CLUB, _row(theme)))) == ("tb303" in family), theme
        assert mix.bass_synths(CLUB, kn.family_of(CLUB, _row(theme)), "acid16") == (("tb303",) if "tb303" in family else ())
        assert all("tb303" not in mix.bass_synths(CLUB, kn.family_of(CLUB, _row(theme)), f) for f in CLUB.bass_figures if f != "acid16")


def test_acid16_is_in_the_registry_and_in_the_club_window_only():
    assert "acid16" in BASS_GENERATORS and "acid16" in kn.BASS_FIGURES
    assert [w for w, spec in CLUB.genre_windows.items() if "acid16" in spec.bass_figures] == ["club"]


def _other_family_set(case):
    theme, seed = case
    plan = seeded_plan(seeded_profile(theme), seed, set_id=f"o{seed}")
    for no in (1, 2, 3):
        track = compose(plan, no)
        tb303 = track.parts["bass"].synth_or_sample == "tb303"
        assert tb303 == (track.history_key.bass_figure == "acid16")
        assert plan.family == "hard" or not tb303, (theme, seed, plan.family)


def test_tracks_of_other_families_never_play_tb303_or_acid():
    """Темы вне таблицы получают семью по сиду (#3460): ``hard`` среди них законна — проверяется семья ПЛАНА.
    Сеты независимы и считаются параллельно (#3539)."""
    themes = ("космос", "детский праздник", "калинка", "бухгалтерский отчёт", "")
    pmap(_other_family_set, [(theme, seed) for theme in themes for seed in range(15)], chunksize=5)


def test_hard_tracks_pair_tb303_with_acid16_and_nothing_else(acid_tracks):
    assert len(acid_tracks) >= 5
    for track in acid_tracks:
        assert track.parts["bass"].synth_or_sample == "tb303"
    pmap(_hard_set_pairs_tb303_with_acid16, range(40), chunksize=5)


def _hard_set_pairs_tb303_with_acid16(seed):
    plan = seeded_plan(seeded_profile("киберпанк"), seed, set_id=f"a{seed}")
    for no in (1, 2, 3):
        track = compose(plan, no)
        assert (track.parts["bass"].synth_or_sample == "tb303") == (track.history_key.bass_figure == "acid16")


def test_acid_notes_are_sixteenths_off_the_beat_with_a_cutoff_each(acid_tracks):
    lo, hi = CLUB.registers["bass"]
    for track in acid_tracks:
        events = track.parts["bass"].pitches
        assert events and all(lo <= e.midi <= hi for e in events)
        assert all(e.lpf and kn.LPF_RANGE_HZ[0] <= e.lpf <= kn.LPF_RANGE_HZ[1] for e in events)
        assert all(round(e.beat / STEP_BEATS) % STEPS_PER_BAR in FIGURE.steps for e in events)
        assert all(e.dur_beats == STEP_BEATS or e.beat % 1 == 0.25 for e in events)
        assert {e.accent for e in events} == {2, 3} and len({e.lpf for e in events}) >= 12, "срез ходит волной"
        assert len({e.midi for e in events}) > 1, "есть октавные прыжки"


def test_cutoff_wave_opens_and_closes_over_two_bars():
    wave = FIGURE.lpf
    assert len(wave) == 2 * len(FIGURE.steps) == 24
    peak = wave.index(max(wave))
    assert 0 < peak < len(wave) - 1 and wave[0] < max(wave) and wave[-1] < max(wave)
    beat_starts = wave[0::3]  # первая 16-я доли (с акцентом): срез растёт до пика и спадает после
    top = beat_starts.index(max(beat_starts))
    assert all(a < b for a, b in zip(beat_starts[:top], beat_starts[1:top + 1]))
    assert all(a > b for a, b in zip(beat_starts[top:], beat_starts[top + 1:]))


def test_render_sends_a_cutoff_per_note_and_drops_the_section_sweep(acid_tracks):
    """Рендер: ``lpf=[...]`` у баса — по событию (симулятор Renardo видит срез ноты), свипа секции на басе нет."""
    track = acid_tracks[0]
    assert "bass" not in track.mix.lpf and {"pad", "lead"} <= set(track.mix.lpf)
    program = render(track, "A")
    _p, events = program_events(program.code, program.form_beats)
    bass = [e for e in events if e.slot == program.slots["bass"]]
    assert bass
    by_beat = {round(e.beat, 3): e.fx.get("lpf") for e in bass}
    for ev in track.parts["bass"].pitches[:48]:
        assert by_beat[round(ev.beat, 3)] == pytest.approx(ev.lpf), ev
    assert len({e.fx.get("lpf") for e in bass}) >= 12


def test_acid_is_complementary_to_the_kick_of_every_window_that_has_it():
    """Бас не на шаге бочки дропа окна (в ``breaks`` ломаная бочка — acid в пул окна не входит), и не на доле."""
    for name, window in CLUB.genre_windows.items():
        kicks = {s for _t, look in window.looks for s in mix.kick_steps(look.kick)}
        for figure in window.bass_figures:
            assert not kicks & set(kn.BASS_FIGURES[figure].steps), (name, figure)
    assert all(step % 4 for step in FIGURE.steps)


def test_a9_model_covers_tb303_with_acid16_for_every_hard_pad():
    """Перебор A9 (``test_pad_figures``) содержит ``tb303`` × ``acid16`` на каждом пэде и рисунке пэда ``hard``."""
    from test_pad_figures import _combinations  # noqa: PLC0415

    combos = [c for c in _combinations() if c[0] == "hard" and c[3] == "tb303"]
    pads = set(CLUB.timbres["hard"]["pad"])
    assert {c[4] for c in combos} == {"acid16"} and {c[2] for c in combos} <= pads and len(combos) >= 6


def test_validate_rejects_a_note_cutoff_out_of_range(acid_tracks):
    track = acid_tracks[0]
    part = track.parts["bass"]
    bad = dataclasses.replace(part.pitches[0], lpf=9000.0)
    broken = dataclasses.replace(track, parts={**track.parts, "bass": dataclasses.replace(
        part, pitches=(bad,) + part.pitches[1:])})
    with pytest.raises(TrackError, match="lpf"):
        validate(broken)
    validate(track)


def test_acid_tracks_hold_the_a9_model(acid_tracks):
    """Реальные треки с ``acid16``: низ худшего дропа ≥ порога, поправка баса в пределах ``A9_BASS_BOOST_DB``."""
    for track in acid_tracks:
        assert 0.0 <= track.mix.a9_trim.get("bass", 0.0) <= kn.A9_BASS_BOOST_DB
        assert track.mix.a9_model >= CLUB.a9_model_low, (track.track_id, track.mix.a9_model)
