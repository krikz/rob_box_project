"""ADR-0152 PR-6: лиды и басы — 4 лида и 2–3 баса на семью, рисунки баса ``offbeat``/``rolling8``, штраф по истории.

Поведение — на модели ``Track`` и событиях ``render.events`` (симулятор Renardo), а не на тексте программы. A9-модель
по всем комбинациям семья × пэд × бас × рисунки × лид — ``test_pad_figures.test_a9_model_holds_for_every_family_...``.
"""

from __future__ import annotations

import sqlite3
from dataclasses import replace

import pytest

from parallel import pmap
from rob_box_music import knowledge as kn
from rob_box_music.arrange import mix
from rob_box_music.arrange.compose import BASS_GENERATORS, compose
from rob_box_music.diversity import MusicHistory, track_composition, track_history
from rob_box_music.model import BEATS_PER_BAR, STEPS_PER_BAR
from rob_box_music.render.events import program_events
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import seeded_profile

CLUB = kn.STYLES["club"]
#: Рисунки баса всех жанровых окон клуба (PR-8): ``broken`` — только окно ``breaks`` (бас мимо ломаной бочки).
WINDOW_BASS = {f for w in CLUB.genre_windows.values() for f in w.bass_figures}
THEMES = ("космос", "киберпанк", "детский праздник", "калинка", "бухгалтерский отчёт")
SEEDS = range(30)
TRACKS = 3
STEP_BEATS = BEATS_PER_BAR / STEPS_PER_BAR


def _set_tracks(case):
    """Треки одного сета (тема, сид) с историей сета; сеты независимы."""
    theme, seed = case
    profile = seeded_profile(theme)
    # семья темы одна на все сиды (темы вне таблицы выбирают её по сиду, #3460 — отдельный тест)
    plan = replace(seeded_plan(profile, seed, set_id=f"lb{seed}"), timbre=kn.family_of(CLUB, profile.row))
    history: list = []
    tracks = []
    for no in range(1, TRACKS + 1):
        track = compose(plan, no, history=history)
        history.insert(0, track_history(track, plan.set_id))
        tracks.append(track)
    return tracks


@pytest.fixture(scope="module")
def sets():
    """{тема: [треки 30 сетов по 3 трека, с историей сета]}; сеты считаются параллельно (#3504, #3539: по сету, а не
    по теме — 5 тем на 4 ядра давали два неполных круга)."""
    cases = [(theme, seed) for theme in THEMES for seed in SEEDS]
    built = pmap(_set_tracks, cases, chunksize=5)
    return {theme: [t for (th, _s), tracks in zip(cases, built) if th == theme for t in tracks] for theme in THEMES}


def _family(theme):
    return CLUB.timbres[kn.THEME_TIMBRE.get(seeded_profile(theme).row or "", CLUB.default_timbre)]


def _synth(track, role):
    return track.parts[role].synth_or_sample


def test_every_theme_gets_four_leads_and_both_bass_figures(sets):
    """A16a (ADR-0152 §6.2): 30 сидов темы дают ≥ 4 лида, все басы семьи (≥ 2) и оба рисунка баса; по темам — ≥ 3
    баса. Синты — только из семьи темы."""
    basses_all = set()
    for theme, tracks in sets.items():
        family = _family(theme)
        leads = {_synth(t, "lead") for t in tracks}
        basses = {_synth(t, "bass") for t in tracks}
        figures = {t.history_key.bass_figure for t in tracks}
        assert len(leads) >= 4 and leads <= set(family["lead"]), (theme, leads)
        assert basses == set(family["bass"]) and len(basses) >= 2, (theme, basses)
        assert figures == set(mix.bass_figures(CLUB, kn.family_of(CLUB, seeded_profile(theme).row))) | {"broken"}, (theme, figures)
        basses_all |= basses
    assert len(basses_all) >= 3, basses_all


def test_families_have_three_to_four_leads_and_two_to_three_basses():
    for name, family in CLUB.timbres.items():
        paired = {s for synths in kn.BASS_FIGURE_SYNTHS.values() for s in synths}  # PR-9: ``tb303`` — в ``hard``
        assert 3 <= len(family["lead"]) <= 4 and 2 <= len(set(family["bass"]) - paired) <= 3, name
        assert all(kn.LAYER_BANDS["lead"][s][0] < 0.05 for s in family["lead"]), name
        assert all(kn.bass_low_on_robot(s) >= kn.BASS_MIN_LOW for s in family["bass"] if s not in paired), name
        # лид достаёт цель роли на потолке ``amp`` (иначе в модели он тише, чем задумано)
        lead = CLUB.role_level_db["lead"]
        assert all(mix.layer_db(kn.LANE_DB_AT_UNIT["lead"][s], kn.AMP_EXPONENT.get(s, 1.0), kn.MAX_LAYER_AMP) >= lead
                   for s in family["lead"]), name


@pytest.mark.parametrize("role", ["lead", "bass"])
def test_synth_does_not_freeze_within_a_set(sets, role):
    """Один синт роли во всех трёх треках сета — не чаще трети сетов (штраф за недавнее, ``music_history.<роль>``)."""
    for theme, tracks in sets.items():
        frozen = sum(len({_synth(t, role) for t in tracks[i:i + TRACKS]}) == 1 for i in range(0, len(tracks), TRACKS))
        assert frozen <= len(SEEDS) // 3, (theme, role, frozen)


@pytest.mark.parametrize("field,role", [("bass_figure", None), ("bass", "bass"), ("lead", "lead")])
def test_history_penalises_the_last_choice(field, role):
    """Значение прошлого трека повторяется редко: вес свежего повтора — ``DEFAULT_FLOOR`` (3 %)."""
    repeats = 0
    for seed in SEEDS:
        plan = seeded_plan(seeded_profile("киберпанк"), seed, set_id="h", genre="club")  # у breaks один рисунок
        first = compose(plan, 2)
        last = first.history_key.bass_figure if field == "bass_figure" else _synth(first, role)
        again = compose(plan, 2, history=[track_history(first, "h")])
        repeats += (again.history_key.bass_figure if field == "bass_figure" else _synth(again, role)) == last
    assert repeats <= 3, (field, repeats)


def _bass_sections(track):
    """(секция, такт начала) где звучит бас."""
    start = 0
    for sec in track.form.sections:
        if "bass" in sec.roles:
            yield sec, start
        start += sec.bars


def test_bass_never_on_a_kick_step_and_ends_by_the_next_beat(sets):
    """Комплементарность (ADR-0149 §3.4): ни одной ноты баса на шаге бочки вида секции, нота кончается к доле."""
    checked = set()
    for tracks in sets.values():
        for track in tracks:
            pitches = track.parts["bass"].pitches
            for sec, start in _bass_sections(track):
                kick = set(mix.kick_steps(mix.look(kn.genre_style(CLUB, track.history_key.genre), sec.energy).kick))
                lo, hi = start * BEATS_PER_BAR, (start + sec.bars) * BEATS_PER_BAR
                for e in (e for e in pitches if lo <= e.beat < hi):
                    step = round(e.beat / STEP_BEATS) % STEPS_PER_BAR
                    assert step not in kick, (track.history_key.bass_figure, sec.name, e)
                    assert e.beat + e.dur_beats <= int(e.beat) + 1 + 1e-9, e
            checked.add(track.history_key.bass_figure)
    assert checked == WINDOW_BASS


def test_bass_stays_in_register(sets):
    lo, hi = CLUB.registers["bass"]
    for tracks in sets.values():
        for track in tracks:
            assert all(lo <= e.midi <= hi for e in track.parts["bass"].pitches)


def _first(sets, figure):
    return next(t for tracks in sets.values() for t in tracks if t.history_key.bass_figure == figure)


def _bass_events(track):
    program = render(track, "A")
    _p, events = program_events(program.code, program.form_beats)
    return [e for e in events if e.slot == program.slots["bass"]]


def test_rolling8_is_eight_tonic_notes_on_and_and_a(sets):
    """``rolling8``: 8 нот на такт — «и» и «а» каждой доли по 16-й, одна высота в такте (тоника, без октавы); в
    рендере (симулятор Renardo) — те же доли."""
    track = _first(sets, "rolling8")
    by_bar = {}
    for e in track.parts["bass"].pitches:
        by_bar.setdefault(int(e.beat // BEATS_PER_BAR), []).append(e)
    for bar, notes in by_bar.items():
        assert sorted(round(e.beat % 1, 3) for e in notes) == sorted([0.5, 0.75] * 4), bar
        assert len({e.midi for e in notes}) == 1 and all(e.dur_beats == STEP_BEATS for e in notes), bar
    assert {round(e.beat % 1, 3) for e in _bass_events(track)} == {0.5, 0.75}


def test_offbeat_is_unchanged_root_root_root_fifth(sets):
    """``offbeat`` — как до PR-6: 4 ноты на «и», полдоли, тоника ×3 и квинта (или тоника, если квинта вне регистра)."""
    track = _first(sets, "offbeat")
    by_bar = {}
    for e in track.parts["bass"].pitches:
        by_bar.setdefault(int(e.beat // BEATS_PER_BAR), []).append(e)
    for notes in by_bar.values():
        assert [e.beat % BEATS_PER_BAR for e in notes] == [0.5, 1.5, 2.5, 3.5]
        assert all(e.dur_beats == 0.5 for e in notes) and len({e.midi for e in notes[:3]}) == 1
    assert {round(e.beat % 1, 3) for e in _bass_events(track)} == {0.5}


def test_bass_figures_are_registered_and_style_driven():
    """Рисунок — ключ реестра стиля (``Style.bass_figures`` ⊆ ``BASS_GENERATORS`` = ``knowledge.BASS_FIGURES`` + линии
    ``knowledge.BASS_LINES``, ADR-0153 S4), шаги рисунков — не на долях (линия walking — на долях, она не рисунок)."""
    assert set(CLUB.bass_figures) <= set(BASS_GENERATORS) == set(kn.BASS_FIGURES) | set(kn.BASS_LINES)
    assert not set(kn.BASS_FIGURES) & set(kn.BASS_LINES)
    assert all(step % 4 for figure in kn.BASS_FIGURES.values() for step in figure.steps)


def test_bass_figure_goes_to_history_and_composition(sets):
    track = _first(sets, "rolling8")
    row = track_history(track, "s1")
    assert row["bass_figure"] == "rolling8" and row["bass"] == _synth(track, "bass") and row["lead"] == _synth(
        track, "lead")
    store = MusicHistory(":memory:")
    assert store.record(**row) and store.recent(1)[0]["bass_figure"] == "rolling8"
    assert track_composition(track)["bass_figure"] == "rolling8"


def test_old_history_db_gets_the_bass_figure_column(tmp_path):
    """БД, созданная до PR-6 (без ``bass_figure``), дописывается миграцией, а не падает на ``record``."""
    path = tmp_path / "voice_memory.db"
    conn = sqlite3.connect(path)
    conn.execute("CREATE TABLE music_history (id INTEGER PRIMARY KEY AUTOINCREMENT, ts REAL NOT NULL, kit TEXT, "
                 "pad_figure TEXT)")
    conn.commit()
    conn.close()
    store = MusicHistory(str(path))
    assert store.record(kit="four", bass_figure="rolling8")
    assert store.recent(1)[0]["bass_figure"] == "rolling8"
