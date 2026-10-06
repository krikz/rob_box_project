"""ADR-0152 PR-8 (§3.5, В1): жанровые окна клуба — ``club``/``deep``/``breaks`` на СЕТ, штраф между сетами.

Окно — данные ``Style.genre_windows``; ``arrange/*`` о жанре не знают (``knowledge.genre_style`` подменяет поля
``Style``). Поведение — на ``SetPlan`` и модели ``Track``.
"""

from __future__ import annotations

import dataclasses
import random
import sqlite3
from collections import Counter

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.arrange import mix
from rob_box_music.arrange.compose import compose
from rob_box_music.diversity import MusicHistory, kick_name, track_composition, track_history
from rob_box_music.model import TrackError, blend_bars, validate
from rob_box_music.render.renardo import render
from rob_box_music.set_plan import pick_genre, plan_bpm, recent_genres, seeded_plan
from rob_box_music.theme import seeded_profile

CLUB = kn.STYLES[kn.DEFAULT_STYLE]
WINDOWS = CLUB.genre_windows
THEMES = ("космос", "киберпанк", "детский праздник", "калинка", "бухгалтерский отчёт", "")


def _plan(theme: str, seed: int, history=()):
    return seeded_plan(seeded_profile(theme), seed, set_id=f"g{seed}", history=history)


def test_table_has_three_windows_and_the_default_is_the_style_itself():
    assert list(WINDOWS) == ["club", "deep", "breaks"] and kn.DEFAULT_GENRE == "club"
    assert kn.genre_style(CLUB, "club") == CLUB, "окно club — сегодняшние таблицы стиля"
    for name, w in WINDOWS.items():
        assert w.bpm[0] < w.bpm[1] and len(w.kick_pool) >= 2 and set(w.kick_pool) <= set(kn.KICK_SOUNDS), name
        assert set(w.pad_figures) <= set(kn.PAD_FIGURES), name
        assert kn.genre_style(CLUB, name).kick_pool == w.kick_pool


def test_breaks_drop_plays_the_breakbeat_and_blend_sections_stay_straight():
    for name in WINDOWS:
        style = kn.genre_style(CLUB, name)
        expected = kn.KICK_PATTERNS["breakbeat" if name == "breaks" else "four_on_floor"]
        assert mix.look(style, 9).kick == expected, name
        assert mix.look(style, 2).kick == kn.KICK_PATTERNS["four_on_floor"], "интро/аутро: одна бочка на такт блэнда"


def test_breaks_set_has_the_breakbeat_in_the_kick_grid_of_the_drop():
    plan = next(p for p in (_plan("", s) for s in range(80)) if p.genre == "breaks")
    track = compose(plan, 2)
    bar = sum(s.bars for s in track.form.sections if s.name in ("intro", "intro_low", "build"))
    steps = track.parts["kick"].grid.steps[bar * 16:(bar + 1) * 16]
    assert tuple(i for i, st in enumerate(steps) if st.on) == mix.kick_steps(kn.KICK_PATTERNS["breakbeat"])


@pytest.mark.parametrize("theme", THEMES)
def test_bpm_and_kick_come_from_the_window_of_the_set(theme):
    for seed in range(30):
        plan = _plan(theme, seed)
        w = WINDOWS[plan.genre]
        assert w.bpm[0] <= plan.bpm <= w.bpm[1], (theme, seed, plan.genre, plan.bpm)
        assert all(t.kick in w.kick_pool for t in plan.tracks), (seed, plan.genre)
    for seed in range(5):
        plan = _plan(theme, seed)
        for no in (1, 2, 5, 12):  # 12 — за пределами плана: выбор в compose по истории
            track = compose(plan, no)
            assert track.bpm == plan.bpm and kick_name(track.parts["kick"].sample) in WINDOWS[plan.genre].kick_pool
            validate(track)


def test_one_tempo_and_one_window_for_the_whole_set():
    for seed in range(10):
        plan = _plan("космос", seed)
        tracks = [compose(plan, no) for no in range(1, 8)]
        assert {t.bpm for t in tracks} == {plan.bpm}
        assert {track_composition(t)["genre"] for t in tracks} == {plan.genre}
        assert {t.history_key.genre for t in tracks} == {plan.genre}


def test_theme_tempo_inside_the_window_is_kept_and_outside_is_seeded_in_it():
    w = WINDOWS["deep"]
    assert plan_bpm(w, 124, 1, "t") == 124
    picks = {plan_bpm(w, 138, s, "t") for s in range(30)}
    assert picks <= set(range(w.bpm[0], w.bpm[1] + 1)) and len(picks) > 1


def test_thirty_seeds_give_every_window():
    used = Counter(_plan("", seed).genre for seed in range(30))
    assert set(used) == set(WINDOWS), used


def test_five_random_sets_give_at_least_two_windows():
    for base in range(0, 60, 5):
        assert len({_plan("космос", s).genre for s in range(base, base + 5)}) >= 2, base


def test_history_never_repeats_the_window_of_the_last_set():
    for seed in range(60):
        for last in WINDOWS:
            rows = [{"set_id": "s2", "genre": last}] * 3 + [{"set_id": "s1", "genre": "club"}] * 3
            assert pick_genre(CLUB, rows, random.Random(seed)) != last
            assert _plan("космос", seed, rows).genre != last


def test_penalty_prefers_the_window_not_played_for_longest():
    rows = [{"set_id": "s2", "genre": "deep"}, {"set_id": "s1", "genre": "club"}]
    picks = Counter(pick_genre(CLUB, rows, random.Random(s)) for s in range(300))
    assert picks["deep"] == 0 and picks["breaks"] > picks["club"], picks


def test_recent_genres_collapse_tracks_into_sets_and_skip_old_rows():
    rows = [{"set_id": "b", "genre": "deep"}, {"set_id": "b", "genre": "deep"}, {"set_id": "a", "genre": None},
            {"set_id": "a", "genre": "club"}, {"set_id": "z"}]
    assert recent_genres(rows) == ["deep", "club"]
    assert recent_genres([]) == []


@pytest.mark.parametrize("name", list(WINDOWS))
def test_every_window_blends_and_renders(name):
    plan = next(p for p in (_plan("", s) for s in range(80)) if p.genre == name)
    a, b = compose(plan, 2, deck="A"), compose(plan, 3, deck="B")
    assert blend_bars(a, b) == plan.table.blend[0]
    for track in (a, b):
        validate(track)
        render(track, "A")


def test_pad_figures_of_the_window_weight_the_choice():
    seen = {name: Counter() for name in WINDOWS}
    for seed in range(40):
        plan = _plan("", seed)
        for no in (2, 3, 4):
            seen[plan.genre][compose(plan, no).history_key.pad_figure] += 1
    for name, counter in seen.items():
        assert set(counter) <= set(WINDOWS[name].pad_figures), (name, counter)


def test_unknown_window_in_history_key_is_rejected():
    track = compose(_plan("", 3), 1)
    key = dataclasses.replace(track.history_key, genre="polka")
    with pytest.raises(TrackError):
        validate(dataclasses.replace(track, history_key=key))


def test_history_stores_the_window_and_old_databases_are_migrated(tmp_path):
    path = str(tmp_path / "h.db")
    MusicHistory(path).close()
    old = sqlite3.connect(path)  # база версии до PR-8: та же схема без колонки genre
    old.execute("ALTER TABLE music_history DROP COLUMN genre")
    assert "genre" not in {r[1] for r in old.execute("PRAGMA table_info(music_history)")}
    old.commit()
    old.close()
    history = MusicHistory(path)
    plan = _plan("космос", 7)
    track = compose(plan, 1)
    assert history.record(**track_history(track, plan.set_id))
    assert history.recent(1)[0]["genre"] == plan.genre == track_composition(track)["genre"]
    history.close()
