"""#3550 (приёмка S7 08.10): шесть электронных стилей звучат по-разному — составы стилей различаются ТАБЛИЦАМИ.

На роботе club/rave/synthwave/chiptune/breaks/dnb были «клубом с другим синтом» (отношение межстилевого расстояния
к внутристилевому 0.68–1.14): общие клубные бочки, каркасы хэтов, клэп, пэды, psr-слой; «клубный» сет в окне
``breaks`` и стиль ``breaks`` на одной теме собрали одинаковый трек 1. Звук офлайн не мерится (рендер событий без
scsynth) — проверка на роботе ``s7_metrics.py``; здесь — что оси звука стиля (кит, рисунки, тембры, сцена, свип,
слои) — разные данные, а сид плана и трека зависит от стиля.
"""

from __future__ import annotations

import dataclasses
import itertools

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.arrange.compose import compose
from rob_box_music.set_plan import plan_kicks, plan_salt, plan_templates, seeded_plan
from rob_box_music.theme import seeded_profile

ELECTRONIC = ("club", "rave", "synthwave", "chiptune", "breaks", "dnb")
PAIRS = list(itertools.combinations(ELECTRONIC, 2))


def _kicks(style: str) -> set:
    return {k for w in kn.STYLES[style].genre_windows.values() for k in w.kick_pool}


def _synths(style: str, role: str) -> set:
    return {s for fam in kn.STYLES[style].timbres.values() for s in fam[role]}


def _jaccard(a: set, b: set) -> float:
    return len(a & b) / len(a | b)


@pytest.mark.parametrize("a,b", PAIRS)
def test_kits_of_two_styles_share_no_kick_no_drum_file_and_no_hats_pattern(a, b):
    """Бочки (все окна), файлы малого и хэтов, рисунки хэтов каркасов — у каждого стиля свои."""
    assert not _kicks(a) & _kicks(b), (a, b, _kicks(a) & _kicks(b))
    for role in ("clap", "hats"):
        files_a, files_b = set(kn.STYLES[a].drum_files.get(role, ())), set(kn.STYLES[b].drum_files.get(role, ()))
        assert not files_a & files_b, (a, b, role)
        assert files_a or files_b, f"{a} и {b} оба играют {role} дефолтным паком Renardo"
    hats_a = {kit["hats"] for kit in kn.STYLES[a].kits.values()}
    hats_b = {kit["hats"] for kit in kn.STYLES[b].kits.values()}
    assert not hats_a & hats_b, (a, b, hats_a & hats_b)


@pytest.mark.parametrize("a,b", PAIRS)
def test_timbre_pools_of_two_styles_overlap_less_than_half(a, b):
    """Пулы синтов (лид ∪ пэд ∪ бас по всем семьям) двух стилей пересекаются меньше чем наполовину; пэды — не больше
    половины (S7: у клуба, рейва, breaks и dnb пэды были одни и те же клубные семьи, у breaks и dnb — все тембры)."""
    pools_a = set().union(*(_synths(a, r) for r in kn.TONAL_ROLES))
    pools_b = set().union(*(_synths(b, r) for r in kn.TONAL_ROLES))
    assert _jaccard(pools_a, pools_b) < 0.55, (a, b, sorted(pools_a & pools_b))
    assert _jaccard(_synths(a, "pad"), _synths(b, "pad")) <= 0.5, (a, b)


@pytest.mark.parametrize("a,b", PAIRS)
def test_mix_of_two_styles_differs_in_scene_sweep_and_levels(a, b):
    """Сцена (стерео ролей), свип секций и уровни ролей — данные стиля; у двух стилей различаются хотя бы два из трёх."""
    sa, sb = kn.STYLES[a], kn.STYLES[b]
    diff = [dict(sa.stereo) != dict(sb.stereo), dict(sa.section_lpf) != dict(sb.section_lpf),
            dict(sa.role_level_db) != dict(sb.role_level_db)]
    assert sum(diff) >= 2, (a, b, diff)


def test_dj_dave_psr_layer_is_the_face_of_the_club_only():
    """psr-слой DJ_Dave (``layer_sections["sample"]``) звучит только в клубе."""
    assert [s for s in ELECTRONIC if "sample" in kn.STYLES[s].layer_sections] == ["club"]


def test_club_has_no_breaks_window_and_the_broken_beat_is_the_breaks_style():
    """S7: «клубный» сет в окне ``breaks`` дублировал стиль ``breaks`` — окна у клуба нет, рисунок ``broken`` — нет."""
    club = kn.STYLES["club"]
    assert "breaks" not in club.genre_windows
    assert all("broken" not in w.bass_figures for w in club.genre_windows.values())
    assert all("broken" in w.bass_figures for w in kn.STYLES["breaks"].genre_windows.values())


def test_tempo_windows_are_one_table():
    """``Style.bpm`` — первое окно стиля (одна таблица окон; S7: rave 160 в окне hardcore при ``bpm`` 140–150);
    окна рейва и dnb не пересекаются, все окна — в тракте Renardo."""
    for name, style in kn.STYLES.items():
        assert style.bpm == next(iter(style.genre_windows.values())).bpm, name
        assert all(kn.BPM_RANGE[0] <= w.bpm[0] < w.bpm[1] <= kn.BPM_RANGE[1] for w in style.genre_windows.values())
    rave = [w.bpm for w in kn.STYLES["rave"].genre_windows.values()]
    dnb = [w.bpm for w in kn.STYLES["dnb"].genre_windows.values()]
    assert max(hi for _, hi in rave) < min(lo for lo, _ in dnb), (rave, dnb)


def test_style_is_part_of_the_plan_and_the_track_seed():
    """Тот же сид и та же тема в двух стилях — разные ГСЧ осей: одна и та же таблица под двумя ключами стиля
    выбирает разные бочки/формы (соль :func:`plan_salt`), сид трека несёт стиль и окно (``SetPlan.track_seed``)."""
    club = seeded_profile("космос", "club")
    other = dataclasses.replace(club, style="breaks")
    table = kn.STYLES["club"]
    kicks = {seed: (plan_kicks(table, seed, plan_salt(club), 6), plan_kicks(table, seed, plan_salt(other), 6))
             for seed in range(20)}
    assert sum(a != b for a, b in kicks.values()) >= 10, kicks
    forms = [(plan_templates(table, s, plan_salt(club), [2, 3, 4, 5, 4]),
              plan_templates(table, s, plan_salt(other), [2, 3, 4, 5, 4])) for s in range(20)]
    assert sum(a != b for a, b in forms) >= 10
    plan = seeded_plan(club, 7)
    assert plan.track_seed(2) == f"7:club:{plan.genre}:2"
    other_window = next(g for g in table.genre_windows if g != plan.genre)
    moved = dataclasses.replace(plan, genre=other_window)
    assert moved.track_seed(2) != plan.track_seed(2)


def test_same_seed_and_theme_give_different_first_tracks_in_every_style():
    """S7: тот же сид и тема «космос» — трек 1 каждого электронного стиля свой по бочке, каркасу и паре бас/пэд."""
    seen = {}
    for style in ELECTRONIC:
        plan = seeded_plan(seeded_profile("космос", style), 31476, set_id="s7")
        track = compose(plan, 1)
        key = track.history_key
        seen[style] = (track.parts["kick"].synth_or_sample, track.parts["kick"].sample, key.kit,
                       track.parts["bass"].synth_or_sample, track.parts["pad"].synth_or_sample)
    for a, b in PAIRS:
        assert seen[a][:3] != seen[b][:3] and seen[a][2] != seen[b][2], (a, b, seen[a], seen[b])
    assert len({v[3:] for v in seen.values()}) >= 5, seen
