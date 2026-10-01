"""Тесты пулов club (issue #3226, ADR-0146): лады, клэп, баланс слоёв, lpf-огибающие."""

import re
import sqlite3
from collections import Counter

import pytest

from rob_box_mcp_tools.core import club_loudness as L
from rob_box_mcp_tools.core.arranger import VALID_ROOTS
from rob_box_mcp_tools.core.club_arranger import (
    CLUB_SYNTHS,
    LAYER_LEVELS,
    MAX_LAYER_AMP,
    PENTATONIC,
    PROGRESSIONS,
    SUPPORTED_SCALES,
    build_matrix,
    club_kit,
    club_progression,
    render_club,
    render_club_kit,
)
from rob_box_mcp_tools.core.club_pools import (
    CLAP_PATTERNS,
    LEVEL_PROFILES,
    LPF_PROFILES,
    REFERENCE_VARIANT,
    club_variant,
    variant_levels,
)
from rob_box_mcp_tools.core.club_progressions import (
    LEGACY_PROGRESSIONS,
    SCALE_PENTATONIC,
    pick_progression_name,
    progression_mode,
    progressions_for,
)
from rob_box_mcp_tools.core.music_diversity import MusicHistory
from rob_box_mcp_tools.core.renardo_sanitizer import _play_steps, sanitize_renando


def _sanitizer_clean(code: str) -> None:
    result = sanitize_renando(code, 0.85, known_synths=CLUB_SYNTHS)
    assert result.security_error is None and result.quality_errors == ()
    assert result.slot_error is None and result.warnings == ()
    assert result.code == code


def _lead_notes(code):
    block = re.search(r"p1 >> \w+\(\[(.*?)\],\n", code, re.DOTALL).group(1)
    return [int(v) for v in re.findall(r"\d+", block)]


def _play(rows, seed, scale="minor"):
    """Один трек с историей (как ``ComposeMusicTool``): рендер + запись выбранного."""
    kit = club_kit(seed, recent=rows)
    variant = club_variant(seed, rows)
    prog = club_progression(seed, rows, scale)
    code = render_club(seed=seed, scale=scale, recent=rows)
    rows.insert(0, dict(kit, progression=prog, **variant))
    return code, kit, variant, prog


# ------------------------------------------------------------- банк прогрессий
def test_legacy_bank_is_frozen_and_first():
    assert [n for n, _ in PROGRESSIONS[:5]] == [n for n, _ in LEGACY_PROGRESSIONS]
    assert [n for n, _ in LEGACY_PROGRESSIONS] == ["VI-III-VII-i", "i-VI-III-VII", "i-VII-VI-VII", "i-iv-VI-v", "VI-VII-i-i"]
    assert len({n for n, _ in PROGRESSIONS}) == len(PROGRESSIONS)


def test_every_supported_scale_has_progressions_and_minor_is_wider():
    assert SUPPORTED_SCALES[0] == "minor"
    for scale in SUPPORTED_SCALES:
        assert progressions_for(scale), scale
    assert len(progressions_for("minor")) >= 10
    assert {progression_mode(n) for n, _ in PROGRESSIONS} == set(SUPPORTED_SCALES)


@pytest.mark.parametrize("scale", SUPPORTED_SCALES)
def test_mode_progressions_start_and_use_their_mode_colour(scale):
    """Дорийская содержит мажорную IV, фригийская — мажорную bII, мажор — мажорную тонику."""
    marker = {
        "minor": None, "dorian": (5, "M"), "phrygian": (1, "M"), "major": (0, "M"),
        "minorPentatonic": (0, "m"), "majorPentatonic": (0, "M"),  # #3268: тоника лада
    }[scale]
    bank = dict(PROGRESSIONS)
    for name in progressions_for(scale):
        chords = bank[name]
        assert len(chords) == 4
        if marker:
            assert marker in chords, name
        else:
            assert (0, "m") in chords, name


# ------------------------------------------------------------ рендер по ладам
@pytest.mark.parametrize("scale", SUPPORTED_SCALES)
@pytest.mark.parametrize("seed", [0, 1, 7, 42, 6261504])
def test_render_each_scale_is_clean_and_uses_scale_progression(scale, seed):
    code = render_club(seed=seed, scale=scale, root="A")
    _sanitizer_clean(code)
    header = re.search(r"^# club: [^,]+, ([^,]+), ([^,]+), ([^,]+),", code, re.MULTILINE)
    assert header.group(1) == f"A {scale}"
    assert progression_mode(header.group(2)) == scale
    assert code == render_club(seed=seed, scale=scale, root="A")


@pytest.mark.parametrize("scale", SUPPORTED_SCALES)
def test_lead_follows_chord_pentatonic_in_every_scale(scale):
    for root in ("A#", "C", "F#"):
        code = render_club(seed=5, scale=scale, root=root)
        name = re.search(r"^# club: [^,]+, [^,]+, ([^,]+),", code, re.MULTILINE).group(1)
        chords = dict(PROGRESSIONS)[name]
        notes = _lead_notes(code)
        tonic = VALID_ROOTS.index(root)
        for idx, (offset, quality) in enumerate(chords):
            steps = SCALE_PENTATONIC.get(scale) or [offset + i for i in PENTATONIC[quality]]  # #3268: лад-пентатоника
            allowed = {(tonic + i) % 12 for i in steps}
            assert {n % 12 for n in notes[idx * 32:(idx + 1) * 32]} <= allowed, (name, idx)


def test_progression_of_other_scale_is_refused():
    kit = club_kit(3)
    with pytest.raises(ValueError, match="лад"):
        render_club_kit(kit, scale="minor", progression="maj:I-V-vi-IV")
    with pytest.raises(ValueError, match="лад"):
        render_club_kit(kit, scale="major", progression="i-VI-III-VII")
    with pytest.raises(ValueError, match="Лад"):
        render_club(scale="lydian")


def test_minor_without_history_keeps_legacy_seed_choice():
    import random

    for seed in (1, 2, 99, 6261504):
        expected = LEGACY_PROGRESSIONS[random.Random(seed).randrange(5)][0]
        assert pick_progression_name(seed) == expected == club_progression(seed, None)
        assert club_progression(seed, []) == expected


def test_history_widens_minor_pool_beyond_legacy_five():
    rows = [{"progression": "VI-III-VII-i"}]
    seen = {club_progression(s, rows) for s in range(1, 300)}
    assert seen - {n for n, _ in LEGACY_PROGRESSIONS}
    assert all(progression_mode(n) == "minor" for n in seen)


# ------------------------------------------------------------ пулы клэпа/lpf/баланса
def test_variant_reference_without_history_and_at_seed_zero():
    assert club_variant(5) == REFERENCE_VARIANT == club_variant(5, [])
    assert club_variant(0, [{"clap": "dry"}]) == REFERENCE_VARIANT


def test_clap_patterns_are_valid_16_step_grids_with_a_clap():
    for name, pattern in CLAP_PATTERNS.items():
        steps = _play_steps(pattern)
        assert steps is not None and len(steps) == 16, name
        assert "*" in pattern


@pytest.mark.parametrize("profile", sorted(LEVEL_PROFILES))
def test_balance_profile_only_attenuates_within_15_percent(profile):
    factors = LEVEL_PROFILES[profile]
    assert all(0.85 <= f <= 1.0 for f in factors.values())
    assert set(factors) <= set(LAYER_LEVELS)


def test_variant_levels_multiplies_explicit_levels():
    assert variant_levels(None) is None
    assert variant_levels(REFERENCE_VARIANT, {"lead": 0.5}) == {"lead": 0.5}
    merged = variant_levels({"balance": "soft_lead"}, {"lead": 0.5, "pad": 0.5})
    assert merged["lead"] == pytest.approx(0.45) and merged["pad"] == 0.5


@pytest.mark.parametrize("clap", sorted(CLAP_PATTERNS))
@pytest.mark.parametrize("lpf", sorted(LPF_PROFILES))
@pytest.mark.parametrize("balance", sorted(LEVEL_PROFILES))
def test_every_variant_combination_renders_sanitizer_clean(clap, lpf, balance):
    variant = {"clap": clap, "lpf": lpf, "balance": balance}
    code = render_club_kit(club_kit(4), seed=4, variant=variant)
    _sanitizer_clean(code)
    assert f'play("{CLAP_PATTERNS[clap]}"' in code
    lead_lpf, bass_lpf = LPF_PROFILES[lpf]
    assert f"lpf={lead_lpf}," in code and f"lpf={bass_lpf}," in code


def test_reference_variant_render_is_byte_identical_to_no_variant():
    kit = club_kit(11)
    assert render_club_kit(kit, seed=11, variant=REFERENCE_VARIANT) == render_club_kit(kit, seed=11)


@pytest.mark.parametrize("profile", sorted(LEVEL_PROFILES))
@pytest.mark.parametrize("seed", [0, 7, 42, 6261504])
def test_loudness_never_rises_and_drops_at_most_2db(profile, seed):
    """Профиль баланса умножает калибровку ≤ 1: основной блок по модели
    #3154 не громче эталона и не тише него более чем на 2 дБ."""
    kit = club_kit(seed)
    matrix = build_matrix(kit["template"])
    ref = L.calibrate_levels(matrix, kit, LAYER_LEVELS, MAX_LAYER_AMP)
    scaled = {
        lane: [v * LEVEL_PROFILES[profile].get(lane, 1.0) for v in values] for lane, values in ref.items()
    }
    delta = L.main_db(matrix, kit, ref) - L.main_db(matrix, kit, scaled)
    assert -0.01 <= delta <= 2.0, (profile, delta)


# ------------------------------------------------------------ разнообразие по 20 трекам
@pytest.mark.parametrize("base", [1000, 6261502, 7690914])
def test_twenty_tracks_with_shared_history_are_diverse(base):
    rows = []
    tracks = [_play(rows, base + i) for i in range(20)]
    progs = Counter(p for _, _, _, p in tracks)
    assert len(progs) >= 6 and max(progs.values()) <= 4, progs
    for role in ("clap", "lpf", "balance"):
        used = Counter(v[role] for _, _, v, _ in tracks)
        assert len(used) >= 3, (role, used)
        assert max(used.values()) <= 10, (role, used)
    assert len({code for code, *_ in tracks}) == 20
    for code, *_ in tracks:
        _sanitizer_clean(code)


def test_twenty_tracks_in_every_scale_use_only_that_scales_progressions():
    for scale in SUPPORTED_SCALES:
        rows = []
        tracks = [_play(rows, 500 + i, scale) for i in range(20)]
        assert {progression_mode(p) for *_, p in tracks} == {scale}
        assert len({p for *_, p in tracks}) >= 3


def test_seed_zero_ignores_history_for_variant_and_progression():
    rows = [dict(progression="VI-III-VII-i", clap="dry", lpf="slow", balance="deep")] * 5
    assert render_club(seed=0, recent=rows) == render_club(seed=0)


# ------------------------------------------------------------ история: новые колонки и старая схема
_OLD_SCHEMA = """
CREATE TABLE music_history (
    id INTEGER PRIMARY KEY AUTOINCREMENT, ts REAL NOT NULL, set_id TEXT, style TEXT,
    melody_name TEXT, fragment_offset INTEGER, hook_fingerprint TEXT, progression TEXT,
    template TEXT, kick TEXT, hats TEXT, lead TEXT, bass TEXT, pad TEXT, root TEXT, bpm REAL, scale TEXT
);
"""


def test_history_records_and_reads_variant_fields():
    hist = MusicHistory(":memory:")
    assert hist.record(style="club", progression="p", clap="dry", lpf="slow", balance="deep")
    (row,) = hist.recent()
    assert (row["clap"], row["lpf"], row["balance"]) == ("dry", "slow", "deep")


def test_old_db_without_new_columns_is_migrated_and_keeps_rows(tmp_path):
    db = str(tmp_path / "old.db")
    conn = sqlite3.connect(db)
    conn.executescript(_OLD_SCHEMA)
    conn.execute("INSERT INTO music_history (ts, style, progression) VALUES (1.0, 'club', 'VI-III-VII-i')")
    conn.commit()
    conn.close()
    hist = MusicHistory(db)
    assert hist.available
    old = hist.recent()
    assert len(old) == 1 and old[0]["progression"] == "VI-III-VII-i" and old[0]["clap"] is None
    assert hist.record(style="club", clap="ghost", lpf="wide", balance="airy")
    assert hist.recent()[0]["clap"] == "ghost"
    hist.close()
    again = MusicHistory(db)  # повторное открытие: миграция идемпотентна
    assert again.available and len(again.recent()) == 2
    cols = {r[1] for r in sqlite3.connect(db).execute("PRAGMA table_info(music_history)")}
    assert {"clap", "lpf", "balance"} <= cols


def test_old_history_rows_without_variant_fields_do_not_break_choice():
    rows = [{"progression": "VI-III-VII-i", "template": "dj_dave_32"}] * 3
    assert club_variant(9, rows) == club_variant(9, rows)
    assert render_club(seed=9, recent=rows) == render_club(seed=9, recent=rows)
