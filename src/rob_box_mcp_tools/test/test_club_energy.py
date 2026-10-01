"""Issue #3311 / ADR-0147 — уровень энергии club-трека (первый инкремент).

Держат: ``energy=None`` — побайтно прежний трек; таблица громкости даёт
размах ~9 dB и только вниз (пик не растёт); плотность и фильтр монотонны
по энергии; модель громкости честно показывает, что плотность RMS почти
не двигает (поэтому громкость — отдельная ось после динамики, ADR-0147 §3).
Числа таблиц — гипотеза дизайна, не замер на роботе.
"""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.core import club_loudness as L
from rob_box_mcp_tools.core.club_arranger import (
    LAYER_LEVELS,
    MAX_LAYER_AMP,
    REFERENCE_KIT,
    build_matrix,
    render_club,
    render_club_kit,
)
from rob_box_mcp_tools.core.club_energy import (
    ENERGY_DENSITY,
    ENERGY_LEVELS,
    ENERGY_LPF_SCALE,
    ENERGY_TRIM_DB,
    apply_energy,
    energy_levels,
    energy_lpf,
    energy_note,
    energy_trim_db,
    validate_energy,
)
from rob_box_mcp_tools.core.club_pools import LPF_PROFILES


@pytest.mark.parametrize("seed", [0, 1, 7, 3311])
def test_energy_none_renders_byte_identical(seed):
    assert render_club(seed=seed, energy=None) == render_club(seed=seed)


@pytest.mark.parametrize("seed", [0, 7])
def test_full_energy_changes_only_header(seed):
    """Энергия 4–5: полный микс и открытый фильтр — отличается только заголовок."""
    base = render_club(seed=seed).splitlines()
    for energy in (4, 5):
        lines = render_club(seed=seed, energy=energy).splitlines()
        assert lines[0] == base[0] + f", энергия {energy}"
        assert lines[1:] == base[1:]


def test_trim_spans_nine_db_and_never_raises_peak():
    trims = [energy_trim_db(e) for e in ENERGY_LEVELS]
    assert trims == [ENERGY_TRIM_DB[e] for e in ENERGY_LEVELS]
    assert trims == sorted(trims)
    assert max(trims) == 0.0
    assert max(trims) - min(trims) == pytest.approx(9.0)
    assert len(set(trims)) == len(trims)


@pytest.mark.parametrize("bad", [0, 6, -1, True, "3", 3.0, None])
def test_validate_energy_rejects_out_of_range(bad):
    with pytest.raises(ValueError):
        validate_energy(bad)


def test_render_club_rejects_bad_energy():
    with pytest.raises(ValueError, match="energy"):
        render_club(seed=0, energy=9)


def test_density_monotonic_in_energy():
    lanes = {lane for d in ENERGY_DENSITY.values() for lane in d}
    for lane in lanes:
        factors = [ENERGY_DENSITY[e].get(lane, 1.0) for e in ENERGY_LEVELS]
        assert factors == sorted(factors), lane
        assert all(0.0 <= f <= 1.0 for f in factors)
    for lane in ("kick", "bass"):  # ритм держат всегда
        assert all(lane not in ENERGY_DENSITY[e] for e in ENERGY_LEVELS)


def test_energy_levels_multiplies_explicit_levels():
    assert energy_levels(1, {"lead": 0.5, "pad": 0.8}) == {
        "clap": 0.0, "hats": 0.5, "lead": pytest.approx(0.35), "pad": 0.8,
    }
    assert energy_levels(5, None) is None
    assert energy_levels(4, {"pad": 0.8}) == {"pad": 0.8}


def test_lpf_scale_monotonic_and_floor():
    scales = [ENERGY_LPF_SCALE[e] for e in ENERGY_LEVELS]
    assert scales == sorted(scales) and scales[-1] == 1.0
    lead, bass = LPF_PROFILES["reference"]
    assert energy_lpf(lead, 1) == "linvar([405, 1800], 31)"
    assert energy_lpf(bass, 1) == "linvar([250, 1125], 61)"  # 225 → пол 250 Гц
    assert energy_lpf(lead, 5) == lead
    for profile in LPF_PROFILES.values():
        for expr in profile:
            assert energy_lpf(expr, 2) != expr


def test_apply_energy_none_is_identity():
    lpf = LPF_PROFILES["slow"]
    levels = {"lead": 0.9}
    assert apply_energy(None, lpf, levels) == (lpf, levels)
    assert energy_note(None) == ""


def test_low_energy_render_mutes_clap_and_darkens():
    code = render_club_kit(REFERENCE_KIT, energy=1)
    d3 = next(line for line in code.splitlines() if line.startswith("d3 >>"))
    assert d3.endswith("amp=0)")
    assert "lpf=linvar([405, 1800], 31)" in code
    assert "lpf=linvar([250, 1125], 61)" in code


def test_density_barely_moves_model_rms():
    """Модель #3154: бочка и бас держат RMS, плотность фактуры его почти не двигает.

    Поэтому дуга громкости не может жить в плотности — только в смещении
    после динамики мастер-шины (ADR-0147 §3, альтернатива В отклонена).
    """
    matrix = build_matrix(REFERENCE_KIT["template"])
    cal = L.calibrate_levels(matrix, REFERENCE_KIT, LAYER_LEVELS, MAX_LAYER_AMP)
    base = L.main_db(matrix, REFERENCE_KIT, cal)
    density = ENERGY_DENSITY[1]
    low = {lane: [v * density.get(lane, 1.0) for v in vals] for lane, vals in cal.items()}
    assert abs(L.main_db(matrix, REFERENCE_KIT, low) - base) < 1.0
