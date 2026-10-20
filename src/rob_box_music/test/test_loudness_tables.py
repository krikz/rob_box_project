"""Таблицы замера слоёв (ADR-0152 §3.1, §7 PR-2): полосы — доли одной энергии, замер покрывает палитру."""

from __future__ import annotations

import pytest

from rob_box_music import knowledge as kn


def test_layer_bands_sum_to_one_and_are_shares():
    for role, table in kn.LAYER_BANDS.items():
        for synth, shares in table.items():
            assert len(shares) == 3 and all(0.0 <= s <= 1.0 for s in shares), (role, synth)
            assert sum(shares) == pytest.approx(1.0, abs=0.002), (role, synth)


def test_every_palette_synth_has_loudness_and_bands():
    for role, synths in kn.SYNTH_PALETTE.items():
        for synth in synths:
            assert synth in kn.LANE_DB_AT_UNIT[role], (role, synth)
            assert synth in kn.LAYER_BANDS[role], (role, synth)


def test_bands_only_for_measured_layers():
    for role, table in kn.LAYER_BANDS.items():
        assert set(table) <= set(kn.LAYER_MEASURED_DB[role][1]), role


def test_band_edges_are_compare_profile_edges():
    assert kn.LAYER_BANDS_HZ == (150.0, 2000.0)
