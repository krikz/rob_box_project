"""Поведение замера A19 и карты растяжения спайка PR-S1 (#3709): без сети и без синтеза."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import approaches as ap  # noqa: E402
from common import SR  # noqa: E402
from metric import a19, coherence, grid_offsets_ms, line_onsets, syllable_onsets  # noqa: E402


def test_offsets_are_signed_from_nearest_node():
    assert grid_offsets_ms([0.11, 0.24], 0.0, 0.125).round(1).tolist() == [-15.0, -10.0]


def test_onsets_on_grid_score_one_random_score_chance():
    bpm, s16 = 120.0, 0.125
    on = np.arange(40) * s16
    got = a19(on, 0.0, bpm)
    assert got["p95_ms"] == 0.0 and got["C16"] == 1.0
    rnd = a19(np.random.default_rng(0).uniform(0, 20, 4000), 0.0, bpm)
    assert rnd["p50_norm"] == pytest.approx(0.25, abs=0.02) and abs(rnd["C16"]) < 0.05


def test_constant_shift_keeps_r16_but_kills_coherence():
    on = np.arange(40) * 0.125 + 0.0625  # ровно между 16-ми
    got = a19(on, 0.0, 120.0)
    assert got["R16"] == 1.0 and coherence(on, 0.0, 0.125) == pytest.approx(-1.0)


def test_line_onset_matched_within_window_only():
    assert line_onsets([1.0, 2.05, 5.0], [1.02, 2.0, 3.0]).tolist() == [1.0, 2.05]


def test_stretch_map_puts_onsets_on_sixteenths():
    bpm = 131.0
    s16 = 15.0 / bpm
    on = np.array([0.2, 0.38, 0.55, 0.77, 1.5, 1.66])
    anchors = ap.stretch_map(on, 2.0, bpm)
    outs = np.array([o for _, o in anchors[1:-1]])
    steps = (outs - outs[0]) / s16
    assert np.allclose(steps, np.round(steps)) and np.all(np.diff(outs) > 0)


def test_wsola_follows_time_map_length():
    x = np.sin(2 * np.pi * 200 * np.arange(SR) / SR).astype(np.float32)
    y = ap.wsola(x, [(0.0, 0.0), (0.5, 0.7), (1.0, 1.2)])
    assert abs(len(y) / SR - 1.2) < 0.01


def test_detector_finds_syllable_like_bursts():
    t = np.arange(int(2.0 * SR)) / SR
    env = np.zeros_like(t)
    for c in (0.3, 0.6, 0.9, 1.2):
        env += np.exp(-((t - c - 0.06) / 0.04) ** 2)
    x = (np.sin(2 * np.pi * 600 * t) * env).astype(np.float32)
    got = syllable_onsets(x)
    assert len(got) == 4 and np.all(np.abs(got - np.array([0.3, 0.6, 0.9, 1.2])) < 0.06)


def test_markup_pause_fills_bar():
    words = [{"word": ch, "t0": 0.1 * i, "t1": 0.1 * i + 0.1} for i, ch in enumerate("абвгд" + "еёжзи")]
    text, speed, n = ap.markup_text("абвгд,\nеёжзи.", words, 120.0)
    assert speed == 1.0 and n == (1, 1) and text == "абвгд <#1.50#> еёжзи."  # такт 2.0 с − строка 0.5 с; после последней паузы нет
