"""Метрики стенда (issue #2977) на синтетических сигналах с известным ответом."""

from __future__ import annotations

import numpy as np

from tools.music_bench.metrics import (
    crest_factor_db,
    pct_time_over,
    peak,
    simultaneous_layers_per_section,
    spectral_centroid_hz,
    true_peak_approx,
)
from tools.music_bench.note_extract import NoteEvent

SR = 16000


def test_pct_time_over_half_the_signal():
    sig = np.concatenate([np.full(500, 2.0), np.full(500, 0.5)])
    assert pct_time_over(sig, 1.0) == 0.5


def test_pct_time_over_zero_when_all_below():
    sig = np.full(1000, 0.3)
    assert pct_time_over(sig, 1.0) == 0.0


def test_peak_and_true_peak_at_least_sample_peak():
    sig = np.array([0.1, 0.9, -0.95, 0.2])
    assert abs(peak(sig) - 0.95) < 1e-9
    assert true_peak_approx(sig) >= peak(sig) - 1e-6


def test_crest_factor_of_pure_sine_is_about_3db():
    t = np.arange(SR) / SR
    sig = np.sin(2 * np.pi * 440 * t)
    crest = crest_factor_db(sig)
    assert 2.5 < crest < 3.5  # crest(sine) = sqrt(2) ~ 3.01 dB


def test_crest_factor_silence_is_zero_not_inf():
    assert crest_factor_db(np.zeros(100)) == 0.0


def test_spectral_centroid_pure_tone_matches_frequency():
    t = np.arange(SR) / SR
    sig = np.sin(2 * np.pi * 1000 * t)
    centroid = spectral_centroid_hz(sig, SR)
    assert 950 < centroid < 1050


def test_spectral_centroid_silence_is_zero():
    assert spectral_centroid_hz(np.zeros(100), SR) == 0.0


def test_simultaneous_layers_counts_distinct_roles_in_window():
    plan = [("intro", 4, {}), ("main", 4, {})]
    events = [
        NoteEvent(role="bass", start_beat=1.0, dur_beat=1.0, amp=0.5),
        NoteEvent(role="pad", start_beat=1.0, dur_beat=1.0, amp=0.3),
        NoteEvent(role="lead", start_beat=20.0, dur_beat=1.0, amp=0.5),  # 2-я секция
    ]
    result = simultaneous_layers_per_section(events, plan, beats_per_bar=4)
    names = {name: count for name, _bars, count in result}
    assert names["intro"] == 2
    assert names["main"] == 1


def test_simultaneous_layers_ignores_zero_amp_events():
    plan = [("intro", 4, {})]
    events = [NoteEvent(role="bass", start_beat=0.0, dur_beat=1.0, amp=0.0)]
    result = simultaneous_layers_per_section(events, plan, beats_per_bar=4)
    assert result[0][2] == 0
