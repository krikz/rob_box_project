"""Нотные события -> буферы: базовые контракты синтеза (issue #2977)."""

from __future__ import annotations

import numpy as np

from tools.music_bench.note_extract import NoteEvent
from tools.music_bench.synth import render_events, sum_buffers

SR = 16000


def test_render_events_places_tone_at_role_buffer():
    events = [NoteEvent(role="bass", start_beat=0.0, dur_beat=1.0, amp=0.5, freqs_hz=(110.0,))]
    buffers = render_events(events, total_beats=2.0, bpm=120.0, sample_rate=SR)
    assert "bass" in buffers
    assert np.max(np.abs(buffers["bass"])) > 0.0


def test_render_events_noise_burst_for_drum_role():
    events = [NoteEvent(role="drums", start_beat=0.0, dur_beat=0.5, amp=0.5, noise_band_hz=(50.0, 150.0))]
    buffers = render_events(events, total_beats=1.0, bpm=120.0, sample_rate=SR)
    assert "drums" in buffers
    assert np.max(np.abs(buffers["drums"])) > 0.0


def test_render_events_amp_scales_output():
    quiet = [NoteEvent(role="lead", start_beat=0.0, dur_beat=1.0, amp=0.1, freqs_hz=(440.0,))]
    loud = [NoteEvent(role="lead", start_beat=0.0, dur_beat=1.0, amp=0.9, freqs_hz=(440.0,))]
    quiet_buf = render_events(quiet, total_beats=2.0, bpm=120.0, sample_rate=SR)["lead"]
    loud_buf = render_events(loud, total_beats=2.0, bpm=120.0, sample_rate=SR)["lead"]
    assert np.max(np.abs(loud_buf)) > np.max(np.abs(quiet_buf))


def test_sum_buffers_adds_roles_together():
    buffers = {
        "bass": np.array([0.1, 0.2, 0.3]),
        "lead": np.array([0.05, 0.0, -0.1]),
    }
    total = sum_buffers(buffers)
    np.testing.assert_allclose(total, [0.15, 0.2, 0.2])


def test_sum_buffers_empty_is_silence():
    total = sum_buffers({})
    assert np.max(np.abs(total)) == 0.0
