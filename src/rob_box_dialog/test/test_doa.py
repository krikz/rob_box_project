"""Направление фразы по DOA во время речи (ADR-0150 §7.3 В12, PR-5)."""

import pytest

from rob_box_dialog.doa import DoaTrack, circular_mean_deg


def test_circular_mean_wraps_around_zero():
    assert circular_mean_deg([350.0, 10.0]) == pytest.approx(0.0, abs=1e-6)
    assert circular_mean_deg([80.0, 100.0]) == pytest.approx(90.0)
    assert circular_mean_deg([]) is None
    assert circular_mean_deg([0.0, 180.0]) is None


def test_only_samples_during_speech_count():
    track = DoaTrack()
    track.on_direction(270.0, 1.0)  # телевизор без VAD
    track.on_vad(True, 2.0)
    track.on_direction(90.0, 2.1)
    track.on_direction(100.0, 2.2)
    track.on_vad(False, 3.0)
    track.on_direction(270.0, 3.1)
    assert track.during(0.0, 4.0) == pytest.approx(95.0)


def test_window_selects_the_phrase():
    track = DoaTrack()
    track.on_vad(True, 0.0)
    track.on_direction(10.0, 1.0)
    track.on_direction(200.0, 5.0)
    assert track.during(4.0, 6.0) == pytest.approx(200.0)
    assert track.during(7.0, 8.0) is None


def test_old_samples_are_dropped():
    track = DoaTrack(history_s=10.0)
    track.on_vad(True, 0.0)
    track.on_direction(10.0, 0.0)
    track.on_direction(20.0, 20.0)
    assert track.during(-1.0, 1.0) is None
    assert track.during(19.0, 21.0) == pytest.approx(20.0)
