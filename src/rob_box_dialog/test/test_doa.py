"""Направление фразы по DOA во время речи (ADR-0150 §7.3 В12, PR-5)."""

import pytest

from rob_box_dialog.doa import DoaTrack, circular_mean_deg
from rob_box_dialog.knowledge import DOA_CONFIDENCE_MIN


def _angle(values):
    angle, _confidence = circular_mean_deg(values)
    return angle


def test_circular_mean_wraps_around_zero():
    assert _angle([350.0, 10.0]) == pytest.approx(0.0, abs=1e-6)
    assert _angle([80.0, 100.0]) == pytest.approx(90.0)
    assert _angle([]) is None
    assert _angle([0.0, 180.0]) is None


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


def test_agreed_samples_are_confident_and_opposed_cancel():
    """Согласованные отсчёты — R близко к 1, угол уходит потребителю; ~180° гасит вектор — угол не уходит."""
    agreed_angle, agreed_r = circular_mean_deg([80.0, 90.0, 100.0])
    assert agreed_angle == pytest.approx(90.0, abs=0.1)
    assert agreed_r == pytest.approx(1.0, abs=0.02)
    assert agreed_r >= DOA_CONFIDENCE_MIN

    cancelled_angle, cancelled_r = circular_mean_deg([0.0, 180.0])
    assert cancelled_angle is None
    assert cancelled_r == pytest.approx(0.0, abs=1e-9)

    track = DoaTrack()
    track.on_vad(True, 0.0)
    track.on_direction(80.0, 1.0)
    track.on_direction(90.0, 1.1)
    track.on_direction(100.0, 1.2)
    assert track.during(0.0, 2.0) == pytest.approx(90.0, abs=0.1)

    track.on_direction(10.0, 3.0)
    track.on_direction(190.0, 3.1)
    assert track.during(2.5, 4.0) is None, "разброс ~180° — направление не отдаём, потребитель не крутится"
