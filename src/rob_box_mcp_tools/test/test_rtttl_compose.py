"""Tests for ``core.rtttl_compose`` — RTTTL → flat ``compose_music`` params."""

from __future__ import annotations

from rob_box_mcp_tools.core.rtttl_compose import (
    detect_key,
    melody_to_compose_params,
    rtttl_to_melody,
)


def test_rtttl_to_melody_parses_bpm_and_notes():
    melody = rtttl_to_melody("fifth:d=4,o=5,b=63:8p,8g5,8g5,8g5,2d#5")
    assert melody.bpm == 63
    assert melody.notes == (
        (None, 0.5),
        (79, 0.5),
        (79, 0.5),
        (79, 0.5),
        (75, 2.0),
    )


def test_melody_to_compose_params_fills_bpm_midi_dur():
    melody = rtttl_to_melody("fifth:d=4,o=5,b=63:8p,8g5,8g5,8g5,2d#5")
    params = melody_to_compose_params(melody)
    assert params["bpm"] == 63
    assert params["lead_midi"] == "None, 79, 79, 79, 75"
    assert params["lead_dur"] == "0.5, 0.5, 0.5, 0.5, 2"
    assert params["lead_synth"] == "pluck"
    # root/scale определяются по нотам — валидные значения для аранжировщика.
    assert params["root"] in ("C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B")
    assert params["scale"]


def test_detect_key_picks_major_for_diatonic_c_major():
    # C, D, E, F, G, A, B — чистая C-dur гамма.
    midi = [60, 62, 64, 65, 67, 69, 71]
    assert detect_key(midi) == ("C", "major")


def test_detect_key_picks_harmonic_minor_when_leading_tone_present():
    # A, B, C, D, E, F, G# — гармонический ля-минор. Натуральный ля-минор и
    # C-dur неразличимы по набору нот (относительные тональности), поэтому
    # «минорность» здесь доказывает именно повышенная VII ступень (G#).
    midi = [69, 71, 72, 74, 76, 77, 80]
    root, scale = detect_key(midi)
    assert root == "A"
    assert scale == "harmonicMinor"


def test_detect_key_ignores_rests_and_defaults_without_notes():
    assert detect_key([None, None]) == ("C", "major")


def test_known_melody_roundtrip_has_matching_midi_and_dur_lengths():
    """lead_midi и lead_dur обязаны иметь одинаковую длину — аранжировщик
    проверяет это и играет ноту в ноту."""
    melody = rtttl_to_melody("imperial:d=4,o=5,b=100:8g5,8g5,8g5,8d#6,16a#5,8g5,8d#6,16a#5,8g5")
    params = melody_to_compose_params(melody)
    midi_tokens = params["lead_midi"].split(",")
    dur_tokens = params["lead_dur"].split(",")
    assert len(midi_tokens) == len(dur_tokens) == len(melody.notes)
