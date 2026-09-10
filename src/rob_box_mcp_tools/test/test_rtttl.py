"""Tests for ``core.rtttl`` — RTTTL (Nokia ringtone) → Renardo converter."""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.core.rtttl import parse_rtttl, rtttl_to_renardo


def test_parse_simple_fifth_example():
    """Канонический пример из спеки: пауза + три G5 + D#5."""
    name, bpm, notes = parse_rtttl("fifth:d=4,o=5,b=63:8p,8g5,8g5,8g5,2d#5")
    assert name == "fifth"
    assert bpm == 63
    # 8p → rest 0.5; 8g5 → G5=79 (0.5); 2d#5 → D#5=75 (2.0)
    assert notes == [
        (None, 0.5),
        (79, 0.5),
        (79, 0.5),
        (79, 0.5),
        (75, 2.0),
    ]


def test_default_duration_and_octave_apply():
    """Токены без длительности/октавы берут значения из заголовка."""
    _name, _bpm, notes = parse_rtttl("t:d=4,o=5,b=100:c,d,e")
    # c5=72, d5=74, e5=76, все четверти.
    assert notes == [(72, 1.0), (74, 1.0), (76, 1.0)]


def test_dotted_note_is_one_and_a_half_times():
    _name, _bpm, notes = parse_rtttl("t:d=4,o=5,b=100:4c5.")
    assert notes == [(72, 1.5)]


def test_sharp_note():
    _name, _bpm, notes = parse_rtttl("t:d=4,o=5,b=100:8g#5")
    assert notes == [(80, 0.5)]


def test_underscore_sharp_picaxe_extension():
    """PICAXE-диалект: ``_`` — историческая замена ``#`` (диез)."""
    _name, _bpm, notes = parse_rtttl("t:d=4,o=5,b=100:8a_5")
    # a5 = 81, диез → 82.
    assert notes == [(82, 0.5)]


def test_arduino_dot_after_duration_variant():
    """Некоторые библиотеки пишут точку сразу после длительности: 4.f5."""
    _name, _bpm, notes = parse_rtttl("t:d=4,o=5,b=100:4.f5")
    assert notes == [(77, 1.5)]


def test_dot_before_octave_picaxe_variant():
    """PICAXE-диалект: точка до октавы (16d#.6 = D#6, dotted sixteenth)."""
    _name, _bpm, notes = parse_rtttl("t:d=4,o=5,b=100:16d#.6")
    # d#6 = 87, 16-я с точкой = 0.375 доли.
    assert notes == [(87, 0.375)]


def test_invalid_token_raises():
    with pytest.raises(ValueError):
        parse_rtttl("t:d=4,o=5,b=100:xyz")


def test_missing_colon_raises():
    with pytest.raises(ValueError):
        parse_rtttl("no colons here")


def test_zero_duration_raises():
    """d=0 или токен с нулевой длительностью — ValueError, не ZeroDivisionError."""
    with pytest.raises(ValueError):
        parse_rtttl("t:d=0,o=5,b=100:c")


def test_renardo_output_has_midinote_and_dur():
    code = rtttl_to_renardo("fifth:d=4,o=5,b=63:8p,8g5,8g5,8g5,2d#5")
    assert "Clock.clear()" in code
    assert "Clock.bpm = 63" in code
    assert "midinote=[None, 79, 79, 79, 75]" in code
    assert "dur=[0.5, 0.5, 0.5, 0.5, 2]" in code
