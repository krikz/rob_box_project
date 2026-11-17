"""Issue #3154 — события нот Renardo-программы без Renardo (``rob_box_music.render.events``, перенос из mcp_tools)."""

from __future__ import annotations

import pytest

from rob_box_music.render.events import (
    ProgramError,
    TimeVar,
    degree_to_midi,
    program_events,
    run_program,
)


def test_var_and_linvar_follow_the_clock():
    gate = TimeVar([0, 0.5, 0], [4, 8, 4])
    assert [gate.at(b) for b in (0, 3.9, 4, 11.9, 12, 16)] == [0, 0, 0.5, 0.5, 0, 0]
    sweep = TimeVar([700, 4500], 10, linear=True)
    assert sweep.at(5) == pytest.approx(2600)
    assert sweep.at(15) == pytest.approx(2600)


def test_degree_to_midi_matches_renardo_scale_midi():
    assert degree_to_midi(60, 0, 0, "chromatic") == 60
    assert degree_to_midi(0, 5, 9, "minor") == 69
    assert degree_to_midi(7, 4, 0, "major") == 60
    assert degree_to_midi(-1, 5, 0, "minor") == 58


def test_play_pattern_amp_times_amplify_and_sus_defaults_to_dur():
    code = (
        "Clock.bpm = 120\n"
        "d1 >> play('X.o.', dur=0.5, amp=var([0.4, 0], [2, 2]), amplify=[1, 0.5])\n"
        "Clock.future(4, Clock.clear)\n"
    )
    program, events = program_events(code)
    assert program.bpm == 120 and program.form_beats == 4
    assert [(e.beat, e.sample, e.amp, e.sus_beats) for e in events] == [
        (0.0, "X0", 0.4, 0.5), (1.0, "o0", 0.4, 0.5),
    ]


def test_chords_rests_and_scale_degrees_with_root_var():
    code = (
        'Root.default = var([0, 5], 4)\nScale.default = "major"\n'
        "p1_motif = Pvar([[0, None], [(0, 2)]], [4, 4])\n"
        "p1 >> pluck(p1_motif, dur=2, oct=5, amp=0.3)\n"
    )
    _program, events = program_events(code, form_beats=8)
    assert [(e.beat, e.midi) for e in events] == [(0.0, 60), (4.0, 65), (4.0, 69), (6.0, 65), (6.0, 69)]


def test_program_has_no_builtins():
    with pytest.raises(ProgramError):
        run_program("__import__('os').system('echo hi')")
    with pytest.raises(ProgramError):
        run_program("p1 >> pluck([0], dur=1, amp=P[1, 2])")
