"""#3580: нота без октавы в мелодии с явными октавами («Celebrat») — скачок в хуке; санитайзер ``fix_octaves`` и
штраф ``hook_defect`` в очереди хуков."""

from __future__ import annotations

import logging
import random

from melodies import LONG, SHORT, profile
from rob_box_music import knowledge as kn
from rob_box_music.arrange import hook as hooks
from rob_box_music.arrange.compose import hook_order
from rob_box_music.rtttl import fix_octaves, parse_rtttl

#: Живая запись архива (picaxe:mixed3): ``32d_`` и ``c`` без октавы при ``o=6`` — на октаву выше соседей с «5».
CELEBRAT = ("Celebrat:d=4,o=6,b=225:a_5,a5,a5,g_5,g5,f_5,f5,e5,e5,8f5,p,8g5,p,2c.5,32d_,e5,32c_5,32d5,32d_5,8e5,"
            "16d_5,32d5,32c_5,8a5,32p,8a.5,16p,8a_5,32p,8a_5,32p,a_5,a5,a5,a_5,8a_5,8a5,p,2c.5,c,8c5,8c5")
FUR_ELISE = ("FurElise:d=8,o=5,b=125:32p,e6,d#6,e6,d#6,e6,b,d6,c6,4a.,32p,c,e,a,4b.,32p,e,g#,b,4c.6,32p,e,e6,d#6,e6,"
             "d#6,e6,b,d6,c6,4a.,32p,c,e,a,4b.,32p,d,c6,b,2a")
TETRIS = ("Tetris:d=4,o=5,b=160:e6,8b,8c6,8d6,16e6,16d6,8c6,8b,a,8a,8c6,e6,8d6,8c6,b.,8c6,d6,e6,c6,a,2a,8p,d6,8f6,"
          "a6,8g6,8f6,e.6,8c6,e6,8d6,8c6,b,8b,8c6,d6,e6,c6,a,2a")


def _sounding(rtttl):
    return [m for m, _d in parse_rtttl(rtttl)[2] if m is not None]


def _max_leap(pitches):
    return max(abs(b - a) for a, b in zip(pitches, pitches[1:]))


def test_celebrat_octave_gap_is_closed_and_nothing_else_changes(caplog):
    before = _sounding(CELEBRAT)
    assert 87 in before and 84 in before  # D#6 и C6 (o=6 по умолчанию) посреди линии на октаву ниже
    with caplog.at_level(logging.INFO, logger="rob_box_music.rtttl"):
        fixed = fix_octaves(CELEBRAT, "celebrat")
    after = _sounding(fixed)
    assert [(a, b) for a, b in zip(before, after) if a != b] == [(87, 75), (84, 72)]  # D#, C: октава соседей (5)
    assert 87 not in after and 84 not in after and _max_leap(after) < _max_leap(before)
    assert [d for _m, d in parse_rtttl(fixed)[2]] == [d for _m, d in parse_rtttl(CELEBRAT)[2]]  # ритм тот же
    assert "исправил октаву 2 нот" in caplog.text and "celebrat" in caplog.text


def test_the_note_between_neighbours_takes_the_octave_of_the_previous_one():
    fixed = fix_octaves("t:d=4,o=6,b=120:e5,e5,32g_,e5,e5,e5,e5,e5,e5,e5,e5")
    assert fixed.split(":")[2].split(",")[2] == "32g_5"


def test_melody_where_default_octave_is_the_norm_is_unchanged():
    plain = "t:d=8,o=6,b=120:c,e,g,e,4c,4p,d,f,a,f,4d,4p,c,e,g,c7"
    assert fix_octaves(plain) == plain


def test_archive_hooks_without_octave_slips_are_unchanged():
    for rtttl in (FUR_ELISE, TETRIS, LONG, SHORT):
        assert fix_octaves(rtttl) == rtttl


def test_real_big_leap_between_explicit_octaves_is_not_touched():
    explicit = "t:d=4,o=5,b=120:c5,e5,g5,c7,g5,e5,c5,e5,g5,c5"
    assert fix_octaves(explicit) == explicit


def test_not_rtttl_is_returned_as_is():
    assert fix_octaves("no colons") == "no colons"
    assert fix_octaves("t:d=4,o=5,b=100:xyz") == "t:d=4,o=5,b=100:xyz"


def test_hook_is_built_from_the_corrected_melody():
    hook, _key = hooks.from_rtttl(CELEBRAT, "celebrat", 125, 0, "major")
    assert _max_leap([ev.midi for ev in hook.notes]) <= 12


def test_defect_counts_returning_leaps_and_notes_shorter_than_a_sixteenth():
    assert hooks.hook_defect(FUR_ELISE, 125) == 0.0
    assert hooks.hook_defect(LONG, 130) == 0.0
    spike = "t:d=8,o=5,b=120:c5,e5,g5,c7,g5,e5,c5,e5,g5,c5"
    assert hooks.hook_defect(spike, 120) == 0.1  # один выброс из десяти нот
    fast = "t:d=32,o=5,b=120:c5,e5,g5,e5"
    assert hooks.hook_defect(fast, 120) == 1.0  # 32-е = 1/8 доли < 1/16 сетки при темпе трека
    assert 0.0 < hooks.hook_defect(CELEBRAT, 125) < 0.5
    assert hooks.hook_defect("not rtttl", 125) == 0.0


def test_defective_hook_goes_later_in_the_queue_and_clean_order_is_kept():
    melodies = {"celeb": CELEBRAT, "long": LONG, "short": SHORT}
    prof = profile(hooks=("celeb", "long", "short"), bpm=125)
    assert hook_order(prof, melodies, random.Random(0), opening=True) == ["long", "celeb", "short"]
    assert kn.HOOK_DEFECT_PLACES * hooks.hook_defect(CELEBRAT, 125) > 1  # сдвиг больше одного места
    clean = profile(hooks=("long", "short"), bpm=125)
    assert hook_order(clean, melodies, random.Random(0), opening=True) == ["long", "short"]
