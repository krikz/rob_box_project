"""#3580 (решение Шифу 13.10 «давай доиграем»): build/break доигрывают фразу хука до конца, новая не наслаивается."""

from __future__ import annotations

import pytest

from rob_box_music import knowledge as kn
from rob_box_music.arrange import hook as hooks
from rob_box_music.arrange.compose import _lead
from rob_box_music.model import BEATS_PER_BAR, Hook, Key, PitchEvent

KEY = Key(0, "minor")


def _phrase_hook(bars):
    """Хук из ``bars`` тактов: нота на каждой доле, высота растёт — по высоте видно, какая нота звучит."""
    return Hook(tuple(PitchEvent(60 + i % 12, float(i), 0.5, 3) for i in range(bars * BEATS_PER_BAR)), bars, "t")


def _sounded(hook, section, bars):
    notes = list(hooks.develop(hook, section, bars, KEY)) + list(hooks.spill(hook, section, bars, KEY))
    return [e.midi for e in sorted(notes, key=lambda e: e.beat)]


@pytest.mark.parametrize("section,bars,hook_bars", [("build", 8, 4), ("build", 8, 8), ("build", 5, 8), ("build", 6, 4),
                                                    ("break", 8, 4), ("break", 8, 8), ("break", 4, 8), ("break", 4, 4)])
def test_phrase_started_in_build_or_break_sounds_to_its_end(section, bars, hook_bars):
    """Каждая начатая фраза звучит целиком, по порядку; ни одна не обрезана границей секции."""
    hook = _phrase_hook(hook_bars)
    whole = [e.midi for e in hook.notes]
    pitches = _sounded(hook, section, bars)
    assert pitches and pitches == whole * (len(pitches) // len(whole)), "фраза оборвана или перемешана"


@pytest.mark.parametrize("section", ["build", "break"])
def test_phrase_longer_than_section_spills_instead_of_cutting(section):
    hook = _phrase_hook(8)
    tail = hooks.spill(hook, section, 5, KEY)
    assert tail and min(e.beat for e in tail) >= 5 * BEATS_PER_BAR
    assert len(hooks.develop(hook, section, 5, KEY)) + len(tail) == len(hook.notes)
    assert not hooks.spill(hook, section, 8, KEY), "фраза уместилась — хвоста нет"


def test_next_section_does_not_layer_a_new_phrase_over_the_spilled_tail():
    spec = (("build", 5, 5, frozenset({"lead"})), ("drop", 8, 8, frozenset({"lead"})))
    part = _lead(kn.STYLES["club"], spec, _phrase_hook(8), KEY, "keys")
    starts = [e.beat for e in part.pitches]
    assert len(set(starts)) == len(starts), "две ноты на одной доле — фразы наложились"
    ends = sorted((e.beat, e.beat + e.dur_beats) for e in part.pitches)
    assert all(a_end <= b_start + 1e-9 for (_a, a_end), (b_start, _b) in zip(ends, ends[1:])), "ноты перекрылись"
    assert max(starts) < sum(b for _n, b, _e, _r in spec) * BEATS_PER_BAR, "хвост ушёл за конец формы"
