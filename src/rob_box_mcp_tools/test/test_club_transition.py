"""Issue #3113 — переход между клубными треками фейдом (``core.club_transition``).

Renardo здесь не поднимается: семантика обёртки проверяется ``exec`` с
подставными ``Clock``/``Master``/``linvar``, которые записывают, что код с
ними сделал. Как это звучит на роботе — НЕ проверено (нужен живой прогон).
"""

from __future__ import annotations

import re

import pytest

from rob_box_mcp_tools.core.arrangement_matrix import SECTION_TEMPLATES
from rob_box_mcp_tools.core.club_arranger import CLUB_SYNTHS, KICK_PATTERNS, render_club
from rob_box_mcp_tools.core.club_transition import (
    FADE_BARS,
    NEXT_TRACK_FN,
    fade_seconds,
    wrap_with_fade,
)
from rob_box_mcp_tools.core.renardo_sanitizer import sanitize_renando

CASES = [
    dict(template=t, kick=k, seed=s, repeat=r)
    for t in SECTION_TEMPLATES
    for k in KICK_PATTERNS
    for s in (0, 7)
    for r in (False, True)
]


@pytest.mark.parametrize("case", CASES, ids=lambda c: f"{c['template']}-{c['kick']}-{c['seed']}-{c['repeat']}")
def test_wrapped_club_passes_sanitizer_unchanged(case):
    """Санитайзер не ослаблен и ничего в обёртке не переписывает."""
    code = wrap_with_fade(render_club(**case))
    result = sanitize_renando(code, 0.85, known_synths=CLUB_SYNTHS)
    assert result.security_error is None
    assert result.quality_errors == ()
    assert result.slot_error is None
    assert result.warnings == ()
    assert result.code == code


def test_wrapped_club_uses_only_six_slots():
    code = wrap_with_fade(render_club(seed=3))
    slots = set(re.findall(r"^\s*([dpsl]\d+)\s*>>", code, re.MULTILINE))
    assert slots == {"d1", "d2", "d3", "p1", "p2", "p3"}


def test_new_track_body_is_the_original_program():
    program = render_club(seed=5)
    code = wrap_with_fade(program)
    body = code.split(f"def {NEXT_TRACK_FN}():\n", 1)[1].split("\n\nif Clock.playing:", 1)[0]
    unindented = "\n".join(line[4:] if line.startswith("    ") else line for line in body.splitlines())
    assert unindented == program.rstrip("\n")


class _Recorder:
    def __init__(self):
        self.calls = []


class _FakeClock:
    def __init__(self, playing, beat=10.0):
        self.playing = list(playing)
        self.beat = beat
        self.bpm = 124
        self.scheduled = []
        self.cleared = 0

    def now(self):
        return self.beat

    def next_bar(self):
        return self.beat + (4 - self.beat % 4)

    def schedule(self, obj, beat=None, args=(), kwargs={}):  # noqa: B006
        self.scheduled.append((obj, beat))

    def clear(self):
        self.cleared += 1
        self.playing = []


class _FakeGroup:
    def __init__(self, sink):
        object.__setattr__(self, "_sink", sink)

    def __setattr__(self, name, value):
        self._sink.append((name, value))


def _namespace(clock, sink):
    return {
        "Clock": clock,
        "Master": lambda: _FakeGroup(sink),
        "linvar": lambda values, dur, start=0: ("linvar", tuple(values), tuple(dur), start),
    }


PROGRAM = "Clock.clear()\nClock.bpm = 124\nmarker = Clock.cleared\n"


def test_playing_track_fades_then_new_track_on_bar():
    clock = _FakeClock(playing=["old_player"], beat=10.0)
    sink = []
    ns = _namespace(clock, sink)
    exec(wrap_with_fade(PROGRAM), ns)  # noqa: S102
    beats = FADE_BARS * 4
    assert sink == [
        ("lpf", ("linvar", (4000, 300, 300), (beats, beats, beats), 10.0)),
        ("amplify", ("linvar", (1, 0, 0), (beats, beats, beats), 10.0)),
    ]
    # Новый трек ещё не стартовал: старые плееры играют и гаснут.
    assert clock.cleared == 0 and "marker" not in ns
    assert len(clock.scheduled) == 1
    fn, beat = clock.scheduled[0]
    assert beat == 12.0 + beats  # граница такта + фейд
    fn()
    assert clock.cleared == 1
    # Имена верхнего уровня программы остаются глобальными.
    assert ns["marker"] == 1


def test_nothing_playing_starts_new_track_immediately():
    clock = _FakeClock(playing=[])
    sink = []
    ns = _namespace(clock, sink)
    exec(wrap_with_fade(PROGRAM), ns)  # noqa: S102
    assert sink == [] and clock.scheduled == []
    assert clock.cleared == 1 and ns["marker"] == 1


@pytest.mark.parametrize("bars", [0, 17, 2.5, True])
def test_bad_fade_bars_rejected(bars):
    with pytest.raises(ValueError):
        wrap_with_fade(PROGRAM, fade_bars=bars)


def test_multiline_string_literal_rejected():
    with pytest.raises(ValueError, match="многострочный"):
        wrap_with_fade('x = """a\nb"""\n')


def test_unparseable_program_rejected():
    with pytest.raises(ValueError, match="не парсится"):
        wrap_with_fade("d1 >> play(\n")


def test_fade_seconds_upper_bound():
    # 8 тактов фейда + до 1 такта до границы, 124 BPM
    assert fade_seconds(124) == pytest.approx(9 * 4 * 60 / 124)
