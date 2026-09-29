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
    FADE_AMPLIFY_TO,
    FADE_BARS,
    NEXT_TRACK_FN,
    SWITCH_EARLY_BEATS,
    TRACK_STARTED_FN,
    fade_seconds,
    is_fade_wrapped,
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


@pytest.mark.parametrize("dj_entry", [False, True], ids=["plain", "dj_entry"])
@pytest.mark.parametrize("case", CASES, ids=lambda c: f"{c['template']}-{c['kick']}-{c['seed']}-{c['repeat']}")
def test_wrapped_club_passes_sanitizer_unchanged(case, dj_entry):
    """Санитайзер не ослаблен и ничего в обёртке не переписывает (и в #3166-входе)."""
    code = wrap_with_fade(render_club(**case, align_clock=dj_entry, dj_entry=dj_entry), form_beats=128)
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


def test_new_track_body_is_the_original_program_then_started_callback():
    program = render_club(seed=5)
    code = wrap_with_fade(program, form_beats=128, entry_beats=16)
    body = code.split(f"def {NEXT_TRACK_FN}():\n", 1)[1].split("\n\nif Clock.playing:", 1)[0]
    unindented = "\n".join(line[4:] if line.startswith("    ") else line for line in body.splitlines())
    head, _, last = unindented.rpartition("\n")
    assert head == program.rstrip("\n")
    # ADR-0142 §10.1: колбэк — ПОСЛЕДНЕЙ строкой, после плееров нового трека.
    assert last == f"{TRACK_STARTED_FN}(128, 16)"
    assert is_fade_wrapped(code) and not is_fade_wrapped(program)


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


def _namespace(clock, sink, started=None):
    return {
        "Clock": clock,
        "Master": lambda: _FakeGroup(sink),
        "linvar": lambda values, dur, start=0: ("linvar", tuple(values), tuple(dur), start),
        TRACK_STARTED_FN: lambda *args: (started if started is not None else []).append((clock.cleared, args)),
    }


PROGRAM = "Clock.clear()\nClock.bpm = 124\nmarker = Clock.cleared\n"


def test_playing_track_fades_then_new_track_at_fade_end():
    """Issue #3166: фейд кончается РОВНО в долю старта нового трека (не
    раньше, как было: linvar от now() до нуля, а старт — от next_bar())."""
    clock = _FakeClock(playing=["old_player"], beat=10.0)
    sink, started = [], []
    ns = _namespace(clock, sink, started)
    exec(wrap_with_fade(PROGRAM, form_beats=128), ns)  # noqa: S102
    beats = FADE_BARS * 4
    switch = 12.0 + beats - SWITCH_EARLY_BEATS
    fade = switch - 10.0
    assert sink == [
        ("lpf", ("linvar", (4000, 300, 300), (fade, fade, fade), 10.0)),
        ("amplify", ("linvar", (1, FADE_AMPLIFY_TO, FADE_AMPLIFY_TO), (fade, fade, fade), 10.0)),
    ]
    # Новый трек ещё не стартовал: старые плееры играют и гаснут.
    assert clock.cleared == 0 and "marker" not in ns and started == []
    assert len(clock.scheduled) == 1
    fn, beat = clock.scheduled[0]
    # Старт — в долю конца фейда: start(10) + fade == switch, без лишнего такта.
    assert beat == switch == 10.0 + fade
    fn()
    assert clock.cleared == 1
    # Имена верхнего уровня программы остаются глобальными.
    assert ns["marker"] == 1
    # Колбэк «started» — после Clock.clear нового трека, один раз.
    assert started == [(1, (128, 0))]


def test_switch_callback_precedes_bar_grid_of_outgoing_players():
    """Issue #3166: колбэк НЕ в одном QueueBlock с событиями уходящих плееров
    на границе такта (иначе ``d1`` был бы вызван дважды), но и не раньше
    ближайшего события сетки 1/8 доли (группы ``(.X)`` при ``dur=1/4``)."""
    assert 0 < SWITCH_EARLY_BEATS < 1 / 8
    clock = _FakeClock(playing=["old"], beat=13.5)
    exec(wrap_with_fade(PROGRAM), _namespace(clock, []))  # noqa: S102
    _, beat = clock.scheduled[0]
    assert beat % 4 == pytest.approx(4 - SWITCH_EARLY_BEATS)


def test_fade_never_reaches_digital_silence():
    """Issue #3166: уходящий трек не гасится в ноль до старта нового."""
    clock = _FakeClock(playing=["old"], beat=0.0)
    sink = []
    exec(wrap_with_fade(PROGRAM), _namespace(clock, sink))  # noqa: S102
    amplify = dict(sink)["amplify"]
    values, durs = amplify[1], amplify[2]
    # до доли смены (первый сегмент) уровень идёт 1 → FADE_AMPLIFY_TO > 0
    assert values[:2] == (1, FADE_AMPLIFY_TO) and FADE_AMPLIFY_TO > 0
    assert durs[0] == clock.scheduled[0][1]


def test_new_track_amplify_back_to_one_after_clear():
    """``Clock.clear`` нового трека → ``Player.kill`` → ``reset`` (amplify=1):
    фейд вешается только на уходящие плееры и снимается очисткой ДО того,
    как программа переприсвоит те же слоты. Здесь — порядок в обёртке."""
    program = render_club(seed=2)
    code = wrap_with_fade(program)
    body = code.split(f"def {NEXT_TRACK_FN}():\n", 1)[1]
    assert body.index("Clock.clear()") < body.index("d1 >>")
    # Master().amplify — только на верхнем уровне обёртки (при exec, по
    # уходящим плеерам), а не внутри запуска нового трека.
    assert "Master()" not in body.split("\n\nif Clock.playing:", 1)[0]


def test_nothing_playing_starts_new_track_immediately():
    clock = _FakeClock(playing=[])
    sink, started = [], []
    ns = _namespace(clock, sink, started)
    exec(wrap_with_fade(PROGRAM, form_beats=64, entry_beats=8), ns)  # noqa: S102
    assert sink == [] and clock.scheduled == []
    assert clock.cleared == 1 and ns["marker"] == 1
    assert started == [(1, (64, 8))]


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
