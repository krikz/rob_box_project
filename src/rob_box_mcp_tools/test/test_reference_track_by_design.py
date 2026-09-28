"""Эталон DJ-качества «By Design [DJ_Dave Edit]» и счёт шагов play().

Перекладка лежит в ``core/reference_tracks/by_design_dj_dave.foxdot``.
Тест держит два инварианта:

* эталон проходит боевой санитайзер БЕЗ изменений — иначе на роботе звучит
  не то, что лежит в репозитории;
* ``_fix_pattern_length`` считает шаги, а не символы: группа ``(.X)`` —
  один шаг. До фикса ``"X..X..X..(.X)X....."`` (16 шагов) обрезался до 13
  и бочка уплывала относительно такта.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from rob_box_mcp_tools.core.renardo_sanitizer import (
    _fix_pattern_length,
    _play_steps,
    sanitize_renando,
)

REFERENCE = (
    Path(__file__).resolve().parents[1]
    / "rob_box_mcp_tools" / "core" / "reference_tracks" / "by_design_dj_dave.foxdot"
)
SYNTHS = frozenset({"pluck", "bass", "sinepad"})


def _code() -> str:
    return REFERENCE.read_text(encoding="utf-8")


def test_reference_passes_sanitizer_unchanged():
    result = sanitize_renando(_code(), 0.85, known_synths=SYNTHS)
    assert result.security_error is None
    assert result.quality_errors == ()
    assert result.slot_error is None
    assert result.code == _code()


def test_every_drum_pattern_is_one_bar_of_sixteenths():
    code_lines = "\n".join(line for line in _code().splitlines() if not line.startswith("#"))
    patterns = re.findall(r'play\("([^"]*)"', code_lines)
    assert len(patterns) == 3
    for pattern in patterns:
        assert len(_play_steps(pattern)) == 16, pattern


def test_sidechain_dips_exactly_on_kick_steps():
    code = _code()
    kick = _play_steps(re.search(r'd1 >> play\("([^"]*)"', code).group(1))
    kick_steps = {i for i, step in enumerate(kick) if "X" in step}
    for player in ("p1", "p2"):
        block = code[code.index(f"{player} >>"):]
        amps = [float(v) for v in re.search(r"amp=P\[([^\]]*)\]", block).group(1).split(",")]
        assert len(amps) == 16
        dips = {i for i, v in enumerate(amps) if v == min(amps)}
        assert dips == kick_steps, player
        assert max(amps) / min(amps) == pytest.approx(3.0)


@pytest.mark.parametrize(
    "pattern, expected",
    [
        ("X..X..X..(.X)X.....", "X..X..X..(.X)X....."),
        ("---(-.)(-.)----------(-.)", "---(-.)(-.)----------(-.)"),
        ("[xx]o", "[xx]o"),
        ("|x2|.o", "|x2|.o."),
        ("x(o.)", "x(o.)"),
        ("x.o", "x.o."),
        ("x..o..", "x..o"),
        # слои и битые скобки не разбираем — не трогаем
        ("<x.o.><-->", "<x.o.><-->"),
        ("(x.", "(x."),
    ],
)
def test_fix_pattern_length_counts_steps_not_chars(pattern, expected):
    assert _fix_pattern_length(f'play("{pattern}")') == f'play("{expected}")'
