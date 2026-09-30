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

from rob_box_mcp_tools.core import sample_dave
from rob_box_mcp_tools.core.renardo_sanitizer import (
    _fix_pattern_length,
    _play_steps,
    sanitize_renando,
)

REFERENCE = (
    Path(__file__).resolve().parents[1]
    / "rob_box_mcp_tools" / "core" / "reference_tracks" / "by_design_dj_dave.foxdot"
)
ARRAY_REFERENCE = REFERENCE.with_name("array_dj_dave.foxdot")
SYNTHS = frozenset({"pluck", "bass", "loop"})


def _code(path: Path = REFERENCE) -> str:
    return path.read_text(encoding="utf-8")


def _with_resolved_samples(code: str) -> str:
    """Что санитайзер обязан сделать с именами сэмплов пака DJ_Dave (#3219)."""
    for info in sample_dave.sample_catalog().values():
        code = code.replace(f"loop('{info.name}'", f"loop({info.path!r}")
    return code


def test_reference_passes_sanitizer_unchanged():
    """Ни слова не меняется, кроме имён сэмплов → пути до файла."""
    result = sanitize_renando(_code(), 0.85, known_synths=SYNTHS, pack1_loops_enabled=True)
    assert result.security_error is None
    assert result.quality_errors == ()
    assert result.slot_error is None
    assert result.code == _with_resolved_samples(_code())


def test_reference_uses_dj_dave_samples_from_the_catalog():
    code = _code()
    assert "loop('algorave_spilltab'" in code
    assert "loop('dirt_psr_10'" in code
    assert sample_dave.find_sample("algorave_spilltab") and sample_dave.find_sample("dirt_psr_10")


def test_reference_is_blocked_honestly_while_pack_flag_is_off():
    result = sanitize_renando(_code(), 0.85, known_synths=SYNTHS)
    assert result.quality_errors, "без флага ROB_BOX_PACK1_LOOPS трек с сэмплами играть нельзя"


def test_array_reference_passes_sanitizer_and_fits_six_slots():
    code = _code(ARRAY_REFERENCE)
    result = sanitize_renando(code, 0.85, known_synths=SYNTHS, pack1_loops_enabled=True)
    assert result.security_error is None
    assert result.quality_errors == ()
    assert result.slot_error is None
    assert result.code == _with_resolved_samples(code)
    slots = set(re.findall(r"([dps]\d) >>", code))
    assert slots == {"d1", "d2", "d3", "p1", "p2", "p3"}


def test_array_reference_follows_the_arrangement_of_the_original():
    """arrange([16,build],[2,predrop],[16,drop],[16,verse],[4,postverse]), 140 bpm."""
    code = _code(ARRAY_REFERENCE)
    assert "Clock.bpm = 140" in code
    bar = 4
    build, predrop, drop, verse, post = (16 * bar, 2 * bar, 16 * bar, 16 * bar, 4 * bar)
    starts = {"predrop": build, "drop": build + predrop}
    starts["verse"] = starts["drop"] + drop
    starts["post"] = starts["verse"] + verse
    end = starts["post"] + post
    assert (starts["drop"], starts["verse"], starts["post"], end) == (72, 136, 200, 216)
    times = [int(t) for t in re.findall(r"Clock\.future\((\d+),", code)]
    assert max(times) == end and min(times) >= 16
    # каждый файл секции запускается ровно в её границу
    assert f"Clock.future({starts['predrop']}, lambda: p3 >> loop('array_vox_drop_0'" in code
    assert f"Clock.future({starts['drop']}, lambda: p3 >> loop('array_vox_drop_1'" in code
    assert f"Clock.future({starts['verse']}, lambda: p3 >> loop('array_vox_verse_0'" in code
    assert f"Clock.future({starts['post']}, lambda: p3 >> loop('array_vox_verse_8'" in code


def test_array_reference_uses_only_catalog_samples_and_documented_gaps():
    code = _code(ARRAY_REFERENCE)
    for sample in re.findall(r"loop\('([^']+)'", code):
        assert sample_dave.find_sample(sample), sample
    assert "@license CC BY-NC-SA (code)" in code
    assert "НЕ перенесено" in code, "нехватка шести слотов должна быть записана в шапке"


def test_every_drum_pattern_is_one_bar_of_sixteenths():
    code_lines = "\n".join(line for line in _code().splitlines() if not line.startswith("#"))
    patterns = re.findall(r'play\("([^"]*)"', code_lines)
    assert len(patterns) == 2
    for pattern in patterns:
        assert len(_play_steps(pattern)) == 16, pattern
    # По умолчанию у play() dur=0.5 (renardo_lib Players.py:602): без явного
    # dur=1/4 16 шагов длились бы 2 такта, а бас/арп с dur=1/4 — один, и
    # просадки сайдчейна не совпали бы с бочкой.
    play_lines = [line for line in code_lines.splitlines() if "play(" in line]
    assert len(play_lines) == 2
    for line in play_lines:
        assert "dur=1/4" in line, line


def test_sidechain_dips_exactly_on_kick_steps():
    code = _code()
    kick = _play_steps(re.search(r'd1 >> play\("([^"]*)"', code).group(1))
    kick_steps = {i for i, step in enumerate(kick) if "X" in step}
    # p1/p2 и psr (d3, 16 ударов на такт с sidechain-постгейном оригинала)
    for player in ("p1", "p2", "d3"):
        block = code[code.index(f"{player} >>"):]
        amps = [float(v) for v in re.search(r"amp=P\[([^\]]*)\]", block).group(1).split(",")]
        assert len(amps) == 16
        dips = {i for i, v in enumerate(amps) if v == min(amps)}
        assert dips == kick_steps, player
        assert max(amps) / min(amps) == pytest.approx(3.0)


def test_spilltab_is_32_sequential_beat_chops_of_an_8_bar_file():
    """loopAt(8).chop(32): 32 куска по доле; 19.2 с = 32 доли при 100 bpm."""
    code = _code()
    positions = re.search(r"algorave_spilltab', P\[([^\]]*)\]", code).group(1)
    assert [int(v) for v in positions.split(",")] == list(range(32))
    assert "dur=1," in code[code.index("algorave_spilltab"):]
    assert sample_dave.find_sample("algorave_spilltab").seconds == pytest.approx(32 * 60 / 100)


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
