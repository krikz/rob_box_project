"""Тесты клубного режима ``core.club_arranger`` (план §5 п.2, §7.2, §7.6, §7.11).

Снимок кода для seed=0 лежит в ``fixtures/club_arranger_seed0.foxdot``;
осознанное изменение звучания — перегенерировать и приложить diff к PR::

    python src/rob_box_mcp_tools/test/test_club_arranger.py --regen
"""

from __future__ import annotations

import ast
import re
import sys
from pathlib import Path

import pytest

from rob_box_mcp_tools.core.club_arranger import (
    CLUB_SYNTHS,
    KICK_PATTERNS,
    LAYER_LEVELS,
    LEAD_TOP_LIMIT,
    MAX_LAYER_AMP,
    PENTATONIC,
    PROGRESSIONS,
    PUMP_HIGH,
    PUMP_LOW,
    build_matrix,
    chord_pentatonic,
    kick_steps,
    peak_levels,
    predrop_hpf,
    predrop_runs,
    pump_weights,
    render_club,
)
from rob_box_mcp_tools.core.arranger import VALID_ROOTS
from rob_box_mcp_tools.core.arrangement_matrix import SECTION_TEMPLATES
from rob_box_mcp_tools.core.renardo_sanitizer import _play_steps, sanitize_renando

SNAPSHOT = Path(__file__).parent / "fixtures" / "club_arranger_seed0.foxdot"
GATED = {"d1": "kick", "d2": "hats", "d3": "clap", "p1": "lead", "p2": "bass", "p3": "pad"}


def _player_block(code: str, slot: str) -> str:
    start = code.index(f"{slot} >> ")
    nxt = re.search(r"^(?:[dp]\d >> |Clock\.future)", code[start + 1:], re.MULTILINE)
    return code[start:start + 1 + nxt.start()] if nxt else code[start:]


def _lead_notes(code: str):
    block = _player_block(code, "p1")
    body = block[block.index("pluck([") + len("pluck(["):block.index("],")]
    return [int(v) for v in body.replace("\n", " ").split(",")]


def _var_durations(expr: str):
    m = re.fullmatch(r"var\(\[[^\]]*\], \[([^\]]*)\]\)", expr)
    assert m, expr
    return [float(v) for v in m.group(1).split(",")]


def _amp_expr(block: str) -> str:
    m = re.search(r"\bamp=(var\([^)]*\)|[\d.]+)", block)
    assert m, block
    return m.group(1)


ALL_CASES = [
    dict(template=t, kick=k, seed=s)
    for t in SECTION_TEMPLATES
    for k in KICK_PATTERNS
    for s in (0, 1, 7)
]


@pytest.mark.parametrize("case", ALL_CASES, ids=lambda c: f"{c['template']}-{c['kick']}-{c['seed']}")
def test_passes_sanitizer_unchanged(case):
    code = render_club(**case)
    result = sanitize_renando(code, 0.85, known_synths=CLUB_SYNTHS)
    assert result.security_error is None
    assert result.quality_errors == ()
    assert result.slot_error is None
    assert result.warnings == ()
    assert result.code == code


def test_club_synths_are_critical():
    """Синты club есть в ``CRITICAL_SYNTHS`` (читаем AST: tools тянет rclpy)."""
    music_py = Path(__file__).resolve().parents[1] / "rob_box_mcp_tools" / "tools" / "music.py"
    tree = ast.parse(music_py.read_text(encoding="utf-8"))
    critical = next(
        ast.literal_eval(node.value)
        for node in tree.body
        if isinstance(node, ast.AnnAssign) and getattr(node.target, "id", "") == "CRITICAL_SYNTHS"
    )
    assert CLUB_SYNTHS <= set(critical)


def test_program_frame():
    code = render_club()
    body = [ln for ln in code.splitlines() if ln and not ln.startswith("#")]
    assert body[0] == "Clock.clear()"
    assert body[1] == "Clock.bpm = 124"
    assert body[-1] == "Clock.future(128, Clock.clear)"
    assert "Clock.future" not in render_club(repeat=True)
    assert set(re.findall(r"^([dp]\d) >> ", code, re.MULTILINE)) == set(GATED)


@pytest.mark.parametrize("kick", sorted(KICK_PATTERNS))
def test_every_play_pattern_is_one_bar_of_sixteenths(kick):
    code = render_club(kick=kick)
    lines = [ln for ln in code.splitlines() if " >> play(" in ln]
    assert len(lines) == 3
    for line in lines:
        pattern = re.search(r'play\("([^"]*)"', line).group(1)
        assert len(_play_steps(pattern)) == 16, pattern
        # renardo_lib: дефолт dur у play() = 0.5 → без явного 1/4 рисунок
        # растягивается на 2 такта и пампинг промахивается мимо бочки.
        assert "dur=1/4" in line


@pytest.mark.parametrize("kick", sorted(KICK_PATTERNS))
def test_pump_dips_exactly_on_kick_steps(kick):
    code = render_club(kick=kick)
    pattern = re.search(r'd1 >> play\("([^"]*)"', code).group(1)
    hits = {i for i, step in enumerate(_play_steps(pattern)) if "X" in step}
    assert hits == set(kick_steps(pattern))
    for slot in ("p1", "p2"):
        block = _player_block(code, slot)
        assert "dur=1/4" in block
        values = [float(v) for v in re.search(r"amplify=\[([^\]]*)\]", block).group(1).split(",")]
        assert len(values) == 16
        assert {i for i, v in enumerate(values) if v == min(values)} == hits, slot
        assert max(values) / min(values) == pytest.approx(3.0)
        assert max(values) <= MAX_LAYER_AMP


def test_by_design_kick_steps_match_reference():
    assert kick_steps(KICK_PATTERNS["by_design"]) == [0, 3, 6, 9, 10]
    assert pump_weights(KICK_PATTERNS["four_on_floor"]).count(PUMP_LOW) == 4


@pytest.mark.parametrize("template", sorted(SECTION_TEMPLATES))
def test_gate_durations_sum_to_matrix_length(template):
    code = render_club(template=template)
    matrix = build_matrix(template)
    for slot, lane in GATED.items():
        expr = _amp_expr(_player_block(code, slot))
        assert expr == matrix.gate_var(lane, LAYER_LEVELS[lane])
        if expr.startswith("var("):
            assert sum(_var_durations(expr)) == pytest.approx(matrix.total_beats), slot
    assert f"Clock.future({int(matrix.total_beats)}, Clock.clear)" in code


def test_levels_below_cap_and_gain_budget():
    assert all(0 < v <= MAX_LAYER_AMP for v in LAYER_LEVELS.values())
    peaks = peak_levels()
    assert peaks["bass"] == pytest.approx(LAYER_LEVELS["bass"] * PUMP_HIGH)
    assert sum(peaks.values()) <= 1.6 + 1e-9


def test_deterministic():
    assert render_club(seed=5) == render_club(seed=5)
    assert render_club(root="C", kick="four_on_floor", seed=3) == render_club(
        root="C", kick="four_on_floor", seed=3
    )


def test_different_seeds_give_different_riffs():
    riffs = {tuple(_lead_notes(render_club(seed=s))) for s in range(10)}
    assert len(riffs) >= 8


@pytest.mark.parametrize("root", VALID_ROOTS)
@pytest.mark.parametrize("seed", range(6))
def test_registers(root, seed):
    code = render_club(root=root, seed=seed)
    notes = _lead_notes(code)
    assert len(notes) == 128  # 4 аккорда × 2 такта × 16
    assert max(notes) <= LEAD_TOP_LIMIT
    assert max(notes) - min(notes) <= 24
    bass = [int(v) for v in re.findall(r"\[\((\d+), \d+\)\] \* 32", code)]
    assert len(bass) == 4 and all(34 <= n <= 45 for n in bass)
    pads = re.search(r"sinepad\(\[(.*?)\], dur=8", code).group(1)
    pad_notes = [int(v) for v in re.findall(r"\d+", pads)]
    assert all(50 <= n <= 70 for n in pad_notes)


@pytest.mark.parametrize("root", ["A#", "C", "F#"])
@pytest.mark.parametrize("seed", range(10))
def test_lead_follows_current_chord_pentatonic(root, seed):
    """Каждая нота лида — в пентатонике аккорда, под которым она звучит (§7.11)."""
    code = render_club(root=root, seed=seed)
    prog_name = re.search(r"^# club: [^,]+, [^,]+, ([^,]+),", code, re.MULTILINE).group(1)
    chords = dict(PROGRESSIONS)[prog_name]
    tonic = VALID_ROOTS.index(root)
    notes = _lead_notes(code)
    for idx, chord in enumerate(chords):
        allowed = {(tonic + chord[0] + i) % 12 for i in PENTATONIC[chord[1]]}
        chunk = notes[idx * 32:(idx + 1) * 32]
        assert {n % 12 for n in chunk} <= allowed, (prog_name, chord)
        assert set(chunk) <= set(chord_pentatonic(tonic, chord))
    # и бас играет корень того же аккорда
    bass = [int(v) for v in re.findall(r"\[\((\d+), \d+\)\] \* 32", code)]
    assert [n % 12 for n in bass] == [(tonic + c[0]) % 12 for c in chords]


def test_predrop_hpf_on_kick_drop():
    matrix = build_matrix("dj_dave_32")
    # бочка "0 1 x x x x 14 0 ..." → пред-дроп — блоки 6-7 (доли 48..64)
    assert predrop_runs(matrix) == [(6, 8)]
    assert predrop_hpf(matrix) == "linvar([0, 0, 1500, 0], [48, 16, 0, 64])"
    assert "hpf=linvar([0, 0, 1500, 0], [48, 16, 0, 64])" in _player_block(render_club(), "p1")


def test_predrop_hpf_durations_cover_form():
    for template in SECTION_TEMPLATES:
        matrix = build_matrix(template)
        expr = predrop_hpf(matrix)
        if not expr:
            continue
        durs = [float(v) for v in re.search(r"\], \[([^\]]*)\]\)$", expr).group(1).split(",")]
        assert sum(durs) == pytest.approx(matrix.total_beats)


@pytest.mark.parametrize(
    "kwargs, message",
    [
        (dict(root="H"), "тоника"),
        (dict(scale="major"), "Лад"),
        (dict(template="nope"), "шаблон"),
        (dict(kick="nope"), "бочки"),
        (dict(bpm=500), "bpm"),
        (dict(bpm="fast"), "bpm"),
    ],
)
def test_validation_errors(kwargs, message):
    with pytest.raises(ValueError, match=message):
        render_club(**kwargs)


def test_seed0_snapshot():
    assert render_club(seed=0) == SNAPSHOT.read_text(encoding="utf-8")


if __name__ == "__main__" and "--regen" in sys.argv:
    SNAPSHOT.write_text(render_club(seed=0), encoding="utf-8")
    print(f"записан {SNAPSHOT}")
