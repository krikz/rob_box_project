"""Тесты стерео-раскладки club (issue #3310, эпик #3312)."""

from __future__ import annotations

import re

import pytest

from rob_box_mcp_tools.core.club_arranger import (
    CLUB_TEMPLATES,
    HATS_PATTERNS,
    club_kit,
    render_club,
    render_club_kit,
)
from rob_box_mcp_tools.core.club_pools import CLAP_PATTERNS, REFERENCE_VARIANT
from rob_box_mcp_tools.core.club_samples import SAMPLE_LAYERS
from rob_box_mcp_tools.core.club_stereo import PAN_HATS, PAN_PAD_WIDTH, pan_steps


def _block(code: str, slot: str) -> str:
    """Плеер вместе со строками-продолжениями (лид/бас/пэд многострочные)."""
    lines = code.splitlines()
    start = next(i for i, line in enumerate(lines) if line.startswith(f"{slot} >>"))
    out = [lines[start]]
    for line in lines[start + 1:]:
        if not line.strip() or re.match(r"\w+ >>", line):
            break
        out.append(line)
    return "\n".join(out)


def _pan_list(text: str):
    m = re.search(r"pan=\[([^\]]*)\]", text)
    return [float(v) for v in m.group(1).split(",")] if m else None


@pytest.mark.parametrize("seed", [0, 3, 7, 11])
def test_centre_parts_have_no_pan(seed):
    code = render_club(seed=seed)
    for slot in ("d1", "p1", "p2"):
        assert "pan=" not in _block(code, slot), slot


@pytest.mark.parametrize("seed", [0, 3, 7, 11])
def test_side_parts_have_pan(seed):
    code = render_club(seed=seed)
    for slot in ("d2", "d3", "p3"):
        assert "pan=" in _block(code, slot), slot


def _hit_sides(pattern: str, pan):
    """Стороны только на реальных ударах (круглая группа — один шаг-удар)."""
    steps = re.findall(r"\([^)]*\)|.", pattern)
    return [pan[i % len(pan)] for i in range(2 * len(steps)) if steps[i % len(steps)] != "."]


@pytest.mark.parametrize("hats", sorted(HATS_PATTERNS))
def test_hats_pan_balanced_on_actual_hits(hats):
    """Баланс L/R по реальным ударам хэтов за два такта, а не по всем шагам."""
    code = render_club_kit({**club_kit(5), "hats": hats}, seed=5)
    pan = _pan_list(_block(code, "d2"))
    assert len(pan) == 32
    hits = _hit_sides(HATS_PATTERNS[hats], pan)
    assert hits and abs(sum(hits)) < 1e-9, (hats, sum(hits), len(hits))


@pytest.mark.parametrize("clap", sorted(CLAP_PATTERNS))
def test_clap_pan_balanced_on_actual_hits(clap):
    code = render_club_kit(club_kit(7), seed=7, variant={"clap": clap})
    hits = _hit_sides(CLAP_PATTERNS[clap], _pan_list(_block(code, "d3")))
    assert hits and abs(sum(hits)) < 1e-9, (clap, sum(hits))


def test_pan_steps_alternate_and_start_side():
    assert pan_steps("x.x.", PAN_HATS, 1)[:4] == [PAN_HATS, PAN_HATS, -PAN_HATS, -PAN_HATS]
    assert pan_steps("x.x.", PAN_HATS, -1)[0] == -PAN_HATS
    assert max(abs(v) for v in pan_steps("xxxx", PAN_HATS)) == PAN_HATS


def test_d3_starts_on_opposite_side_to_hats():
    kit = club_kit(7)
    code = render_club_kit(kit, seed=7)
    hat = _hit_sides(HATS_PATTERNS[kit["hats"]], _pan_list(_block(code, "d2")))[0]
    clap = _hit_sides(CLAP_PATTERNS[REFERENCE_VARIANT["clap"]], _pan_list(_block(code, "d3")))[0]
    assert (hat, clap) == (PAN_HATS, -PAN_HATS)


def test_pad_pan_is_slow_symmetric_triangle():
    p3 = _block(render_club(seed=7), "p3")
    m = re.search(r"pan=linvar\(\[([^\]]*)\], \[([^\]]*)\]\)", p3)
    vals = [float(v) for v in m.group(1).split(",")]
    assert vals[0] == vals[2] == -vals[1] and abs(vals[1]) == PAN_PAD_WIDTH
    assert sum(float(v) for v in m.group(2).split(",")) == 32  # 8 тактов


@pytest.mark.parametrize("name", sorted(SAMPLE_LAYERS))
def test_sample_layer_is_panned_symmetrically(name):
    code = render_club(seed=7, sample=name)
    d3 = _block(code, "d3")
    pan = _pan_list(d3)
    assert "loop(" in d3 and pan and abs(sum(pan)) < 1e-9, d3


@pytest.mark.parametrize("template", CLUB_TEMPLATES)
def test_all_templates_render_with_pan(template):
    code = render_club(template=template, seed=3)
    assert code.count("pan=") == 3  # d2, d3, p3
