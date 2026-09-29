"""Issue #3113 — DJ-сет играет ``style="club"`` в одном темпе и родственных
тональностях, переход — фейдом (``transition="fade"``), а не скачком.

План: docs/design/2026-09-28-dj-live-coding-quality-plan.md §5 п.3, §7.1.
"""

from __future__ import annotations

import ast
import json
import logging
import re
from pathlib import Path

import pytest

from rob_box_voice.core.dj_mode import (
    CLUB_ROOTS,
    DJ_SET_BPM_RANGE,
    DJ_SET_DEFAULT_BPM,
    KEY_WALK,
    DJHook,
    DJModeController,
    related_root,
)

START = 1_759_000_000.0  # % 12 == 4 → тоника сета "E"


def _controller(now=START):
    clock = {"t": now}
    dispatched = []
    hook = DJHook(
        dispatch=lambda prompt, from_tick=False: dispatched.append(prompt),
        is_active=lambda: False,
        is_dialogue_active=lambda: False,
    )
    ctrl = DJModeController(hook=hook, logger=logging.getLogger("test"), clock=lambda: clock["t"])
    return ctrl, clock, dispatched


def _enable(ctrl, **extra):
    ctrl.handle_message(json.dumps({"enabled": True, "theme": "техно", **extra}))


def _club_calls(prompt):
    return re.findall(r'compose_music\(style="club", bpm=(\d+), root="([A-G]#?)", '
                      r'scale="minor", seed=(\d+), repeat=(true|false), transition="fade"\)', prompt)


def test_mid_set_prompt_has_no_other_bpm_demand_and_fixed_bpm():
    ctrl, _, _ = _controller()
    _enable(ctrl)
    ctrl.state.tracks_started = 1
    prompt = ctrl.build_auto_prompt(2)
    assert "другой" not in prompt
    assert "bpm/root/scale" not in prompt
    calls = _club_calls(prompt)
    assert len(calls) == 1
    assert calls[0][0] == str(DJ_SET_DEFAULT_BPM)
    assert calls[0][3] == "true"
    assert f"Темп сета {DJ_SET_DEFAULT_BPM} BPM" in prompt
    # Путь classic для названной юзером песни остаётся, в том же темпе.
    assert "compose_music(name=..., seed=" in prompt
    assert f"bpm={DJ_SET_DEFAULT_BPM}, repeat=true)" in prompt
    # Техническая «кухня» перехода в промпт не утекает.
    for leak in ("Clock.clear()", "Master()", "linvar"):
        assert leak not in prompt


def test_bpm_persists_across_tracks_and_changes_only_on_request():
    ctrl, clock, dispatched = _controller()
    _enable(ctrl, next_transition_sec=15)
    bpms = []
    for track in range(1, 5):
        clock["t"] += 20
        ctrl.tick()
        ctrl.state.tracks_started = track
        bpms.extend(c[0] for c in _club_calls(dispatched[-1]))
        # модель повторяет set_dj_mode на каждом переходе — без bpm
        _enable(ctrl, next_transition_sec=15)
    assert len(dispatched) == 4
    assert bpms and set(bpms) == {"124"}
    # Явная просьба юзера («давай 128») меняет темп сета.
    _enable(ctrl, bpm=128, next_transition_sec=15)
    assert ctrl.state.set_bpm == 128
    clock["t"] += 20
    ctrl.tick()
    assert len(dispatched) == 5
    assert {c[0] for c in _club_calls(dispatched[-1])} == {"128"}
    # Мусор/вне диапазона — клампится, не ломает сет.
    _enable(ctrl, bpm=999)
    assert ctrl.state.set_bpm == 180


def test_fresh_set_resets_bpm_and_root():
    ctrl, clock, _ = _controller()
    _enable(ctrl, bpm=100)
    assert ctrl.state.set_bpm == 100
    ctrl.handle_message(json.dumps({"enabled": False}))
    assert ctrl.state.set_bpm == DJ_SET_DEFAULT_BPM and ctrl.state.set_root == ""
    clock["t"] += 1
    _enable(ctrl)
    assert ctrl.state.set_bpm == DJ_SET_DEFAULT_BPM


@pytest.mark.parametrize(
    "track_no, expected",
    [(1, "E"), (2, "B"), (3, "E"), (4, "A"), (5, "E"), (6, "B")],
)
def test_related_root_walks_circle_of_fifths(track_no, expected):
    assert related_root("E", track_no) == expected


def test_related_root_stays_within_one_fifth():
    for root in CLUB_ROOTS:
        for n in range(1, 9):
            shift = (CLUB_ROOTS.index(related_root(root, n)) - CLUB_ROOTS.index(root)) % 12
            assert shift in {0, 5, 7}
    assert KEY_WALK == (0, 7, 0, 5)
    assert related_root("H", 2) == "H"  # неизвестная тоника — как есть


def test_prompt_root_follows_rule_and_seed_differs_per_track():
    ctrl, _, _ = _controller()
    _enable(ctrl)
    seen = []
    for started in range(0, 4):
        ctrl.state.tracks_started = started
        (call,) = _club_calls(ctrl.build_auto_prompt(started + 2))
        seen.append(call)
    set_root = CLUB_ROOTS[int(START) % 12]
    assert [c[1] for c in seen] == [related_root(set_root, n) for n in range(1, 5)]
    assert len({c[2] for c in seen}) == 4


def test_roots_match_arranger_spelling():
    """Константы читаются из исходника AST-ом: в полном прогоне voice-сьюта
    другие тесты подменяют ``rob_box_mcp_tools`` заглушкой в sys.modules."""
    src = Path(__file__).resolve().parents[4] / "rob_box_mcp_tools" / "rob_box_mcp_tools" / "core" / "arranger.py"
    consts = {
        node.targets[0].id: ast.literal_eval(node.value)
        for node in ast.parse(src.read_text(encoding="utf-8")).body
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") in {"VALID_ROOTS", "BPM_RANGE"}
    }
    assert CLUB_ROOTS == consts["VALID_ROOTS"]
    assert DJ_SET_BPM_RANGE == tuple(int(x) for x in consts["BPM_RANGE"])


def test_first_track_and_final_track_use_club_in_set_tempo():
    ctrl, _, _ = _controller()
    _enable(ctrl)
    first = ctrl.build_auto_prompt(1)
    (call,) = _club_calls(first)
    assert call[0] == "124" and call[3] == "true"
    ctrl.state.set_plan = "Трек 1: разгон\nТрек 2: финал"
    ctrl.state.tracks_started = 1
    final = ctrl.build_auto_prompt(2)
    assert "ФИНАЛЬНЫЙ ТРЕК" in final
    assert any(c[3] == "false" for c in _club_calls(final))


def test_named_plan_track_keeps_classic_path_with_set_bpm():
    ctrl, _, _ = _controller()
    _enable(ctrl, plan="Трек 1: Still Dre\nТрек 2: Next Episode\nТрек 3: финал")
    prompt = ctrl.build_auto_prompt(1)
    assert 'compose_music(name="Still Dre", seed=' in prompt
    assert f"+ bpm={DJ_SET_DEFAULT_BPM} (темп сета)" in prompt
