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
    DJHook,
    DJModeController,
    related_root,
)
from rob_box_voice.core.dj_set_walk import BPM_DRIFT_MAX_STEP, bpm_walk, track_bpm, track_key

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
    return re.findall(r'compose_music\(style="club", (?:theme="[^"]*", )?bpm=(\d+), root="([A-G]#?)", '
                      r'scale="(\w+)", seed=(\d+), repeat=(true|false), transition="fade"\)', prompt)


def test_mid_set_prompt_has_no_other_bpm_demand_and_drifting_bpm():
    ctrl, _, _ = _controller()
    _enable(ctrl)
    ctrl.state.tracks_started = 1
    prompt = ctrl.build_auto_prompt(2)
    assert "другой" not in prompt
    assert "bpm/root/scale" not in prompt
    calls = _club_calls(prompt)
    assert len(calls) == 1
    assert 120 <= int(calls[0][0]) <= 128  # #3226: темп дрейфует около 124
    assert calls[0][4] == "true"
    # Issue #3226: темп дрейфует около базового, а не «один на весь сет».
    assert f"плавно дрейфует около {DJ_SET_DEFAULT_BPM} BPM" in prompt
    # Путь classic для названной юзером песни остаётся, в темпе этого трека.
    assert "compose_music(name=..., seed=" in prompt
    assert f"bpm={calls[0][0]}, repeat=true)" in prompt
    # Техническая «кухня» перехода в промпт не утекает.
    for leak in ("Clock.clear()", "Master()", "linvar"):
        assert leak not in prompt


def test_bpm_drifts_within_range_until_user_names_a_tempo():
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
    nums = [int(b) for b in bpms]
    assert len(nums) == 4 and all(120 <= b <= 128 for b in nums)
    assert all(abs(a - b) <= BPM_DRIFT_MAX_STEP for a, b in zip(nums, nums[1:]))
    assert len(set(nums)) >= 2  # темп реально двигается
    assert not ctrl.state.bpm_locked
    # Явная просьба юзера («давай 128») меняет темп сета и фиксирует его.
    _enable(ctrl, bpm=128, next_transition_sec=15)
    assert ctrl.state.set_bpm == 128 and ctrl.state.bpm_locked
    clock["t"] += 20
    ctrl.tick()
    assert len(dispatched) == 5
    assert {c[0] for c in _club_calls(dispatched[-1])} == {"128"}
    # Мусор/вне диапазона — клампится, не ломает сет.
    _enable(ctrl, bpm=999)
    assert ctrl.state.set_bpm == 180


def test_model_echoing_drift_bpm_does_not_lock_tempo():
    """Модель повторяет bpm из готового вызова — это не просьба юзера (#3226)."""
    ctrl, clock, dispatched = _controller()
    _enable(ctrl, next_transition_sec=15)
    ctrl.state.tracks_started = 1
    clock["t"] += 20
    ctrl.tick()
    (call,) = _club_calls(dispatched[-1])
    _enable(ctrl, bpm=int(call[0]), next_transition_sec=15)
    assert not ctrl.state.bpm_locked and ctrl.state.set_bpm == DJ_SET_DEFAULT_BPM


def test_fresh_set_resets_bpm_and_root():
    ctrl, clock, _ = _controller()
    _enable(ctrl, bpm=100)
    assert ctrl.state.set_bpm == 100
    ctrl.handle_message(json.dumps({"enabled": False}))
    assert ctrl.state.set_bpm == DJ_SET_DEFAULT_BPM and ctrl.state.set_root == ""
    assert not ctrl.state.bpm_locked
    clock["t"] += 1
    _enable(ctrl)
    assert ctrl.state.set_bpm == DJ_SET_DEFAULT_BPM


@pytest.mark.parametrize(
    "track_no, expected",
    [(1, "E"), (2, "B"), (3, "F#"), (4, "C#"), (5, "G#"), (6, "D#"), (7, "A#"), (12, "A")],
)
def test_related_root_walks_circle_of_fifths(track_no, expected):
    assert related_root("E", track_no) == expected


def test_related_root_covers_all_keys_in_12_tracks():
    for root in CLUB_ROOTS:
        roots = [related_root(root, n) for n in range(1, 13)]
        assert len(set(roots)) == 12
        assert related_root(root, 13) == root
    assert related_root("H", 2) == "H"  # неизвестная тоника — как есть


def test_track_key_relations_over_20_track_sets():
    """Тональность родственна предыдущей: квинта; лады — из поддержанных;
    ≥6 разных root за 12 треков, цикла из 3 нот нет (#3226)."""
    for started in range(1_759_000_000, 1_759_000_040):
        set_root = CLUB_ROOTS[started % 12]
        keys = [track_key(set_root, n, started) for n in range(1, 21)]
        assert keys[0] == (set_root, "minor") and keys[1][1] == "minor"
        assert {scale for _, scale in keys} <= {"minor", "dorian", "phrygian", "major"}
        assert len({root for root, _ in keys[:12]}) >= 6
        for i in range(len(keys) - 3):
            assert len({root for root, _ in keys[i:i + 4]}) >= 3  # нет цикла 2-3 нот
        for n, (root, scale) in enumerate(keys, start=1):
            minor_root = related_root(set_root, n)
            shift = 3 if scale == "major" else 0
            assert CLUB_ROOTS.index(root) == (CLUB_ROOTS.index(minor_root) + shift) % 12
        assert track_key(set_root, 7, started) == keys[6]  # детерминизм


def test_track_key_uses_more_than_minor_across_sets():
    scales = {track_key("E", n, started)[1] for started in range(200) for n in range(3, 13)}
    assert scales == {"minor", "dorian", "phrygian", "major"}


def test_bpm_walk_range_step_and_variety():
    for started in range(500):
        walk = bpm_walk(DJ_SET_DEFAULT_BPM, 20, started)
        assert walk[0] == DJ_SET_DEFAULT_BPM
        assert all(120 <= b <= 128 for b in walk)
        assert all(0 < abs(a - b) <= BPM_DRIFT_MAX_STEP for a, b in zip(walk, walk[1:]))
        assert len(set(walk[:12])) >= 3
        assert walk == [track_bpm(DJ_SET_DEFAULT_BPM, n, started) for n in range(1, 21)]
    assert bpm_walk(124, 12, 1) != bpm_walk(124, 12, 2)


def test_prompt_root_follows_rule_and_seed_differs_per_track():
    ctrl, _, _ = _controller()
    _enable(ctrl)
    seen = []
    for started in range(0, 4):
        ctrl.state.tracks_started = started
        (call,) = _club_calls(ctrl.build_auto_prompt(started + 2))
        seen.append(call)
    set_root = CLUB_ROOTS[int(START) % 12]
    assert [c[1] for c in seen] == [track_key(set_root, n, int(ctrl.state.started_at))[0] for n in range(1, 5)]
    assert [c[2] for c in seen] == [track_key(set_root, n, int(ctrl.state.started_at))[1] for n in range(1, 5)]
    assert [int(c[0]) for c in seen] == [track_bpm(DJ_SET_DEFAULT_BPM, n, int(ctrl.state.started_at)) for n in range(1, 5)]
    assert len({c[3] for c in seen}) == 4


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
    assert call[0] == "124" and call[4] == "true"
    ctrl.state.set_plan = "Трек 1: разгон\nТрек 2: финал"
    ctrl.state.tracks_started = 1
    final = ctrl.build_auto_prompt(2)
    assert "ФИНАЛЬНЫЙ ТРЕК" in final
    assert any(c[4] == "false" for c in _club_calls(final))


def test_named_plan_track_keeps_classic_path_with_set_bpm():
    ctrl, _, _ = _controller()
    _enable(ctrl, plan="Трек 1: Still Dre\nТрек 2: Next Episode\nТрек 3: финал")
    prompt = ctrl.build_auto_prompt(1)
    assert 'compose_music(name="Still Dre", seed=' in prompt
    assert f"+ bpm={DJ_SET_DEFAULT_BPM} (темп сета)" in prompt
