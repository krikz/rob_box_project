"""Вердикт стояния по синтетическим CriticScore (#4682).

Схема — dwb_msgs/msg/CriticScore: name, raw_score, scale. Сообщение
LocalPlanEvaluation несёт массив таких оценок на образец и best_index.
Здесь образец — словарь той же формы, без ROS.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MOD_PATH = ROOT / "scripts" / "hil" / "diagnostics" / "eval_stall_verdict.py"

_spec = importlib.util.spec_from_file_location("eval_stall_verdict", MOD_PATH)
mod = importlib.util.module_from_spec(_spec)
assert _spec.loader is not None
sys.modules["eval_stall_verdict"] = mod
_spec.loader.exec_module(mod)


def sample(wz, rotate, chosen=False, extra=()):
    scores = [{"name": "RotateToGoal", "raw_score": rotate, "scale": 8.0}]
    scores.extend(extra)
    return {"wz": wz, "chosen": chosen, "scores": scores}


def test_confirmed_when_rotate_scores_match_and_zero_chosen():
    ticks = [
        sample(0.0, 0.420, chosen=True),
        sample(0.3, 0.424),
        sample(-0.3, 0.416),
    ]
    result = mod.verdict_of(ticks)
    assert result["verdict"] == mod.CONFIRMED
    assert result["spread"] <= 0.01
    assert result["chosen_wz"] == 0.0


def test_refuted_when_scores_differ_and_best_is_not_zero():
    ticks = [
        sample(0.0, 0.420, chosen=True),
        sample(-0.6, 0.050),
        sample(0.6, 0.800),
    ]
    result = mod.verdict_of(ticks)
    assert result["verdict"] == mod.REFUTED
    assert result["best_rotate_wz"] == -0.6


def test_short_circuit_without_rotate_is_not_confirmation():
    # DWB оборвал подсчёт раньше RotateToGoal: оценки нет, вердикт не подтверждается.
    ticks = [
        {"wz": 0.0, "chosen": True, "scores": [{"name": "PathAlign", "raw_score": 1.0, "scale": 48.0}]},
        {"wz": -0.6, "chosen": False, "scores": [{"name": "PathAlign", "raw_score": 3.0, "scale": 48.0}]},
    ]
    result = mod.verdict_of(ticks)
    assert result["verdict"] == mod.INSUFFICIENT
    assert result["missing_rotate"] == 2


def test_spread_above_threshold_with_zero_best_is_not_a_verdict():
    ticks = [
        sample(0.0, 0.10, chosen=True),
        sample(0.6, 0.30),
    ]
    result = mod.verdict_of(ticks)
    assert result["verdict"] == mod.INSUFFICIENT


def test_empty_window():
    assert mod.verdict_of([])["verdict"] == mod.INSUFFICIENT


def test_two_chosen_samples_are_rejected():
    ticks = [sample(0.0, 0.42, chosen=True), sample(0.0, 0.42, chosen=True)]
    assert mod.verdict_of(ticks)["verdict"] == mod.INSUFFICIENT
