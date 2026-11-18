"""Кто режет wz>0 на полном /evaluation (#4682).

Маркер отвергнутого образца — как его пишет DWB: одна запись CriticScore
с именем критика и raw_score = -1 (dwb_local_planner.cpp, coreScoringAlgorithm).
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MOD_PATH = ROOT / "scripts" / "hil" / "diagnostics" / "oscillation_eval_verdict.py"

_spec = importlib.util.spec_from_file_location("oscillation_eval_verdict", MOD_PATH)
mod = importlib.util.module_from_spec(_spec)
assert _spec.loader is not None
sys.modules["oscillation_eval_verdict"] = mod
_spec.loader.exec_module(mod)


def rejected(wz, critic):
    return {"wz": wz, "scores": [{"name": critic, "raw_score": -1.0}]}


def scored(wz):
    return {"wz": wz, "scores": [{"name": "RotateToGoal", "raw_score": 0.4, "scale": 8.0}]}


def test_oscillation_confirmed_when_it_rejects_positive_wz():
    samples = [rejected(0.3, "Oscillation"), rejected(0.6, "Oscillation"), scored(-0.6)]
    result = mod.verdict_of(samples)
    assert result["verdict"] == mod.CONFIRMED
    assert result["rejected_by"] == {"Oscillation": 2}


def test_other_critic_when_positive_wz_is_rejected_elsewhere():
    samples = [rejected(0.3, "ObstacleFootprint"), rejected(0.6, "ObstacleFootprint")]
    result = mod.verdict_of(samples)
    assert result["verdict"] == mod.OTHER


def test_insufficient_when_samples_carry_neither_score_nor_marker():
    samples = [{"wz": 0.3, "scores": []}, {"wz": 0.6, "scores": []}]
    result = mod.verdict_of(samples)
    assert result["verdict"] == mod.INSUFFICIENT
    assert result["unscored"] == 2


def test_insufficient_when_no_positive_wz():
    assert mod.verdict_of([scored(-0.6)])["verdict"] == mod.INSUFFICIENT


def test_split_blame_is_not_a_verdict():
    samples = [rejected(0.3, "Oscillation"), rejected(0.6, "ObstacleFootprint")]
    assert mod.verdict_of(samples)["verdict"] == mod.INSUFFICIENT
