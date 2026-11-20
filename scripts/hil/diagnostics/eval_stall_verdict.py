#!/usr/bin/env python3
"""Вердикт по /evaluation на стоянии у цели (#4682, повторный M1).

Гипотеза стояния (комментарий #4682): RotateToGoal при lookahead_time 1.0
оценивает позу через 1 с, а не команду. На стоящем роботе (vx=0, wz=0) внутри
xy-допуска с недобранным yaw все образцы wz дают одну оценку, и выбран wz=0.

Подтверждение — только по полному списку критиков. При
short_circuit_trajectory_evaluation DWB обрывает подсчёт, как только сумма
превысила лучшую (dwb_local_planner.cpp, scoreTrajectory), и в сообщении
нет оценки RotateToGoal. Такое сообщение вердикт не подтверждает.

Скрипт не ходит на стенд и не декодирует CDR: разбор bag делает
decode_eval_bag.py, этот модуль считает вердикт по уже разобранным тикам.
"""

from __future__ import annotations

import argparse
import json
import sys
from typing import Iterable, Optional

ROTATE = "RotateToGoal"
SPREAD = 0.01

CONFIRMED = "lookahead_time_confirmed"
REFUTED = "refuted_other_critic"
INSUFFICIENT = "insufficient"


def critic(tick: dict, name: str) -> Optional[dict]:
    for item in tick.get("scores", []):
        if item.get("name") == name:
            return item
    return None


def rotate_raw(tick: dict) -> Optional[float]:
    found = critic(tick, ROTATE)
    if not found or "raw_score" not in found:
        return None
    return float(found["raw_score"])


def verdict_of(ticks: Iterable[dict], spread: float = SPREAD) -> dict:
    """Вердикт по тикам одного окна стояния.

    ticks: список образцов одного такта контроллера. У каждого wz, chosen
    (ровно один True) и scores — список {name, raw_score, scale}.
    """
    rows = list(ticks)
    if not rows:
        return {"verdict": INSUFFICIENT, "reason": "нет тиков", "n": 0}

    chosen = [row for row in rows if row.get("chosen")]
    raws = [rotate_raw(row) for row in rows]
    present = [value for value in raws if value is not None]
    missing = len(rows) - len(present)

    if missing:
        return {
            "verdict": INSUFFICIENT,
            "reason": (
                f"RotateToGoal отсутствует у {missing} из {len(rows)} образцов: "
                "список критиков оборван (short_circuit) или топик не тот"
            ),
            "n": len(rows),
            "missing_rotate": missing,
        }
    if len(chosen) != 1:
        return {
            "verdict": INSUFFICIENT,
            "reason": f"выбранных образцов {len(chosen)}, нужен ровно один",
            "n": len(rows),
        }

    lo, hi = min(present), max(present)
    chosen_wz = float(chosen[0]["wz"])
    best = min(rows, key=lambda row: rotate_raw(row))
    best_wz = float(best["wz"])
    same = (hi - lo) <= spread
    if same and chosen_wz == 0.0:
        verdict = CONFIRMED
        reason = f"разброс RotateToGoal {hi - lo:.4f} <= {spread}, выбран wz=0"
    elif (not same) and best_wz != 0.0:
        verdict = REFUTED
        reason = f"лучший wz={best_wz} при разбросе {hi - lo:.4f}, выбран wz={chosen_wz}"
    else:
        verdict = INSUFFICIENT
        reason = f"разброс {hi - lo:.4f}, лучший wz={best_wz}, выбран wz={chosen_wz}: ни одно правило"
    return {
        "verdict": verdict,
        "reason": reason,
        "n": len(rows),
        "rotate_min": lo,
        "rotate_max": hi,
        "spread": hi - lo,
        "chosen_wz": chosen_wz,
        "best_rotate_wz": best_wz,
    }


def load_ticks(path: str) -> list:
    """Принимает и список образцов, и файл декодера {ticks: [{samples: [...]}]}."""
    with open(path, encoding="utf-8") as handle:
        data = json.load(handle)
    if isinstance(data, dict):
        data = data.get("ticks", [])
    if data and isinstance(data[0], dict) and "samples" in data[0]:
        data = data[0]["samples"]
    return data


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("ticks", help="JSON со списком образцов одного такта")
    parser.add_argument("--spread", type=float, default=SPREAD)
    args = parser.parse_args(argv)
    result = verdict_of(load_ticks(args.ticks), args.spread)
    json.dump(result, sys.stdout, ensure_ascii=False, indent=1)
    sys.stdout.write("\n")
    return 0 if result["verdict"] != INSUFFICIENT else 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
