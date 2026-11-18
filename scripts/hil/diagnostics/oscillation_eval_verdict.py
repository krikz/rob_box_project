#!/usr/bin/env python3
"""Кто режет wz>0 на полном /evaluation (#4682).

Полный — это запись с short_circuit_trajectory_evaluation: false, где каждый
образец досчитан до конца либо отвергнут критиком. DWB помечает отвергнутый
образец так (dwb_local_planner.cpp, coreScoringAlgorithm): одна запись CriticScore
с именем критика и raw_score = -1.

oscillation_confirmed — среди образцов wz>0 есть отвергнутые критиком
Oscillation, и ни один такой образец не отвергнут другим критиком раньше неё.
other_critic — wz>0 отвергнуты, но другим критиком, Oscillation их не резала.
insufficient — отвергнутых wz>0 нет или запись неполная (у образца нет ни
оценки, ни маркера).
"""

from __future__ import annotations

import argparse
import json
import sys
from typing import Iterable, Optional

OSCILLATION = "Oscillation"
REJECTED = -1.0

CONFIRMED = "oscillation_confirmed"
OTHER = "other_critic"
INSUFFICIENT = "insufficient"


def rejection(sample: dict) -> Optional[str]:
    """Имя критика, отвергнувшего образец, либо None, если образец досчитан."""
    scores = sample.get("scores", [])
    if len(scores) == 1 and float(scores[0].get("raw_score", 0.0)) == REJECTED:
        return scores[0].get("name")
    return None


def verdict_of(samples: Iterable[dict]) -> dict:
    rows = [row for row in samples if float(row.get("wz", 0.0)) > 0.0]
    if not rows:
        return {"verdict": INSUFFICIENT, "reason": "нет образцов wz>0", "n": 0}

    rejected_by = {}
    unscored = 0
    for row in rows:
        critic = rejection(row)
        if critic is None and not row.get("scores"):
            unscored += 1
            continue
        if critic is not None:
            rejected_by[critic] = rejected_by.get(critic, 0) + 1

    if unscored:
        return {
            "verdict": INSUFFICIENT,
            "reason": f"без оценки и без маркера: {unscored} из {len(rows)}",
            "n": len(rows),
            "unscored": unscored,
        }
    oscillation = rejected_by.get(OSCILLATION, 0)
    others = {name: count for name, count in rejected_by.items() if name != OSCILLATION}
    if oscillation and not others:
        return {
            "verdict": CONFIRMED,
            "reason": f"Oscillation отвергла {oscillation} из {len(rows)} образцов wz>0",
            "n": len(rows),
            "rejected_by": rejected_by,
        }
    if others and not oscillation:
        return {
            "verdict": OTHER,
            "reason": f"wz>0 отвергнуты не Oscillation: {others}",
            "n": len(rows),
            "rejected_by": rejected_by,
        }
    return {
        "verdict": INSUFFICIENT,
        "reason": f"Oscillation {oscillation}, другие {others}: одного виновника нет",
        "n": len(rows),
        "rejected_by": rejected_by,
    }


def load_samples(path: str) -> list:
    with open(path, encoding="utf-8") as handle:
        data = json.load(handle)
    if isinstance(data, dict):
        data = data.get("samples", data.get("ticks", []))
    if data and isinstance(data[0], dict) and "samples" in data[0]:
        data = data[0]["samples"]
    return data


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("samples")
    args = parser.parse_args(argv)
    result = verdict_of(load_samples(args.samples))
    json.dump(result, sys.stdout, ensure_ascii=False, indent=1)
    sys.stdout.write("\n")
    return 0 if result["verdict"] != INSUFFICIENT else 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
