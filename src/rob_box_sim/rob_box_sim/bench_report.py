"""Сводка прогонов стенда по конфигурациям: таблица «контроллер × мир × цель». Без ROS.

План: docs/research/2026-11-04-gazebo-sim-navigation.md §6 шаг 6.

Вход — json, которые пишет nav_bench (``<мир>.json``), из любого числа каталогов.
Конфигурация берётся из метки прогона ``--label``: всё до ``@`` (``dwb-navfn@3`` →
``dwb-navfn``), так повторы одной конфигурации складываются в одну строку.

Что в строке (по запущенным целям; SKIPPED не считается попыткой):
  * успех k/n;
  * время до цели — медиана по успешным (провал по таймауту не «быстрый»);
  * мин. зазор — наименьший по всем попыткам и медиана;
  * смены знака w на метр — медиана по ``/cmd_vel`` и по выходу контроллера ``/cmd_vel_nav``;
  * recovery — медиана и максимум; столкновения — число попыток с касанием;
  * RTF min — худший real-time factor попытки; «⚠ k<0.8» — k попыток под нагрузкой,
    «не мерян» — json старше счётчика RTF.

    python3 -m rob_box_sim.bench_report results/*/ > table.md
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from collections import defaultdict
from pathlib import Path
from typing import Iterable, Optional, Sequence

from rob_box_sim.nav_metrics import RTF_OK


def config_of(label: str) -> str:
    return label.split("@", 1)[0]


def load_rows(paths: Iterable[Path]) -> list[dict]:
    """Строки целей из json стенда; каталоги обходятся по ``*.json``."""
    rows: list[dict] = []
    for p in paths:
        files = sorted(p.glob("*.json")) if p.is_dir() else [p]
        for f in files:
            doc = json.loads(f.read_text(encoding="utf-8"))
            for g in doc.get("goals", []):
                rows.append({**g, "label": doc.get("label", g.get("label", "")), "source": str(f)})
    return rows


def _median(vals: Sequence[float]) -> Optional[float]:
    return statistics.median(vals) if vals else None


def _vals(rows: Sequence[dict], key: str) -> list[float]:
    return [r[key] for r in rows if r.get(key) is not None]


def aggregate(rows: Sequence[dict]) -> list[dict]:
    """Строка на (конфигурация, мир, цель) в порядке первого появления."""
    groups: dict[tuple[str, str, str], list[dict]] = defaultdict(list)
    for r in rows:
        groups[(config_of(r["label"]), r["world"], r["goal"])].append(r)
    out = []
    for (cfg, world, goal), rs in groups.items():
        ran = [r for r in rs if r.get("status") != "SKIPPED"]
        ok = [r for r in ran if r.get("success")]
        clear = _vals(ran, "min_clearance_m")
        rec = _vals(ran, "recoveries")
        out.append({
            "config": cfg,
            "world": world,
            "goal": goal,
            "ok": len(ok),
            "ran": len(ran),
            "skipped": len(rs) - len(ran),
            "time_med_s": _median(_vals(ok, "time_s")),
            "clear_min_m": min(clear) if clear else None,
            "clear_med_m": _median(clear),
            "osc_med": _median(_vals(ran, "osc_per_m")),
            "osc_raw_med": _median(_vals(ran, "osc_raw_per_m")),
            "rec_med": _median(rec),
            "rec_max": max(rec) if rec else None,
            "collisions": sum(1 for r in ran if r.get("collision")),
            "tilt_max_deg": max(_vals(ran, "tilt_max_deg"), default=None),
            "rtf_min": min(_vals(ran, "rtf"), default=None),
            "rtf_low": sum(1 for r in ran if r.get("rtf") is not None and r["rtf"] < RTF_OK),
            "rtf_unknown": sum(1 for r in ran if r.get("rtf") is None),
        })
    return out


def totals(agg: Sequence[dict]) -> dict[str, tuple[int, int]]:
    """Успех k/n по конфигурации, все миры вместе."""
    t: dict[str, list[int]] = defaultdict(lambda: [0, 0])
    for a in agg:
        t[a["config"]][0] += a["ok"]
        t[a["config"]][1] += a["ran"]
    return {k: (v[0], v[1]) for k, v in t.items()}


def _f(v, nd: int = 2) -> str:
    if v is None:
        return "—"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


COLUMNS = (
    "| конфигурация | мир | цель | успех | время, с (медиана успешных) | мин. зазор, м (min / медиана) "
    "| смен знака w/м: cmd_vel / cmd_vel_nav | recovery (медиана / max) | столкн. | наклон max, ° | RTF min |"
)


def _rtf_cell(a: dict) -> str:
    """RTF min и пометка: «⚠ k<0.8» — k попыток под нагрузкой, «не мерян» — старый прогон."""
    if a["ran"] and a["rtf_unknown"] == a["ran"]:
        return "не мерян"
    cell = _f(a["rtf_min"])
    if a["rtf_low"]:
        cell += f" ⚠ {a['rtf_low']}<{RTF_OK}"
    if a["rtf_unknown"]:
        cell += f" ({a['rtf_unknown']} не мерян)"
    return cell


def markdown(agg: Sequence[dict]) -> str:
    lines = [COLUMNS, "|" + "---|" * 11]
    for a in agg:
        skipped = f" (+{a['skipped']} SKIPPED)" if a["skipped"] else ""
        lines.append(
            f"| {a['config']} | {a['world']} | {a['goal']} | {a['ok']}/{a['ran']}{skipped} | "
            f"{_f(a['time_med_s'], 1)} | {_f(a['clear_min_m'], 3)} / {_f(a['clear_med_m'], 3)} | "
            f"{_f(a['osc_med'])} / {_f(a['osc_raw_med'])} | {_f(a['rec_med'], 0)} / {_f(a['rec_max'])} | "
            f"{a['collisions']} | {_f(a['tilt_max_deg'], 1)} | {_rtf_cell(a)} |"
        )
    lines.append("")
    lines.append("| конфигурация | успех по всем целям |")
    lines.append("|---|---|")
    for cfg, (ok, ran) in totals(agg).items():
        lines.append(f"| {cfg} | {ok}/{ran} |")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Сводка прогонов стенда навигации по конфигурациям")
    ap.add_argument("paths", nargs="+", type=Path, help="каталоги результатов или json")
    args = ap.parse_args(argv)
    print(markdown(aggregate(load_rows(args.paths))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
