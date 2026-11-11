#!/usr/bin/env python3
"""Отчёт прогона здания (#4468) по скачанному каталогу: вердикты кода, таблица отрезков, анимации.

    building_report.py OUT_DIR [--frames 120] [--no-anim]

Читает ``steps/*.log``, ``maprec*/poses.csv``, ``tools.log`` (их собирает ``run_building.sh``). Пишет:

  * ``verdicts.txt`` — строки ``VERDICT - preflight …`` прошлых проверок (их пишет checks/verdicts.py) и
    ``VERDICT #4468 <блок> <статус>: …`` (``rob_box_sim.building_run.evaluate``);
  * ``report.md`` — те же строки и таблица отрезков (что было, сколько sim-секунд, где);
  * ``anim/<отрезок>.gif`` — вид сверху: зелёный — истина Gazebo, красный — оценка rtabmap; фон — стены и мебель
    этажа из той же геометрии, что ``building.sdf`` (``building_run.floor_grid``). Рисует ``video/pose_anim.py``.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "rob_box_sim"))
sys.path.insert(0, str(ROOT / "scripts" / "hil" / "video"))

from rob_box_sim import building_run as br  # noqa: E402


def _rows(samples, t0: float, t1: float) -> list:
    import pose_anim as pa

    rows = []
    for s in samples:
        if not (t0 <= s.t <= t1) or s.truth is None:
            continue
        est = s.estimate or (None, None, None)
        rows.append(pa.Row(s.t, s.truth[0], s.truth[1], s.truth[2], est[0], est[1], est[2]))
    return rows


def write_anim(out: Path, segs, samples, frames: int) -> list[str]:
    """GIF на отрезок; отрезок без поз пропускается со строкой (не рисуем пустое)."""
    import pose_anim as pa

    floor = br.floor_grid()
    done = []
    (out / "anim").mkdir(exist_ok=True)
    for seg in segs:
        rows = _rows(samples, seg.t0, seg.t1)
        if len(rows) < 2:
            done.append(f"{seg.name}: поз нет — анимации нет")
            continue
        run = pa.Run(rows, f"{seg.name} ({seg.kind})", maps=[floor])
        title = f"#4468 здание: {seg.name} — правда Gazebo против оценки rtabmap"
        imgs = list(pa.side_by_side_frames([run], title, frames, size=(960, 400), hold=max(1, frames // 12)))
        path = out / "anim" / f"{seg.name}.gif"
        size = pa.save_gif(imgs, path, fps=10)
        done.append(f"{seg.name}: {path.name} {size / 1e6:.2f} МБ")
    return done


def segment_table(segs) -> str:
    lines = ["| отрезок | вид | sim, с | целей | где |", "|---|---|---|---|---|"]
    for s in segs:
        where = f"({s.start.get('x')}, {s.start.get('y')})" if s.start.get("x") is not None else ""
        lines.append(f"| {s.name} | {s.kind} | {s.t1 - s.t0:.0f} | {len(s.goals)} | {where} |")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("out", type=Path)
    ap.add_argument("--frames", type=int, default=120)
    ap.add_argument("--no-anim", action="store_true")
    a = ap.parse_args(argv)
    events, goals, samples, tools = br.load_run(a.out)
    verdicts = br.evaluate(events, goals, samples, tool_results=tools)
    old = a.out / "verdicts.txt"
    keep = [x for x in old.read_text(encoding="utf-8").splitlines() if " preflight " in x] if old.is_file() else []
    lines = keep + [v.line() for v in verdicts]
    old.write_text("\n".join(lines) + "\n", encoding="utf-8")
    segs = br.segments(events, goals)
    anim = [] if a.no_anim else write_anim(a.out, segs, samples, a.frames)
    md = ["# Прогон здания #4468", "", "```", *lines, "```", "", segment_table(segs), ""]
    if anim:
        md += ["Анимации (anim/):", "", *[f"- {x}" for x in anim], ""]
    (a.out / "report.md").write_text("\n".join(md), encoding="utf-8")
    print("\n".join(lines))
    for x in anim:
        print("ANIM " + x)
    return 0


if __name__ == "__main__":
    sys.exit(main())
