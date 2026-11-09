#!/usr/bin/env python3
"""Предельная скорость в HIL: метрики прогона и вердикт «проходит / падает» строит код (решение Шифу 09.11).

Чистая логика без ROS. Её используют две стороны:

* ``speed_bench.py`` (rclpy в контейнере gazebo песочницы) считает здесь метрики цели: ошибку позы, интервалы
  ``/cmd_vel``, потери ICP, тормозной путь у внезапной стенки;
* оператор (``sweep.sh``) зовёт CLI: вердикт прогона по ``result.json`` и замеру Main Pi, следующая скорость
  дихотомии и какую сцену прогнать — по журналу точек.

Критерии (``Criteria``) — числа, а не слова; их обоснование — в ``docs/research/2026-11-05-nav2-hil-tuning.md`` §11.

    python speed_verdict.py run <result.json> <main.json> [--speed V --scene S --rep N --ledger L.jsonl]
    python speed_verdict.py next --ledger L.jsonl --cls open|tight   # NEXT <класс> <скорость> <сцена> <повтор> | DONE
    python speed_verdict.py cpu <cpu.log>                            # замер Main Pi → json

Коды выхода ``run``: 0 PASS, 1 FAIL.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

try:  # в контейнере сима rob_box_sim установлен
    from rob_box_sim.geometry2d import Box, Pose2D, clearance
except ImportError:  # в репо (тесты, машина оператора)
    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src" / "rob_box_sim"))
    from rob_box_sim.geometry2d import Box, Pose2D, clearance

PASS, FAIL = "PASS", "FAIL"

_PARENTS = Path(__file__).resolve().parents
ROOT = _PARENTS[3] if len(_PARENTS) > 3 else _PARENTS[0]  # в контейнере сима файл лежит в /tmp/speed
#: Правило защиты (#4438) берётся из её же модуля и footprint Nav2 — числа здесь не копируются.
PROXIMITY_FILE = ROOT / "src" / "rob_box_teleop" / "rob_box_teleop" / "proximity.py"
NAV2_PARAMS_FILE = ROOT / "docker" / "main" / "config" / "nav2" / "nav2_params.yaml"

#: Сцена → класс места. ``open`` — открытый зал (Depot, внезапная стенка), ``tight`` — проёмы и угол.
SCENE_CLASS = {"depot": "open", "polygon": "tight", "turn90": "tight"}


@dataclass(frozen=True)
class Criteria:
    """Пороги вердикта. Меняются только здесь (тест держит значения)."""

    #: Ошибка позы ``map → base_footprint`` против правды Gazebo, p95, м.
    pose_p95_max_m: float = 0.10
    #: Самая длинная серия потерянных кадров ICP (``/odom`` covariance ≥ 9999). 1 кадр со сбросом
    #: (``Odom/ResetCountdown=1``) одометрия переживает (§9.5); серия ≥ 2 — срыв.
    icp_max_lost_run: int = 1
    #: Интервал ``/cmd_vel`` контроллера в движении (команда v > 0.02), p95, с. Номинал 0.05 (20 Гц);
    #: 0.10 — в 95 % тактов не больше одного пропуска подряд.
    cmd_p95_max_s: float = 0.10
    #: Интервал больше этого — «серия пропусков» (≥ 4 такта 20 Гц без команды), с.
    cmd_gap_s: float = 0.25
    #: Допустимо серий пропусков на минуту движения.
    cmd_gaps_per_min_max: float = 1.0
    #: CPU контейнера nav2 на Main Pi (``docker stats``, % из 400), p95.
    nav2_cpu_p95_max: float = 200.0
    #: Зазор корпус — стенка после остановки перед внезапной стенкой, м (зона стопа защиты 0.20).
    obstacle_gap_min_m: float = 0.20
    #: Зазор меньше этого — касание (как ``nav_metrics.CONTACT_M``), м.
    contact_m: float = 0.01
    #: Ошибка правды на цели: ``xy_goal_tolerance`` 0.25 + допуск позы 0.10 (Nav2 сверяет цель в ``map``), м.
    goal_tol_m: float = 0.35
    #: Сколько прогонов набора нужно точке для PASS.
    min_runs: int = 2


CRITERIA = Criteria()


# ── статистика ───────────────────────────────────────────────────────────────


def percentile(values: Sequence[float], q: float) -> Optional[float]:
    """Перцентиль ``q`` (0..100) с линейной интерполяцией, как ``numpy.percentile``; пусто — None."""
    xs = sorted(v for v in values if v is not None and math.isfinite(v))
    if not xs:
        return None
    pos = (len(xs) - 1) * q / 100.0
    lo = int(math.floor(pos))
    hi = min(lo + 1, len(xs) - 1)
    return xs[lo] + (xs[hi] - xs[lo]) * (pos - lo)


def _r(v: Optional[float], nd: int = 3) -> Optional[float]:
    return None if v is None else round(v, nd)


def spread_stats(values: Sequence[float]) -> Dict[str, Optional[float]]:
    """p50 / p95 / max и число образцов."""
    xs = [v for v in values if v is not None and math.isfinite(v)]
    return {
        "n": len(xs),
        "p50": _r(percentile(xs, 50)),
        "p95": _r(percentile(xs, 95)),
        "max": _r(max(xs, default=None)),
    }


def pose_error(truth: Tuple[float, float], spawn: Pose2D, map_pose: Tuple[float, float, float]) -> float:
    """Ошибка оценки: ``map`` rtabmap = поза спавна, значит мировая оценка = spawn ∘ map_pose."""
    est = spawn.compose(Pose2D(*map_pose))
    return math.hypot(est.x - truth[0], est.y - truth[1])


def lost_runs(flags: Iterable[bool]) -> Tuple[int, int]:
    """(всего потерянных кадров, самая длинная серия подряд)."""
    total = run = longest = 0
    for lost in flags:
        run = run + 1 if lost else 0
        total += int(lost)
        longest = max(longest, run)
    return total, longest


def cmd_interval_stats(
    cmds: Sequence[Tuple[float, float]], gap_s: float = CRITERIA.cmd_gap_s, moving_v: float = 0.02
) -> Dict[str, Optional[float]]:
    """Интервалы ``/cmd_vel`` (t, v) между соседними командами, когда обе едут вперёд (v > ``moving_v``).

    Развороты на месте (Spin, v = 0) идут от behavior_server со своей частотой — их не считаем.
    """
    dts = [b[0] - a[0] for a, b in zip(cmds, cmds[1:]) if a[1] > moving_v and b[1] > moving_v]
    moving_s = sum(dts)
    gaps = sum(1 for d in dts if d > gap_s)
    out = spread_stats(dts)
    out.update(
        {
            "moving_s": round(moving_s, 1),
            "gaps": gaps,
            "gaps_per_min": round(gaps / (moving_s / 60.0), 2) if moving_s > 0 else 0.0,
        }
    )
    return out


def speeds(track: Sequence[Tuple[float, float, float]], window_s: float = 0.2) -> List[Tuple[float, float]]:
    """Скорость по правде Gazebo (t, x, y) конечной разностью на окне ``window_s``: [(t, v)]."""
    out: List[Tuple[float, float]] = []
    j = 0
    for i, (t, x, y) in enumerate(track):
        while j < i and t - track[j][0] > window_s:
            j += 1
        if j < i and t - track[j][0] > 0.5 * window_s:
            t0, x0, y0 = track[j]
            out.append((t, math.hypot(x - x0, y - y0) / (t - t0)))
    return out


# ── внезапная стенка ────────────────────────────────────────────────────────


def wall_ahead(
    pose: Pose2D,
    ahead_m: float,
    width_m: float = 3.0,
    depth_m: float = 0.2,
    height_m: float = 1.0,
    front_m: float = 0.3064,
) -> Box:
    """Стенка поперёк хода: ближняя грань в ``ahead_m`` от носа корпуса (``front_m`` — нос от base_footprint)."""
    centre = pose.compose(Pose2D(front_m + ahead_m + depth_m / 2, 0.0))
    return Box("speed_wall", centre.x, centre.y, depth_m, width_m, height_m, pose.yaw)


def obstacle_metrics(
    track: Sequence[Tuple[float, float, float, float]],
    wall: Box,
    t_spawn: float,
    still_v: float = 0.02,
    still_s: float = 0.5,
) -> Dict[str, Optional[float]]:
    """Тормозной путь и зазоры у стенки по правде (t, x, y, yaw) после её появления в ``t_spawn``.

    Остановка — первый момент, после которого скорость < ``still_v`` держится ``still_s``.
    ``stop_path_m`` — путь корпуса от появления до остановки, ``stop_gap_m`` — зазор в момент остановки,
    ``min_gap_m`` — наименьший зазор за всё время после появления (касание видно и при объезде).
    """
    after = [p for p in track if p[0] >= t_spawn]
    if not after:
        return {"stopped": False}
    vs = dict(speeds([(t, x, y) for t, x, y, _ in after]))
    gaps = [(p[0], clearance(Pose2D(p[1], p[2], p[3]), [wall])) for p in after]
    t_stop = _first_still(after, vs, still_v, still_s)
    path = 0.0
    for a, b in zip(after, after[1:]):
        if t_stop is not None and b[0] > t_stop:
            break
        path += math.hypot(b[1] - a[1], b[2] - a[2])
    v_spawn = next((v for t, v in sorted(vs.items()) if t >= t_spawn), None)
    gap_at = dict(gaps)
    return {
        "stopped": t_stop is not None,
        "v_at_spawn": _r(v_spawn),
        "gap_at_spawn_m": _r(gaps[0][1]),
        "stop_after_s": _r(None if t_stop is None else t_stop - t_spawn, 2),
        "stop_path_m": _r(path),
        "stop_gap_m": _r(None if t_stop is None else gap_at[t_stop]),
        "min_gap_m": _r(min(g for _, g in gaps)),
    }


def _first_still(after, vs: Mapping[float, float], still_v: float, still_s: float) -> Optional[float]:
    since = None
    for t, *_ in after:
        v = vs.get(t)
        if v is None:
            continue
        if v < still_v:
            since = t if since is None else since
            if t - since >= still_s:
                return since
        else:
            since = None
    return None


def load_guard():
    """(модуль ``proximity``, корпус ``Body`` из footprint Nav2, ``Thresholds()``) — правило защиты как в узле."""
    import importlib.util

    import yaml

    spec = importlib.util.spec_from_file_location("speed_proximity", PROXIMITY_FILE)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    body = mod.body_from_nav2_params(yaml.safe_load(NAV2_PARAMS_FILE.read_text(encoding="utf-8")))
    return mod, body, mod.Thresholds()


def box_outline(box: Box, step: float = 0.01) -> List[Tuple[float, float]]:
    """Точки контура коробки в мире через ``step`` м — то, что видел бы лидар."""
    corners = box.corners()
    out: List[Tuple[float, float]] = []
    for (x0, y0), (x1, y1) in zip(corners, corners[1:] + corners[:1]):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
        out += [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n)]
    return out


def guard_rule_gap(
    track: Sequence[Tuple[float, float, float, float]], wall: Box, t0: float, t1: float, guard=None
) -> Dict[str, Optional[float]]:
    """Зазор до стенки по правилу защиты (#4438) по правде (t, x, y, yaw) в окне [t0, t1].

    Для каждой позы точки контура стенки переводятся в ``base_footprint``, порог каждой точки даёт
    ``proximity.stop_limits`` (коридор хода — ``stop_m``, сбоку — ``side_stop_m``, разворот — ``spin_stop_m``),
    скорость вдоль курса — по соседним позам правды. ``guard_margin_m`` — наименьший запас «зазор − порог»
    (меньше 0 — правило нарушено), ``guard_gap_m`` и ``guard_limit_m`` — зазор и порог в этой точке.
    """
    import numpy as np

    mod, body, th = guard or load_guard()
    pts = np.array(box_outline(wall))
    best: Optional[Tuple[float, float, float, float]] = None
    for a, b in zip(track, track[1:]):
        dt = b[0] - a[0]
        if not (t0 <= b[0] <= t1) or dt <= 0:
            continue
        _, x, y, yaw = b
        vx = ((b[1] - a[1]) * math.cos(yaw) + (b[2] - a[2]) * math.sin(yaw)) / dt
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = pts[:, 0] - x, pts[:, 1] - y
        local = np.column_stack([c * dx + s * dy, -s * dx + c * dy])
        qx = np.clip(local[:, 0], body.x_min, body.x_max)
        qy = np.clip(local[:, 1], body.y_min, body.y_max)
        gaps = np.hypot(local[:, 0] - qx, local[:, 1] - qy)
        limits = mod.stop_limits(body, local, vx, th)
        i = int(np.argmin(gaps - limits))
        cand = (float(gaps[i] - limits[i]), float(gaps[i]), float(limits[i]), b[0])
        best = cand if best is None or cand[0] < best[0] else best
    if best is None:
        return {"guard_margin_m": None}
    return {
        "guard_margin_m": _r(best[0]),
        "guard_gap_m": _r(best[1]),
        "guard_limit_m": _r(best[2]),
        "guard_t": _r(best[3], 2),
    }


def add_guard_rule(tasks: Sequence[Dict], track: Sequence[Tuple[float, float, float, float]], guard=None) -> None:
    """Дописать в задачи стенки метрику (б) — зазор по правилу защиты — от появления стенки до конца задачи."""
    for t in tasks:
        ob = t.get("obstacle") or {}
        if t.get("kind") != "obstacle" or not ob.get("spawned") or "wall" not in ob:
            continue
        wx, wy, wyaw = ob["wall"]
        wall = wall_ahead(Pose2D(0.0, 0.0, 0.0), 0.0)  # размеры стенки — одни на весь набор
        wall = Box(wall.name, wx, wy, wall.sx, wall.sy, wall.height, wyaw)
        ob.update(guard_rule_gap(track, wall, ob["t_spawn"], t["t_start"] + t["sim_s"], guard))


def read_track(path: Path) -> List[Tuple[float, float, float, float]]:
    """``track.csv`` бенча (t, x, y, yaw) → список поз."""
    if not path.exists():
        return []
    rows = path.read_text(encoding="utf-8").splitlines()[1:]
    return [tuple(float(v) for v in r.split(",")) for r in rows if r.strip()]


# ── вердикт ─────────────────────────────────────────────────────────────────


def task_failures(task: Mapping, crit: Criteria = CRITERIA) -> List[str]:
    """Причины провала одной задачи (цель или внезапная стенка). Пусто — задача прошла."""
    out: List[str] = []
    out += _pose_failures(task, crit)
    out += _cmd_failures(task, crit)
    clr = task.get("min_clearance_m")
    if clr is not None and clr < crit.contact_m:
        out.append(f"касание: зазор {clr:.3f} м < {crit.contact_m}")
    if task.get("kind") == "obstacle":
        out += _obstacle_failures(task.get("obstacle") or {}, crit)
    elif task.get("status") != "SKIPPED":
        out += _goal_failures(task, crit)
    return out


def _pose_failures(task: Mapping, crit: Criteria) -> List[str]:
    out = []
    pe = task.get("pose_err") or {}
    if pe.get("p95") is not None and pe["p95"] >= crit.pose_p95_max_m:
        out.append(f"ошибка позы p95 {pe['p95']:.3f} м ≥ {crit.pose_p95_max_m}")
    icp = task.get("icp") or {}
    if icp.get("longest_lost_run", 0) > crit.icp_max_lost_run:
        out.append(f"срыв ICP: {icp['longest_lost_run']} кадров подряд lost")
    return out


def _cmd_failures(task: Mapping, crit: Criteria) -> List[str]:
    out = []
    ci = task.get("cmd_interval") or {}
    if ci.get("p95") is not None and ci["p95"] > crit.cmd_p95_max_s:
        out.append(f"такт контроллера p95 {ci['p95']:.3f} с > {crit.cmd_p95_max_s}")
    return out


def run_cmd_gaps(tasks: Sequence[Mapping]) -> Tuple[int, float]:
    """Серии пропусков ``/cmd_vel`` и секунды движения за весь прогон сцены (одна серия в 6 с короткой
    задачи не должна считаться как «10 в минуту»)."""
    gaps = sum((t.get("cmd_interval") or {}).get("gaps", 0) for t in tasks)
    moving = sum((t.get("cmd_interval") or {}).get("moving_s", 0.0) for t in tasks)
    return gaps, moving


def _gap_failures(tasks: Sequence[Mapping], crit: Criteria) -> List[str]:
    gaps, moving = run_cmd_gaps(tasks)
    per_min = gaps / (moving / 60.0) if moving > 0 else 0.0
    if per_min > crit.cmd_gaps_per_min_max:
        return [f"серии пропусков /cmd_vel: {gaps} за {moving:.0f} с движения ({per_min:.2f}/мин)"]
    return []


def _goal_failures(task: Mapping, crit: Criteria) -> List[str]:
    if task.get("status") != "SUCCEEDED":
        return [f"цель {task.get('name')}: {task.get('status')}"]
    err = task.get("err_xy_m")
    if err is None or err > crit.goal_tol_m:
        return [f"цель {task.get('name')}: ошибка по правде {err} м > {crit.goal_tol_m}"]
    return []


def _obstacle_failures(ob: Mapping, crit: Criteria) -> List[str]:
    if not ob.get("spawned"):
        return ["стенка не появилась"]
    # Две метрики (координатор, разбор #4631): (а) ``min_gap_m`` — наименьший зазор до любой точки стенки после
    # её появления, для сведения; (б) ``guard_margin_m`` — запас по правилу защиты (#4438): в коридоре хода
    # против ``stop_m`` 0.20, сбоку против ``side_stop_m`` 0.08. «Падает» — по (б) и по касанию.
    gap = ob.get("min_gap_m")
    if gap is not None and gap < crit.contact_m:
        return [f"касание стенки: {gap:.3f} м"]
    if ob.get("guard_margin_m") is None:
        return ["нет зазора по правилу защиты (track.csv)"]
    if ob["guard_margin_m"] < 0:
        return [f"по правилу защиты: зазор {ob['guard_gap_m']:.3f} м < порога {ob['guard_limit_m']:.2f}"]
    return []


def run_verdict(tasks: Sequence[Mapping], main: Mapping, crit: Criteria = CRITERIA) -> Tuple[str, List[str]]:
    """Вердикт прогона сцены: все задачи прошли и CPU nav2 в пределе. Пустой прогон — FAIL."""
    reasons: List[str] = []
    if not tasks:
        reasons.append("нет задач в result.json")
    for t in tasks:
        reasons += [f"{t.get('name')}: {r}" for r in task_failures(t, crit)]
    reasons += _gap_failures(tasks, crit)
    cpu = (main.get("cpu") or {}).get("nav2") or {}
    if cpu.get("p95") is not None and cpu["p95"] > crit.nav2_cpu_p95_max:
        reasons.append(f"CPU nav2 p95 {cpu['p95']}% > {crit.nav2_cpu_p95_max}%")
    if not cpu:
        reasons.append("нет замера CPU Main Pi")
    return (FAIL if reasons else PASS), reasons


# ── дихотомия ───────────────────────────────────────────────────────────────


def point_verdicts(ledger: Iterable[Mapping], cls: str, crit: Criteria = CRITERIA) -> Dict[float, str]:
    """Скорость → PASS / FAIL / PENDING для класса места. FAIL — хоть один прогон упал; PASS — ``min_runs``
    прогонов каждой сцены класса прошли; иначе PENDING (прогонов мало)."""
    by: Dict[float, Dict[str, List[str]]] = {}
    for row in ledger:
        if SCENE_CLASS.get(row["scene"]) != cls:
            continue
        by.setdefault(round(row["speed"], 3), {}).setdefault(row["scene"], []).append(row["verdict"])
    scenes = sorted(s for s, c in SCENE_CLASS.items() if c == cls)
    out: Dict[float, str] = {}
    for v, per in by.items():
        if any(FAIL in vs for vs in per.values()):
            out[v] = FAIL
        elif all(len(per.get(s, [])) >= crit.min_runs for s in scenes):
            out[v] = PASS
        else:
            out[v] = "PENDING"
    return out


def next_speed(
    verdicts: Mapping[float, str], start: float = 1.0, step: float = 0.05, floor: float = 0.05
) -> Tuple[Optional[float], str]:
    """Следующая точка дихотомии: (скорость, пояснение) или (None, итог).

    С ``start``; провал → половина (до сетки ``step``); есть «проходит» и «падает» выше — середина между
    ними по сетке, пока разрыв > ``step``. Незаконченная точка (PENDING) — её и доделать.
    """
    top_pass = max((v for v, s in verdicts.items() if s == PASS), default=-1.0)
    # незаконченная точка ниже уже проходящей границы не меняет — её не доделываем
    pending = sorted(v for v, s in verdicts.items() if s == "PENDING" and v > top_pass)
    verdicts = {v: s for v, s in verdicts.items() if s != "PENDING"}
    if pending:
        return pending[0], "точка не досчитана"
    passes = sorted(v for v, s in verdicts.items() if s == PASS)
    fails = sorted(v for v, s in verdicts.items() if s == FAIL)
    if not verdicts:
        return start, "старт"
    if not passes:
        half = _grid(min(fails) / 2, step)
        if half < floor - 1e-9 or half >= min(fails) - 1e-9:
            return None, f"не проходит ни одна скорость до {min(fails)}"
        return half, f"{min(fails)} падает → половина"
    lo = max(passes)
    above = [f for f in fails if f > lo]
    if not above:
        return None, f"проходит {lo} (выше не пробовали: старт {start})"
    hi = min(above)
    n = int(round((hi - lo) / step))
    if n <= 1:
        return None, f"граница: {lo} проходит, {hi} падает"
    return round(lo + step * (n // 2), 3), f"бисекция {lo}…{hi}"


def _grid(v: float, step: float) -> float:
    return round(max(step, math.floor(v / step + 1e-9) * step), 3)


def class_scenes(cls: str) -> List[str]:
    """Сцены класса в порядке прогона (короткие — раньше: провал виден быстрее)."""
    order = {"turn90": 0, "polygon": 1, "depot": 2}
    return sorted((s for s, c in SCENE_CLASS.items() if c == cls), key=order.get)


def next_run(ledger: Sequence[Mapping], cls: str, speed: float, crit: Criteria = CRITERIA) -> Optional[Tuple[str, int]]:
    """Какой прогон точки ``speed`` сделать следующим: (сцена, номер повтора) или None — точка готова.

    Упала сцена — повторяем только её до ``min_runs`` (провал воспроизведён), остальные сцены точки не нужны.
    Иначе каждую сцену класса — до ``min_runs`` прогонов.
    """
    runs: Dict[str, List[str]] = {s: [] for s in class_scenes(cls)}
    for row in ledger:
        if row["scene"] in runs and abs(row["speed"] - speed) < 1e-6:
            runs[row["scene"]].append(row["verdict"])
    failed = [s for s, vs in runs.items() if FAIL in vs]
    if failed:
        short = [s for s in failed if len(runs[s]) < crit.min_runs]
        return (short[0], len(runs[short[0]]) + 1) if short else None
    for s, vs in runs.items():
        if len(vs) < crit.min_runs:
            return s, len(vs) + 1
    return None


def step(ledger: Sequence[Mapping], cls: str, start: float = 1.0) -> Tuple[Optional[float], Optional[str], int, str]:
    """Следующий прогон дихотомии класса: (скорость, сцена, повтор, пояснение); скорость None — граница найдена."""
    verdicts = point_verdicts(ledger, cls)
    top_pass = max((v for v, s in verdicts.items() if s == PASS), default=-1.0)
    for v in sorted(v for v in verdicts if v > top_pass):  # ниже проходящей точки доделывать нечего
        nr = next_run(ledger, cls, v)
        if nr:
            return v, nr[0], nr[1], "доделать точку"
    v, why = next_speed(verdicts, start=start)
    if v is None:
        return None, None, 0, why
    scene, rep = next_run(ledger, cls, v) or (class_scenes(cls)[0], 1)
    return v, scene, rep, why


# ── замер Main Pi ───────────────────────────────────────────────────────────


def parse_cpu_log(text: str) -> Dict:
    """Лог ``cpu_sampler`` (строки ``L <loadavg>`` и ``S <имя> <cpu%>``) → p50/p95/max по контейнерам и load."""
    load: List[float] = []
    cpu: Dict[str, List[float]] = {}
    for line in text.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0] == "L":
            load.append(_num(parts[1]))
        elif len(parts) >= 3 and parts[0] == "S":
            cpu.setdefault(parts[1], []).append(_num(parts[2].rstrip("%")))
    return {"load1": spread_stats(load), "cpu": {k: spread_stats(v) for k, v in cpu.items()}}


def _num(s: str) -> float:
    try:
        return float(s.replace(",", "."))
    except ValueError:
        return math.nan


# ── CLI ─────────────────────────────────────────────────────────────────────


def _cmd_run(a) -> int:
    tasks = json.loads(Path(a.result).read_text(encoding="utf-8")) if Path(a.result).exists() else []
    main = json.loads(Path(a.main).read_text(encoding="utf-8")) if Path(a.main).exists() else {}
    add_guard_rule(tasks, read_track(Path(a.result).with_name("track.csv")))
    verdict, reasons = run_verdict(tasks, main)
    # задачи с метрикой (б) — рядом с result.json, для таблицы
    Path(a.result).with_name("verdict.json").write_text(
        json.dumps({"verdict": verdict, "reasons": reasons, "tasks": tasks}, ensure_ascii=False, indent=1),
        encoding="utf-8",
    )
    print(f"VERDICT {a.scene} v={a.speed} rep={a.rep} {verdict}" + (": " + "; ".join(reasons) if reasons else ""))
    if a.ledger:
        row = {
            "speed": a.speed,
            "scene": a.scene,
            "rep": a.rep,
            "label": a.label,
            "verdict": verdict,
            "reasons": reasons,
        }
        with open(a.ledger, "a", encoding="utf-8") as f:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")
    return 0 if verdict == PASS else 1


def _read_ledger(path: str) -> List[Dict]:
    p = Path(path)
    if not p.exists():
        return []
    return [json.loads(line) for line in p.read_text(encoding="utf-8").splitlines() if line.strip()]


def _cmd_next(a) -> int:
    v, scene, rep, why = step(_read_ledger(a.ledger), a.cls, start=a.start)
    print(f"NEXT {a.cls} DONE - 0 {why}" if v is None else f"NEXT {a.cls} {v} {scene} {rep} {why}")
    return 0


def _cmd_cpu(a) -> int:
    print(json.dumps(parse_cpu_log(Path(a.log).read_text(encoding="utf-8", errors="replace")), ensure_ascii=False))
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("result")
    r.add_argument("main")
    r.add_argument("--speed", type=float, default=0.0)
    r.add_argument("--scene", default="?")
    r.add_argument("--rep", type=int, default=1)
    r.add_argument("--label", default="")
    r.add_argument("--ledger", default="")
    n = sub.add_parser("next")
    n.add_argument("--ledger", required=True)
    n.add_argument("--cls", choices=sorted(set(SCENE_CLASS.values())), required=True)
    n.add_argument("--start", type=float, default=1.0)
    c = sub.add_parser("cpu")
    c.add_argument("log")
    a = ap.parse_args(argv)
    return {"run": _cmd_run, "next": _cmd_next, "cpu": _cmd_cpu}[a.cmd](a)


if __name__ == "__main__":
    raise SystemExit(main())
