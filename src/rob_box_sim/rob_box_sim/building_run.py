"""Прогон «большая карта здания» в HIL (#4468, шаг 2): план прогона, разбор лога и вердикты кода. Без ROS.

Что решает этот модуль, а не оператор и не модель:

  * **порядок объезда** этажа ``building`` (``building_floor``): ближайший сосед по пути через коридор
    (:func:`corridor_cost`) от холла, в конце — назад в холл (замыкание петли для rtabmap);
  * **случайные старты** глобальной локализации (:func:`random_starts`): свободные позы этажа по сиду,
    попеременно в западной части (разные кабинеты) и в одинаковом восточном крыле («похожие места»);
  * **похищения** (:data:`KIDNAPS`) и цель телепорта «в такое же место» (:func:`similar_target`);
  * **вердикты** по логу шагов (``scripts/hil/video/building_step.py``) и ``maprec/poses.csv``:
    «доехал» (:func:`arrival`), «нашёл себя» (:func:`fix_result`), «ложный скачок» (:func:`jump_events`),
    дрейф (:func:`drift_stats`) и строки ``VERDICT #4468 <блок> <статус>: …`` (:func:`evaluate`).

Локализационная часть (нашёл себя, скачки, дрейф) стоит на ``rob_box_sim.loc_metrics`` стенда #4346
(PR #4568): истина Gazebo против оценки ``map → base_footprint``. Пока его нет в чекауте, эти функции
падают с понятной ошибкой (:func:`_loc`), а не считают «как получится».

Кадры: ``map`` = мир. Карту снимаем со спавна (0, 0, 0), начало ``map`` rtabmap кладёт в спавн
(``building_floor``, runbook hil-video §1 п. 6), поэтому истина Gazebo и оценка сравниваются без пересчёта.

    python -m rob_box_sim.building_run plan --starts 6         # порядок, старты, длина, оценка времени
    python -m rob_box_sim.building_run tour                    # имена целей объезда через пробел
    python -m rob_box_sim.building_run starts --starts 6       # «x y yaw°» по строке
    python -m rob_box_sim.building_run nearest X Y             # ближайшая точка-кабинет по пути
    python -m rob_box_sim.building_run label dean              # название точки для голоса и реестра
    python -m rob_box_sim.building_run verdicts OUT_DIR        # VERDICT-строки по OUT_DIR/steps/*.log и maprec
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import re
import statistics
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Sequence

from rob_box_sim.coverage_metric import GoalOutcome
from rob_box_sim.geometry2d import Box, Pose2D, clearance

from rob_box_sim import building_floor as bf

ISSUE = "#4468"
HOME = "hall"  # точка спавна: (0, 0) мира = начало map


# ── Пороги вердиктов ────────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class Criteria:
    """Пороги «доехал / нашёл себя / ложный скачок». Числа — не из прогона здания (его ещё не было)."""

    # доехал: истина Gazebo не дальше arrive_m от точки кабинета. Nav2 xy_goal_tolerance 0.25 м по оценке
    # (docker/main/config/nav2/nav2_params.yaml) + ошибка оценки (Depot с Odom/ResetCountdown: 0.17–0.34 м).
    arrive_m: float = 0.5
    # нашёл себя: ошибка оценки держится в found_pos_m / found_yaw_deg не меньше found_hold_s,
    # и случилось это не позже found_budget_s sim-секунд после события (старт стека, телепорт).
    found_pos_m: float = 0.35
    found_yaw_deg: float = 10.0
    found_hold_s: float = 5.0
    found_budget_s: float = 120.0
    final_window_s: float = 5.0
    # скачок: за шаг записи (10 Гц) оценка сдвинулась сверх движения истины больше jump_m;
    # ложный — после него ошибка больше false_m и выросла (узнал «не то место»).
    jump_m: float = 0.5
    false_m: float = 1.0
    # шаг истины больше kidnap_m — это телепорт, а не скачок оценки
    kidnap_m: float = 0.5
    # между замерами с оценкой дольше max_gap_s — оценки не было (рестарт), новая — не скачок
    max_gap_s: float = 1.0

    def tolerance(self, lm):
        """Те же пороги в виде ``loc_metrics.Tolerance`` стенда #4346."""
        return lm.Tolerance(
            pos_tol_m=self.found_pos_m,
            yaw_tol_deg=self.found_yaw_deg,
            hold_s=self.found_hold_s,
            final_window_s=self.final_window_s,
            jump_m=self.jump_m,
            kidnap_m=self.kidnap_m,
        )


def _loc():
    """``rob_box_sim.loc_metrics`` (PR #4568, #4346) или понятный отказ: без него «нашёл себя» не посчитать."""
    try:
        from rob_box_sim import loc_metrics
    except ImportError as exc:  # pragma: no cover — ветка «нет #4568» проверяется тестом через monkeypatch
        raise RuntimeError(
            "локализационные вердикты (#4468) считает rob_box_sim.loc_metrics стенда #4346 (PR #4568); "
            "в этом чекауте его нет — влей #4568 или подставь его ветку в PYTHONPATH"
        ) from exc
    return loc_metrics


# ── План: порядок объезда, старты, похищения ─────────────────────────────────────────────────


def points_by_name() -> dict[str, bf.NamedPoint]:
    return {p.name: p for p in bf.named_points()}


def _in_hall(p) -> bool:
    return p.x < bf.CORRIDOR_X0


def corridor_cost(a, b) -> float:
    """Длина пути a → b по этажу (оценка): из кабинета в коридор (|y|), вдоль коридора (|Δx|), в кабинет.

    Холл (x < ``CORRIDOR_X0``) открытый — внутри него по прямой; из холла в коридор — через его начало (x0, 0).
    """
    if _in_hall(a) and _in_hall(b):
        return math.hypot(a.x - b.x, a.y - b.y)
    if _in_hall(a) or _in_hall(b):
        hall, other = (a, b) if _in_hall(a) else (b, a)
        x0 = bf.CORRIDOR_X0
        return math.hypot(hall.x - x0, hall.y) + abs(other.x - x0) + abs(other.y)
    return abs(a.y) + abs(a.x - b.x) + abs(b.y)


def nearest_neighbour(points: Sequence, start) -> list:
    """Обход ближайшим соседом по :func:`corridor_cost` от ``start``; при равной цене — порядок ``points``."""
    left = list(points)
    order, here = [], start
    while left:
        nxt = min(left, key=lambda p: corridor_cost(here, p))
        order.append(nxt)
        left.remove(nxt)
        here = nxt
    return order


def explore_tour() -> list[str]:
    """Объезд для картографии: все точки этажа ближайшим соседом от холла, в конце — назад в холл."""
    pts = points_by_name()
    rest = [p for name, p in pts.items() if name != HOME]
    return [p.name for p in nearest_neighbour(rest, pts[HOME])] + [HOME]


def tour_length(names: Sequence[str], start: str = HOME) -> float:
    pts = points_by_name()
    seq = [pts[start]] + [pts[n] for n in names]
    return sum(corridor_cost(a, b) for a, b in zip(seq, seq[1:]))


def nearest_point(x: float, y: float) -> str:
    """Ближайшая по пути точка-кабинет (имя) к позе (x, y): куда ехать после случайного старта."""
    here = bf.NamedPoint("here", "", x, y, 0.0)
    return min(bf.named_points(), key=lambda p: corridor_cost(here, p)).name


def similar_span() -> tuple[float, float]:
    """Участок одинаковых кабинетов (восточное крыло): от западной стены первого до восточной последнего."""
    rooms = [r for r in bf.ROOMS if r.id.startswith("room_")]
    return min(r.x0 for r in rooms), max(r.x1 for r in rooms)


def in_similar(x: float) -> bool:
    lo, hi = similar_span()
    return lo <= x <= hi


def _free(pose: Pose2D, boxes: Sequence[Box], min_clear: float) -> bool:
    return clearance(pose, boxes) >= min_clear


def random_starts(
    n: int, seed: int = 4468, min_clear: float = 0.35, min_sep: float = 6.0, boxes: Sequence[Box] | None = None
) -> list[Pose2D]:
    """``n`` свободных поз этажа по сиду: чётные — в западной части (холл, разные кабинеты), нечётные — в
    одинаковом восточном крыле. Зазор корпуса до стен и мебели ≥ ``min_clear``; до других стартов и до спавна
    (0, 0) — начала ``map`` — не меньше ``min_sep``: старт у начала карты глобальную локализацию не проверяет."""
    boxes = list(bf.floor_boxes()) if boxes is None else list(boxes)
    rng = random.Random(seed)
    lo, hi = similar_span()
    zones = ((bf.X_WEST + 0.5, lo - 0.5), (lo + 0.5, bf.X_EAST - 0.5))
    taken: list[Pose2D] = [Pose2D(0.0, 0.0, 0.0)]
    for i in range(n):
        x0, x1 = zones[i % 2]
        for _ in range(20000):
            p = Pose2D(
                round(rng.uniform(x0, x1), 2),
                round(rng.uniform(-bf.Y_OUT + 0.5, bf.Y_OUT - 0.5), 2),
                round(rng.uniform(-math.pi, math.pi), 3),
            )
            if _free(p, boxes, min_clear) and all(p.distance_to(q) >= min_sep for q in taken):
                taken.append(p)
                break
        else:
            raise ValueError(f"старт {i + 1}: не нашлось свободной позы (зазор {min_clear} м, разнос {min_sep} м)")
    return taken[1:]


@dataclass(frozen=True)
class Kidnap:
    """Похищение: доехать до ``approach``, поехать к ``goal``, через ``after_s`` sim-с — телепорт.

    Куда: ``to`` (x, y, yaw°) или ``similar`` — на столько кабинетов (по 4 м) вдоль коридора в том же крыле
    (:func:`similar_target`): тот же вид, другое место.
    """

    name: str
    approach: str
    goal: str
    after_s: float
    to: tuple[float, float, float] | None = None
    similar: int = 0


KIDNAPS: tuple[Kidnap, ...] = (
    # одинаковое восточное крыло: телепорт на два кабинета (8 м) — проверка ложного «узнавания»
    Kidnap("kidnap_similar", approach="room_203", goal="room_210", after_s=15.0, similar=2),
    # из западного крыла в холл: место другое, вопрос — заметит ли и найдёт ли себя
    Kidnap("kidnap_far", approach="dean", goal="library", after_s=15.0, to=(-2.0, 2.0, 90.0)),
)


def similar_target(x: float, y: float, yaw: float, periods: int, boxes: Sequence[Box] | None = None) -> Pose2D:
    """Поза «в таком же месте»: сдвиг на ``periods`` кабинетов вдоль X внутри восточного крыла, туда, где
    свободно (сначала на восток, потом на запад). Нет такой — ``ValueError`` (похищение не выдумываем)."""
    boxes = list(bf.floor_boxes()) if boxes is None else list(boxes)
    lo, hi = similar_span()
    step = 4.0 * periods
    for dx in (step, -step):
        p = Pose2D(round(x + dx, 3), y, yaw)
        if lo + 0.5 <= p.x <= hi - 0.5 and _free(p, boxes, 0.2):
            return p
    raise ValueError(f"нет свободной позы в {step:g} м от ({x:.2f}, {y:.2f}) внутри крыла {lo:g}…{hi:g}")


def set_pose_request(model: str, x: float, y: float, yaw: float, z: float = 0.05) -> str:
    """Тело запроса ``ign service …/set_pose`` (ignition.msgs.Pose), yaw в радианах."""
    qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
    return (
        f'name: "{model}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.3f}}}, '
        f"orientation: {{z: {qz:.6f}, w: {qw:.6f}}}"
    )


@dataclass(frozen=True)
class Pace:
    """Темп для оценки времени [оценка, на здании не мерено]: max_vel_x Nav2 0.15 м/с, с разгоном и поворотами
    берём 0.10; RTF 1.0 при CAMERAS=0 (runbook hil-video §3, замер 09.11)."""

    speed_mps: float = 0.10
    per_goal_s: float = 15.0
    restart_s: float = 90.0  # docker stop до 60 с (база rtabmap пишется на стопе) + подъём
    probe_s: float = 45.0  # поворот 360° на 0.3 рад/с + выдержка
    voice_s: float = 300.0
    rtf: float = 1.0


def plan_estimate(n_starts: int, voice: int = 0, pace: Pace = Pace()) -> dict:
    """Длина и время прогона по блокам, минуты стены. Это оценка по плану, не замер."""
    tour = explore_tour()
    cabinets = len(tour)
    drive = tour_length(tour) / pace.speed_mps + cabinets * pace.per_goal_s
    restart = pace.restart_s + pace.probe_s
    per_start = 5.0 + restart + 12.0 / pace.speed_mps + pace.per_goal_s  # до ближайшей точки ~12 м [оценка]
    per_kidnap = 2 * (25.0 / pace.speed_mps + pace.per_goal_s) + pace.probe_s
    blocks = {
        "map": drive + restart,
        "route": drive + cabinets * 5.0,  # + save_waypoint на каждую точку
        "loc": restart + n_starts * per_start + len(KIDNAPS) * per_kidnap,
        "voice": voice * pace.voice_s,
    }
    out = {k: round(v / pace.rtf / 60.0, 1) for k, v in blocks.items()}
    out["total"] = round(sum(blocks.values()) / pace.rtf / 60.0, 1)
    out["tour_m"] = round(tour_length(tour), 1)
    out["goals"] = cabinets
    return out


def floor_grid(res: float = 0.1, pad: float = 1.0) -> dict:
    """Стены и мебель этажа растром OccupancyGrid (100 — занято): фон анимации из той же геометрии, что SDF.

    Формат — снимок ``maprec`` (``t, resolution, origin_x, origin_y, data``), его рисует ``pose_anim.map_layer``.
    """
    boxes = [b for b in bf.floor_boxes()]
    x0 = min(b.cx - b.sx / 2 for b in boxes) - pad
    y0 = min(b.cy - b.sy / 2 for b in boxes) - pad
    x1 = max(b.cx + b.sx / 2 for b in boxes) + pad
    y1 = max(b.cy + b.sy / 2 for b in boxes) + pad
    w, h = int(math.ceil((x1 - x0) / res)), int(math.ceil((y1 - y0) / res))
    data = [[0] * w for _ in range(h)]
    for b in boxes:
        c0, c1 = int((b.cx - b.sx / 2 - x0) / res), int(math.ceil((b.cx + b.sx / 2 - x0) / res))
        r0, r1 = int((b.cy - b.sy / 2 - y0) / res), int(math.ceil((b.cy + b.sy / 2 - y0) / res))
        for r in range(max(0, r0), min(h, r1)):
            data[r][max(0, c0) : min(w, c1)] = [100] * (min(w, c1) - max(0, c0))  # noqa: E203
    return {"t": 0.0, "resolution": res, "origin_x": x0, "origin_y": y0, "data": data}


# ── Лог шагов: события ────────────────────────────────────────────────────────────────────────

_EVENT_RE = re.compile(r"\bEVENT (\w+)((?: \w+=\S+)*)")
_KV_RE = re.compile(r"(\w+)=(\S+)")
_GOAL_END_RE = re.compile(
    r"\bGOAL_END (\S+) result=(\S+) sim=(\S+) frame=\S+ dur_sim=(\S+) err=(\S+) em_episodes=(\d+) recoveries=(\d+)"
)


def event_line(event: str, sim: float, **kv) -> str:
    """``EVENT <ВИД> sim=<t> k=v …``: значения без пробелов (пробел → «_»), вещественные — 3 знака.

    Вид события — позиционный ``event``: ключ ``kind=`` у ``SEG_START`` — это вид отрезка, а не события.
    """

    def fmt(v) -> str:
        return f"{v:.3f}" if isinstance(v, float) else str(v).replace(" ", "_")

    tail = "".join(f" {k}={fmt(v)}" for k, v in kv.items())
    return f"EVENT {event} sim={sim:.3f}{tail}"


@dataclass(frozen=True)
class Event:
    kind: str
    sim: float
    kv: dict = field(default_factory=dict)

    def get(self, key: str, default=None):
        return self.kv.get(key, default)

    def num(self, key: str) -> float | None:
        try:
            return float(self.kv[key])
        except (KeyError, ValueError):
            return None


@dataclass(frozen=True)
class GoalEnd:
    """Строка ``GOAL_END`` ``route.py``: исход Nav2 и ошибка истины до цели."""

    name: str
    outcome: GoalOutcome
    sim: float
    dur_s: float
    err_m: float
    recoveries: int


def parse_events(lines: Iterable[str]) -> list[Event]:
    out = []
    for line in lines:
        m = _EVENT_RE.search(line)
        if not m:
            continue
        kv = dict(_KV_RE.findall(m.group(2)))
        try:
            sim = float(kv.pop("sim"))
        except (KeyError, ValueError):
            continue
        out.append(Event(m.group(1), sim, kv))
    return out


def parse_goal_ends(lines: Iterable[str]) -> list[GoalEnd]:
    out = []
    for line in lines:
        m = _GOAL_END_RE.search(line)
        if not m:
            continue
        name, raw, sim, dur, err, em, rec = m.groups()
        out.append(
            GoalEnd(name, GoalOutcome.from_route(name, raw, int(em)), float(sim), float(dur), float(err), int(rec))
        )
    return out


@dataclass(frozen=True)
class Segment:
    """Отрезок прогона между ``SEG_START`` и ``SEG_END``: kind — map, restart, route, start, kidnap, voice."""

    name: str
    kind: str
    t0: float
    t1: float
    start: Event
    events: tuple[Event, ...]
    goals: tuple[GoalEnd, ...]

    def first(self, kind: str, name: str | None = None) -> Event | None:
        for e in self.events:
            if e.kind == kind and (name is None or e.get("name") == name):
                return e
        return None


def segments(events: Sequence[Event], goals: Sequence[GoalEnd]) -> list[Segment]:
    """Отрезки по меткам; незакрытый (прогон оборвался) кончается последним событием или целью."""
    last = max([e.sim for e in events] + [g.sim for g in goals] + [0.0])
    starts = [e for e in events if e.kind == "SEG_START"]
    ends = {e.get("name"): e.sim for e in events if e.kind == "SEG_END"}
    out = []
    for s in starts:
        name = s.get("name")
        t1 = ends.get(name, last)
        inner = tuple(e for e in events if s.sim <= e.sim <= t1 and e.kind not in ("SEG_START", "SEG_END"))
        gs = tuple(g for g in goals if s.sim <= g.sim <= t1)
        out.append(Segment(name, s.get("kind", "?"), s.sim, t1, s, inner, gs))
    return out


# ── Вердикты: доехал ─────────────────────────────────────────────────────────────────────────

ARRIVED, FALSE_SUCCESS, FAILED = "ARRIVED", "FALSE_SUCCESS", "FAILED"


def arrival(goal: GoalEnd, crit: Criteria = Criteria()) -> str:
    """ARRIVED — Nav2 «дошёл» и истина у точки; FALSE_SUCCESS — Nav2 «дошёл», а робот не там; FAILED — не дошёл."""
    if not goal.outcome.succeeded:
        return FAILED
    return ARRIVED if goal.err_m <= crit.arrive_m else FALSE_SUCCESS


# ── Вердикты: нашёл себя, скачки, дрейф (loc_metrics #4346) ─────────────────────────────────────


def read_poses_csv(lines: Iterable[str]) -> list:
    """``maprec/poses.csv`` (``t,tx,ty,tyaw,ex,ey,eyaw``, yaw в радианах) → ``loc_metrics.Sample``."""
    lm = _loc()

    def pose(row, keys):
        try:
            return tuple(float(row[k]) for k in keys)
        except (KeyError, TypeError, ValueError):
            return None

    out = []
    for row in csv.DictReader(lines):
        try:
            t = float(row["t"])
        except (KeyError, TypeError, ValueError):
            continue
        out.append(lm.Sample(t, pose(row, ("tx", "ty", "tyaw")), pose(row, ("ex", "ey", "eyaw"))))
    out.sort(key=lambda s: s.t)
    return out


def _window(samples: Sequence, t0: float, t1: float) -> list:
    return [s for s in samples if t0 <= s.t <= t1]


def _paired(samples: Sequence) -> list:
    return [s for s in samples if s.truth is not None and s.estimate is not None]


FOUND, LATE, NOT_FOUND, WRONG, NO_ESTIMATE = "FOUND", "LATE", "NOT_FOUND", "WRONG", "NO_ESTIMATE"


@dataclass(frozen=True)
class Fix:
    """Нашёл ли себя после события: статус, ошибка первой оценки и в конце, за сколько секунд и метров."""

    status: str
    err0_m: float | None = None
    err_final_m: float | None = None
    found_s: float | None = None
    found_path_m: float | None = None

    def facts(self) -> str:
        def r(v, nd=2):
            return "—" if v is None else f"{v:.{nd}f}"

        return (
            f"{self.status} err0={r(self.err0_m)}м found_s={r(self.found_s, 1)} "
            f"path={r(self.found_path_m, 1)}м err_end={r(self.err_final_m)}м"
        )


def fix_result(samples: Sequence, t_event: float, t_end: float, crit: Criteria = Criteria()) -> Fix:
    """FOUND — сошлось в допуск не позже бюджета; LATE — позже; WRONG — до конца не сошлось и в конце ошибка
    больше ``false_m`` (уверенно «не там»); NOT_FOUND — не сошлось; NO_ESTIMATE — оценки не было вовсе."""
    lm = _loc()
    tol = crit.tolerance(lm)
    win = _window(samples, t_event, t_end)
    paired = _paired(win)
    if not paired:
        return Fix(NO_ESTIMATE)
    err0 = lm.pose_error(paired[0].truth, paired[0].estimate)[0]
    tail = [lm.pose_error(s.truth, s.estimate)[0] for s in paired if s.t >= t_end - crit.final_window_s]
    final = statistics.median(tail) if tail else None
    conv = lm.converged_at(win, t_event, tol)
    if conv is None:
        status = WRONG if final is not None and final > crit.false_m else NOT_FOUND
        return Fix(status, err0, final)
    found_s = conv - t_event
    path = lm.path_length(win, t_event, conv)
    return Fix(FOUND if found_s <= crit.found_budget_s else LATE, err0, final, found_s, path)


@dataclass(frozen=True)
class Jump:
    """Скачок оценки: где (истина), ошибка до и после, вид — false (узнал не то место), fix (вернулся), other."""

    t: float
    x: float
    y: float
    err_before: float
    err_after: float
    kind: str

    @property
    def similar(self) -> bool:
        return in_similar(self.x)


def _jump_kind(e0: float, e1: float, crit: Criteria) -> str:
    if e1 > crit.false_m and e1 > e0:
        return "false"
    if e1 <= crit.found_pos_m < e0:
        return "fix"
    return "other"


def jump_events(samples: Sequence, t0: float, t1: float, crit: Criteria = Criteria()) -> list[Jump]:
    """Скачки оценки на [t0, t1]. Не скачок: шаг истины больше ``kidnap_m`` (телепорт) и первая оценка после
    разрыва дольше ``max_gap_s`` (рестарт стека — её судит :func:`fix_result`)."""
    lm = _loc()
    pairs = _paired(_window(samples, t0, t1))
    out = []
    for a, b in zip(pairs, pairs[1:]):
        dtx, dty = b.truth[0] - a.truth[0], b.truth[1] - a.truth[1]
        if b.t - a.t > crit.max_gap_s or math.hypot(dtx, dty) > crit.kidnap_m:
            continue
        dex, dey = b.estimate[0] - a.estimate[0], b.estimate[1] - a.estimate[1]
        if math.hypot(dex - dtx, dey - dty) <= crit.jump_m:
            continue
        e0 = lm.pose_error(a.truth, a.estimate)[0]
        e1 = lm.pose_error(b.truth, b.estimate)[0]
        out.append(Jump(b.t, b.truth[0], b.truth[1], round(e0, 3), round(e1, 3), _jump_kind(e0, e1, crit)))
    return out


def drift_stats(samples: Sequence, t0: float, t1: float) -> dict:
    """Ошибка позы истина против map на [t0, t1]: средняя, 95-й процентиль, максимум и путь по истине."""
    lm = _loc()
    win = _window(samples, t0, t1)
    errs = sorted(lm.pose_error(s.truth, s.estimate)[0] for s in _paired(win))
    if not errs:
        return {"samples": 0}
    p95 = errs[min(len(errs) - 1, int(round(0.95 * (len(errs) - 1))))]
    return {
        "samples": len(errs),
        "mean_m": round(statistics.fmean(errs), 3),
        "p95_m": round(p95, 3),
        "max_m": round(errs[-1], 3),
        "path_m": round(lm.path_length(win, t0, t1), 1),
    }


def voice_arrival(samples: Sequence, seg: Segment, crit: Criteria = Criteria()) -> str:
    """Голосовой сценарий: ARRIVED — в конце отрезка истина у точки ``target``; NOT_TRIGGERED — робот не поехал
    (модель не позвала тул: путь < 0.5 м); иначе FAILED."""
    lm = _loc()
    win = [s for s in _window(samples, seg.t0, seg.t1) if s.truth is not None]
    target = points_by_name().get(seg.start.get("target", ""))
    if target is None or not win:
        return "NOT_TRIGGERED"
    if lm.path_length(win, seg.t0, seg.t1) < 0.5:
        return "NOT_TRIGGERED"
    last = win[-1].truth
    return ARRIVED if math.hypot(last[0] - target.x, last[1] - target.y) <= crit.arrive_m else FAILED


# ── Вердикты блоков ───────────────────────────────────────────────────────────────────────────

PASS, FAIL, NOT_TRIGGERED = "PASS", "FAIL", "NOT_TRIGGERED"


@dataclass(frozen=True)
class Verdict:
    block: str
    status: str
    facts: tuple[str, ...]

    def line(self) -> str:
        return f"VERDICT {ISSUE} {self.block} {self.status}: " + "; ".join(self.facts)


def _pct(k: int, n: int) -> str:
    return f"{k}/{n} ({100.0 * k / n:.0f} %)" if n else "0/0"


def route_verdict(block: str, goals: Sequence[GoalEnd], crit: Criteria = Criteria()) -> Verdict:
    """Доля доездов, время в пути, стопы защиты и восстановления. PASS — доехал до каждой точки."""
    if not goals:
        return Verdict(block, NOT_TRIGGERED, ("целей не было",))
    kinds = [arrival(g, crit) for g in goals]
    ok = kinds.count(ARRIVED)
    durs = sorted(g.dur_s for g in goals)
    facts = [
        f"доехал {_pct(ok, len(goals))}",
        f"время цели медиана {statistics.median(durs):.0f} с, макс {durs[-1]:.0f} с, всего {sum(durs) / 60:.1f} мин",
        f"стопов защиты {sum(g.outcome.em_episodes for g in goals)}, восстановлений {sum(g.recoveries for g in goals)}",
    ]
    for kind in (FALSE_SUCCESS, FAILED):
        bad = [f"{g.name}({g.outcome.result},{g.err_m:.2f}м)" for g, k in zip(goals, kinds) if k == kind]
        if bad:
            facts.append(f"{kind}: " + " ".join(bad))
    return Verdict(block, PASS if ok == len(goals) else FAIL, tuple(facts))


def _event_time(seg: Segment) -> float:
    """Момент события отрезка: телепорт (похищение) или конец рестарта стека; нет метки — начало отрезка."""
    if seg.kind == "kidnap":
        tp = seg.first("TELEPORT")
        return tp.sim if tp is not None else seg.t0
    mark = seg.first("MARK", "restart_end")
    return mark.sim if mark is not None else seg.t0


def fix_verdict(block: str, segs: Sequence[Segment], samples: Sequence, crit: Criteria = Criteria()) -> Verdict:
    """Нашёл ли себя в каждом отрезке (рестарт, старт из случайной точки, похищение). PASS — FOUND везде."""
    if not segs:
        return Verdict(block, NOT_TRIGGERED, ("отрезков не было",))
    facts, found = [], 0
    for seg in segs:
        fix = fix_result(samples, _event_time(seg), seg.t1, crit)
        found += fix.status == FOUND
        where = ""
        if seg.start.get("x") is not None:
            where = f" @({seg.start.get('x')},{seg.start.get('y')})" + (
                " похожее" if in_similar(seg.start.num("x")) else ""
            )
        tail = ""
        if seg.goals:
            g = seg.goals[-1]
            tail = f" затем {g.name}:{arrival(g, crit)}"
        facts.append(f"{seg.name}{where}: {fix.facts()}{tail}")
    facts.insert(
        0,
        f"нашёл себя {_pct(found, len(segs))} (допуск {crit.found_pos_m} м/{crit.found_yaw_deg}°, "
        f"бюджет {crit.found_budget_s:.0f} с)",
    )
    return Verdict(block, PASS if found == len(segs) else FAIL, tuple(facts))


def drift_verdict(segs: Sequence[Segment], samples: Sequence, crit: Criteria = Criteria()) -> Verdict:
    """Дрейф на длинном маршруте по кабинетам: PASS — максимум ошибки не больше ``false_m``."""
    if not segs:
        return Verdict("drift", NOT_TRIGGERED, ("маршрута не было",))
    facts, worst = [], 0.0
    for seg in segs:
        st = drift_stats(samples, seg.t0, seg.t1)
        if not st.get("samples"):
            facts.append(f"{seg.name}: оценки нет")
            worst = math.inf
            continue
        worst = max(worst, st["max_m"])
        facts.append(
            f"{seg.name}: путь {st['path_m']} м, ошибка средняя {st['mean_m']} м, "
            f"p95 {st['p95_m']} м, макс {st['max_m']} м"
        )
    return Verdict("drift", PASS if worst <= crit.false_m else FAIL, tuple(facts))


def similar_verdict(segs: Sequence[Segment], samples: Sequence, crit: Criteria = Criteria()) -> Verdict:
    """Ложные узнавания по всем отрезкам: PASS — ни одного скачка вида false."""
    jumps = [(seg.kind, j) for seg in segs for j in jump_events(samples, seg.t0, seg.t1, crit)]
    if not segs:
        return Verdict("similar", NOT_TRIGGERED, ("отрезков не было",))
    false = [(k, j) for k, j in jumps if j.kind == "false"]
    facts = [
        f"скачков оценки {len(jumps)}: ложных {len(false)} "
        f"(из них в одинаковом крыле {sum(j.similar for _, j in false)}), "
        f"возвратов {sum(j.kind == 'fix' for _, j in jumps)}"
    ]
    facts += [
        f"{k} t={j.t:.1f} @({j.x:.1f},{j.y:.1f}) ошибка {j.err_before:.2f}→{j.err_after:.2f} м" for k, j in false[:10]
    ]
    return Verdict("similar", PASS if not false else FAIL, tuple(facts))


def voice_verdict(segs: Sequence[Segment], samples: Sequence, crit: Criteria = Criteria()) -> Verdict:
    if not segs:
        return Verdict("voice", NOT_TRIGGERED, ("голосовой сценарий выключен (BUILDING_VOICE=0)",))
    kinds = [voice_arrival(samples, seg, crit) for seg in segs]
    facts = [f"{seg.name}→{seg.start.get('target')}: {k}" for seg, k in zip(segs, kinds)]
    if all(k == ARRIVED for k in kinds):
        status = PASS
    elif any(k == FAILED for k in kinds):
        status = FAIL
    else:
        status = NOT_TRIGGERED
    return Verdict("voice", status, tuple([f"доехал {_pct(kinds.count(ARRIVED), len(kinds))}"] + facts))


def parse_tool_results(lines: Iterable[str]) -> list[tuple[str, bool, str]]:
    """``tools.log`` оркестра: ответы ``/mcp/result`` одной строкой JSON → (тул, успех, сообщение или ошибка).

    Строка без ответа (``TOOL_FAILED``, таймаут ``{"error": …}``) — неуспех: «тул молчал» не равно «тул сделал».
    """
    out, pending = [], None
    for line in lines:
        head = re.match(r"^\S+ TOOL (\w+) ", line)
        if head:
            if pending:
                out.append((pending, False, "ответа нет"))
            pending = head.group(1)
            continue
        if pending is None or not (line.startswith("{") or line.startswith("TOOL_FAILED")):
            continue
        try:
            payload = json.loads(line) if line.startswith("{") else {"error": line.strip()}
        except ValueError:
            continue
        res = payload.get("result") or {}
        ok = bool(res.get("success")) and "error" not in payload
        out.append((pending, ok, str(res.get("message") or res.get("error") or payload.get("error") or "")))
        pending = None
    if pending:
        out.append((pending, False, "ответа нет"))
    return out


def registry_verdict(results: Sequence[tuple[str, bool, str]]) -> Verdict:
    """Карта и точки-кабинеты в HIL-реестре: PASS — каждый вызов тула успешен (по ответу mcp_server, не по нам)."""
    if not results:
        return Verdict("registry", NOT_TRIGGERED, ("тулы реестра не вызывались (BUILDING_REGISTRY=0)",))
    by: dict[str, list[bool]] = {}
    for name, ok, _ in results:
        by.setdefault(name, []).append(ok)
    facts = [f"{name} {sum(oks)}/{len(oks)}" for name, oks in by.items()]
    facts += [f"{name}: {msg[:80]}" for name, ok, msg in results if not ok][:10]
    return Verdict("registry", PASS if all(ok for _, ok, _ in results) else FAIL, tuple(facts))


def evaluate(
    events: Sequence[Event],
    goals: Sequence[GoalEnd],
    samples: Sequence,
    crit: Criteria = Criteria(),
    tool_results: Sequence[tuple[str, bool, str]] = (),
):
    """Все вердикты прогона по порядку блоков приёмки #4468."""
    segs = segments(events, goals)

    def of(*kinds):
        return [s for s in segs if s.kind in kinds]

    map_goals = [g for s in of("map") for g in s.goals]
    route_goals = [g for s in of("route") for g in s.goals]
    return [
        route_verdict("map", map_goals, crit),
        registry_verdict(tool_results),
        route_verdict("route", route_goals, crit),
        drift_verdict(of("route"), samples, crit),
        fix_verdict("restart", of("restart"), samples, crit),
        fix_verdict("global_loc", of("start"), samples, crit),
        similar_verdict(segs, samples, crit),
        fix_verdict("kidnap", of("kidnap"), samples, crit),
        voice_verdict(of("voice"), samples, crit),
    ]


# ── CLI ──────────────────────────────────────────────────────────────────────────────────────


def _read_lines(paths: Iterable[Path]) -> list[str]:
    lines: list[str] = []
    for p in sorted(paths):
        lines += p.read_text(encoding="utf-8", errors="replace").splitlines()
    return lines


def load_run(out: Path) -> tuple[list[Event], list[GoalEnd], list, list]:
    """Каталог прогона: ``steps/*.log`` (события и GOAL_END), ``maprec*/poses.csv`` (истина и оценка) и
    ``tools.log`` (ответы тулов реестра)."""
    lines = _read_lines(out.glob("steps/*.log"))
    samples = []
    for csv_path in sorted(out.glob("maprec*/poses.csv")):
        with open(csv_path, encoding="utf-8", errors="replace", newline="") as fh:
            samples += read_poses_csv(fh)
    samples.sort(key=lambda s: s.t)
    tools = parse_tool_results(_read_lines([out / "tools.log"]) if (out / "tools.log").is_file() else [])
    return parse_events(lines), parse_goal_ends(lines), samples, tools


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("cmd", choices=("plan", "tour", "starts", "nearest", "label", "verdicts"))
    ap.add_argument("args", nargs="*")
    ap.add_argument("--starts", type=int, default=6)
    ap.add_argument("--seed", type=int, default=4468)
    ap.add_argument("--voice", type=int, default=0)
    a = ap.parse_args(argv)
    if a.cmd == "tour":
        print(" ".join(explore_tour()))
    elif a.cmd == "starts":
        for p in random_starts(a.starts, a.seed):
            print(f"{p.x:.2f} {p.y:.2f} {math.degrees(p.yaw):.1f}")
    elif a.cmd == "nearest":
        print(nearest_point(float(a.args[0]), float(a.args[1])))
    elif a.cmd == "label":
        print(points_by_name()[a.args[0]].label)
    elif a.cmd == "plan":
        est = plan_estimate(a.starts, a.voice)
        print("TOUR " + " ".join(explore_tour()))
        for i, p in enumerate(random_starts(a.starts, a.seed), 1):
            print(f"START {i} x={p.x:.2f} y={p.y:.2f} yaw={math.degrees(p.yaw):.0f} similar={int(in_similar(p.x))}")
        for k in KIDNAPS:
            to = "-" if k.to is None else ",".join(f"{v:g}" for v in k.to)
            print(
                f"KIDNAP {k.name} approach={k.approach} goal={k.goal} after={k.after_s:g}s to={to} similar={k.similar}"
            )
        print("ESTIMATE " + json.dumps(est, ensure_ascii=False))
    else:
        events, goals, samples, tools = load_run(Path(a.args[0]))
        for v in evaluate(events, goals, samples, tool_results=tools):
            print(v.line())
    return 0


if __name__ == "__main__":
    sys.exit(main())
