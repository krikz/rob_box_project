// Nav2 симулятора мостика (issue #3149 × #3151): планировщик на «SLAM»-
// решётке, pure-pursuit и жизненный цикл цели — чтобы nav-слой клиента
// (цель лучом, путь на полу, строка NAV, отмена) проверялся без робота.
//
// Зеркало серверной стороны (rob_box_quest):
//   * nav_goal → nav_goal_ack | nav_goal_nack{reason} (ws_server.py
//     _json_cmd_nav_goal; причины — core/nav_goal.py NACK_*);
//   * nav_status {state, seq, x, y, yaw, distance_remaining?, reason?, ts_ms}:
//     accepted → active (≤ 2 Гц, NAV_FEEDBACK_MIN_PERIOD_S) → succeeded /
//     canceled / aborted (nav2_goal.py Nav2GoalBridge);
//   * nav_cancel → nav_cancel_ack{had_goal} + nav_status{canceled};
//   * stop_emergency отменяет цель (quest_node.emergency_stop, #3151);
//   * новая цель вытесняет старую МОЛЧА — статус старой не шлётся
//     (Nav2GoalBridge._current(token) → None);
//   * nav_path — план от робота до цели, пересчёт раз в 1 с (BT
//     RateController hz=1.0), пустой путь (n = 0) по завершении цели.
//
// Телеоп НЕ отменяет цель, а перебивает её (twist_mux: quest 90 > nav 10):
// пока идут свежие teleop_twist, навигатор молчит, потом едет дальше с
// пересчётом пути. Так ведёт себя настоящий робот — отмена только
// аварийным стопом или nav_cancel.
//
// Чистый модуль (без DOM/Three.js): решётка SimGrid, поза SimRobotState.

import type { SimGrid } from "./sim_world";
import { ROBOT_RADIUS_M } from "./sim_world";

export interface Xy {
  x: number;
  y: number;
}

// ────────────────────────── планировщик ──────────────────────────

/** Запас к радиусу робота при раздувании препятствий, м. */
export const NAV_INFLATION_MARGIN_M = 0.07;
/** Шаг точек пути после сглаживания, м (как у NavFn — плотно). */
export const NAV_PATH_SPACING_M = 0.1;
/** Максимум точек в кадре nav_path (streams/nav_path.py NAV_PATH_MAX_POINTS). */
export const NAV_PATH_MAX_POINTS = 200;
/** Цена шага по неизвестной клетке (Nav2 allow_unknown: можно, но дороже). */
const UNKNOWN_COST = 2;

/**
 * Маска «сюда центр робота нельзя»: занятые клетки, раздутые на радиус
 * робота + запас. Неизвестные клетки проходимы (allow_unknown, как Nav2
 * по умолчанию) — карта открывается по ходу, путь уточняется пересчётом.
 */
export function inflateObstacles(g: SimGrid, radiusM = ROBOT_RADIUS_M + NAV_INFLATION_MARGIN_M): Uint8Array {
  const blocked = new Uint8Array(g.width * g.height);
  const r = Math.ceil(radiusM / g.resolution);
  const offsets: number[][] = [];
  for (let dy = -r; dy <= r; dy += 1) {
    for (let dx = -r; dx <= r; dx += 1) {
      if (dx * dx + dy * dy <= r * r) offsets.push([dx, dy]);
    }
  }
  for (let cy = 0; cy < g.height; cy += 1) {
    for (let cx = 0; cx < g.width; cx += 1) {
      if (g.data[cy * g.width + cx] !== 100) continue;
      for (const [dx, dy] of offsets) {
        const x = cx + dx;
        const y = cy + dy;
        if (x >= 0 && y >= 0 && x < g.width && y < g.height) blocked[y * g.width + x] = 1;
      }
    }
  }
  return blocked;
}

function toCell(g: SimGrid, p: Xy): [number, number] {
  return [Math.floor((p.x - g.originX) / g.resolution), Math.floor((p.y - g.originY) / g.resolution)];
}

function cellCenter(g: SimGrid, cx: number, cy: number): Xy {
  return { x: g.originX + (cx + 0.5) * g.resolution, y: g.originY + (cy + 0.5) * g.resolution };
}

/** Минимальная бинарная куча по приоритету (A*). */
class MinHeap {
  private readonly items: number[] = [];
  private readonly prio: number[] = [];
  get size(): number {
    return this.items.length;
  }
  push(item: number, p: number): void {
    this.items.push(item);
    this.prio.push(p);
    let i = this.items.length - 1;
    while (i > 0) {
      const parent = (i - 1) >> 1;
      if (this.prio[parent] <= this.prio[i]) break;
      this.swap(i, parent);
      i = parent;
    }
  }
  pop(): number {
    const top = this.items[0];
    const lastItem = this.items.pop()!;
    const lastPrio = this.prio.pop()!;
    if (this.items.length > 0) {
      this.items[0] = lastItem;
      this.prio[0] = lastPrio;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1;
        const r = l + 1;
        let m = i;
        if (l < this.items.length && this.prio[l] < this.prio[m]) m = l;
        if (r < this.items.length && this.prio[r] < this.prio[m]) m = r;
        if (m === i) break;
        this.swap(i, m);
        i = m;
      }
    }
    return top;
  }
  private swap(a: number, b: number): void {
    [this.items[a], this.items[b]] = [this.items[b], this.items[a]];
    [this.prio[a], this.prio[b]] = [this.prio[b], this.prio[a]];
  }
}

/** Отрезок a→b целиком по проходимым клеткам (шаг — полклетки). */
function lineFree(g: SimGrid, blocked: Uint8Array, a: Xy, b: Xy): boolean {
  const len = Math.hypot(b.x - a.x, b.y - a.y);
  const n = Math.max(1, Math.ceil(len / (g.resolution * 0.5)));
  for (let i = 0; i <= n; i += 1) {
    const t = i / n;
    const [cx, cy] = toCell(g, { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t });
    if (cx < 0 || cy < 0 || cx >= g.width || cy >= g.height) return false;
    if (blocked[cy * g.width + cx]) return false;
  }
  return true;
}

/**
 * A* по 8-связной решётке от `from` до `to` (кадр `map`). `null` — пути
 * нет: цель в препятствии/вне карты или отрезана. Клетку старта считаем
 * проходимой всегда: робот, прижатый к стене, должен уметь от неё уехать.
 */
export function planPath(g: SimGrid, blocked: Uint8Array, from: Xy, to: Xy): Xy[] | null {
  const [sx, sy] = toCell(g, from);
  const [tx, ty] = toCell(g, to);
  const W = g.width;
  const H = g.height;
  const inside = (x: number, y: number): boolean => x >= 0 && y >= 0 && x < W && y < H;
  if (!inside(sx, sy) || !inside(tx, ty)) return null;
  const start = sy * W + sx;
  const goal = ty * W + tx;
  if (blocked[goal]) return null;
  const gScore = new Float32Array(W * H).fill(Infinity);
  const came = new Int32Array(W * H).fill(-1);
  const closed = new Uint8Array(W * H);
  const h = (i: number): number => {
    const dx = Math.abs((i % W) - tx);
    const dy = Math.abs(Math.floor(i / W) - ty);
    return Math.max(dx, dy) + (Math.SQRT2 - 1) * Math.min(dx, dy);
  };
  const heap = new MinHeap();
  gScore[start] = 0;
  heap.push(start, h(start));
  const dirs: Array<[number, number, number]> = [
    [1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1],
    [1, 1, Math.SQRT2], [1, -1, Math.SQRT2], [-1, 1, Math.SQRT2], [-1, -1, Math.SQRT2]
  ];
  let found = false;
  while (heap.size > 0) {
    const cur = heap.pop();
    if (closed[cur]) continue;
    if (cur === goal) {
      found = true;
      break;
    }
    closed[cur] = 1;
    const cx = cur % W;
    const cy = Math.floor(cur / W);
    for (const [dx, dy, step] of dirs) {
      const nx = cx + dx;
      const ny = cy + dy;
      if (!inside(nx, ny)) continue;
      const ni = ny * W + nx;
      if (closed[ni] || blocked[ni]) continue;
      // Диагональ не срезает угол препятствия.
      if (dx !== 0 && dy !== 0 && (blocked[cy * W + nx] || blocked[ny * W + cx])) continue;
      const cost = step * (g.data[ni] === -1 ? UNKNOWN_COST : 1);
      const tentative = gScore[cur] + cost;
      if (tentative < gScore[ni]) {
        gScore[ni] = tentative;
        came[ni] = cur;
        heap.push(ni, tentative + h(ni));
      }
    }
  }
  if (!found) return null;
  const cells: Xy[] = [];
  for (let i = goal; i !== -1; i = came[i]) cells.push(cellCenter(g, i % W, Math.floor(i / W)));
  cells.reverse();
  // Концы — точные точки старта и цели, а не центры клеток.
  cells[0] = { x: from.x, y: from.y };
  cells[cells.length - 1] = { x: to.x, y: to.y };
  return smoothPath(g, blocked, cells);
}

/**
 * «Натянуть нить»: из ломаной по клеткам выкинуть вершины, которые видно
 * напрямую, затем раздробить отрезки до NAV_PATH_SPACING_M (путь на полу
 * рисуется ленточкой, редкие точки дали бы кривую без изгибов).
 */
export function smoothPath(g: SimGrid, blocked: Uint8Array, pts: Xy[]): Xy[] {
  if (pts.length <= 2) return densify(pts);
  const out: Xy[] = [pts[0]];
  let anchor = 0;
  while (anchor < pts.length - 1) {
    let next = anchor + 1;
    for (let j = pts.length - 1; j > anchor + 1; j -= 1) {
      if (lineFree(g, blocked, pts[anchor], pts[j])) {
        next = j;
        break;
      }
    }
    out.push(pts[next]);
    anchor = next;
  }
  return densify(out);
}

function densify(pts: Xy[]): Xy[] {
  if (pts.length < 2) return pts.slice();
  const out: Xy[] = [pts[0]];
  for (let i = 1; i < pts.length; i += 1) {
    const a = pts[i - 1];
    const b = pts[i];
    const n = Math.max(1, Math.ceil(Math.hypot(b.x - a.x, b.y - a.y) / NAV_PATH_SPACING_M));
    for (let k = 1; k <= n; k += 1) out.push({ x: a.x + ((b.x - a.x) * k) / n, y: a.y + ((b.y - a.y) * k) / n });
  }
  return out;
}

/** Равномерное прорежение до `max` точек, концы сохраняются (decimate_path). */
export function decimatePath(pts: readonly Xy[], max = NAV_PATH_MAX_POINTS): Xy[] {
  if (pts.length <= max) return pts.slice();
  const step = (pts.length - 1) / (max - 1);
  const out: Xy[] = [];
  for (let i = 0; i < max; i += 1) out.push(pts[Math.round(i * step)]);
  return out;
}

export function pathLength(pts: readonly Xy[]): number {
  let s = 0;
  for (let i = 1; i < pts.length; i += 1) s += Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
  return s;
}

// ────────────────────────── pure pursuit ──────────────────────────

export const PP_LOOKAHEAD_M = 0.6;
export const PP_MAX_V = 0.45;
export const PP_MAX_W = 1.5;
/** Цель достигнута: позиция и курс (goal_checker Nav2 — 0.25 м / 0.25 рад). */
export const GOAL_XY_TOLERANCE_M = 0.15;
export const GOAL_YAW_TOLERANCE_RAD = 0.2;

function wrap(a: number): number {
  return Math.atan2(Math.sin(a), Math.cos(a));
}

export interface PursuitResult {
  v: number;
  w: number;
  /** Индекс ближайшей точки пути (прогресс, не убывает). */
  index: number;
  /** Позиция и курс в допуске — цель достигнута. */
  arrived: boolean;
  /** Остаток пути от робота до цели, м. */
  remaining: number;
}

/**
 * Один шаг pure-pursuit по пути `path` от прогресса `fromIndex`. У цели —
 * доворот на месте на `goalYaw`. Крутой угол на точку упреждения —
 * разворот на месте (дифф-привод, как RotationShim Nav2).
 */
export function purePursuit(
  pose: { x: number; y: number; yaw: number },
  path: readonly Xy[],
  fromIndex: number,
  goalYaw: number
): PursuitResult {
  const goal = path[path.length - 1];
  const distGoal = Math.hypot(goal.x - pose.x, goal.y - pose.y);
  // Ближайшая точка впереди (окно — чтобы не прыгать через петли).
  let index = Math.min(fromIndex, path.length - 1);
  let best = Infinity;
  for (let i = index; i < Math.min(path.length, index + 40); i += 1) {
    const d = Math.hypot(path[i].x - pose.x, path[i].y - pose.y);
    if (d < best) {
      best = d;
      index = i;
    }
  }
  const remaining = best + pathLength(path.slice(index));
  if (distGoal <= GOAL_XY_TOLERANCE_M) {
    const err = wrap(goalYaw - pose.yaw);
    if (Math.abs(err) <= GOAL_YAW_TOLERANCE_RAD) return { v: 0, w: 0, index, arrived: true, remaining: distGoal };
    return { v: 0, w: clamp(2 * err, -1, 1), index, arrived: false, remaining: distGoal };
  }
  // Точка упреждения: первая точка дальше lookahead от робота.
  let target = goal;
  for (let i = index; i < path.length; i += 1) {
    if (Math.hypot(path[i].x - pose.x, path[i].y - pose.y) >= PP_LOOKAHEAD_M) {
      target = path[i];
      break;
    }
  }
  const alpha = wrap(Math.atan2(target.y - pose.y, target.x - pose.x) - pose.yaw);
  if (Math.abs(alpha) > Math.PI / 3) {
    return { v: 0, w: Math.sign(alpha) * 1.2, index, arrived: false, remaining };
  }
  let v = PP_MAX_V * clamp(1 - Math.abs(alpha) / 1.2, 0.2, 1);
  v = Math.min(v, 0.08 + distGoal * 0.8); // плавный подход
  const L = Math.max(0.2, Math.hypot(target.x - pose.x, target.y - pose.y));
  const w = clamp((2 * v * Math.sin(alpha)) / L, -PP_MAX_W, PP_MAX_W);
  return { v, w, index, arrived: false, remaining };
}

function clamp(v: number, lo: number, hi: number): number {
  return Math.min(hi, Math.max(lo, v));
}

// ────────────────────────── жизненный цикл цели ──────────────────────────

export interface SimNavGoal {
  seq: number;
  x: number;
  y: number;
  yaw: number;
}

export type SimNavState = "idle" | "accepted" | "active";

/** Пересчёт пути, мс (BT RateController hz=1.0). */
export const NAV_REPLAN_PERIOD_MS = 1000;
/** nav_status{active} не чаще, мс (NAV_FEEDBACK_MIN_PERIOD_S = 0.5). */
export const NAV_FEEDBACK_PERIOD_MS = 500;
/** Нет прогресса к цели столько — сдаёмся (progress_checker Nav2 ~10 с). */
export const NAV_STUCK_TIMEOUT_MS = 10_000;
/** Прогресс = приблизились хотя бы на столько. */
const NAV_PROGRESS_M = 0.1;

export interface SimNavigatorDeps {
  grid: SimGrid;
  pose: () => { x: number; y: number; yaw: number };
  /** Отдать команду скорости (как Nav2 → cmd_vel, приоритет 10). */
  drive: (v: number, w: number) => void;
  /** JSON_EVENT всем сессиям. */
  emit: (event: Record<string, unknown>) => void;
}

/**
 * Одна активная цель за раз (Nav2GoalBridge). Шаги — из такта мок-робота
 * (`step(now, overridden)`), события — через `emit`.
 */
export class SimNavigator {
  private goal: SimNavGoal | null = null;
  private state: SimNavState = "idle";
  private path: Xy[] = [];
  private index = 0;
  private lastPlanMs = -Infinity;
  private lastFeedbackMs = -Infinity;
  private bestRemaining = Infinity;
  private lastProgressMs = 0;
  private blocked: Uint8Array | null = null;
  private blockedRevision = -1;
  /** Растёт при каждой смене пути (в т.ч. на пустой) — стрим nav_path шлёт по ней. */
  pathRevision = 0;

  constructor(private readonly deps: SimNavigatorDeps) {}

  activeGoal(): SimNavGoal | null {
    return this.goal;
  }

  navState(): SimNavState {
    return this.state;
  }

  /** Текущий план (кадр `map`); пустой — пути нет. */
  currentPath(): readonly Xy[] {
    return this.path;
  }

  /** Принять цель (после проверок моста). Старая вытесняется молча. */
  start(goal: SimNavGoal, now: number): void {
    this.goal = { ...goal };
    this.state = "idle"; // accepted уйдёт на ближайшем шаге (асинхронно, как Nav2)
    this.index = 0;
    this.lastPlanMs = -Infinity;
    this.lastFeedbackMs = -Infinity;
    this.bestRemaining = Infinity;
    this.lastProgressMs = now;
  }

  /** Отменить. `false` — отменять нечего. */
  cancel(now: number): boolean {
    if (!this.goal) return false;
    this.finish("canceled", now);
    return true;
  }

  /** Такт. `overridden` — телеоп сейчас перебивает навигацию (twist_mux). */
  step(now: number, overridden: boolean): void {
    const goal = this.goal;
    if (!goal) return;
    if (this.state === "idle") {
      this.state = "accepted";
      this.emitStatus("accepted", now);
    }
    if (now - this.lastPlanMs >= NAV_REPLAN_PERIOD_MS) {
      this.lastPlanMs = now;
      const planned = planPath(this.deps.grid, this.blockedMask(), this.deps.pose(), goal);
      if (!planned) {
        this.finish("aborted", now, "sim_no_path");
        return;
      }
      this.path = decimatePath(planned);
      this.index = 0;
      this.pathRevision += 1;
    }
    if (this.state === "accepted") this.state = "active";
    if (overridden) {
      // Оператор рулит сам — прогресс не штрафуем, команду не шлём.
      this.lastProgressMs = now;
      return;
    }
    const r = purePursuit(this.deps.pose(), this.path, this.index, goal.yaw);
    this.index = r.index;
    if (r.arrived) {
      this.deps.drive(0, 0);
      this.finish("succeeded", now);
      return;
    }
    this.deps.drive(r.v, r.w);
    if (r.remaining < this.bestRemaining - NAV_PROGRESS_M) {
      this.bestRemaining = r.remaining;
      this.lastProgressMs = now;
    } else if (now - this.lastProgressMs >= NAV_STUCK_TIMEOUT_MS) {
      this.deps.drive(0, 0);
      this.finish("aborted", now, "sim_stuck");
      return;
    }
    if (now - this.lastFeedbackMs >= NAV_FEEDBACK_PERIOD_MS) {
      this.lastFeedbackMs = now;
      this.emitStatus("active", now, r.remaining);
    }
  }

  private blockedMask(): Uint8Array {
    if (!this.blocked || this.blockedRevision !== this.deps.grid.revision) {
      this.blocked = inflateObstacles(this.deps.grid);
      this.blockedRevision = this.deps.grid.revision;
    }
    return this.blocked;
  }

  private finish(state: "succeeded" | "canceled" | "aborted", now: number, reason?: string): void {
    const goal = this.goal;
    if (!goal) return;
    this.emitStatus(state, now, undefined, reason);
    // Nav2 при завершении/отмене публикует нулевой cmd_vel.
    this.deps.drive(0, 0);
    this.goal = null;
    this.state = "idle";
    this.path = [];
    this.pathRevision += 1;
  }

  private emitStatus(state: string, now: number, distance?: number, reason?: string): void {
    const g = this.goal!;
    const ev: Record<string, unknown> = {
      type: "nav_status",
      state,
      seq: g.seq,
      x: g.x,
      y: g.y,
      yaw: g.yaw,
      ts_ms: Math.round(now)
    };
    if (distance !== undefined && Number.isFinite(distance)) ev.distance_remaining = distance;
    if (reason) ev.reason = reason;
    this.deps.emit(ev);
  }
}
