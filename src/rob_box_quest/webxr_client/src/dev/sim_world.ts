// Мир симулятора мостика (issue #3149): 2D-план помещения, кинематика
// дифф-привода, лидар-рейкаст и «SLAM»-карта, которая открывается по мере
// того, как робот её просматривает.
//
// Кадр — `map` по REP-103: x — восток, y — север, yaw против часовой от +x.
// Ровно в этих координатах сервер отдаёт позу робота в map_2d, и ровно в
// них LaserScan меряет углы (0 = вперёд робота), поэтому энкодеры
// (sim_encoders.ts) ничего не пересчитывают — только сериализуют.
//
// Никакого DOM и Three.js: модуль чистый, гоняется в vitest как есть.

/** Отрезок стены/препятствия. `height` — для псевдо-3D камеры. */
export interface SimSegment {
  ax: number;
  ay: number;
  bx: number;
  by: number;
  /** Высота над полом, м. */
  height: number;
  /** Базовый цвет для камеры, 0xRRGGBB. */
  color: number;
  /** Что это — для подписи в HUD камеры и для цвета depth-вида. */
  kind: "wall" | "obstacle";
}

export interface SimWorld {
  /** Внешний контур помещения (замкнутый полигон, CCW). */
  outline: ReadonlyArray<readonly [number, number]>;
  segments: ReadonlyArray<SimSegment>;
  /** Границы решётки карты (с запасом вокруг контура). */
  bounds: { minX: number; minY: number; maxX: number; maxY: number };
  /** Стартовая поза робота. */
  start: { x: number; y: number; yaw: number };
  /** Высота потолка, м (для потолочной камеры). */
  ceilingHeight: number;
}

const WALL_COLOR = 0x5d6b7a;
const WALL_H = 2.6;

function box(
  x0: number,
  y0: number,
  x1: number,
  y1: number,
  height: number,
  color: number
): SimSegment[] {
  const s = (ax: number, ay: number, bx: number, by: number): SimSegment => ({
    ax,
    ay,
    bx,
    by,
    height,
    color,
    kind: "obstacle"
  });
  return [s(x0, y0, x1, y0), s(x1, y0, x1, y1), s(x1, y1, x0, y1), s(x0, y1, x0, y0)];
}

/**
 * Мастерская: комната A 8×6 м → коридор 6×2 м на восток → комната B 5×8 м.
 * В комнатах стол, колонна, стеллаж, ящики — чтобы лидару и камере было
 * за что зацепиться, а карта не была пустым прямоугольником.
 */
export function createDefaultWorld(): SimWorld {
  const outline: Array<[number, number]> = [
    [0, 0],
    [8, 0],
    [8, 2],
    [14, 2],
    [14, -1],
    [19, -1],
    [19, 7],
    [14, 7],
    [14, 4],
    [8, 4],
    [8, 6],
    [0, 6]
  ];
  const segments: SimSegment[] = [];
  for (let i = 0; i < outline.length; i += 1) {
    const [ax, ay] = outline[i];
    const [bx, by] = outline[(i + 1) % outline.length];
    segments.push({ ax, ay, bx, by, height: WALL_H, color: WALL_COLOR, kind: "wall" });
  }
  segments.push(
    ...box(2.0, 3.6, 3.6, 4.6, 0.75, 0xb07a4a), // стол
    ...box(5.5, 1.5, 5.9, 1.9, WALL_H, 0x8a8f96), // колонна
    ...box(0.0, 5.3, 1.8, 6.0, 1.8, 0x3f7fbf), // шкаф у стены
    ...box(10.5, 3.6, 11.3, 4.0, 1.0, 0xd0a040), // тумба в коридоре
    ...box(16.0, 5.0, 18.5, 5.5, 1.8, 0x4a9a6a), // стеллаж
    ...box(16.4, 0.4, 17.2, 1.2, 0.6, 0xc05a40), // ящик
    ...box(15.0, 2.4, 15.3, 2.7, WALL_H, 0x8a8f96) // стойка
  );
  return {
    outline,
    segments,
    bounds: { minX: -1, minY: -2, maxX: 20, maxY: 8 },
    start: { x: 1.4, y: 1.4, yaw: Math.PI / 6 },
    ceilingHeight: WALL_H
  };
}

// ────────────────────────── геометрия ──────────────────────────

/** Пересечение луча (ox,oy)+t·(dx,dy) с отрезком: t ≥ 0 или null. */
export function raySegment(
  ox: number,
  oy: number,
  dx: number,
  dy: number,
  s: SimSegment
): { t: number; u: number } | null {
  const ex = s.bx - s.ax;
  const ey = s.by - s.ay;
  const den = dx * ey - dy * ex;
  if (Math.abs(den) < 1e-12) return null;
  const wx = s.ax - ox;
  const wy = s.ay - oy;
  const t = (wx * ey - wy * ex) / den;
  const u = (wx * dy - wy * dx) / den;
  if (t < 0 || u < 0 || u > 1) return null;
  return { t, u };
}

/** Ближайшее попадание луча по миру; `Infinity`, если мимо всего. */
export function castRay(
  world: SimWorld,
  ox: number,
  oy: number,
  angle: number,
  maxRange = Infinity
): number {
  const dx = Math.cos(angle);
  const dy = Math.sin(angle);
  let best = maxRange;
  for (const s of world.segments) {
    const hit = raySegment(ox, oy, dx, dy, s);
    if (hit && hit.t < best) best = hit.t;
  }
  return best;
}

/** Расстояние от точки до отрезка. */
export function pointSegmentDistance(px: number, py: number, s: SimSegment): number {
  const ex = s.bx - s.ax;
  const ey = s.by - s.ay;
  const len2 = ex * ex + ey * ey;
  let t = len2 > 0 ? ((px - s.ax) * ex + (py - s.ay) * ey) / len2 : 0;
  t = Math.max(0, Math.min(1, t));
  const cx = s.ax + t * ex;
  const cy = s.ay + t * ey;
  return Math.hypot(px - cx, py - cy);
}

/** Точка внутри контура помещения (even-odd). */
export function insideOutline(world: SimWorld, x: number, y: number): boolean {
  const pts = world.outline;
  let inside = false;
  for (let i = 0, j = pts.length - 1; i < pts.length; j = i, i += 1) {
    const [xi, yi] = pts[i];
    const [xj, yj] = pts[j];
    if (yi > y !== yj > y && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
}

/** Робот радиуса `radius` в точке (x,y) не задевает ни одной стены. */
export function isFree(world: SimWorld, x: number, y: number, radius: number): boolean {
  if (!insideOutline(world, x, y)) return false;
  for (const s of world.segments) {
    if (pointSegmentDistance(x, y, s) < radius) return false;
  }
  return true;
}

// ────────────────────────── кинематика ──────────────────────────

export const ROBOT_RADIUS_M = 0.28;
/** Команда без обновления дольше этого — робот тормозит (как cmd_vel timeout). */
export const CMD_TIMEOUT_MS = 300;
const MAX_LIN_ACCEL = 1.2; // м/с²
const MAX_ANG_ACCEL = 4.0; // рад/с²

export interface SimRobotState {
  x: number;
  y: number;
  yaw: number;
  /** Фактические скорости. */
  v: number;
  w: number;
  /** Последняя принятая команда. */
  cmdV: number;
  cmdW: number;
  lastCmdMs: number;
  /** Упёрся в стену на последнем шаге (для HUD камеры). */
  bumped: boolean;
}

export function createRobotState(world: SimWorld): SimRobotState {
  return {
    x: world.start.x,
    y: world.start.y,
    yaw: world.start.yaw,
    v: 0,
    w: 0,
    cmdV: 0,
    cmdW: 0,
    lastCmdMs: -Infinity,
    bumped: false
  };
}

function approach(cur: number, target: number, maxDelta: number): number {
  if (target > cur) return Math.min(target, cur + maxDelta);
  return Math.max(target, cur - maxDelta);
}

function wrapAngle(a: number): number {
  let r = a;
  while (r > Math.PI) r -= 2 * Math.PI;
  while (r < -Math.PI) r += 2 * Math.PI;
  return r;
}

/**
 * Шаг дифф-привода на `dtS` секунд. Команда протухает через
 * CMD_TIMEOUT_MS; ускорения ограничены; поступательное движение в стену
 * отбрасывается (поворот на месте остаётся — как у настоящего робота,
 * упёршегося бампером).
 */
export function stepRobot(world: SimWorld, st: SimRobotState, dtS: number, nowMs: number): void {
  const fresh = nowMs - st.lastCmdMs <= CMD_TIMEOUT_MS;
  const targetV = fresh ? st.cmdV : 0;
  const targetW = fresh ? st.cmdW : 0;
  st.v = approach(st.v, targetV, MAX_LIN_ACCEL * dtS);
  st.w = approach(st.w, targetW, MAX_ANG_ACCEL * dtS);
  st.yaw = wrapAngle(st.yaw + st.w * dtS);
  const nx = st.x + st.v * Math.cos(st.yaw) * dtS;
  const ny = st.y + st.v * Math.sin(st.yaw) * dtS;
  if (st.v !== 0 && !isFree(world, nx, ny, ROBOT_RADIUS_M)) {
    st.v = 0;
    st.bumped = true;
    return;
  }
  st.bumped = false;
  st.x = nx;
  st.y = ny;
}

// ────────────────────────── лидар ──────────────────────────

export interface SimScan {
  angleMin: number;
  angleMax: number;
  angleIncrement: number;
  rangeMin: number;
  rangeMax: number;
  scanTime: number;
  ranges: Float32Array;
  intensities: Float32Array;
}

export const LIDAR_POINTS = 360;
export const LIDAR_RANGE_MIN = 0.15;
export const LIDAR_RANGE_MAX = 12;

/**
 * Скан 360° из позы робота. Промах — `Infinity` (как у RPLidar: клиент
 * отбрасывает точки вне [range_min, range_max]). Немного шума, чтобы
 * облако не выглядело нарисованным по линейке.
 */
export function scanLidar(
  world: SimWorld,
  st: Pick<SimRobotState, "x" | "y" | "yaw">,
  rand: () => number = Math.random
): SimScan {
  const n = LIDAR_POINTS;
  const angleMin = -Math.PI;
  const inc = (2 * Math.PI) / n;
  const ranges = new Float32Array(n);
  const intensities = new Float32Array(n);
  for (let i = 0; i < n; i += 1) {
    const a = angleMin + i * inc;
    const r = castRay(world, st.x, st.y, st.yaw + a, LIDAR_RANGE_MAX);
    if (r >= LIDAR_RANGE_MAX) {
      ranges[i] = Infinity;
      intensities[i] = 0;
    } else {
      ranges[i] = r + (rand() - 0.5) * 0.02;
      intensities[i] = 47;
    }
  }
  return {
    angleMin,
    angleMax: angleMin + (n - 1) * inc,
    angleIncrement: inc,
    rangeMin: LIDAR_RANGE_MIN,
    rangeMax: LIDAR_RANGE_MAX,
    scanTime: 0.1,
    ranges,
    intensities
  };
}

// ────────────────────────── карта ──────────────────────────

/**
 * Решётка в формате nav_msgs/OccupancyGrid: -1 unknown, 0 free, 100 occupied;
 * `data[0]` — клетка у origin (низ-лево), строки идут на север.
 */
export interface SimGrid {
  resolution: number;
  width: number;
  height: number;
  originX: number;
  originY: number;
  data: Int8Array;
  /**
   * Счётчик попаданий лидара по клетке (гистерезис занятости): клетка
   * становится занятой только после OCC_HITS_TO_MARK попаданий, а луч,
   * прошедший сквозь неё, счётчик уменьшает. Иначе шум дальномера (±1 см)
   * то и дело «зажигал» соседние со стеной клетки — ревизия карты росла
   * каждый скан, и мок слал 336-килобайтный PNG раз в секунду.
   */
  hits: Uint8Array;
  /** Растёт при каждом изменении решётки (планировщик пересчитывает маску). */
  revision: number;
  /** Сколько клеток изменилось за всё время — «значимость» изменения карты. */
  changedCells: number;
}

/** Попаданий подряд (без пролётов насквозь), чтобы клетка стала занятой. */
export const OCC_HITS_TO_MARK = 3;

export function createGrid(world: SimWorld, resolution = 0.05): SimGrid {
  const { minX, minY, maxX, maxY } = world.bounds;
  const width = Math.ceil((maxX - minX) / resolution);
  const height = Math.ceil((maxY - minY) / resolution);
  return {
    resolution,
    width,
    height,
    originX: minX,
    originY: minY,
    data: new Int8Array(width * height).fill(-1),
    hits: new Uint8Array(width * height),
    revision: 0,
    changedCells: 0
  };
}

function cellIndex(g: SimGrid, x: number, y: number): number {
  const cx = Math.floor((x - g.originX) / g.resolution);
  const cy = Math.floor((y - g.originY) / g.resolution);
  if (cx < 0 || cy < 0 || cx >= g.width || cy >= g.height) return -1;
  return cy * g.width + cx;
}

/**
 * «SLAM»: протащить каждый луч скана по решётке — клетки вдоль луча
 * свободны, клетка попадания копит попадания и занимается после
 * OCC_HITS_TO_MARK (см. SimGrid.hits). Так карта открывается ровно там,
 * куда робот реально посмотрел. Возвращает число изменившихся клеток.
 */
export function integrateScan(
  g: SimGrid,
  st: Pick<SimRobotState, "x" | "y" | "yaw">,
  scan: SimScan
): number {
  let changed = 0;
  const step = g.resolution * 0.8;
  for (let i = 0; i < scan.ranges.length; i += 1) {
    const r = scan.ranges[i];
    const hit = Number.isFinite(r);
    const len = hit ? r : scan.rangeMax;
    const a = st.yaw + scan.angleMin + i * scan.angleIncrement;
    const dx = Math.cos(a);
    const dy = Math.sin(a);
    for (let t = 0; t < len - g.resolution; t += step) {
      const idx = cellIndex(g, st.x + dx * t, st.y + dy * t);
      if (idx < 0) break;
      if (g.data[idx] === -1) {
        g.data[idx] = 0;
        changed += 1;
      }
      // Луч прошёл насквозь — улика против занятости (кроме уже занятых:
      // занятое не «гаснет», карта сходится, а не мерцает).
      if (g.hits[idx] > 0 && g.data[idx] !== 100) g.hits[idx] -= 1;
    }
    if (hit) {
      const idx = cellIndex(g, st.x + dx * r, st.y + dy * r);
      if (idx >= 0 && g.data[idx] !== 100) {
        g.hits[idx] = Math.min(255, g.hits[idx] + 1);
        if (g.hits[idx] >= OCC_HITS_TO_MARK) {
          g.data[idx] = 100;
          changed += 1;
        }
      }
    }
  }
  if (changed > 0) {
    g.revision += 1;
    g.changedCells += changed;
  }
  return changed;
}
