// Nav2 симулятора (sim_nav.ts) + команды/стримы мок-робота вокруг него:
// max_hz на SUBSCRIBE, nav_goal/nav_cancel, nav_path, гистерезис карты.
//
// Мок гоняется синхронно: MockSession напрямую (без Connection и сокета),
// часы робота — ручные, такт — robot.tick(). Так 60 с симуляции проходят за
// доли секунды и без флаки-таймеров.

import { describe, it, expect } from "vitest";
import {
  DEFAULT_STREAMS,
  MAP_PNG_MIN_PERIOD_MS,
  MockRobot,
  MockSession,
  mapPngDue,
  parseMaxHz,
  subscriptionPeriodMs,
  type MockSubscription
} from "../src/dev/mock_robot";
import { decodeFrame, encodeJsonFrame, FrameType } from "../src/wire/protocol";
import { parseNavPath } from "../src/nav/nav_path_payload";
import { parseMapFrame } from "../src/scene/map_payload";
import {
  inflateObstacles,
  pathLength,
  planPath,
  purePursuit,
  GOAL_XY_TOLERANCE_M
} from "../src/dev/sim_nav";
import {
  createDefaultWorld,
  createGrid,
  integrateScan,
  isFree,
  OCC_HITS_TO_MARK,
  ROBOT_RADIUS_M,
  scanLidar
} from "../src/dev/sim_world";

// ────────────────────────── харнесс ──────────────────────────

interface Rx {
  events: Array<Record<string, unknown>>;
  frames: Array<{ streamId: number; payload: Uint8Array }>;
}

function harness(opts: { exploreFirst?: boolean } = {}) {
  let t = 1_000_000;
  const robot = new MockRobot({ now: () => t });
  const rx: Rx = { events: [], frames: [] };
  const session = new MockSession(
    robot,
    (bytes) => {
      const f = decodeFrame(bytes);
      if (f.type === FrameType.JSON_EVENT) rx.events.push(JSON.parse(new TextDecoder().decode(f.payload)));
      else if (f.type === FrameType.BINARY_FRAME) rx.frames.push({ streamId: f.streamId, payload: f.payload });
    },
    () => undefined
  );
  robot.attach(session);
  const send = (type: FrameType, obj: unknown): void => session.handleFrame(encodeJsonFrame(type, 0, obj));
  send(FrameType.HELLO, { session_pin: "000000", client_version: "test" });
  const cmd = (obj: Record<string, unknown>): void => send(FrameType.JSON_CMD, { ts_ms: t, ...obj });
  /** Прогнать `ms` миллисекунд симуляции тактами по 20 мс. */
  const run = (ms: number, until?: () => boolean): void => {
    for (let k = 0; k < ms / 20; k += 1) {
      t += 20;
      robot.tick();
      if (until?.()) return;
    }
  };
  if (opts.exploreFirst) {
    // Прогреть карту на старте: 1 с сканов (гистерезис занятости = 3 скана).
    run(1000);
  }
  const navStatuses = (): Array<Record<string, unknown>> => rx.events.filter((e) => e.type === "nav_status");
  return { robot, session, rx, send, cmd, run, navStatuses, now: () => t };
}

// ────────────────────────── max_hz ──────────────────────────

describe("mock SUBSCRIBE.max_hz (#3150)", () => {
  it("parseMaxHz mirrors stream_rate.parse_max_hz", () => {
    expect(parseMaxHz(5)).toBe(5);
    expect(parseMaxHz(0.5)).toBe(0.5);
    expect(parseMaxHz(undefined)).toBeNull();
    expect(parseMaxHz(true)).toBeNull();
    expect(parseMaxHz("5")).toBeNull();
    expect(parseMaxHz(0)).toBeNull();
    expect(parseMaxHz(-1)).toBeNull();
    expect(parseMaxHz(121)).toBeNull();
    expect(parseMaxHz(Number.NaN)).toBeNull();
  });

  it("subscriptionPeriodMs thins to max_hz but never speeds a stream up", () => {
    const lidar = DEFAULT_STREAMS.lidar_2d; // 10 Гц
    const sub = (maxHz: number | null): MockSubscription => ({
      topic: "lidar_2d",
      streamId: 1,
      quality: "high",
      request: {},
      maxHz,
      nextDueMs: 0,
      busy: false,
      memo: {}
    });
    expect(subscriptionPeriodMs(lidar, sub(null))).toBe(100);
    expect(subscriptionPeriodMs(lidar, sub(2))).toBe(500);
    expect(subscriptionPeriodMs(lidar, sub(50))).toBe(100);
    expect(subscriptionPeriodMs(DEFAULT_STREAMS.nav_path, sub(0.5))).toBe(2000);
  });

  it("ack echoes max_hz; frames are thinned; re-SUBSCRIBE without it lifts the limit", () => {
    const h = harness();
    h.send(FrameType.SUBSCRIBE, { topic: "lidar_2d", max_hz: 2 });
    const ack = h.rx.events.find((e) => e.type === "subscribe_ack")!;
    expect(ack.max_hz).toBe(2);
    const sid = ack.stream_id as number;
    h.run(3000);
    const limited = h.rx.frames.filter((f) => f.streamId === sid).length;
    expect(limited).toBeGreaterThanOrEqual(5);
    expect(limited).toBeLessThanOrEqual(7); // 2 Гц × 3 с (+ первый кадр)
    h.send(FrameType.SUBSCRIBE, { topic: "lidar_2d" });
    const ack2 = h.rx.events.filter((e) => e.type === "subscribe_ack")[1];
    expect(ack2.stream_id).toBe(sid);
    expect("max_hz" in ack2).toBe(false);
    const before = h.rx.frames.length;
    h.run(3000);
    const free = h.rx.frames.slice(before).filter((f) => f.streamId === sid).length;
    expect(free).toBeGreaterThanOrEqual(28); // 10 Гц × 3 с
  });
});

// ────────────────────────── карта: гистерезис ──────────────────────────

describe("sim map churn (hysteresis + PNG throttle)", () => {
  it("occupancy needs several hits; converges while idle", () => {
    const world = createDefaultWorld();
    const grid = createGrid(world);
    const pose = world.start;
    const occupied = (): number => grid.data.reduce((n, v) => n + (v === 100 ? 1 : 0), 0);
    // Первый скан: свободные клетки открылись; занятыми стали лишь клетки,
    // куда в одном скане попало ≥ OCC_HITS_TO_MARK лучей (дальние стены).
    integrateScan(grid, pose, scanLidar(world, pose));
    const afterOne = occupied();
    for (let i = 1; i < OCC_HITS_TO_MARK; i += 1) integrateScan(grid, pose, scanLidar(world, pose));
    expect(occupied()).toBeGreaterThan(afterOne * 1.5);
    // Стоим на месте 60 с (10 Гц сканов): изменения затухают.
    const perSecond: number[] = [];
    for (let s = 0; s < 60; s += 1) {
      let changed = 0;
      for (let k = 0; k < 10; k += 1) changed += integrateScan(grid, pose, scanLidar(world, pose));
      perSecond.push(changed);
    }
    const lastTen = perSecond.slice(-10).reduce((a, b) => a + b, 0);
    // eslint-disable-next-line no-console
    console.log("[map churn] changed cells per second, idle:", perSecond.join(","));
    expect(lastTen).toBeLessThan(25); // меньше порога «значимого» изменения за 10 с
  });

  it("mapPngDue: first frame, 5 s for big change, 30 s for small, 60 s keepalive", () => {
    expect(mapPngDue(0, null, 0)).toBe(true);
    const last = { ms: 0, changedCells: 100 };
    expect(mapPngDue(MAP_PNG_MIN_PERIOD_MS - 1, last, 200)).toBe(false);
    expect(mapPngDue(MAP_PNG_MIN_PERIOD_MS, last, 200)).toBe(true);
    expect(mapPngDue(10_000, last, 105)).toBe(false);
    expect(mapPngDue(30_000, last, 105)).toBe(true);
    expect(mapPngDue(59_000, last, 100)).toBe(false);
    expect(mapPngDue(60_000, last, 100)).toBe(true);
  });

  it("idle robot: map_2d PNG goes out rarely, pose frames keep flowing", () => {
    const h = harness();
    h.send(FrameType.SUBSCRIBE, { topic: "map_2d" });
    const sid = h.rx.events.find((e) => e.type === "subscribe_ack")!.stream_id as number;
    h.run(60_000);
    const frames = h.rx.frames.filter((f) => f.streamId === sid).map((f) => parseMapFrame(f.payload)!);
    const pngs = frames.filter((f) => f.png !== null).length;
    // eslint-disable-next-line no-console
    console.log(`[map churn] 60 s idle: ${frames.length} map_2d frames, ${pngs} with PNG`);
    expect(frames.length).toBeGreaterThan(250); // поза 5 Гц
    expect(pngs).toBeGreaterThanOrEqual(1); // первый кадр подписки
    expect(pngs).toBeLessThanOrEqual(4); // было ~60 (раз в секунду)
  });
});

// ────────────────────────── планировщик / pure pursuit ──────────────────────────

/** Решётка, открытая «идеальным» проходом по всему миру (для тестов планировщика). */
function exploredGrid() {
  const world = createDefaultWorld();
  const grid = createGrid(world);
  const spots = [
    { x: 1.4, y: 1.4 },
    { x: 4, y: 3 },
    { x: 7, y: 5 },
    { x: 7, y: 1 },
    { x: 10, y: 3 },
    { x: 13, y: 3 },
    { x: 16.5, y: 3 },
    { x: 17.5, y: 6.3 },
    { x: 15, y: 0 }
  ];
  for (const p of spots) {
    for (let k = 0; k < OCC_HITS_TO_MARK + 1; k += 1) integrateScan(grid, { ...p, yaw: 0 }, scanLidar(world, { ...p, yaw: 0 }));
  }
  return { world, grid };
}

describe("sim planner", () => {
  it("plans around the room into the corridor; every point is collision-free", () => {
    const { world, grid } = exploredGrid();
    const blocked = inflateObstacles(grid);
    const path = planPath(grid, blocked, { x: 1.4, y: 1.4 }, { x: 16.5, y: 3 })!;
    expect(path).not.toBeNull();
    expect(path[0]).toEqual({ x: 1.4, y: 1.4 });
    expect(path[path.length - 1]).toEqual({ x: 16.5, y: 3 });
    // Через дверь коридора (x 8..14, y 2..4), без срезания стен.
    for (const p of path) expect(isFree(world, p.x, p.y, ROBOT_RADIUS_M)).toBe(true);
    const straight = Math.hypot(16.5 - 1.4, 3 - 1.4);
    expect(pathLength(path)).toBeGreaterThanOrEqual(straight - 1e-6);
    expect(pathLength(path)).toBeLessThan(straight * 1.4); // сглажен, а не лесенка по клеткам
  });

  it("goal inside an obstacle or outside the map → no path", () => {
    const { grid } = exploredGrid();
    const blocked = inflateObstacles(grid);
    expect(planPath(grid, blocked, { x: 1.4, y: 1.4 }, { x: 5.7, y: 1.7 })).toBeNull(); // колонна
    expect(planPath(grid, blocked, { x: 1.4, y: 1.4 }, { x: 50, y: 50 })).toBeNull();
  });

  it("pure pursuit: turns in place toward a target behind, arrives and aligns yaw", () => {
    const path = [
      { x: 0, y: 0 },
      { x: 2, y: 0 }
    ];
    const behind = purePursuit({ x: 0, y: 0, yaw: Math.PI }, path, 0, 0);
    expect(behind.v).toBe(0);
    expect(Math.abs(behind.w)).toBeGreaterThan(0.5);
    const ahead = purePursuit({ x: 0, y: 0, yaw: 0 }, path, 0, 0);
    expect(ahead.v).toBeGreaterThan(0.2);
    expect(Math.abs(ahead.w)).toBeLessThan(0.05);
    const atGoalWrongYaw = purePursuit({ x: 2 - GOAL_XY_TOLERANCE_M / 2, y: 0, yaw: 1 }, path, 1, 0);
    expect(atGoalWrongYaw.arrived).toBe(false);
    expect(atGoalWrongYaw.v).toBe(0);
    expect(atGoalWrongYaw.w).toBeLessThan(0);
    const done = purePursuit({ x: 2, y: 0, yaw: 0.05 }, path, 1, 0);
    expect(done.arrived).toBe(true);
  });
});

// ────────────────────────── nav_goal / nav_cancel в моке ──────────────────────────

describe("mock nav_goal / nav_cancel / nav_path (#3151)", () => {
  const goalCmd = (seq: number, x: number, y: number, yaw = 0, extra: Record<string, unknown> = {}) => ({
    cmd: "nav_goal",
    frame: "map",
    seq,
    x,
    y,
    yaw,
    ...extra
  });

  it("drives to a reachable goal: ack → accepted → active(distance↓) → succeeded; nav_path then empties", () => {
    const h = harness({ exploreFirst: true });
    h.send(FrameType.SUBSCRIBE, { topic: "nav_path" });
    const pathSid = h.rx.events.find((e) => e.type === "subscribe_ack")!.stream_id as number;
    h.cmd(goalCmd(1, 4.0, 2.2, Math.PI / 2));
    expect(h.rx.events.find((e) => e.type === "nav_goal_ack")).toMatchObject({ seq: 1 });
    h.run(60_000, () => h.navStatuses().some((s) => s.state !== "accepted" && s.state !== "active"));
    const states = h.navStatuses().map((s) => s.state);
    expect(states[0]).toBe("accepted");
    expect(states).toContain("active");
    expect(states[states.length - 1]).toBe("succeeded");
    const actives = h.navStatuses().filter((s) => s.state === "active");
    const d = actives.map((s) => s.distance_remaining as number);
    expect(d.every((v) => typeof v === "number" && v >= 0)).toBe(true);
    expect(d[d.length - 1]).toBeLessThan(d[0]);
    const last = h.navStatuses().at(-1)!;
    expect(last).toMatchObject({ seq: 1, x: 4.0, y: 2.2 });
    expect(Math.hypot(h.robot.state.x - 4.0, h.robot.state.y - 2.2)).toBeLessThanOrEqual(GOAL_XY_TOLERANCE_M + 1e-9);
    // feedback ≤ 2 Гц
    const ts = actives.map((s) => s.ts_ms as number);
    for (let i = 1; i < ts.length; i += 1) expect(ts[i] - ts[i - 1]).toBeGreaterThanOrEqual(500);
    // nav_path: реальный формат сервера, от робота к цели; по завершении n = 0.
    h.run(1000);
    const paths = h.rx.frames.filter((f) => f.streamId === pathSid).map((f) => parseNavPath(f.payload)!);
    expect(paths.length).toBeGreaterThanOrEqual(2);
    expect(paths[0].points.length).toBeGreaterThan(2);
    expect(paths[0].points.length).toBeLessThanOrEqual(200);
    const end = paths[0].points.at(-1)!;
    expect(end.x).toBeCloseTo(4.0, 4);
    expect(end.y).toBeCloseTo(2.2, 4);
    expect(paths.at(-1)!.points).toEqual([]);
  });

  it("unreachable goal → accepted then aborted (reason), empty path", () => {
    const h = harness({ exploreFirst: true });
    h.cmd(goalCmd(7, 5.7, 1.7)); // внутри колонны
    h.run(2000);
    const states = h.navStatuses().map((s) => s.state);
    expect(states).toEqual(["accepted", "aborted"]);
    expect(h.navStatuses()[1].reason).toBe("sim_no_path");
    expect(h.robot.nav.currentPath()).toEqual([]);
  });

  it("nav_cancel → nav_cancel_ack{had_goal} + canceled; robot stops", () => {
    const h = harness({ exploreFirst: true });
    h.cmd(goalCmd(2, 6.5, 4.5));
    h.run(3000);
    expect(Math.abs(h.robot.state.v) + Math.abs(h.robot.state.w)).toBeGreaterThan(0.05);
    h.cmd({ cmd: "nav_cancel" });
    expect(h.rx.events.find((e) => e.type === "nav_cancel_ack")).toMatchObject({ had_goal: true });
    expect(h.navStatuses().at(-1)).toMatchObject({ state: "canceled", seq: 2 });
    h.run(1500);
    expect(Math.abs(h.robot.state.v)).toBeLessThan(0.01);
    h.cmd({ cmd: "nav_cancel" });
    expect(h.rx.events.filter((e) => e.type === "nav_cancel_ack")[1]).toMatchObject({ had_goal: false });
  });

  it("stop_emergency cancels the goal; nav_goal during E-STOP → nack emergency_active", () => {
    const h = harness({ exploreFirst: true });
    h.cmd(goalCmd(3, 6.5, 4.5));
    h.run(2000);
    h.cmd({ cmd: "stop_emergency", source: "ui_button" });
    expect(h.navStatuses().at(-1)).toMatchObject({ state: "canceled", seq: 3 });
    h.cmd(goalCmd(4, 3, 3));
    expect(h.rx.events.find((e) => e.type === "nav_goal_nack")).toMatchObject({ seq: 4, reason: "emergency_active" });
  });

  it("bad frame / bad payload → nack like parse_nav_goal", () => {
    const h = harness();
    h.cmd(goalCmd(5, 1, 1, 0, { frame: "odom" }));
    h.cmd({ cmd: "nav_goal", frame: "map", seq: 6, x: "1", y: 1, yaw: 0 });
    h.cmd({ cmd: "nav_goal", frame: "map", x: 1, y: 1, yaw: 0 });
    const nacks = h.rx.events.filter((e) => e.type === "nav_goal_nack");
    expect(nacks[0]).toMatchObject({ seq: 5, reason: "bad_frame" });
    expect(nacks[1]).toMatchObject({ seq: 6, reason: "bad_payload" });
    expect(nacks[2]).toMatchObject({ reason: "bad_payload" });
    expect("seq" in nacks[2]).toBe(false);
  });

  it("teleop overrides navigation (twist_mux 90 > 10) without cancelling the goal", () => {
    const h = harness({ exploreFirst: true });
    h.cmd(goalCmd(8, 6.5, 4.5));
    h.run(2000);
    // Оператор держит «назад на месте» — навигатор молчит, цель жива.
    for (let i = 0; i < 20; i += 1) {
      h.cmd({ cmd: "teleop_twist", linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0.8 }, deadman: true });
      h.run(50);
    }
    expect(h.robot.state.cmdW).toBeCloseTo(0.8);
    expect(h.robot.state.cmdV).toBe(0);
    expect(h.navStatuses().some((s) => ["canceled", "aborted"].includes(s.state as string))).toBe(false);
    // Телеоп отпущен — через 0.5 с навигация продолжает и доезжает.
    h.run(60_000, () => h.navStatuses().some((s) => s.state === "succeeded"));
    expect(h.navStatuses().at(-1)).toMatchObject({ state: "succeeded", seq: 8 });
  });

  it("a new goal silently preempts the old one (no status for the old seq)", () => {
    const h = harness({ exploreFirst: true });
    h.cmd(goalCmd(10, 6.5, 4.5));
    h.run(1500);
    h.cmd(goalCmd(11, 3.0, 2.2));
    h.run(60_000, () => h.navStatuses().some((s) => s.seq === 11 && s.state === "succeeded"));
    const old = h.navStatuses().filter((s) => s.seq === 10).map((s) => s.state);
    expect(old.every((s) => s === "accepted" || s === "active")).toBe(true);
    expect(h.navStatuses().at(-1)).toMatchObject({ state: "succeeded", seq: 11 });
  });
});
