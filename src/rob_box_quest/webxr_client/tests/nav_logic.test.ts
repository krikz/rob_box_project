// Чистая логика навигационного слоя (issue #3151): nav_path, след, жест,
// редьюсер статуса, лента.

import { describe, it, expect } from "vitest";
import { encodeMsgpackMap } from "../src/wire/msgpack";
import { parseNavPath } from "../src/nav/nav_path_payload";
import { OdomTrail } from "../src/nav/odom_trail";
import {
  MAX_GOAL_RANGE_M,
  NavGoalGesture,
  YAW_DRAG_MIN_M,
  goalYaw,
  rayFloorHit,
  type GestureRay
} from "../src/nav/nav_goal_gesture";
import { INITIAL_NAV_STATE, navStatusLine, reduceNav, isGoalLive, type NavState } from "../src/nav/nav_state";
import { buildRibbon } from "../src/nav/ribbon";

/** msgpack-кадр nav_path. Кодер клиента bin не умеет — дописываем руками. */
function navPathPayload(points: number[][], frame = "map", nOverride?: number): Uint8Array {
  const base = encodeMsgpackMap({ frame, n: nOverride ?? points.length, ts_ms: 7 });
  const xy = new Uint8Array(points.length * 8);
  const dv = new DataView(xy.buffer);
  points.forEach(([x, y], i) => {
    dv.setFloat32(i * 8, x, true);
    dv.setFloat32(i * 8 + 4, y, true);
  });
  const key = new Uint8Array([0xa2, 0x78, 0x79]); // fixstr "xy"
  const val = new Uint8Array([0xc4, xy.length, ...xy]); // bin8
  const out = new Uint8Array(base.length + key.length + val.length);
  out.set(base, 0);
  out.set(key, base.length);
  out.set(val, base.length + key.length);
  out[0] = 0x80 | ((base[0] & 0x0f) + 1);
  return out;
}

describe("parseNavPath", () => {
  it("читает пары float32 LE", () => {
    const f = parseNavPath(navPathPayload([[1.5, -2], [3.25, 4]]));
    expect(f?.points).toEqual([{ x: 1.5, y: -2 }, { x: 3.25, y: 4 }]);
    expect(f?.tsMs).toBe(7);
  });
  it("n = 0 — пустой путь (сигнал погасить линию)", () => {
    expect(parseNavPath(navPathPayload([]))?.points).toEqual([]);
  });
  it("не map — null", () => {
    expect(parseNavPath(navPathPayload([[0, 0]], "odom"))).toBeNull();
  });
  it("n не совпадает с длиной xy — null", () => {
    expect(parseNavPath(navPathPayload([[0, 0], [1, 1]], "map", 3))).toBeNull();
  });
  it("мусор — null", () => {
    expect(parseNavPath(new Uint8Array([1, 2, 3]))).toBeNull();
  });
});

describe("OdomTrail", () => {
  it("копит, пропускает стояние на месте, гасит по возрасту", () => {
    const t = new OdomTrail({ maxAgeMs: 1000 });
    expect(t.push({ x: 0, y: 0, yaw: 0 }, 0)).toBe(true);
    expect(t.push({ x: 0.01, y: 0, yaw: 0 }, 100)).toBe(false);
    expect(t.push({ x: 0.5, y: 0, yaw: 0 }, 500)).toBe(true);
    const s = t.samples(1000);
    expect(s.map((p) => p.x)).toEqual([0, 0.5]);
    expect(s[0].freshness).toBeCloseTo(0, 6);
    expect(s[1].freshness).toBeCloseTo(0.5, 6);
    expect(t.samples(1600).length).toBe(0);
  });
  it("держит не больше maxPoints", () => {
    const t = new OdomTrail({ maxPoints: 3 });
    for (let i = 0; i < 10; i += 1) t.push({ x: i * 0.1, y: 0, yaw: 0 }, i);
    expect(t.samples(10).map((p) => p.x)).toEqual([0.7, 0.8, 0.9].map((v) => expect.closeTo(v, 6)));
  });
  it("прыжок релокализации начинает след заново", () => {
    const t = new OdomTrail();
    t.push({ x: 0, y: 0, yaw: 0 }, 0);
    t.push({ x: 0.2, y: 0, yaw: 0 }, 1);
    t.push({ x: 8, y: 0, yaw: 0 }, 2);
    expect(t.size()).toBe(1);
  });
});

function ray(origin: [number, number, number], dir: [number, number, number], pressed = false): GestureRay {
  return {
    origin: { x: origin[0], y: origin[1], z: origin[2] },
    direction: { x: dir[0], y: dir[1], z: dir[2] },
    pressed
  };
}

describe("rayFloorHit / goalYaw", () => {
  it("луч вниз-вперёд попадает в пол перед оператором", () => {
    const h = rayFloorHit(ray([0, 1.6, 0], [0, -1, -1]));
    expect(h!.x).toBeCloseTo(0, 9);
    expect(h!.z).toBeCloseTo(-1.6, 9);
  });
  it("луч вверх / вдоль пола / слишком далеко — null", () => {
    expect(rayFloorHit(ray([0, 1.6, 0], [0, 1, -1]))).toBeNull();
    expect(rayFloorHit(ray([0, 1.6, 0], [0, 0, -1]))).toBeNull();
    expect(rayFloorHit(ray([0, 1.6, 0], [0, -0.01, -1]))).toBeNull(); // ~160 м
    expect(MAX_GOAL_RANGE_M).toBe(20);
  });
  it("без натяга курс — по ходу движения от робота к цели", () => {
    expect(goalYaw({ x: -2, z: 0 }, { x: -2, z: 0 })).toBeCloseTo(Math.PI / 2, 9); // цель слева
    expect(goalYaw({ x: 0, z: -3 }, null)).toBeCloseTo(0, 9); // цель впереди
  });
  it("натяг задаёт курс прибытия", () => {
    // Цель впереди, тянем вправо (+X) → курс −π/2 (направо).
    expect(goalYaw({ x: 0, z: -3 }, { x: YAW_DRAG_MIN_M + 0.5, z: -3 })).toBeCloseTo(-Math.PI / 2, 9);
  });
});

describe("NavGoalGesture", () => {
  const down = ray([0, 1.6, 0], [0, -1, -1]);
  it("без взвода клик по полу ничего не делает", () => {
    const g = new NavGoalGesture();
    g.update({ armed: false, ray: { ...down, pressed: true }, blocked: false });
    const out = g.update({ armed: false, ray: down, blocked: false });
    expect(out.commit).toBeNull();
  });
  it("взвёл → прицел на полу → нажал → отпустил = commit", () => {
    const g = new NavGoalGesture();
    expect(g.update({ armed: true, ray: down, blocked: false }).reticle).not.toBeNull();
    const held = g.update({ armed: true, ray: { ...down, pressed: true }, blocked: false });
    expect(held.preview?.point.z).toBeCloseTo(-1.6, 9);
    const out = g.update({ armed: true, ray: down, blocked: false });
    expect(out.commit?.point.z).toBeCloseTo(-1.6, 9);
    expect(out.commit?.yaw).toBeCloseTo(0, 9);
  });
  it("натяг во время удержания меняет курс", () => {
    const g = new NavGoalGesture();
    g.update({ armed: true, ray: down, blocked: false });
    g.update({ armed: true, ray: { ...down, pressed: true }, blocked: false });
    // Луч уехал влево: точка (−1, −1.6).
    g.update({ armed: true, ray: ray([0, 1.6, 0], [-1 / 1.6, -1, -1], true), blocked: false });
    const out = g.update({ armed: true, ray: ray([0, 1.6, 0], [-1 / 1.6, -1, -1], false), blocked: false });
    expect(out.commit?.point.z).toBeCloseTo(-1.6, 9);
    expect(out.commit?.yaw).toBeCloseTo(Math.PI / 2, 6);
  });
  it("панель под лучом приоритетнее пола", () => {
    const g = new NavGoalGesture();
    g.update({ armed: true, ray: down, blocked: true });
    g.update({ armed: true, ray: { ...down, pressed: true }, blocked: true });
    expect(g.update({ armed: true, ray: down, blocked: true }).commit).toBeNull();
  });
  it("разрядка или потеря луча посреди жеста — отмена", () => {
    const g = new NavGoalGesture();
    g.update({ armed: true, ray: down, blocked: false });
    g.update({ armed: true, ray: { ...down, pressed: true }, blocked: false });
    g.update({ armed: false, ray: { ...down, pressed: true }, blocked: false });
    expect(g.update({ armed: true, ray: down, blocked: false }).commit).toBeNull();
    g.update({ armed: true, ray: { ...down, pressed: true }, blocked: false });
    g.update({ armed: true, ray: null, blocked: false });
    expect(g.isHolding()).toBe(false);
  });
});

describe("reduceNav / navStatusLine", () => {
  const goal = { seq: 1, x: 1, y: 2, yaw: 0 };
  const status = (state: string, seq = 1, distanceM: number | null = null): Parameters<typeof reduceNav>[1] => ({
    kind: "status",
    state,
    seq,
    x: 1,
    y: 2,
    yaw: 0,
    distanceM,
    reason: null
  });

  it("sent → ack → accepted → active → succeeded", () => {
    let s: NavState = reduceNav(INITIAL_NAV_STATE, { kind: "sent", goal });
    expect(navStatusLine(s, false)?.value).toBe("отправка…");
    s = reduceNav(s, { kind: "ack", seq: 1 });
    // ack — «ушла в Nav2», не «едет».
    expect(navStatusLine(s, false)?.value).toBe("ОТПРАВЛЕНО");
    s = reduceNav(s, status("accepted"));
    s = reduceNav(s, status("active", 1, 2.84));
    expect(navStatusLine(s, false)?.value).toBe("ЕДЕТ 2.8 м");
    s = reduceNav(s, status("active", 1, null)); // feedback без расстояния
    expect(s.distanceM).toBe(2.84);
    s = reduceNav(s, status("succeeded"));
    expect(isGoalLive(s)).toBe(false);
    expect(navStatusLine(s, false)?.value).toBe("ПРИЕХАЛ");
  });
  it("nack с причиной", () => {
    let s = reduceNav(INITIAL_NAV_STATE, { kind: "sent", goal });
    s = reduceNav(s, { kind: "nack", seq: 1, reason: "floor_held" });
    expect(navStatusLine(s, false)).toEqual({ label: "NAV", value: "НЕ ПРИНЯТО: руль у другого", level: "bad" });
  });
  it("терминальный статус чужой цели не гасит нашу", () => {
    let s = reduceNav(INITIAL_NAV_STATE, { kind: "sent", goal });
    s = reduceNav(s, status("accepted"));
    s = reduceNav(s, status("aborted", 99));
    expect(s.phase).toBe("accepted");
  });
  it("разрыв связи посреди цели — «нет связи», а не «едет»", () => {
    let s = reduceNav(INITIAL_NAV_STATE, { kind: "sent", goal });
    s = reduceNav(s, status("active", 1, 3));
    s = reduceNav(s, { kind: "disconnected" });
    expect(navStatusLine(s, false)?.value).toBe("нет связи");
  });
  it("прицел перекрывает строку", () => {
    expect(navStatusLine(INITIAL_NAV_STATE, true)?.value).toBe("ПРИЦЕЛ: пол");
    expect(navStatusLine(INITIAL_NAV_STATE, false)).toBeNull();
  });
});

describe("buildRibbon", () => {
  it("две вершины на точку, два треугольника на сегмент, ширина по нормали", () => {
    const r = buildRibbon(
      [
        { x: 0, z: 0 },
        { x: 0, z: -1 },
        { x: 0, z: -2 }
      ],
      0.1,
      0.08
    );
    expect(r.positions.length).toBe(3 * 2 * 3);
    expect(r.indices.length).toBe(2 * 6);
    // Путь вдоль −Z → лента раздвинута по X на ±0.1.
    expect(Math.abs(r.positions[0] - r.positions[3])).toBeCloseTo(0.2, 6);
    expect(r.positions[1]).toBeCloseTo(0.08, 6);
  });
  it("меньше двух точек — пусто", () => {
    expect(buildRibbon([{ x: 0, z: 0 }], 0.1, 0).indices.length).toBe(0);
  });
});
