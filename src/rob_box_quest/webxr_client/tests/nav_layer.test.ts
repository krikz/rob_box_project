// nav_layer (issue #3151): жест лучом → nav_goal в map, события → HUD,
// кнопка отмены в слое указателя.

import { describe, it, expect, beforeAll } from "vitest";
import { createNavLayer } from "../src/nav/nav_layer";

// jsdom без npm-пакета canvas: getContext не реализован и шумит в stderr.
// nav_overlay без 2D-контекста рисует кнопку отмены однотонной — этого
// тестам достаточно.
beforeAll(() => {
  HTMLCanvasElement.prototype.getContext = (() => null) as unknown as HTMLCanvasElement["getContext"];
});
import { PointerSystem } from "../src/interaction/pointer";
import type { StatusLine } from "../src/scene/status_hud";

function setup(poseAgeOk = true) {
  let t = 1000;
  const sent: Record<string, unknown>[] = [];
  const lines: (StatusLine | null)[] = [];
  const notes: string[] = [];
  const pointer = new PointerSystem();
  const nav = createNavLayer({
    send: (cmd) => {
      sent.push(cmd as unknown as Record<string, unknown>);
      return true;
    },
    onStatusLine: (l) => lines.push(l),
    notify: (text) => notes.push(text),
    pointer,
    now: () => t
  });
  // Робот в (10, 5), смотрит на север (yaw = +π/2).
  nav.setPose({ x: 10, y: 5, yaw: Math.PI / 2 });
  if (!poseAgeOk) t += 5000;
  return { nav, sent, lines, notes, pointer, advance: (ms: number) => (t += ms) };
}

const aimAt = (x: number, z: number, pressed: boolean) => {
  // Луч с высоты 1.6 м в точку пола (x, z).
  const d = { x, y: -1.6, z };
  return { origin: { x: 0, y: 1.6, z: 0 }, direction: d, pressed };
};

describe("nav_layer: цель лучом", () => {
  it("точка 2 м впереди робота → nav_goal в map (на север от робота)", () => {
    const { nav, sent } = setup();
    nav.toggleAim();
    nav.updatePointer(aimAt(0, -2, false), false);
    nav.updatePointer(aimAt(0, -2, true), false);
    nav.updatePointer(aimAt(0, -2, false), false);
    expect(sent).toHaveLength(1);
    const cmd = sent[0];
    expect(cmd.cmd).toBe("nav_goal");
    expect(cmd.frame).toBe("map");
    expect(cmd.x as number).toBeCloseTo(10, 6);
    expect(cmd.y as number).toBeCloseTo(7, 6);
    // Без натяга курс = по ходу движения = курс робота.
    expect(cmd.yaw as number).toBeCloseTo(Math.PI / 2, 6);
    // Одна взводка — одна цель.
    expect(nav.isAiming()).toBe(false);
  });

  it("точка слева от робота → запад от него в map", () => {
    const { nav, sent } = setup();
    nav.toggleAim();
    nav.updatePointer(aimAt(-3, 0, true), false);
    nav.updatePointer(aimAt(-3, 0, false), false);
    expect(sent[0].x as number).toBeCloseTo(7, 6);
    expect(sent[0].y as number).toBeCloseTo(5, 6);
  });

  it("устаревшая поза — цель не уходит, оператору сказано почему", () => {
    const { nav, sent, notes } = setup(false);
    nav.toggleAim();
    nav.updatePointer(aimAt(0, -2, true), false);
    nav.updatePointer(aimAt(0, -2, false), false);
    expect(sent).toHaveLength(0);
    expect(notes[0]).toMatch(/нет свежей позы/);
  });
});

describe("nav_layer: события и HUD", () => {
  it("ack → accepted → active → succeeded; кнопка отмены живёт, пока цель жива", () => {
    const { nav, sent, lines, pointer } = setup();
    nav.toggleAim();
    expect(lines.at(-1)?.value).toBe("ПРИЦЕЛ: пол");
    nav.updatePointer(aimAt(0, -2, true), false);
    nav.updatePointer(aimAt(0, -2, false), false);
    const seq = sent[0].seq as number;
    expect(nav.handleEvent({ type: "nav_goal_ack", seq, ts_ms: 1 })).toBe(true);
    expect(pointer.getTarget("nav:cancel")).not.toBeNull();
    nav.handleEvent({ type: "nav_status", state: "accepted", seq, x: 10, y: 7, yaw: 1.57, ts_ms: 2 });
    nav.handleEvent({ type: "nav_status", state: "active", seq, x: 10, y: 7, yaw: 1.57, distance_remaining: 1.96, ts_ms: 3 });
    expect(lines.at(-1)?.value).toBe("ЕДЕТ 2.0 м");
    nav.handleSelect("nav:cancel");
    expect(sent.at(-1)?.cmd).toBe("nav_cancel");
    nav.handleEvent({ type: "nav_status", state: "canceled", seq, x: 10, y: 7, yaw: 1.57, ts_ms: 4 });
    expect(lines.at(-1)?.value).toBe("ОТМЕНЕНО");
    expect(pointer.getTarget("nav:cancel")).toBeNull();
  });

  it("nack → тост с причиной", () => {
    const { nav, sent, notes, lines } = setup();
    nav.toggleAim();
    nav.updatePointer(aimAt(0, -2, true), false);
    nav.updatePointer(aimAt(0, -2, false), false);
    nav.handleEvent({ type: "nav_goal_nack", seq: sent[0].seq, reason: "emergency_active", ts_ms: 1 });
    expect(notes.at(-1)).toMatch(/аварийный стоп/);
    expect(lines.at(-1)?.value).toBe("НЕ ПРИНЯТО: аварийный стоп");
  });

  it("чужие события не трогает", () => {
    const { nav } = setup();
    expect(nav.handleEvent({ type: "voice_state" })).toBe(false);
    expect(nav.handleEvent(null)).toBe(false);
  });

  it("итог цели гаснет на HUD через 10 с", () => {
    const { nav, lines, advance } = setup();
    nav.handleEvent({ type: "nav_status", state: "succeeded", seq: 5, x: 0, y: 0, yaw: 0, ts_ms: 1 });
    expect(lines.at(-1)?.value).toBe("ПРИЕХАЛ");
    advance(11_000);
    nav.updatePointer(null, false);
    expect(lines.at(-1)).toBeNull();
  });
});
