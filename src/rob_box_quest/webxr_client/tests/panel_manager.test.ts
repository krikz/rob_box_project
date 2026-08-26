import { describe, it, expect, beforeEach } from "vitest";
import { PanelManager, DEFAULT_VIDEO_TOPICS } from "../src/scene/panel_manager";

describe("PanelManager", () => {
  let mgr: PanelManager;
  beforeEach(() => {
    mgr = new PanelManager();
  });

  it("resetLayout creates 4 panels on semicircle facing inward", () => {
    const ids = mgr.resetLayout();
    expect(ids.length).toBe(4);
    expect(mgr.count()).toBe(4);
    for (const p of mgr.list()) {
      // Position должна лежать на окружности радиуса 2 ± 1e-6.
      const r = Math.hypot(p.position.x, p.position.z);
      expect(r).toBeCloseTo(2.0, 5);
      // facing указывает к началу координат (отрицательный вектор позиции).
      const dot = p.facing.x * p.position.x + p.facing.z * p.position.z;
      expect(dot).toBeLessThan(-0.9); // nearly opposite
    }
  });

  it("default topics match design §1", () => {
    const ids = mgr.resetLayout();
    const topics = mgr.list().map((p) => p.topic);
    expect(topics).toEqual([...DEFAULT_VIDEO_TOPICS]);
    // id'ы уникальны
    expect(new Set(ids).size).toBe(ids.length);
  });

  it("createPanel adds new panel", () => {
    mgr.resetLayout();
    const id = mgr.createPanel("lidar_2d", { x: 1, y: 0, z: -1 }, { x: -1, z: 1 });
    expect(mgr.count()).toBe(5);
    expect(mgr.get(id)?.topic).toBe("lidar_2d");
  });

  it("close removes a panel", () => {
    const ids = mgr.resetLayout();
    const removed = mgr.close(ids[0]);
    expect(removed).toBe(true);
    expect(mgr.count()).toBe(3);
    expect(mgr.get(ids[0])).toBeUndefined();
  });

  it("switchStream changes topic of existing panel", () => {
    const [id] = mgr.resetLayout();
    const ok = mgr.switchStream(id, "camera_ceiling");
    expect(ok).toBe(true);
    expect(mgr.get(id)?.topic).toBe("camera_ceiling");
  });

  it("move updates position and re-orients facing", () => {
    const [id] = mgr.resetLayout();
    mgr.move(id, 1, -1);
    const p = mgr.get(id);
    expect(p).not.toBeUndefined();
    expect(p!.position.x).toBe(1);
    expect(p!.position.z).toBe(-1);
    // facing нормализован и направлен к началу координат.
    const len = Math.hypot(p!.facing.x, p!.facing.z);
    expect(len).toBeCloseTo(1, 5);
    const dot = p!.facing.x * 1 + p!.facing.z * -1;
    expect(dot).toBeLessThan(-0.9);
  });

  it("select marks only one panel selected", () => {
    const [a, b] = mgr.resetLayout();
    mgr.select(a);
    expect(mgr.get(a)?.selected).toBe(true);
    expect(mgr.get(b)?.selected).toBe(false);
    mgr.select(b);
    expect(mgr.get(a)?.selected).toBe(false);
    expect(mgr.get(b)?.selected).toBe(true);
  });

  it("returns null on close of unknown id", () => {
    expect(mgr.close("nope")).toBe(false);
  });

  it("returns false on move/switch of unknown id", () => {
    expect(mgr.move("nope", 0, 0)).toBe(false);
    expect(mgr.switchStream("nope", "x")).toBe(false);
  });

  it("list() returns defensive copies", () => {
    const [id] = mgr.resetLayout();
    const p1 = mgr.list()[0];
    p1.position.x = 999;
    expect(mgr.get(id)?.position.x).not.toBe(999);
  });

  // ---------- Phase 2 §2.3: resize ----------

  it("resize enforces min/max bounds", () => {
    const [id] = mgr.resetLayout();
    // Из дефолта 1.2×0.7 → aspect ≈ 1.714. Просим (0.1, 0.1) → clamp width до 0.4,
    // затем aspect-preserving height = 0.4 / 1.714 = 0.234, clamp до 0.3.
    expect(mgr.resize(id, 0.1, 0.1)).toBe(true);
    expect(mgr.get(id)?.size.width).toBeCloseTo(0.4, 5);
    expect(mgr.get(id)?.size.height).toBeCloseTo(0.3, 5);
    // Из (0.4, 0.3) просим (10, 10): widthChanged=9.6, heightChanged=9.7,
    // height-ветка побеждает: w = clamp(2.0 * (0.4/0.3), 0.4, 3.0) = clamp(2.667,...) = 2.667,
    // h = 2.0 (clamped).
    expect(mgr.resize(id, 10, 10)).toBe(true);
    expect(mgr.get(id)?.size.width).toBeCloseTo(2.667, 3);
    expect(mgr.get(id)?.size.height).toBeCloseTo(2.0, 5);
    // А с freeAspect — оба clamp-нутых максимума.
    expect(mgr.resize(id, 10, 10, { freeAspect: true })).toBe(true);
    expect(mgr.get(id)?.size.width).toBeCloseTo(3.0, 5);
    expect(mgr.get(id)?.size.height).toBeCloseTo(2.0, 5);
  });

  it("resize preserves aspect ratio by default", () => {
    const [id] = mgr.resetLayout();
    // default 1.2 × 0.7 → aspect ≈ 1.714
    const initial = mgr.get(id)!;
    const aspect = initial.size.width / initial.size.height;
    mgr.resize(id, 2.0, 2.0); // widthChanged=0.8, heightChanged=1.3 → height wins
    const after = mgr.get(id)!;
    // width = clamp(height * aspect, 0.4, 3.0) = clamp(2.0 * 1.714, ...) = 3.0
    expect(after.size.width).toBeCloseTo(3.0, 5);
    expect(after.size.height).toBeCloseTo(2.0, 5);
    void aspect;
  });

  it("resize with freeAspect ignores aspect lock", () => {
    const [id] = mgr.resetLayout();
    expect(mgr.resize(id, 2.0, 0.5, { freeAspect: true })).toBe(true);
    const p = mgr.get(id)!;
    expect(p.size.width).toBeCloseTo(2.0, 5);
    expect(p.size.height).toBeCloseTo(0.5, 5);
  });

  it("resize returns false when no change", () => {
    const [id] = mgr.resetLayout();
    const s = mgr.get(id)!;
    expect(mgr.resize(id, s.size.width, s.size.height)).toBe(false);
  });

  it("resize returns false on unknown id", () => {
    expect(mgr.resize("nope", 1, 1)).toBe(false);
  });

  // ---------- Phase 2 §2.4: opacity ----------

  it("default opacity is 100%", () => {
    const [id] = mgr.resetLayout();
    expect(mgr.get(id)?.opacity).toBe(1.0);
  });

  it("setOpacity applies valid levels", () => {
    const [id] = mgr.resetLayout();
    expect(mgr.setOpacity(id, 0.5)).toBe(true);
    expect(mgr.get(id)?.opacity).toBe(0.5);
    expect(mgr.setOpacity(id, 0.75)).toBe(true);
    expect(mgr.get(id)?.opacity).toBe(0.75);
    expect(mgr.setOpacity(id, 1.0)).toBe(true);
    expect(mgr.get(id)?.opacity).toBe(1.0);
  });

  it("setOpacity returns false when no change", () => {
    const [id] = mgr.resetLayout();
    expect(mgr.setOpacity(id, 1.0)).toBe(false);
  });

  it("cycleOpacity goes 100 → 75 → 50 → 100", () => {
    const [id] = mgr.resetLayout();
    expect(mgr.cycleOpacity(id)).toBe(0.75);
    expect(mgr.cycleOpacity(id)).toBe(0.5);
    expect(mgr.cycleOpacity(id)).toBe(1.0);
    expect(mgr.cycleOpacity(id)).toBe(0.75);
  });

  it("setOpacity/cycleOpacity return false/null on unknown id", () => {
    expect(mgr.setOpacity("nope", 0.5)).toBe(false);
    expect(mgr.cycleOpacity("nope")).toBeNull();
  });

  // ---------- Phase 2 §2.1+§2.2: move + snap-to-zone ----------

  it("move classifies snapZone by X-threshold", () => {
    const [id] = mgr.resetLayout();
    mgr.move(id, -1.5, -2); // left
    expect(mgr.get(id)?.snapZone).toBe("left");
    mgr.move(id, 0, -2); // center
    expect(mgr.get(id)?.snapZone).toBe("center");
    mgr.move(id, 1.5, -2); // right
    expect(mgr.get(id)?.snapZone).toBe("right");
  });

  it("finalizeSnap pulls panel to nearest anchor within 0.5m", () => {
    const [id] = mgr.resetLayout();
    // Свободная позиция рядом с left anchor (-1.5, -2).
    mgr.move(id, -1.3, -1.8);
    expect(mgr.get(id)?.snapZone).toBe("left");
    mgr.finalizeSnap(id);
    const p = mgr.get(id)!;
    expect(p.position.x).toBeCloseTo(-1.5, 5);
    expect(p.position.z).toBeCloseTo(-2.0, 5);
    expect(p.snapZone).toBe("left");
  });

  it("finalizeSnap leaves panel in free zone when too far", () => {
    const [id] = mgr.resetLayout();
    mgr.move(id, -1.2, -1.5); // near but not on left anchor
    // Расстояние до left anchor ≈ sqrt(0.09+0.25)≈0.58 → чуть больше 0.5
    mgr.finalizeSnap(id, 0.5);
    const p = mgr.get(id)!;
    expect(p.snapZone).toBe("free");
    // позиция не должна измениться
    expect(p.position.x).toBeCloseTo(-1.2, 5);
  });

  it("finalizeSnap returns null on unknown id", () => {
    expect(mgr.finalizeSnap("nope")).toBeNull();
  });

  // ---------- Phase 2 §2.6: persistence ----------

  it("toJSON returns version 1 with all panels", () => {
    mgr.resetLayout();
    const json = mgr.toJSON();
    expect(json.version).toBe(1);
    expect(json.panels.length).toBe(4);
    // Каждый panel должен иметь opacity/snapZone
    for (const p of json.panels) {
      expect(typeof p.opacity).toBe("number");
      expect(["left", "center", "right", "free"]).toContain(p.snapZone);
    }
  });

  it("fromJSON restores panels", () => {
    mgr.resetLayout();
    const json = mgr.toJSON();
    // mutate
    mgr.createPanel("lidar_2d");
    expect(mgr.count()).toBe(5);
    // restore
    expect(mgr.fromJSON(json)).toBe(true);
    expect(mgr.count()).toBe(4);
  });

  it("fromJSON rejects unknown version", () => {
    expect(mgr.fromJSON({ version: 99, panels: [] })).toBe(false);
  });

  it("fromJSON rejects malformed panels", () => {
    mgr.resetLayout();
    expect(mgr.fromJSON({ version: 1, panels: [{ bad: true } as unknown as never] })).toBe(false);
    // оригинальные panels не должны пострадать при отказе
    expect(mgr.count()).toBe(4);
  });

  it("onChange fires events for create/close/move/resize/opacity/reset", () => {
    const events: string[] = [];
    mgr.onChange((e) => events.push(e.type));
    const ids = mgr.resetLayout();
    events.length = 0; // сбросить reset-событие
    mgr.createPanel("lidar_2d");
    mgr.move(ids[0], 1, -1);
    mgr.resize(ids[0], 2, 1.2);
    mgr.setOpacity(ids[0], 0.5);
    mgr.close(ids[1]);
    expect(events).toContain("created");
    expect(events).toContain("moved");
    expect(events).toContain("resized");
    expect(events).toContain("opacity");
    expect(events).toContain("closed");
  });

  // ---------- Phase 2 §2.7: XR drag API ----------

  it("startDragFromXr captures offset between panel and controller", () => {
    const [id] = mgr.resetLayout();
    expect(mgr.startDragFromXr(id, { x: 1.0, z: -1.0 })).toBe(true);
    expect(mgr.isXrDragging()).toBe(true);
  });

  it("updateDragFromXr moves panel following controller (with offset)", () => {
    const [id] = mgr.resetLayout();
    // panel: position ≈ (sin(-60°)*2, -cos(-60°)*2) = (-1.732, -1.0)
    mgr.startDragFromXr(id, { x: 0, z: 0 });
    // controller двинулся на (0.5, -0.5), панель двинулась на то же + offset
    mgr.updateDragFromXr({ x: 0.5, z: -0.5 });
    const p = mgr.get(id)!;
    // Оригинальная panel-центр: (-1.732, -1.0). Offset = (-1.732 - 0, -1.0 - 0).
    // Новая позиция = (0.5, -0.5) + (-1.732, -1.0) = (-1.232, -1.5).
    expect(p.position.x).toBeCloseTo(-1.232, 3);
    expect(p.position.z).toBeCloseTo(-1.5, 3);
  });

  it("updateDragFromXr without active drag is no-op", () => {
    expect(mgr.updateDragFromXr({ x: 0, z: 0 })).toBe(false);
  });

  it("endDragFromXr finalizes snap zone", () => {
    const [id] = mgr.resetLayout();
    mgr.startDragFromXr(id, { x: 0, z: 0 });
    // Перетащим panel близко к right anchor (+1.5, -2.0).
    // offset = (orig - 0, 0). orig = (-1.732, -1.0).
    // Цель: панель в (+1.4, -1.9) → controller = (1.4 - (-1.732), -1.9 - (-1.0)) = (3.132, -0.9).
    mgr.updateDragFromXr({ x: 3.132, z: -0.9 });
    expect(mgr.get(id)?.snapZone).toBe("right");
    const zone = mgr.endDragFromXr();
    expect(zone).toBe("right");
    // Панель должна быть притянута к anchor (+1.5, -2.0).
    const p = mgr.get(id)!;
    expect(p.position.x).toBeCloseTo(1.5, 5);
    expect(p.position.z).toBeCloseTo(-2.0, 5);
    expect(mgr.isXrDragging()).toBe(false);
  });

  it("endDragFromXr without active drag returns null", () => {
    expect(mgr.endDragFromXr()).toBeNull();
  });

  it("startDragFromXr returns false on unknown id", () => {
    expect(mgr.startDragFromXr("nope", { x: 0, z: 0 })).toBe(false);
  });
});