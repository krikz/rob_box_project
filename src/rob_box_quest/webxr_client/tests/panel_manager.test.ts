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
});

describe("PanelManager — snapToZone", () => {
  it("snaps to left zone when within snap radius", () => {
    const mgr = new PanelManager({ radius: 2.0 });
    const r = mgr.snapToZone("x", -1.7, 0.2, 0.5);
    expect(r.snapped).toBe(true);
    expect(r.zone).toBe("left");
    expect(r.x).toBe(-2.0);
    expect(r.z).toBe(0);
  });

  it("snaps to center zone when close to forward axis", () => {
    const mgr = new PanelManager({ radius: 2.0 });
    const r = mgr.snapToZone("x", 0.1, -1.6, 0.5);
    expect(r.snapped).toBe(true);
    expect(r.zone).toBe("center");
    expect(r.x).toBe(0);
    expect(r.z).toBe(-2.0);
  });

  it("snaps to right zone symmetric to left", () => {
    const mgr = new PanelManager({ radius: 2.0 });
    const r = mgr.snapToZone("x", 1.8, -0.1, 0.5);
    expect(r.snapped).toBe(true);
    expect(r.zone).toBe("right");
    expect(r.x).toBe(2.0);
    expect(r.z).toBe(0);
  });

  it("does not snap when outside snap radius (returns input coords)", () => {
    const mgr = new PanelManager({ radius: 2.0 });
    const r = mgr.snapToZone("x", 0, 0, 0.5);
    // (0, 0) равноудалён от center (0, -2) и left/right — но distance=2,
    // > snapRadius → no snap.
    expect(r.snapped).toBe(false);
    expect(r.zone).toBeNull();
    expect(r.x).toBe(0);
    expect(r.z).toBe(0);
  });

  it("picks nearest zone when in overlap", () => {
    const mgr = new PanelManager({ radius: 2.0 });
    // Точка (0.1, 0):
    //   left  (-2, 0)  → hypot(2.1, 0)   = 2.1
    //   center (0, -2) → hypot(0.1, 2)   = 2.005
    //   right (2, 0)   → hypot(1.9, 0)   = 1.9
    // snapRadius=2.5 → попадает в обе зоны, выбираем ближайшую (right).
    const r = mgr.snapToZone("x", 0.1, 0, 2.5);
    expect(r.snapped).toBe(true);
    expect(r.zone).toBe("right");
  });

  it("respects custom snapRadius", () => {
    const mgr = new PanelManager({ radius: 2.0 });
    expect(mgr.snapToZone("x", -1.0, 0, 0.3).snapped).toBe(false);
    expect(mgr.snapToZone("x", -1.0, 0, 1.5).snapped).toBe(true);
  });
});

describe("PanelManager — moveWithSnap", () => {
  it("applies snap and updates panel position+orientation", () => {
    const mgr = new PanelManager({ radius: 2.0 });
    const [id] = mgr.resetLayout();
    const r = mgr.moveWithSnap(id, -1.9, 0.1, 0.5);
    expect(r.moved).toBe(true);
    expect(r.snapped).toBe(true);
    expect(r.zone).toBe("left");
    const p = mgr.get(id)!;
    expect(p.position.x).toBe(-2.0);
    expect(p.position.z).toBe(0);
    // facing после move(): направлен к началу координат от (-2, 0) → (1, 0).
    expect(p.facing.x).toBeCloseTo(1, 5);
    expect(p.facing.z).toBeCloseTo(0, 5);
  });

  it("keeps original position when no snap", () => {
    const mgr = new PanelManager({ radius: 2.0 });
    const [id] = mgr.resetLayout();
    const r = mgr.moveWithSnap(id, 0.1, 0.1, 0.3);
    expect(r.moved).toBe(true);
    expect(r.snapped).toBe(false);
    expect(r.zone).toBeNull();
    expect(mgr.get(id)!.position.x).toBe(0.1);
    expect(mgr.get(id)!.position.z).toBe(0.1);
  });

  it("returns moved=false for unknown panel id", () => {
    const mgr = new PanelManager({ radius: 2.0 });
    const r = mgr.moveWithSnap("nope", 0, 0, 0.5);
    expect(r.moved).toBe(false);
    expect(r.snapped).toBe(false);
  });
});