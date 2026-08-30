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
// --- Wave 3.A: раскладка мостика (боковые панели вокруг экрана-стены) ---

describe("PanelManager — custom layout (captain bridge)", () => {
  it("honours custom angles", () => {
    const mgr = new PanelManager({
      defaultTopics: ["camera_oak_depth", "camera_ceiling"],
      angles: [-75, 75]
    });
    mgr.resetLayout();
    const panels = mgr.list();
    expect(panels.length).toBe(2);
    // angle=-75° → x = 2*sin(-75°) ≈ -1.932, z = -2*cos(-75°) ≈ -0.518
    expect(panels[0].position.x).toBeCloseTo(2 * Math.sin((-75 * Math.PI) / 180), 5);
    expect(panels[0].position.z).toBeCloseTo(-2 * Math.cos((-75 * Math.PI) / 180), 5);
    expect(panels[1].position.x).toBeCloseTo(2 * Math.sin((75 * Math.PI) / 180), 5);
  });

  it("side panels still face the operator", () => {
    const mgr = new PanelManager({ defaultTopics: ["a", "b"], angles: [-75, 75] });
    mgr.resetLayout();
    for (const p of mgr.list()) {
      const dot = p.facing.x * p.position.x + p.facing.z * p.position.z;
      expect(dot).toBeLessThan(-0.9);
    }
  });

  it("stops at whichever list runs out first", () => {
    const mgr = new PanelManager({ defaultTopics: ["a", "b", "c"], angles: [-45, 45] });
    expect(mgr.resetLayout().length).toBe(2);
  });
});

describe("captain bridge layout constants", () => {
  it("side panels do not duplicate the main screen topic", async () => {
    const { MAIN_SCREEN_TOPIC, SIDE_PANEL_TOPICS, SIDE_PANEL_ANGLES_DEG } = await import(
      "../src/scene/captain_bridge"
    );
    expect(SIDE_PANEL_TOPICS).not.toContain(MAIN_SCREEN_TOPIC);
    // Один угол на панель — иначе панель молча потеряется.
    expect(SIDE_PANEL_ANGLES_DEG.length).toBe(SIDE_PANEL_TOPICS.length);
    // Панели уходят из фронтального сектора, чтобы не перекрывать экран.
    for (const a of SIDE_PANEL_ANGLES_DEG) {
      expect(Math.abs(a)).toBeGreaterThan(60);
    }
  });
});
