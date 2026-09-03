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

  it("потолочная камера — на потолочном экране, а не на боковой панели", async () => {
    const { CEILING_SCREEN_TOPIC, SIDE_PANEL_TOPICS, MAIN_SCREEN_TOPIC } = await import(
      "../src/scene/captain_bridge"
    );
    // Иначе один и тот же поток показывался бы дважды.
    expect(SIDE_PANEL_TOPICS).not.toContain(CEILING_SCREEN_TOPIC);
    expect(CEILING_SCREEN_TOPIC).not.toBe(MAIN_SCREEN_TOPIC);
  });

  it("потолочный экран висит над глазами и смотрит нормалью в оператора", async () => {
    const { CEILING_SCREEN_POS, EYE_HEIGHT_M, ceilingScreenPitchRad } = await import(
      "../src/scene/captain_bridge"
    );
    // Над головой и впереди — «поднял голову и увидел, что над роботом».
    expect(CEILING_SCREEN_POS.y).toBeGreaterThan(EYE_HEIGHT_M);
    expect(CEILING_SCREEN_POS.z).toBeLessThan(0);
    // Тот же азимут, что у экрана-стены: на роботе обе камеры стоят на
    // осевой линии и различаются только направлением взгляда.
    expect(CEILING_SCREEN_POS.x).toBe(0);
    // Комната 3 м высотой — экран должен помещаться под потолок.
    expect(CEILING_SCREEN_POS.y).toBeLessThan(3);

    const pitch = ceilingScreenPitchRad();
    // Наклон вниз-назад, к оператору, и не «плашмя в потолок».
    expect(pitch).toBeGreaterThan(0);
    expect(pitch).toBeLessThan(Math.PI / 2);

    // Нормаль плоскости после поворота вокруг X: (0, -sin φ, cos φ).
    // Она обязана смотреть из центра экрана в глаза оператора.
    const nx = 0;
    const ny = -Math.sin(pitch);
    const nz = Math.cos(pitch);
    const toEye = {
      x: -CEILING_SCREEN_POS.x,
      y: EYE_HEIGHT_M - CEILING_SCREEN_POS.y,
      z: -CEILING_SCREEN_POS.z
    };
    const len = Math.hypot(toEye.x, toEye.y, toEye.z);
    const dot = (nx * toEye.x + ny * toEye.y + nz * toEye.z) / len;
    expect(dot).toBeCloseTo(1, 5);
  });
});

// --- AV-25: resize + createPanelWithId ---

describe("PanelManager.resize (AV-25)", () => {
  it("changes width/height", () => {
    const mgr = new PanelManager();
    const [id] = mgr.resetLayout();
    const ok = mgr.resize(id, 1.6, 0.9);
    expect(ok).toBe(true);
    expect(mgr.get(id)?.size.width).toBe(1.6);
    expect(mgr.get(id)?.size.height).toBe(0.9);
  });

  it("clamps to minWidthM / minHeightM (panel can't shrink to a dot)", () => {
    const mgr = new PanelManager();
    const [id] = mgr.resetLayout();
    mgr.resize(id, 0.01, 0.01);
    const p = mgr.get(id)!;
    expect(p.size.width).toBe(0.4); // PANEL_DEFAULT_MIN_WIDTH_M
    expect(p.size.height).toBe(0.3); // PANEL_DEFAULT_MIN_HEIGHT_M
  });

  it("clamps to maxWidthM / maxHeightM (panel can't fill the room)", () => {
    const mgr = new PanelManager();
    const [id] = mgr.resetLayout();
    mgr.resize(id, 99, 99);
    const p = mgr.get(id)!;
    expect(p.size.width).toBe(3.0); // PANEL_DEFAULT_MAX_WIDTH_M
    expect(p.size.height).toBe(2.0); // PANEL_DEFAULT_MAX_HEIGHT_M
  });

  it("returns false when value didn't change (caller's debounce-friendly)", () => {
    const mgr = new PanelManager();
    const [id] = mgr.resetLayout();
    // После resetLayout размеры = дефолт (1.2 × 0.7). Резайз в те же — false.
    expect(mgr.resize(id, 1.2, 0.7)).toBe(false);
  });

  it("returns false for unknown id", () => {
    expect(new PanelManager().resize("nope", 1, 1)).toBe(false);
  });

  it("respects custom min/max bounds from opts", () => {
    const mgr = new PanelManager({ minWidthM: 1.0, maxWidthM: 2.0, minHeightM: 0.5, maxHeightM: 1.0 });
    const [id] = mgr.resetLayout();
    mgr.resize(id, 0.1, 0.1);
    expect(mgr.get(id)?.size.width).toBe(1.0);
    expect(mgr.get(id)?.size.height).toBe(0.5);
    mgr.resize(id, 9, 9);
    expect(mgr.get(id)?.size.width).toBe(2.0);
    expect(mgr.get(id)?.size.height).toBe(1.0);
  });
});

describe("PanelManager.createPanel — overload with id (AV-25)", () => {
  it("creates a panel with the requested id", () => {
    const mgr = new PanelManager();
    const id = mgr.createPanel("p100", "camera_rear");
    expect(id).toBe("p100");
    expect(mgr.get("p100")?.topic).toBe("camera_rear");
    expect(mgr.list().length).toBe(1);
  });

  it("returns existing id without mutation if id already taken", () => {
    const mgr = new PanelManager();
    const first = mgr.createPanel("p100", "camera_rear");
    expect(first).toBe("p100");
    // Повторный вызов с тем же id — тихий no-op, не дубликат.
    const second = mgr.createPanel("p100", "camera_ceiling");
    expect(second).toBe("p100");
    expect(mgr.list().length).toBe(1);
    // topic не поменялся (потому что это no-op).
    expect(mgr.get("p100")?.topic).toBe("camera_rear");
  });

  it("bumps nextId above restored id so autogeneration does not collide", () => {
    const mgr = new PanelManager();
    mgr.createPanel("p50", "camera_rear");
    const fresh = mgr.createPanel("camera_oak_color");
    expect(fresh).not.toBe("p50");
    // Свежесозданная должна иметь id выше, чем p50.
    expect(parseInt(fresh.slice(1), 10)).toBeGreaterThan(50);
  });

  it("honours position and facing when id overload is used", () => {
    const mgr = new PanelManager();
    const pos = { x: 1.2, y: 1.5, z: -0.8 };
    const facing = { x: -1, z: 0.5 };
    const id = mgr.createPanel("p7", "camera_oak_color", pos, facing);
    expect(id).toBe("p7");
    const p = mgr.get("p7")!;
    expect(p.position.x).toBe(1.2);
    expect(p.position.y).toBe(1.5);
    expect(p.position.z).toBe(-0.8);
    expect(p.facing.x).toBe(-1);
    expect(p.facing.z).toBe(0.5);
  });
});
