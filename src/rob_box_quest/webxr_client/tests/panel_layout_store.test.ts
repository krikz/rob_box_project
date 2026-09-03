// Тесты layout-store (AV-25 / B1).
//
// Проверяем по acceptance:
//   - ключ ровно rob_box_quest.panel_layout.v1;
//   - round-trip serialize → parse → apply;
//   - битый JSON → default (parseLayout возвращает null + console.warn);
//   - чужой version → default;
//   - топик не из реестра → панель отбрасывается, остальные живут;
//   - частично битая панель → клампится по границам;
//   - сохранение с дебаунсом (fake-timers).

import { describe, it, expect, beforeEach, afterEach, vi } from "vitest";
import {
  applyLayout,
  createLayoutSaver,
  PANEL_LAYOUT_STORAGE_KEY,
  PANEL_MAX_HEIGHT_M,
  PANEL_MAX_WIDTH_M,
  PANEL_MIN_HEIGHT_M,
  PANEL_MIN_WIDTH_M,
  parseLayout,
  serializeLayout,
  type LayoutStorage,
  type PersistedLayout
} from "../src/scene/panel_layout_store";
import type { PanelState } from "../src/scene/panel_manager";

class MemoryStorage implements LayoutStorage {
  private data = new Map<string, string>();
  getItem(key: string): string | null {
    return this.data.has(key) ? this.data.get(key)! : null;
  }
  setItem(key: string, value: string): void {
    this.data.set(key, value);
  }
  removeItem(key: string): void {
    this.data.delete(key);
  }
  /** Тестовый хелпер: посмотреть «сырое» значение. */
  peek(key: string): string | null {
    return this.data.get(key) ?? null;
  }
}

function makePanel(over: Partial<PanelState> = {}): PanelState {
  return {
    id: "p1",
    topic: "camera_rear",
    position: { x: 0, y: 1.6, z: -2 },
    facing: { x: 0, z: 1 },
    size: { width: 1.2, height: 0.7 },
    selected: false,
    ...over
  };
}

const KNOWN = new Set<string>([
  "camera_rear",
  "camera_oak_color",
  "camera_oak_depth",
  "camera_ceiling"
]);

describe("panel_layout_store — key & round-trip", () => {
  it("uses the exact storage key from the audit", () => {
    expect(PANEL_LAYOUT_STORAGE_KEY).toBe("rob_box_quest.panel_layout.v1");
  });

  it("serialize → parse round-trip preserves all fields", () => {
    const panels: PanelState[] = [
      makePanel({ id: "p2", topic: "camera_oak_color", position: { x: 2, y: 1.6, z: -1.5 } }),
      makePanel({ id: "p1", topic: "camera_rear", position: { x: -1, y: 1.7, z: -1 } }),
      makePanel({ id: "p3", topic: "camera_oak_depth", size: { width: 0.8, height: 0.5 } })
    ];
    const serialized = serializeLayout(panels);
    expect(serialized.version).toBe(1);
    expect(serialized.panels.map((p) => p.id)).toEqual(["p1", "p2", "p3"]); // отсортированы по id
    const raw = JSON.stringify(serialized);
    const parsed = parseLayout(raw, KNOWN);
    expect(parsed).not.toBeNull();
    expect(parsed!.panels).toHaveLength(3);
    const p2 = parsed!.panels.find((p) => p.id === "p2")!;
    expect(p2.topic).toBe("camera_oak_color");
    // Азимут от позиции (x=2, z=-1.5): atan2(2, 1.5) ≈ 53°.
    expect(p2.angleDeg).toBeCloseTo(53.13, 1);
    expect(p2.heightM).toBeCloseTo(1.6, 5);
    expect(p2.widthM).toBeCloseTo(1.2, 5);
    expect(p2.heightPanelM).toBeCloseTo(0.7, 5);
  });

  it("serialize is stable across runs (sorted by id)", () => {
    const a = serializeLayout([makePanel({ id: "p2" }), makePanel({ id: "p1" })]);
    const b = serializeLayout([makePanel({ id: "p1" }), makePanel({ id: "p2" })]);
    expect(JSON.stringify(a)).toBe(JSON.stringify(b));
  });
});

describe("panel_layout_store — degradation", () => {
  let warnSpy: ReturnType<typeof vi.spyOn>;
  beforeEach(() => {
    warnSpy = vi.spyOn(console, "warn").mockImplementation(() => undefined);
  });
  afterEach(() => {
    warnSpy.mockRestore();
  });

  it("returns null for invalid JSON", () => {
    const result = parseLayout("not json{", KNOWN);
    expect(result).toBeNull();
    expect(warnSpy).toHaveBeenCalled();
  });

  it("returns null for non-object payload", () => {
    expect(parseLayout(JSON.stringify("just a string"), KNOWN)).toBeNull();
    expect(parseLayout(JSON.stringify(42), KNOWN)).toBeNull();
    expect(parseLayout(JSON.stringify([1, 2]), KNOWN)).toBeNull();
    expect(warnSpy).toHaveBeenCalled();
  });

  it("returns null for unknown version", () => {
    const raw = JSON.stringify({ version: 2, panels: [] });
    expect(parseLayout(raw, KNOWN)).toBeNull();
    expect(warnSpy).toHaveBeenCalled();
  });

  it("returns null when panels is not an array", () => {
    const raw = JSON.stringify({ version: 1, panels: { not: "array" } });
    expect(parseLayout(raw, KNOWN)).toBeNull();
    expect(warnSpy).toHaveBeenCalled();
  });

  it("drops panels with unknown topic and keeps the rest", () => {
    const raw = JSON.stringify({
      version: 1,
      panels: [
        { id: "p1", topic: "camera_rear", angleDeg: 0, heightM: 1.6, widthM: 1.2, heightPanelM: 0.7 },
        { id: "p2", topic: "camera_unknown", angleDeg: 30, heightM: 1.6, widthM: 1.0, heightPanelM: 0.7 },
        { id: "p3", topic: "camera_oak_color", angleDeg: -30, heightM: 1.6, widthM: 1.0, heightPanelM: 0.7 }
      ]
    });
    const parsed = parseLayout(raw, KNOWN);
    expect(parsed).not.toBeNull();
    expect(parsed!.panels.map((p) => p.id)).toEqual(["p1", "p3"]);
    expect(warnSpy).toHaveBeenCalled();
  });

  it("returns null when every panel is unusable", () => {
    const raw = JSON.stringify({
      version: 1,
      panels: [
        { id: "p1", topic: "unknown_topic", angleDeg: 0, heightM: 1.6, widthM: 1.2, heightPanelM: 0.7 }
      ]
    });
    expect(parseLayout(raw, KNOWN)).toBeNull();
    expect(warnSpy).toHaveBeenCalled();
  });

  it("clamps partially broken panel sizes (does not drop)", () => {
    // size widthM=0.05 (ниже MIN) и heightPanelM=99 (выше MAX) — панель
    // не отбрасывается целиком, размер клампится.
    const raw = JSON.stringify({
      version: 1,
      panels: [
        {
          id: "p1",
          topic: "camera_rear",
          angleDeg: 0,
          heightM: 1.6,
          widthM: 0.05,
          heightPanelM: 99
        }
      ]
    });
    const parsed = parseLayout(raw, KNOWN);
    expect(parsed).not.toBeNull();
    expect(parsed!.panels[0].widthM).toBe(PANEL_MIN_WIDTH_M);
    expect(parsed!.panels[0].heightPanelM).toBe(PANEL_MAX_HEIGHT_M);
    // И наоборот: widthM=99 → MAX.
    const raw2 = JSON.stringify({
      version: 1,
      panels: [
        {
          id: "p1",
          topic: "camera_rear",
          angleDeg: 0,
          heightM: 1.6,
          widthM: 99,
          heightPanelM: 0.01
        }
      ]
    });
    const parsed2 = parseLayout(raw2, KNOWN)!;
    expect(parsed2.panels[0].widthM).toBe(PANEL_MAX_WIDTH_M);
    expect(parsed2.panels[0].heightPanelM).toBe(PANEL_MIN_HEIGHT_M);
  });
});

describe("panel_layout_store — applyLayout", () => {
  it("creates missing, updates existing, removes extras", () => {
    // Mock PanelManager с подсчётом операций.
    const ops: string[] = [];
    const existing = new Map<string, PanelState>();
    const fake = {
      list: () => [...existing.values()],
      close: (id: string) => {
        if (existing.has(id)) {
          existing.delete(id);
          ops.push(`close:${id}`);
          return true;
        }
        return false;
      },
      move: (id: string, x: number, z: number, y?: number) => {
        const p = existing.get(id);
        if (!p) return false;
        p.position.x = x;
        p.position.z = z;
        if (y !== undefined) p.position.y = y;
        ops.push(`move:${id}`);
        return true;
      },
      createPanel: (
        id: string,
        topic: string,
        _position?: { x: number; y: number; z: number },
        _facing?: { x: number; z: number }
      ) => {
        existing.set(id, {
          id,
          topic,
          position: { x: 0, y: 1.6, z: -2 },
          facing: { x: 0, z: 1 },
          size: { width: 1.2, height: 0.7 },
          selected: false
        });
        ops.push(`create:${id}/${topic}`);
        return id;
      },
      resize: (id: string, w: number, h: number) => {
        const p = existing.get(id);
        if (!p) return false;
        p.size.width = w;
        p.size.height = h;
        ops.push(`resize:${id}`);
        return true;
      }
    };

    // До apply: одна панель p1 (existing), и лишняя extra.
    existing.set("p1", makePanel({ id: "p1", topic: "camera_rear" }));
    existing.set("extra", makePanel({ id: "extra", topic: "camera_oak_depth" }));

    const layout = {
      version: 1 as const,
      panels: [
        { id: "p1", topic: "camera_rear", angleDeg: 30, heightM: 1.7, widthM: 1.5, heightPanelM: 0.9 },
        { id: "p2", topic: "camera_oak_color", angleDeg: -30, heightM: 1.5, widthM: 1.0, heightPanelM: 0.6 }
      ]
    };
    const changed = applyLayout(layout, fake);
    expect(changed).toBeGreaterThan(0);
    expect(existing.has("p1")).toBe(true);
    expect(existing.has("p2")).toBe(true);
    expect(existing.has("extra")).toBe(false); // лишняя удалена
    expect(ops.some((o) => o.startsWith("close:extra"))).toBe(true);
    const p1 = existing.get("p1")!;
    expect(p1.size.width).toBe(1.5);
    expect(p1.position.y).toBeCloseTo(1.7, 5);
  });
});

describe("panel_layout_store — debounced saver", () => {
  beforeEach(() => {
    vi.useFakeTimers();
  });
  afterEach(() => {
    vi.useRealTimers();
  });

  it("does not write on every schedule() within debounce window", () => {
    const store = new MemoryStorage();
    const saver = createLayoutSaver(store, 500);
    let count = 0;
    const layout: PersistedLayout = {
      version: 1,
      panels: [
        { id: "p1", topic: "camera_rear", angleDeg: 0, heightM: 1.6, widthM: 1.2, heightPanelM: 0.7 }
      ]
    };
    const serialize = (): PersistedLayout => {
      count += 1;
      return layout;
    };
    saver.schedule(serialize);
    saver.schedule(serialize);
    saver.schedule(serialize);
    // До срабатывания debounce ничего не записано, serialize ещё не вызывался.
    expect(count).toBe(0);
    expect(store.peek(PANEL_LAYOUT_STORAGE_KEY)).toBeNull();
    vi.advanceTimersByTime(499);
    expect(store.peek(PANEL_LAYOUT_STORAGE_KEY)).toBeNull();
    vi.advanceTimersByTime(2);
    expect(count).toBe(1);
    expect(store.peek(PANEL_LAYOUT_STORAGE_KEY)).not.toBeNull();
    const parsed = JSON.parse(store.peek(PANEL_LAYOUT_STORAGE_KEY)!);
    expect(parsed.version).toBe(1);
    expect(parsed.panels[0].id).toBe("p1");
  });

  it("flush() writes immediately", () => {
    const store = new MemoryStorage();
    const saver = createLayoutSaver(store, 1000);
    saver.schedule(() => ({
      version: 1,
      panels: [
        { id: "p1", topic: "camera_rear", angleDeg: 0, heightM: 1.6, widthM: 1.2, heightPanelM: 0.7 }
      ]
    }));
    saver.flush(() => ({
      version: 1,
      panels: [
        { id: "p1", topic: "camera_rear", angleDeg: 10, heightM: 1.5, widthM: 1.0, heightPanelM: 0.6 }
      ]
    }));
    expect(store.peek(PANEL_LAYOUT_STORAGE_KEY)).not.toBeNull();
    const parsed = JSON.parse(store.peek(PANEL_LAYOUT_STORAGE_KEY)!);
    expect(parsed.panels[0].angleDeg).toBe(10);
  });

  it("cancel() drops pending write", () => {
    const store = new MemoryStorage();
    const saver = createLayoutSaver(store, 500);
    saver.schedule(() => ({
      version: 1,
      panels: [
        { id: "p1", topic: "camera_rear", angleDeg: 0, heightM: 1.6, widthM: 1.2, heightPanelM: 0.7 }
      ]
    }));
    saver.cancel();
    vi.advanceTimersByTime(1000);
    expect(store.peek(PANEL_LAYOUT_STORAGE_KEY)).toBeNull();
  });
});
