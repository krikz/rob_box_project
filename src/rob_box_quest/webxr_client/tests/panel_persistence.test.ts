// tests/panel_persistence.test.ts
import { describe, it, expect } from "vitest";
import {
  serializeLayout,
  parseLayout,
  saveLayout,
  loadLayout,
  clearLayout,
  PANEL_LAYOUT_STORAGE_KEY,
  type StorageLike
} from "../src/scene/panel_persistence";
import type { PanelState } from "../src/scene/panel_manager";

function makeStorage(): { store: Map<string, string>; api: StorageLike } {
  const store = new Map<string, string>();
  return {
    store,
    api: {
      getItem: (k) => store.get(k) ?? null,
      setItem: (k, v) => store.set(k, v),
      removeItem: (k) => store.delete(k)
    }
  };
}

const sample: PanelState[] = [
  {
    id: "p1",
    topic: "camera_rear",
    position: { x: -1.0, y: 1.6, z: -1.732 },
    facing: { x: 0.5, z: 0.866 },
    size: { width: 1.2, height: 0.7 },
    selected: false
  },
  {
    id: "p2",
    topic: "lidar_2d",
    position: { x: 1.0, y: 1.6, z: -1.732 },
    facing: { x: -0.5, z: 0.866 },
    size: { width: 1.5, height: 0.9 },
    selected: true
  }
];

describe("serializeLayout", () => {
  it("round-trips through parseLayout", () => {
    const json = serializeLayout(sample);
    const r = parseLayout(json);
    expect(r.ok).toBe(true);
    if (r.ok) expect(r.layout.panels).toEqual(sample);
  });

  it("embeds version=1", () => {
    const json = serializeLayout(sample);
    expect(JSON.parse(json).version).toBe(1);
  });

  it("throws on non-array input", () => {
    expect(() => serializeLayout(null as unknown as PanelState[])).toThrow();
  });
});

describe("parseLayout", () => {
  it("returns ok:false on empty", () => {
    const r = parseLayout(null);
    expect(r).toEqual({ ok: false, error: "empty" });
  });

  it("returns ok:false on malformed JSON", () => {
    const r = parseLayout("{not json");
    expect(r.ok).toBe(false);
    if (!r.ok) expect(r.error.startsWith("json parse:")).toBe(true);
  });

  it("returns ok:false on wrong version", () => {
    const r = parseLayout(JSON.stringify({ version: 2, panels: [] }));
    expect(r.ok).toBe(false);
    if (!r.ok) expect(r.error).toContain("unsupported version 2");
  });

  it("returns ok:false when panels is not array", () => {
    const r = parseLayout(JSON.stringify({ version: 1, panels: "oops" }));
    expect(r.ok).toBe(false);
  });

  it("returns ok:false on schema mismatch (missing topic)", () => {
    const bad = { version: 1, panels: [{ id: "p1", position: {}, facing: {}, size: {}, selected: false }] };
    const r = parseLayout(JSON.stringify(bad));
    expect(r.ok).toBe(false);
    if (!r.ok) expect(r.error).toContain("schema mismatch");
  });
});

describe("saveLayout / loadLayout / clearLayout", () => {
  it("round-trips via storage", () => {
    const { store, api } = makeStorage();
    expect(saveLayout(api, sample)).toBe(true);
    expect(store.has(PANEL_LAYOUT_STORAGE_KEY)).toBe(true);
    const loaded = loadLayout(api);
    expect(loaded).toEqual(sample);
  });

  it("loadLayout returns null for missing key", () => {
    const { api } = makeStorage();
    expect(loadLayout(api)).toBeNull();
  });

  it("loadLayout returns null for corrupt JSON", () => {
    const { store, api } = makeStorage();
    store.set(PANEL_LAYOUT_STORAGE_KEY, "{garbage");
    expect(loadLayout(api)).toBeNull();
  });

  it("clearLayout removes the key", () => {
    const { store, api } = makeStorage();
    saveLayout(api, sample);
    clearLayout(api);
    expect(store.has(PANEL_LAYOUT_STORAGE_KEY)).toBe(false);
    expect(loadLayout(api)).toBeNull();
  });

  it("saveLayout with null storage is no-op (returns false)", () => {
    expect(saveLayout(null, sample)).toBe(false);
    expect(loadLayout(null)).toBeNull();
    expect(() => clearLayout(null)).not.toThrow();
  });
});