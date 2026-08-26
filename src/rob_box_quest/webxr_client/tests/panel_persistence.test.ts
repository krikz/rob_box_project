// Phase 2 §2.6: persistence через localStorage.
//
// Тестируем только схему PanelManager.toJSON/fromJSON + вспомогательный
// PanelPersistence адаптер (debounce 300ms, key v1). localStorage мокаем
// через globalThis.localStorage в jsdom (Vitest environment).
//
// Подход: хранилище читается синхронно (localStorage), но debounce делает
// save асинхронным — поэтому для тестов debounce используем vi.useFakeTimers().

import { describe, it, expect, beforeEach, afterEach, vi } from "vitest";
import {
  PanelManager,
  PANEL_LAYOUT_STORAGE_KEY,
  PANEL_LAYOUT_VERSION,
  type PanelLayoutJson
} from "../src/scene/panel_manager";
import {
  PanelPersistence,
  createPanelPersistence
} from "../src/scene/panel_persistence";

function makeStorage(): Storage {
  // Минимальный in-memory mock (Vitest+jsdom даёт global localStorage, но
  // для чистоты делаем явный fresh storage).
  const store = new Map<string, string>();
  return {
    get length() {
      return store.size;
    },
    clear() {
      store.clear();
    },
    getItem(key: string): string | null {
      return store.has(key) ? store.get(key)! : null;
    },
    key(i: number): string | null {
      return Array.from(store.keys())[i] ?? null;
    },
    removeItem(key: string): void {
      store.delete(key);
    },
    setItem(key: string, value: string): void {
      store.set(key, value);
    }
  };
}

describe("PanelPersistence", () => {
  let storage: Storage;
  let mgr: PanelManager;
  let persistence: PanelPersistence;

  beforeEach(() => {
    storage = makeStorage();
    vi.stubGlobal("localStorage", storage);
    mgr = new PanelManager();
    persistence = createPanelPersistence(mgr, { storage, debounceMs: 300 });
  });

  afterEach(() => {
    persistence.dispose();
    vi.unstubAllGlobals();
    vi.useRealTimers();
  });

  it("save() persists current layout to localStorage", () => {
    mgr.resetLayout();
    persistence.save();
    expect(storage.getItem(PANEL_LAYOUT_STORAGE_KEY)).not.toBeNull();
    const raw = storage.getItem(PANEL_LAYOUT_STORAGE_KEY)!;
    const parsed = JSON.parse(raw) as PanelLayoutJson;
    expect(parsed.version).toBe(PANEL_LAYOUT_VERSION);
    expect(parsed.panels.length).toBe(4);
  });

  it("load() restores panels from localStorage", () => {
    mgr.resetLayout();
    const snapshot = mgr.toJSON();
    persistence.save();
    // очищаем текущий mgr
    mgr.close(mgr.list()[0].id);
    mgr.close(mgr.list()[0].id);
    mgr.close(mgr.list()[0].id);
    mgr.close(mgr.list()[0].id);
    expect(mgr.count()).toBe(0);
    expect(persistence.load()).toBe(true);
    expect(mgr.count()).toBe(4);
    expect(JSON.stringify(mgr.toJSON())).toBe(JSON.stringify(snapshot));
  });

  it("load() returns false when no saved layout", () => {
    expect(persistence.load()).toBe(false);
    expect(mgr.count()).toBe(0); // ничего не изменилось
  });

  it("load() returns false on corrupted JSON", () => {
    storage.setItem(PANEL_LAYOUT_STORAGE_KEY, "not json {");
    expect(persistence.load()).toBe(false);
  });

  it("load() returns false on unknown version", () => {
    storage.setItem(
      PANEL_LAYOUT_STORAGE_KEY,
      JSON.stringify({ version: 99, panels: [] })
    );
    expect(persistence.load()).toBe(false);
  });

  it("clear() removes saved layout", () => {
    persistence.save();
    expect(storage.getItem(PANEL_LAYOUT_STORAGE_KEY)).not.toBeNull();
    persistence.clear();
    expect(storage.getItem(PANEL_LAYOUT_STORAGE_KEY)).toBeNull();
  });

  it("attaching to manager auto-saves on changes (debounced)", () => {
    vi.useFakeTimers();
    persistence.attach();
    mgr.resetLayout();
    // До продвижения таймера ничего в storage нет
    expect(storage.getItem(PANEL_LAYOUT_STORAGE_KEY)).toBeNull();
    vi.advanceTimersByTime(300);
    expect(storage.getItem(PANEL_LAYOUT_STORAGE_KEY)).not.toBeNull();
    // Дополнительное изменение → ещё один debounce-цикл
    const raw = storage.getItem(PANEL_LAYOUT_STORAGE_KEY)!;
    const parsed = JSON.parse(raw) as PanelLayoutJson;
    const firstId = parsed.panels[0].id;
    mgr.move(firstId, 1, -1);
    vi.advanceTimersByTime(150); // ещё не сработало
    expect(storage.getItem(PANEL_LAYOUT_STORAGE_KEY)).toBe(raw);
    vi.advanceTimersByTime(150); // +150 = 300 от последнего изменения
    const raw2 = storage.getItem(PANEL_LAYOUT_STORAGE_KEY)!;
    expect(raw2).not.toBe(raw);
  });

  it("attach() не запускает save при layout_reset от fromJSON (нет рекурсии)", () => {
    vi.useFakeTimers();
    persistence.attach();
    // Сначала зальём snapshot
    mgr.resetLayout();
    persistence.save();
    vi.advanceTimersByTime(0);
    const initial = storage.getItem(PANEL_LAYOUT_STORAGE_KEY)!;
    // load() вызывает layout_reset (emit). Persistence НЕ должен триггерить
    // новый save, иначе закольцуемся.
    persistence.load();
    vi.advanceTimersByTime(1000);
    expect(storage.getItem(PANEL_LAYOUT_STORAGE_KEY)).toBe(initial);
  });

  it("dispose() отписывается от изменений", () => {
    vi.useFakeTimers();
    persistence.attach();
    persistence.dispose();
    mgr.resetLayout();
    vi.advanceTimersByTime(1000);
    expect(storage.getItem(PANEL_LAYOUT_STORAGE_KEY)).toBeNull();
  });
});
