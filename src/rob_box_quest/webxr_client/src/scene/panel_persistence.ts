// src/scene/panel_persistence.ts
//
// Сохранение/восстановление layout panels в localStorage. Это pure-JS
// модуль без Three.js — отдельно от PanelManager, чтобы тестироваться
// без webxr/three-фикстур.
//
// Layout — список PanelState[], сериализуется в JSON:
//   { version: 1, panels: PanelState[] }
//
// version нужен для будущих миграций (поле size/facing изменится →
// поднимем version и обработаем в migrate()).
//
// Безопасность: panel.id — внутренний (`p1`, `p2`...). При десериализации
// они переиспользуются как есть; PanelManager их не валидирует, но это
// безопасно, потому что id генерируются createPanel() и не интерпретируются
// за пределами клиента.

import type { PanelState } from "./panel_manager";

export const PANEL_LAYOUT_STORAGE_KEY = "robbox.quest.panelLayout.v1";
const LAYOUT_VERSION = 1;

export interface PersistedLayout {
  version: number;
  panels: PanelState[];
}

export interface StorageLike {
  getItem(key: string): string | null;
  setItem(key: string, value: string): void;
  removeItem(key: string): void;
}

/** Default-реализация StorageLike поверх window.localStorage. */
export function defaultStorage(): StorageLike | null {
  if (typeof window === "undefined" || !window.localStorage) return null;
  return window.localStorage;
}

/**
 * Сериализует список panels в JSON-строку для localStorage.
 * Бросает Error если panels пустой (анти-паттерн: сохранять «ничего» —
 * caller должен clearLayout() вместо saveLayout([])).
 */
export function serializeLayout(panels: PanelState[]): string {
  if (!Array.isArray(panels)) {
    throw new Error("serializeLayout: panels must be an array");
  }
  const payload: PersistedLayout = { version: LAYOUT_VERSION, panels };
  return JSON.stringify(payload);
}

/**
 * Парсит JSON из localStorage. Возвращает {ok, layout} или {ok: false, error}.
 * Не бросает — caller решает, использовать ли дефолт.
 */
export function parseLayout(raw: string | null):
  | { ok: true; layout: PersistedLayout }
  | { ok: false; error: string } {
  if (raw === null || raw === "") return { ok: false, error: "empty" };
  let parsed: unknown;
  try {
    parsed = JSON.parse(raw);
  } catch (e) {
    return { ok: false, error: `json parse: ${(e as Error).message}` };
  }
  if (!parsed || typeof parsed !== "object") {
    return { ok: false, error: "not an object" };
  }
  const obj = parsed as Record<string, unknown>;
  if (obj.version !== LAYOUT_VERSION) {
    return { ok: false, error: `unsupported version ${String(obj.version)}` };
  }
  if (!Array.isArray(obj.panels)) {
    return { ok: false, error: "panels is not an array" };
  }
  const panels: PanelState[] = [];
  for (let i = 0; i < obj.panels.length; i += 1) {
    const p = obj.panels[i] as Partial<PanelState>;
    if (
      !p ||
      typeof p.id !== "string" ||
      typeof p.topic !== "string" ||
      !p.position ||
      typeof p.position.x !== "number" ||
      typeof p.position.y !== "number" ||
      typeof p.position.z !== "number" ||
      !p.facing ||
      typeof p.facing.x !== "number" ||
      typeof p.facing.z !== "number" ||
      !p.size ||
      typeof p.size.width !== "number" ||
      typeof p.size.height !== "number" ||
      typeof p.selected !== "boolean"
    ) {
      return { ok: false, error: `panel[${i}] schema mismatch` };
    }
    panels.push({
      id: p.id,
      topic: p.topic,
      position: { x: p.position.x, y: p.position.y, z: p.position.z },
      facing: { x: p.facing.x, z: p.facing.z },
      size: { width: p.size.width, height: p.size.height },
      selected: p.selected
    });
  }
  return { ok: true, layout: { version: LAYOUT_VERSION, panels } };
}

/**
 * Сохранить layout. storage === null (например, SSR) — no-op.
 * При ошибке сериализации — пробрасывает (caller должен валидировать вход).
 */
export function saveLayout(
  storage: StorageLike | null,
  panels: PanelState[]
): boolean {
  if (!storage) return false;
  try {
    storage.setItem(PANEL_LAYOUT_STORAGE_KEY, serializeLayout(panels));
    return true;
  } catch {
    return false;
  }
}

/**
 * Загрузить layout. Возвращает null если нет ключа / corrupt / неподдерживаемая версия.
 */
export function loadLayout(storage: StorageLike | null): PanelState[] | null {
  if (!storage) return null;
  const raw = storage.getItem(PANEL_LAYOUT_STORAGE_KEY);
  const r = parseLayout(raw);
  return r.ok ? r.layout.panels : null;
}

/** Удалить сохранённый layout (Reset to Default). */
export function clearLayout(storage: StorageLike | null): void {
  if (!storage) return;
  storage.removeItem(PANEL_LAYOUT_STORAGE_KEY);
}