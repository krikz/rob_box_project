// Хранилище раскладки панелей Captain Bridge в localStorage.
//
// Дизайн §3 (docs/plans/2026-08-25-webxr-captain-bridge-design.md):
// раскладка должна переживать перезапуск клиента. Ключ —
// `rob_box_quest.panel_layout.v1` (имя зафиксировано в аудите 30.08 B1).
//
// Схема (v1):
//   {
//     version: 1,
//     panels: [
//       {
//         id: string,            // стабильный id панели в PanelManager
//         topic: string,         // топик стрима, должен быть в реестре
//         angleDeg: number,      // азимут от центра (по дизайну, полукруг)
//         heightM: number,       // высота центра панели по Y
//         widthM: number,        // ширина панели в метрах
//         heightPanelM: number,  // высота панели в метрах (отдельная от heightM)
//       },
//       ...
//     ],
//   }
//
// Все функции чистые (без Three.js, без DOM), localStorage инжектится.
// Тесты в tests/panel_layout_store.test.ts мокают store и проверяют
// round-trip + деградацию на битом JSON / чужой version / неизвестном
// топике / частично битой панели. «Молча падать или молча брать половину
// сохранённого — нельзя» (acceptance): при любой ошибке возвращаем
// `null` и пишем console.warn.

import type { PanelId, PanelState } from "./panel_manager";

export const PANEL_LAYOUT_STORAGE_KEY = "rob_box_quest.panel_layout.v1";

/** Минимально-разумные границы размера, чтобы resize не схлопнул панель. */
export const PANEL_MIN_WIDTH_M = 0.4;
export const PANEL_MAX_WIDTH_M = 3.0;
export const PANEL_MIN_HEIGHT_M = 0.3;
export const PANEL_MAX_HEIGHT_M = 2.0;

export interface PersistedPanel {
  id: PanelId;
  topic: string;
  angleDeg: number;
  heightM: number;
  widthM: number;
  heightPanelM: number;
}

export interface PersistedLayout {
  version: 1;
  panels: PersistedPanel[];
}

/** Минимальный store-интерфейс: localStorage в браузере, mock в тестах. */
export interface LayoutStorage {
  getItem(key: string): string | null;
  setItem(key: string, value: string): void;
  removeItem(key: string): void;
}

/**
 * Из PanelManager в плоскую сериализуемую форму. Стабильный порядок по
 * id — иначе round-trip будет менять JSON от запуска к запуску, а это
 * лишний шум в git/дифф при проверке persistence.
 */
export function serializeLayout(panels: ReadonlyArray<PanelState>): PersistedLayout {
  const sorted = [...panels].sort((a, b) => (a.id < b.id ? -1 : a.id > b.id ? 1 : 0));
  return {
    version: 1,
    panels: sorted.map((p) => ({
      id: p.id,
      topic: p.topic,
      angleDeg: azimuthDegFromPosition(p.position),
      heightM: p.position.y,
      widthM: p.size.width,
      heightPanelM: p.size.height
    }))
  };
}

/**
 * Спарсить строку из localStorage. `null` означает «сохранённое
 * игнорируем, берите дефолт». При любой ошибке пишем console.warn —
 * не throw: layout-store не должен ломать старт клиента.
 *
 * Параметры:
 *   `raw`            — строка из localStorage (или mock-store).
 *   `knownTopicSet`  — реестр топиков из stream_list: если панель ссылается
 *                      на топик не из реестра — она целиком отбрасывается,
 *                      и это не должно валить всю раскладку.
 */
export function parseLayout(
  raw: string | null,
  knownTopicSet: ReadonlySet<string>
): PersistedLayout | null {
  if (raw === null) return null;
  let data: unknown;
  try {
    data = JSON.parse(raw);
  } catch (err) {
    // eslint-disable-next-line no-console
    console.warn("[panel_layout_store] parseLayout: invalid JSON, using default", err);
    return null;
  }
  if (!isPlainObject(data)) {
    // eslint-disable-next-line no-console
    console.warn("[panel_layout_store] parseLayout: not an object, using default");
    return null;
  }
  if (data.version !== 1) {
    // eslint-disable-next-line no-console
    console.warn(
      `[panel_layout_store] parseLayout: unknown version ${String(data.version)}, using default`
    );
    return null;
  }
  if (!Array.isArray(data.panels)) {
    // eslint-disable-next-line no-console
    console.warn("[panel_layout_store] parseLayout: panels is not array, using default");
    return null;
  }

  const accepted: PersistedPanel[] = [];
  for (const entry of data.panels) {
    const parsed = parsePanelEntry(entry);
    if (parsed === null) continue;
    if (!knownTopicSet.has(parsed.topic)) {
      // eslint-disable-next-line no-console
      console.warn(
        `[panel_layout_store] parseLayout: dropping panel ${parsed.id} (unknown topic "${parsed.topic}")`
      );
      continue;
    }
    accepted.push(parsed);
  }
  if (accepted.length === 0) {
    // eslint-disable-next-line no-console
    console.warn("[panel_layout_store] parseLayout: no usable panels, using default");
    return null;
  }
  return { version: 1, panels: accepted };
}

/**
 * Применить PersistedLayout поверх текущего состояния PanelManager.
 * Лишние панели удаляются, отсутствующие создаются, существующие
 * переезжают и меняют размер. Возвращает число изменённых панелей —
 * нужно для дебаунса сохранения (нечего сохранять — не сохраняем).
 *
 * `createPanel` принимает `id` первым параметром: реализация может
 * проигнорировать его (PanelManager.createPanel) и вернуть свой id, а
 * может создать с этим id (PanelManager.createPanelWithId). В первом
 * случае панель попадёт в Map под автогенерированным id, и мы её не
 * найдём по saved id — это нормально для тестов, но не для
 * production (см. `applyStoredLayout` в captain_bridge.ts: он
 * предварительно создаёт панели через createPanelWithId, чтобы
 * applyLayout нашёл их по id).
 */
export function applyLayout(
  layout: PersistedLayout,
  state: {
    list(): PanelState[];
    close(id: PanelId): boolean;
    move(id: PanelId, x: number, z: number, y?: number): boolean;
    createPanel(
      id: PanelId,
      topic: string,
      position?: { x: number; y: number; z: number },
      facing?: { x: number; z: number }
    ): PanelId;
    resize(id: PanelId, width: number, height: number): boolean;
  }
): number {
  let changed = 0;
  const desiredIds = new Set<PanelId>();
  for (const p of layout.panels) {
    desiredIds.add(p.id);
  }
  // Удалить лишние.
  for (const existing of state.list()) {
    if (!desiredIds.has(existing.id)) {
      state.close(existing.id);
      changed += 1;
    }
  }
  // Создать / обновить.
  for (const p of layout.panels) {
    const existing = state.list().find((s) => s.id === p.id);
    if (!existing) {
      const pos = positionFromAngleAndHeight(p.angleDeg, p.heightM, 2.0);
      const id = state.createPanel(
        p.id,
        p.topic,
        pos,
        {
          x: -Math.sin((p.angleDeg * Math.PI) / 180),
          z: Math.cos((p.angleDeg * Math.PI) / 180)
        }
      );
      state.resize(id, p.widthM, p.heightPanelM);
      changed += 1;
      continue;
    }
    const pos = positionFromAngleAndHeight(p.angleDeg, p.heightM, 2.0);
    const moved = state.move(p.id, pos.x, pos.z, pos.y);
    if (moved) changed += 1;
    const resized = state.resize(p.id, p.widthM, p.heightPanelM);
    if (resized) changed += 1;
  }
  return changed;
}

/**
 * Дебаунс-обёртка над записью. Сохраняет в store не чаще раза в
 * `debounceMs` миллисекунд. Последний вызов всегда сохраняется — иначе
 * пользователь уронит layout на полпути, и при перезапуске откатится
 * назад. `serialize()` инжектируется, чтобы не тянуть PanelManager в
 * чистый модуль.
 */
export function createLayoutSaver(
  storage: LayoutStorage,
  debounceMs: number = 500
): {
  schedule(serialize: () => PersistedLayout): void;
  flush(serialize: () => PersistedLayout): void;
  cancel(): void;
} {
  let timer: ReturnType<typeof setTimeout> | null = null;
  let pendingSerialize: (() => PersistedLayout) | null = null;
  return {
    schedule(serialize: () => PersistedLayout): void {
      pendingSerialize = serialize;
      if (timer !== null) clearTimeout(timer);
      timer = setTimeout(() => {
        timer = null;
        const fn = pendingSerialize;
        pendingSerialize = null;
        if (fn) writeLayout(storage, fn());
      }, debounceMs);
    },
    flush(serialize: () => PersistedLayout): void {
      if (timer !== null) {
        clearTimeout(timer);
        timer = null;
      }
      pendingSerialize = null;
      writeLayout(storage, serialize());
    },
    cancel(): void {
      if (timer !== null) {
        clearTimeout(timer);
        timer = null;
      }
      pendingSerialize = null;
    }
  };
}

function writeLayout(storage: LayoutStorage, layout: PersistedLayout): void {
  try {
    storage.setItem(PANEL_LAYOUT_STORAGE_KEY, JSON.stringify(layout));
  } catch (err) {
    // eslint-disable-next-line no-console
    console.warn("[panel_layout_store] save failed", err);
  }
}

// --- internals --------------------------------------------------------------

function isPlainObject(v: unknown): v is Record<string, unknown> {
  return typeof v === "object" && v !== null && !Array.isArray(v);
}

function parsePanelEntry(entry: unknown): PersistedPanel | null {
  if (!isPlainObject(entry)) return null;
  const id = entry.id;
  const topic = entry.topic;
  const angleDeg = entry.angleDeg;
  const heightM = entry.heightM;
  const widthM = entry.widthM;
  const heightPanelM = entry.heightPanelM;
  if (typeof id !== "string" || id.length === 0) return null;
  if (typeof topic !== "string" || topic.length === 0) return null;
  if (typeof angleDeg !== "number" || !Number.isFinite(angleDeg)) return null;
  if (typeof heightM !== "number" || !Number.isFinite(heightM)) return null;
  if (typeof widthM !== "number" || !Number.isFinite(widthM)) return null;
  if (typeof heightPanelM !== "number" || !Number.isFinite(heightPanelM)) return null;
  // Размеры должны быть в разумных границах: иначе схлопнутая или
  // гигантская панель не отвечает ничьему здравому смыслу. Молча
  // клампим (не отбрасываем): битый JSON уважительнее, чем «всё
  // пропало». Это всё ещё в рамках «целиком default» — мы не берём
  // половину, мы аккуратно чиним одну панель.
  return {
    id,
    topic,
    angleDeg,
    heightM,
    widthM: clampSize(widthM, PANEL_MIN_WIDTH_M, PANEL_MAX_WIDTH_M),
    heightPanelM: clampSize(heightPanelM, PANEL_MIN_HEIGHT_M, PANEL_MAX_HEIGHT_M)
  };
}

function clampSize(v: number, lo: number, hi: number): number {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/** Азимут в градусах от позиции на полу. 0° = прямо перед оператором (-Z). */
function azimuthDegFromPosition(p: { x: number; z: number }): number {
  // atan2(x, -z): angleDeg > 0 → справа, < 0 → слева; 0 → центр.
  return (Math.atan2(p.x, -p.z) * 180) / Math.PI;
}

function positionFromAngleAndHeight(
  angleDeg: number,
  heightM: number,
  radius: number
): { x: number; y: number; z: number } {
  const a = (angleDeg * Math.PI) / 180;
  return {
    x: radius * Math.sin(a),
    y: heightM,
    z: -radius * Math.cos(a)
  };
}
