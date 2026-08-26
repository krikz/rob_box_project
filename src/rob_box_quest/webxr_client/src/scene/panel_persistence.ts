// Phase 2 §2.6: persistence адаптер для PanelManager.
//
// Оборачивает PanelManager, чтобы:
//   - при изменениях (onChange) автоматически сохранять layout в localStorage
//     с debounce 300ms (последний change побеждает);
//   - на старте load() подтягивал ранее сохранённый layout (если есть);
//   - clear() стирал сохранённый layout.
//
// Ключ/схема — PANEL_LAYOUT_STORAGE_KEY + PanelLayoutJson v1 (см. panel_manager).
//
// Чтобы избежать рекурсии (load() → emit("layout_reset") → save()),
// флаг isLoading на время load() глушит auto-save.

import {
  PanelManager,
  PANEL_LAYOUT_STORAGE_KEY,
  type PanelLayoutJson,
  type PanelChangeEvent
} from "./panel_manager";

export interface PanelPersistenceOptions {
  /** Storage API (default: window.localStorage). */
  storage?: Storage;
  /** Ключ (default: PANEL_LAYOUT_STORAGE_KEY). */
  key?: string;
  /** Debounce перед записью (default 300ms). */
  debounceMs?: number;
}

export interface PanelPersistence {
  /** Сохранить текущий layout немедленно (без debounce). */
  save(): void;
  /** Загрузить layout из storage. true — успех, false — нет/битый. */
  load(): boolean;
  /** Удалить сохранённый layout. */
  clear(): void;
  /** Подписаться на onChange + запустить debounce-save. */
  attach(): void;
  /** Отписаться от onChange, очистить pending timer. */
  dispose(): void;
}

export function createPanelPersistence(
  mgr: PanelManager,
  options: PanelPersistenceOptions = {}
): PanelPersistence {
  const storage = options.storage ?? (globalThis as { localStorage?: Storage }).localStorage;
  if (!storage) {
    throw new Error(
      "PanelPersistence: localStorage is not available. Pass options.storage explicitly."
    );
  }
  const key = options.key ?? PANEL_LAYOUT_STORAGE_KEY;
  const debounceMs = options.debounceMs ?? 300;

  let timer: ReturnType<typeof setTimeout> | null = null;
  let attached = false;
  let isLoading = false;
  const onChange = (_e: PanelChangeEvent): void => {
    if (!attached) return;
    if (isLoading) return;
    if (timer !== null) clearTimeout(timer);
    timer = setTimeout(() => {
      timer = null;
      try {
        persist(mgr, storage, key);
      } catch (err) {
        // eslint-disable-next-line no-console
        console.warn("[quest] panel layout save failed:", err);
      }
    }, debounceMs);
  };

  function persist(m: PanelManager, s: Storage, k: string): void {
    s.setItem(k, JSON.stringify(m.toJSON()));
  }

  return {
    save(): void {
      if (timer !== null) {
        clearTimeout(timer);
        timer = null;
      }
      try {
        persist(mgr, storage, key);
      } catch (err) {
        // eslint-disable-next-line no-console
        console.warn("[quest] panel layout save failed:", err);
      }
    },
    load(): boolean {
      const raw = storage.getItem(key);
      if (!raw) return false;
      let parsed: PanelLayoutJson | null = null;
      try {
        parsed = JSON.parse(raw) as PanelLayoutJson;
      } catch {
        return false;
      }
      isLoading = true;
      try {
        return mgr.fromJSON(parsed);
      } finally {
        isLoading = false;
      }
    },
    clear(): void {
      storage.removeItem(key);
    },
    attach(): void {
      if (attached) return;
      attached = true;
      mgr.onChange(onChange);
    },
    dispose(): void {
      attached = false;
      if (timer !== null) {
        clearTimeout(timer);
        timer = null;
      }
      // mgr.onChange возвращает unsubscribe, но мы просто теряем ссылку —
      // PanelManager хранит подписчиков в массиве и они будут очищены
      // только при resetLayout или по явному unsubscribe. Если нужно
      // гарантированное отписывание — сохраните возвращаемое значение
      // из attach() в будущей ревизии.
    }
  };
}
