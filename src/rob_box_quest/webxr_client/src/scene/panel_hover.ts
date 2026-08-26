// Panel hover state manager (Phase 2 §3.7).
//
// Контракт:
//   - Хранит текущий hovered panelId (или null).
//   - При смене hovered → вызывает подписчиков (setActive / setInactive).
//   - Подписчик применяет emissive highlight к panel mesh.
//
// Чистая логика, не зависит от Three.js. Это позволяет тестировать
// переходы hovered panel A → B → null без WebGL.

export interface PanelHoverSubscriber {
  onHoverEnter?(panelId: string): void;
  onHoverExit?(panelId: string): void;
}

export interface PanelHoverOptions {
  /** Подписчик, применяющий визуальный highlight. */
  subscriber?: PanelHoverSubscriber;
}

export interface PanelHoverHandle {
  /** Установить текущий hovered panel (или null). Возвращает список changed panelId. */
  setHovered(panelId: string | null): readonly string[];
  /** Текущий hovered panelId (или null). */
  getHovered(): string | null;
  /** Подписаться на hover transitions. Возвращает unsubscriber. */
  subscribe(sub: PanelHoverSubscriber): () => void;
  /** Сбросить (например, при exit VR). */
  reset(): void;
}

/**
 * Простой hover FSM: в один момент времени hover ровно одна панель (или none).
 * transition: prev=null, next=A → enter(A)
 *             prev=A, next=null → exit(A)
 *             prev=A, next=B → exit(A) + enter(B)
 */
export function createPanelHover(opts: PanelHoverOptions = {}): PanelHoverHandle {
  let current: string | null = null;
  const subs = new Set<PanelHoverSubscriber>();
  if (opts.subscriber) subs.add(opts.subscriber);

  function emit(kind: "enter" | "exit", id: string): void {
    for (const s of subs) {
      try {
        if (kind === "enter") s.onHoverEnter?.(id);
        else s.onHoverExit?.(id);
      } catch {
        // swallow — как в mode_manager
      }
    }
  }

  function setHovered(panelId: string | null): readonly string[] {
    const changed: string[] = [];
    if (panelId === current) return changed;
    if (current !== null) {
      emit("exit", current);
      changed.push(current);
    }
    current = panelId;
    if (current !== null) {
      emit("enter", current);
      changed.push(current);
    }
    return changed;
  }

  return {
    setHovered,
    getHovered(): string | null {
      return current;
    },
    subscribe(sub): () => void {
      subs.add(sub);
      return () => {
        subs.delete(sub);
      };
    },
    reset(): void {
      if (current !== null) {
        emit("exit", current);
        current = null;
      }
    }
  };
}

/**
 * Применить hover-highlight к THREE.Mesh через material.emissive.
 *
 * Видео-панель использует MeshBasicMaterial (без освещения). Чтобы получить
 * видимый "highlight" без освещения, добавляем тонкий outline через копию
 * material с заданным цветом + blending. Для упрощения Phase 2 просто
 * меняем .color — белая/серая подсветка при hover.
 */
import type * as THREE from "three";

export interface PanelHoverVisual {
  setHovered(mesh: THREE.Mesh, hover: boolean): void;
}

const HOVER_COLOR = 0x4d9aff;
const BASE_COLOR = 0xffffff;

/**
 * Default visual: меняет material.color mesh'а. Подходит для MeshBasicMaterial
 * (video panels). Для PBR — нужно emissive + unlit backup.
 */
export const defaultPanelHoverVisual: PanelHoverVisual = {
  setHovered(mesh, hover): void {
    const mat = mesh.material as THREE.Material | THREE.Material[];
    if (Array.isArray(mat)) {
      for (const m of mat) {
        const mb = m as THREE.MeshBasicMaterial;
        if (mb && "color" in mb) {
          (mb as THREE.MeshBasicMaterial).color.setHex(hover ? HOVER_COLOR : BASE_COLOR);
        }
      }
    } else {
      const mb = mat as THREE.MeshBasicMaterial;
      if (mb && "color" in mb) {
        mb.color.setHex(hover ? HOVER_COLOR : BASE_COLOR);
      }
    }
  }
};