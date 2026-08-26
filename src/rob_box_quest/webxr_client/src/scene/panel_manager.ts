// Менеджер floating panels — модель состояний, тестируемая без Three.js.
//
// Концепция "капитанский мостик" (дизайн §1): пользователь стоит в центре,
// вокруг — N floating panels с видео-потоками. Каждой panel можно:
//   - переместить (desktop drag, XR — point+trigger)
//   - сменить stream (например, с camera_rear на camera_oak_color)
//   - закрыть
//   - resize / сменить opacity
// Layout reset возвращает дефолтную раскладку (полукруг, radius 2м).
//
// Phase 2 §2 добавляет:
//   - snap-to-zone (3 X-anchors: left=-1.5, center=0, right=+1.5; z=-radius)
//   - resize с min/max и aspect-ratio lock (Shift — free resize)
//   - opacity cycle 50/75/100%
//   - persistence в localStorage через JSON v1
//   - XR drag API (startDragFromXr / updateDragFromXr / endDragFromXr)

export type PanelId = string;

export type OpacityLevel = 0.5 | 0.75 | 1.0;
export const OPACITY_LEVELS: OpacityLevel[] = [0.5, 0.75, 1.0];

export type SnapZone = "left" | "center" | "right" | "free";

export interface PanelState {
  id: PanelId;
  topic: string;
  /** позиция в плоскости пола (XZ), y=panelHeight (глаза пользователя) */
  position: { x: number; z: number; y: number };
  /** нормаль (panel смотрит на пользователя) — единичный вектор */
  facing: { x: number; z: number };
  /** ширина/высота панели в метрах */
  size: { width: number; height: number };
  /** текущая прозрачность материала (0.5 / 0.75 / 1.0) */
  opacity: OpacityLevel;
  /** последняя зона, к которой привязана панель после snap */
  snapZone: SnapZone;
  selected: boolean;
}

export interface PanelManagerOptions {
  radius?: number;       // радиус полукруга, дефолт 2.0 м
  panelWidth?: number;   // 1.2 м (default)
  panelHeight?: number;  // 0.7 м (default; ~1.71:1 aspect ratio)
  panelYOffset?: number; // 0.0 (высота глаз ±0.2 — см. дизайн)
  defaultTopics?: string[]; // порядок по умолчанию
  minWidth?: number;     // resize lower bound, default 0.4
  minHeight?: number;    // resize lower bound, default 0.3
  maxWidth?: number;     // resize upper bound, default 3.0
  maxHeight?: number;    // resize upper bound, default 2.0
  /** X-threshold для snap-зон (default 0.7 — см. дизайн §2.2). */
  snapThreshold?: number;
}

export const DEFAULT_VIDEO_TOPICS = [
  "camera_rear",
  "camera_oak_color",
  "camera_oak_depth",
  "camera_ceiling"
] as const;

/** Snap-zone anchors (X, фиксированный Z=-radius). */
export interface SnapAnchor {
  zone: SnapZone;
  x: number;
  z: number;
}

export function getSnapAnchors(radius: number): SnapAnchor[] {
  return [
    { zone: "left", x: -1.5, z: -radius },
    { zone: "center", x: 0, z: -radius },
    { zone: "right", x: 1.5, z: -radius }
  ];
}

/** Схема persistence в localStorage (version 1). */
export const PANEL_LAYOUT_STORAGE_KEY = "rob_box_quest.panel_layout.v1";
export const PANEL_LAYOUT_VERSION = 1;

export interface PanelLayoutJson {
  version: number;
  panels: PanelState[];
}

export interface PanelChangeEvent {
  type: "moved" | "resized" | "opacity" | "stream" | "closed" | "created" | "layout_reset";
  id?: PanelId;
}

/** Аргумент для resize (aspect-ratio lock — по умолчанию). */
export interface ResizeOptions {
  /** true → свободное изменение без сохранения aspect ratio. */
  freeAspect?: boolean;
  /** целевой aspect ratio, если нужно (default — текущий). */
  aspect?: number;
}

export class PanelManager {
  private panels = new Map<PanelId, PanelState>();
  private nextId = 1;
  private readonly opts: Required<PanelManagerOptions>;
  private listeners: Array<(e: PanelChangeEvent) => void> = [];
  /** Текущая XR drag-сессия (panelId, lastControllerPosition). */
  private xrDrag: {
    panelId: PanelId;
    position: { x: number; z: number };
    /** сдвиг между контроллером и центром panel в начале drag. */
    offset: { x: number; z: number };
  } | null = null;

  constructor(opts: PanelManagerOptions = {}) {
    this.opts = {
      radius: opts.radius ?? 2.0,
      panelWidth: opts.panelWidth ?? 1.2,
      panelHeight: opts.panelHeight ?? 0.7,
      panelYOffset: opts.panelYOffset ?? 0.0,
      defaultTopics: opts.defaultTopics ?? [...DEFAULT_VIDEO_TOPICS],
      minWidth: opts.minWidth ?? 0.4,
      minHeight: opts.minHeight ?? 0.3,
      maxWidth: opts.maxWidth ?? 3.0,
      maxHeight: opts.maxHeight ?? 2.0,
      snapThreshold: opts.snapThreshold ?? 0.7
    };
  }

  // ---------- listeners ----------

  /** Подписка на изменения panels (для persistence / debug). */
  onChange(fn: (e: PanelChangeEvent) => void): () => void {
    this.listeners.push(fn);
    return () => {
      const i = this.listeners.indexOf(fn);
      if (i >= 0) this.listeners.splice(i, 1);
    };
  }

  private emit(e: PanelChangeEvent): void {
    for (const fn of this.listeners) fn(e);
  }

  // ---------- layout core ----------

  // Создаёт дефолтную раскладку: 4 panel полукругом перед пользователем.
  // Первый panel — по центру впереди, остальные — с отклонением ±60°, ±20°.
  resetLayout(): PanelId[] {
    this.panels.clear();
    const topics = this.opts.defaultTopics;
    const radius = this.opts.radius;
    const ids: PanelId[] = [];
    // Углы: -60, -20, +20, +60 от направления "вперёд" (ось -Z в three.js,
    // но для layout-store нам нужен угол вокруг Y). Используем соглашение:
    // angle=0 → прямо перед пользователем (x=0, z=-radius).
    const anglesDeg = [-60, -20, 20, 60];
    for (let i = 0; i < topics.length && i < anglesDeg.length; i += 1) {
      const a = (anglesDeg[i] * Math.PI) / 180;
      const pos = {
        x: radius * Math.sin(a),
        y: this.opts.panelYOffset,
        z: -radius * Math.cos(a)
      };
      const facing = { x: -Math.sin(a), z: Math.cos(a) };
      const id = this.createPanel(topics[i], pos, facing);
      ids.push(id);
    }
    this.emit({ type: "layout_reset" });
    return ids;
  }

  createPanel(
    topic: string,
    position?: { x: number; y: number; z: number },
    facing?: { x: number; z: number }
  ): PanelId {
    const id = `p${this.nextId}`;
    this.nextId += 1;
    const pos = position ?? this.defaultPosition();
    const face = facing ?? this.facingTowards(pos);
    this.panels.set(id, {
      id,
      topic,
      position: { ...pos },
      facing: { ...face },
      size: { width: this.opts.panelWidth, height: this.opts.panelHeight },
      opacity: 1.0,
      snapZone: "free",
      selected: false
    });
    this.emit({ type: "created", id });
    return id;
  }

  close(id: PanelId): boolean {
    const ok = this.panels.delete(id);
    if (ok) this.emit({ type: "closed", id });
    return ok;
  }

  switchStream(id: PanelId, newTopic: string): boolean {
    const p = this.panels.get(id);
    if (!p) return false;
    p.topic = newTopic;
    this.emit({ type: "stream", id });
    return true;
  }

  // ---------- move + snap-to-zone ----------

  /**
   * Переместить panel в (x, z). Автоматически:
   *  - пересчитывает facing на пользователя (нормализует)
   *  - определяет snapZone (left|center|right|free) на основе X-threshold
   *
   * Если x/z близко (в пределах snap-зоны) к одному из 3 anchor-ов — ставим
   * позицию ровно на anchor, иначе оставляем свободную.
   * Вызывающий код может затем вызвать `finalizeSnap()` для окончательного
   * snap с проверкой дистанции 0.5м от anchor (как в дизайне §2.2).
   */
  move(id: PanelId, x: number, z: number): boolean {
    const p = this.panels.get(id);
    if (!p) return false;
    p.position.x = x;
    p.position.z = z;
    p.facing.x = -x;
    p.facing.z = -z;
    const len = Math.hypot(p.facing.x, p.facing.z);
    if (len > 1e-6) {
      p.facing.x /= len;
      p.facing.z /= len;
    }
    p.snapZone = classifyByX(x, this.opts.snapThreshold);
    this.emit({ type: "moved", id });
    return true;
  }

  /**
   * Финальный snap после drag-end. Если panel ближе 0.5м к одному из 3 anchor'ов —
   * притягиваем к нему. Иначе остаёмся в свободной зоне (текущая позиция).
   * Возвращает финальную snapZone.
   */
  finalizeSnap(id: PanelId, thresholdMeters = 0.5): SnapZone | null {
    const p = this.panels.get(id);
    if (!p) return null;
    const anchors = getSnapAnchors(this.opts.radius);
    let best: SnapAnchor | null = null;
    let bestDist = Infinity;
    for (const a of anchors) {
      const d = Math.hypot(p.position.x - a.x, p.position.z - a.z);
      if (d < bestDist) {
        bestDist = d;
        best = a;
      }
    }
    if (best && bestDist <= thresholdMeters) {
      p.position.x = best.x;
      p.position.z = best.z;
      // facing остаётся направленным к пользователю (нормализован выше).
      p.facing.x = -best.x;
      p.facing.z = -best.z;
      const len = Math.hypot(p.facing.x, p.facing.z);
      if (len > 1e-6) {
        p.facing.x /= len;
        p.facing.z /= len;
      }
      p.snapZone = best.zone;
    } else {
      p.snapZone = "free";
    }
    this.emit({ type: "moved", id });
    return p.snapZone;
  }

  // ---------- resize (§2.3) ----------

  /**
   * Изменить размер panel. По умолчанию — сохраняем aspect ratio.
   * Если передан opts.freeAspect=true (Shift+drag) — width/height свободны.
   * Применяются min/max из opts.
   */
  resize(
    id: PanelId,
    width: number,
    height: number,
    opts: ResizeOptions = {}
  ): boolean {
    const p = this.panels.get(id);
    if (!p) return false;
    let w = clamp(width, this.opts.minWidth, this.opts.maxWidth);
    let h = clamp(height, this.opts.minHeight, this.opts.maxHeight);

    if (!opts.freeAspect) {
      // Сохраняем aspect ratio. Приоритет: ширина (как в типичном resize за
      // правый-нижний угол), высоту подгоняем.
      const aspect = opts.aspect ?? p.size.width / p.size.height;
      // Если меняли ширину — высоту берём из aspect, затем clamp.
      // Если меняли высоту (а ширину оставили) — берём из height.
      // Сравниваем переданные vs текущие: кто изменился больше.
      const widthChanged = Math.abs(width - p.size.width);
      const heightChanged = Math.abs(height - p.size.height);
      if (widthChanged >= heightChanged) {
        h = clamp(w / aspect, this.opts.minHeight, this.opts.maxHeight);
      } else {
        w = clamp(h * aspect, this.opts.minWidth, this.opts.maxWidth);
      }
    }

    if (w === p.size.width && h === p.size.height) return false;
    p.size.width = w;
    p.size.height = h;
    this.emit({ type: "resized", id });
    return true;
  }

  // ---------- opacity (§2.4) ----------

  /** Установить конкретное значение opacity (один из OPACITY_LEVELS). */
  setOpacity(id: PanelId, level: OpacityLevel): boolean {
    const p = this.panels.get(id);
    if (!p) return false;
    if (p.opacity === level) return false;
    p.opacity = level;
    this.emit({ type: "opacity", id });
    return true;
  }

  /** Циклически сменить opacity: 1.0 → 0.75 → 0.5 → 1.0. */
  cycleOpacity(id: PanelId): OpacityLevel | null {
    const p = this.panels.get(id);
    if (!p) return null;
    const i = OPACITY_LEVELS.indexOf(p.opacity);
    const next = OPACITY_LEVELS[(i + 1) % OPACITY_LEVELS.length];
    p.opacity = next;
    this.emit({ type: "opacity", id });
    return next;
  }

  // ---------- XR drag API (§2.7) ----------

  /**
   * Начать drag panel из XR-контроллера. controllerPosition — мировая позиция
   * ray-cast на panel. Сохраняем offset между центром panel и контроллером,
   * чтобы при последующих updateDragFromXr panel следовал за контроллером
   * "в руке".
   */
  startDragFromXr(id: PanelId, controllerPosition: { x: number; z: number }): boolean {
    const p = this.panels.get(id);
    if (!p) return false;
    this.xrDrag = {
      panelId: id,
      position: { ...controllerPosition },
      offset: {
        x: p.position.x - controllerPosition.x,
        z: p.position.z - controllerPosition.z
      }
    };
    return true;
  }

  /** Обновить позицию panel по XR-контроллеру. */
  updateDragFromXr(controllerPosition: { x: number; z: number }): boolean {
    if (!this.xrDrag) return false;
    const newX = controllerPosition.x + this.xrDrag.offset.x;
    const newZ = controllerPosition.z + this.xrDrag.offset.z;
    return this.move(this.xrDrag.panelId, newX, newZ);
  }

  /**
   * Завершить drag. Финализируем snap-зону (если близко к anchor — притянем).
   * Возвращает финальную SnapZone (или null, если не было активного drag).
   */
  endDragFromXr(thresholdMeters = 0.5): SnapZone | null {
    if (!this.xrDrag) return null;
    const id = this.xrDrag.panelId;
    this.xrDrag = null;
    return this.finalizeSnap(id, thresholdMeters);
  }

  /** Активен ли сейчас XR drag (для подсветки/курсора). */
  isXrDragging(): boolean {
    return this.xrDrag !== null;
  }

  // ---------- selection ----------

  select(id: PanelId | null): void {
    for (const p of this.panels.values()) p.selected = p.id === id;
  }

  // ---------- queries ----------

  list(): PanelState[] {
    return [...this.panels.values()].map((p) => clonePanel(p));
  }

  get(id: PanelId): PanelState | undefined {
    const p = this.panels.get(id);
    if (!p) return undefined;
    return clonePanel(p);
  }

  count(): number {
    return this.panels.size;
  }

  // ---------- persistence (§2.6) ----------

  /** Сериализовать текущий layout в JSON v1. */
  toJSON(): PanelLayoutJson {
    return {
      version: PANEL_LAYOUT_VERSION,
      panels: [...this.panels.values()].map((p) => clonePanel(p))
    };
  }

  /**
   * Загрузить layout из JSON. Если version неизвестен — false (без изменений).
   * При успехе заменяет текущие panels на загруженные. side-effect НЕ вызывает
   * emit-ы "created/closed" (только финальный "layout_reset"), чтобы
   * persistence-слой не закольцевался.
   */
  fromJSON(json: PanelLayoutJson): boolean {
    if (!json || json.version !== PANEL_LAYOUT_VERSION) return false;
    if (!Array.isArray(json.panels)) return false;
    this.panels.clear();
    let maxN = 0;
    for (const p of json.panels) {
      if (!isValidPanel(p)) continue;
      this.panels.set(p.id, clonePanel(p));
      const n = parseInt(String(p.id).replace(/^p/, ""), 10);
      if (!Number.isNaN(n) && n > maxN) maxN = n;
    }
    this.nextId = maxN + 1;
    this.emit({ type: "layout_reset" });
    return true;
  }

  // ---------- internals ----------

  private defaultPosition(): { x: number; y: number; z: number } {
    const offset = this.panels.size * 0.5;
    return { x: 0, y: this.opts.panelYOffset, z: -this.opts.radius + offset };
  }

  private facingTowards(pos: { x: number; z: number }): { x: number; z: number } {
    // Нормаль направлена к началу координат (где стоит пользователь).
    const fx = -pos.x;
    const fz = -pos.z;
    const len = Math.hypot(fx, fz);
    if (len < 1e-6) return { x: 0, z: 1 };
    return { x: fx / len, z: fz / len };
  }
}

// ---------- helpers ----------

function clamp(v: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, v));
}

/** Классифицировать позицию по X-threshold. */
function classifyByX(x: number, threshold: number): SnapZone {
  if (x < -threshold) return "left";
  if (x > threshold) return "right";
  return "center";
}

function clonePanel(p: PanelState): PanelState {
  return {
    ...p,
    position: { ...p.position },
    facing: { ...p.facing },
    size: { ...p.size }
  };
}

function isValidPanel(p: unknown): p is PanelState {
  if (!p || typeof p !== "object") return false;
  const obj = p as Record<string, unknown>;
  return (
    typeof obj.id === "string" &&
    typeof obj.topic === "string" &&
    typeof obj.position === "object" &&
    obj.position !== null &&
    typeof obj.facing === "object" &&
    obj.facing !== null &&
    typeof obj.size === "object" &&
    obj.size !== null
  );
}
