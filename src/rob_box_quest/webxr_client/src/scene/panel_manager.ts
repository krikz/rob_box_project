// Менеджер floating panels — модель состояний, тестируемая без Three.js.
//
// Концепция "капитанский мостик" (дизайн §1): пользователь стоит в центре,
// вокруг — N floating panels с видео-потоками. Каждой panel можно:
//   - переместить (desktop drag, XR — point+trigger)
//   - сменить stream (например, с camera_rear на camera_oak_color)
//   - закрыть
// Layout reset возвращает дефолтную раскладку (полукруг, radius 2м).

export type PanelId = string;

export interface PanelState {
  id: PanelId;
  topic: string;
  /** позиция в плоскости пола (XZ), y=panelHeight (глаза пользователя) */
  position: { x: number; z: number; y: number };
  /** нормаль (panel смотрит на пользователя) — единичный вектор */
  facing: { x: number; z: number };
  /** ширина/высота панели в метрах */
  size: { width: number; height: number };
  selected: boolean;
}

/** Минимально-разумные границы размера панели в метрах (см. AV-25 B2). */
export const PANEL_DEFAULT_MIN_WIDTH_M = 0.4;
export const PANEL_DEFAULT_MAX_WIDTH_M = 3.0;
export const PANEL_DEFAULT_MIN_HEIGHT_M = 0.3;
export const PANEL_DEFAULT_MAX_HEIGHT_M = 2.0;

export interface PanelManagerOptions {
  radius?: number;       // радиус полукруга, дефолт 2.0 м
  panelWidth?: number;   // 1.2 м
  panelHeight?: number;  // 0.7 м
  panelYOffset?: number; // 1.6 (высота глаз — см. дизайн §3)
  defaultTopics?: string[]; // порядок по умолчанию
  /**
   * Углы (в градусах) вокруг направления «вперёд» для resetLayout.
   * Дефолт — полукруг из дизайна §3. Мостик передаёт свои: экран-стена
   * занимает фронт, поэтому боковые панели уезжают к ±75°.
   */
  angles?: number[];
  /** Границы resize. Дефолты — PANEL_DEFAULT_*_M, чтобы панель нельзя
   * было схлопнуть в точку или растянуть на всю комнату. */
  minWidthM?: number;
  maxWidthM?: number;
  minHeightM?: number;
  maxHeightM?: number;
}

/** Углы дефолтной раскладки (дизайн §3, 4 панели полукругом). */
export const DEFAULT_PANEL_ANGLES_DEG = [-60, -20, 20, 60];

export const DEFAULT_VIDEO_TOPICS = [
  "camera_rear",
  "camera_oak_color",
  "camera_oak_depth",
  "camera_ceiling"
] as const;

export class PanelManager {
  private panels = new Map<PanelId, PanelState>();
  private nextId = 1;
  private readonly opts: Required<PanelManagerOptions>;

  constructor(opts: PanelManagerOptions = {}) {
    this.opts = {
      radius: opts.radius ?? 2.0,
      panelWidth: opts.panelWidth ?? 1.2,
      panelHeight: opts.panelHeight ?? 0.7,
      panelYOffset: opts.panelYOffset ?? 1.6,
      defaultTopics: opts.defaultTopics ?? [...DEFAULT_VIDEO_TOPICS],
      angles: opts.angles ?? [...DEFAULT_PANEL_ANGLES_DEG],
      minWidthM: opts.minWidthM ?? PANEL_DEFAULT_MIN_WIDTH_M,
      maxWidthM: opts.maxWidthM ?? PANEL_DEFAULT_MAX_WIDTH_M,
      minHeightM: opts.minHeightM ?? PANEL_DEFAULT_MIN_HEIGHT_M,
      maxHeightM: opts.maxHeightM ?? PANEL_DEFAULT_MAX_HEIGHT_M
    };
  }

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
    const anglesDeg = this.opts.angles;
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
    return ids;
  }

  /**
   * Перегрузка для двух вариантов вызова:
   *   - старая: `createPanel(topic, position?, facing?)` — id генерируется;
   *   - новая:  `createPanel(id, topic, position?, facing?)` — для layout-store.
   *
   * Различаем по типу ВТОРОГО аргумента: string = topic (новая сигнатура,
   * id задан первым аргументом), object/undefined = position (старая).
   */
  createPanel(
    topicOrId: string,
    positionOrTopic?: { x: number; y: number; z: number } | string,
    facingOrPosition?: { x: number; z: number } | { x: number; y: number; z: number },
    maybeFacing?: { x: number; z: number }
  ): PanelId {
    let id: PanelId;
    let topic: string;
    let pos: { x: number; y: number; z: number } | undefined;
    let face: { x: number; z: number } | undefined;
    if (typeof positionOrTopic === "string") {
      // (id, topic, pos?, facing?) — новая сигнатура для layout-store.
      id = topicOrId;
      topic = positionOrTopic;
      pos = facingOrPosition as { x: number; y: number; z: number } | undefined;
      face = maybeFacing;
    } else {
      // (topic, pos?, facing?) — старая сигнатура.
      id = `p${this.nextId}`;
      topic = topicOrId;
      pos = positionOrTopic;
      face = facingOrPosition as { x: number; z: number } | undefined;
    }
    if (this.panels.has(id)) return id;
    this.nextId += 1;
    const finalPos = pos ?? this.defaultPosition();
    const finalFace = face ?? this.facingTowards(finalPos);
    this.panels.set(id, {
      id,
      topic,
      position: { ...finalPos },
      facing: { ...finalFace },
      size: { width: this.opts.panelWidth, height: this.opts.panelHeight },
      selected: false
    });
    // Поддерживаем nextId выше этого значения, чтобы автогенерация
    // не пересеклась с явно заданными id.
    const m = /^p(\d+)$/.exec(id);
    if (m) {
      const n = parseInt(m[1], 10);
      if (Number.isFinite(n) && n >= this.nextId) this.nextId = n + 1;
    }
    return id;
  }

  close(id: PanelId): boolean {
    return this.panels.delete(id);
  }

  switchStream(id: PanelId, newTopic: string): boolean {
    const p = this.panels.get(id);
    if (!p) return false;
    p.topic = newTopic;
    return true;
  }

  /**
   * Переставить панель. `y` опционален: драг лучом (interaction/pointer)
   * катает панель по сфере вокруг оператора и меняет высоту тоже, а
   * старые вызовы (раскладка) двигают только по полу.
   */
  move(id: PanelId, x: number, z: number, y?: number): boolean {
    const p = this.panels.get(id);
    if (!p) return false;
    p.position.x = x;
    p.position.z = z;
    if (y !== undefined) p.position.y = y;
    p.facing.x = -x;
    p.facing.z = -z;
    const len = Math.hypot(p.facing.x, p.facing.z);
    if (len > 1e-6) {
      p.facing.x /= len;
      p.facing.z /= len;
    }
    return true;
  }

  select(id: PanelId | null): void {
    for (const p of this.panels.values()) p.selected = p.id === id;
  }

  /**
   * Изменить размер панели. Возвращает true, если что-то реально
   * поменялось (после клампа) — нужно для дебаунса layout-store, чтобы
   * он не дёргал localStorage из-за попыток сжать панель в ноль.
   */
  resize(id: PanelId, width: number, height: number): boolean {
    const p = this.panels.get(id);
    if (!p) return false;
    const w = clamp(width, this.opts.minWidthM, this.opts.maxWidthM);
    const h = clamp(height, this.opts.minHeightM, this.opts.maxHeightM);
    if (w === p.size.width && h === p.size.height) return false;
    p.size.width = w;
    p.size.height = h;
    return true;
  }

  list(): PanelState[] {
    return [...this.panels.values()].map((p) => ({
      ...p,
      position: { ...p.position },
      facing: { ...p.facing },
      size: { ...p.size }
    }));
  }

  get(id: PanelId): PanelState | undefined {
    const p = this.panels.get(id);
    if (!p) return undefined;
    return {
      ...p,
      position: { ...p.position },
      facing: { ...p.facing },
      size: { ...p.size }
    };
  }

  count(): number {
    return this.panels.size;
  }

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

function clamp(v: number, lo: number, hi: number): number {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}