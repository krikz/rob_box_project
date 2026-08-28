// Менеджер floating panels — модель состояний, тестируемая без Three.js.
//
// Концепция "капитанский мостик" (дизайн §1): пользователь стоит в центре,
// вокруг — N floating panels с видео-потоками. Каждой panel можно:
//   - переместить (desktop drag, XR — point+trigger)
//   - сменить stream (например, с camera_rear на camera_oak_color)
//   - закрыть
// Layout reset возвращает дефолтную раскладку (полукруг, radius 2м).

export type PanelId = string;

/** Snap-zone куда магнитит panel: left/center/right относительно "вперёд" (-Z). */
export type PanelZone = "left" | "center" | "right";

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

export interface PanelManagerOptions {
  radius?: number;       // радиус полукруга, дефолт 2.0 м
  panelWidth?: number;   // 1.2 м
  panelHeight?: number;  // 0.7 м
  panelYOffset?: number; // 1.6 (высота глаз — см. дизайн §3)
  defaultTopics?: string[]; // порядок по умолчанию
}

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
      defaultTopics: opts.defaultTopics ?? [...DEFAULT_VIDEO_TOPICS]
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
      selected: false
    });
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
    return true;
  }

  /**
   * Snap-to-zone: если panel в пределах snapRadius от центра зоны —
   * магнитит к центру зоны. Зоны: left/center/right относительно
   * направления "вперёд" (ось -Z).
   *
   * Возвращает {x, z, snapped: boolean} — caller решает, применить ли.
   * snapRadius в метрах; типично 0.4–0.6 м.
   */
  snapToZone(
    _id: PanelId,
    x: number,
    z: number,
    snapRadius = 0.5
  ): { x: number; z: number; snapped: boolean; zone: PanelZone | null } {
    const zones: Array<{ zone: PanelZone; cx: number; cz: number }> = [
      { zone: "left", cx: -this.opts.radius, cz: 0 },
      { zone: "center", cx: 0, cz: -this.opts.radius },
      { zone: "right", cx: this.opts.radius, cz: 0 }
    ];
    let best: { zone: PanelZone; cx: number; cz: number; d: number } | null = null;
    for (const zone of zones) {
      const d = Math.hypot(x - zone.cx, z - zone.cz);
      if (best === null || d < best.d) best = { ...zone, d };
    }
    if (best && best.d <= snapRadius) {
      return { x: best.cx, z: best.cz, snapped: true, zone: best.zone };
    }
    return { x, z, snapped: false, zone: null };
  }

  /**
   * Переместить panel с автоматическим snap-to-zone. Применяет
   * snapToZone() и, если примагнитило, обновляет panel через move().
   * Возвращает {snapped, zone} для UI feedback.
   */
  moveWithSnap(
    id: PanelId,
    x: number,
    z: number,
    snapRadius = 0.5
  ): { moved: boolean; snapped: boolean; zone: PanelZone | null } {
    const snap = this.snapToZone(id, x, z, snapRadius);
    const moved = this.move(id, snap.x, snap.z);
    return { moved, snapped: snap.snapped, zone: snap.zone };
  }

  select(id: PanelId | null): void {
    for (const p of this.panels.values()) p.selected = p.id === id;
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