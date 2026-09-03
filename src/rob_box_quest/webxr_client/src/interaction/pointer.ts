// Указатель мостика: луч → наведение, клик, перетаскивание.
//
// Один слой на всех потребителей (B2 в аудите фич): панели тянутся и
// переключаются, панель режимов супервизора нажимается, по карте
// кликается цель навигации. Поэтому здесь нет ничего про панели —
// только «луч попал в объект X», «нажали», «тянут», «отпустили».
//
// Источники луча:
//   • desktop — мышь через камеру (`desktop_pointer.ts`);
//   • WebXR — контроллер, trigger (кнопка 0 oculus-touch-v2; grip'ы
//     заняты голосом, стик — телеопом) (`xr_pointer.ts`).
//
// Клик vs драг различаются по пройденному расстоянию (CLICK_SLOP_M):
// увёл луч дальше порога — это перетаскивание, не увёл — это выбор.

import * as THREE from "three";
import {
  cornerFromUv,
  dragTargetPosition,
  horizontalRadius,
  isDrag,
  type PanelCorner,
  type Vec3
} from "./pointer_math";

export interface PointerTarget {
  id: string;
  object: THREE.Object3D;
  /** Можно ли тащить. Кнопки панели режимов — нет, видео-панели — да. */
  draggable?: boolean;
}

export interface PointerRay {
  origin: Vec3;
  direction: Vec3;
  /** Нажат ли trigger / левая кнопка мыши в этом кадре. */
  pressed: boolean;
}

export interface PointerHandlers {
  /** Луч зашёл на объект или ушёл со всех (`null`). */
  onHover?(id: string | null): void;
  /** Нажали и отпустили, не уводя луч — выбор. */
  onSelect?(id: string): void;
  onDragStart?(id: string): void;
  /** Новая позиция объекта на сфере вокруг оператора. */
  onDrag?(id: string, position: Vec3): void;
  onDragEnd?(id: string): void;
  /**
   * Луч попал в угловую «ручку» resize. Хендлер должен изменить
   * размер панели; сигнатура аналогична onDrag — позиция луча на
   * сфере вокруг оператора. `corner` — какой угол схватили.
   */
  onResizeStart?(id: string, corner: PanelCorner): void;
  onResize?(id: string, corner: PanelCorner, position: Vec3): void;
  onResizeEnd?(id: string, corner: PanelCorner): void;
}

export interface PointerSystemOptions {
  /** Центр сферы перетаскивания — оператор. Default (0, 1.6, 0). */
  center?: Vec3;
  handlers?: PointerHandlers;
}

const DEFAULT_CENTER: Vec3 = { x: 0, y: 1.6, z: 0 };

export class PointerSystem {
  private targets: PointerTarget[] = [];
  private raycaster = new THREE.Raycaster();
  private handlers: PointerHandlers;
  private center: Vec3;

  private hoveredId: string | null = null;
  private pressedId: string | null = null;
  private pressOrigin: Vec3 | null = null;
  private pressCorner: PanelCorner | null = null;
  private dragging = false;
  private resizing = false;
  private dragRadius = 2;
  private wasPressed = false;

  constructor(opts: PointerSystemOptions = {}) {
    this.center = opts.center ?? DEFAULT_CENTER;
    this.handlers = opts.handlers ?? {};
  }

  setCenter(center: Vec3): void {
    this.center = center;
  }

  addTarget(target: PointerTarget): void {
    this.removeTarget(target.id);
    this.targets.push(target);
  }

  removeTarget(id: string): void {
    this.targets = this.targets.filter((t) => t.id !== id);
  }

  clearTargets(): void {
    this.targets = [];
  }

  getHovered(): string | null {
    return this.hoveredId;
  }

  isDragging(): boolean {
    return this.dragging;
  }

  /**
   * Один кадр указателя. `null` — луча нет (мышь ушла с канваса,
   * контроллер пропал): снимаем наведение и корректно закрываем драг.
   */
  update(ray: PointerRay | null): void {
    if (!ray) {
      this.finishPress(null);
      this.setHover(null);
      this.wasPressed = false;
      return;
    }

    const pick = this.pick(ray);
    const hitId = pick?.id ?? null;

    // Во время драга/ресайза наведение не переезжает на другие объекты —
    // иначе панель «перепрыгивает» на соседнюю при пересечении лучом.
    if (!this.dragging && !this.resizing) this.setHover(hitId);

    const justPressed = ray.pressed && !this.wasPressed;
    const justReleased = !ray.pressed && this.wasPressed;
    this.wasPressed = ray.pressed;

    if (justPressed && hitId) {
      this.pressedId = hitId;
      this.pressOrigin = this.rayPoint(ray);
      this.pressCorner = pick?.corner ?? null;
      this.dragRadius = this.radiusOf(hitId);
      // Угловой «зажим» начинается сразу как resize (без CLICK_SLOP_M):
      // оператор целился в угол, ждать порог — только мешать.
      if (this.pressCorner !== null) {
        this.resizing = true;
        this.handlers.onResizeStart?.(hitId, this.pressCorner);
      }
      return;
    }

    if (ray.pressed && this.pressedId) {
      const point = this.rayPoint(ray);
      if (this.resizing && this.pressCorner) {
        this.handlers.onResize?.(this.pressedId, this.pressCorner, point);
        return;
      }
      const target = this.targets.find((t) => t.id === this.pressedId);
      const canDrag = target?.draggable ?? false;
      if (!this.dragging && canDrag && this.pressOrigin && isDrag(this.pressOrigin, point)) {
        this.dragging = true;
        this.handlers.onDragStart?.(this.pressedId);
      }
      if (this.dragging) {
        this.handlers.onDrag?.(
          this.pressedId,
          dragTargetPosition(ray.origin, ray.direction, this.center, this.dragRadius)
        );
      }
      return;
    }

    if (justReleased) this.finishPress(hitId);
  }

  /** Луч попал в объект? Возвращает id ближайшего + угол (если угловой хит). */
  private pick(ray: PointerRay): { id: string; corner: PanelCorner | null } | null {
    if (this.targets.length === 0) return null;
    this.raycaster.set(
      new THREE.Vector3(ray.origin.x, ray.origin.y, ray.origin.z),
      new THREE.Vector3(ray.direction.x, ray.direction.y, ray.direction.z).normalize()
    );
    const objects = this.targets.map((t) => t.object);
    const hits = this.raycaster.intersectObjects(objects, true);
    if (hits.length === 0) return null;
    // Ищем, какому таргету принадлежит попавшийся меш (может быть ребёнком).
    for (const hit of hits) {
      const id = this.ownerOf(hit.object);
      if (!id) continue;
      const corner = hit.uv ? cornerFromUv(hit.uv) : null;
      return { id, corner };
    }
    return null;
  }

  private ownerOf(object: THREE.Object3D): string | null {
    for (const t of this.targets) {
      let node: THREE.Object3D | null = object;
      while (node) {
        if (node === t.object) return t.id;
        node = node.parent;
      }
    }
    return null;
  }

  private radiusOf(id: string): number {
    const target = this.targets.find((t) => t.id === id);
    if (!target) return 2;
    const p = target.object.position;
    return horizontalRadius({ x: p.x, y: p.y, z: p.z }, this.center);
  }

  /** Точка на сфере, куда сейчас светит луч — база для клик/драг порога. */
  private rayPoint(ray: PointerRay): Vec3 {
    return dragTargetPosition(ray.origin, ray.direction, this.center, this.dragRadius);
  }

  private finishPress(hitId: string | null): void {
    const pressedId = this.pressedId;
    const pressedCorner = this.pressCorner;
    if (pressedId) {
      if (this.resizing && pressedCorner) {
        this.handlers.onResizeEnd?.(pressedId, pressedCorner);
      } else if (this.dragging) {
        this.handlers.onDragEnd?.(pressedId);
      } else if (hitId === pressedId) {
        // Отпустили на том же объекте и не тянули — это выбор.
        this.handlers.onSelect?.(pressedId);
      }
    }
    this.pressedId = null;
    this.pressOrigin = null;
    this.pressCorner = null;
    this.dragging = false;
    this.resizing = false;
  }

  private setHover(id: string | null): void {
    if (id === this.hoveredId) return;
    this.hoveredId = id;
    this.handlers.onHover?.(id);
  }
}
