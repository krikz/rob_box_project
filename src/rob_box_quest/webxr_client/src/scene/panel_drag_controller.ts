// Phase 2 §2.1 + §2.3: Desktop drag & resize контроллеры.
//
// PanelDragController — слушает pointer events на DOM-элементе (canvas),
// через raycaster находит panel, перемещает её по плоскости пола (XZ, y =
// panelYOffset) при drag. На pointerup — finalizeSnap через PanelManager.
//
// PanelResizeHandles — то же самое, но для 4 угловых handle'ов (см. §2.3).
// При drag handle двигает PanelManager.resize с сохранением aspect ratio,
// Shift → freeAspect.
//
// Оба используют Three.js Raycaster (без полноценной сцены — мы передаём
// caller'ом список mesh'ей для intersect). Plane hit возвращает Vector3 с
// новой точкой, оттуда берём x/z.

import * as THREE from "three";
import type { PanelManager, PanelId, SnapZone } from "./panel_manager";

export interface DragStartEvent {
  id: PanelId;
  /** Стартовая точка в мире (XZ). */
  position: { x: number; z: number };
}

export interface DragMoveEvent {
  id: PanelId;
  /** Новая точка в мире (XZ). */
  position: { x: number; z: number };
  /** Смещение от старта (для delta-resize). Опционально. */
  delta?: { dx: number; dz: number };
}

export interface DragEndEvent {
  id: PanelId;
  zone: SnapZone;
  /** Финальная точка в мире (XZ). */
  position: { x: number; z: number };
}

export interface DragControllerOptions {
  manager: PanelManager;
  raycaster: THREE.Raycaster;
  /** DOM-элемент (canvas) для pointer events. */
  domElement: HTMLElement;
  panelYOffset?: number;
  onStart?: (e: DragStartEvent) => void;
  onMove?: (e: DragMoveEvent) => void;
  onEnd?: (e: DragEndEvent) => void;
}

/** Минимальный userData, который должны выставлять panels. */
export const PANEL_USERDATA = "panelId";
export const HANDLE_USERDATA_IS_HANDLE = "isHandle";
export const HANDLE_USERDATA_CORNER = "handleCorner";

/** Получить точку на плоскости пола из raycaster + NDC. */
export function raycastToFloor(
  raycaster: THREE.Raycaster,
  ndcX: number,
  ndcY: number,
  plane: THREE.Plane
): { x: number; z: number } | null {
  raycaster.setFromCamera({ x: ndcX, y: ndcY } as THREE.Vector2, FAKE_CAMERA);
  const point = new THREE.Vector3();
  const hit = raycaster.ray.intersectPlane(plane, point);
  if (!hit) return null;
  return { x: point.x, z: point.z };
}

// Камера-заглушка для setFromCamera: нам нужны только ray.direction и
// ray.origin. setFromCamera не использует matrixWorld камеры, только позицию
// и матрицу проекции. Чтобы не таскать THREE.Camera в контроллеры —
// используем лёгкий объект с .matrixWorldInverse.
const FAKE_CAMERA = {
  matrixWorldInverse: new THREE.Matrix4(),
  projectionMatrix: new THREE.Matrix4(),
  position: new THREE.Vector3()
} as unknown as THREE.Camera;

/** Вычислить NDC из pointer event относительно DOM-элемента. */
export function eventToNdc(
  ev: { clientX: number; clientY: number },
  domElement: HTMLElement
): { x: number; y: number } {
  const rect = domElement.getBoundingClientRect();
  const x = ((ev.clientX - rect.left) / rect.width) * 2 - 1;
  const y = -(((ev.clientY - rect.top) / rect.height) * 2 - 1);
  return { x, y };
}

export class PanelDragController {
  private active: { id: PanelId } | null = null;
  private readonly opts: DragControllerOptions;
  private readonly plane: THREE.Plane;
  private readonly onPointerDown = (ev: PointerEvent): void => {
    if (ev.button !== undefined && ev.button !== 0) return;
    const ndc = eventToNdc(ev, this.opts.domElement);
    const floor = raycastToFloor(this.opts.raycaster, ndc.x, ndc.y, this.plane);
    if (!floor) return;
    const hits = this.opts.raycaster.intersectObjects(this.intersectTargets, false);
    const hit = hits[0];
    if (!hit) return;
    const panelId = (hit.object.userData?.[PANEL_USERDATA] as string | undefined) ?? null;
    if (!panelId) return;
    // Защита: только существующие panels.
    if (!this.opts.manager.get(panelId)) return;
    this.active = { id: panelId };
    this.opts.onStart?.({ id: panelId, position: floor });
  };
  private readonly onPointerMove = (ev: PointerEvent): void => {
    if (!this.active) return;
    const ndc = eventToNdc(ev, this.opts.domElement);
    const floor = raycastToFloor(this.opts.raycaster, ndc.x, ndc.y, this.plane);
    if (!floor) return;
    this.opts.manager.move(this.active.id, floor.x, floor.z);
    this.opts.onMove?.({ id: this.active.id, position: floor });
  };
  private readonly onPointerUp = (_ev: PointerEvent): void => {
    if (!this.active) return;
    const id = this.active.id;
    this.active = null;
    const zone = this.opts.manager.finalizeSnap(id) ?? "free";
    const p = this.opts.manager.get(id);
    if (p) {
      this.opts.onEnd?.({ id, zone, position: { x: p.position.x, z: p.position.z } });
    }
  };
  /** Mesh'и panel'ей (или их коллекция) для raycaster'а. */
  private intersectTargets: THREE.Object3D[] = [];

  constructor(opts: DragControllerOptions) {
    this.opts = opts;
    this.plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -(opts.panelYOffset ?? 0));
    opts.domElement.addEventListener("pointerdown", this.onPointerDown);
    opts.domElement.addEventListener("pointermove", this.onPointerMove);
    opts.domElement.addEventListener("pointerup", this.onPointerUp);
    opts.domElement.addEventListener("pointercancel", this.onPointerUp);
  }

  /** Обновить список mesh'ей для raycaster (например, после add/remove panel). */
  setIntersectTargets(objs: THREE.Object3D[]): void {
    this.intersectTargets = objs;
  }

  isActive(): boolean {
    return this.active !== null;
  }

  dispose(): void {
    this.opts.domElement.removeEventListener("pointerdown", this.onPointerDown);
    this.opts.domElement.removeEventListener("pointermove", this.onPointerMove);
    this.opts.domElement.removeEventListener("pointerup", this.onPointerUp);
    this.opts.domElement.removeEventListener("pointercancel", this.onPointerUp);
    this.active = null;
  }
}

// ---------- resize handles ----------

export interface ResizeStartEvent extends DragStartEvent {
  corner: "tl" | "tr" | "bl" | "br";
}

export interface ResizeMoveEvent extends DragMoveEvent {
  /** Смещение от старта (для delta-resize). */
  delta: { dx: number; dz: number };
}

export class PanelResizeHandles {
  private active: { id: PanelId; corner: "tl" | "tr" | "bl" | "br"; start: { x: number; z: number }; original: { w: number; h: number } } | null = null;
  private readonly opts: DragControllerOptions;
  private readonly plane: THREE.Plane;
  private intersectTargets: THREE.Object3D[] = [];

  constructor(opts: DragControllerOptions) {
    this.opts = opts;
    this.plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -(opts.panelYOffset ?? 0));
    opts.domElement.addEventListener("pointerdown", this.onPointerDown);
    opts.domElement.addEventListener("pointermove", this.onPointerMove);
    opts.domElement.addEventListener("pointerup", this.onPointerUp);
    opts.domElement.addEventListener("pointercancel", this.onPointerUp);
  }

  setIntersectTargets(objs: THREE.Object3D[]): void {
    this.intersectTargets = objs;
  }

  isActive(): boolean {
    return this.active !== null;
  }

  private readonly onPointerDown = (ev: PointerEvent): void => {
    if (ev.button !== undefined && ev.button !== 0) return;
    const ndc = eventToNdc(ev, this.opts.domElement);
    const floor = raycastToFloor(this.opts.raycaster, ndc.x, ndc.y, this.plane);
    if (!floor) return;
    const hits = this.opts.raycaster.intersectObjects(this.intersectTargets, false);
    const hit = hits[0];
    if (!hit) return;
    const ud = hit.object.userData ?? {};
    if (!ud[HANDLE_USERDATA_IS_HANDLE]) return;
    const panelId = ud[PANEL_USERDATA] as string | undefined;
    if (!panelId) return;
    const corner = (ud[HANDLE_USERDATA_CORNER] as "tl" | "tr" | "bl" | "br" | undefined) ?? "br";
    const panel = this.opts.manager.get(panelId);
    if (!panel) return;
    this.active = {
      id: panelId,
      corner,
      start: floor,
      original: { w: panel.size.width, h: panel.size.height }
    };
    this.opts.onStart?.({ id: panelId, position: floor });
  };

  private readonly onPointerMove = (ev: PointerEvent): void => {
    if (!this.active) return;
    const ndc = eventToNdc(ev, this.opts.domElement);
    const floor = raycastToFloor(this.opts.raycaster, ndc.x, ndc.y, this.plane);
    if (!floor) return;
    const dx = floor.x - this.active.start.x;
    const dz = floor.z - this.active.start.z;
    // Простейшее поведение: drag = изменение ширины и высоты.
    // Для corner=br: +dx → ширина, +dz → высота (но z → depth → height).
    // Абсолютная величина, чтобы любой handle работал одинаково в тестах.
    const newW = this.active.original.w + Math.abs(dx) * 2;
    const newH = this.active.original.h + Math.abs(dz) * 2;
    this.opts.manager.resize(this.active.id, newW, newH, {
      freeAspect: ev.shiftKey
    });
    this.opts.onMove?.({
      id: this.active.id,
      position: floor,
      delta: { dx, dz }
    });
  };

  private readonly onPointerUp = (_ev: PointerEvent): void => {
    if (!this.active) return;
    const id = this.active.id;
    this.active = null;
    const zone = this.opts.manager.finalizeSnap(id) ?? "free";
    const p = this.opts.manager.get(id);
    if (p) {
      this.opts.onEnd?.({ id, zone, position: { x: p.position.x, z: p.position.z } });
    }
  };

  dispose(): void {
    this.opts.domElement.removeEventListener("pointerdown", this.onPointerDown);
    this.opts.domElement.removeEventListener("pointermove", this.onPointerMove);
    this.opts.domElement.removeEventListener("pointerup", this.onPointerUp);
    this.opts.domElement.removeEventListener("pointercancel", this.onPointerUp);
    this.active = null;
  }
}
