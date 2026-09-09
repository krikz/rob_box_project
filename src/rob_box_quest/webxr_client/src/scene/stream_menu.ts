// Меню выбора стрима для панели (R10, A5).
//
// Всплывает рядом с выбранной панелью: список доступных топиков из
// `stream_list`, текущий помечен. Каждая строка — отдельный маленький
// меш, зарегистрированный в PointerSystem как цель `menu:<topic>`:
// так попадание считает сам raycaster, и не нужно вычислять строку по
// UV-координате.
//
// Почему не DOM-оверлей: в immersive-vr DOM не виден вообще, а меню
// нужно именно в VR. Canvas-текстура на меше — тот же приём, что у
// status HUD и ARM-индикатора.

import * as THREE from "three";

/** Префикс id целей указателя, чтобы отличать строки меню от панелей. */
export const MENU_TARGET_PREFIX = "menu:";

export interface StreamMenuRow {
  topic: string;
  /** Человеко-читаемое описание из registry (`stream_list`). */
  description?: string;
}

export interface StreamMenuHandle {
  readonly object: THREE.Group;
  /** Меши строк для регистрации в PointerSystem: id → объект. */
  targets(): Array<{ id: string; object: THREE.Object3D }>;
  /** Показать меню у панели с текущим топиком `current`. */
  show(position: THREE.Vector3, facingAngleY: number, current: string): void;
  hide(): void;
  isVisible(): boolean;
  dispose(): void;
}

const ROW_W = 1.1;
const ROW_H = 0.16;
const ROW_GAP = 0.02;
const TEX_W = 512;
const TEX_H = 72;

/** Из id цели указателя обратно в topic (`menu:camera_rear` → `camera_rear`). */
export function topicFromTargetId(id: string): string | null {
  return id.startsWith(MENU_TARGET_PREFIX) ? id.slice(MENU_TARGET_PREFIX.length) : null;
}

export function createStreamMenu(rows: StreamMenuRow[]): StreamMenuHandle {
  const group = new THREE.Group();
  group.visible = false;
  // Меню — поверх видео-панелей: иначе строка ныряет внутрь панели,
  // рядом с которой она всплыла.
  group.renderOrder = 20;

  const entries = rows.map((row, i) => {
    const canvas = document.createElement("canvas");
    canvas.width = TEX_W;
    canvas.height = TEX_H;
    const ctx = canvas.getContext("2d");
    if (!ctx) throw new Error("stream_menu: failed to acquire 2D context");
    const texture = new THREE.CanvasTexture(canvas);
    texture.minFilter = THREE.LinearFilter;
    texture.magFilter = THREE.LinearFilter;
    const mesh = new THREE.Mesh(
      new THREE.PlaneGeometry(ROW_W, ROW_H),
      new THREE.MeshBasicMaterial({ map: texture, transparent: true, depthTest: false })
    );
    mesh.renderOrder = 20;
    // Строки идут вниз от верха меню.
    mesh.position.set(0, -i * (ROW_H + ROW_GAP), 0);
    group.add(mesh);
    return { row, canvas, ctx, texture, mesh };
  });

  function draw(current: string): void {
    for (const e of entries) {
      const active = e.row.topic === current;
      const { ctx } = e;
      ctx.clearRect(0, 0, TEX_W, TEX_H);
      ctx.fillStyle = active ? "rgba(46, 194, 126, 0.85)" : "rgba(10, 13, 17, 0.85)";
      ctx.fillRect(0, 0, TEX_W, TEX_H);
      ctx.fillStyle = active ? "#0a0d11" : "#2ec27e";
      ctx.fillRect(0, 0, 8, TEX_H);
      ctx.fillStyle = active ? "#0a0d11" : "#d6dde5";
      ctx.font = "bold 30px monospace";
      ctx.textBaseline = "middle";
      ctx.fillText(e.row.topic, 24, TEX_H / 2);
      e.texture.needsUpdate = true;
    }
  }

  return {
    object: group,
    targets() {
      return entries.map((e) => ({ id: MENU_TARGET_PREFIX + e.row.topic, object: e.mesh }));
    },
    show(position: THREE.Vector3, facingAngleY: number, current: string): void {
      draw(current);
      // Меню встаёт над панелью и повёрнуто так же, как она.
      group.position.set(position.x, position.y + 0.5, position.z);
      group.rotation.y = facingAngleY;
      group.visible = true;
    },
    hide(): void {
      group.visible = false;
    },
    isVisible(): boolean {
      return group.visible;
    },
    dispose(): void {
      for (const e of entries) {
        e.mesh.geometry.dispose();
        (e.mesh.material as THREE.Material).dispose();
        e.texture.dispose();
      }
    }
  };
}
