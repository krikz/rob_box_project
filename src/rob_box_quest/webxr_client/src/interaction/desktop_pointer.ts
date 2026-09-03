// Мышь → PointerRay (desktop-режим мостика).
//
// В VR тот же PointerSystem кормится лучом контроллера (`xr_pointer.ts`);
// здесь — луч из камеры через позицию курсора. Состояние копится в
// обработчиках событий, а сам кадр забирает `poll()` из render-loop:
// так указатель обновляется синхронно со сценой, а не в произвольный
// момент между кадрами.

import * as THREE from "three";
import type { PointerRay } from "./pointer";

export interface DesktopPointerHandle {
  /** Луч текущего кадра или `null`, если курсор ушёл с канваса. */
  poll(): PointerRay | null;
  destroy(): void;
}

export interface DesktopPointerOptions {
  canvas: HTMLElement;
  camera: THREE.Camera;
}

export function createDesktopPointer(opts: DesktopPointerOptions): DesktopPointerHandle {
  const { canvas, camera } = opts;
  const ndc = new THREE.Vector2();
  const raycaster = new THREE.Raycaster();
  let inside = false;
  let pressed = false;

  function onMove(ev: MouseEvent): void {
    const rect = canvas.getBoundingClientRect();
    if (rect.width < 1 || rect.height < 1) return;
    ndc.x = ((ev.clientX - rect.left) / rect.width) * 2 - 1;
    ndc.y = -((ev.clientY - rect.top) / rect.height) * 2 + 1;
    inside = true;
  }

  function onDown(ev: MouseEvent): void {
    if (ev.button !== 0) return; // только левая кнопка
    pressed = true;
  }

  function onUp(ev: MouseEvent): void {
    if (ev.button !== 0) return;
    pressed = false;
  }

  function onLeave(): void {
    inside = false;
    // Кнопку тоже отпускаем: mouseup за пределами канваса мы не увидим,
    // и панель осталась бы «прилипшей» к курсору.
    pressed = false;
  }

  canvas.addEventListener("mousemove", onMove);
  canvas.addEventListener("mousedown", onDown);
  window.addEventListener("mouseup", onUp);
  canvas.addEventListener("mouseleave", onLeave);

  return {
    poll(): PointerRay | null {
      if (!inside) return null;
      raycaster.setFromCamera(ndc, camera);
      const o = raycaster.ray.origin;
      const d = raycaster.ray.direction;
      return {
        origin: { x: o.x, y: o.y, z: o.z },
        direction: { x: d.x, y: d.y, z: d.z },
        pressed
      };
    },
    destroy(): void {
      canvas.removeEventListener("mousemove", onMove);
      canvas.removeEventListener("mousedown", onDown);
      window.removeEventListener("mouseup", onUp);
      canvas.removeEventListener("mouseleave", onLeave);
    }
  };
}
