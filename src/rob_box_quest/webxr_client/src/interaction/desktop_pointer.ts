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
  /**
   * Режим ходьбы (#3149, input/desktop_walk.ts): клик по канвасу без
   * захвата мыши только захватывает её (requestPointerLock) и НЕ жмёт
   * панель; при захвате луч идёт из центра экрана (прицел), клик = выбор.
   * Если pointer lock в браузере нет или он отказал (pointerlockerror /
   * отклонённый промис: встроенные панели, iframe без allow, политика) —
   * дальше поведение как без флага, иначе ни один клик не доходил бы до
   * сцены.
   */
  lockOnClick?: boolean;
}

export function createDesktopPointer(opts: DesktopPointerOptions): DesktopPointerHandle {
  const { canvas, camera } = opts;
  const ndc = new THREE.Vector2();
  const raycaster = new THREE.Raycaster();
  let inside = false;
  let pressed = false;
  /** Браузер отказал в pointer lock — дальше клики жмут, а не захватывают. */
  let lockDenied = false;

  const locked = (): boolean =>
    typeof document !== "undefined" && document.pointerLockElement === canvas;

  function onMove(ev: MouseEvent): void {
    if (locked()) return; // луч из центра, курсора нет
    const rect = canvas.getBoundingClientRect();
    if (rect.width < 1 || rect.height < 1) return;
    ndc.x = ((ev.clientX - rect.left) / rect.width) * 2 - 1;
    ndc.y = -((ev.clientY - rect.top) / rect.height) * 2 + 1;
    inside = true;
  }

  function onDown(ev: MouseEvent): void {
    if (ev.button !== 0) return; // только левая кнопка
    if (opts.lockOnClick && !lockDenied && !locked() && typeof canvas.requestPointerLock === "function") {
      // Захватывающий клик — только захват, панель под курсором не жмём.
      try {
        const p = canvas.requestPointerLock() as unknown;
        if (p instanceof Promise) p.catch(() => (lockDenied = true));
      } catch {
        lockDenied = true; // браузер отказал — остаёмся в режиме курсора
      }
      return;
    }
    pressed = true;
  }

  function onLockChange(): void {
    // Захват снят (Esc) посреди драга — отпускаем панель.
    pressed = false;
    if (locked()) inside = true;
    else inside = false;
  }

  function onLockError(): void {
    lockDenied = true;
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
  document.addEventListener("pointerlockchange", onLockChange);
  document.addEventListener("pointerlockerror", onLockError);
  const center = new THREE.Vector2(0, 0);

  return {
    poll(): PointerRay | null {
      const isLocked = locked();
      if (!inside && !isLocked) return null;
      raycaster.setFromCamera(isLocked ? center : ndc, camera);
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
      document.removeEventListener("pointerlockchange", onLockChange);
      document.removeEventListener("pointerlockerror", onLockError);
    }
  };
}
