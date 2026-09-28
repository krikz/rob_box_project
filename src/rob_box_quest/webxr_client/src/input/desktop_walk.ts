// Ходьба оператора по мостику на десктопе (issue #3149): pointer-lock
// mouse look + WASD, Shift — бегом. Для симулятора (?sim=1) и для обычного
// десктопа без шлема.
//
// Двигается КАМЕРА оператора внутри мостика, не робот: сцена эгоцентрична
// (начало координат — base_link), робота ведут стрелки при зажатом Space
// (input/desktop_teleop.ts). В VR всё это выключено — позу головы там
// задаёт WebXR.
//
// Захват мыши: клик по канвасу → requestPointerLock (сам клик делает
// interaction/desktop_pointer.ts, чтобы захватывающий клик не нажал
// заодно панель). Esc — браузер отпускает захват сам.

import type * as THREE from "three";
import { EYE_HEIGHT_M } from "../scene/captain_bridge";

/** Скорость шага и бега, м/с. */
export const WALK_SPEED_MPS = 1.6;
export const RUN_SPEED_MPS = 3.5;
/** Радиан на пиксель движения мыши. */
export const MOUSE_SENSITIVITY = 0.0022;
const MAX_PITCH = (85 * Math.PI) / 180;

/**
 * Где можно ходить (x/z сцены). Комната мостика — ROOM_W × ROOM_D =
 * 11.6 × 9.12 м (scripts/build_bridge_assets.mjs), минус 0.35 м до стен.
 * Спереди граница — перед экраном-стеной (z = −3.9), чтобы не проходить
 * сквозь него.
 */
export const WALK_BOUNDS = Object.freeze({ minX: -5.45, maxX: 5.45, minZ: -3.55, maxZ: 4.21 });

export interface WalkPose {
  x: number;
  z: number;
  yaw: number;
  pitch: number;
}

export interface WalkInput {
  forward: number; // -1..1 (W − S)
  strafe: number; // -1..1 (D − A)
  run: boolean;
}

/**
 * Чистый шаг: сдвиг позы по вводу за `dtS` с зажимом в WALK_BOUNDS.
 * yaw = 0 смотрит в −Z (на экран-стену), положительный yaw — налево.
 */
export function stepWalk(pose: WalkPose, input: WalkInput, dtS: number): WalkPose {
  let f = input.forward;
  let s = input.strafe;
  const len = Math.hypot(f, s);
  if (len > 1) {
    f /= len;
    s /= len;
  }
  const speed = input.run ? RUN_SPEED_MPS : WALK_SPEED_MPS;
  const sin = Math.sin(pose.yaw);
  const cos = Math.cos(pose.yaw);
  // Вперёд камеры в XZ: (−sin yaw, −cos yaw); вправо: (cos yaw, −sin yaw).
  const dx = (-sin * f + cos * s) * speed * dtS;
  const dz = (-cos * f - sin * s) * speed * dtS;
  return {
    x: clamp(pose.x + dx, WALK_BOUNDS.minX, WALK_BOUNDS.maxX),
    z: clamp(pose.z + dz, WALK_BOUNDS.minZ, WALK_BOUNDS.maxZ),
    yaw: pose.yaw,
    pitch: pose.pitch
  };
}

/** Поворот головы мышью; pitch зажат ±85°. */
export function applyMouseLook(pose: WalkPose, movementX: number, movementY: number): WalkPose {
  return {
    ...pose,
    yaw: pose.yaw - movementX * MOUSE_SENSITIVITY,
    pitch: clamp(pose.pitch - movementY * MOUSE_SENSITIVITY, -MAX_PITCH, MAX_PITCH)
  };
}

function clamp(v: number, lo: number, hi: number): number {
  return Math.min(hi, Math.max(lo, v));
}

/** Клавиши ходьбы. Не пересекаются с телеопом (стрелки/Space/E). */
export const WALK_KEYS: ReadonlyArray<string> = Object.freeze([
  "KeyW",
  "KeyA",
  "KeyS",
  "KeyD",
  "ShiftLeft",
  "ShiftRight"
]);

export interface DesktopWalkOptions {
  canvas: HTMLElement;
  camera: THREE.Camera;
  /** В VR ходьба выключена: позу головы задаёт WebXR. */
  isXrActive: () => boolean;
  /** Родитель прицела (по умолчанию document.body). */
  overlayParent?: HTMLElement;
}

export interface DesktopWalkHandle {
  /** Захвачена ли мышь (прицел в центре экрана). */
  isLocked(): boolean;
  getPose(): WalkPose;
  destroy(): void;
}

export function createDesktopWalk(opts: DesktopWalkOptions): DesktopWalkHandle {
  const { canvas, camera } = opts;
  const pressed = new Set<string>();
  let pose: WalkPose = { x: camera.position.x, z: camera.position.z, yaw: 0, pitch: 0 };
  let raf = 0;
  let last = performance.now();

  // Прицел: видно только при захвате мыши — туда смотрит луч указателя.
  const crosshair = document.createElement("div");
  crosshair.className = "walk-crosshair";
  crosshair.setAttribute("aria-hidden", "true");
  crosshair.hidden = true;
  (opts.overlayParent ?? document.body).appendChild(crosshair);

  // Подсказка «кликни, чтобы осмотреться», пока мышь не захвачена.
  const hint = document.createElement("div");
  hint.className = "walk-hint";
  hint.textContent = "Клик — осмотреться мышью · WASD — ходить · Shift — бегом · Esc — отпустить";
  (opts.overlayParent ?? document.body).appendChild(hint);

  const locked = (): boolean => document.pointerLockElement === canvas;

  function onLockChange(): void {
    const on = locked();
    crosshair.hidden = !on;
    hint.hidden = on;
    if (!on) pressed.clear();
  }

  function onMouseMove(ev: MouseEvent): void {
    if (!locked() || opts.isXrActive()) return;
    pose = applyMouseLook(pose, ev.movementX ?? 0, ev.movementY ?? 0);
  }

  function isTyping(ev: KeyboardEvent): boolean {
    const t = ev.target as HTMLElement | null;
    const tag = t?.tagName;
    return tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT" || Boolean(t?.isContentEditable);
  }

  function onKeyDown(ev: KeyboardEvent): void {
    if (isTyping(ev) || !WALK_KEYS.includes(ev.code)) return;
    if (ev.ctrlKey || ev.metaKey || ev.altKey) return;
    pressed.add(ev.code);
  }

  function onKeyUp(ev: KeyboardEvent): void {
    pressed.delete(ev.code);
  }

  function onBlur(): void {
    pressed.clear();
  }

  function frame(now: number): void {
    raf = requestAnimationFrame(frame);
    const dt = Math.min(0.05, Math.max(0, (now - last) / 1000));
    last = now;
    if (opts.isXrActive()) return;
    const input: WalkInput = {
      forward: (pressed.has("KeyW") ? 1 : 0) - (pressed.has("KeyS") ? 1 : 0),
      strafe: (pressed.has("KeyD") ? 1 : 0) - (pressed.has("KeyA") ? 1 : 0),
      run: pressed.has("ShiftLeft") || pressed.has("ShiftRight")
    };
    if (input.forward !== 0 || input.strafe !== 0) pose = stepWalk(pose, input, dt);
    camera.position.set(pose.x, EYE_HEIGHT_M, pose.z);
    camera.rotation.set(pose.pitch, pose.yaw, 0, "YXZ");
  }

  document.addEventListener("pointerlockchange", onLockChange);
  document.addEventListener("mousemove", onMouseMove);
  window.addEventListener("keydown", onKeyDown);
  window.addEventListener("keyup", onKeyUp);
  window.addEventListener("blur", onBlur);
  raf = requestAnimationFrame(frame);

  return {
    isLocked: locked,
    getPose: () => ({ ...pose }),
    destroy(): void {
      cancelAnimationFrame(raf);
      document.removeEventListener("pointerlockchange", onLockChange);
      document.removeEventListener("mousemove", onMouseMove);
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
      window.removeEventListener("blur", onBlur);
      crosshair.remove();
      hint.remove();
      if (locked()) document.exitPointerLock?.();
    }
  };
}
