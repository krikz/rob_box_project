// Жест «цель навигации лучом по полу» (issue #3151) — чистая логика.
//
// Дизайн жеста (docs/architecture/meta-quest-api.md §5 «Nav-цель из VR»,
// docs/plans/2026-09-28-captain-bridge-rviz-vr-roadmap.md волна 2):
//
//   1. ВЗВЕСТИ прицел — отдельное действие, а не просто клик по полу:
//      XR — кнопка A/X (любая рука), desktop — клавиша G. Клик по полу
//      без взвода ничего не делает. Цель двигает робот — случайный клик
//      мимо панели не должен отправлять его через комнату.
//   2. НАЖАТЬ trigger / ЛКМ, целясь в пол (луч не на панели — панели
//      приоритетнее), — точка цели = пересечение луча с полом.
//   3. ПОТЯНУТЬ, не отпуская, — курс прибытия: из точки цели в сторону,
//      куда сейчас светит луч (порог `YAW_DRAG_MIN_M`). Не потянул — курс
//      «по ходу движения»: от робота к точке цели.
//   4. ОТПУСТИТЬ — цель уходит (`commit`), прицел сам разряжается: одна
//      взводка — одна цель.
//
// Отмена жеста: разрядить прицел (A/X или G ещё раз) или потерять луч,
// пока кнопка зажата, — ничего не уходит.
//
// Всё в координатах сцены = base_link (см. nav_frames.ts); в `map` цель
// переводит вызывающий по последней позе.

import type { SceneXz } from "./nav_frames";
import { sceneDirToBaseYaw } from "./nav_frames";

export interface GestureRay {
  origin: { x: number; y: number; z: number };
  direction: { x: number; y: number; z: number };
  pressed: boolean;
}

/** Дальше этого пересечение с полом не считаем целью (луч почти вдоль пола). */
export const MAX_GOAL_RANGE_M = 20;
/** Меньше этого натяг — курс не задан, берём «по ходу движения». */
export const YAW_DRAG_MIN_M = 0.3;
/** Точка цели ближе к роботу — курс «как сейчас» (0 в base_link). */
const NEAR_ROBOT_M = 0.05;

export interface GestureFrame {
  /** Взведён ли прицел. */
  armed: boolean;
  ray: GestureRay | null;
  /** Луч сейчас на панели/кнопке — она приоритетнее пола. */
  blocked: boolean;
}

export interface GestureOutput {
  /** Куда луч попал в пол (прицел взведён, кнопка не зажата). */
  reticle: SceneXz | null;
  /** Цель, которую сейчас выставляют (кнопка зажата). */
  preview: { point: SceneXz; yaw: number } | null;
  /** Жест завершён — отправить эту цель (курс в base_link, рад). */
  commit: { point: SceneXz; yaw: number } | null;
}

/** Пересечение луча с полом y = floorY. `null` — луч не вниз или дальше предела. */
export function rayFloorHit(
  ray: Pick<GestureRay, "origin" | "direction">,
  floorY = 0,
  maxRange = MAX_GOAL_RANGE_M
): SceneXz | null {
  const dy = ray.direction.y;
  if (dy >= -1e-6) return null;
  const t = (floorY - ray.origin.y) / dy;
  if (t <= 0) return null;
  const x = ray.origin.x + ray.direction.x * t;
  const z = ray.origin.z + ray.direction.z * t;
  if (Math.hypot(x, z) > maxRange) return null;
  return { x, z };
}

/** Курс прибытия для точки `anchor` при текущем натяге до `current`. */
export function goalYaw(anchor: SceneXz, current: SceneXz | null): number {
  if (current) {
    const dx = current.x - anchor.x;
    const dz = current.z - anchor.z;
    if (Math.hypot(dx, dz) >= YAW_DRAG_MIN_M) return sceneDirToBaseYaw(dx, dz);
  }
  if (Math.hypot(anchor.x, anchor.z) < NEAR_ROBOT_M) return 0;
  return sceneDirToBaseYaw(anchor.x, anchor.z);
}

export class NavGoalGesture {
  private wasPressed = false;
  private anchor: SceneXz | null = null;
  private current: SceneXz | null = null;

  isHolding(): boolean {
    return this.anchor !== null;
  }

  reset(): void {
    this.anchor = null;
    this.current = null;
  }

  update(f: GestureFrame): GestureOutput {
    const pressed = f.ray?.pressed ?? false;
    const justPressed = pressed && !this.wasPressed;
    const justReleased = !pressed && this.wasPressed;
    this.wasPressed = pressed;
    const none: GestureOutput = { reticle: null, preview: null, commit: null };

    if (!f.armed || !f.ray) {
      this.reset();
      return none;
    }
    const hit = rayFloorHit(f.ray);

    if (this.anchor) {
      if (hit) this.current = hit;
      const out = { point: this.anchor, yaw: goalYaw(this.anchor, this.current) };
      if (justReleased) {
        this.reset();
        return { reticle: null, preview: null, commit: out };
      }
      return { reticle: null, preview: out, commit: null };
    }

    if (justPressed && hit && !f.blocked) {
      this.anchor = hit;
      this.current = hit;
      return { reticle: null, preview: { point: hit, yaw: goalYaw(hit, hit) }, commit: null };
    }
    return { reticle: f.blocked || pressed ? null : hit, preview: null, commit: null };
  }
}
