// След робота на полу (issue #3151) — чистая логика, без Three.js.
//
// Копим позы робота в `map` из кадров map_2d (~5 Гц) и отдаём хвост на
// отрисовку: последние `maxPoints` точек не старше `maxAgeMs`, у каждой —
// «свежесть» 1 → 0, по ней линия гаснет к хвосту.
//
// Почему в `map`, а не в base_link. Робот едет, base_link едет вместе с
// ним; след по определению — это где робот БЫЛ, то есть неподвижные точки
// мира. Храним в `map` и каждый кадр перекладываем в сцену по свежей позе
// (nav_frames.mapToScene) — ровно как путь Nav2 и карту.
//
// Прыжок локализации. rtabmap при релокализации переносит позу на метры
// за один кадр. Отрезок через полкомнаты — не след, а артефакт; поэтому
// скачок больше `jumpResetM` начинает след заново.

import type { Pose2D, Xy } from "./nav_frames";

export interface TrailPoint extends Xy {
  tMs: number;
}

export interface TrailSample extends Xy {
  /** 1 — только что, 0 — на границе `maxAgeMs`. */
  freshness: number;
}

export interface OdomTrailOptions {
  /** Сколько точек держать. Default 300 (≈ 1 мин при 5 Гц без прорежения). */
  maxPoints?: number;
  /** Сколько жить точке. Default 120 с. */
  maxAgeMs?: number;
  /** Точки ближе этого к последней не добавляем (стоим на месте). Default 5 см. */
  minStepM?: number;
  /** Скачок дальше — сброс следа (релокализация). Default 2 м. */
  jumpResetM?: number;
}

export class OdomTrail {
  private readonly maxPoints: number;
  private readonly maxAgeMs: number;
  private readonly minStepM: number;
  private readonly jumpResetM: number;
  private pts: TrailPoint[] = [];

  constructor(opts: OdomTrailOptions = {}) {
    this.maxPoints = opts.maxPoints ?? 300;
    this.maxAgeMs = opts.maxAgeMs ?? 120_000;
    this.minStepM = opts.minStepM ?? 0.05;
    this.jumpResetM = opts.jumpResetM ?? 2.0;
  }

  /** Новая поза робота. Возвращает `true`, если след изменился. */
  push(pose: Pose2D, tMs: number): boolean {
    const last = this.pts[this.pts.length - 1];
    if (last) {
      const d = Math.hypot(pose.x - last.x, pose.y - last.y);
      if (d > this.jumpResetM) {
        this.pts = [];
      } else if (d < this.minStepM) {
        return false;
      }
    }
    this.pts.push({ x: pose.x, y: pose.y, tMs });
    if (this.pts.length > this.maxPoints) this.pts.splice(0, this.pts.length - this.maxPoints);
    return true;
  }

  /** Живые точки на момент `nowMs`, от старых к новым. */
  samples(nowMs: number): TrailSample[] {
    const cutoff = nowMs - this.maxAgeMs;
    let first = 0;
    while (first < this.pts.length && this.pts[first].tMs < cutoff) first += 1;
    if (first > 0) this.pts.splice(0, first);
    return this.pts.map((p) => ({
      x: p.x,
      y: p.y,
      freshness: Math.max(0, Math.min(1, 1 - (nowMs - p.tMs) / this.maxAgeMs))
    }));
  }

  clear(): void {
    this.pts = [];
  }

  size(): number {
    return this.pts.length;
  }
}
