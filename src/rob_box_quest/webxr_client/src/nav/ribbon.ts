// Лента на полу из ломаной (issue #3151) — чистая геометрия без Three.js.
//
// Почему лента, а не THREE.Line. WebGL рисует линии толщиной ровно в
// 1 пиксель (linewidth игнорируется на всех платформах, включая Quest).
// Путь Nav2 в 1 px на полу с высоты 1.6 м не читается; плоская лента
// заданной ширины в метрах читается на любом расстоянии и светится с
// аддитивным блендингом.

import type { SceneXz } from "./nav_frames";

export interface RibbonGeometry {
  /** xyz на вершину: по две вершины (лево/право) на точку ломаной. */
  positions: Float32Array;
  /** Треугольники: по два на сегмент. */
  indices: Uint32Array;
}

/**
 * Лента полуширины `halfWidth` на высоте `y` вдоль ломаной `pts`.
 * Нормаль в каждой точке — по среднему направлению соседних сегментов,
 * так что изломы пути не рвут ленту. Меньше двух точек — пустая геометрия.
 */
export function buildRibbon(pts: readonly SceneXz[], halfWidth: number, y: number): RibbonGeometry {
  const n = pts.length;
  if (n < 2) return { positions: new Float32Array(0), indices: new Uint32Array(0) };
  const positions = new Float32Array(n * 2 * 3);
  for (let i = 0; i < n; i += 1) {
    const a = pts[Math.max(0, i - 1)];
    const b = pts[Math.min(n - 1, i + 1)];
    let tx = b.x - a.x;
    let tz = b.z - a.z;
    const len = Math.hypot(tx, tz);
    if (len < 1e-9) {
      tx = 0;
      tz = -1;
    } else {
      tx /= len;
      tz /= len;
    }
    // Нормаль в плоскости пола: поворот касательной на 90°.
    const nx = -tz * halfWidth;
    const nz = tx * halfWidth;
    const o = i * 6;
    positions[o] = pts[i].x + nx;
    positions[o + 1] = y;
    positions[o + 2] = pts[i].z + nz;
    positions[o + 3] = pts[i].x - nx;
    positions[o + 4] = y;
    positions[o + 5] = pts[i].z - nz;
  }
  const indices = new Uint32Array((n - 1) * 6);
  for (let i = 0; i < n - 1; i += 1) {
    const v = i * 2;
    const o = i * 6;
    indices[o] = v;
    indices[o + 1] = v + 1;
    indices[o + 2] = v + 2;
    indices[o + 3] = v + 1;
    indices[o + 4] = v + 3;
    indices[o + 5] = v + 2;
  }
  return { positions, indices };
}
