// LiDAR-overlay: THREE.Points на полу сцены (y=0), vertex colors по дистанции.

import * as THREE from "three";
import { parseLidar2d, scanToFloorPoints } from "./lidar_payload";

const MAX_POINTS = 4096; // 360° LiDAR × 2 не нужно; ограничиваем для VRAM.

export class LidarOverlay {
  readonly object: THREE.Points;
  private geometry: THREE.BufferGeometry;
  private positionAttr: THREE.BufferAttribute;
  private colorAttr: THREE.BufferAttribute;

  constructor() {
    this.geometry = new THREE.BufferGeometry();
    this.positionAttr = new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3);
    this.colorAttr = new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3);
    this.positionAttr.setUsage(THREE.DynamicDrawUsage);
    this.colorAttr.setUsage(THREE.DynamicDrawUsage);
    this.geometry.setAttribute("position", this.positionAttr);
    this.geometry.setAttribute("color", this.colorAttr);
    this.geometry.setDrawRange(0, 0);

    const material = new THREE.PointsMaterial({
      size: 0.05,
      vertexColors: true,
      sizeAttenuation: true,
      transparent: true,
      opacity: 0.95
    });
    this.object = new THREE.Points(this.geometry, material);
    this.object.position.y = 0.01; // чуть выше пола
  }

  /** Принять LiDAR-payload. */
  ingestPayload(payload: Uint8Array): void {
    try {
      const scan = parseLidar2d(payload);
      const points = scanToFloorPoints(scan);
      const count = Math.min(points.length, MAX_POINTS);
      const posArr = this.positionAttr.array as Float32Array;
      const colArr = this.colorAttr.array as Float32Array;
      for (let i = 0; i < count; i += 1) {
        const p = points[i];
        posArr[i * 3] = p.x;
        posArr[i * 3 + 1] = 0;
        posArr[i * 3 + 2] = p.z;
        // цвет: близко=красный, далеко=синий (HSL hue: 0°→240°).
        const t = Math.min(1, p.range / 8);
        const c = hslToRgb(t * 0.66, 1, 0.5);
        colArr[i * 3] = c[0];
        colArr[i * 3 + 1] = c[1];
        colArr[i * 3 + 2] = c[2];
      }
      this.positionAttr.needsUpdate = true;
      this.colorAttr.needsUpdate = true;
      this.geometry.setDrawRange(0, count);
    } catch {
      // Ошибка парсинга → молча игнорируем (см. parseLidar2d).
    }
  }

  dispose(): void {
    this.geometry.dispose();
    (this.object.material as THREE.Material).dispose();
  }
}

function hslToRgb(h: number, s: number, l: number): [number, number, number] {
  // h ∈ [0,1]; s,l ∈ [0,1]. Возвращает rgb ∈ [0,1].
  const f = (n: number): number => {
    const k = (n + h * 12) % 12;
    const a = s * Math.min(l, 1 - l);
    const v = l - a * Math.max(-1, Math.min(Math.min(k - 3, 9 - k), 1));
    return v;
  };
  return [f(0), f(8), f(4)];
}