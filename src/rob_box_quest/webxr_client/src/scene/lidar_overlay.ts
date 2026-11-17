// LiDAR-overlay: облако точек скана, построенное В ПРОСТРАНСТВЕ вокруг
// точки, которую мостик считает центром робота.
//
// Где центр. Начало координат сцены (0, 0, 0) — это `base_link` робота,
// то есть пол ровно под оператором (камера стоит в (0, 1.6, 0)). Скан
// строится от этой точки, а не «где-то на полу»: оператор физически
// стоит там, где робот, и видит вокруг себя ту же геометрию, что видит
// лидар.
//
// Высота. Луч N10 идёт не по полу, а по плоскости на 0.4765 м выше
// base_link (`rob_box.xacro:338`), сам лидар смещён на 0.17 м назад.
// Рисуем точки именно на этой высоте — иначе это уже не «показания в
// пространстве», а проекция на пол, и стены комнаты читаются неверно.
//
// Видимость (почему не просто THREE.Points на y=0):
//   1. `fog: false` — у сцены туман 6→16 м, а лидар видит до 10 м:
//      дальние точки просто растворялись в фоне.
//   2. `depthTest: false` — виртуальная комната мостика ~7×8 м, скан —
//      до 10 м в радиусе, то есть половина точек оказывалась ЗА стенами
//      и была не видна. Данные сенсора важнее декорации, поэтому рисуем
//      поверх (renderOrder). Отключается опцией `alwaysVisible: false`.
//   3. «Занавес» — вертикальные линии от пола до плоскости луча. Точка
//      диаметром 6 см на дистанции 8 м с высоты глаз почти не читается;
//      вертикальный штрих читается всегда и сразу показывает стены.
//   4. Контрастная палитра: близко — красный, дальше — жёлтый, далеко —
//      зелёный. Прежний градиент уходил в синий (0x0000ff) на фоне
//      #0a0d11, то есть дальние точки были не видны в принципе.

import * as THREE from "three";
import { parseLidar2d, scanToFloorPoints } from "./lidar_payload";

const MAX_POINTS = 4096; // 360° LiDAR × 2 не нужно; ограничиваем для VRAM.

/** Высота плоскости луча N10 над base_link (`rob_box.xacro:338`). */
export const LIDAR_HEIGHT_M = 0.4765;
/** Смещение лидара назад от центра робота (там же, x = −0.170806). */
export const LIDAR_OFFSET_Z_M = 0.170806;
/** Дальность N10 (`lsx10_custom.yaml: max_range`) — потолок цветовой шкалы. */
export const LIDAR_MAX_RANGE_M = 10.0;

export interface LidarOverlayOptions {
  /** Центр робота в координатах сцены. Default (0,0,0) — под оператором. */
  center?: { x: number; y: number; z: number };
  /** Высота плоскости луча над центром. Default — из URDF. */
  beamHeight?: number;
  /** Рисовать вертикальный «занавес» от пола до луча. Default true. */
  curtain?: boolean;
  /** Рисовать поверх геометрии комнаты и без тумана. Default true. */
  alwaysVisible?: boolean;
  /** Дальность, на которой цвет доходит до «далеко». Default 10 м. */
  maxRange?: number;
}

export class LidarOverlay {
  /** Корень оверлея — двигается вместе с центром робота. */
  readonly object: THREE.Group;
  readonly points: THREE.Points;
  readonly curtain: THREE.LineSegments | null;

  private geometry: THREE.BufferGeometry;
  private positionAttr: THREE.BufferAttribute;
  private colorAttr: THREE.BufferAttribute;
  private curtainGeometry: THREE.BufferGeometry | null = null;
  private curtainPositionAttr: THREE.BufferAttribute | null = null;
  private curtainColorAttr: THREE.BufferAttribute | null = null;
  private readonly beamHeight: number;
  private readonly maxRange: number;

  constructor(opts: LidarOverlayOptions = {}) {
    const alwaysVisible = opts.alwaysVisible ?? true;
    this.beamHeight = opts.beamHeight ?? LIDAR_HEIGHT_M;
    this.maxRange = opts.maxRange ?? LIDAR_MAX_RANGE_M;

    this.geometry = new THREE.BufferGeometry();
    this.positionAttr = new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3);
    this.colorAttr = new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3);
    this.positionAttr.setUsage(THREE.DynamicDrawUsage);
    this.colorAttr.setUsage(THREE.DynamicDrawUsage);
    this.geometry.setAttribute("position", this.positionAttr);
    this.geometry.setAttribute("color", this.colorAttr);
    this.geometry.setDrawRange(0, 0);

    const material = new THREE.PointsMaterial({
      size: 0.07,
      vertexColors: true,
      sizeAttenuation: true,
      transparent: true,
      opacity: 1.0,
      depthTest: !alwaysVisible,
      depthWrite: false,
      fog: false
    });
    this.points = new THREE.Points(this.geometry, material);
    this.points.frustumCulled = false; // точки живут дальше стен комнаты
    this.points.renderOrder = 10;

    this.object = new THREE.Group();
    const c = opts.center ?? { x: 0, y: 0, z: 0 };
    this.object.position.set(c.x, c.y, c.z);
    this.object.add(this.points);

    if (opts.curtain ?? true) {
      this.curtainGeometry = new THREE.BufferGeometry();
      // 2 вершины на луч: пол → плоскость луча.
      this.curtainPositionAttr = new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 6), 3);
      this.curtainColorAttr = new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 6), 3);
      this.curtainPositionAttr.setUsage(THREE.DynamicDrawUsage);
      this.curtainColorAttr.setUsage(THREE.DynamicDrawUsage);
      this.curtainGeometry.setAttribute("position", this.curtainPositionAttr);
      this.curtainGeometry.setAttribute("color", this.curtainColorAttr);
      this.curtainGeometry.setDrawRange(0, 0);
      const curtainMat = new THREE.LineBasicMaterial({
        vertexColors: true,
        transparent: true,
        opacity: 0.45,
        depthTest: !alwaysVisible,
        depthWrite: false,
        fog: false
      });
      this.curtain = new THREE.LineSegments(this.curtainGeometry, curtainMat);
      this.curtain.frustumCulled = false;
      this.curtain.renderOrder = 9;
      this.object.add(this.curtain);
    } else {
      this.curtain = null;
    }
  }

  /** Перенести центр робота (например, если оператор не в точке робота). */
  setCenter(x: number, y: number, z: number): void {
    this.object.position.set(x, y, z);
  }

  /** Принять LiDAR-payload. */
  ingestPayload(payload: Uint8Array): void {
    try {
      const scan = parseLidar2d(payload);
      const points = scanToFloorPoints(scan);
      const count = Math.min(points.length, MAX_POINTS);
      const posArr = this.positionAttr.array as Float32Array;
      const colArr = this.colorAttr.array as Float32Array;
      const cPos = this.curtainPositionAttr?.array as Float32Array | undefined;
      const cCol = this.curtainColorAttr?.array as Float32Array | undefined;
      // Плоскость луча выше пола; сам лидар сдвинут назад от центра робота.
      const y = this.beamHeight;
      const zOff = LIDAR_OFFSET_Z_M;
      for (let i = 0; i < count; i += 1) {
        const p = points[i];
        const px = p.x;
        const pz = p.z + zOff;
        posArr[i * 3] = px;
        posArr[i * 3 + 1] = y;
        posArr[i * 3 + 2] = pz;
        const c = rangeColor(p.range, this.maxRange);
        colArr[i * 3] = c[0];
        colArr[i * 3 + 1] = c[1];
        colArr[i * 3 + 2] = c[2];
        if (cPos && cCol) {
          // Нижняя вершина — на полу, верхняя — в точке луча.
          cPos[i * 6] = px;
          cPos[i * 6 + 1] = 0.01;
          cPos[i * 6 + 2] = pz;
          cPos[i * 6 + 3] = px;
          cPos[i * 6 + 4] = y;
          cPos[i * 6 + 5] = pz;
          cCol[i * 6] = c[0];
          cCol[i * 6 + 1] = c[1];
          cCol[i * 6 + 2] = c[2];
          cCol[i * 6 + 3] = c[0];
          cCol[i * 6 + 4] = c[1];
          cCol[i * 6 + 5] = c[2];
        }
      }
      this.positionAttr.needsUpdate = true;
      this.colorAttr.needsUpdate = true;
      this.geometry.setDrawRange(0, count);
      if (this.curtainGeometry && this.curtainPositionAttr && this.curtainColorAttr) {
        this.curtainPositionAttr.needsUpdate = true;
        this.curtainColorAttr.needsUpdate = true;
        this.curtainGeometry.setDrawRange(0, count * 2);
      }
    } catch {
      // Ошибка парсинга → молча игнорируем (см. parseLidar2d).
    }
  }

  dispose(): void {
    this.geometry.dispose();
    (this.points.material as THREE.Material).dispose();
    this.curtainGeometry?.dispose();
    if (this.curtain) (this.curtain.material as THREE.Material).dispose();
  }
}

/**
 * Цвет по дистанции: близко (опасно) — красный, средне — жёлтый,
 * далеко — зелёный. Красно-жёлто-зелёная шкала читается на тёмном фоне
 * мостика и совпадает с ADR-0027 §4.4 («зелёный → красный»).
 */
export function rangeColor(range: number, maxRange: number): [number, number, number] {
  const t = Math.max(0, Math.min(1, range / (maxRange || 1)));
  // hue 0 (красный) → 1/3 (зелёный), насыщенность полная, светлота 0.55.
  return hslToRgb(t / 3, 1, 0.55);
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
