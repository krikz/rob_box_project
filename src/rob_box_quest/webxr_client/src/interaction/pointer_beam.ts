// Визуализация указателя: луч + курсор в точке попадания.
//
// Что было не так с прежней отрисовкой (`captain_bridge.attachXrSession`):
// на каждый контроллер вешалась `THREE.Line` фиксированной длины 1.5 м
// цветом `0x2ec27e`. Три следствия, и все три оператор чувствует как
// «луча не видно и жмётся не туда»:
//
//   1. Панели мостика стоят на радиусе 2.4–4 м, а луч кончался на 1.5 м —
//      он физически не доставал до того, во что целятся. Оператор видел
//      огрызок в воздухе и гадал, куда тот показывает.
//   2. `0x2ec27e` — это COLOR_ACCENT всех меню и активных кнопок. Зелёный
//      луч на зелёной подсветке кнопки не читается вовсе.
//   3. Не было ни курсора, ни реакции на наведение: попал луч в кнопку или
//      прошёл в сантиметре мимо — картинка одна и та же.
//
// Здесь всё три чинятся: длина луча = реальное расстояние до цели (из
// `PointerSystem.getHit()`, то есть ровно то, во что попадёт клик), цвет
// зависит от состояния, а в точке попадания рисуется диск-курсор.
//
// Луч живёт в мировых координатах сцены (не как child контроллера):
// `xrPointerRay` отдаёт позу в том же reference space ('local-floor'),
// что использует three.js для XR-камеры, поэтому позиции совпадают, а
// сцена-родитель избавляет от возни с локальными матрицами контроллера.

import * as THREE from "three";
import type { PointerHit } from "./pointer";

/** Длина луча, когда он не попал ни во что: «в пустоту», но видимо. */
export const BEAM_MISS_LENGTH_M = 6;
/** Луч не короче этого, даже если цель вплотную (иначе он схлопывается). */
export const BEAM_MIN_LENGTH_M = 0.08;

/** Радиус курсора-диска на расстоянии 1 м; дальше масштабируется линейно. */
const CURSOR_RADIUS_AT_1M = 0.012;

/**
 * Цвета состояний. Намеренно НЕ зелёные: зелёный занят подсветкой активных
 * кнопок, и луч на них сливался. Холодный белый читается на любой панели.
 */
const COLOR_MISS = 0x6b7d8f; // серо-голубой: луч в пустоту
const COLOR_HOVER = 0x8fd4ff; // светло-голубой: под лучом есть цель
const COLOR_PRESS = 0xf5c211; // янтарный: trigger зажат

export type BeamState = "miss" | "hover" | "press";

export function beamState(hit: PointerHit): BeamState {
  if (hit.pressed) return "press";
  return hit.id !== null ? "hover" : "miss";
}

function colorFor(state: BeamState): number {
  if (state === "press") return COLOR_PRESS;
  return state === "hover" ? COLOR_HOVER : COLOR_MISS;
}

/**
 * Длина луча под кадр указателя: до цели, если попали, иначе фиксированная
 * «в пустоту». Чистая функция — тестируется без three.js.
 */
export function beamLength(hit: PointerHit): number {
  if (hit.distanceM === null || !Number.isFinite(hit.distanceM)) {
    return BEAM_MISS_LENGTH_M;
  }
  return Math.max(BEAM_MIN_LENGTH_M, hit.distanceM);
}

export interface PointerBeamHandle {
  /** Добавляется в сцену один раз. */
  readonly object: THREE.Group;
  /**
   * Обновить под кадр указателя. `ray === null` (контроллер пропал, мышь
   * ушла с канваса) — луч прячется целиком: рисовать указатель, которого
   * нет, значит врать.
   */
  update(
    ray: { origin: { x: number; y: number; z: number }; direction: { x: number; y: number; z: number } } | null,
    hit: PointerHit
  ): void;
  setVisible(visible: boolean): void;
  dispose(): void;
}

export function createPointerBeam(): PointerBeamHandle {
  const group = new THREE.Group();
  group.visible = false;
  // Поверх видео-панелей и меню: указатель, спрятанный за панелью, —
  // ровно та беда, ради которой этот модуль и написан.
  group.renderOrder = 30;

  // Луч: цилиндр, а не Line. LineBasicMaterial на Quest игнорирует
  // linewidth (WebGL-ограничение) и рисуется волоском в один пиксель —
  // на фоне видео его не видно. Цилиндр имеет реальную толщину.
  const beamGeom = new THREE.CylinderGeometry(0.004, 0.0022, 1, 6, 1, true);
  // Геометрия строится вдоль +Y с центром в нуле; сдвигаем так, чтобы
  // луч начинался в начале координат и рос в +Y — тогда длина задаётся
  // одним scale.y, без пересборки геометрии на каждый кадр.
  beamGeom.translate(0, 0.5, 0);
  const beamMat = new THREE.MeshBasicMaterial({
    color: COLOR_MISS,
    transparent: true,
    opacity: 0.75,
    depthTest: false,
    depthWrite: false
  });
  const beam = new THREE.Mesh(beamGeom, beamMat);
  beam.renderOrder = 30;
  group.add(beam);

  // Курсор: диск в точке попадания. Именно он отвечает на вопрос «куда я
  // тыкаю» — луч показывает направление, курсор показывает точку.
  const cursorGeom = new THREE.CircleGeometry(1, 20);
  const cursorMat = new THREE.MeshBasicMaterial({
    color: COLOR_HOVER,
    transparent: true,
    opacity: 0.95,
    depthTest: false,
    depthWrite: false,
    side: THREE.DoubleSide
  });
  const cursor = new THREE.Mesh(cursorGeom, cursorMat);
  cursor.renderOrder = 31;
  group.add(cursor);

  // Кольцо вокруг курсора: контрастный контур, чтобы точка читалась и на
  // светлом кадре видео, и на тёмной панели.
  const ringGeom = new THREE.RingGeometry(1.5, 2.1, 20);
  const ringMat = new THREE.MeshBasicMaterial({
    color: 0x0a0d11,
    transparent: true,
    opacity: 0.8,
    depthTest: false,
    depthWrite: false,
    side: THREE.DoubleSide
  });
  const ring = new THREE.Mesh(ringGeom, ringMat);
  ring.renderOrder = 31;
  group.add(ring);

  const dirVec = new THREE.Vector3();
  const originVec = new THREE.Vector3();
  const upY = new THREE.Vector3(0, 1, 0);
  const quat = new THREE.Quaternion();

  function update(
    ray: { origin: { x: number; y: number; z: number }; direction: { x: number; y: number; z: number } } | null,
    hit: PointerHit
  ): void {
    if (!ray) {
      group.visible = false;
      return;
    }
    dirVec.set(ray.direction.x, ray.direction.y, ray.direction.z);
    if (dirVec.lengthSq() === 0) {
      group.visible = false;
      return;
    }
    dirVec.normalize();
    originVec.set(ray.origin.x, ray.origin.y, ray.origin.z);

    const state = beamState(hit);
    const color = colorFor(state);
    beamMat.color.setHex(color);
    cursorMat.color.setHex(color);

    const length = beamLength(hit);
    beam.position.copy(originVec);
    // Геометрия растёт вдоль +Y — доворачиваем её на направление луча.
    quat.setFromUnitVectors(upY, dirVec);
    beam.quaternion.copy(quat);
    beam.scale.set(1, length, 1);
    // Луч в пустоту приглушаем: он информирует о направлении, но не должен
    // спорить за внимание с лучом, реально наведённым на цель.
    beamMat.opacity = state === "miss" ? 0.35 : 0.85;

    const hasPoint = hit.point !== null;
    cursor.visible = hasPoint;
    ring.visible = hasPoint;
    if (hit.point) {
      cursor.position.set(hit.point.x, hit.point.y, hit.point.z);
      ring.position.copy(cursor.position);
      // Курсор смотрит на оператора вдоль луча — иначе на панели,
      // повёрнутой ребром, он вырождается в полоску.
      cursor.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), dirVec.clone().negate());
      ring.quaternion.copy(cursor.quaternion);
      // Постоянный угловой размер: близкий курсор не раздувается,
      // дальний не превращается в точку.
      const scale = CURSOR_RADIUS_AT_1M * Math.max(0.5, length);
      cursor.scale.setScalar(scale);
      ring.scale.setScalar(scale);
      // Чуть приподнимаем над поверхностью, чтобы курсор не z-fight'ил
      // с панелью (depthTest выключен, но порядок отрисовки помогает).
      cursor.position.addScaledVector(dirVec, -0.005);
      ring.position.copy(cursor.position);
    }
    group.visible = true;
  }

  return {
    object: group,
    update,
    setVisible(visible: boolean): void {
      group.visible = visible;
    },
    dispose(): void {
      beamGeom.dispose();
      beamMat.dispose();
      cursorGeom.dispose();
      cursorMat.dispose();
      ringGeom.dispose();
      ringMat.dispose();
    }
  };
}
