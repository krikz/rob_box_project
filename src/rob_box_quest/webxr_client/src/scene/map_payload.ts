// Парсер map_2d payload (0x1103). Сервер: streams/occupancy.py:encode_map_2d.
//
// Кадр — MessagePack-map. Приходит в двух видах:
//   полный  — есть поле `png`: RGBA-картинка решётки, клиент пересобирает
//             текстуру пола;
//   лёгкий  — без `png`: только поза робота (~130 байт, 5 Гц), клиент
//             двигает уже готовую плоскость.
// Карта меняется редко, а поза — постоянно; гонять из-за позы PNG незачем.

import { decodeMsgpackMap } from "../wire/msgpack";

export interface MapFrame {
  /** Метров на клетку решётки. */
  resolution: number;
  /** Размер решётки в клетках. */
  width: number;
  height: number;
  /** Нижний-левый угол решётки в кадре `map` (метры). */
  originX: number;
  originY: number;
  /** Поза робота в кадре `map`. `null` — tf ещё не собрался. */
  robot: { x: number; y: number; yaw: number } | null;
  tsMs: number;
  /** RGBA PNG решётки. `null` у лёгкого кадра «только поза». */
  png: Uint8Array | null;
}

function num(v: unknown): number | null {
  return typeof v === "number" && Number.isFinite(v) ? v : null;
}

/**
 * Разобрать map_2d payload. `null` — кадр битый или неполный: вызывающий
 * его просто пропускает (как и остальные стримы, битый кадр не должен
 * ронять мостик).
 */
export function parseMapFrame(payload: Uint8Array): MapFrame | null {
  const map = decodeMsgpackMap(payload);
  if (!map) return null;
  const resolution = num(map.resolution);
  const width = num(map.width);
  const height = num(map.height);
  const originX = num(map.origin_x);
  const originY = num(map.origin_y);
  if (
    resolution === null ||
    width === null ||
    height === null ||
    originX === null ||
    originY === null ||
    resolution <= 0 ||
    width <= 0 ||
    height <= 0
  ) {
    return null;
  }
  const rx = num(map.robot_x);
  const ry = num(map.robot_y);
  const ryaw = num(map.robot_yaw);
  const robot =
    rx !== null && ry !== null && ryaw !== null ? { x: rx, y: ry, yaw: ryaw } : null;
  const raw = map.png;
  const png = raw instanceof Uint8Array && raw.length > 0 ? raw : null;
  return {
    resolution,
    width: Math.round(width),
    height: Math.round(height),
    originX,
    originY,
    robot,
    tsMs: num(map.ts_ms) ?? 0,
    png
  };
}

/**
 * Трансформ плоскости карты в координатах сцены.
 *
 * Системы координат (те же соглашения, что у лидара — см.
 * lidar_payload.ts: «вперёд робота» = −Z сцены, «влево робота» = −X):
 *
 *   Кадр `map` (REP-103): x — восток решётки, y — север.
 *   Плоскость карты кладём горизонтально через `rotation.x = −π/2`, тогда
 *   её локальный +X идёт в +X сцены, а локальный +Y — в −Z сцены. То есть
 *   без поворота группы «север карты» смотрел бы вперёд оператора.
 *
 * Оператор стоит в точке робота и смотрит туда же, куда робот, поэтому
 * группу надо довернуть так, чтобы вперёд смотрел не север карты, а курс
 * робота: `yaw_scene = π/2 − robot.yaw`.
 *
 * Плоскость центрирована, её центр — середина решётки; смещение центра
 * относительно робота считается в кадре `map` и кладётся в локальные
 * координаты группы (там ось Z ещё «минус север»).
 *
 * Возвращает то, что нужно положить в сцену: поворот группы вокруг Y и
 * позицию плоскости внутри группы.
 */
export function mapPlaneTransform(frame: MapFrame): {
  groupYaw: number;
  planeX: number;
  planeZ: number;
  sizeX: number;
  sizeZ: number;
} | null {
  if (!frame.robot) return null;
  const sizeX = frame.width * frame.resolution;
  const sizeZ = frame.height * frame.resolution;
  // Центр решётки в кадре `map`.
  const centerX = frame.originX + sizeX / 2;
  const centerY = frame.originY + sizeZ / 2;
  return {
    groupYaw: Math.PI / 2 - frame.robot.yaw,
    planeX: centerX - frame.robot.x,
    planeZ: -(centerY - frame.robot.y),
    sizeX,
    sizeZ
  };
}
