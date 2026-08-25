// Парсер LiDAR 2D payload (документация: docs/architecture/meta-quest-api.md §4,
// реализация сервера: src/rob_box_quest/rob_box_quest/streams/lidar.py +
// protocol.topics.encode_lidar_2d).
//
// Формат (little-endian float32):
//   [8 × float32 header: angle_min, angle_max, angle_inc, range_min, range_max,
//                        time_increment, scan_time, n_points]
//   [n_points × float32 ranges]
//   [n_points × float32 intensities]

export interface LidarHeader {
  angle_min: number;
  angle_max: number;
  angle_increment: number;
  range_min: number;
  range_max: number;
  time_increment: number;
  scan_time: number;
  n_points: number;
}

export interface LidarScan {
  header: LidarHeader;
  ranges: Float32Array;
  intensities: Float32Array;
}

const HEADER_FLOATS = 8;
const HEADER_BYTES = HEADER_FLOATS * 4;

export class LidarParseError extends Error {}

export function parseLidar2d(payload: Uint8Array): LidarScan {
  if (payload.length < HEADER_BYTES) {
    throw new LidarParseError(`payload too short for header: ${payload.length} < ${HEADER_BYTES}`);
  }
  const view = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
  const header: LidarHeader = {
    angle_min: view.getFloat32(0, true),
    angle_max: view.getFloat32(4, true),
    angle_increment: view.getFloat32(8, true),
    range_min: view.getFloat32(12, true),
    range_max: view.getFloat32(16, true),
    time_increment: view.getFloat32(20, true),
    scan_time: view.getFloat32(24, true),
    n_points: view.getUint32(28, true)
  };
  const expected = HEADER_BYTES + header.n_points * 4 * 2;
  if (payload.length < expected) {
    throw new LidarParseError(
      `payload truncated: expected ${expected} bytes for ${header.n_points} points, got ${payload.length}`
    );
  }
  const ranges = new Float32Array(payload.buffer, payload.byteOffset + HEADER_BYTES, header.n_points);
  const intensities = new Float32Array(
    payload.buffer,
    payload.byteOffset + HEADER_BYTES + header.n_points * 4,
    header.n_points
  );
  // Копируем (буфер payload'а может быть переиспользован/усечён).
  return {
    header,
    ranges: new Float32Array(ranges),
    intensities: new Float32Array(intensities)
  };
}

// Преобразует LiDAR-scan в пары (x, z) в плоскости пола (y=0).
// Расстояние 'r' под углом 'a' от робота → x = r*cos(a), z = r*sin(a).
// Интенсивность используется как вес (зарезервировано под цвет).
export interface LidarPoint {
  x: number;
  z: number;
  range: number;
  intensity: number;
}

export function scanToFloorPoints(scan: LidarScan): LidarPoint[] {
  const { header, ranges, intensities } = scan;
  const out: LidarPoint[] = [];
  const { angle_min, angle_increment, range_min, range_max } = header;
  for (let i = 0; i < header.n_points; i += 1) {
    const r = ranges[i];
    if (!Number.isFinite(r) || r < range_min || r > range_max) continue;
    const a = angle_min + i * angle_increment;
    out.push({
      x: r * Math.cos(a),
      z: r * Math.sin(a),
      range: r,
      intensity: intensities[i] ?? 0
    });
  }
  return out;
}