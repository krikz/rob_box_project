// Энкодеры стримов симулятора (issue #3149) — зеркала серверных:
//
//   lidar_2d     → rob_box_quest/protocol/topics.py: encode_lidar_2d
//   map_2d       → rob_box_quest/streams/occupancy.py: encode_map_2d + grid_to_png
//   robot_status → rob_box_quest/protocol/topics.py: encode_robot_status
//   voice_state  → rob_box_quest/protocol/topics.py: encode_voice_state
//   STATE_UPDATE → rob_box_supervisor/core/state.py: pack (плоская форма)
//
// Каждый энкодер покрыт тестом, который гоняет байты через НАСТОЯЩИЙ
// клиентский декодер (tests/sim_encoders.test.ts): если формат сервера
// и клиента разъедется, симулятор не должен молча это маскировать.

import { encodeMsgpackMap, type MsgpackInput } from "../wire/msgpack";
import type { SimGrid, SimScan } from "./sim_world";

// ────────────────────────── lidar_2d ──────────────────────────

/** 8 × float32 LE заголовок + ranges + intensities (n — тоже float32!). */
export function encodeLidar2d(scan: SimScan): Uint8Array {
  const n = scan.ranges.length;
  if (scan.intensities.length !== n) {
    throw new RangeError(`ranges (${n}) and intensities (${scan.intensities.length}) length mismatch`);
  }
  const out = new Uint8Array(32 + n * 8);
  const view = new DataView(out.buffer);
  const header = [
    scan.angleMin,
    scan.angleMax,
    scan.angleIncrement,
    scan.rangeMin,
    scan.rangeMax,
    0, // time_increment
    scan.scanTime,
    n
  ];
  header.forEach((v, i) => view.setFloat32(i * 4, v, true));
  for (let i = 0; i < n; i += 1) {
    view.setFloat32(32 + i * 4, scan.ranges[i], true);
    view.setFloat32(32 + n * 4 + i * 4, scan.intensities[i], true);
  }
  return out;
}

// ────────────────────────── PNG (без зависимостей) ──────────────────────────

const CRC_TABLE = (() => {
  const t = new Uint32Array(256);
  for (let n = 0; n < 256; n += 1) {
    let c = n;
    for (let k = 0; k < 8; k += 1) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
    t[n] = c >>> 0;
  }
  return t;
})();

function crc32(bytes: Uint8Array, start: number, end: number): number {
  let c = 0xffffffff;
  for (let i = start; i < end; i += 1) c = CRC_TABLE[(c ^ bytes[i]) & 0xff] ^ (c >>> 8);
  return (c ^ 0xffffffff) >>> 0;
}

/**
 * Валидный RGBA PNG из сырых пикселей (строки сверху вниз). Deflate —
 * stored-блоки без сжатия: симулятор живёт в той же вкладке, байты по
 * сети не ходят, а свой inflate-совместимый компрессор тут был бы лишним
 * кодом. Браузерный `Image` такой PNG декодирует как любой другой.
 */
export function encodePngRgba(width: number, height: number, rgba: Uint8Array): Uint8Array {
  if (rgba.length !== width * height * 4) {
    throw new RangeError(`rgba length ${rgba.length} != ${width}×${height}×4`);
  }
  // Сырые строки с filter-байтом 0.
  const rowLen = width * 4 + 1;
  const raw = new Uint8Array(rowLen * height);
  for (let y = 0; y < height; y += 1) {
    raw[y * rowLen] = 0;
    raw.set(rgba.subarray(y * width * 4, (y + 1) * width * 4), y * rowLen + 1);
  }
  // zlib: заголовок + stored-блоки по ≤ 65535 байт + adler32.
  const nBlocks = Math.max(1, Math.ceil(raw.length / 65535));
  const zlib = new Uint8Array(2 + raw.length + nBlocks * 5 + 4);
  let o = 0;
  zlib[o++] = 0x78;
  zlib[o++] = 0x01;
  for (let b = 0; b < nBlocks; b += 1) {
    const start = b * 65535;
    const len = Math.min(65535, raw.length - start);
    zlib[o++] = b === nBlocks - 1 ? 1 : 0;
    zlib[o++] = len & 0xff;
    zlib[o++] = (len >>> 8) & 0xff;
    zlib[o++] = ~len & 0xff;
    zlib[o++] = (~len >>> 8) & 0xff;
    zlib.set(raw.subarray(start, start + len), o);
    o += len;
  }
  let a = 1;
  let s2 = 0;
  for (let i = 0; i < raw.length; i += 1) {
    a = (a + raw[i]) % 65521;
    s2 = (s2 + a) % 65521;
  }
  const adler = ((s2 << 16) | a) >>> 0;
  zlib[o++] = (adler >>> 24) & 0xff;
  zlib[o++] = (adler >>> 16) & 0xff;
  zlib[o++] = (adler >>> 8) & 0xff;
  zlib[o++] = adler & 0xff;

  const chunk = (type: string, data: Uint8Array): Uint8Array => {
    const c = new Uint8Array(12 + data.length);
    const v = new DataView(c.buffer);
    v.setUint32(0, data.length);
    for (let i = 0; i < 4; i += 1) c[4 + i] = type.charCodeAt(i);
    c.set(data, 8);
    v.setUint32(8 + data.length, crc32(c, 4, 8 + data.length));
    return c;
  };
  const ihdr = new Uint8Array(13);
  const iv = new DataView(ihdr.buffer);
  iv.setUint32(0, width);
  iv.setUint32(4, height);
  ihdr[8] = 8; // bit depth
  ihdr[9] = 6; // RGBA
  const parts = [
    new Uint8Array([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a]),
    chunk("IHDR", ihdr),
    chunk("IDAT", zlib),
    chunk("IEND", new Uint8Array(0))
  ];
  const total = parts.reduce((n, p) => n + p.length, 0);
  const png = new Uint8Array(total);
  let off = 0;
  for (const p of parts) {
    png.set(p, off);
    off += p.length;
  }
  return png;
}

// ────────────────────────── map_2d ──────────────────────────

// Палитра — ровно серверная (streams/occupancy.py).
export const MAP_FREE_RGBA = [0x1b, 0x4a, 0x63, 105] as const;
export const MAP_OCCUPIED_RGBA = [0x44, 0xdd, 0xff, 255] as const;
const OCCUPIED_THRESHOLD = 50;

/**
 * OccupancyGrid → RGBA PNG, как `grid_to_png` сервера: unknown прозрачный,
 * строки перевёрнуты (нулевая строка PNG — северный край решётки).
 */
export function gridToPng(g: Pick<SimGrid, "width" | "height" | "data">): Uint8Array {
  const { width, height, data } = g;
  const rgba = new Uint8Array(width * height * 4);
  for (let y = 0; y < height; y += 1) {
    const dstRow = height - 1 - y; // flipud
    for (let x = 0; x < width; x += 1) {
      const v = data[y * width + x];
      if (v < 0) continue;
      const c = v >= OCCUPIED_THRESHOLD ? MAP_OCCUPIED_RGBA : MAP_FREE_RGBA;
      const o = (dstRow * width + x) * 4;
      rgba[o] = c[0];
      rgba[o + 1] = c[1];
      rgba[o + 2] = c[2];
      rgba[o + 3] = c[3];
    }
  }
  return encodePngRgba(width, height, rgba);
}

export interface MapFrameInput {
  grid: Pick<SimGrid, "resolution" | "width" | "height" | "originX" | "originY">;
  robot: { x: number; y: number; yaw: number } | null;
  tsMs: number;
  /** `null` — лёгкий кадр «только поза». */
  png: Uint8Array | null;
}

/**
 * msgpack-map map_2d (encode_map_2d). Сервер пишет позу как float64; JS-
 * энкодер целое значение (скажем, origin_x = -1) кодирует int-тэгом —
 * декодеру клиента это безразлично, number остаётся number.
 */
export function encodeMap2d(f: MapFrameInput): Uint8Array {
  const payload: { [k: string]: MsgpackInput } = {
    resolution: f.grid.resolution,
    width: f.grid.width,
    height: f.grid.height,
    origin_x: f.grid.originX,
    origin_y: f.grid.originY,
    robot_x: f.robot ? f.robot.x : null,
    robot_y: f.robot ? f.robot.y : null,
    robot_yaw: f.robot ? f.robot.yaw : null,
    ts_ms: Math.round(f.tsMs)
  };
  if (f.png) payload.png = f.png;
  return encodeMsgpackMap(payload);
}

// ────────────────────────── robot_status / voice_state ──────────────────────────

export interface RobotStatusInput {
  batteryPct: number;
  batteryV: number | null;
  wifiRssi: number;
  mode: string;
  velLinear: number;
  velAngular: number;
  tsMs: number;
}

export function encodeRobotStatus(s: RobotStatusInput): Uint8Array {
  return encodeMsgpackMap({
    battery_pct: Math.round(s.batteryPct),
    battery_v: s.batteryV,
    wifi_rssi: Math.round(s.wifiRssi),
    mode: s.mode,
    vel_linear: s.velLinear,
    vel_angular: s.velAngular,
    ts_ms: Math.round(s.tsMs)
  });
}

export function encodeVoiceState(state: string, tsMs: number, detail?: string): Uint8Array {
  const body: { [k: string]: MsgpackInput } = { state, ts_ms: Math.round(tsMs) };
  if (detail) body.detail = detail;
  return encodeMsgpackMap(body);
}

// ────────────────────────── STATE_UPDATE (supervisor) ──────────────────────────

export interface SupervisorStateInput {
  mode: string;
  teleopHolder: string | null;
  voiceHolder: string | null;
  sinceMs: number;
  lastEvent: string;
  version: number;
}

export function encodeSupervisorState(s: SupervisorStateInput): Uint8Array {
  const floor = (holder: string | null): MsgpackInput =>
    holder === null
      ? null
      : { client_id: holder, since_ms: Math.round(s.sinceMs), last_heartbeat_ms: Math.round(s.sinceMs) };
  return encodeMsgpackMap({
    mode: s.mode,
    teleop_floor: floor(s.teleopHolder),
    voice_floor: floor(s.voiceHolder),
    last_event: s.lastEvent,
    since_ms: Math.round(s.sinceMs),
    version: s.version
  });
}
