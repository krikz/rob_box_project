// Парсер nav_path payload (0x1104, issue #3151). Сервер:
// rob_box_quest/streams/nav_path.py:encode_nav_path.
//
// MessagePack `{frame: "map", n, xy: bin, ts_ms}`; `xy` — n пар
// little-endian float32 [x0, y0, x1, y1, …] в кадре `map`. `n = 0` — путь
// погас (цель завершилась).

import { decodeMsgpackMap } from "../wire/msgpack";
import type { Xy } from "./nav_frames";

export interface NavPathFrame {
  frame: string;
  /** Точки пути в `map`; пустой массив — «пути нет». */
  points: Xy[];
  tsMs: number;
}

/** `null` — кадр битый или не в `map` (рисовать его некуда). */
export function parseNavPath(payload: Uint8Array): NavPathFrame | null {
  const map = decodeMsgpackMap(payload);
  if (!map) return null;
  const frame = typeof map.frame === "string" ? map.frame : "";
  if (frame !== "map") return null;
  const n = typeof map.n === "number" && Number.isInteger(map.n) && map.n >= 0 ? map.n : null;
  const xy = map.xy instanceof Uint8Array ? map.xy : n === 0 ? new Uint8Array(0) : null;
  if (n === null || xy === null || xy.length !== n * 8) return null;
  // DataView, а не Float32Array поверх буфера: bin внутри msgpack не
  // выровнен на 4 байта, а endianness на Quest и так LE — но явно.
  const view = new DataView(xy.buffer, xy.byteOffset, xy.byteLength);
  const points: Xy[] = [];
  for (let i = 0; i < n; i += 1) {
    const x = view.getFloat32(i * 8, true);
    const y = view.getFloat32(i * 8 + 4, true);
    if (!Number.isFinite(x) || !Number.isFinite(y)) return null;
    points.push({ x, y });
  }
  const tsMs = typeof map.ts_ms === "number" ? map.ts_ms : 0;
  return { frame, points, tsMs };
}
