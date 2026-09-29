// Энкодеры симулятора мостика (#3149) обязаны давать байты, которые
// разбирают НАСТОЯЩИЕ клиентские декодеры — иначе симулятор врёт о том,
// что мостик работает.

import { describe, it, expect } from "vitest";
import { inflateSync } from "node:zlib";
import {
  encodeLidar2d,
  encodeMap2d,
  encodePngRgba,
  encodeRobotStatus,
  encodeSupervisorState,
  encodeVoiceState,
  gridToPng,
  MAP_FREE_RGBA,
  MAP_OCCUPIED_RGBA
} from "../src/dev/sim_encoders";
import { parseLidar2d, scanToFloorPoints } from "../src/scene/lidar_payload";
import { mapPlaneTransform, parseMapFrame } from "../src/scene/map_payload";
import { parseRobotStatus } from "../src/scene/status_hud";
import { parseVoiceState } from "../src/ui/voice_state_indicator";
import { parseSupervisorState } from "../src/state/supervisor_state";
import { decodeMsgpackMap, encodeMsgpackMap } from "../src/wire/msgpack";
import { createDefaultWorld, createGrid, scanLidar } from "../src/dev/sim_world";

/** Минимальный PNG-ридер для теста: IHDR + склейка IDAT + inflate. */
function readPng(png: Uint8Array): { width: number; height: number; rgba: Uint8Array } {
  const sig = [0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a];
  expect(Array.from(png.subarray(0, 8))).toEqual(sig);
  const view = new DataView(png.buffer, png.byteOffset, png.byteLength);
  let off = 8;
  let width = 0;
  let height = 0;
  const idat: Uint8Array[] = [];
  while (off < png.length) {
    const len = view.getUint32(off);
    const type = String.fromCharCode(...png.subarray(off + 4, off + 8));
    const data = png.subarray(off + 8, off + 8 + len);
    if (type === "IHDR") {
      width = view.getUint32(off + 8);
      height = view.getUint32(off + 12);
      expect(data[8]).toBe(8);
      expect(data[9]).toBe(6);
    } else if (type === "IDAT") {
      idat.push(data);
    }
    off += 12 + len;
  }
  const raw = new Uint8Array(inflateSync(Buffer.concat(idat.map((d) => Buffer.from(d)))));
  const rgba = new Uint8Array(width * height * 4);
  for (let y = 0; y < height; y += 1) {
    expect(raw[y * (width * 4 + 1)]).toBe(0);
    rgba.set(raw.subarray(y * (width * 4 + 1) + 1, (y + 1) * (width * 4 + 1)), y * width * 4);
  }
  return { width, height, rgba };
}

describe("sim encoders ↔ real client decoders", () => {
  it("lidar_2d round-trips through parseLidar2d (n_points as float32)", () => {
    const world = createDefaultWorld();
    const scan = scanLidar(world, world.start, () => 0.5);
    const bytes = encodeLidar2d(scan);
    const parsed = parseLidar2d(bytes);
    expect(parsed.header.n_points).toBe(360);
    expect(parsed.header.angle_min).toBeCloseTo(-Math.PI, 5);
    expect(parsed.header.range_max).toBe(12);
    expect(parsed.ranges[10]).toBeCloseTo(scan.ranges[10], 4);
    // Лучи в стену.
    // Промахи (коридор длиннее range_max) приходят как Infinity и отбрасываются клиентом.
    const finite = Array.from(scan.ranges).filter((r) => Number.isFinite(r)).length;
    expect(finite).toBeGreaterThan(300);
    expect(scanToFloorPoints(parsed).length).toBe(finite);
  });

  it("lidar: the ray straight ahead lands in front of the operator (−Z)", () => {
    const world = createDefaultWorld();
    const pose = { x: 4, y: 1, yaw: 0 }; // на восток, стена x=8 в 4 м (коридор — выше, y 2..4)
    const scan = scanLidar(world, pose, () => 0.5);
    const pts = scanToFloorPoints(parseLidar2d(encodeLidar2d(scan)));
    const front = pts.filter((p) => p.z < 0);
    const ahead = front.reduce((best, p) => (Math.abs(p.x) < Math.abs(best.x) ? p : best));
    expect(ahead.z).toBeCloseTo(-4, 1);
  });

  it("PNG encoder produces a valid RGBA PNG (inflate + filter bytes)", () => {
    const rgba = new Uint8Array(3 * 2 * 4).map((_, i) => i * 7);
    const png = encodePngRgba(3, 2, rgba);
    const back = readPng(png);
    expect(back.width).toBe(3);
    expect(back.height).toBe(2);
    expect(Array.from(back.rgba)).toEqual(Array.from(rgba));
  });

  it("PNG encoder handles multi-block payloads (>65535 bytes)", () => {
    const w = 200;
    const h = 100; // 80 100 байт сырых строк → два stored-блока
    const rgba = new Uint8Array(w * h * 4).map((_, i) => i & 0xff);
    expect(Array.from(readPng(encodePngRgba(w, h, rgba)).rgba.subarray(0, 64))).toEqual(
      Array.from(rgba.subarray(0, 64))
    );
  });

  it("gridToPng: server palette, unknown transparent, rows flipped", () => {
    const g = { width: 2, height: 2, data: Int8Array.from([100, -1, 0, -1]) };
    const { rgba } = readPng(gridToPng(g));
    // data[0] (низ-лево) → нижняя строка PNG → rgba со смещения 8.
    expect(Array.from(rgba.subarray(8, 12))).toEqual([...MAP_OCCUPIED_RGBA]);
    expect(Array.from(rgba.subarray(0, 4))).toEqual([...MAP_FREE_RGBA]);
    expect(rgba[7]).toBe(0);
    expect(rgba[15]).toBe(0);
  });

  it("map_2d full frame round-trips through parseMapFrame + mapPlaneTransform", () => {
    const world = createDefaultWorld();
    const grid = createGrid(world);
    const png = gridToPng(grid);
    const bytes = encodeMap2d({ grid, robot: { x: 1.5, y: 2.25, yaw: 0.5 }, tsMs: 1234, png });
    const f = parseMapFrame(bytes);
    expect(f).not.toBeNull();
    expect(f!.resolution).toBeCloseTo(0.05);
    expect(f!.width).toBe(grid.width);
    expect(f!.originX).toBe(-1);
    expect(f!.robot).toEqual({ x: 1.5, y: 2.25, yaw: 0.5 });
    expect(f!.png).toEqual(png);
    expect(mapPlaneTransform(f!)!.groupYaw).toBeCloseTo(Math.PI / 2 - 0.5);
  });

  it("map_2d light frame has no png; null pose stays null", () => {
    const grid = createGrid(createDefaultWorld());
    const light = parseMapFrame(encodeMap2d({ grid, robot: { x: 0, y: 0, yaw: 0 }, tsMs: 1, png: null }));
    expect(light!.png).toBeNull();
    const noPose = parseMapFrame(encodeMap2d({ grid, robot: null, tsMs: 1, png: null }));
    expect(noPose!.robot).toBeNull();
  });

  it("robot_status round-trips through parseRobotStatus", () => {
    const s = parseRobotStatus(
      encodeRobotStatus({
        batteryPct: 86.6,
        batteryV: 24.8,
        wifiRssi: -52.4,
        mode: "teleop",
        velLinear: 0.25,
        velAngular: -0.5,
        tsMs: 1700000000123
      })
    );
    expect(s).toEqual({
      battery_pct: 87,
      battery_v: 24.8,
      wifi_rssi: -52,
      mode: "teleop",
      vel_linear: 0.25,
      vel_angular: -0.5,
      ts_ms: 1700000000123
    });
  });

  it("voice_state round-trips through parseVoiceState", () => {
    const f = parseVoiceState(encodeVoiceState("speaking", 42, "silenced"));
    expect(f).toEqual({ state: "speaking", detail: "silenced", tsMs: 42 });
  });

  it("STATE_UPDATE round-trips through parseSupervisorState", () => {
    const bytes = encodeSupervisorState({
      mode: "avatar_present",
      teleopHolder: "sim-abc",
      voiceHolder: null,
      sinceMs: 1000,
      lastEvent: "sim",
      version: 3
    });
    const st = parseSupervisorState(decodeMsgpackMap(bytes));
    expect(st).toEqual({
      mode: "avatar_present",
      teleopFloor: { clientId: "sim-abc", sinceMs: 1000 },
      voiceFloor: { clientId: null, sinceMs: 0 },
      updatedMs: 1000
    });
  });

  it("msgpack encoder writes bin 8/16/32 that the decoder reads back", () => {
    for (const n of [0, 5, 300, 70_000]) {
      const bin = new Uint8Array(n).map((_, i) => i & 0xff);
      const back = decodeMsgpackMap(encodeMsgpackMap({ b: bin }));
      expect(back!.b).toEqual(bin);
    }
  });
});
