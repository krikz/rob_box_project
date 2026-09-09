import { describe, it, expect } from "vitest";
import { parseLidar2d, scanToFloorPoints, LidarParseError } from "../src/scene/lidar_payload";

function buildScan(opts: {
  angle_min: number;
  angle_max: number;
  angle_inc: number;
  range_min: number;
  range_max: number;
  time_inc: number;
  scan_time: number;
  ranges: number[];
  intensities?: number[];
}): Uint8Array {
  const n = opts.ranges.length;
  const intensities = opts.intensities ?? new Array(n).fill(0.5);
  const total = 32 + n * 4 * 2;
  const buf = new ArrayBuffer(total);
  const view = new DataView(buf);
  view.setFloat32(0, opts.angle_min, true);
  view.setFloat32(4, opts.angle_max, true);
  view.setFloat32(8, opts.angle_inc, true);
  view.setFloat32(12, opts.range_min, true);
  view.setFloat32(16, opts.range_max, true);
  view.setFloat32(20, opts.time_inc, true);
  view.setFloat32(24, opts.scan_time, true);
  // n_points — float32, как его пишет сервер (protocol/topics.py:
  // _LIDAR_HEADER_FMT = "<ffffffff"). Фикстура раньше клала сюда uint32 и
  // тем самым повторяла баг парсера, из-за чего он не ловился тестами.
  view.setFloat32(28, n, true);
  for (let i = 0; i < n; i += 1) {
    view.setFloat32(32 + i * 4, opts.ranges[i], true);
    view.setFloat32(32 + n * 4 + i * 4, intensities[i], true);
  }
  return new Uint8Array(buf);
}

describe("parseLidar2d", () => {
  it("parses header + ranges + intensities", () => {
    const scan = buildScan({
      angle_min: -Math.PI,
      angle_max: Math.PI,
      angle_inc: Math.PI / 2,
      range_min: 0.05,
      range_max: 30,
      time_inc: 0,
      scan_time: 0.1,
      ranges: [1.0, 2.0, 3.0, 4.0],
      intensities: [0.1, 0.2, 0.3, 0.4]
    });
    const out = parseLidar2d(scan);
    expect(out.header.n_points).toBe(4);
    expect(out.header.angle_min).toBeCloseTo(-Math.PI);
    expect(Array.from(out.ranges)).toEqual([1.0, 2.0, 3.0, 4.0]);
    // intensities хранятся во float32, поэтому сравниваем с toBeCloseTo.
    expect(Array.from(out.intensities)).toHaveLength(4);
    expect(out.intensities[0]).toBeCloseTo(0.1, 5);
    expect(out.intensities[1]).toBeCloseTo(0.2, 5);
    expect(out.intensities[2]).toBeCloseTo(0.3, 5);
    expect(out.intensities[3]).toBeCloseTo(0.4, 5);
  });

  // Регрессия: байты, снятые с реального сервера (struct.pack("<ffffffff", ...)
  // + ranges + intensities, n=3). Клиент читал n_points как uint32 и на этом
  // payload'е получал 1 077 936 128 точек → LidarParseError → лидар не
  // рисовался. Фикстура захардкожена намеренно: она ловит рассинхрон
  // клиента с protocol/topics.py даже если хелпер buildScan снова уедет.
  it("parses bytes produced by the Python server (n_points as float32)", () => {
    const raw = new Uint8Array([
      249, 15, 73, 192, 249, 15, 73, 64, 41, 92, 143, 60, 205, 204, 204, 61,
      0, 0, 32, 65, 0, 0, 0, 0, 205, 204, 204, 61, 0, 0, 64, 64,
      0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 64, 64,
      0, 0, 0, 63, 0, 0, 0, 63, 0, 0, 0, 63
    ]);
    const out = parseLidar2d(raw);
    expect(out.header.n_points).toBe(3);
    expect(out.header.range_max).toBeCloseTo(10.0, 5);
    expect(Array.from(out.ranges)).toEqual([1.0, 2.0, 3.0]);
    expect(scanToFloorPoints(out)).toHaveLength(3);
  });

  it("throws LidarParseError on too-short payload", () => {
    const tooShort = new Uint8Array(16);
    expect(() => parseLidar2d(tooShort)).toThrow(LidarParseError);
  });

  it("throws LidarParseError on truncated payload", () => {
    // header says n_points=10, but payload has only 4 ranges.
    const buf = new ArrayBuffer(32 + 4 * 4 * 2);
    const view = new DataView(buf);
    view.setFloat32(28, 10, true);
    expect(() => parseLidar2d(new Uint8Array(buf))).toThrow(/truncated/);
  });

  it("scanToFloorPoints filters invalid ranges", () => {
    const scan = parseLidar2d(
      buildScan({
        angle_min: 0,
        angle_max: Math.PI,
        angle_inc: Math.PI / 2,
        range_min: 0.1,
        range_max: 10,
        time_inc: 0,
        scan_time: 0.1,
        ranges: [0.05, 1.0, 20.0, 2.0], // 0.05 < min, 20 > max
        intensities: [0.1, 0.2, 0.3, 0.4]
      })
    );
    const pts = scanToFloorPoints(scan);
    expect(pts.length).toBe(2);
    expect(pts[0].range).toBeCloseTo(1.0);
    expect(pts[1].range).toBeCloseTo(2.0);
  });

  it("maps the forward ray to scene −Z (in front of the operator)", () => {
    const scan = parseLidar2d(
      buildScan({
        angle_min: 0,
        angle_max: 0,
        angle_inc: 1,
        range_min: 0,
        range_max: 100,
        time_inc: 0,
        scan_time: 0,
        ranges: [3.0],
        intensities: [0.5]
      })
    );
    const pts = scanToFloorPoints(scan);
    expect(pts.length).toBe(1);
    // ROS a=0 — прямо перед роботом. В сцене «вперёд» = −Z (там экран-стена),
    // поэтому луч 3 м должен лечь в (0, −3), а не в (3, 0) вправо от оператора.
    expect(pts[0].x).toBeCloseTo(0, 5);
    expect(pts[0].z).toBeCloseTo(-3.0, 5);
  });

  it("maps the left ray to scene −X (to the operator's left)", () => {
    const scan = parseLidar2d(
      buildScan({
        angle_min: Math.PI / 2, // +90° по REP-103 — влево от робота
        angle_max: Math.PI / 2,
        angle_inc: 1,
        range_min: 0,
        range_max: 100,
        time_inc: 0,
        scan_time: 0,
        ranges: [2.0],
        intensities: [0.5]
      })
    );
    const pts = scanToFloorPoints(scan);
    expect(pts[0].x).toBeCloseTo(-2.0, 5);
    expect(pts[0].z).toBeCloseTo(0, 5);
  });

  it("maps the right ray to scene +X", () => {
    const scan = parseLidar2d(
      buildScan({
        angle_min: -Math.PI / 2,
        angle_max: -Math.PI / 2,
        angle_inc: 1,
        range_min: 0,
        range_max: 100,
        time_inc: 0,
        scan_time: 0,
        ranges: [2.0],
        intensities: [0.5]
      })
    );
    const pts = scanToFloorPoints(scan);
    expect(pts[0].x).toBeCloseTo(2.0, 5);
    expect(pts[0].z).toBeCloseTo(0, 5);
  });
});
// --- Цветовая шкала оверлея (видимость на тёмном фоне мостика) ---

describe("rangeColor", () => {
  it("paints near hits red", async () => {
    const { rangeColor } = await import("../src/scene/lidar_overlay");
    const [r, g, b] = rangeColor(0, 10);
    expect(r).toBeGreaterThan(0.8);
    expect(g).toBeLessThan(0.3);
    expect(b).toBeLessThan(0.3);
  });

  it("paints far hits green, not near-black blue", async () => {
    const { rangeColor } = await import("../src/scene/lidar_overlay");
    const [r, g, b] = rangeColor(10, 10);
    expect(g).toBeGreaterThan(0.8);
    expect(r).toBeLessThan(0.3);
    // Ключевое: прежняя шкала уходила в чистый синий, невидимый на #0a0d11.
    expect(b).toBeLessThan(0.3);
  });

  it("clamps beyond max range", async () => {
    const { rangeColor } = await import("../src/scene/lidar_overlay");
    expect(rangeColor(50, 10)).toEqual(rangeColor(10, 10));
  });
});
