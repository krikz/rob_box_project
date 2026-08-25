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
  view.setUint32(28, n, true);
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

  it("throws LidarParseError on too-short payload", () => {
    const tooShort = new Uint8Array(16);
    expect(() => parseLidar2d(tooShort)).toThrow(LidarParseError);
  });

  it("throws LidarParseError on truncated payload", () => {
    // header says n_points=10, but payload has only 4 ranges.
    const buf = new ArrayBuffer(32 + 4 * 4 * 2);
    const view = new DataView(buf);
    view.setUint32(28, 10, true);
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

  it("scanToFloorPoints computes (x,z) for forward ray", () => {
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
    expect(pts[0].x).toBeCloseTo(3.0, 5); // cos(0) = 1
    expect(pts[0].z).toBeCloseTo(0, 5); // sin(0) = 0
  });
});