import { describe, it, expect } from "vitest";
import { parseMapFrame, mapPlaneTransform } from "../src/scene/map_payload";
import { encodeMsgpackMap } from "../src/wire/msgpack";

// Кодер клиента bin-типы не умеет (см. wire/msgpack.ts), поэтому кадр «с
// PNG» собираем руками: map с четырьмя полями + bin8-значение.
function withPng(base: Uint8Array, png: Uint8Array): Uint8Array {
  // base — msgpack-map, первый байт fixmap (0x80 | n). Добавляем ещё одну
  // пару ключ/значение и правим счётчик.
  const n = base[0] & 0x0f;
  expect(base[0] & 0xf0).toBe(0x80); // fixmap, иначе фикстуру надо чинить
  const key = new Uint8Array([0xa3, 0x70, 0x6e, 0x67]); // fixstr "png"
  const val = new Uint8Array([0xc4, png.length, ...png]); // bin8
  const out = new Uint8Array(base.length + key.length + val.length);
  out.set(base, 0);
  out.set(key, base.length);
  out.set(val, base.length + key.length);
  out[0] = 0x80 | (n + 1);
  return out;
}

function poseFrame(over: Record<string, number> = {}): Uint8Array {
  return encodeMsgpackMap({
    resolution: 0.05,
    width: 958,
    height: 744,
    origin_x: 5.25,
    origin_y: -25.25,
    robot_x: 34.5,
    robot_y: 4.6,
    robot_yaw: -1.17,
    ts_ms: 1700000000000,
    ...over
  });
}

describe("parseMapFrame", () => {
  it("parses a pose-only frame (no png)", () => {
    const f = parseMapFrame(poseFrame());
    expect(f).not.toBeNull();
    expect(f!.width).toBe(958);
    expect(f!.height).toBe(744);
    expect(f!.resolution).toBeCloseTo(0.05, 6);
    expect(f!.png).toBeNull();
    expect(f!.robot).toEqual({ x: 34.5, y: 4.6, yaw: -1.17 });
  });

  it("parses a full frame with png bytes", () => {
    const png = new Uint8Array([0x89, 0x50, 0x4e, 0x47, 1, 2, 3]);
    const f = parseMapFrame(withPng(poseFrame(), png));
    expect(f).not.toBeNull();
    expect(f!.png).toEqual(png);
  });

  it("keeps robot null when the server had no tf pose", () => {
    // Сервер кладёт явный null, а не 0.0 — «позы нет» должно отличаться от
    // «робот в начале карты».
    const raw = encodeMsgpackMap({
      resolution: 0.05,
      width: 10,
      height: 10,
      origin_x: 0,
      origin_y: 0,
      robot_x: null,
      robot_y: null,
      robot_yaw: null,
      ts_ms: 1
    });
    const f = parseMapFrame(raw);
    expect(f).not.toBeNull();
    expect(f!.robot).toBeNull();
  });

  it("rejects a frame with a degenerate grid", () => {
    expect(parseMapFrame(poseFrame({ width: 0 }))).toBeNull();
    expect(parseMapFrame(poseFrame({ resolution: 0 }))).toBeNull();
  });

  it("rejects garbage without throwing", () => {
    expect(parseMapFrame(new Uint8Array([0xff, 0xff, 0xff]))).toBeNull();
    expect(parseMapFrame(new Uint8Array(0))).toBeNull();
  });
});

describe("mapPlaneTransform", () => {
  it("has no transform without a robot pose", () => {
    const f = parseMapFrame(
      encodeMsgpackMap({
        resolution: 0.05,
        width: 10,
        height: 10,
        origin_x: 0,
        origin_y: 0,
        robot_x: null,
        robot_y: null,
        robot_yaw: null,
        ts_ms: 1
      })
    );
    expect(mapPlaneTransform(f!)).toBeNull();
  });

  it("sizes the plane in metres", () => {
    const t = mapPlaneTransform(parseMapFrame(poseFrame())!)!;
    expect(t.sizeX).toBeCloseTo(958 * 0.05, 6);
    expect(t.sizeZ).toBeCloseTo(744 * 0.05, 6);
  });

  it("puts the robot at the scene origin", () => {
    // Робот ровно в центре решётки → центр плоскости совпадает с ним,
    // значит смещение нулевое.
    const f = parseMapFrame(
      poseFrame({
        origin_x: 0,
        origin_y: 0,
        width: 100,
        height: 100,
        robot_x: 2.5, // 100 × 0.05 / 2
        robot_y: 2.5,
        robot_yaw: 0
      })
    )!;
    const t = mapPlaneTransform(f)!;
    expect(t.planeX).toBeCloseTo(0, 6);
    expect(t.planeZ).toBeCloseTo(0, 6);
  });

  // Точка карты p переносится в сцену так: сдвиг относительно робота в
  // локальных осях группы (x = mx, z = −my), затем поворот группы вокруг Y.
  function toScene(
    t: { groupYaw: number },
    frame: { robot: { x: number; y: number } },
    p: { x: number; y: number }
  ): { x: number; z: number } {
    const lx = p.x - frame.robot.x;
    const lz = -(p.y - frame.robot.y);
    const c = Math.cos(t.groupYaw);
    const s = Math.sin(t.groupYaw);
    return { x: lx * c + lz * s, z: -lx * s + lz * c };
  }

  it("points the robot heading down −Z, whatever the yaw", () => {
    // −Z сцены — это «вперёд оператора», туда же смотрит фронтальная
    // камера на экране-стене и туда же лидар кладёт «перед робота».
    for (const yaw of [0, 0.7, -1.17, Math.PI, -2.9]) {
      const f = parseMapFrame(poseFrame({ robot_yaw: yaw }))!;
      const t = mapPlaneTransform(f)!;
      // Точка в метре прямо по курсу робота.
      const ahead = {
        x: f.robot!.x + Math.cos(yaw),
        y: f.robot!.y + Math.sin(yaw)
      };
      const s = toScene(t, { robot: f.robot! }, ahead);
      expect(s.x).toBeCloseTo(0, 6);
      expect(s.z).toBeCloseTo(-1, 6);
    }
  });

  it("puts the robot's left at −X, matching the lidar convention", () => {
    const yaw = 0.7;
    const f = parseMapFrame(poseFrame({ robot_yaw: yaw }))!;
    const t = mapPlaneTransform(f)!;
    const left = {
      x: f.robot!.x - Math.sin(yaw),
      y: f.robot!.y + Math.cos(yaw)
    };
    const s = toScene(t, { robot: f.robot! }, left);
    expect(s.x).toBeCloseTo(-1, 6);
    expect(s.z).toBeCloseTo(0, 6);
  });
});
