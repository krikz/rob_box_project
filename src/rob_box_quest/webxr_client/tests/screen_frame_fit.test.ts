// Посадка рамки экрана вокруг видео главного экрана (bridge_screen.optimized.glb).
//
// Баг, который это ловит: рамку масштабировали равномерно «по ширине 4.8 м»
// и ставили origin'ом (низом модели) в (0, 1.5, −3.95). Модель — портретная
// плита 0.47 × 0.62 × 0.13, так что после ×10.2 она вставала на 6.3 м вверх
// от уровня глаз и на 1.3 м выпирала к оператору, закрывая верхнюю половину
// видео нижней перекладиной.
//
// Два слоя проверки:
//   1. геометрия настоящего GLB (gltf-transform + meshopt, как
//      gltf_pipeline.test.ts) совпадает с долями BRIDGE_SCREEN_FACE — если
//      модель перегенерируют, тест упадёт, а не рамка молча съедет;
//   2. математика fitScreenFrame: полотно = видео-прямоугольник, вся рамка
//      позади плоскости видео и перед стеной комнаты.

import { describe, it, expect, beforeAll } from "vitest";
import { resolve } from "node:path";
import * as THREE from "three";
import { NodeIO } from "@gltf-transform/core";
import { ALL_EXTENSIONS } from "@gltf-transform/extensions";
import { dequantize } from "@gltf-transform/functions";
import { MeshoptDecoder } from "meshoptimizer";
import draco3d from "draco3dgltf";
import { BRIDGE_SCREEN_FACE, fitScreenFrame } from "../src/scene/bridge_assets";

const GLB = resolve(__dirname, "..", "public", "models", "environment", "bridge_screen.optimized.glb");

type Tri = [THREE.Vector3, THREE.Vector3, THREE.Vector3];

async function loadTriangles(): Promise<Tri[]> {
  await MeshoptDecoder.ready;
  const io = new NodeIO()
    .registerExtensions(ALL_EXTENSIONS)
    .registerDependencies({
      "meshopt.decoder": MeshoptDecoder,
      "draco3d.decoder": await draco3d.createDecoderModule()
    });
  const doc = await io.read(GLB);
  await doc.transform(dequantize());
  const tris: Tri[] = [];
  for (const node of doc.getRoot().listNodes()) {
    const mesh = node.getMesh();
    if (!mesh) continue;
    const world = new THREE.Matrix4().fromArray(node.getWorldMatrix() as number[]);
    for (const prim of mesh.listPrimitives()) {
      const pos = prim.getAttribute("POSITION")!;
      const idx = prim.getIndices();
      const pts: THREE.Vector3[] = [];
      const v = [0, 0, 0];
      for (let i = 0; i < pos.getCount(); i += 1) {
        pos.getElement(i, v);
        pts.push(new THREE.Vector3(v[0], v[1], v[2]).applyMatrix4(world));
      }
      const count = idx ? idx.getCount() : pos.getCount();
      for (let i = 0; i < count; i += 3) {
        const a = idx ? idx.getScalar(i) : i;
        const b = idx ? idx.getScalar(i + 1) : i + 1;
        const c = idx ? idx.getScalar(i + 2) : i + 2;
        tris.push([pts[a], pts[b], pts[c]]);
      }
    }
  }
  return tris;
}

describe("bridge_screen.optimized.glb — геометрия полотна", () => {
  let tris: Tri[] = [];
  let box: THREE.Box3;

  beforeAll(async () => {
    tris = await loadTriangles();
    box = new THREE.Box3();
    for (const t of tris) for (const p of t) box.expandByPoint(p);
  });

  it("модель — портретная плита лицом +Z (origin внизу)", () => {
    const size = box.getSize(new THREE.Vector3());
    expect(size.x).toBeCloseTo(0.4703, 3);
    expect(size.y).toBeCloseTo(0.6184, 3);
    expect(size.z).toBeCloseTo(0.1314, 3);
    expect(box.min.y).toBeCloseTo(0, 4);
  });

  it("самая большая +Z-плоскость (полотно) совпадает с BRIDGE_SCREEN_FACE", () => {
    // Треугольники с нормалью +Z, сгруппированные по глубине (5 мм).
    const levels = new Map<number, { area: number; box: THREE.Box3 }>();
    for (const [a, b, c] of tris) {
      const n = new THREE.Vector3().crossVectors(b.clone().sub(a), c.clone().sub(a));
      const area = n.length() / 2;
      if (area === 0 || n.z / n.length() < 0.95) continue;
      const key = Math.round(((a.z + b.z + c.z) / 3) * 200);
      const e = levels.get(key) ?? { area: 0, box: new THREE.Box3() };
      e.area += area;
      e.box.expandByPoint(a).expandByPoint(b).expandByPoint(c);
      levels.set(key, e);
    }
    const face = [...levels.values()].sort((p, q) => q.area - p.area)[0];
    const size = box.getSize(new THREE.Vector3());
    const u0 = (face.box.min.x - box.min.x) / size.x;
    const u1 = (face.box.max.x - box.min.x) / size.x;
    const v0 = (face.box.min.y - box.min.y) / size.y;
    const v1 = (face.box.max.y - box.min.y) / size.y;
    // Допуск 0.005 доли ≈ 2–3 мм модели.
    expect(u0).toBeCloseTo(BRIDGE_SCREEN_FACE.u0, 2);
    expect(u1).toBeCloseTo(BRIDGE_SCREEN_FACE.u1, 2);
    expect(v0).toBeCloseTo(BRIDGE_SCREEN_FACE.v0, 2);
    expect(v1).toBeCloseTo(BRIDGE_SCREEN_FACE.v1, 2);
    // Полотно почти у переднего края: перед ним только кант ≤ 1 см.
    expect(box.max.z - face.box.max.z).toBeLessThan(0.01);
  });

  it("после посадки вся рамка позади видео, полотно накрыто видео, безель виден вокруг", () => {
    const center = { x: 0, y: 1.5, z: -3.9 };
    const fit = fitScreenFrame(box, BRIDGE_SCREEN_FACE, {
      center,
      width: 4.8,
      height: 2.7,
      depth: 0.08,
      gap: 0.01
    })!;
    const m = new THREE.Matrix4().compose(
      new THREE.Vector3(fit.position.x, fit.position.y, fit.position.z),
      new THREE.Quaternion(),
      new THREE.Vector3(fit.scale.x, fit.scale.y, fit.scale.z)
    );
    const world = new THREE.Box3();
    for (const t of tris) for (const p of t) world.expandByPoint(p.clone().applyMatrix4(m));
    // Ничего между оператором и видео.
    expect(world.max.z).toBeLessThanOrEqual(center.z - 0.01 + 1e-6);
    // Перед стеной комнаты (ROOM_D/2 = 4.56).
    expect(world.min.z).toBeGreaterThan(-4.56);
    // Безель выходит за видео со всех сторон и центрирован на нём.
    expect(world.min.x).toBeLessThan(-2.4);
    expect(world.max.x).toBeGreaterThan(2.4);
    expect(world.min.y).toBeLessThan(1.5 - 1.35);
    expect(world.max.y).toBeGreaterThan(1.5 + 1.35);
    expect((world.min.x + world.max.x) / 2).toBeCloseTo(0, 2);
    expect((world.min.y + world.max.y) / 2).toBeCloseTo(1.5, 2);
    // Безель — рамка, а не вторая стена: не больше 0.3 м с каждой стороны.
    expect(-2.4 - world.min.x).toBeLessThan(0.3);
    expect(world.min.y).toBeGreaterThan(1.5 - 1.35 - 0.3);
    expect(world.max.y).toBeLessThan(1.5 + 1.35 + 0.3);
  });
});

describe("fitScreenFrame — математика", () => {
  const face = { u0: 0.1, u1: 0.9, v0: 0.2, v1: 0.8 };
  const box = new THREE.Box3(new THREE.Vector3(-1, 0, 0), new THREE.Vector3(1, 2, 0.5));

  it("полотно ложится ровно на видео-прямоугольник (с overlap)", () => {
    const fit = fitScreenFrame(box, face, {
      center: { x: 1, y: 2, z: -3 },
      width: 4,
      height: 3,
      depth: 0.1,
      gap: 0.02,
      overlap: 0
    })!;
    // полотно в модели: x ∈ [−0.8, 0.8] (1.6), y ∈ [0.4, 1.6] (1.2)
    expect(fit.scale.x).toBeCloseTo(4 / 1.6);
    expect(fit.scale.y).toBeCloseTo(3 / 1.2);
    expect(fit.scale.z).toBeCloseTo(0.1 / 0.5);
    // центр полотна (0, 1.0) → (1, 2)
    expect(fit.position.x + fit.scale.x * 0).toBeCloseTo(1);
    expect(fit.position.y + fit.scale.y * 1.0).toBeCloseTo(2);
    // передний край (z = 0.5) → −3 − gap
    expect(fit.position.z + fit.scale.z * 0.5).toBeCloseTo(-3.02);
  });

  it("overlap по умолчанию 1 %: полотно чуть меньше видео", () => {
    const fit = fitScreenFrame(box, face, {
      center: { x: 0, y: 0, z: 0 },
      width: 4,
      height: 3,
      depth: 0.1,
      gap: 0.01
    })!;
    expect(fit.scale.x * 1.6).toBeCloseTo(4 * 0.99);
    expect(fit.scale.y * 1.2).toBeCloseTo(3 * 0.99);
  });

  it("пустой/вырожденный bbox → null (рамку не трогаем)", () => {
    const target = { center: { x: 0, y: 0, z: 0 }, width: 1, height: 1, depth: 0.1, gap: 0.01 };
    expect(fitScreenFrame(new THREE.Box3(), face, target)).toBeNull();
    const flat = new THREE.Box3(new THREE.Vector3(0, 0, 0), new THREE.Vector3(1, 1, 0));
    expect(fitScreenFrame(flat, face, target)).toBeNull();
  });
});
