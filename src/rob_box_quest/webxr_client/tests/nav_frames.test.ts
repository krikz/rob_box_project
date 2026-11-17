// nav_frames (issue #3151): знаки map ↔ base_link ↔ сцена + сверка с тем,
// как на полу уже лежит карта (mapPlaneTransform).

import { describe, it, expect } from "vitest";
import * as THREE from "three";
import {
  baseLinkToMap,
  baseLinkToScene,
  mapToBaseLink,
  mapToScene,
  mapYawToSceneRotY,
  sceneDirToBaseYaw,
  sceneToBaseLink,
  sceneToMap,
  wrapAngle,
  type Pose2D
} from "../src/nav/nav_frames";
import { mapPlaneTransform, type MapFrame } from "../src/scene/map_payload";

const close = (a: number, b: number) => expect(a).toBeCloseTo(b, 9);

describe("base_link ↔ сцена", () => {
  it("вперёд робота = −Z сцены", () => {
    expect(baseLinkToScene({ x: 1, y: 0 })).toEqual({ x: 0, z: -1 });
  });
  it("влево робота = −X сцены", () => {
    expect(baseLinkToScene({ x: 0, y: 1 })).toEqual({ x: -1, z: 0 });
  });
  it("обратимо", () => {
    const b = { x: 2.5, y: -0.75 };
    expect(sceneToBaseLink(baseLinkToScene(b))).toEqual(b);
  });
});

describe("map → base_link", () => {
  it("робот в начале карты, курс 0: map = base_link", () => {
    const p = mapToBaseLink({ x: 0, y: 0, yaw: 0 }, { x: 3, y: 1 });
    close(p.x, 3);
    close(p.y, 1);
  });
  it("курс +90° (робот смотрит на север): точка на севере — впереди", () => {
    const p = mapToBaseLink({ x: 0, y: 0, yaw: Math.PI / 2 }, { x: 0, y: 2 });
    close(p.x, 2);
    close(p.y, 0);
  });
  it("курс +90°: точка на востоке — справа (y < 0)", () => {
    const p = mapToBaseLink({ x: 0, y: 0, yaw: Math.PI / 2 }, { x: 2, y: 0 });
    close(p.x, 0);
    close(p.y, -2);
  });
  it("смещение робота вычитается", () => {
    const p = mapToBaseLink({ x: 10, y: -4, yaw: 0 }, { x: 11, y: -4 });
    close(p.x, 1);
    close(p.y, 0);
  });
  it("baseLinkToMap — обратное", () => {
    const pose: Pose2D = { x: 3.2, y: -1.1, yaw: -2.3 };
    for (const q of [
      { x: 0, y: 0 },
      { x: 5, y: -7 },
      { x: -2.5, y: 0.25 }
    ]) {
      const back = baseLinkToMap(pose, mapToBaseLink(pose, q));
      close(back.x, q.x);
      close(back.y, q.y);
    }
  });
  it("sceneToMap ∘ mapToScene = id", () => {
    const pose: Pose2D = { x: -6, y: 2, yaw: 0.7 };
    const s = mapToScene(pose, { x: -4, y: 3 });
    const m = sceneToMap(pose, s);
    close(m.x, -4);
    close(m.y, 3);
  });
});

describe("курсы", () => {
  it("направление −Z сцены = курс 0 (вперёд)", () => {
    close(sceneDirToBaseYaw(0, -1), 0);
  });
  it("направление −X сцены = +π/2 (влево)", () => {
    close(sceneDirToBaseYaw(-1, 0), Math.PI / 2);
  });
  it("rotY: объект носом в −Z после поворота смотрит по курсу", () => {
    const pose: Pose2D = { x: 0, y: 0, yaw: 0.4 };
    const mapYaw = 1.4; // на 1 рад левее курса робота
    const obj = new THREE.Object3D();
    obj.rotation.y = mapYawToSceneRotY(pose, mapYaw);
    obj.updateMatrixWorld();
    const nose = new THREE.Vector3(0, 0, -1).applyMatrix4(obj.matrixWorld);
    close(sceneDirToBaseYaw(nose.x, nose.z), 1.0);
  });
  it("wrapAngle", () => {
    close(wrapAngle(3 * Math.PI), Math.PI);
    close(wrapAngle(-Math.PI / 2 - 2 * Math.PI), -Math.PI / 2);
  });
});

describe("сверка с картой на полу (mapPlaneTransform)", () => {
  // Точка карты, положенная через группу пола (поворот группы + сдвиг
  // плоскости), обязана оказаться там же, куда её кладёт mapToScene.
  // Иначе путь Nav2 поедет относительно стен карты.
  const frames: MapFrame[] = [
    { resolution: 0.05, width: 958, height: 744, originX: 5.25, originY: -25.25, robot: { x: 34.5, y: 4.6, yaw: -1.17 }, tsMs: 0, png: null },
    { resolution: 0.1, width: 200, height: 100, originX: -3, originY: 2, robot: { x: 1, y: 4, yaw: 2.9 }, tsMs: 0, png: null }
  ];
  for (const frame of frames) {
    it(`robot ${JSON.stringify(frame.robot)}`, () => {
      const t = mapPlaneTransform(frame)!;
      const group = new THREE.Group();
      group.rotation.y = t.groupYaw;
      const plane = new THREE.Object3D();
      plane.position.set(t.planeX, 0, t.planeZ);
      plane.rotation.x = -Math.PI / 2; // как mapMesh в floor_overlay.ts
      group.add(plane);
      group.updateMatrixWorld(true);
      const centerX = frame.originX + t.sizeX / 2;
      const centerY = frame.originY + t.sizeZ / 2;
      for (const q of [
        { x: frame.robot!.x + 2, y: frame.robot!.y - 1 },
        { x: frame.originX, y: frame.originY },
        { x: centerX, y: centerY + 3 }
      ]) {
        // Локальные координаты плоскости: +X — восток, +Y — север.
        const local = new THREE.Vector3(q.x - centerX, q.y - centerY, 0);
        const world = local.applyMatrix4(plane.matrixWorld);
        const s = mapToScene(frame.robot!, q);
        expect(world.x).toBeCloseTo(s.x, 6);
        expect(world.z).toBeCloseTo(s.z, 6);
      }
    });
  }
});
