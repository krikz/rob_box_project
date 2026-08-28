// src/scene/hand_visual.ts
//
// Phase 2.2 — визуализация hand-tracking на мостике.
//
// Использует XRHandModelFactory из three/examples (profile="spheres"),
// который рендерит 25 joint-spheres на каждой руке. Не требует внешних
// GLB-ассетов — работает на Quest 3 / Quest Pro «из коробки».
//
// Скелет:
//   - 25 маленьких сфер (wrist + 5 пальцев × 4 phalanges + tip),
//   - "bones" — линии между последовательными joint-spheres,
//   - подсветка pinch (thumb-tip ↔ index-tip, < threshold) — жёлтый pulse,
//   - подсветка grip (кулак) — зелёный glow.
//
// Что НЕ делает:
//   - не отправляет WSS cmds (это hand_teleop.ts),
//   - не хранит state между XR-кадрами (per-frame rebuild bones).

import * as THREE from "three";
import { XRHandModelFactory } from "three/examples/jsm/webxr/XRHandModelFactory.js";
import type { PosePosition } from "../input/hand_teleop";
import { PINCH_THRESHOLD_M, isPinching, isGripping } from "../input/hand_teleop";

export interface HandVisualOptions {
  /** Цвет idle (open hand). */
  idleColor?: number;
  /** Цвет во время pinch (select). */
  pinchColor?: number;
  /** Цвет во время grip (fist). */
  gripColor?: number;
}

const DEFAULT_OPTIONS: Required<HandVisualOptions> = {
  idleColor: 0x6b7a8f,
  pinchColor: 0xf6d32d,
  gripColor: 0x2ec27e
};

/** Дескриптор одной визуализированной руки. */
export interface HandVisualHandle {
  /** Three.js Group, который надо добавить в scene под XRController
   *  (renderer.xr.getController(i) for handedness). */
  group: THREE.Group;
  /** Обновить цвета joint-spheres на основе позы. */
  updateFromJoints(joints: {
    thumbTip: PosePosition | null;
    indexTip: PosePosition | null;
    wrist: PosePosition | null;
    fingerTips: ReadonlyArray<PosePosition | null>;
  }): void;
  /** Освободить ресурсы. */
  dispose(): void;
}

/** Создать hand-visual для одной руки. Подключается к XRController
 *  (renderer.xr.getController(i)) — three.js автоматически переносит
 *  в правильное место в XR-space. */
export function createHandVisual(
  handedness: "left" | "right",
  opts: HandVisualOptions = {}
): HandVisualHandle {
  const o = { ...DEFAULT_OPTIONS, ...opts };
  const handModelFactory = new XRHandModelFactory();

  // profile="spheres" — примитивный skeleton без GLB-ассета.
  // Создаём «глупый» controller (XRHandModelFactory ожидает Group
  // или Object3D, но createHandModel внутри делает cast в Group).
  const dummyController = new THREE.Group();
  dummyController.name = `hand-controller-${handedness}`;
  const handModel = handModelFactory.createHandModel(dummyController, "spheres");
  // handModel — Object3D, нужно настроить цвета.
  const group = new THREE.Group();
  group.name = `hand-visual-${handedness}`;
  group.add(handModel);

  // Кэш всех mesh-материалов для updateFromJoints.
  const jointMeshes: THREE.Mesh[] = [];
  handModel.traverse((obj) => {
    if ((obj as THREE.Mesh).isMesh) {
      const mesh = obj as THREE.Mesh;
      jointMeshes.push(mesh);
    }
  });

  // Контейнер для bones (линий) — найдём по имени позже, если есть.
  const bonesContainer = handModel as unknown as { bones?: THREE.Object3D[] };

  // Pulsing dot для pinch — для обратной связи «pinch detected».
  const pulseGeom = new THREE.SphereGeometry(0.008, 12, 12);
  const pulseMat = new THREE.MeshBasicMaterial({
    color: o.pinchColor,
    transparent: true,
    opacity: 0
  });
  const pulseMesh = new THREE.Mesh(pulseGeom, pulseMat);
  pulseMesh.name = `hand-pinch-pulse-${handedness}`;
  group.add(pulseMesh);

  function setColor(color: number): void {
    for (const m of jointMeshes) {
      const mat = m.material as THREE.MeshBasicMaterial;
      if (mat && "color" in mat) {
        mat.color.set(color);
      }
    }
  }

  function setBonesColor(color: number): void {
    const bones = bonesContainer.bones;
    if (!bones) return;
    for (const b of bones) {
      const mat = (b as THREE.Mesh).material as THREE.MeshBasicMaterial | undefined;
      if (mat && "color" in mat) {
        mat.color.set(color);
      }
    }
  }

  return {
    group,
    updateFromJoints(joints) {
      const pinching = isPinching(joints.thumbTip, joints.indexTip, PINCH_THRESHOLD_M);
      const gripping = isGripping(joints.wrist, joints.fingerTips);
      if (pinching) {
        setColor(o.pinchColor);
        setBonesColor(o.pinchColor);
        pulseMat.opacity = 0.6;
        // Позиция pulse = среднее между thumb и index tips (если есть).
        if (joints.thumbTip && joints.indexTip) {
          pulseMesh.position.set(
            (joints.thumbTip.x + joints.indexTip.x) / 2,
            (joints.thumbTip.y + joints.indexTip.y) / 2,
            (joints.thumbTip.z + joints.indexTip.z) / 2
          );
        }
      } else if (gripping) {
        setColor(o.gripColor);
        setBonesColor(o.gripColor);
        pulseMat.opacity = 0;
      } else {
        setColor(o.idleColor);
        setBonesColor(o.idleColor);
        pulseMat.opacity = 0;
      }
    },
    dispose() {
      pulseGeom.dispose();
      pulseMat.dispose();
      // Mesh-материалы внутри handModel освободятся автоматически при
      // удалении из scene и вызове renderer.dispose().
    }
  };
}

/** Контейнер для визуализации обеих рук (left+right). */
export interface HandsVisualBundle {
  left: HandVisualHandle | null;
  right: HandVisualHandle | null;
  addToScene(scene: THREE.Scene, leftController: THREE.Object3D, rightController: THREE.Object3D): void;
  removeFromScene(scene: THREE.Scene): void;
  dispose(): void;
}

export function createHandsVisual(opts: HandVisualOptions = {}): HandsVisualBundle {
  const left = createHandVisual("left", opts);
  const right = createHandVisual("right", opts);
  return {
    left,
    right,
    addToScene(scene, leftController, rightController) {
      leftController.add(left.group);
      rightController.add(right.group);
      scene.add(leftController);
      scene.add(rightController);
    },
    removeFromScene(scene) {
      scene.remove(left.group.parent ?? left.group);
      scene.remove(right.group.parent ?? right.group);
    },
    dispose() {
      left.dispose();
      right.dispose();
    }
  };
}
