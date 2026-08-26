import { describe, expect, it } from "vitest";
import * as THREE from "three";

// jsdom не предоставляет XRRigidTransform — стаб глобально.
if (typeof (globalThis as { XRRigidTransform?: unknown }).XRRigidTransform === "undefined") {
  class XRRigidTransform {
    position = { x: 0, y: 0, z: 0 };
    orientation = { x: 0, y: 0, z: 0, w: 1 };
  }
  (globalThis as unknown as { XRRigidTransform: unknown }).XRRigidTransform = XRRigidTransform;
}

import {
  getControllerWorldPose,
  pickPanelByRay,
  isControllerInputSource,
  isHandInputSource,
  type XrInputSourceForRay,
  type XrFrameLike,
  type PanelRaycastTarget
} from "../src/input/xr_panel_raycast";

/**
 * Stub getPose: возвращает XRPose с position (x,0,0) и forward -Z (identity quat).
 */
function makeSource(handedness: "left" | "right" | "none" = "right"): XrInputSourceForRay {
  // Создаём XRRigidTransform через конструктор (он есть в jsdom в lib.dom).
  const transform = new XRRigidTransform();
  const pose: XRPose = {
    transform,
    linearVelocity: undefined,
    angularVelocity: undefined,
    emulatedPosition: false
  };
  return {
    handedness,
    targetRaySpace: {
      getPose(_ref: XRReferenceSpace, _frame: XRFrame): XRPose | undefined {
        return pose;
      }
    }
  };
}

function makeFrame(): XrFrameLike {
  // Минимальный frame с referenceSpace.
  return {
    session: {
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      referenceSpace: {} as any
    }
  };
}

describe("getControllerWorldPose", () => {
  it("returns position + direction when source has targetRaySpace + pose", () => {
    const src = makeSource("right");
    const frame = makeFrame();
    const out = getControllerWorldPose(src, frame);
    expect(out).not.toBe(null);
    expect(out!.position.x).toBeCloseTo(0);
    expect(out!.position.z).toBeCloseTo(0);
    // forward = -Z (identity rotation)
    expect(out!.direction.z).toBeCloseTo(-1);
  });

  it("returns null when source has no targetRaySpace", () => {
    const src: XrInputSourceForRay = { handedness: "right" };
    expect(getControllerWorldPose(src, makeFrame())).toBe(null);
  });

  it("returns null when targetRaySpace.getPose is not a function", () => {
    const src: XrInputSourceForRay = { handedness: "right", targetRaySpace: {} };
    expect(getControllerWorldPose(src, makeFrame())).toBe(null);
  });

  it("returns null when frame has no referenceSpace", () => {
    const src = makeSource("right");
    const frame: XrFrameLike = { session: {} };
    expect(getControllerWorldPose(src, frame)).toBe(null);
  });

  it("returns null when getPose returns undefined", () => {
    const src: XrInputSourceForRay = {
      handedness: "right",
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      targetRaySpace: { getPose: () => undefined } as any
    };
    expect(getControllerWorldPose(src, makeFrame())).toBe(null);
  });
});

describe("pickPanelByRay", () => {
  // Создаём plane mesh перед источником луча.
  function makePlaneMesh(z: number): { mesh: THREE.Object3D } {
    const g = new THREE.PlaneGeometry(1, 1);
    const m = new THREE.MeshBasicMaterial({ side: THREE.DoubleSide });
    const mesh = new THREE.Mesh(g, m);
    mesh.position.set(0, 0, z);
    // raycaster использует matrixWorld — нужно вызвать updateMatrixWorld(),
    // иначе для свеже-созданного mesh'а matrixWorld = identity (origin).
    mesh.updateMatrixWorld(true);
    return { mesh };
  }

  it("returns null when panels list is empty", () => {
    const rc = new THREE.Raycaster();
    const origin = new THREE.Vector3();
    const dir = new THREE.Vector3(0, 0, -1);
    expect(pickPanelByRay(rc, origin, dir, [])).toBe(null);
  });

  it("returns panelId when ray hits a plane in front of origin", () => {
    const rc = new THREE.Raycaster();
    const origin = new THREE.Vector3(0, 0, 0);
    const dir = new THREE.Vector3(0, 0, -1);
    const a = makePlaneMesh(-1);
    a.mesh.userData["panelId"] = "A";
    const b = makePlaneMesh(-2);
    b.mesh.userData["panelId"] = "B";
    const targets: PanelRaycastTarget[] = [
      { panelId: "A", mesh: a.mesh },
      { panelId: "B", mesh: b.mesh }
    ];
    const hit = pickPanelByRay(rc, origin, dir, targets);
    expect(hit).not.toBe(null);
    // Ближайшая = A.
    expect(hit!.panelId).toBe("A");
    expect(hit!.distance).toBeCloseTo(1, 2);
  });

  it("returns null when all planes are behind the ray", () => {
    const rc = new THREE.Raycaster();
    const origin = new THREE.Vector3(0, 0, 0);
    const dir = new THREE.Vector3(0, 0, -1);
    const a = makePlaneMesh(1); // +Z — луч идёт в -Z
    const targets: PanelRaycastTarget[] = [{ panelId: "A", mesh: a.mesh }];
    expect(pickPanelByRay(rc, origin, dir, targets)).toBe(null);
  });

  it("respects maxDistance", () => {
    const rc = new THREE.Raycaster();
    const origin = new THREE.Vector3(0, 0, 0);
    const dir = new THREE.Vector3(0, 0, -1);
    const far = makePlaneMesh(-10);
    const targets: PanelRaycastTarget[] = [{ panelId: "far", mesh: far.mesh }];
    // maxDistance=5 → панель в 10м не видна.
    expect(pickPanelByRay(rc, origin, dir, targets, 5)).toBe(null);
  });
});

describe("isControllerInputSource / isHandInputSource", () => {
  it("controller: has gamepad, no hand", () => {
    expect(isControllerInputSource({ gamepad: {} as unknown })).toBe(true);
    expect(isHandInputSource({ gamepad: {} as unknown })).toBe(false);
  });
  it("hand: has hand property", () => {
    expect(isHandInputSource({ hand: {} as unknown })).toBe(true);
    expect(isControllerInputSource({ hand: {} as unknown })).toBe(false);
  });
  it("empty source — neither", () => {
    expect(isControllerInputSource({})).toBe(false);
    expect(isHandInputSource({})).toBe(false);
  });
});