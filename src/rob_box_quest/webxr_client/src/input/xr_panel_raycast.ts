// XR panel raycast helpers (Phase 2 §3.7).
//
// Чистая часть: формирует THREE.Raycaster, вычисляет origin/direction из
// XRInputSource.gripSpace + frame, и возвращает panelId ближайшей
// пересечённой панели (или null).
//
// Зависимости: three. Используется в main.ts внутри XR animation loop.
// Тестируется с jsdom + stub THREE.Object3D.

import * as THREE from "three";

/** Минимальный интерфейс XRInputSource, нужный нам. */
export interface XrInputSourceForRay {
  handedness: "left" | "right" | "none";
  /** XRTargetRaySpace для controller pointer (pointing direction). */
  targetRaySpace?: { getPose?: (ref: XRReferenceSpace, f: XRFrame) => XRPose | undefined };
  /** Если это hand (XRHand), а не controller — pinching detection идёт через hand_controller. */
  hand?: unknown;
}

/** Минимальный интерфейс XRFrame для получения referenceSpace. */
export interface XrFrameLike {
  session: { referenceSpace?: XRReferenceSpace };
}

/** Минимальный интерфейс панели (mesh + id) для raycast. */
export interface PanelRaycastTarget {
  panelId: string;
  mesh: THREE.Object3D;
}

/** Получить world-space pose из targetRaySpace. */
export function getControllerWorldPose(
  source: XrInputSourceForRay,
  frame: XrFrameLike
): { position: THREE.Vector3; direction: THREE.Vector3 } | null {
  if (!source.targetRaySpace || typeof (source.targetRaySpace as { getPose?: unknown }).getPose !== "function") {
    return null;
  }
  const ref = frame.session.referenceSpace;
  if (!ref) return null;
  // XRSpace.getPose(ref, frame) — нативный WebXR API. Cast в нужный тип.
  const space = source.targetRaySpace as unknown as {
    getPose: (ref: XRReferenceSpace, f: XRFrame) => XRPose | undefined;
  };
  const pose = space.getPose(ref, frame as unknown as XRFrame);
  if (!pose) return null;
  const p = pose.transform.position;
  const o = pose.transform.orientation;
  const position = new THREE.Vector3(p.x, p.y, p.z);
  const q = new THREE.Quaternion(o.x, o.y, o.z, o.w);
  // XR convention: controller forward = local -Z.
  const forward = new THREE.Vector3(0, 0, -1).applyQuaternion(q).normalize();
  return { position, direction: forward };
}

/**
 * Raycast от controller: найти ближайшую панель, в которую "упёрся"
 * контроллер. Если ни одна панель не пересечена — вернуть null.
 *
 * `maxDistance` ограничивает длину луча (метры). По умолчанию 6м — больше,
 * чем радиус полукруга панелей (2м).
 */
export function pickPanelByRay(
  raycaster: THREE.Raycaster,
  origin: THREE.Vector3,
  direction: THREE.Vector3,
  panels: readonly PanelRaycastTarget[],
  maxDistance = 6
): { panelId: string; distance: number } | null {
  if (panels.length === 0) return null;
  raycaster.set(origin, direction);
  raycaster.far = maxDistance;
  raycaster.near = 0;
  const objects: THREE.Object3D[] = [];
  for (const p of panels) objects.push(p.mesh);
  const hits = raycaster.intersectObjects(objects, true);
  if (hits.length === 0) return null;
  // intersectObjects может вернуть hit на child (если mesh — группа); поднимаемся до panel mesh.
  const first = hits[0];
  let obj: THREE.Object3D | null = first.object;
  let bestPanelId: string | null = null;
  while (obj) {
    const found = panels.find((p) => p.mesh === obj);
    if (found) {
      bestPanelId = found.panelId;
      break;
    }
    obj = obj.parent;
  }
  if (bestPanelId == null) return null;
  return { panelId: bestPanelId, distance: first.distance };
}

/** True, если XRInputSource — это controller (имеет .gamepad), а не hand. */
export function isControllerInputSource(source: { gamepad?: unknown; hand?: unknown }): boolean {
  return source.gamepad != null && source.hand == null;
}

/** True, если XRInputSource — это hand. */
export function isHandInputSource(source: { gamepad?: unknown; hand?: unknown }): boolean {
  return source.hand != null;
}