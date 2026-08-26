// Phase 2 §2.1 + §2.3: PanelDragController (desktop raycaster) +
// PanelResizeHandles (4 corner spheres).
//
// Тестируем логику перемещения/ресайза через плоскость пола (XZ). Three.js
// raycaster мокаем: intersectObjects возвращает заранее подготовленный hit
// (object с userData.panelId/isHandle + Vector3 point). Плоскость
// raycast'а — реальная THREE.Plane, setFromCamera не использует GPU.

import { describe, it, expect, beforeEach, vi } from "vitest";
import * as THREE from "three";
import {
  PanelDragController,
  PanelResizeHandles,
  PANEL_USERDATA_KEY,
  PANEL_USERDATA_IS_HANDLE_KEY,
  PANEL_USERDATA_HANDLE_CORNER_KEY,
  type DragStartEvent,
  type DragEndEvent
} from "../src/scene/panel_drag_controller";
import { PanelManager } from "../src/scene/panel_manager";

interface FakeHit {
  object: THREE.Object3D;
  point: THREE.Vector3;
}

interface FakeRay extends THREE.Raycaster {
  /** Текущий hit, который вернёт intersectObjects. */
  currentHit: FakeHit | null;
}

function makeRaycaster(): FakeRay {
  const r = new THREE.Raycaster() as FakeRay;
  r.currentHit = null;
  vi.spyOn(r, "intersectObjects").mockImplementation((_objs, _recursive) => {
    return r.currentHit ? [r.currentHit as unknown as THREE.Intersection] : [];
  });
  // Мокаем intersectPlane: возвращает точку из currentHit (если задан).
  // raycastToFloor использует ray.intersectPlane(plane, target) — нам нужно,
  // чтобы plane+ray давали нужную точку.
  vi.spyOn(r.ray, "intersectPlane").mockImplementation((_plane: THREE.Plane, target: THREE.Vector3) => {
    if (r.currentHit) {
      target.copy(r.currentHit.point);
      return target;
    }
    return null;
  });
  // setFromCamera должен что-то делать (иначе ray.origin/direction = 0).
  // Заглушка: ничего не делаем, потому что intersectPlane уже замокан.
  vi.spyOn(r, "setFromCamera").mockImplementation(() => undefined);
  return r;
}

/** Fake panel-mesh с userData.panelId. */
function makePanelMesh(id: string): THREE.Object3D {
  const obj = new THREE.Object3D();
  obj.userData = { [PANEL_USERDATA_KEY]: id };
  return obj;
}

/** Fake resize-handle mesh. */
function makeHandleMesh(id: string, corner: "tl" | "tr" | "bl" | "br"): THREE.Object3D {
  const obj = new THREE.Object3D();
  obj.userData = {
    [PANEL_USERDATA_KEY]: id,
    [PANEL_USERDATA_IS_HANDLE_KEY]: true,
    [PANEL_USERDATA_HANDLE_CORNER_KEY]: corner
  };
  return obj;
}

function makePointerEvent(x: number, y: number, buttons = 1): PointerEvent {
  // jsdom не имеет нативного PointerEvent — соберём минимальный объект.
  const ev = new MouseEvent("pointerdown", {
    clientX: x,
    clientY: y,
    button: 0,
    bubbles: true
  }) as unknown as PointerEvent;
  Object.defineProperty(ev, "buttons", { value: buttons, writable: false });
  Object.defineProperty(ev, "pointerId", { value: 1, writable: false });
  return ev;
}

/** Алиас для других pointermove/pointerup событий. */
function pe(type: string, x: number, y: number, buttons = 1): PointerEvent {
  const ev = new MouseEvent(type, { clientX: x, clientY: y, bubbles: true }) as unknown as PointerEvent;
  Object.defineProperty(ev, "buttons", { value: buttons, writable: false });
  Object.defineProperty(ev, "pointerId", { value: 1, writable: false });
  return ev;
}

describe("PanelDragController (desktop)", () => {
  let mgr: PanelManager;
  let ray: FakeRay;
  let domEl: HTMLElement;
  let captured: string[];
  let meshes: THREE.Object3D[];

  beforeEach(() => {
    mgr = new PanelManager();
    mgr.resetLayout();
    ray = makeRaycaster();
    domEl = document.createElement("div");
    document.body.appendChild(domEl);
    captured = [];
    meshes = mgr.list().map((p) => makePanelMesh(p.id));
  });

  function start(opts?: { hit?: FakeHit }): {
    controller: PanelDragController;
    events: DragStartEvent[];
  } {
    const events: DragStartEvent[] = [];
    const controller = new PanelDragController({
      manager: mgr,
      raycaster: ray,
      domElement: domEl,
      panelYOffset: 0,
      onStart: (e) => events.push(e),
      onMove: (e) => captured.push(`move:${e.id}:${e.position.x.toFixed(2)},${e.position.z.toFixed(2)}`),
      onEnd: (e) => captured.push(`end:${e.id}:${e.zone}`)
    });
    controller.setIntersectTargets(meshes);
    ray.currentHit = opts?.hit ?? null;
    return { controller, events };
  }

  it("start() ray hits panel → onStart fires", () => {
    const id = mgr.list()[0].id;
    const hit: FakeHit = {
      object: meshes[0],
      point: new THREE.Vector3(0, 0, -2)
    };
    const { controller, events } = start({ hit });
    domEl.dispatchEvent(makePointerEvent(100, 100));
    expect(events.length).toBe(1);
    expect(events[0].id).toBe(id);
    controller.dispose();
  });

  it("start() miss → no drag", () => {
    const { controller, events } = start({ hit: undefined });
    domEl.dispatchEvent(makePointerEvent(100, 100));
    expect(events.length).toBe(0);
    controller.dispose();
  });

  it("pointermove → onMove fires with new XZ from raycaster hit", () => {
    const id = mgr.list()[0].id;
    const hit1: FakeHit = {
      object: meshes[0],
      point: new THREE.Vector3(0, 0, -2)
    };
    const { controller } = start({ hit: hit1 });
    domEl.dispatchEvent(makePointerEvent(100, 100));
    // Перемещаем курсор — raycaster возвращает новую точку (1.2, 0, -1.5).
    ray.currentHit = {
      object: meshes[0],
      point: new THREE.Vector3(1.2, 0, -1.5)
    };
    domEl.dispatchEvent(pe("pointermove", 150, 100, 1));
    expect(captured.some((s) => s.startsWith(`move:${id}:`))).toBe(true);
    // panel в mgr должна обновиться
    expect(mgr.get(id)?.position.x).toBeCloseTo(1.2, 5);
    expect(mgr.get(id)?.position.z).toBeCloseTo(-1.5, 5);
    controller.dispose();
  });

  it("pointerup → onEnd fires with snapZone", () => {
    const id = mgr.list()[0].id;
    const { controller } = start({
      hit: { object: meshes[0], point: new THREE.Vector3(0, 0, -2) }
    });
    domEl.dispatchEvent(makePointerEvent(100, 100));
    // Перемещаем близко к left anchor (-1.5, -2).
    ray.currentHit = {
      object: meshes[0],
      point: new THREE.Vector3(-1.3, 0, -1.8)
    };
    domEl.dispatchEvent(pe("pointermove", 0, 0, 1));
    domEl.dispatchEvent(pe("pointerup", 0, 0, 0));
    // onEnd должен выдать zone="left" и panel притянута к (-1.5, -2).
    const endEvent = captured.find((s) => s.startsWith(`end:${id}:`));
    expect(endEvent).toBeDefined();
    expect(endEvent).toContain(`end:${id}:left`);
    expect(mgr.get(id)?.position.x).toBeCloseTo(-1.5, 5);
    expect(mgr.get(id)?.position.z).toBeCloseTo(-2.0, 5);
    controller.dispose();
  });

  it("pointerup без активного drag → no-op", () => {
    const { controller } = start({ hit: undefined });
    domEl.dispatchEvent(pe("pointerup", 0, 0, 0));
    expect(captured.filter((s) => s.startsWith("end:")).length).toBe(0);
    controller.dispose();
  });

  it("ray.hit с неизвестным panelId игнорируется", () => {
    const fakeMesh = makePanelMesh("nope");
    const { controller, events } = start({
      hit: { object: fakeMesh, point: new THREE.Vector3(0, 0, -2) }
    });
    domEl.dispatchEvent(makePointerEvent(100, 100));
    expect(events.length).toBe(0);
    controller.dispose();
  });
});

describe("PanelResizeHandles", () => {
  let mgr: PanelManager;
  let ray: FakeRay;
  let domEl: HTMLElement;
  let meshes: THREE.Object3D[];

  beforeEach(() => {
    mgr = new PanelManager();
    mgr.resetLayout();
    ray = makeRaycaster();
    domEl = document.createElement("div");
    document.body.appendChild(domEl);
    // 4 panel mesh + 4 handle mesh per panel
    meshes = [];
    for (const p of mgr.list()) {
      meshes.push(makePanelMesh(p.id));
      meshes.push(makeHandleMesh(p.id, "tl"));
      meshes.push(makeHandleMesh(p.id, "tr"));
      meshes.push(makeHandleMesh(p.id, "bl"));
      meshes.push(makeHandleMesh(p.id, "br"));
    }
  });

  it("start() ray hits handle → onStart fires", () => {
    const id = mgr.list()[0].id;
    const startEvent: DragStartEvent[] = [];
    const resizeController = new PanelResizeHandles({
      manager: mgr,
      raycaster: ray,
      domElement: domEl,
      panelYOffset: 0,
      onStart: (e) => startEvent.push(e)
    });
    resizeController.setIntersectTargets(meshes);

    // hit на правый-нижний handle (последний добавлен в meshes)
    const handleMesh = meshes.find(
      (m) =>
        m.userData[PANEL_USERDATA_HANDLE_CORNER_KEY] === "br" &&
        m.userData[PANEL_USERDATA_KEY] === id
    );
    expect(handleMesh).toBeDefined();
    ray.currentHit = {
      object: handleMesh!,
      point: new THREE.Vector3(0.6, 0, -2)
    };
    domEl.dispatchEvent(makePointerEvent(100, 100));
    expect(startEvent.length).toBe(1);
    expect(startEvent[0].id).toBe(id);
    resizeController.dispose();
  });

  it("start() ray hits panel (no handle) → onStart НЕ срабатывает", () => {
    const id = mgr.list()[0].id;
    const startEvent: DragStartEvent[] = [];
    const resizeController = new PanelResizeHandles({
      manager: mgr,
      raycaster: ray,
      domElement: domEl,
      panelYOffset: 0,
      onStart: (e) => startEvent.push(e)
    });
    resizeController.setIntersectTargets(meshes);

    const panelMesh = meshes.find((m) => !m.userData[PANEL_USERDATA_IS_HANDLE_KEY] && m.userData[PANEL_USERDATA_KEY] === id);
    ray.currentHit = {
      object: panelMesh!,
      point: new THREE.Vector3(0, 0, -2)
    };
    domEl.dispatchEvent(makePointerEvent(100, 100));
    expect(startEvent.length).toBe(0);
    resizeController.dispose();
  });

  it("pointermove по handle → manager.resize вызывается", () => {
    const id = mgr.list()[0].id;
    const resizeController = new PanelResizeHandles({
      manager: mgr,
      raycaster: ray,
      domElement: domEl,
      panelYOffset: 0
    });
    resizeController.setIntersectTargets(meshes);

    const handleMesh = meshes.find(
      (m) =>
        m.userData[PANEL_USERDATA_HANDLE_CORNER_KEY] === "br" &&
        m.userData[PANEL_USERDATA_KEY] === id
    )!;
    ray.currentHit = { object: handleMesh, point: new THREE.Vector3(0.6, 0, -2) };
    domEl.dispatchEvent(makePointerEvent(100, 100));
    expect(resizeController.isActive()).toBe(true);

    // pointermove: drag handle в (1.0, 0, -2.2)
    ray.currentHit = { object: handleMesh, point: new THREE.Vector3(1.0, 0, -2.2) };
    domEl.dispatchEvent(pe("pointermove", 200, 200, 1));

    const p = mgr.get(id)!;
    // Размер должен измениться (хотя бы один параметр).
    expect(p.size.width !== 1.2 || p.size.height !== 0.7).toBe(true);
    resizeController.dispose();
  });

  it("pointerup → onEnd fires", () => {
    const id = mgr.list()[0].id;
    const endEvents: DragEndEvent[] = [];
    const resizeController = new PanelResizeHandles({
      manager: mgr,
      raycaster: ray,
      domElement: domEl,
      panelYOffset: 0,
      onEnd: (e) => endEvents.push(e)
    });
    resizeController.setIntersectTargets(meshes);

    const handleMesh = meshes.find(
      (m) =>
        m.userData[PANEL_USERDATA_HANDLE_CORNER_KEY] === "br" &&
        m.userData[PANEL_USERDATA_KEY] === id
    )!;
    ray.currentHit = { object: handleMesh, point: new THREE.Vector3(0.6, 0, -2) };
    domEl.dispatchEvent(makePointerEvent(100, 100));
    domEl.dispatchEvent(pe("pointerup", 0, 0, 0));
    expect(endEvents.length).toBe(1);
    resizeController.dispose();
  });
});
