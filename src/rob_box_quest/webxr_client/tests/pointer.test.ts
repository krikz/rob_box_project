// Тесты слоя указателя: математика драга, машина состояний
// (наведение / клик / перетаскивание) и выбор XR-контроллера.

import { describe, it, expect, vi } from "vitest";
import * as THREE from "three";
import {
  dragTargetPosition,
  intersectRaySphere,
  facingToward,
  horizontalRadius,
  isDrag,
  clampPanelY,
  PANEL_MIN_Y,
  PANEL_MAX_Y
} from "../src/interaction/pointer_math";
import { PointerSystem } from "../src/interaction/pointer";
import { pickSource, isPointerPressed, POINTER_BUTTON } from "../src/interaction/xr_pointer";

const CENTER = { x: 0, y: 1.6, z: 0 };

function panel(x: number, y: number, z: number): THREE.Mesh {
  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry(1.2, 0.7),
    new THREE.MeshBasicMaterial()
  );
  mesh.position.set(x, y, z);
  mesh.lookAt(CENTER.x, y, CENTER.z);
  mesh.updateMatrixWorld(true);
  return mesh;
}

describe("pointer_math — sphere intersection", () => {
  it("hits the sphere ahead of the operator", () => {
    const p = intersectRaySphere(CENTER, { x: 0, y: 0, z: -1 }, CENTER, 2);
    expect(p).not.toBeNull();
    expect(p!.z).toBeCloseTo(-2, 5);
    expect(p!.x).toBeCloseTo(0, 5);
  });

  it("takes the far root, not the one behind the operator", () => {
    // Оператор в центре сферы: ближний корень отрицательный (за спиной).
    const p = intersectRaySphere(CENTER, { x: 1, y: 0, z: 0 }, CENTER, 2)!;
    expect(p.x).toBeCloseTo(2, 5);
  });

  it("returns null when the ray misses", () => {
    const origin = { x: 10, y: 1.6, z: 0 };
    expect(intersectRaySphere(origin, { x: 0, y: 0, z: -1 }, CENTER, 2)).toBeNull();
  });
});

describe("pointer_math — drag target", () => {
  it("keeps the panel at the same distance from the operator", () => {
    const p = dragTargetPosition(CENTER, { x: 1, y: 0, z: -1 }, CENTER, 2);
    expect(Math.hypot(p.x - CENTER.x, p.y - CENTER.y, p.z - CENTER.z)).toBeCloseTo(2, 4);
  });

  it("falls back to the ray direction when the ray misses the sphere", () => {
    const origin = { x: 10, y: 1.6, z: 0 };
    const p = dragTargetPosition(origin, { x: 0, y: 0, z: -1 }, CENTER, 2);
    // Панель не залипает: кладём её на сферу по направлению луча.
    expect(p.z).toBeCloseTo(-2, 4);
  });

  it("clamps height so the panel cannot go under the floor or the ceiling", () => {
    const low = dragTargetPosition(CENTER, { x: 0, y: -1, z: -0.05 }, CENTER, 2);
    expect(low.y).toBeCloseTo(PANEL_MIN_Y, 5);
    const high = dragTargetPosition(CENTER, { x: 0, y: 1, z: -0.05 }, CENTER, 2);
    expect(high.y).toBeCloseTo(PANEL_MAX_Y, 5);
    expect(clampPanelY(1.5)).toBe(1.5);
  });
});

describe("pointer_math — helpers", () => {
  it("facing always points back at the operator", () => {
    const f = facingToward({ x: 2, y: 1.6, z: 0 }, CENTER);
    expect(f.x).toBeCloseTo(-1, 5);
    expect(f.z).toBeCloseTo(0, 5);
  });

  it("horizontal radius ignores height", () => {
    expect(horizontalRadius({ x: 0, y: 5, z: -2 }, CENTER)).toBeCloseTo(2, 5);
  });

  it("small moves are clicks, large ones are drags", () => {
    const a = { x: 0, y: 1.6, z: -2 };
    expect(isDrag(a, { x: 0.01, y: 1.6, z: -2 })).toBe(false);
    expect(isDrag(a, { x: 0.5, y: 1.6, z: -2 })).toBe(true);
  });
});

describe("PointerSystem", () => {
  function setup(draggable = true) {
    const handlers = {
      onHover: vi.fn(),
      onSelect: vi.fn(),
      onDragStart: vi.fn(),
      onDrag: vi.fn(),
      onDragEnd: vi.fn()
    };
    const sys = new PointerSystem({ center: CENTER, handlers });
    const mesh = panel(0, 1.6, -2);
    sys.addTarget({ id: "p1", object: mesh, draggable });
    return { sys, handlers, mesh };
  }

  const forward = (pressed: boolean) => ({
    origin: CENTER,
    direction: { x: 0, y: 0, z: -1 },
    pressed
  });
  const aside = (pressed: boolean) => ({
    origin: CENTER,
    direction: { x: 1, y: 0, z: -0.2 },
    pressed
  });

  it("reports hover when the ray enters and leaves the target", () => {
    const { sys, handlers } = setup();
    sys.update(forward(false));
    expect(handlers.onHover).toHaveBeenLastCalledWith("p1");
    expect(sys.getHovered()).toBe("p1");
    sys.update(aside(false));
    expect(handlers.onHover).toHaveBeenLastCalledWith(null);
  });

  it("does not repeat the hover callback for the same target", () => {
    const { sys, handlers } = setup();
    sys.update(forward(false));
    sys.update(forward(false));
    expect(handlers.onHover).toHaveBeenCalledTimes(1);
  });

  it("press and release without moving is a select, not a drag", () => {
    const { sys, handlers } = setup();
    sys.update(forward(false));
    sys.update(forward(true));
    sys.update(forward(false));
    expect(handlers.onSelect).toHaveBeenCalledWith("p1");
    expect(handlers.onDragStart).not.toHaveBeenCalled();
  });

  it("press then move is a drag, and no select fires", () => {
    const { sys, handlers } = setup();
    sys.update(forward(false));
    sys.update(forward(true));
    sys.update(aside(true));
    expect(handlers.onDragStart).toHaveBeenCalledWith("p1");
    expect(handlers.onDrag).toHaveBeenCalled();
    expect(sys.isDragging()).toBe(true);
    sys.update(aside(false));
    expect(handlers.onDragEnd).toHaveBeenCalledWith("p1");
    expect(handlers.onSelect).not.toHaveBeenCalled();
  });

  it("keeps the dragged panel at its original radius", () => {
    const { sys, handlers } = setup();
    sys.update(forward(false));
    sys.update(forward(true));
    sys.update(aside(true));
    const pos = handlers.onDrag.mock.calls.at(-1)![1];
    expect(Math.hypot(pos.x - CENTER.x, pos.z - CENTER.z)).toBeCloseTo(2, 2);
  });

  it("never drags a target marked non-draggable (buttons)", () => {
    const { sys, handlers } = setup(false);
    sys.update(forward(false));
    sys.update(forward(true));
    sys.update(aside(true));
    expect(handlers.onDragStart).not.toHaveBeenCalled();
    expect(handlers.onDrag).not.toHaveBeenCalled();
  });

  it("releasing over a different target does not select", () => {
    const { sys, handlers } = setup(false);
    sys.update(forward(true));
    sys.update(aside(false));
    expect(handlers.onSelect).not.toHaveBeenCalled();
  });

  it("losing the ray mid-drag ends the drag instead of sticking", () => {
    const { sys, handlers } = setup();
    sys.update(forward(false));
    sys.update(forward(true));
    sys.update(aside(true));
    sys.update(null);
    expect(handlers.onDragEnd).toHaveBeenCalledWith("p1");
    expect(sys.isDragging()).toBe(false);
    expect(handlers.onHover).toHaveBeenLastCalledWith(null);
  });

  it("ignores removed targets", () => {
    const { sys, handlers } = setup();
    sys.removeTarget("p1");
    sys.update(forward(true));
    sys.update(forward(false));
    expect(handlers.onSelect).not.toHaveBeenCalled();
    expect(sys.getHovered()).toBeNull();
  });

  it("picks the nearest target when two overlap", () => {
    const handlers = { onHover: vi.fn() };
    const sys = new PointerSystem({ center: CENTER, handlers });
    sys.addTarget({ id: "far", object: panel(0, 1.6, -4) });
    sys.addTarget({ id: "near", object: panel(0, 1.6, -2) });
    sys.update(forward(false));
    expect(handlers.onHover).toHaveBeenLastCalledWith("near");
  });
});

describe("xr_pointer", () => {
  const src = (handedness: string, buttons: Array<{ pressed: boolean; value: number }>) =>
    ({ handedness, gamepad: { buttons, axes: [] } }) as unknown as XRInputSource;

  it("prefers the right hand", () => {
    const left = src("left", []);
    const right = src("right", []);
    expect(pickSource([left, right])).toBe(right);
  });

  it("falls back to whatever controller has a gamepad", () => {
    const left = src("left", []);
    expect(pickSource([{ handedness: "none" } as XRInputSource, left])).toBe(left);
  });

  it("reads the trigger as the select button", () => {
    const buttons = Array.from({ length: 6 }, () => ({ pressed: false, value: 0 }));
    const source = src("right", buttons);
    expect(isPointerPressed(source)).toBe(false);
    buttons[POINTER_BUTTON] = { pressed: false, value: 0.8 }; // аналоговый ход
    expect(isPointerPressed(source)).toBe(true);
  });

  it("survives a controller without a gamepad", () => {
    expect(isPointerPressed({ handedness: "right" } as XRInputSource)).toBe(false);
  });
});
