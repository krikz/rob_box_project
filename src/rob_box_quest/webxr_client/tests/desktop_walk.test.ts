// Десктоп (#3149): WASD ходит оператором, робота ведут стрелки при
// зажатом Space, E — emergency. Раскладки не пересекаются.

import { describe, it, expect, afterEach } from "vitest";
import * as THREE from "three";
import { createDesktopTeleop, DESKTOP_TELEOP_KEYS } from "../src/input/desktop_teleop";
import {
  applyMouseLook,
  stepWalk,
  WALK_BOUNDS,
  WALK_KEYS,
  WALK_SPEED_MPS,
  RUN_SPEED_MPS
} from "../src/input/desktop_walk";
import { createDesktopPointer } from "../src/interaction/desktop_pointer";
import { DEFAULT_HOTKEYS } from "../src/ui/help_overlay";
import { TeleopFSM } from "../src/input/teleop_fsm";

function key(type: "keydown" | "keyup", code: string, target: EventTarget = window): void {
  target.dispatchEvent(new KeyboardEvent(type, { code, bubbles: true }));
}

describe("desktop teleop keys", () => {
  let destroy: (() => void) | null = null;
  afterEach(() => destroy?.());

  it("WASD no longer drives the robot", () => {
    const fsm = new TeleopFSM();
    const t = createDesktopTeleop({ fsm });
    destroy = t.destroy;
    key("keydown", "Space");
    key("keydown", "KeyW");
    key("keydown", "KeyA");
    const out = fsm.tick(1000, true);
    expect(out?.cmd.cmd).toBe("teleop_twist");
    const tw = out!.cmd as { linear: { x: number }; angular: { z: number } };
    expect(tw.linear.x).toBe(0);
    expect(tw.angular.z).toBe(0);
  });

  it("arrows drive only with Space (deadman) held", () => {
    const fsm = new TeleopFSM();
    const t = createDesktopTeleop({ fsm });
    destroy = t.destroy;
    key("keydown", "ArrowUp");
    expect(fsm.tick(1000, true)).toBeNull(); // без deadman — тишина
    key("keydown", "Space");
    key("keydown", "ArrowLeft");
    const tw = fsm.tick(2000, true)!.cmd as { linear: { x: number }; angular: { z: number } };
    expect(tw.linear.x).toBeGreaterThan(0);
    expect(tw.angular.z).toBeGreaterThan(0);
  });

  it("walk and teleop keys do not overlap", () => {
    for (const k of WALK_KEYS) expect(DESKTOP_TELEOP_KEYS).not.toContain(k);
    expect(DESKTOP_TELEOP_KEYS).toEqual(
      expect.arrayContaining(["ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight", "Space", "KeyE"])
    );
  });

  it("help overlay documents the new layout", () => {
    const keys = DEFAULT_HOTKEYS.map((h) => h.key);
    expect(keys).toContain("W A S D");
    expect(keys.some((k) => k.includes("Space") && k.includes("↑"))).toBe(true);
    expect(DEFAULT_HOTKEYS.some((h) => /boost/i.test(h.description))).toBe(false);
  });
});

describe("stepWalk / applyMouseLook", () => {
  const origin = { x: 0, z: 0, yaw: 0, pitch: 0 };

  it("W walks toward −Z (the main screen) at walking speed", () => {
    const p = stepWalk(origin, { forward: 1, strafe: 0, run: false }, 0.5);
    expect(p.x).toBeCloseTo(0);
    expect(p.z).toBeCloseTo(-WALK_SPEED_MPS * 0.5);
  });

  it("D strafes right (+X); Shift runs", () => {
    const p = stepWalk(origin, { forward: 0, strafe: 1, run: true }, 0.5);
    expect(p.x).toBeCloseTo(RUN_SPEED_MPS * 0.5);
    expect(p.z).toBeCloseTo(0);
  });

  it("diagonal is not faster than straight", () => {
    const p = stepWalk(origin, { forward: 1, strafe: 1, run: false }, 1);
    expect(Math.hypot(p.x, p.z)).toBeCloseTo(WALK_SPEED_MPS);
  });

  it("forward follows yaw (turned left 90° → walks to −X)", () => {
    const p = stepWalk({ ...origin, yaw: Math.PI / 2 }, { forward: 1, strafe: 0, run: false }, 1);
    expect(p.x).toBeCloseTo(-WALK_SPEED_MPS);
    expect(p.z).toBeCloseTo(0);
  });

  it("clamps to the walkable area (cannot walk through the main screen)", () => {
    const p = stepWalk(origin, { forward: 1, strafe: 0, run: true }, 100);
    expect(p.z).toBe(WALK_BOUNDS.minZ);
    const q = stepWalk(origin, { forward: 0, strafe: -1, run: true }, 100);
    expect(q.x).toBe(WALK_BOUNDS.minX);
  });

  it("mouse right turns right (yaw decreases), pitch is clamped", () => {
    const p = applyMouseLook(origin, 100, 0);
    expect(p.yaw).toBeLessThan(0);
    const q = applyMouseLook(origin, 0, -1e6);
    expect(q.pitch).toBeCloseTo((85 * Math.PI) / 180);
  });
});

describe("desktop pointer when the browser refuses pointer lock", () => {
  it("after pointerlockerror clicks press the scene instead of re-requesting the lock", () => {
    const canvas = document.createElement("canvas");
    document.body.appendChild(canvas);
    let lockRequested = 0;
    (canvas as unknown as { requestPointerLock: () => void }).requestPointerLock = () => {
      lockRequested += 1;
    };
    const camera = new THREE.PerspectiveCamera(70, 1, 0.05, 50);
    camera.updateMatrixWorld();
    const ptr = createDesktopPointer({ canvas, camera, lockOnClick: true });
    canvas.dispatchEvent(new MouseEvent("mousemove", { clientX: 0, clientY: 0 }));
    canvas.dispatchEvent(new MouseEvent("mousedown", { button: 0 }));
    expect(lockRequested).toBe(1);
    document.dispatchEvent(new Event("pointerlockerror"));
    canvas.dispatchEvent(new MouseEvent("mousedown", { button: 0 }));
    expect(lockRequested).toBe(1);
    // jsdom: getBoundingClientRect = 0×0 → курсор не «внутри»; нажатие всё
    // равно зафиксировано и уйдёт с первым лучом.
    Object.defineProperty(document, "pointerLockElement", { configurable: true, get: () => canvas });
    expect(ptr.poll()!.pressed).toBe(true);
    ptr.destroy();
    canvas.remove();
    delete (document as unknown as { pointerLockElement?: unknown }).pointerLockElement;
  });

  it("a rejected requestPointerLock() promise also falls back to plain clicks", async () => {
    const canvas = document.createElement("canvas");
    document.body.appendChild(canvas);
    let lockRequested = 0;
    (canvas as unknown as { requestPointerLock: () => Promise<void> }).requestPointerLock = () => {
      lockRequested += 1;
      return Promise.reject(new Error("NotAllowedError"));
    };
    const camera = new THREE.PerspectiveCamera(70, 1, 0.05, 50);
    const ptr = createDesktopPointer({ canvas, camera, lockOnClick: true });
    canvas.dispatchEvent(new MouseEvent("mousedown", { button: 0 }));
    await Promise.resolve();
    await Promise.resolve();
    canvas.dispatchEvent(new MouseEvent("mousedown", { button: 0 }));
    expect(lockRequested).toBe(1);
    ptr.destroy();
    canvas.remove();
  });
});

describe("desktop pointer with pointer lock", () => {
  it("first click only captures the mouse; when locked the ray comes from screen centre", () => {
    const canvas = document.createElement("canvas");
    document.body.appendChild(canvas);
    let lockRequested = 0;
    (canvas as unknown as { requestPointerLock: () => void }).requestPointerLock = () => {
      lockRequested += 1;
    };
    const camera = new THREE.PerspectiveCamera(70, 1, 0.05, 50);
    camera.position.set(0, 1.6, 0);
    camera.updateMatrixWorld();
    const ptr = createDesktopPointer({ canvas, camera, lockOnClick: true });

    canvas.dispatchEvent(new MouseEvent("mousedown", { button: 0 }));
    expect(lockRequested).toBe(1);
    expect(ptr.poll()).toBeNull(); // курсора не было, и клик не «нажал»

    // Эмулируем захват.
    Object.defineProperty(document, "pointerLockElement", { configurable: true, get: () => canvas });
    document.dispatchEvent(new Event("pointerlockchange"));
    const ray = ptr.poll()!;
    expect(ray.pressed).toBe(false);
    expect(ray.direction.z).toBeCloseTo(-1); // прямо вперёд — центр экрана
    canvas.dispatchEvent(new MouseEvent("mousedown", { button: 0 }));
    expect(lockRequested).toBe(1);
    expect(ptr.poll()!.pressed).toBe(true);

    // Esc (захват снят) — кнопка отпущена.
    Object.defineProperty(document, "pointerLockElement", { configurable: true, get: () => null });
    document.dispatchEvent(new Event("pointerlockchange"));
    expect(ptr.poll()).toBeNull();

    ptr.destroy();
    canvas.remove();
    delete (document as unknown as { pointerLockElement?: unknown }).pointerLockElement;
  });
});
