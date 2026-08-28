// xr_teleop: маппинг WebXR Gamepad ("xr-standard" / oculus-touch-v2) →
// TeleopFSM-входы. Проверяем по webxr-input-profiles (oculus-touch-v2, Quest 2):
// buttons 0=trigger, 1=squeeze, 3=thumbstick press, 4=A/X, 5=B/Y, 6=thumbrest;
// axes 2/3 = thumbstick.

import { describe, it, expect } from "vitest";
import {
  pollXrInput,
  applySmoothing,
  createSmoothedAxes
} from "../src/input/xr_teleop";
import { GAMEPAD_AXES, GAMEPAD_BUTTONS } from "../src/input/teleop_config";

interface FakeGamepadButton {
  value: number;
  pressed: boolean;
}

interface FakeGamepad {
  buttons: FakeGamepadButton[];
  axes: number[];
}

function makeGamepad(overrides: Partial<FakeGamepad> = {}): FakeGamepad {
  const buttons: FakeGamepadButton[] = Array.from({ length: 8 }, () => ({
    value: 0,
    pressed: false
  }));
  const axes = new Array(4).fill(0);
  return { buttons, axes, ...overrides };
}

function makeSource(gamepad: FakeGamepad | null, handedness: XRHandedness = "right"): XRInputSource {
  return { gamepad, handedness } as unknown as XRInputSource;
}

describe("pollXrInput", () => {
  it("pins GAMEPAD_BUTTONS to oculus-touch-v2 hardware mapping", () => {
    // Источник: webxr-input-profiles (oculus-touch-v2, Quest 2).
    expect(GAMEPAD_BUTTONS.trigger).toBe(0);
    expect(GAMEPAD_BUTTONS.squeeze).toBe(1);
    expect(GAMEPAD_BUTTONS.thumbstickPress).toBe(3);
    expect(GAMEPAD_BUTTONS.aX).toBe(4);
    expect(GAMEPAD_BUTTONS.bY).toBe(5);
    expect(GAMEPAD_BUTTONS.thumbrest).toBe(6);
  });

  it("returns zeroes when the source has no gamepad", () => {
    expect(pollXrInput(makeSource(null))).toEqual({
      linear: 0,
      angular: 0,
      armPress: false,
      emergency: false,
      ptt: false,
      robotPtt: false
    });
  });

  it("right thumbstick press → armPress=true", () => {
    const gp = makeGamepad();
    gp.buttons[GAMEPAD_BUTTONS.thumbstickPress].pressed = true;
    expect(pollXrInput(makeSource(gp, "right")).armPress).toBe(true);
  });

  it("left thumbstick press → armPress=false (arm только на правой)", () => {
    const gp = makeGamepad();
    gp.buttons[GAMEPAD_BUTTONS.thumbstickPress].pressed = true;
    expect(pollXrInput(makeSource(gp, "left")).armPress).toBe(false);
  });

  it("B/Y → emergency=true; thumbrest does not", () => {
    const gp = makeGamepad();
    gp.buttons[GAMEPAD_BUTTONS.thumbrest].pressed = true; // НЕ emergency
    expect(pollXrInput(makeSource(gp)).emergency).toBe(false);

    gp.buttons[GAMEPAD_BUTTONS.thumbrest].pressed = false;
    gp.buttons[GAMEPAD_BUTTONS.bY].pressed = true;
    expect(pollXrInput(makeSource(gp)).emergency).toBe(true);
  });

  it("thumbstick up (y=-1) → linear=+1 (вперёд); right (x=+1) → angular<0 (направо)", () => {
    const gp = makeGamepad();
    gp.axes[GAMEPAD_AXES.thumbstickX] = 0.5; // вправо
    gp.axes[GAMEPAD_AXES.thumbstickY] = -1.0; // вверх/вперёд
    const r = pollXrInput(makeSource(gp));
    expect(r.linear).toBeCloseTo(1.0);
    // applyDeadzone(0.5) = (0.5 - 0.15) / (1 - 0.15)
    expect(r.angular).toBeCloseTo(-(0.5 - 0.15) / (1 - 0.15), 5);
  });

  it("applies deadzone: values below 0.15 map to 0", () => {
    const gp = makeGamepad();
    gp.axes[GAMEPAD_AXES.thumbstickX] = 0.1;
    gp.axes[GAMEPAD_AXES.thumbstickY] = -0.05;
    const r = pollXrInput(makeSource(gp));
    expect(r.linear).toBe(0);
    expect(r.angular).toBe(0);
  });

  it("respects custom bindings (arm=trigger on left, emergency=button 3)", () => {
    const gp = makeGamepad();
    gp.buttons[0].pressed = true; // trigger → arm (custom)
    gp.buttons[3].pressed = true; // button 3 → emergency (custom)
    const r = pollXrInput(makeSource(gp, "left"), {
      armButton: 0,
      armHandedness: "left",
      emergencyButton: 3,
      linearAxis: 3,
      angularAxis: 2,
      invertLinear: false,
      invertAngular: true,
      deadzone: 0.15,
      smoothingAlpha: 0.4,
      pttButton: 1,
      pttHandedness: "right",
      robotPttButton: 1,
      robotPttHandedness: "left"
    });
    expect(r.armPress).toBe(true);
    expect(r.emergency).toBe(true);
  });

  it("right grip (squeeze) → ptt=true, armPress=false", () => {
    const gp = makeGamepad();
    gp.buttons[GAMEPAD_BUTTONS.squeeze].value = 1;
    const r = pollXrInput(makeSource(gp, "right"));
    expect(r.ptt).toBe(true);
    expect(r.armPress).toBe(false);
  });

  it("left grip (squeeze) → armPress=false, ptt=false", () => {
    const gp = makeGamepad();
    gp.buttons[GAMEPAD_BUTTONS.squeeze].value = 1;
    const r = pollXrInput(makeSource(gp, "left"));
    expect(r.armPress).toBe(false);
    expect(r.ptt).toBe(false);
  });

  it("left grip (squeeze) → robotPtt=true (робот-голос)", () => {
    const gp = makeGamepad();
    gp.buttons[GAMEPAD_BUTTONS.squeeze].value = 1;
    const r = pollXrInput(makeSource(gp, "left"));
    expect(r.robotPtt).toBe(true);
    expect(r.armPress).toBe(false);
  });

  it("right grip (squeeze) → robotPtt=false", () => {
    const gp = makeGamepad();
    gp.buttons[GAMEPAD_BUTTONS.squeeze].value = 1;
    const r = pollXrInput(makeSource(gp, "right"));
    expect(r.robotPtt).toBe(false);
  });

  it("right trigger → ptt=false", () => {
    const gp = makeGamepad();
    gp.buttons[GAMEPAD_BUTTONS.trigger].value = 1;
    const r = pollXrInput(makeSource(gp, "right"));
    expect(r.ptt).toBe(false);
  });
});

describe("EMA smoothing (Phase 2.2)", () => {
  it("applySmoothing(α=1) returns current unchanged (no smoothing)", () => {
    const prev = createSmoothedAxes();
    const next = applySmoothing(prev, { linear: 0.7, angular: -0.3 }, 1);
    expect(next.linear).toBeCloseTo(0.7);
    expect(next.angular).toBeCloseTo(-0.3);
  });

  it("applySmoothing(α=0) keeps previous unchanged (max smoothing)", () => {
    const prev = { linear: 0.5, angular: 0.2 };
    const next = applySmoothing(prev, { linear: 0.9, angular: -0.9 }, 0);
    expect(next.linear).toBe(0.5);
    expect(next.angular).toBe(0.2);
  });

  it("applySmoothing(α=0.4) blends prev/current ~60/40", () => {
    const prev = { linear: 1.0, angular: 0.0 };
    const next = applySmoothing(prev, { linear: 0.0, angular: 1.0 }, 0.4);
    // prev.linear + α × (current.linear - prev.linear) = 1.0 + 0.4 × -1.0 = 0.6
    expect(next.linear).toBeCloseTo(0.6);
    // prev.angular + α × (current.angular - prev.angular) = 0.0 + 0.4 × 1.0 = 0.4
    expect(next.angular).toBeCloseTo(0.4);
  });

  it("applySmoothing converges to steady-state over many ticks", () => {
    let s = createSmoothedAxes();
    for (let i = 0; i < 200; i++) {
      s = applySmoothing(s, { linear: 1, angular: -1 }, 0.4);
    }
    expect(s.linear).toBeCloseTo(1, 3);
    expect(s.angular).toBeCloseTo(-1, 3);
  });
});
