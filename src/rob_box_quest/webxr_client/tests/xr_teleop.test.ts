// xr_teleop: маппинг WebXR Gamepad ("xr-standard") → TeleopFSM-входы.
// Проверяем по докам Meta (Quest Touch): buttons 0=trigger, 1=squeeze,
// 2=thumbstick press, 3=A/X, 4=B/Y, 5=thumbrest; axes 2/3 = thumbstick.

import { describe, it, expect } from "vitest";
import { pollXrInput } from "../src/input/xr_teleop";

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

function makeSource(gamepad: FakeGamepad | null): XRInputSource {
  return { gamepad } as unknown as XRInputSource;
}

describe("pollXrInput", () => {
  it("returns zeroes when the source has no gamepad", () => {
    expect(pollXrInput(makeSource(null))).toEqual({
      linear: 0,
      angular: 0,
      deadman: false,
      emergency: false
    });
  });

  it("squeeze (button 1) → deadman=true", () => {
    const gp = makeGamepad();
    gp.buttons[1].value = 1;
    expect(pollXrInput(makeSource(gp)).deadman).toBe(true);
  });

  it("B/Y (button 4) → emergency=true; thumbrest (button 5) does not", () => {
    const gp = makeGamepad();
    gp.buttons[5].pressed = true; // thumbrest — НЕ emergency
    expect(pollXrInput(makeSource(gp)).emergency).toBe(false);

    gp.buttons[5].pressed = false;
    gp.buttons[4].pressed = true; // B/Y
    expect(pollXrInput(makeSource(gp)).emergency).toBe(true);
  });

  it("thumbstick axes 2/3 → linear (y) and angular (-x), re-scaled past deadzone", () => {
    const gp = makeGamepad();
    gp.axes[2] = 0.5; // thumbstick x → angular = -0.5 (после deadzone)
    gp.axes[3] = 1.0; // thumbstick y → linear = 1.0
    const r = pollXrInput(makeSource(gp));
    expect(r.linear).toBeCloseTo(1.0);
    // applyDeadzone(0.5) = (0.5 - 0.12) / (1 - 0.12)
    expect(r.angular).toBeCloseTo(-(0.5 - 0.12) / (1 - 0.12), 5);
  });

  it("applies deadzone: values below 0.12 map to 0", () => {
    const gp = makeGamepad();
    gp.axes[2] = 0.1;
    gp.axes[3] = -0.05;
    const r = pollXrInput(makeSource(gp));
    expect(r.linear).toBe(0);
    expect(r.angular).toBe(0);
  });
});
