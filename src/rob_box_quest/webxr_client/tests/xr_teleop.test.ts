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

function makeSource(gamepad: FakeGamepad | null, handedness: XRHandedness = "right"): XRInputSource {
  return { gamepad, handedness } as unknown as XRInputSource;
}

describe("pollXrInput", () => {
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

  it("right thumbstick press (button 2) → armPress=true", () => {
    const gp = makeGamepad();
    gp.buttons[2].pressed = true;
    expect(pollXrInput(makeSource(gp, "right")).armPress).toBe(true);
  });

  it("left thumbstick press → armPress=false (arm только на правой)", () => {
    const gp = makeGamepad();
    gp.buttons[2].pressed = true;
    expect(pollXrInput(makeSource(gp, "left")).armPress).toBe(false);
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

  it("respects custom bindings (arm=trigger on left, emergency=A/X)", () => {
    const gp = makeGamepad();
    gp.buttons[0].pressed = true; // trigger → arm (custom)
    gp.buttons[3].pressed = true; // A/X → emergency (custom)
    const r = pollXrInput(makeSource(gp, "left"), {
      armButton: 0,
      armHandedness: "left",
      emergencyButton: 3,
      linearAxis: 3,
      angularAxis: 2,
      invertLinear: false,
      invertAngular: true,
      deadzone: 0.12,
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
    gp.buttons[1].value = 1;
    const r = pollXrInput(makeSource(gp, "right"));
    expect(r.ptt).toBe(true);
    expect(r.armPress).toBe(false);
  });

  it("left grip (squeeze) → armPress=false, ptt=false", () => {
    const gp = makeGamepad();
    gp.buttons[1].value = 1;
    const r = pollXrInput(makeSource(gp, "left"));
    expect(r.armPress).toBe(false);
    expect(r.ptt).toBe(false);
  });

  it("left grip (squeeze) → robotPtt=true (робот-голос)", () => {
    const gp = makeGamepad();
    gp.buttons[1].value = 1;
    const r = pollXrInput(makeSource(gp, "left"));
    expect(r.robotPtt).toBe(true);
    expect(r.armPress).toBe(false);
  });

  it("right grip (squeeze) → robotPtt=false", () => {
    const gp = makeGamepad();
    gp.buttons[1].value = 1;
    const r = pollXrInput(makeSource(gp, "right"));
    expect(r.robotPtt).toBe(false);
  });

  it("right trigger (button 0) → ptt=false", () => {
    const gp = makeGamepad();
    gp.buttons[0].value = 1;
    const r = pollXrInput(makeSource(gp, "right"));
    expect(r.ptt).toBe(false);
  });
});
