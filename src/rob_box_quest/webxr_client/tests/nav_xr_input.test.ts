import { describe, it, expect } from "vitest";
import { NAV_AIM_BUTTON, createEdge, navAimPressed } from "../src/nav/nav_xr_input";
import { DEFAULT_BINDINGS, GAMEPAD_BUTTONS } from "../src/input/teleop_config";

function src(pressedIdx: number[]) {
  const buttons = Array.from({ length: 7 }, (_, i) => ({ pressed: pressedIdx.includes(i) }));
  return { gamepad: { buttons } };
}

describe("nav_xr_input", () => {
  it("A/X не занята телеопом, голосом и аварийкой", () => {
    expect(NAV_AIM_BUTTON).toBe(GAMEPAD_BUTTONS.aX);
    const taken = [
      DEFAULT_BINDINGS.armButton,
      DEFAULT_BINDINGS.emergencyButton,
      DEFAULT_BINDINGS.pttButton,
      DEFAULT_BINDINGS.robotPttButton,
      GAMEPAD_BUTTONS.trigger
    ];
    expect(taken).not.toContain(NAV_AIM_BUTTON);
  });
  it("любая рука", () => {
    expect(navAimPressed([src([]), src([4])])).toBe(true);
    expect(navAimPressed([src([0, 5]), { gamepad: null }])).toBe(false);
  });
  it("фронт", () => {
    const e = createEdge();
    expect([e(false), e(true), e(true), e(false), e(true)]).toEqual([false, true, false, false, true]);
  });
});
