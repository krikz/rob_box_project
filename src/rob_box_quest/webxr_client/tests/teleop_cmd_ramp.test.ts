import { describe, it, expect } from "vitest";
import { TeleopCmdRamp, RAMP_MS } from "../src/input/teleop_cmd_ramp";

describe("TeleopCmdRamp — basic behaviour", () => {
  it("starts at zero (0, 0)", () => {
    const r = new TeleopCmdRamp();
    expect(r.getCurrent()).toEqual({ linear: 0, angular: 0 });
    expect(r.getTarget()).toEqual({ linear: 0, angular: 0 });
  });

  it("snap() sets current directly without ramp", () => {
    const r = new TeleopCmdRamp();
    r.snap({ linear: 1, angular: -0.5 });
    expect(r.getCurrent()).toEqual({ linear: 1, angular: -0.5 });
    expect(r.getTarget()).toEqual({ linear: 1, angular: -0.5 });
  });

  it("setTarget() ramps over RAMP_MS to reach target", () => {
    let now = 0;
    const r = new TeleopCmdRamp({ now: () => now });
    r.setTarget({ linear: 1, angular: 0 });
    // Halfway: smoothstep(0.5) = 0.5
    now = RAMP_MS / 2;
    const mid = r.tick();
    expect(mid.linear).toBeCloseTo(0.5, 1);
    // After RAMP_MS, snap to target.
    now = RAMP_MS + 1;
    const end = r.tick();
    expect(end.linear).toBeCloseTo(1, 5);
    expect(end.ramping).toBe(false);
  });

  it("setTarget() to same target is no-op (no ramp started)", () => {
    let now = 0;
    const r = new TeleopCmdRamp({ now: () => now });
    r.snap({ linear: 1, angular: 0 });
    r.setTarget({ linear: 1, angular: 0 }); // same
    // tick at t=0 — should not register as ramping.
    now = 0;
    expect(r.isRamping()).toBe(false);
  });

  it("isRamping() false after ramp completes", () => {
    let now = 0;
    const r = new TeleopCmdRamp({ now: () => now });
    r.setTarget({ linear: 1, angular: 0 });
    expect(r.isRamping()).toBe(true);
    now = RAMP_MS + 1;
    r.tick();
    expect(r.isRamping()).toBe(false);
  });
});

describe("TeleopCmdRamp — handoff semantics", () => {
  it("switching from old target to new: ramp continues smoothly from current (no jump)", () => {
    let now = 0;
    const r = new TeleopCmdRamp({ now: () => now });
    // First target: ramp up to 1.0 over RAMP_MS.
    r.setTarget({ linear: 1, angular: 0 });
    now = RAMP_MS / 4; // 25% in: smoothstep(0.25) ≈ 0.156
    const mid1 = r.tick();
    const partial = mid1.linear;
    expect(partial).toBeGreaterThan(0);
    expect(partial).toBeLessThan(1);
    // Now switch to new target (e.g., 0.5). Ramp should start from CURRENT (not from 0).
    r.setTarget({ linear: 0.5, angular: 0 });
    // One tick immediately: current should equal what we had (no jump back to 0).
    const mid2 = r.tick();
    expect(mid2.linear).toBeCloseTo(partial, 3);
  });

  it("ramping to 0 from non-zero (mode exit): smoothly decays", () => {
    let now = 0;
    const r = new TeleopCmdRamp({ now: () => now });
    r.snap({ linear: 1, angular: 0 });
    r.setTarget({ linear: 0, angular: 0 });
    // Halfway: should be ~0.5
    now = RAMP_MS / 2;
    const mid = r.tick();
    expect(mid.linear).toBeGreaterThan(0);
    expect(mid.linear).toBeLessThan(1);
    expect(mid.ramping).toBe(true);
    // Done
    now = RAMP_MS + 1;
    const end = r.tick();
    expect(end.linear).toBeCloseTo(0, 5);
    expect(end.ramping).toBe(false);
  });
});

describe("TeleopCmdRamp — reset and edge cases", () => {
  it("reset() clears everything to zero", () => {
    const r = new TeleopCmdRamp();
    r.snap({ linear: 1, angular: 1 });
    r.reset();
    expect(r.getCurrent()).toEqual({ linear: 0, angular: 0 });
    expect(r.getTarget()).toEqual({ linear: 0, angular: 0 });
    expect(r.isRamping()).toBe(false);
  });

  it("zero rampMs means snap (no interpolation)", () => {
    const r = new TeleopCmdRamp({ rampMs: 0 });
    r.snap({ linear: 0, angular: 0 });
    r.setTarget({ linear: 1, angular: 1 });
    const t = r.tick();
    expect(t.linear).toBeCloseTo(1, 5);
    expect(t.ramping).toBe(false);
  });

  it("rampMs=50 by default (Phase 2 spec)", () => {
    expect(RAMP_MS).toBe(50);
  });

  it("smoothstep midpoint = 0.5 (verifies ramp curve shape)", () => {
    let now = 0;
    const r = new TeleopCmdRamp({ now: () => now });
    r.snap({ linear: 0, angular: 0 });
    r.setTarget({ linear: 1, angular: 0 });
    now = RAMP_MS / 2; // exactly halfway
    const mid = r.tick();
    // smoothstep(0.5) = 0.5 * 0.5 * (3 - 1) = 0.5
    expect(mid.linear).toBeCloseTo(0.5, 5);
  });
});