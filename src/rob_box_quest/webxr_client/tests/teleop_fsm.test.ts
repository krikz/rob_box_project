import { describe, it, expect, beforeEach } from "vitest";
import {
  TeleopFSM,
  MAX_LINEAR,
  MAX_ANGULAR,
  THROTTLE_INTERVAL_MS
} from "../src/input/teleop_fsm";

describe("TeleopFSM", () => {
  let fsm: TeleopFSM;
  beforeEach(() => {
    fsm = new TeleopFSM();
  });

  it("starts in idle and emits nothing", () => {
    expect(fsm.getState()).toBe("idle");
    expect(fsm.tick(1000, true)).toBeNull();
  });

  it("emits twist with scaled linear/angular when deadman pressed", () => {
    fsm.setLinear(1);
    fsm.setAngular(-0.5);
    fsm.setDeadman(true);
    const out = fsm.tick(1000, true);
    expect(out).not.toBeNull();
    expect(out!.type).toBe("twist");
    expect(out!.cmd.cmd).toBe("teleop_twist");
    if (out!.cmd.cmd === "teleop_twist") {
      expect(out!.cmd.linear.x).toBeCloseTo(MAX_LINEAR);
      expect(out!.cmd.angular.z).toBeCloseTo(-MAX_ANGULAR * 0.5);
      expect(out!.cmd.deadman).toBe(true);
    }
  });

  it("throttles to ~30 Hz (one per THROTTLE_INTERVAL_MS)", () => {
    fsm.setLinear(0.5);
    fsm.setDeadman(true);
    const t0 = 1000;
    expect(fsm.tick(t0, true)).not.toBeNull();
    expect(fsm.tick(t0 + 10, false)).toBeNull();
    // THROTTLE_INTERVAL_MS = 1000/30 ≈ 33.33... → округлим до 34 для теста.
    expect(fsm.tick(t0 + Math.ceil(THROTTLE_INTERVAL_MS), false)).not.toBeNull();
  });

  it("emits zero-twist stop on deadman release (deadman=false, twist=0)", () => {
    fsm.setLinear(0.5);
    fsm.setAngular(0.2);
    fsm.setDeadman(true);
    fsm.tick(1000, true); // active
    fsm.setDeadman(false); // → stopping
    const out = fsm.tick(1100, true);
    expect(out).not.toBeNull();
    expect(out!.type).toBe("stop");
    if (out!.cmd.cmd === "teleop_twist") {
      expect(out!.cmd.linear.x).toBe(0);
      expect(out!.cmd.angular.z).toBe(0);
      expect(out!.cmd.deadman).toBe(false);
    }
  });

  it("goes back to idle after STOP_DELAY_MS and emits nothing further", () => {
    fsm.setDeadman(true);
    fsm.tick(1000, true);
    fsm.setDeadman(false);
    fsm.tick(1050, true); // первый stop
    const later = 1050 + 100 + 50; // > STOP_DELAY_MS
    expect(fsm.tick(later, true)).toBeNull();
    expect(fsm.getState()).toBe("idle");
  });

  it("clamps input to [-1, 1]", () => {
    fsm.setLinear(5);
    fsm.setAngular(-10);
    fsm.setDeadman(true);
    const out = fsm.tick(1000, true);
    expect(out).not.toBeNull();
    if (out!.cmd.cmd === "teleop_twist") {
      expect(out!.cmd.linear.x).toBe(MAX_LINEAR);
      expect(out!.cmd.angular.z).toBe(-MAX_ANGULAR);
    }
  });

  it("seq is monotonically increasing", () => {
    fsm.setDeadman(true);
    fsm.tick(1000, true);
    fsm.tick(1100, true);
    fsm.tick(1200, true);
    expect(fsm.getSeq()).toBe(3);
  });

  it("triggerEmergency emits stop_emergency regardless of deadman", () => {
    const cmd = fsm.triggerEmergency("controller_b");
    expect(cmd.cmd).toBe("stop_emergency");
    expect(cmd.source).toBe("controller_b");
  });

  it("reset() returns to idle and clears deadman", () => {
    fsm.setLinear(0.5);
    fsm.setDeadman(true);
    fsm.reset();
    expect(fsm.getState()).toBe("idle");
    expect(fsm.tick(2000, true)).toBeNull();
  });
});