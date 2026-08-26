import { describe, it, expect } from "vitest";
import {
  createCameraController,
  DEFAULT_CAMERA_POSITION,
  DEFAULT_CAMERA_YAW,
  DEFAULT_CAMERA_PITCH,
  PITCH_LIMIT_DEG,
  YAW_LIMIT_DEG
} from "../src/scene/camera_controller";

describe("CameraController", () => {
  it("starts at default position (0, 1.6, 0) facing -Z", () => {
    const cam = createCameraController();
    const s = cam.getState();
    expect(s.yaw).toBe(DEFAULT_CAMERA_YAW);
    expect(s.pitch).toBe(DEFAULT_CAMERA_PITCH);
    expect(s.position).toEqual(DEFAULT_CAMERA_POSITION);
  });

  it("applyStickAxes accumulates yaw/pitch at configured rates", () => {
    const cam = createCameraController();
    // 1000ms full deflection → 1.5 rad yaw, 1.0 rad pitch (per default rates)
    cam.applyStickAxes(1, 1, 1000);
    const target = cam.getTarget();
    expect(target.yaw).toBeCloseTo(1.5, 5);
    expect(target.pitch).toBeCloseTo(1.0, 5);
  });

  it("clamps pitch to ±PITCH_LIMIT_DEG (60°)", () => {
    const cam = createCameraController();
    const limitRad = (PITCH_LIMIT_DEG * Math.PI) / 180;
    // Far exceed limit: 10 seconds at full deflection.
    cam.applyStickAxes(0, -1, 10000);
    expect(cam.getTarget().pitch).toBeCloseTo(-limitRad, 5);
    cam.applyStickAxes(0, 1, 10000);
    expect(cam.getTarget().pitch).toBeCloseTo(limitRad, 5);
  });

  it("clamps yaw to ±YAW_LIMIT_DEG (180°)", () => {
    const cam = createCameraController();
    const limitRad = (YAW_LIMIT_DEG * Math.PI) / 180;
    cam.applyStickAxes(1, 0, 10000);
    expect(cam.getTarget().yaw).toBeCloseTo(limitRad, 5);
    cam.applyStickAxes(-1, 0, 10000);
    expect(cam.getTarget().yaw).toBeCloseTo(-limitRad, 5);
  });

  it("reset() snaps target back to default (snap, not lerp)", () => {
    const cam = createCameraController();
    cam.applyStickAxes(1, 1, 500);
    expect(cam.getTarget().yaw).not.toBe(DEFAULT_CAMERA_YAW);
    cam.reset();
    const t = cam.getTarget();
    expect(t.yaw).toBe(DEFAULT_CAMERA_YAW);
    expect(t.pitch).toBe(DEFAULT_CAMERA_PITCH);
    expect(t.position).toEqual(DEFAULT_CAMERA_POSITION);
  });

  it("tickDamping moves current toward target over time", () => {
    const cam = createCameraController();
    cam.applyStickAxes(1, 0, 100); // yaw += 0.15
    const targetYaw = cam.getTarget().yaw;
    // Initial current = 0
    expect(cam.getState().yaw).toBeCloseTo(0, 5);
    // One tick at 16ms (60fps) — move ~15% toward target
    cam.tickDamping(16);
    const after1 = cam.getState().yaw;
    expect(after1).toBeGreaterThan(0);
    expect(after1).toBeLessThan(targetYaw);
    // Many ticks → converge
    for (let i = 0; i < 100; i += 1) cam.tickDamping(16);
    expect(cam.getState().yaw).toBeCloseTo(targetYaw, 2);
  });

  it("setState() updates target without going through applyStickAxes", () => {
    const cam = createCameraController();
    cam.setState({ yaw: 0.5, pitch: 0.2, position: { x: 1, y: 2, z: 3 } });
    const t = cam.getTarget();
    expect(t.yaw).toBeCloseTo(0.5, 5);
    expect(t.pitch).toBeCloseTo(0.2, 5);
    expect(t.position).toEqual({ x: 1, y: 2, z: 3 });
  });

  it("applyStickAxes ignores stick outside [-1, 1] (clamps to range)", () => {
    const cam = createCameraController();
    cam.applyStickAxes(2, -2, 1000);
    // 2 clamped to 1; -2 clamped to -1.
    // With pitchStick direct (no negation): dpitch = -1 * 1.0 = -1.0
    const t = cam.getTarget();
    expect(t.yaw).toBeCloseTo(1.5, 5);
    expect(t.pitch).toBeCloseTo(-1.0, 5);
  });
});

describe("CameraController — yaw rate direction", () => {
  it("negative X stick rotates yaw clockwise (negative yaw)", () => {
    const cam = createCameraController();
    cam.applyStickAxes(-1, 0, 500);
    expect(cam.getTarget().yaw).toBeLessThan(0);
  });

  it("negative Y stick tilts pitch down (negative pitch — stick down = look down)", () => {
    const cam = createCameraController();
    cam.applyStickAxes(0, -1, 500);
    expect(cam.getTarget().pitch).toBeLessThan(0);
  });

  it("positive Y stick tilts pitch up (positive pitch — stick up = look up)", () => {
    const cam = createCameraController();
    cam.applyStickAxes(0, 1, 500);
    expect(cam.getTarget().pitch).toBeGreaterThan(0);
  });
});

describe("CameraController — independent position", () => {
  it("applyStickAxes does NOT change position", () => {
    const cam = createCameraController();
    cam.applyStickAxes(1, 1, 1000);
    expect(cam.getTarget().position).toEqual(DEFAULT_CAMERA_POSITION);
  });
});