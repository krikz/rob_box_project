// hand_teleop: gesture detection (pinch/grip) → WSS hand_pinch/hand_grip.
//
// Тесты покрывают чистую геометрию и edge-detection. Joint-poses
// подсовываются через `makeFrame(jointsMap)`, где jointsMap — Record
// от XRHandJoint до позиции (или null если joint не трекнулся).
// Hand mock реализует Map-like API (XRHand extends Map<XRHandJoint,
// XRJointSpace>), поэтому тест-фреймворк может собрать любую
// позу-руки без реального WebXR.

import { describe, it, expect } from "vitest";
import {
  distance3D,
  isPinching,
  isGripping,
  detectEdge,
  createHandTeleopState,
  processHandFrame,
  handFrameToCommands,
  extractKeyJoints,
  createHandTeleop,
  PINCH_THRESHOLD_M,
  GRIP_THRESHOLD_M,
  type JointFrame,
  type JointPoseLike,
  type PosePosition,
  type HandKeyJoints
} from "../src/input/hand_teleop";

function pos(x: number, y: number, z: number): PosePosition {
  return { x, y, z };
}

/** Fake XRFrame, отдаёт joints из переданной карты. */
function makeFrame(joints: Record<string, PosePosition | null>): JointFrame {
  return {
    getJointPose(jointLike: unknown): JointPoseLike | undefined {
      // jointLike — XRJointSpace { jointName }. Достаём имя через duck typing.
      const jointName = (jointLike as { jointName?: string } | null | undefined)?.jointName;
      if (!jointName) return undefined;
      const p = joints[jointName];
      if (!p) return undefined;
      return { transform: { position: p } };
    }
  };
}

/** Fake XRHand: Map-like, `get(jointName)` → XRJointSpace-like. */
function makeHand(joints: Record<string, PosePosition | null>): {
  get(key: string): unknown;
} {
  return {
    get(key: string): unknown {
      if (!(key in joints)) return undefined;
      return { jointName: key };
    }
  };
}

/** Hand без данных (hand: null). */
function makeEmptyHand(): null {
  return null;
}

describe("distance3D", () => {
  it("computes euclidean distance", () => {
    expect(distance3D(pos(0, 0, 0), pos(3, 4, 0))).toBe(5);
    expect(distance3D(pos(1, 1, 1), pos(1, 1, 1))).toBe(0);
    expect(distance3D(pos(0, 0, 0), pos(0, 0, 0.02))).toBeCloseTo(0.02);
  });
});

describe("isPinching", () => {
  it("returns false when either tip is null", () => {
    expect(isPinching(null, pos(0, 0, 0))).toBe(false);
    expect(isPinching(pos(0, 0, 0), null)).toBe(false);
    expect(isPinching(null, null)).toBe(false);
  });

  it("returns true when thumb-tip and index-tip are < threshold", () => {
    expect(isPinching(pos(0, 0, 0), pos(0.01, 0, 0))).toBe(true);
    expect(isPinching(pos(0, 0, 0), pos(0, 0, 0.019))).toBe(true);
  });

  it("returns false when thumb-tip and index-tip are >= threshold", () => {
    expect(isPinching(pos(0, 0, 0), pos(0.02, 0, 0))).toBe(false);
    expect(isPinching(pos(0, 0, 0), pos(0.05, 0, 0))).toBe(false);
  });

  it("respects custom threshold", () => {
    expect(isPinching(pos(0, 0, 0), pos(0.05, 0, 0), 0.1)).toBe(true);
    expect(isPinching(pos(0, 0, 0), pos(0.05, 0, 0), 0.04)).toBe(false);
  });
});

describe("isGripping", () => {
  const wrist = pos(0, 0, 0);
  it("returns false when wrist is null", () => {
    expect(isGripping(null, [wrist, wrist, wrist, wrist])).toBe(false);
  });

  it("returns false when fewer than 4 tips tracked", () => {
    expect(isGripping(wrist, [null, null, null, null])).toBe(false);
    expect(isGripping(wrist, [wrist, wrist, wrist, null])).toBe(false);
  });

  it("returns true when all 4 tips are close to wrist", () => {
    expect(isGripping(wrist, [
      pos(0.03, 0, 0), // index
      pos(0, 0.03, 0), // middle
      pos(0, 0, 0.03), // ring
      pos(0.02, 0.02, 0) // pinky
    ])).toBe(true);
  });

  it("returns false when tips are spread (open hand)", () => {
    expect(isGripping(wrist, [
      pos(0.1, 0, 0),
      pos(0, 0.1, 0),
      pos(0, 0, 0.1),
      pos(0.1, 0.1, 0)
    ])).toBe(false);
  });
});

describe("detectEdge", () => {
  it("returns 'rising' on false→true", () => {
    expect(detectEdge(false, true)).toBe("rising");
  });

  it("returns 'falling' on true→false", () => {
    expect(detectEdge(true, false)).toBe("falling");
  });

  it("returns null on no transition", () => {
    expect(detectEdge(false, false)).toBeNull();
    expect(detectEdge(true, true)).toBeNull();
  });
});

describe("processHandFrame (geometry integration)", () => {
  it("first call reports rising edge if pose already pinching/gripping", () => {
    // state.pinchPressed=false (initial) → detected pinching → rising edge.
    const state = createHandTeleopState();
    const joints: HandKeyJoints = {
      thumbTip: pos(0, 0, 0),
      indexTip: pos(0.01, 0, 0), // pinching
      wrist: pos(0, 0, 0),
      fingerTips: [pos(0.01, 0, 0), pos(0, 0.01, 0), pos(0, 0, 0.01), pos(0.01, 0.01, 0)]
    };
    const out = processHandFrame(state, joints);
    expect(out.pinching).toBe(true);
    expect(out.gripping).toBe(true);
    expect(out.pinchEdge).toBe("rising");
    expect(out.gripEdge).toBe("rising");
  });

  it("first call reports no edges if pose is open (matches baseline)", () => {
    const state = createHandTeleopState();
    const joints: HandKeyJoints = {
      thumbTip: pos(0, 0, 0),
      indexTip: pos(0.1, 0, 0), // far
      wrist: pos(0, 0, 0),
      fingerTips: [pos(0.1, 0, 0), pos(0, 0.1, 0), pos(0, 0, 0.1), pos(0.1, 0.1, 0)]
    };
    const out = processHandFrame(state, joints);
    expect(out.pinching).toBe(false);
    expect(out.gripping).toBe(false);
    expect(out.pinchEdge).toBeNull();
    expect(out.gripEdge).toBeNull();
  });

  it("rising edge fires when hand transitions to pinch", () => {
    const state = createHandTeleopState();
    const open: HandKeyJoints = {
      thumbTip: pos(0, 0, 0),
      indexTip: pos(0.1, 0, 0), // far apart
      wrist: pos(0, 0, 0),
      fingerTips: [pos(0.1, 0, 0), pos(0, 0.1, 0), pos(0, 0, 0.1), pos(0.1, 0.1, 0)]
    };
    const pinched: HandKeyJoints = {
      thumbTip: pos(0, 0, 0),
      indexTip: pos(0.01, 0, 0), // close
      wrist: pos(0, 0, 0),
      fingerTips: [pos(0.1, 0, 0), pos(0, 0.1, 0), pos(0, 0, 0.1), pos(0.1, 0.1, 0)]
    };
    processHandFrame(state, open); // baseline: nothing pressed
    const out = processHandFrame(state, pinched);
    expect(out.pinchEdge).toBe("rising");
    expect(out.gripEdge).toBeNull();
    expect(state.pinchPressed).toBe(true);
  });

  it("falling edge fires when hand transitions out of pinch", () => {
    const state = createHandTeleopState();
    state.pinchPressed = true; // baseline: pinching
    const open: HandKeyJoints = {
      thumbTip: pos(0, 0, 0),
      indexTip: pos(0.1, 0, 0),
      wrist: pos(0, 0, 0),
      fingerTips: [pos(0.1, 0, 0), pos(0, 0.1, 0), pos(0, 0, 0.1), pos(0.1, 0.1, 0)]
    };
    const out = processHandFrame(state, open);
    expect(out.pinchEdge).toBe("falling");
    expect(state.pinchPressed).toBe(false);
  });

  it("rising grip edge fires when hand closes to fist", () => {
    const state = createHandTeleopState();
    const open: HandKeyJoints = {
      thumbTip: pos(0, 0, 0),
      indexTip: pos(0.1, 0, 0),
      wrist: pos(0, 0, 0),
      fingerTips: [pos(0.1, 0, 0), pos(0, 0.1, 0), pos(0, 0, 0.1), pos(0.1, 0.1, 0)]
    };
    const fist: HandKeyJoints = {
      thumbTip: pos(0, 0, 0),
      indexTip: pos(0.1, 0, 0),
      wrist: pos(0, 0, 0),
      fingerTips: [pos(0.01, 0, 0), pos(0, 0.01, 0), pos(0, 0, 0.01), pos(0.01, 0.01, 0)]
    };
    processHandFrame(state, open);
    const out = processHandFrame(state, fist);
    expect(out.gripEdge).toBe("rising");
    expect(out.pinchEdge).toBeNull();
  });
});

describe("handFrameToCommands (WSS binding)", () => {
  it("emits hand_pinch on rising/falling pinch edges", () => {
    const result = { pinchEdge: "rising" as const, gripEdge: null, pinching: true, gripping: false };
    const cmds = handFrameToCommands(result, "right", 1000);
    expect(cmds).toHaveLength(1);
    expect(cmds[0]).toMatchObject({
      cmd: "hand_pinch",
      ts_ms: 1000,
      pressed: true,
      handedness: "right",
      source: "select"
    });
  });

  it("emits hand_pinch pressed=false on falling edge", () => {
    const result = { pinchEdge: "falling" as const, gripEdge: null, pinching: false, gripping: false };
    const cmds = handFrameToCommands(result, "left", 2000);
    expect(cmds).toHaveLength(1);
    expect(cmds[0]).toMatchObject({
      cmd: "hand_pinch",
      pressed: false,
      handedness: "left"
    });
  });

  it("emits hand_grip with source='squeeze'", () => {
    const result = { pinchEdge: null, gripEdge: "rising" as const, pinching: false, gripping: true };
    const cmds = handFrameToCommands(result, "right", 3000);
    expect(cmds).toHaveLength(1);
    expect(cmds[0]).toMatchObject({
      cmd: "hand_grip",
      ts_ms: 3000,
      pressed: true,
      handedness: "right",
      source: "squeeze"
    });
  });

  it("emits both pinch+grip edges when both transition same frame", () => {
    const result = { pinchEdge: "rising" as const, gripEdge: "rising" as const, pinching: true, gripping: true };
    const cmds = handFrameToCommands(result, "right", 4000);
    expect(cmds).toHaveLength(2);
    expect(cmds.map((c) => (c as { cmd: string }).cmd).sort()).toEqual(["hand_grip", "hand_pinch"]);
  });

  it("emits nothing when no edges", () => {
    const result = { pinchEdge: null, gripEdge: null, pinching: true, gripping: true };
    expect(handFrameToCommands(result, "right", 5000)).toEqual([]);
  });
});

describe("extractKeyJoints (frame/hand integration)", () => {
  it("returns null for untracked joints", () => {
    const frame = makeFrame({ wrist: pos(0, 0, 0) });
    const hand = makeHand({ wrist: pos(0, 0, 0) });
    const k = extractKeyJoints(frame, hand, null);
    expect(k.wrist).toEqual(pos(0, 0, 0));
    expect(k.thumbTip).toBeNull();
    expect(k.indexTip).toBeNull();
    expect(k.fingerTips).toEqual([null, null, null, null]);
  });

  it("extracts positions for all 6 key joints", () => {
    const frame = makeFrame({
      wrist: pos(0, 0, 0),
      "thumb-tip": pos(0, 0, 0.01),
      "index-finger-tip": pos(0, 0, 0.01),
      "middle-finger-tip": pos(0, 0, 0.01),
      "ring-finger-tip": pos(0, 0, 0.01),
      "pinky-finger-tip": pos(0, 0, 0.01)
    });
    const hand = makeHand({
      wrist: pos(0, 0, 0),
      "thumb-tip": pos(0, 0, 0.01),
      "index-finger-tip": pos(0, 0, 0.01),
      "middle-finger-tip": pos(0, 0, 0.01),
      "ring-finger-tip": pos(0, 0, 0.01),
      "pinky-finger-tip": pos(0, 0, 0.01)
    });
    const k = extractKeyJoints(frame, hand, null);
    expect(k.wrist).not.toBeNull();
    expect(k.thumbTip).not.toBeNull();
    expect(k.indexTip).not.toBeNull();
    expect(k.fingerTips.every((t) => t !== null)).toBe(true);
  });

  it("handles null hand gracefully (no XRHand on this source)", () => {
    const frame = makeFrame({ wrist: pos(0, 0, 0) });
    const k = extractKeyJoints(frame, makeEmptyHand(), null);
    expect(k.wrist).toBeNull();
    expect(k.thumbTip).toBeNull();
  });
});

describe("createHandTeleop (end-to-end with mock frame)", () => {
  it("rising pinch produces a hand_pinch cmd with handedness", () => {
    const ht = createHandTeleop();
    const open = makeFrame({
      wrist: pos(0, 0, 0),
      "thumb-tip": pos(0, 0, 0),
      "index-finger-tip": pos(0.1, 0, 0),
      "middle-finger-tip": pos(0, 0.1, 0),
      "ring-finger-tip": pos(0, 0, 0.1),
      "pinky-finger-tip": pos(0.1, 0.1, 0)
    });
    const openHand = makeHand({
      wrist: pos(0, 0, 0),
      "thumb-tip": pos(0, 0, 0),
      "index-finger-tip": pos(0.1, 0, 0),
      "middle-finger-tip": pos(0, 0.1, 0),
      "ring-finger-tip": pos(0, 0, 0.1),
      "pinky-finger-tip": pos(0.1, 0.1, 0)
    });
    const pinched = makeFrame({
      wrist: pos(0, 0, 0),
      "thumb-tip": pos(0, 0, 0),
      "index-finger-tip": pos(0.005, 0, 0), // very close → pinch
      "middle-finger-tip": pos(0, 0.1, 0),
      "ring-finger-tip": pos(0, 0, 0.1),
      "pinky-finger-tip": pos(0.1, 0.1, 0)
    });
    const pinchedHand = makeHand({
      wrist: pos(0, 0, 0),
      "thumb-tip": pos(0, 0, 0),
      "index-finger-tip": pos(0.005, 0, 0),
      "middle-finger-tip": pos(0, 0.1, 0),
      "ring-finger-tip": pos(0, 0, 0.1),
      "pinky-finger-tip": pos(0.1, 0.1, 0)
    });
    // Baseline: open.
    ht.process(open, openHand, "right", null, 1000);
    // Pinched → expects rising pinch edge.
    const { cmds } = ht.process(pinched, pinchedHand, "right", null, 1100);
    expect(cmds).toHaveLength(1);
    expect(cmds[0]).toMatchObject({
      cmd: "hand_pinch",
      pressed: true,
      handedness: "right",
      ts_ms: 1100
    });
  });

  it("rising grip (fist) produces hand_grip with source='squeeze'", () => {
    const ht = createHandTeleop();
    const open = makeFrame({
      wrist: pos(0, 0, 0),
      "thumb-tip": pos(0, 0, 0),
      "index-finger-tip": pos(0.1, 0, 0),
      "middle-finger-tip": pos(0, 0.1, 0),
      "ring-finger-tip": pos(0, 0, 0.1),
      "pinky-finger-tip": pos(0.1, 0.1, 0)
    });
    const openHand = makeHand({
      wrist: pos(0, 0, 0),
      "thumb-tip": pos(0, 0, 0),
      "index-finger-tip": pos(0.1, 0, 0),
      "middle-finger-tip": pos(0, 0.1, 0),
      "ring-finger-tip": pos(0, 0, 0.1),
      "pinky-finger-tip": pos(0.1, 0.1, 0)
    });
    const fist = makeFrame({
      wrist: pos(0, 0, 0),
      // In a real fist, thumb-tip wraps to the SIDE of the index finger
      // (~3-5 cm apart in y-axis) — not pinched against the index tip.
      "thumb-tip": pos(0, 0.04, 0),
      "index-finger-tip": pos(0, 0.01, 0),
      "middle-finger-tip": pos(0, 0.01, 0),
      "ring-finger-tip": pos(0, 0.01, 0),
      "pinky-finger-tip": pos(0, 0.01, 0)
    });
    const fistHand = makeHand({
      wrist: pos(0, 0, 0),
      "thumb-tip": pos(0, 0.04, 0),
      "index-finger-tip": pos(0, 0.01, 0),
      "middle-finger-tip": pos(0, 0.01, 0),
      "ring-finger-tip": pos(0, 0.01, 0),
      "pinky-finger-tip": pos(0, 0.01, 0)
    });
    ht.process(open, openHand, "left", null, 1000);
    const { cmds } = ht.process(fist, fistHand, "left", null, 1050);
    expect(cmds).toHaveLength(1);
    expect(cmds[0]).toMatchObject({
      cmd: "hand_grip",
      pressed: true,
      handedness: "left",
      source: "squeeze"
    });
  });

  it("stable pinching produces no cmds on subsequent frames", () => {
    const ht = createHandTeleop();
    const f = makeFrame({
      wrist: pos(0, 0, 0),
      "thumb-tip": pos(0, 0, 0),
      "index-finger-tip": pos(0.01, 0, 0),
      "middle-finger-tip": pos(0, 0.1, 0),
      "ring-finger-tip": pos(0, 0, 0.1),
      "pinky-finger-tip": pos(0.1, 0.1, 0)
    });
    const h = makeHand({
      wrist: pos(0, 0, 0),
      "thumb-tip": pos(0, 0, 0),
      "index-finger-tip": pos(0.01, 0, 0),
      "middle-finger-tip": pos(0, 0.1, 0),
      "ring-finger-tip": pos(0, 0, 0.1),
      "pinky-finger-tip": pos(0.1, 0.1, 0)
    });
    // First frame: rising pinch edge.
    const a = ht.process(f, h, "right", null, 1000);
    expect(a.cmds).toHaveLength(1);
    // Second frame: same pinch → no edge.
    const b = ht.process(f, h, "right", null, 1100);
    expect(b.cmds).toEqual([]);
    expect(b.result.pinching).toBe(true);
  });

  it("reset() clears edge-state so next pose is fresh baseline", () => {
    const ht = createHandTeleop();
    const f = makeFrame({
      wrist: pos(0, 0, 0),
      "thumb-tip": pos(0, 0, 0),
      "index-finger-tip": pos(0.01, 0, 0),
      "middle-finger-tip": pos(0, 0.1, 0),
      "ring-finger-tip": pos(0, 0, 0.1),
      "pinky-finger-tip": pos(0.1, 0.1, 0)
    });
    const h = makeHand({
      wrist: pos(0, 0, 0),
      "thumb-tip": pos(0, 0, 0),
      "index-finger-tip": pos(0.01, 0, 0),
      "middle-finger-tip": pos(0, 0.1, 0),
      "ring-finger-tip": pos(0, 0, 0.1),
      "pinky-finger-tip": pos(0.1, 0.1, 0)
    });
    ht.process(f, h, "right", null, 1000); // pinching
    expect(ht.isActive()).toBe(true);
    ht.reset();
    expect(ht.isActive()).toBe(false);
    // После reset следующий кадр с тем же pose даст rising edge.
    const out = ht.process(f, h, "right", null, 1500);
    expect(out.cmds).toHaveLength(1);
    expect(out.cmds[0]).toMatchObject({ cmd: "hand_pinch", pressed: true });
  });
});

describe("default thresholds", () => {
  it("PINCH_THRESHOLD_M is 2 cm per Phase 2.2 spec", () => {
    expect(PINCH_THRESHOLD_M).toBe(0.02);
  });
  it("GRIP_THRESHOLD_M is 5 cm per Phase 2.2 spec", () => {
    expect(GRIP_THRESHOLD_M).toBe(0.05);
  });
});
