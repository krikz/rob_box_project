// captain_bridge_place_hero.test.ts — unit tests for the hero-prop
// placement helpers (issue #3047).
//
// `placeHeroProps` itself lives in captain_bridge.ts and is not imported
// here: that module owns the XR bootstrap / renderer loop and pulls in
// browser-only dependencies that jsdom can't drive (see bridge_assets.ts
// module comment). The bbox/scale/position math it depends on —
// `computeFitScale` + `placeOnFloor` — was extracted into `bridge_assets.ts`
// (pure THREE.js math, no loaders) specifically so it's unit-testable in
// isolation.
//
// Covers the silent-corruption edge cases flagged in #3047:
//   1. normal group: scale computed from bbox, sits on the floor (y=0).
//   2. empty group (Box3.makeEmpty): must NOT produce -Infinity/NaN.
//   3. group whose bbox.min.y > 0 (Tripo3D origin offset): position
//      corrected so the group still sits on the floor.
//   4. width + height both given: documented tie-break (height wins,
//      applied after width in computeFitScale).

import { describe, it, expect } from "vitest";
import * as THREE from "three";
import { computeFitScale, placeOnFloor } from "../src/scene/bridge_assets";

function boxGroup(min: THREE.Vector3, max: THREE.Vector3): THREE.Group {
  const group = new THREE.Group();
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1));
  const size = new THREE.Vector3().subVectors(max, min);
  const center = new THREE.Vector3().addVectors(min, max).multiplyScalar(0.5);
  mesh.scale.copy(size);
  mesh.position.copy(center);
  group.add(mesh);
  return group;
}

describe("computeFitScale", () => {
  it("computes scale from bbox width", () => {
    const box = new THREE.Box3(new THREE.Vector3(0, 0, 0), new THREE.Vector3(2, 1, 1));
    expect(computeFitScale(box, { width: 4 })).toBeCloseTo(2);
  });

  it("height overrides width when both are given (documented tie-break)", () => {
    const box = new THREE.Box3(new THREE.Vector3(0, 0, 0), new THREE.Vector3(2, 1, 1));
    // width alone -> 4/2 = 2; height alone -> 3/1 = 3; both -> height wins.
    expect(computeFitScale(box, { width: 4, height: 3 })).toBeCloseTo(3);
  });

  it("falls back to scale 1 for an empty box instead of NaN/Infinity", () => {
    const box = new THREE.Box3(); // makeEmpty() by default
    expect(computeFitScale(box, { width: 5, height: 5 })).toBe(1);
  });

  it("ignores a target axis when the box has zero size on that axis", () => {
    const box = new THREE.Box3(new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 1, 1));
    // size.x === 0 -> width target silently skipped, no NaN/Infinity.
    expect(computeFitScale(box, { width: 4 })).toBe(1);
  });
});

describe("placeOnFloor", () => {
  it("scales and places a normal group on the floor at (x, z)", () => {
    const group = boxGroup(new THREE.Vector3(-1, 0, -1), new THREE.Vector3(1, 2, 1));
    placeOnFloor(group, 5, -3, { width: 4 });
    expect(group.scale.x).toBeCloseTo(2);
    expect(group.position.x).toBeCloseTo(5);
    expect(group.position.z).toBeCloseTo(-3);
    // bbox.min.y (0) scaled and negated -> group sits with its base at y=0.
    expect(group.position.y).toBeCloseTo(0);
  });

  it("never produces -Infinity/NaN for an empty group (#3047)", () => {
    const empty = new THREE.Group(); // no children -> Box3.setFromObject stays empty
    placeOnFloor(empty, 1, -1.8, { height: 0.9 });
    expect(Number.isFinite(empty.position.y)).toBe(true);
    expect(empty.position.y).toBe(0);
    expect(Number.isFinite(empty.scale.x)).toBe(true);
  });

  it("corrects a group whose bbox.min.y > 0 (Tripo3D origin offset) back onto the floor", () => {
    // Model authored with its base already 0.5m above its own origin.
    const group = boxGroup(new THREE.Vector3(-0.5, 0.5, -0.5), new THREE.Vector3(0.5, 1.5, 0.5));
    placeOnFloor(group, 0, 0, {});
    // scale stays 1 (no width/height target); base must land at y=0, not 0.5.
    expect(group.scale.x).toBeCloseTo(1);
    expect(group.position.y).toBeCloseTo(-0.5);
  });
});
