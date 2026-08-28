import { describe, it, expect } from "vitest";
import { existsSync, readFileSync, statSync } from "node:fs";
import { resolve } from "node:path";
import { NodeIO } from "@gltf-transform/core";
import { ALL_EXTENSIONS } from "@gltf-transform/extensions";

// avatar-source smoke test (Phase 2.1, ADR-0032 §3.2, kanban t_cdb60035).
//
// Validates the **raw** glTF asset stored at the repository source root
// (assets/source/avatar/avatar.glb), so that any change to build-avatar.mjs
// that breaks structure (wrong node count, missing animation, broken parse,
// licensing metadata gone) is caught before the Phase 2.0 optimisation
// pipeline runs. Mirrors the contract of avatar_pipeline.test.ts but
// operates on the pre-optimisation artefact and on a slightly stricter set
// of assertions (raw budget + CC0 attribution in SOURCE.md).
//
// The path is resolved relative to the webxr_client package root (this
// file's location), so the test is independent of cwd.

const ASSET_PATH = resolve(
  __dirname,
  "..",
  "..",
  "..",
  "..",
  "assets",
  "source",
  "avatar",
  "avatar.glb"
);

const SOURCE_MD_PATH = resolve(
  __dirname,
  "..",
  "..",
  "..",
  "..",
  "assets",
  "source",
  "avatar",
  "SOURCE.md"
);

const RAW_BUDGET_BYTES = 100 * 1024; // ≤ 100 KB per build-avatar.mjs

const EXPECTED_ANIMATIONS = [
  "idle",
  "drive_forward",
  "drive_backward",
  "turn_left",
  "turn_right",
];

describe("avatar-source smoke (Phase 2.1, ADR-0032 §3.2, kanban t_cdb60035)", () => {
  it("the raw avatar.glb asset exists on disk", () => {
    expect(existsSync(ASSET_PATH)).toBe(true);
  });

  it("the raw avatar.glb is within the raw size budget (≤ 100 KB)", () => {
    const size = statSync(ASSET_PATH).size;
    expect(size).toBeLessThanOrEqual(RAW_BUDGET_BYTES);
  });

  it("parses as a valid glTF 2.0 document via gltf-transform NodeIO", async () => {
    const io = new NodeIO().registerExtensions(ALL_EXTENSIONS);
    const doc = await io.read(ASSET_PATH);

    // glTF 2.0 requires a scene with at least one node; sanity check.
    const root = doc.getRoot();
    expect(root.listScenes().length).toBeGreaterThanOrEqual(1);
    expect(root.listNodes().length).toBeGreaterThanOrEqual(1);
    expect(root.listMeshes().length).toBeGreaterThanOrEqual(1);
  });

  it("declares the 5 expected animation clips (no missing animations after regeneration)", async () => {
    const io = new NodeIO().registerExtensions(ALL_EXTENSIONS);
    const doc = await io.read(ASSET_PATH);
    const animNames = new Set(
      doc.getRoot().listAnimations().map((a) => a.getName())
    );

    for (const expected of EXPECTED_ANIMATIONS) {
      expect(animNames.has(expected)).toBe(true);
    }
  });

  it("every animation has a non-zero duration (no empty clips)", async () => {
    const io = new NodeIO().registerExtensions(ALL_EXTENSIONS);
    const doc = await io.read(ASSET_PATH);
    for (const a of doc.getRoot().listAnimations()) {
      let duration = 0;
      for (const sampler of a.listSamplers()) {
        const accessor = sampler.getInput();
        if (accessor === null) continue;
        const arr = accessor.getArray();
        if (arr === null || arr.length === 0) continue;
        const last: number = arr[arr.length - 1] as number;
        if (last > duration) duration = last;
      }
      expect(duration).toBeGreaterThan(0);
    }
  });

  it("SOURCE.md exists and explicitly documents the CC0 licence", () => {
    expect(existsSync(SOURCE_MD_PATH)).toBe(true);
    const source = readFileSync(SOURCE_MD_PATH, "utf8");
    expect(source.toLowerCase()).toContain("cc0");
    expect(source.toLowerCase()).toContain("public domain");
    // Also assert the explicit CC0 1.0 URL is cited, so a casual editor
    // can't quietly strip the licence clause without the test failing.
    expect(source).toContain("creativecommons.org/publicdomain/zero/1.0");
  });

  it("SOURCE.md identifies the source as programmatic synthesis and links to the generator script", () => {
    const source = readFileSync(SOURCE_MD_PATH, "utf8");
    expect(source.toLowerCase()).toContain("programmatic synthesis");
    expect(source).toContain("build-avatar.mjs");
  });
});