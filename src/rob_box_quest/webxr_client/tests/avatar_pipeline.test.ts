import { describe, it, expect } from "vitest";
import { existsSync, readFileSync, statSync } from "node:fs";
import { resolve } from "node:path";
import { NodeIO } from "@gltf-transform/core";
import { ALL_EXTENSIONS } from "@gltf-transform/extensions";
import draco3d from "draco3dgltf";
import { MeshoptDecoder } from "meshoptimizer";

// avatar-pipeline smoke test (Phase 2.1, ADR-0032 §3.2, kanban t_1fa6e505,
// issue #1677; published as canonical `avatar.glb` per kanban t_90c362bd).
//
// Loads `public/models/avatar/avatar.glb` (committed artifact of
// `npm run gltf:optimize` then `cp avatar/_raw/avatar.optimized.glb avatar/avatar.glb`)
// through gltf-transform's NodeIO + Draco + Meshopt decoders, and asserts:
//   - the asset is reachable on disk;
//   - the file parses without throwing;
//   - the document declares KHR_draco_mesh_compression and
//     EXT_meshopt_compression in extensionsUsed (proves the optimizer
//     actually emitted them, not just claimed them);
//   - the document lists the 5 expected animation names
//     (idle, drive_forward, drive_backward, turn_left, turn_right);
//   - every animation has non-zero duration (no empty clips);
//   - size is under the avatar category budget (500 KB per ADR-0032 §3.2);
//   - CC0 attribution is documented in CREDITS.md.
//
// Mirrors the structure of gltf_pipeline.test.ts (Phase 2.0) — same NodeIO
// + Draco + Meshopt decode path, same jsdom-friendly synchronous round-trip.

const ASSET_PATH = resolve(
  __dirname,
  "..",
  "public",
  "models",
  "avatar",
  "avatar.glb"
);

const CREDITS_PATH = resolve(
  __dirname,
  "..",
  "public",
  "models",
  "avatar",
  "CREDITS.md"
);

const AVATAR_BUDGET_BYTES = 500 * 1024; // ≤ 500 KB per ADR-0032 §3.2

const EXPECTED_ANIMATIONS = [
  "idle",
  "drive_forward",
  "drive_backward",
  "turn_left",
  "turn_right",
];

describe("avatar-pipeline smoke (Phase 2.1, ADR-0032 §3.2)", () => {
  it("the optimised avatar.glb asset exists on disk", () => {
    expect(existsSync(ASSET_PATH)).toBe(true);
  });

  it("the avatar asset carries CC0 attribution in CREDITS.md", () => {
    // Guard against the asset sneaking in without a license file.
    expect(existsSync(CREDITS_PATH)).toBe(true);
    const credits = readFileSync(CREDITS_PATH, "utf8").toLowerCase();
    expect(credits).toContain("cc0");
    expect(credits).toContain("avatar");
  });

  it("parses through gltf-transform + Draco + Meshopt decoders without errors", async () => {
    if (MeshoptDecoder?.ready) {
      await MeshoptDecoder.ready;
    }
    const draco = await draco3d.createDecoderModule();

    const io = new NodeIO()
      .registerExtensions(ALL_EXTENSIONS)
      .registerDependencies({
        "draco3d.decoder": draco,
        "meshopt.decoder": MeshoptDecoder,
      });

    const doc = await io.read(ASSET_PATH);

    const usedExtensions = doc
      .getRoot()
      .listExtensionsUsed()
      .map((e) => e.extensionName);

    expect(usedExtensions).toContain("KHR_draco_mesh_compression");
    expect(usedExtensions).toContain("EXT_meshopt_compression");
  });

  it("declares the 5 expected animation clips", async () => {
    if (MeshoptDecoder?.ready) {
      await MeshoptDecoder.ready;
    }
    const draco = await draco3d.createDecoderModule();

    const io = new NodeIO()
      .registerExtensions(ALL_EXTENSIONS)
      .registerDependencies({
        "draco3d.decoder": draco,
        "meshopt.decoder": MeshoptDecoder,
      });

    const doc = await io.read(ASSET_PATH);
    const animNames = doc.getRoot().listAnimations().map((a) => a.getName());

    for (const expected of EXPECTED_ANIMATIONS) {
      expect(animNames).toContain(expected);
    }
  });

  it("every animation has a non-zero duration (no empty clips)", async () => {
    if (MeshoptDecoder?.ready) {
      await MeshoptDecoder.ready;
    }
    const draco = await draco3d.createDecoderModule();

    const io = new NodeIO()
      .registerExtensions(ALL_EXTENSIONS)
      .registerDependencies({
        "draco3d.decoder": draco,
        "meshopt.decoder": MeshoptDecoder,
      });

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

  it("size is within the avatar category budget (≤ 500 KB per ADR-0032 §3.2)", () => {
    const size = statSync(ASSET_PATH).size;
    expect(size).toBeLessThanOrEqual(AVATAR_BUDGET_BYTES);
  });
});