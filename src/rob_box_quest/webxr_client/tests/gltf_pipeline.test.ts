import { describe, it, expect } from "vitest";
import { existsSync, readFileSync, statSync } from "node:fs";
import { resolve } from "node:path";
import { NodeIO } from "@gltf-transform/core";
import { ALL_EXTENSIONS } from "@gltf-transform/extensions";
import draco3d from "draco3dgltf";
import { MeshoptDecoder } from "meshoptimizer";

// gltf-pipeline smoke test (Phase 2.0, ADR-0032 §3.1).
//
// Loads `public/models/environment/Duck.optimized.glb` (committed artifact
// of `npm run gltf:optimize` applied to a CC0 Khronos Sample Asset Duck.glb)
// through gltf-transform's NodeIO + Draco + Meshopt decoders, and asserts
// that:
//   - the asset is reachable on disk;
//   - the file parses without throwing;
//   - the document declares KHR_draco_mesh_compression and
//     EXT_meshopt_compression in extensionsUsed (proves the optimizer
//     actually emitted them, not just claimed them);
//   - size is under the environment category budget (ADR-0032 §3.2);
//   - CC0 attribution is documented in CREDITS.md.
//
// Why gltf-transform NodeIO and not Three.js GLTFLoader:
//   Three.js DRACOLoader decodes via a Web Worker (`new Worker(...)`),
//   which jsdom (vitest's default environment) does not implement. The
//   production runtime is a real browser, but the unit test must round-trip
//   through node. gltf-transform uses the same decoders but synchronously,
//   which is exactly what we want here: we're validating the pipeline
//   output (the committed .optimized.glb artifact), not the browser
//   loader path. The browser path is exercised by the WebXR client itself
//   and by manual QA on the Quest device.

const ASSET_PATH = resolve(
  __dirname,
  "..",
  "public",
  "models",
  "environment",
  "Duck.optimized.glb"
);

const CREDITS_PATH = resolve(
  __dirname,
  "..",
  "public",
  "models",
  "environment",
  "CREDITS.md"
);

const ENVIRONMENT_BUDGET_BYTES = 2 * 1024 * 1024; // ≤ 2 MB per ADR-0032 §3.2

describe("gltf-pipeline smoke (Phase 2.0, ADR-0032)", () => {
  it("the optimized Duck.glb asset exists on disk", () => {
    expect(existsSync(ASSET_PATH)).toBe(true);
  });

  it("the Duck.optimized.glb carries CC0 attribution in CREDITS.md", () => {
    // Guard against the asset sneaking in without a license file.
    expect(existsSync(CREDITS_PATH)).toBe(true);
    const credits = readFileSync(CREDITS_PATH, "utf8").toLowerCase();
    expect(credits).toContain("cc0");
    expect(credits).toContain("duck");
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

  it("size is within the environment category budget (≤ 2 MB per ADR-0032 §3.2)", () => {
    const size = statSync(ASSET_PATH).size;
    expect(size).toBeLessThanOrEqual(ENVIRONMENT_BUDGET_BYTES);
    // Sanity: optimized Duck must be smaller than the canonical raw Duck.glb
    // (~228 KB). If we ever go over 200 KB something regressed.
    expect(size).toBeLessThan(200 * 1024);
  });
});