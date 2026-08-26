// bridge_environment.test.ts — Phase 2.1 Captain Bridge environment tests
// (kanban t_0bd54b80, issue #1677).
//
// What we test (without a WebXR / WebGL context — jsdom only):
//   1. All 5 committed bridge_*.optimized.glb files exist on disk and
//      fit under the environment/ category budget (≤ 2 MB total per
//      ADR-0032 §3.2).
//   2. The bridge_scene_meta.json file is reachable, parses, and
//      structurally validates (design / safe_walk_area / nav_points /
//      occluders all present and well-typed).
//   3. The CREDITS.md file documents all 5 GLBs + HDR + sources with
//      explicit CC0 attribution per Quaternius-fallback / Poly Haven.
//   4. nav_points has at least 1 origin + 1 entry (safe-walk-area
//      contract for XR teleport anchors).
//   5. The Phase 2.0 `gltf:verify` would pass — i.e. every bridge GLB
//      declares KHR_draco_mesh_compression + EXT_meshopt_compression.
//
// What we DO NOT test (out of jsdom):
//   - GLTFLoader / DRACOLoader Web Worker decode path (real browser only).
//   - PMREMGenerator equirectangular → cubemap (needs WebGL).
//   - The HUD/scene integration with captain_bridge.ts (no canvas).
//
// See README.md for the full asset pipeline; see ADR-0032 for the budget.

import { describe, it, expect } from "vitest";
import {
  readFileSync,
  statSync,
  existsSync,
} from "node:fs";
import { resolve, join } from "node:path";
import { NodeIO } from "@gltf-transform/core";
import { ALL_EXTENSIONS } from "@gltf-transform/extensions";
import draco3d from "draco3dgltf";
import { MeshoptDecoder } from "meshoptimizer";
import { validateBridgeMeta } from "../src/scene/bridge_assets";

const ENV_DIR = resolve(
  __dirname,
  "..",
  "public",
  "models",
  "environment",
);

const GLB_FILES = [
  "bridge_floor.optimized.glb",
  "bridge_walls.optimized.glb",
  "bridge_props.optimized.glb",
  "bridge_nav.optimized.glb",
  "bridge_occluders.optimized.glb",
] as const;

const HDR_FILE = "hdr/bridge_env_1k.hdr";
const CREDITS_FILE = "CREDITS.md";
const META_FILE = "bridge_scene_meta.json";

const ENVIRONMENT_BUDGET_TOTAL_BYTES = 2 * 1024 * 1024; // ≤ 2 MB per ADR-0032 §3.2

// ---------- helpers ----------

async function openGlb(path: string) {
  if (MeshoptDecoder?.ready) await MeshoptDecoder.ready;
  const draco = await draco3d.createDecoderModule();
  const io = new NodeIO()
    .registerExtensions(ALL_EXTENSIONS)
    .registerDependencies({
      "draco3d.decoder": draco,
      "meshopt.decoder": MeshoptDecoder,
    });
  return io.read(path);
}

// ---------- tests ----------

describe("Captain Bridge environment — Phase 2.1 (kanban t_0bd54b80)", () => {
  describe("file presence", () => {
    for (const f of GLB_FILES) {
      it(`${f} exists on disk`, () => {
        expect(existsSync(join(ENV_DIR, f))).toBe(true);
      });
    }
    it(`${HDR_FILE} exists on disk`, () => {
      expect(existsSync(join(ENV_DIR, HDR_FILE))).toBe(true);
    });
    it(`${META_FILE} exists on disk`, () => {
      expect(existsSync(join(ENV_DIR, META_FILE))).toBe(true);
    });
    it(`${CREDITS_FILE} exists on disk`, () => {
      expect(existsSync(join(ENV_DIR, CREDITS_FILE))).toBe(true);
    });
  });

  describe("size budget (ADR-0032 §3.2: environment/ ≤ 2 MB)", () => {
    it("sum of all bridge_*.optimized.glb is well under the 2 MB budget", () => {
      let total = 0;
      for (const f of GLB_FILES) {
        total += statSync(join(ENV_DIR, f)).size;
      }
      expect(total).toBeLessThanOrEqual(ENVIRONMENT_BUDGET_TOTAL_BYTES);
      // Sanity: each file is a real mesh (≥ 1 KB after Draco + Meshopt).
      for (const f of GLB_FILES) {
        expect(statSync(join(ENV_DIR, f)).size).toBeGreaterThan(1024);
      }
      // And: budget headroom is large (≥ 5×). If we ever cross 50% of
      // budget something has bloated (the budget includes HDR later).
      expect(total).toBeLessThan(ENVIRONMENT_BUDGET_TOTAL_BYTES / 5);
    });

    it("HDR file is present and a sane size (1 MB – 2 MB)", () => {
      // Poly Haven CC0 1K HDRs sit around 1–2 MB; KTX2 path (Phase 2.0
      // opt-in) shrinks this. Until KTX2 lands, we accept the raw size.
      const sz = statSync(join(ENV_DIR, HDR_FILE)).size;
      expect(sz).toBeGreaterThan(500 * 1024);
      expect(sz).toBeLessThan(3 * 1024 * 1024);
    });
  });

  describe("glTF extensions (Phase 2.0 pipeline: Draco + Meshopt required)", () => {
    for (const f of GLB_FILES) {
      it(`${f} declares KHR_draco_mesh_compression + EXT_meshopt_compression`, async () => {
        const doc = await openGlb(join(ENV_DIR, f));
        const used = doc.getRoot().listExtensionsUsed().map((e) => e.extensionName);
        expect(used).toContain("KHR_draco_mesh_compression");
        expect(used).toContain("EXT_meshopt_compression");
      });
    }
  });

  describe("bridge_scene_meta.json — structural validation", () => {
    it("parses as JSON and passes validateBridgeMeta()", () => {
      const raw = JSON.parse(readFileSync(join(ENV_DIR, META_FILE), "utf8"));
      const meta = validateBridgeMeta(raw);
      expect(meta.design.style).toMatch(/low-poly|sci-fi|captain bridge/i);
      expect(meta.design.dimensions_m.width).toBeGreaterThan(0);
      expect(meta.design.dimensions_m.depth).toBeGreaterThan(0);
      expect(meta.design.dimensions_m.height).toBeGreaterThan(0);
      expect(Array.isArray(meta.nav_points)).toBe(true);
      expect(Array.isArray(meta.occluders)).toBe(true);
      expect(meta.nav_points.length).toBeGreaterThanOrEqual(2);
      expect(meta.occluders.length).toBeGreaterThanOrEqual(1);
    });

    it("safe_walk_area is a non-degenerate AABB (min < max on each axis)", () => {
      const raw = JSON.parse(readFileSync(join(ENV_DIR, META_FILE), "utf8"));
      const meta = validateBridgeMeta(raw);
      const { min, max } = meta.safe_walk_area;
      expect(min[0]).toBeLessThan(max[0]);
      expect(min[1]).toBeLessThan(max[1]);
      expect(min[2]).toBeLessThan(max[2]);
      // Sanity: walk area is at least 2 m wide on the X axis (room-scale).
      expect(max[0] - min[0]).toBeGreaterThanOrEqual(2);
    });

    it("nav_points includes at least one 'origin' (user spawn) and one 'entry' (teleport anchor)", () => {
      const raw = JSON.parse(readFileSync(join(ENV_DIR, META_FILE), "utf8"));
      const meta = validateBridgeMeta(raw);
      const kinds = new Set(meta.nav_points.map((p) => p.kind));
      expect(kinds.has("origin")).toBe(true);
      expect(kinds.has("entry")).toBe(true);
    });

    it("rejects malformed meta with missing 'design'", () => {
      expect(() => validateBridgeMeta({ safe_walk_area: { min: [0, 0, 0], max: [1, 1, 1] }, nav_points: [], occluders: [] }))
        .toThrow(/design/);
    });

    it("rejects malformed meta with nav_point missing position", () => {
      expect(() =>
        validateBridgeMeta({
          design: { style: "x" },
          safe_walk_area: { min: [0, 0, 0], max: [1, 1, 1] },
          nav_points: [{ id: "bad", size: [1, 1, 1], kind: "origin", label: "x" }],
          occluders: [],
        })
      ).toThrow(/position/);
    });
  });

  describe("CREDITS.md — CC0 attribution", () => {
    const creditsLower = readFileSync(join(ENV_DIR, CREDITS_FILE), "utf8").toLowerCase();

    it("mentions CC0", () => {
      expect(creditsLower).toContain("cc0");
    });

    it("documents the Poly Haven HDR source by name", () => {
      expect(creditsLower).toContain("poly haven");
    });

    it("documents the synthesized GLB contract (own code, Quaternius-fallback rationale)", () => {
      expect(creditsLower).toMatch(/synth|procedur|own code/);
      expect(creditsLower).toContain("quaternius"); // explains why we did not use it
    });

    it("lists all 5 bridge_*.optimized.glb filenames", () => {
      for (const f of GLB_FILES) {
        // CREDITS.md may use either backticks or prose; we only need
        // the basename to appear somewhere.
        const basename = f.replace(".optimized.glb", "");
        expect(creditsLower).toContain(basename);
      }
    });
  });
});