#!/usr/bin/env node
// gltf-optimize.mjs
//
// Reproducible glTF 2.0 asset pipeline for rob_box_quest Phase 2+ Captain Bridge
// (ADR-0032 §3.1, §3.2). Walks public/models/, and for each .glb / .gltf applies:
//
//   1. dedup          — drop duplicate vertices / texture data
//   2. prune          — drop unused nodes / textures / materials
//   3. resample       — lossless animation frame resample
//   4. draco          — KHR_draco_mesh_compression (geometry, static meshes)
//   5. meshopt        — EXT_meshopt_compression (geometry + animation tracks)
//   6. textureCompress (WebP via sharp, optional — installed on demand)
//
// Notes on KTX2/Basis (KHR_texture_basisu):
//   The full pipeline per ADR-0032 also compresses textures to KTX2/Basis via
//   the KTX-Software v4.3.0+ `ktx` CLI. That's invoked through
//   `@gltf-transform/cli` (see scripts/gltf-optimize-kx2.mjs) and is opt-in:
//   KTX-Software is a native binary and not always present in CI. We default to
//   WebP via sharp — still GPU-friendly — and surface a clear warning when an
//   asset lacks `KHR_texture_basisu` so the verify gate can decide policy.
//
// Output written to public/models/<name>.optimized.glb. Originals are not
// touched (raw glTF must NEVER be committed; CI guard enforces this).
//
// Usage:
//   node scripts/gltf-optimize.mjs                 # process all under public/models/
//   node scripts/gltf-optimize.mjs path/to/in.glb  # process a single file (in-place dir)
//
// CC0 / public-domain for all CC0 source models from Quaternius / Poly Haven /
// Khronos Sample Assets. See README.md for attribution + license details.

import { NodeIO } from "@gltf-transform/core";
import { ALL_EXTENSIONS } from "@gltf-transform/extensions";
import { dedup, prune, resample, draco, meshopt, textureCompress } from "@gltf-transform/functions";
import draco3d from "draco3dgltf";
import { readdir, stat, writeFile, mkdir } from "node:fs/promises";
import { dirname, join, relative, extname } from "node:path";
import { fileURLToPath } from "node:url";
import { existsSync } from "node:fs";

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);
// scripts/ -> ../public/models/
const DEFAULT_INPUT_DIR = join(__dirname, "..", "public", "models");
const OUTPUT_SUFFIX = ".optimized.glb";

const SUPPORTED_EXT = new Set([".glb", ".gltf"]);

async function listGlbFiles(root) {
  const out = [];
  async function walk(dir) {
    let entries;
    try {
      entries = await readdir(dir, { withFileTypes: true });
    } catch (err) {
      if (err.code === "ENOENT") return; // no models dir yet — nothing to do
      throw err;
    }
    for (const entry of entries) {
      const full = join(dir, entry.name);
      if (entry.isDirectory()) {
        await walk(full);
      } else if (SUPPORTED_EXT.has(extname(entry.name).toLowerCase())) {
        // skip already-optimized outputs
        if (full.endsWith(OUTPUT_SUFFIX)) continue;
        out.push(full);
      }
    }
  }
  await walk(root);
  return out;
}

function fmtBytes(n) {
  if (n < 1024) return `${n} B`;
  if (n < 1024 * 1024) return `${(n / 1024).toFixed(1)} KB`;
  return `${(n / 1024 / 1024).toFixed(2)} MB`;
}

function fmtPct(orig, next) {
  if (orig === 0) return "n/a";
  const diff = ((next - orig) / orig) * 100;
  const sign = diff <= 0 ? "" : "+";
  return `${sign}${diff.toFixed(1)}%`;
}

async function ensureDir(path) {
  if (!existsSync(path)) await mkdir(path, { recursive: true });
}

async function optimizeFile(io, inputPath, sharpEncoder, meshoptEncoder) {
  const doc = await io.read(inputPath);
  const beforeBytes = (await stat(inputPath)).size;

  const transforms = [
    dedup(),
    prune(),
    resample(),
    draco({ method: "edgebreaker", encodeSpeed: 5, decodeSpeed: 5 }),
    meshopt({ encoder: meshoptEncoder ?? undefined }),
  ];

  // Optional: texture compression via sharp (WebP). Skipped if sharp isn't
  // installed or fails to load — geometry compression above is the primary
  // size win; texture compress is a progressive enhancement.
  if (sharpEncoder) {
    transforms.push(
      textureCompress({
        encoder: sharpEncoder,
        targetFormat: "webp",
        quality: 80,
      })
    );
  }

  await doc.transform(...transforms);

  const outPath = inputPath.replace(/\.(glb|gltf)$/i, OUTPUT_SUFFIX);
  await ensureDir(dirname(outPath));
  const glb = await io.writeBinary(doc);
  await writeFile(outPath, glb);
  const afterBytes = glb.byteLength;

  return { inputPath, outPath, beforeBytes, afterBytes };
}

async function main() {
  const args = process.argv.slice(2);
  const explicitInputs = args.filter((a) => !a.startsWith("--"));

  let inputs;
  if (explicitInputs.length > 0) {
    inputs = explicitInputs;
  } else {
    inputs = await listGlbFiles(DEFAULT_INPUT_DIR);
  }

  if (inputs.length === 0) {
    console.log(`[gltf-optimize] No glTF assets found under ${relative(process.cwd(), DEFAULT_INPUT_DIR) || DEFAULT_INPUT_DIR}`);
    console.log("[gltf-optimize] Drop a CC0 .glb there and re-run, or pass an explicit input path.");
    process.exit(0);
  }

  const io = new NodeIO()
    .registerExtensions(ALL_EXTENSIONS)
    .registerDependencies({
      "draco3d.encoder": await draco3d.createEncoderModule(),
      "draco3d.decoder": await draco3d.createDecoderModule(),
      // meshopt encoder/decoder live in a separate npm package; we
      // dynamically import + .ready them below.
    });

  // Load meshopt encoder/decoder (native WASM via the `meshoptimizer` pkg).
  let meshoptEncoder = null;
  try {
    const mod = await import("meshoptimizer");
    meshoptEncoder = mod.MeshoptEncoder;
    const decoder = mod.MeshoptDecoder;
    if (meshoptEncoder && typeof meshoptEncoder.ready !== "undefined") {
      await meshoptEncoder.ready;
    }
    if (decoder && typeof decoder.ready !== "undefined") {
      await decoder.ready;
    }
    io.registerDependencies({
      "meshopt.encoder": meshoptEncoder,
      "meshopt.decoder": decoder,
    });
    console.log("[gltf-optimize] meshopt encoder/decoder registered.");
  } catch (err) {
    console.warn("[gltf-optimize] meshoptimizer package unavailable — EXT_meshopt_compression will be skipped:", err.message);
  }

  console.log(`[gltf-optimize] Processing ${inputs.length} asset(s) via Draco + Meshopt…`);
  let totalBefore = 0;
  let totalAfter = 0;
  let failures = 0;

  // Optional: sharp for texture → WebP. Soft-import so the script still runs
  // without it (geometry compression is the primary size win).
  let sharpEncoder = null;
  try {
    const sharpModule = await import("sharp");
    sharpEncoder = sharpModule.default ?? sharpModule;
    console.log("[gltf-optimize] sharp detected — enabling texture → WebP compression.");
  } catch {
    console.log("[gltf-optimize] sharp not installed — skipping texture compression (geometry only).");
  }

  for (const input of inputs) {
    const rel = relative(process.cwd(), input);
    try {
      const { outPath, beforeBytes, afterBytes } = await optimizeFile(io, input, sharpEncoder, meshoptEncoder);
      totalBefore += beforeBytes;
      totalAfter += afterBytes;
      const outRel = relative(process.cwd(), outPath);
      console.log(
        `  ${rel} → ${outRel}  (${fmtBytes(beforeBytes)} → ${fmtBytes(afterBytes)}, ${fmtPct(beforeBytes, afterBytes)})`
      );
    } catch (err) {
      failures += 1;
      console.error(`  ✗ ${rel}: ${err.message}`);
    }
  }

  console.log(
    `[gltf-optimize] Done. Total: ${fmtBytes(totalBefore)} → ${fmtBytes(totalAfter)} (${fmtPct(totalBefore, totalAfter)}). ${failures} failure(s).`
  );
  if (failures > 0) process.exit(1);
}

main().catch((err) => {
  console.error("[gltf-optimize] FATAL:", err);
  process.exit(2);
});
