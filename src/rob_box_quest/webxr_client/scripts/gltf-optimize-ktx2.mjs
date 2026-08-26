#!/usr/bin/env node
// gltf-optimize-ktx2.mjs
//
// Phase 2.1: KTX2 (Basis) texture compression pass for rob_box_quest.
// Wraps `ktx2-encoder`'s gltf-transform integration (`@gltf-transform/cli`
// already supports this, but we expose it as a stand-alone Node script so
// the webxr_client pipeline stays declarative and testable).
//
// Inputs:
//   - One or more `<name>.optimized.glb` paths (already through Draco +
//     Meshopt). We rewrite textures → KTX2 + declare KHR_texture_basisu.
//   - With no args: walks `public/models/<category>/` and processes every
//     `*.optimized.glb`.
//
// Outputs:
//   - Same path with the `.optimized` segment replaced by `.ktx2`. E.g.
//     `avatar.optimized.glb` → `avatar.ktx2.glb`.
//
// Why this is a separate step from gltf-optimize.mjs:
//   - The KTX2 transform relies on a WASM basis encoder
//     (`ktx2-encoder`) — heavy at ~30 MB WASM. Pulling it into the
//     always-on Draco+Meshopt pass would slow CI for assets that don't
//     need KTX2 (most environment / panel assets are fine with WebP).
//   - Per ADR-0032 §3.2 the avatar is the one asset that MUST land on a
//     Meta Quest 3 GPU upload fast — that's where KTX2 earns its keep.
//
// Usage:
//   node scripts/gltf-optimize-ktx2.mjs                                       # all *.optimized.glb
//   node scripts/gltf-optimize-ktx2.mjs path/to/foo.optimized.glb             # single file
//   node scripts/gltf-optimize-ktx2.mjs --slots baseColorTexture normalTexture # restrict to slots

import { NodeIO } from "@gltf-transform/core";
import { ALL_EXTENSIONS } from "@gltf-transform/extensions";
import { ktx2 } from "ktx2-encoder/gltf-transform";
import draco3d from "draco3dgltf";
import { MeshoptDecoder, MeshoptEncoder } from "meshoptimizer";
import sharp from "sharp";
import { readdir, stat, writeFile, mkdir } from "node:fs/promises";
import { dirname, join, relative, extname, basename } from "node:path";
import { fileURLToPath } from "node:url";
import { existsSync } from "node:fs";

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);
const DEFAULT_INPUT_DIR = join(__dirname, "..", "public", "models");

// `avatar.optimized.glb` → `avatar.ktx2.glb`
const INPUT_SUFFIX = ".optimized.glb";
const OUTPUT_SUFFIX = ".ktx2.glb";

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

async function listInputs(root) {
  const out = [];
  async function walk(dir) {
    let entries;
    try {
      entries = await readdir(dir, { withFileTypes: true });
    } catch (err) {
      if (err.code === "ENOENT") return;
      throw err;
    }
    for (const entry of entries) {
      const full = join(dir, entry.name);
      if (entry.isDirectory()) {
        await walk(full);
      } else if (extname(entry.name).toLowerCase() === ".glb" && entry.name.endsWith(INPUT_SUFFIX)) {
        out.push(full);
      }
    }
  }
  await walk(root);
  return out;
}

async function processFile(inputPath, io, transform) {
  const doc = await io.read(inputPath);
  const beforeBytes = (await stat(inputPath)).size;
  await doc.transform(transform);
  const outPath = inputPath.replace(INPUT_SUFFIX, OUTPUT_SUFFIX);
  await ensureDir(dirname(outPath));
  const glb = await io.writeBinary(doc);
  await writeFile(outPath, glb);
  const afterBytes = glb.byteLength;
  return { inputPath, outPath, beforeBytes, afterBytes };
}

async function main() {
  const argv = process.argv.slice(2);
  const explicitInputs = argv.filter((a) => !a.startsWith("--"));
  const slotsIdx = argv.indexOf("--slots");
  const slotsArg = slotsIdx >= 0 ? argv[slotsIdx + 1] : null;
  const slotsRe = slotsArg ? new RegExp(`^(?:${slotsArg.split(",").map((s) => s.trim()).join("|")})$`) : null;

  let inputs;
  if (explicitInputs.length > 0) {
    inputs = explicitInputs;
    // Sanity-check: explicit inputs MUST end with `.optimized.glb`. We
    // refuse to silently overwrite a raw `.glb` — the KTX2 pass produces
    // a separate `.ktx2.glb` artifact.
    const bad = inputs.filter((p) => !p.endsWith(INPUT_SUFFIX));
    if (bad.length > 0) {
      console.error(`[gltf-optimize-ktx2] Error: explicit input(s) must end with "${INPUT_SUFFIX}":`);
      for (const p of bad) console.error(`  - ${p}`);
      console.error(`[gltf-optimize-ktx2] Run \`npm run gltf:optimize\` first (or use scripts/build-avatar.mjs which already applies Draco+Meshopt).`);
      process.exit(2);
    }
  } else {
    inputs = await listInputs(DEFAULT_INPUT_DIR);
  }

  if (inputs.length === 0) {
    console.log(`[gltf-optimize-ktx2] No .optimized.glb assets found under ${DEFAULT_INPUT_DIR}`);
    console.log(`[gltf-optimize-ktx2] Run \`npm run gltf:optimize\` first to produce Draco+Meshopt outputs.`);
    process.exit(0);
  }

  if (MeshoptDecoder?.ready) await MeshoptDecoder.ready;
  if (MeshoptEncoder?.ready) await MeshoptEncoder.ready;

  const io = new NodeIO()
    .registerExtensions(ALL_EXTENSIONS)
    .registerDependencies({
      // We need BOTH encoders to rewrite the .glb: draco for the
      // geometry pass that produced `.optimized.glb` in the first
      // place (the transform pipeline doesn't decompress meshes, so
      // we must re-encode them as-is), and meshopt for the same reason.
      "draco3d.encoder": await draco3d.createEncoderModule(),
      "draco3d.decoder": await draco3d.createDecoderModule(),
      "meshopt.encoder": MeshoptEncoder,
      "meshopt.decoder": MeshoptDecoder,
    });

  // `ktx2-encoder` needs an imageDecoder on Node.js (no DOM canvas). We
  // hand it a sharp-based decoder that yields { format, width, height,
  // data } — matching the basis encoder's LDR input contract.
  const sharpImageDecoder = async (buffer) => {
    const img = sharp(buffer);
    const meta = await img.metadata();
    const channels = meta.channels ?? 4;
    const { data } = await img.ensureAlpha().raw().toBuffer({ resolveWithObject: true });
    // `ktx2-encoder` expects format strings it knows: 'R8', 'RG8',
    // 'RGB8', 'RGBA8', 'RGBA16F'. PNG/JPEG with alpha → RGBA8.
    const format = channels >= 4 ? "RGBA8" : channels === 3 ? "RGB8" : channels === 2 ? "RG8" : "R8";
    return { format, width: meta.width, height: meta.height, data };
  };

  // UASTC for albedo/normal (high quality for hero asset); ETC1S for
  // metallic-roughness (channel-packed, fine with smaller bitrate).
  // We don't currently distinguish by slot — the `ktx2()` transform
  // matches every PNG/JPEG/WebP texture; quality=128 + UASTC is the
  // right default for an avatar with three procedural textures.
  const transform = ktx2({
    compression: "UASTC",
    quality: 128,
    srgb: true,
    mipmaps: true,
    imageDecoder: sharpImageDecoder,
    slots: slotsRe,
  });

  console.log(`[gltf-optimize-ktx2] Processing ${inputs.length} asset(s) → KTX2 (UASTC, mipmaps)…`);
  let totalBefore = 0;
  let totalAfter = 0;
  let failures = 0;
  let ktxCount = 0;

  for (const input of inputs) {
    const rel = relative(process.cwd(), input);
    try {
      const { outPath, beforeBytes, afterBytes } = await processFile(input, io, transform);
      totalBefore += beforeBytes;
      totalAfter += afterBytes;
      const outRel = relative(process.cwd(), outPath);
      console.log(`  ${rel} → ${outRel}  (${fmtBytes(beforeBytes)} → ${fmtBytes(afterBytes)}, ${fmtPct(beforeBytes, afterBytes)})`);
      // Count KTX2 textures in the output for the summary
      const outDoc = await io.read(outPath);
      const ktx2Count = outDoc.getRoot().listTextures().filter((t) => t.getMimeType() === "image/ktx2").length;
      ktxCount += ktx2Count;
    } catch (err) {
      failures += 1;
      console.error(`  ✗ ${rel}: ${err.message}`);
    }
  }

  console.log(
    `[gltf-optimize-ktx2] Done. Total: ${fmtBytes(totalBefore)} → ${fmtBytes(totalAfter)} (${fmtPct(totalBefore, totalAfter)}). ${ktxCount} texture(s) on KTX2. ${failures} failure(s).`
  );
  if (failures > 0) process.exit(1);
}

main().catch((err) => {
  console.error("[gltf-optimize-ktx2] FATAL:", err);
  process.exit(2);
});
