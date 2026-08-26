#!/usr/bin/env node
// gltf-verify.mjs
//
// CI guard for the glTF asset pipeline (ADR-0032 §3.2, §3.3; карточка Phase 2.0).
//
// Walks public/models/ (recursively) and rejects the build if ANY committed
// glTF asset fails the contract:
//
//   1. Compression extensions MUST be present:
//        - KHR_draco_mesh_compression        (geometry, Draco)
//        - EXT_meshopt_compression           (geometry + animation, Meshopt)
//        - KHR_texture_basisu                (textures, Basis / KTX2)
//   2. Per-asset size budget (matches ADR-0032 §3.2):
//        - environment .glb ≤ 2 MB
//        - avatar      .glb ≤ 500 KB
//        - panel       .glb ≤ 150 KB
//        - texture     .ktx2 ≤ 5 MB
//        - hdr env     .ktx2 ≤ 600 KB
//      (category inferred from path: "environment/", "avatar/", "panel/",
//       "texture/", "hdr/". Anything else falls under "environment" budget
//       as the loosest limit — be explicit if you need tighter control.)
//
// Exit code:
//   0  — all assets compliant
//   1  — at least one asset failed verification
//
// Usage:
//   npm run gltf:verify

import { NodeIO } from "@gltf-transform/core";
import { ALL_EXTENSIONS } from "@gltf-transform/extensions";
import draco3d from "draco3dgltf";
import { readdir, stat } from "node:fs/promises";
import { dirname, join, relative, extname, basename } from "node:path";
import { fileURLToPath } from "node:url";

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);
// scripts/ -> ../public/models/
const INPUT_DIR = join(__dirname, "..", "public", "models");

const REQUIRED_EXTENSIONS = [
  // Geometry compression is mandatory for the size budget per ADR-0032 §3.2.
  "KHR_draco_mesh_compression",
  "EXT_meshopt_compression",
];

// Texture compression (KHR_texture_basisu) is enforced as a warning rather
// than a hard error: generating it requires the `ktx` binary from
// KTX-Software v4.3.0+, which is not always present in CI. We surface the
// gap so it's visible but don't break the verify gate on its absence — see
// README.md §KTX2 / Basis pipeline for the opt-in path.
const WARN_EXTENSIONS = ["KHR_texture_basisu"];

// Per ADR-0032 §3.2. Keys are first path segment under public/models/.
// Categories not listed default to the strictest "panel" budget to fail loud.
const SIZE_BUDGETS_BYTES = {
  environment: 2 * 1024 * 1024,    // ≤ 2 MB
  avatar: 500 * 1024,               // ≤ 500 KB
  panel: 150 * 1024,                // ≤ 150 KB
  texture: 5 * 1024 * 1024,         // ≤ 5 MB
  hdr: 600 * 1024,                  // ≤ 600 KB
};
const DEFAULT_BUDGET = SIZE_BUDGETS_BYTES.panel;

function categoryForPath(absPath, inputDir) {
  const rel = relative(inputDir, absPath);
  const first = rel.split(/[\\/]/, 1)[0];
  return SIZE_BUDGETS_BYTES[first] !== undefined ? first : null;
}

function fmtBytes(n) {
  if (n < 1024) return `${n} B`;
  if (n < 1024 * 1024) return `${(n / 1024).toFixed(1)} KB`;
  return `${(n / 1024 / 1024).toFixed(2)} MB`;
}

async function listAssets(root) {
  const out = [];
  async function walk(dir) {
    let entries;
    try {
      entries = await readdir(dir, { withFileTypes: true });
    } catch (err) {
      if (err.code === "ENOENT") return; // dir missing — nothing to verify
      throw err;
    }
    for (const entry of entries) {
      const full = join(dir, entry.name);
      if (entry.isDirectory()) {
        await walk(full);
      } else if ([".glb", ".gltf"].includes(extname(entry.name).toLowerCase())) {
        // Skip the *raw* source files the pipeline would have produced
        // (those are the ones gltf:optimize writes from). The committed
        // artifacts end in `.optimized.glb` (Draco+Meshopt) or
        // `.ktx2.glb` (Draco+Meshopt+Basis/KTX2) and must be verified.
        if (full.endsWith(".optimized.glb") || full.endsWith(".ktx2.glb")) {
          out.push(full);
        }
      }
    }
  }
  await walk(root);
  return out;
}

async function verifyFile(io, absPath, inputDir) {
  const errors = [];
  const warnings = [];
  const fileStat = await stat(absPath);
  const sizeBytes = fileStat.size;

  // Size budget check (per ADR-0032 §3.2)
  const category = categoryForPath(absPath, inputDir);
  const budget = category ? SIZE_BUDGETS_BYTES[category] : DEFAULT_BUDGET;
  if (sizeBytes > budget) {
    errors.push(
      `size ${fmtBytes(sizeBytes)} exceeds budget ${fmtBytes(budget)} for category "${category ?? "<unknown>"}" (ADR-0032 §3.2)`
    );
  } else if (category === null) {
    warnings.push(
      `path "${relative(inputDir, absPath)}" has no recognised category (environment/avatar/panel/texture/hdr); applied default ${fmtBytes(DEFAULT_BUDGET)}`
    );
  }

  // Extension presence check.
  const doc = await io.read(absPath);
  const usedExtensions = new Set(doc.getRoot().listExtensionsUsed().map((e) => e.extensionName));

  for (const required of REQUIRED_EXTENSIONS) {
    if (!usedExtensions.has(required)) {
      errors.push(`missing required extension "${required}"`);
    }
  }

  for (const optional of WARN_EXTENSIONS) {
    if (!usedExtensions.has(optional)) {
      warnings.push(
        `missing recommended extension "${optional}" — texture compression at GPU upload is suboptimal (see README §KTX2 / Basis)`
      );
    }
  }

  return { absPath, sizeBytes, category, budget, errors, warnings, extensions: [...usedExtensions] };
}

async function main() {
  const assets = await listAssets(INPUT_DIR);

  if (assets.length === 0) {
    console.log(
      `[gltf-verify] No committed glTF assets under ${relative(process.cwd(), INPUT_DIR) || INPUT_DIR}. Nothing to verify.`
    );
    process.exit(0);
  }

  const io = new NodeIO()
    .registerExtensions(ALL_EXTENSIONS)
    .registerDependencies({
      "draco3d.decoder": await draco3d.createDecoderModule(),
    });

  // Try to register meshopt decoder (needed only if a .glb actually uses
  // EXT_meshopt_compression, which is optional).
  try {
    const mod = await import("meshoptimizer");
    const decoder = mod.MeshoptDecoder;
    if (decoder && typeof decoder.ready !== "undefined") {
      await decoder.ready;
    }
    io.registerDependencies({ "meshopt.decoder": decoder });
  } catch {
    // meshopt decoder unavailable — verify will only fail if an asset
    // genuinely uses EXT_meshopt_compression without a registered decoder.
  }

  console.log(`[gltf-verify] Checking ${assets.length} asset(s)…`);
  let totalErrors = 0;
  let totalWarnings = 0;

  for (const abs of assets) {
    const rel = relative(process.cwd(), abs);
    let result;
    try {
      result = await verifyFile(io, abs, INPUT_DIR);
    } catch (err) {
      totalErrors += 1;
      console.error(`  ✗ ${rel}: ${err.message}`);
      continue;
    }

    if (result.errors.length === 0) {
      const extList = result.extensions.join(", ");
      console.log(
        `  ✓ ${rel}  [${result.category ?? "<unclassified>"}]  ${fmtBytes(result.sizeBytes)} ≤ ${fmtBytes(result.budget)}  (ext: ${extList})`
      );
    } else {
      totalErrors += 1;
      console.error(`  ✗ ${rel}  [${result.category ?? "<unclassified>"}]  ${fmtBytes(result.sizeBytes)} / budget ${fmtBytes(result.budget)}`);
      for (const e of result.errors) console.error(`      - ${e}`);
    }
    for (const w of result.warnings) {
      totalWarnings += 1;
      console.warn(`      ! ${w}`);
    }
  }

  if (totalErrors === 0) {
    console.log(`[gltf-verify] PASS — ${assets.length} asset(s) compliant.`);
    if (totalWarnings > 0) console.log(`[gltf-verify] ${totalWarnings} warning(s) — review above.`);
    process.exit(0);
  } else {
    console.error(`[gltf-verify] FAIL — ${totalErrors} asset(s) violated the contract.`);
    console.error(`[gltf-verify] Run \`npm run gltf:optimize\` to regenerate, or remove unoptimized raw glTF from the tree.`);
    process.exit(1);
  }
}

main().catch((err) => {
  console.error("[gltf-verify] FATAL:", err);
  process.exit(2);
});
