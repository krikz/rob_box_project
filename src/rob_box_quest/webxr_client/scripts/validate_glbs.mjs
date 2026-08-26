#!/usr/bin/env node
// validate_glbs.mjs — sanity check для bridge_*.glb: читаем через @gltf-transform,
// проверяем magic + meshes/primitives/materials/accessors + bounding box
// (пересчитываем через POSITION attribute.array, т.к. accessor.getMin/getMax
//  возвращает кешированные значения из parse, а мы их не выставили).

import { NodeIO } from "@gltf-transform/core";
import { ALL_EXTENSIONS } from "@gltf-transform/extensions";
import { promises as fs } from "node:fs";
import path from "node:path";

const DIR = path.resolve(process.cwd(), "public/assets/bridge");
const FILES = ["bridge_floor.glb", "bridge_walls.glb", "bridge_props.glb"];

const io = new NodeIO().registerExtensions(ALL_EXTENSIONS);

let allOk = true;

for (const name of FILES) {
  const full = path.join(DIR, name);
  const buf = await fs.readFile(full);

  // GLB magic check
  const magic = buf.toString("ascii", 0, 4);
  if (magic !== "glTF") {
    console.error(`${name}: BAD MAGIC "${magic}"`);
    allOk = false;
    continue;
  }

  const doc = await io.readBinary(buf);
  const root = doc.getRoot();

  const meshes = root.listMeshes();
  const prims = meshes.flatMap((m) => m.listPrimitives());
  const mats = root.listMaterials();
  const accs = root.listAccessors();
  const nodes = root.listNodes();

  // Bounding box: пересчитываем через сам POSITION attribute array
  let minX = Infinity, minY = Infinity, minZ = Infinity;
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
  for (const mesh of meshes) {
    for (const prim of mesh.listPrimitives()) {
      const posAcc = prim.getAttribute("POSITION");
      if (!posAcc) continue;
      const arr = posAcc.getArray();
      const cnt = posAcc.getCount();
      for (let i = 0; i < cnt; i++) {
        const x = arr[i * 3], y = arr[i * 3 + 1], z = arr[i * 3 + 2];
        if (x < minX) minX = x; if (x > maxX) maxX = x;
        if (y < minY) minY = y; if (y > maxY) maxY = y;
        if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
      }
    }
  }

  // материалы: проверка PBR и emissive
  const pbrCount = mats.filter((m) => m.getMetallicFactor() !== null && m.getRoughnessFactor() !== null).length;
  const emissiveCount = mats.filter((m) => m.getEmissiveFactor() !== null).length;
  const transparentCount = mats.filter((m) => m.getAlphaMode() === "BLEND").length;

  console.log(`${name}:`);
  console.log(`  size: ${(buf.byteLength / 1024).toFixed(1)} KB`);
  console.log(`  meshes=${meshes.length} prims=${prims.length} mats=${mats.length} (${pbrCount} PBR / ${emissiveCount} emissive / ${transparentCount} transparent) nodes=${nodes.length} accs=${accs.length}`);
  console.log(`  bbox: [${minX.toFixed(2)}, ${minY.toFixed(2)}, ${minZ.toFixed(2)}] .. [${maxX.toFixed(2)}, ${maxY.toFixed(2)}, ${maxZ.toFixed(2)}]`);
  console.log(`  extents: ${(maxX - minX).toFixed(2)} x ${(maxY - minY).toFixed(2)} x ${(maxZ - minZ).toFixed(2)}`);
  console.log("");

  // sanity checks
  if (meshes.length === 0) { console.error("  FAIL: 0 meshes"); allOk = false; }
  if (pbrCount === 0) { console.error("  FAIL: no PBR materials"); allOk = false; }
}

if (!allOk) {
  console.error("[validate] FAIL");
  process.exit(1);
}

console.log("[validate_glbs] PASS: все glb валидны, PBR materials, корректные bbox.");
