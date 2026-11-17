import { NodeIO } from "@gltf-transform/core";
import { ALL_EXTENSIONS } from "@gltf-transform/extensions";
import draco3d from "draco3dgltf";
import { stat } from "node:fs/promises";

const f = process.argv[2];

const io = new NodeIO()
  .registerExtensions(ALL_EXTENSIONS)
  .registerDependencies({
    "draco3d.decoder": await draco3d.createDecoderModule(),
  });

// Register meshopt decoder so committed EXT_meshopt_compression assets
// (see scripts/gltf-optimize.mjs) can actually be read. Best-effort, same
// as gltf-verify.mjs: skip silently if the package is unavailable and only
// fail if an asset genuinely needs it.
try {
  const mod = await import("meshoptimizer");
  const decoder = mod.MeshoptDecoder;
  if (decoder && typeof decoder.ready !== "undefined") {
    await decoder.ready;
  }
  io.registerDependencies({ "meshopt.decoder": decoder });
} catch {
  // meshopt decoder unavailable — will only fail if the asset actually
  // uses EXT_meshopt_compression without a registered decoder.
}

const doc = await io.read(f);
const r = doc.getRoot();

let t = 0;
for (const m of r.listMeshes()) {
  for (const p of m.listPrimitives()) {
    const i = p.getIndices();
    const pos = p.getAttribute("POSITION");
    t += i ? i.getCount() / 3 : pos ? pos.getCount() / 3 : 0;
  }
}

const b = (await stat(f)).size;
console.log(`file ${(b / 1024 / 1024).toFixed(2)} MB | tris ${Math.round(t).toLocaleString()} | meshes ${r.listMeshes().length} | materials ${r.listMaterials().length}`);
for (const x of r.listTextures()) {
  const s = x.getSize();
  const im = x.getImage();
  console.log(`  tex ${x.getName() || "(unnamed)"} ${s ? s[0] + "x" + s[1] : "?"} ${x.getMimeType()} ${Math.round((im ? im.byteLength : 0) / 1024)} KB`);
}
