import { NodeIO } from "@gltf-transform/core";
import { stat } from "node:fs/promises";

const f = process.argv[2];
const io = new NodeIO();
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
