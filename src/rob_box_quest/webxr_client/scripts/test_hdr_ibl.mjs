#!/usr/bin/env node
// test_hdr_ibl.mjs — проверка, что bridge_env_1k.hdr парсится через RGBELoader
// (это первый шаг IBL pipeline; PMREMGenerator требует WebGL — это runtime,
// в Phase 2.0 отдельная карточка. Здесь мы только валидируем, что HDR корректный.)

import * as THREE from "three";
import { RGBELoader } from "three/examples/jsm/loaders/RGBELoader.js";
import { RoomEnvironment } from "three/examples/jsm/environments/RoomEnvironment.js";
import { promises as fs } from "node:fs";
import path from "node:path";

const HDR_PATH = path.resolve("public/assets/bridge/bridge_env_1k.hdr");

console.log("[hdr_ibl_test] валидация bridge_env_1k.hdr как IBL source");

const buf = await fs.readFile(HDR_PATH);
console.log(`  file size: ${(buf.byteLength / 1024).toFixed(1)} KB`);

const magic = buf.toString("ascii", 0, 10);
console.log(`  magic: "${magic}" (ожидаем '#?RADIANCE')`);
if (!magic.startsWith("#?RADIANCE")) {
  console.error("  FAIL: not Radiance HDR format");
  process.exit(1);
}

const loader = new RGBELoader();
let tex;
try {
  // parse() возвращает DataTexture синхронно (не Promise)
  tex = loader.parse(buf);
} catch (e) {
  console.error("  FAIL: RGBELoader.parse error:", e.message);
  process.exit(1);
}

console.log(`  RGBELoader.parse: OK`);
console.log(`  format: ${tex.format}, type: ${tex.type}, isDataTexture: ${tex.isDataTexture}`);
if (tex.image) {
  const { width, height, data } = tex.image;
  console.log(`  dims: ${width} x ${height}, channels: ${data?.length / (width * height)}`);

  function px(x, y) {
    const i = (y * width + x) * 4;
    return [data[i], data[i + 1], data[i + 2], data[i + 3]];
  }
  console.log(`  px(0,0):       [${px(0, 0).map((v) => v.toFixed(2)).join(", ")}]`);
  console.log(`  px(mid,mid):   [${px(width >> 1, height >> 1).map((v) => v.toFixed(2)).join(", ")}]`);
  console.log(`  px(W-1,0):     [${px(width - 1, 0).map((v) => v.toFixed(2)).join(", ")}]`);
  console.log(`  px(mid,H-1):   [${px(width >> 1, height - 1).map((v) => v.toFixed(2)).join(", ")}]`);

  let maxR = 0, maxG = 0, maxB = 0;
  for (let i = 0; i < data.length; i += 4) {
    if (data[i] > maxR) maxR = data[i];
    if (data[i + 1] > maxG) maxG = data[i + 1];
    if (data[i + 2] > maxB) maxB = data[i + 2];
  }
  console.log(`  max RGB: [${maxR.toFixed(1)}, ${maxG.toFixed(1)}, ${maxB.toFixed(1)}] (ожидаем > 0)`);
  if (maxR === 0 || maxG === 0 || maxB === 0) {
    console.error("  FAIL: HDR содержит нулевые каналы");
    process.exit(1);
  }
}

const ro = new RoomEnvironment();
console.log(`  RoomEnvironment: ${ro.children?.length || "n/a"} children (procedural fallback для тестов без HDR)`);

console.log("[hdr_ibl_test] PASS: HDR валиден как IBL source.");
console.log("  Runtime IBL setup (Phase 2.0):");
console.log("    const pmrem = new THREE.PMREMGenerator(renderer);");
console.log("    pmrem.compileEquirectangularShader();");
console.log("    const env = pmrem.fromEquirectangular(tex).texture;");
console.log("    scene.environment = env;");
