#!/usr/bin/env node
// build-avatar.mjs
//
// Синтез CC0 4-колёсного робота-аватара для rob_box_quest/webxr_client
// (Phase 2.1, kanban t_1fa6e505, issue #1677).
//
// Программная генерация .glb через three.js примитивы + @gltf-transform/core.
// Без внешних зависимостей (никаких STL, никаких Blender-экспортов).
//
// Выход: public/models/avatar/_raw/avatar.glb
//   Затем Phase 2.0 pipeline (`npm run gltf:optimize`) пережмёт его в
//   `avatar.optimized.glb` с Draco + Meshopt.
//
// Геометрия (low-poly ~700 треугольников):
//   - chassis (Box 0.6×0.18×0.8)            — корпус, PBR grey
//   - 4 × wheel (Cylinder R=0.16, h=0.12, 16 segments)  — black rubber
//   - head (Box 0.28×0.18×0.18)              — камера-сенсор, PBR cyan
//   - antenna mast (Cylinder R=0.012, h=0.22) — антенна
//   - antenna tip (Sphere R=0.025)           — emissive LED-маркер
//
// Анимации (5 шт, LOOP, LINEAR interpolation):
//   - idle (3.0s)            — body bobbing ±1 см по Z + head yaw ±10° (sens­or scan)
//   - drive_forward (1.0s)   — все 4 колеса вращаются +π/с вокруг X
//   - drive_backward (1.0s)  — все 4 колеса вращаются −π/с
//   - turn_left (1.0s)       — front ×1.5 скорости, rear ×0.5 скорости
//   - turn_right (1.0s)      — зеркально turn_left
//
// Лицензия: CC0 (вся геометрия — собственный код, без внешних моделей).

import * as THREE from "three";
import { Document, NodeIO } from "@gltf-transform/core";
import { ALL_EXTENSIONS } from "@gltf-transform/extensions";
import { mkdir, writeFile, rename, unlink } from "node:fs/promises";
import path from "node:path";

const OUT_DIR = path.resolve(
  process.cwd(),
  "public/models/avatar/_raw"
);
const FINAL_DIR = path.resolve(
  process.cwd(),
  "public/models/avatar"
);
const RAW_FILE = path.join(OUT_DIR, "avatar.glb");
const OPTIMIZED_FILE = path.join(OUT_DIR, "avatar.optimized.glb");
const FINAL_FILE = path.join(FINAL_DIR, "avatar.optimized.glb");
const RAW_BUDGET_BYTES = 100 * 1024; // raw budget: 100 KB до optimization

const FRAMES = 16;            // frames per second-ish (для всех drive/turn animations)
const IDLE_FRAMES = 32;       // higher fidelity для idle (плавный sensor scan)
const IDLE_DURATION = 3.0;    // 3-секундный цикл idle
const DRIVE_DURATION = 1.0;   // 1-секундный цикл drive
const TURN_DURATION = 1.0;    // 1-секундный цикл turn

const COL = {
  chassis: 0x9aa3ad,    // light grey-blue (PBR body)
  wheel: 0x1d2027,      // dark rubber
  head: 0x0a4d4d,       // teal sensor head
  headEdge: 0x44ddff,   // cyan edge accent
  mast: 0x2a2e36,       // dark mast
  tip: 0xff5252,        // emissive red LED
};

// --- Geometry helpers ---

function bodyBox(w, h, d) {
  const g = new THREE.BoxGeometry(w, h, d);
  g.computeVertexNormals();
  return g;
}

function wheelCylinder(radius, width) {
  // Wheel лежит на оси X (long axis), rotates around X (relative to body).
  const g = new THREE.CylinderGeometry(radius, radius, width, 16);
  g.rotateZ(Math.PI / 2); // lie on X
  g.computeVertexNormals();
  return g;
}

function mastCylinder(radius, height) {
  // Mast стоит вертикально (Y-axis).
  const g = new THREE.CylinderGeometry(radius, radius, height, 8);
  g.translate(0, height / 2, 0);
  g.computeVertexNormals();
  return g;
}

function tipSphere(radius) {
  const g = new THREE.SphereGeometry(radius, 12, 8);
  g.computeVertexNormals();
  return g;
}

function pushGeometryToDoc(doc, buf, geom, material) {
  const positions = new Float32Array(geom.attributes.position.array);
  const normals = new Float32Array(geom.attributes.normal.array);
  const indices = new Uint32Array(geom.index.array);

  const posAcc = doc.createAccessor()
    .setType("VEC3")
    .setArray(positions)
    .setBuffer(buf);
  const nrmAcc = doc.createAccessor()
    .setType("VEC3")
    .setArray(normals)
    .setBuffer(buf);
  const idxAcc = doc.createAccessor()
    .setType("SCALAR")
    .setArray(indices)
    .setBuffer(buf);

  const prim = doc.createPrimitive()
    .setAttribute("POSITION", posAcc)
    .setAttribute("NORMAL", nrmAcc)
    .setIndices(idxAcc)
    .setMaterial(material);

  return prim;
}

// --- Animation helpers ---

function vec3Animation(doc, buf, name, targetNode, path, keyframes, times) {
  const inputAcc = doc.createAccessor()
    .setType("SCALAR")
    .setArray(new Float32Array(times))
    .setBuffer(buf);
  const outputAcc = doc.createAccessor()
    .setType("VEC3")
    .setArray(new Float32Array(keyframes.flat()))
    .setBuffer(buf);

  const sampler = doc.createAnimationSampler()
    .setInput(inputAcc)
    .setOutput(outputAcc)
    .setInterpolation("LINEAR");

  const channel = doc.createAnimationChannel()
    .setSampler(sampler)
    .setTargetNode(targetNode)
    .setTargetPath(path);

  return doc.createAnimation(name)
    .addSampler(sampler)
    .addChannel(channel);
}

// --- Build ---

function buildAvatar() {
  const doc = new Document();
  const buf = doc.createBuffer();

  // Materials (PBR)
  const chassisMat = doc.createMaterial("chassis")
    .setBaseColorFactor([0.604, 0.639, 0.678, 1])
    .setMetallicFactor(0.6)
    .setRoughnessFactor(0.45);
  const wheelMat = doc.createMaterial("wheel")
    .setBaseColorFactor([0.114, 0.125, 0.153, 1])
    .setMetallicFactor(0.2)
    .setRoughnessFactor(0.85);
  const headMat = doc.createMaterial("head")
    .setBaseColorFactor([0.039, 0.302, 0.302, 1])
    .setMetallicFactor(0.5)
    .setRoughnessFactor(0.3)
    .setEmissiveFactor([0.05, 0.4, 0.45]);
  const headEdgeMat = doc.createMaterial("head_edge")
    .setBaseColorFactor([0.267, 0.867, 1.0, 1])
    .setMetallicFactor(0.6)
    .setRoughnessFactor(0.3)
    .setEmissiveFactor([0.2, 0.6, 0.7]);
  const mastMat = doc.createMaterial("mast")
    .setBaseColorFactor([0.165, 0.180, 0.212, 1])
    .setMetallicFactor(0.7)
    .setRoughnessFactor(0.5);
  const tipMat = doc.createMaterial("tip")
    .setBaseColorFactor([1.0, 0.322, 0.322, 1])
    .setMetallicFactor(0.0)
    .setRoughnessFactor(0.4)
    .setEmissiveFactor([1.0, 0.322, 0.322]);

  // Chassis (body) — box at origin, sits on top of wheels
  const chassisGeom = bodyBox(0.6, 0.18, 0.8);
  const chassisMesh = doc.createMesh("chassis")
    .addPrimitive(pushGeometryToDoc(doc, buf, chassisGeom, chassisMat));
  const chassisNode = doc.createNode("chassis")
    .setMesh(chassisMesh)
    .setTranslation([0, 0.16 + 0.09, 0]); // bottom-of-chassis at y=0.16

  // Wheels — 4 cylinders, all children of chassis
  const wheelGeom = wheelCylinder(0.16, 0.12);
  function addWheel(name, tx, tz) {
    const mesh = doc.createMesh(name)
      .addPrimitive(pushGeometryToDoc(doc, buf, wheelGeom, wheelMat));
    return doc.createNode(name)
      .setMesh(mesh)
      .setTranslation([tx, 0.16, tz]); // wheel centre at y=0.16 (its own axis)
  }
  const wheelFL = addWheel("wheel_fl", -0.27, 0.30);
  const wheelFR = addWheel("wheel_fr", 0.27, 0.30);
  const wheelRL = addWheel("wheel_rl", -0.27, -0.30);
  const wheelRR = addWheel("wheel_rr", 0.27, -0.30);
  chassisNode.addChild(wheelFL).addChild(wheelFR).addChild(wheelRL).addChild(wheelRR);

  // Head — small box on top of chassis, front-facing
  const headGeom = bodyBox(0.28, 0.18, 0.18);
  const headMesh = doc.createMesh("head")
    .addPrimitive(pushGeometryToDoc(doc, buf, headGeom, headMat));
  const headNode = doc.createNode("head")
    .setMesh(headMesh)
    .setTranslation([0, 0.34, 0.35]); // sits on top, slightly forward

  // Head edge accent — slightly larger thin box, emissive cyan
  const headEdgeGeom = bodyBox(0.30, 0.04, 0.04);
  const headEdgeMesh = doc.createMesh("head_edge")
    .addPrimitive(pushGeometryToDoc(doc, buf, headEdgeGeom, headEdgeMat));
  const headEdgeNode = doc.createNode("head_edge")
    .setMesh(headEdgeMesh)
    .setTranslation([0, 0.34, 0.46]);

  // Antenna mast
  const mastGeom = mastCylinder(0.012, 0.22);
  const mastMesh = doc.createMesh("mast")
    .addPrimitive(pushGeometryToDoc(doc, buf, mastGeom, mastMat));
  const mastNode = doc.createNode("mast")
    .setMesh(mastMesh)
    .setTranslation([0.12, 0.43, -0.25]);

  // Antenna tip (LED)
  const tipGeom = tipSphere(0.025);
  const tipMesh = doc.createMesh("tip")
    .addPrimitive(pushGeometryToDoc(doc, buf, tipGeom, tipMat));
  const tipNode = doc.createNode("tip")
    .setMesh(tipMesh)
    .setTranslation([0.12, 0.66, -0.25]);

  // Hierarchy: chassis → head/head_edge/mast/tip; wheels are chassis children
  chassisNode.addChild(headNode)
    .addChild(headEdgeNode)
    .addChild(mastNode)
    .addChild(tipNode);

  // Scene root
  doc.createScene("avatar").addChild(chassisNode);

  // --- Animations ---

  // idle: chassis Y bobs ±1 cm (sin) + head yaw scan ±10° (cos for offset)
  const idleTimes = Array.from({ length: IDLE_FRAMES + 1 }, (_, i) => (i * IDLE_DURATION) / IDLE_FRAMES);
  const idleChassisKeys = idleTimes.map((t) => [
    0,
    0.16 + 0.09 + Math.sin((t / IDLE_DURATION) * 2 * Math.PI) * 0.01,
    0,
  ]);
  // Note — chassisNode already has translation [0, 0.25, 0]; we OVERWRITE it with keyframes.
  // For proper "additive bobbing" we'd need a separate child, but for low-budget avatar
  // it's fine: we just oscillate the chassis node directly.
  vec3Animation(doc, buf, "idle", chassisNode, "translation", idleChassisKeys, idleTimes);

  // head yaw — sin sweep for sensor scan
  const headYawKeys = idleTimes.map((t) => [
    0,
    Math.sin((t / IDLE_DURATION) * 2 * Math.PI) * (10 * Math.PI / 180),
    0,
  ]);
  vec3Animation(doc, buf, "idle", headNode, "rotation", headYawKeys, idleTimes);

  // drive_forward: 4 wheels rotate around X axis, +π per second
  const driveTimes = Array.from({ length: FRAMES + 1 }, (_, i) => (i * DRIVE_DURATION) / FRAMES);
  const fwdKeys = driveTimes.map((t) => [t * Math.PI * 2, 0, 0]); // rotation around X (wheel long axis)
  vec3Animation(doc, buf, "drive_forward", wheelFL, "rotation", fwdKeys, driveTimes);
  vec3Animation(doc, buf, "drive_forward", wheelFR, "rotation", fwdKeys, driveTimes);
  vec3Animation(doc, buf, "drive_forward", wheelRL, "rotation", fwdKeys, driveTimes);
  vec3Animation(doc, buf, "drive_forward", wheelRR, "rotation", fwdKeys, driveTimes);

  // drive_backward: −π per second
  const backKeys = driveTimes.map((t) => [-t * Math.PI * 2, 0, 0]);
  vec3Animation(doc, buf, "drive_backward", wheelFL, "rotation", backKeys, driveTimes);
  vec3Animation(doc, buf, "drive_backward", wheelFR, "rotation", backKeys, driveTimes);
  vec3Animation(doc, buf, "drive_backward", wheelRL, "rotation", backKeys, driveTimes);
  vec3Animation(doc, buf, "drive_backward", wheelRR, "rotation", backKeys, driveTimes);

  // turn_left: front wheels ×1.5 speed, rear wheels ×0.5 speed
  const turnTimes = Array.from({ length: FRAMES + 1 }, (_, i) => (i * TURN_DURATION) / FRAMES);
  const lFastKeys = turnTimes.map((t) => [t * Math.PI * 1.5, 0, 0]);
  const lSlowKeys = turnTimes.map((t) => [t * Math.PI * 0.5, 0, 0]);
  vec3Animation(doc, buf, "turn_left", wheelFL, "rotation", lFastKeys, turnTimes);
  vec3Animation(doc, buf, "turn_left", wheelFR, "rotation", lFastKeys, turnTimes);
  vec3Animation(doc, buf, "turn_left", wheelRL, "rotation", lSlowKeys, turnTimes);
  vec3Animation(doc, buf, "turn_left", wheelRR, "rotation", lSlowKeys, turnTimes);

  // turn_right: mirror of turn_left
  const rFastKeys = turnTimes.map((t) => [-t * Math.PI * 1.5, 0, 0]);
  const rSlowKeys = turnTimes.map((t) => [-t * Math.PI * 0.5, 0, 0]);
  vec3Animation(doc, buf, "turn_right", wheelFL, "rotation", rFastKeys, turnTimes);
  vec3Animation(doc, buf, "turn_right", wheelFR, "rotation", rFastKeys, turnTimes);
  vec3Animation(doc, buf, "turn_right", wheelRL, "rotation", rSlowKeys, turnTimes);
  vec3Animation(doc, buf, "turn_right", wheelRR, "rotation", rSlowKeys, turnTimes);

  return doc;
}

// --- Main ---

async function main() {
  const args = process.argv.slice(2);
  if (args.includes("--publish")) {
    // ---- publish step: move optimized → FINAL_DIR, remove raw ----
    const { stat: statFn } = await import("node:fs/promises");
    let optimizedSize = 0;
    try {
      const st = await statFn(OPTIMIZED_FILE);
      optimizedSize = st.size;
    } catch (err) {
      console.error(`[build-avatar --publish] optimized file not found: ${OPTIMIZED_FILE}`);
      console.error(`  Run 'npm run gltf:optimize' first.`);
      process.exit(10);
    }
    await mkdir(FINAL_DIR, { recursive: true });
    await rename(OPTIMIZED_FILE, FINAL_FILE);
    console.log(`[build-avatar --publish] moved → ${FINAL_FILE} (${(optimizedSize / 1024).toFixed(1)} KB)`);
    try {
      await unlink(RAW_FILE);
      console.log(`[build-avatar --publish] removed raw ${RAW_FILE}`);
    } catch (err) {
      console.warn(`[build-avatar --publish] could not remove raw ${RAW_FILE}: ${err.message}`);
    }
    console.log(`[build-avatar --publish] DONE — commit ${path.relative(process.cwd(), FINAL_FILE)} and avatar_meta.json.`);
    return;
  }

  console.log("[build-avatar] синтез CC0 4-колёсного robot avatar (Phase 2.1)...");
  console.log(`OUT_DIR = ${OUT_DIR}`);

  await mkdir(OUT_DIR, { recursive: true });

  const doc = buildAvatar();

  const io = new NodeIO().registerExtensions(ALL_EXTENSIONS);
  const glb = await io.writeBinary(doc);
  await writeFile(RAW_FILE, glb);

  const stats = {
    size_bytes: glb.byteLength,
    raw_budget_bytes: RAW_BUDGET_BYTES,
    raw_budget_ok: glb.byteLength <= RAW_BUDGET_BYTES,
    nodes: doc.getRoot().listNodes().length,
    meshes: doc.getRoot().listMeshes().length,
    materials: doc.getRoot().listMaterials().length,
    animations: doc.getRoot().listAnimations().map((a) => {
      // Compute duration from the last keyframe time of any sampler.
      let duration = 0;
      for (const sampler of a.listSamplers()) {
        const arr = sampler.getInput().getArray();
        if (arr && arr.length > 0) {
          const last = arr[arr.length - 1];
          if (last > duration) duration = last;
        }
      }
      return {
        name: a.getName(),
        duration_s: duration,
        channels: a.listChannels().length,
      };
    }),
  };

  console.log(`[build-avatar] wrote ${RAW_FILE}`);
  console.log(`[build-avatar] size: ${(stats.size_bytes / 1024).toFixed(1)} KB / raw budget ${(RAW_BUDGET_BYTES / 1024).toFixed(0)} KB [${stats.raw_budget_ok ? "OK" : "OVER"}]`);
  console.log(`[build-avatar] ${stats.nodes} nodes, ${stats.meshes} meshes, ${stats.materials} materials, ${stats.animations.length} animations:`);
  for (const a of stats.animations) {
    console.log(`  - ${a.name}: duration=${a.duration_s.toFixed(2)}s, channels=${a.channels}`);
  }
  console.log(`[build-avatar] Next: run 'npm run gltf:optimize' (writes avatar.optimized.glb), then 'node scripts/build-avatar.mjs --publish' to move optimized → ${FINAL_FILE} and delete raw.`);

  // Sidecar JSON metadata for runtime/tests/CREDITS
  const meta = {
    name: "avatar",
    description: "CC0 4-wheeled robot avatar for Meta Quest WebXR Captain Bridge (Phase 2.1).",
    source: "programmatic synthesis (build-avatar.mjs); no external models.",
    license: "CC0-1.0",
    license_note: "All geometry is original code (Three.js primitives). CC0 public-domain dedication.",
    raw_budget_bytes: RAW_BUDGET_BYTES,
    optimized_budget_bytes: 500 * 1024,
    dimensions_m: { width: 0.6, height: 0.66, depth: 0.8 },
    animations_expected: ["idle", "drive_forward", "drive_backward", "turn_left", "turn_right"],
    stats,
  };
  await writeFile(path.join(OUT_DIR, "avatar_meta.json"), JSON.stringify(meta, null, 2) + "\n");

  if (!stats.raw_budget_ok) {
    console.error(`[FAIL] raw .glb ${(stats.size_bytes / 1024).toFixed(1)} KB превысил raw budget ${(RAW_BUDGET_BYTES / 1024).toFixed(0)} KB`);
    process.exit(1);
  }

  // Sanity: required animations exist
  const animNames = stats.animations.map((a) => a.name);
  for (const expected of meta.animations_expected) {
    if (!animNames.includes(expected)) {
      console.error(`[FAIL] expected animation "${expected}" not found (got: ${animNames.join(", ")})`);
      process.exit(2);
    }
  }

  console.log("[build-avatar] PASS");
}

main().catch((err) => {
  console.error("[build-avatar] FATAL:", err);
  process.exit(99);
});