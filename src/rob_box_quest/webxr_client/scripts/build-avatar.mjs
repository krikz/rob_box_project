#!/usr/bin/env node
// build-avatar.mjs
//
// Programmatic CC0 robot-avatar builder for rob_box_quest Phase 2.1
// (ADR-0032 §3.1, issue #1677). Produces a low-poly 4-wheeled platform
// avatar with PBR materials (albedo + normal + metallic-roughness maps),
// authored entirely in code (no external Blender export, no licence
// ambiguity), and exported as `public/models/avatar/avatar.glb`.
//
// Why programmatic and not Quaternius / Khronos sample:
//   - Quaternius Ultimate Animated Character Pack is humanoid, not a
//     4-wheeled robot platform; the avatar needs to look like our
//     actual robot.
//   - Khronos Sample Assets (RiggedFigure etc.) are humanoid with 1-2
//     cycles; no driving / turning animations.
//   - Hand-authored CC0 source stays under our CC0 policy (README §"Asset
//     pipeline (CC0-only)") with no licence chain.
//
// Animations emitted (5 clips, non-zero durations):
//   1. idle        — gentle body bob + sensor head yaw
//   2. drive-fwd   — wheel rotation about X axis (forward)
//   3. drive-back  — wheel rotation about X axis (reverse)
//   4. turn-left   — asymmetric wheel rotation (left slow, right fast)
//   5. turn-right  — asymmetric wheel rotation (left fast, right slow)
//
// Materials use procedural canvas textures (sharp-friendly JPEG) so that
// the KTX2 step has real sRGB image data to encode. All textures are
// authored at 256×256 / 512×512 — small enough to keep the avatar under
// 500 KB after KTX2 + Draco + Meshopt.
//
// Usage:
//   node scripts/build-avatar.mjs                   # writes public/models/avatar/avatar.glb
//   node scripts/build-avatar.mjs --out some/path.glb

import { Document, NodeIO } from "@gltf-transform/core";
import { ALL_EXTENSIONS } from "@gltf-transform/extensions";
import { dedup, draco, meshopt, resample } from "@gltf-transform/functions";
import draco3d from "draco3dgltf";
import { MeshoptDecoder, MeshoptEncoder } from "meshoptimizer";
import sharp from "sharp";
import { writeFile, mkdir } from "node:fs/promises";
import { dirname, join, resolve as pathResolve } from "node:path";
import { fileURLToPath } from "node:url";
import { Buffer } from "node:buffer";

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);
const DEFAULT_OUT = join(__dirname, "..", "public", "models", "avatar", "avatar.glb");

// ----- CLI args -----
const argv = process.argv.slice(2);
const outIdx = argv.indexOf("--out");
const outPath = outIdx >= 0 ? pathResolve(argv[outIdx + 1]) : DEFAULT_OUT;

// ----- Procedural canvas texture builders -----
// Build an RGBA image (Uint8Array) using a tiny per-channel callback so we
// don't pull in `canvas` (native dep) or pull PNGs out of an asset dir.
// Returns { mimeType, buffer } suitable for Document.createTexture().setImage().

function buildAlbedo({ width = 256, height = 256 } = {}) {
  // Charcoal-anthracite body with a subtle radial gradient + emissive accent
  // stripe (the LED "spine") running fore-aft. Output: JPEG (sharp-friendly).
  const channels = 4;
  const data = Buffer.alloc(width * height * channels);
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const i = (y * width + x) * channels;
      // radial gradient: centre slightly lighter
      const dx = (x - width / 2) / (width / 2);
      const dy = (y - height / 2) / (height / 2);
      const r = Math.sqrt(dx * dx + dy * dy);
      const lift = Math.max(0, 1 - r) * 0.18;
      // horizontal accent stripe at y ∈ [40..48]/256 (top 1/6)
      const inStripe = y > 40 && y < 52;
      data[i + 0] = Math.round((36 + lift * 255) | 0);                 // R
      data[i + 1] = Math.round((40 + lift * 255) | 0);                 // G
      data[i + 2] = inStripe ? Math.round(70 + lift * 255) : Math.round((46 + lift * 255) | 0); // B
      data[i + 3] = 255;                                               // A
    }
  }
  return data;
}

function buildNormal({ width = 256, height = 256 } = {}) {
  // Tangent-space normal map: dominant "up" (Z=1 → 128,128,255 in tangent
  // space) with a gentle X/Y tilt across the surface to give the PBR
  // shader some micro-roughness response.
  const channels = 4;
  const data = Buffer.alloc(width * height * channels);
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const i = (y * width + x) * channels;
      // subtle radial X/Y wobble, ±~5°
      const dx = (x - width / 2) / (width / 2);
      const dy = (y - height / 2) / (height / 2);
      const r = Math.min(1, Math.sqrt(dx * dx + dy * dy));
      const nx = Math.sin(r * Math.PI) * 0.10;
      const ny = Math.cos(r * Math.PI) * 0.10;
      const nz = Math.sqrt(Math.max(0, 1 - nx * nx - ny * ny));
      data[i + 0] = Math.round((nx * 0.5 + 0.5) * 255);
      data[i + 1] = Math.round((ny * 0.5 + 0.5) * 255);
      data[i + 2] = Math.round((nz * 0.5 + 0.5) * 255);
      data[i + 3] = 255;
    }
  }
  return data;
}

function buildMetallicRoughness({ width = 256, height = 256 } = {}) {
  // PBR metallic-roughness packed: B = metallic, G = roughness.
  // Robot chassis: low metallic (machined plastic), medium roughness.
  const channels = 4;
  const data = Buffer.alloc(width * height * channels);
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const i = (y * width + x) * channels;
      data[i + 0] = 0;                          // R unused (occlusion slot)
      data[i + 1] = Math.round(0.62 * 255);     // G = roughness ~0.62
      data[i + 2] = Math.round(0.18 * 255);     // B = metallic ~0.18
      data[i + 3] = 255;
    }
  }
  return data;
}

async function encodeJPEG(buffer, width, height, quality = 80) {
  return await sharp(buffer, { raw: { width, height, channels: 4 } })
    .jpeg({ quality })
    .toBuffer();
}

// ----- Avatar mesh authoring -----
//
// Build a low-poly 4-wheeled robot directly as glTF buffers using
// gltf-transform primitives. Each part (chassis, wheels, sensor head,
// antenna) is its own Node+Mesh so AnimationMixer can target individual
// channels by node name.
//
// Geometry is hand-rolled: BoxGeometry-like for chassis (24 verts),
// CylinderGeometry-like for wheels (32 verts each), low-poly sphere
// for sensor head (icosahedron, ~42 verts), small cylinder for antenna.
// Total ~120 verts — well under any mobile GPU's vertex budget.

function buildBoxMesh({ sx = 1, sy = 1, sz = 1 } = {}) {
  const hx = sx / 2, hy = sy / 2, hz = sz / 2;
  // 24 verts (4 per face) so face normals are flat per face.
  const positions = new Float32Array([
    // +X face
    hx, -hy, -hz,  hx,  hy, -hz,  hx,  hy,  hz,  hx, -hy,  hz,
    // -X face
    -hx, -hy,  hz, -hx,  hy,  hz, -hx,  hy, -hz, -hx, -hy, -hz,
    // +Y face
    -hx, hy, -hz, -hx, hy,  hz,  hx, hy,  hz,  hx, hy, -hz,
    // -Y face
    -hx, -hy, hz, -hx, -hy, -hz, hx, -hy, -hz, hx, -hy, hz,
    // +Z face
    -hx, -hy, hz,  hx, -hy, hz,  hx,  hy, hz, -hx,  hy, hz,
    // -Z face
    hx, -hy, -hz, -hx, -hy, -hz, -hx, hy, -hz, hx,  hy, -hz,
  ]);
  const normals = new Float32Array([
    // +X
    1, 0, 0,  1, 0, 0,  1, 0, 0,  1, 0, 0,
    // -X
    -1, 0, 0, -1, 0, 0, -1, 0, 0, -1, 0, 0,
    // +Y
    0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0,
    // -Y
    0, -1, 0, 0, -1, 0, 0, -1, 0, 0, -1, 0,
    // +Z
    0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1,
    // -Z
    0, 0, -1, 0, 0, -1, 0, 0, -1, 0, 0, -1,
  ]);
  // UVs: each face maps 0..1 with the accent stripe running fore-aft (Z).
  // Albedo stripe is at y ∈ [40..48] in texel space (top 1/6 of body
  // texture) — we map +Z face to that band.
  const uvs = new Float32Array(24 * 2);
  // face uv helper
  const setUV = (faceStart, u0, v0, u1, v1, u2, v2, u3, v3) => {
    uvs[faceStart * 2 + 0] = u0; uvs[faceStart * 2 + 1] = v0;
    uvs[faceStart * 2 + 2] = u1; uvs[faceStart * 2 + 3] = v1;
    uvs[faceStart * 2 + 4] = u2; uvs[faceStart * 2 + 5] = v2;
    uvs[faceStart * 2 + 6] = u3; uvs[faceStart * 2 + 7] = v3;
  };
  setUV(0,  0, 0, 1, 0, 1, 1, 0, 1);
  setUV(4,  1, 0, 0, 0, 0, 1, 1, 1);
  setUV(8,  0, 1, 0, 0, 1, 0, 1, 1);
  setUV(12, 0, 0, 0, 1, 1, 1, 1, 0);
  setUV(16, 0, 0, 1, 0, 1, 1, 0, 1);
  setUV(20, 1, 0, 0, 0, 0, 1, 1, 1);
  const indices = new Uint16Array([
    0, 1, 2,  0, 2, 3,       // +X
    4, 5, 6,  4, 6, 7,       // -X
    8, 9, 10, 8, 10, 11,     // +Y
    12, 13, 14, 12, 14, 15,  // -Y
    16, 17, 18, 16, 18, 19,  // +Z
    20, 21, 22, 20, 22, 23,  // -Z
  ]);
  return { positions, normals, uvs, indices };
}

function buildCylinderMesh({ radius = 1, height = 1, segments = 16 } = {}) {
  // Open-ended cylinder for the wheel. 2 * segments side verts + 2 centre
  // verts (top & bottom) → 2*segments + 2 verts total. We keep side
  // verts twice (separate top/bottom positions) so the normal points
  // outward without needing a shared vertex.
  const top = new Float32Array((segments + 1) * 3);
  const bot = new Float32Array((segments + 1) * 3);
  const normals = new Float32Array((segments + 1) * 6); // top+bot
  const uvs = new Float32Array((segments + 1) * 4);
  for (let i = 0; i <= segments; i++) {
    const a = (i / segments) * Math.PI * 2;
    const cx = Math.cos(a) * radius;
    const cz = Math.sin(a) * radius;
    // Top ring
    top[(i * 3) + 0] = cx; top[(i * 3) + 1] = height / 2; top[(i * 3) + 2] = cz;
    // Bottom ring
    bot[(i * 3) + 0] = cx; bot[(i * 3) + 1] = -height / 2; bot[(i * 3) + 2] = cz;
    // Normals (outward radial)
    normals[(i * 6) + 0] = Math.cos(a);
    normals[(i * 6) + 1] = 0;
    normals[(i * 6) + 2] = Math.sin(a);
    normals[(i * 6) + 3] = Math.cos(a);
    normals[(i * 6) + 4] = 0;
    normals[(i * 6) + 5] = Math.sin(a);
    // UVs: u runs around, v runs vertically (top=1, bot=0)
    const u = i / segments;
    uvs[(i * 4) + 0] = u; uvs[(i * 4) + 1] = 1;
    uvs[(i * 4) + 2] = u; uvs[(i * 4) + 3] = 0;
  }
  const positions = new Float32Array(top.length + bot.length);
  positions.set(top, 0);
  positions.set(bot, top.length);
  // Indices: pair of triangles per segment band
  const indices = new Uint16Array(segments * 6);
  const topOffset = 0;
  const botOffset = segments + 1;
  for (let i = 0; i < segments; i++) {
    const t0 = topOffset + i;
    const t1 = topOffset + i + 1;
    const b0 = botOffset + i;
    const b1 = botOffset + i + 1;
    const o = i * 6;
    indices[o + 0] = t0; indices[o + 1] = b0; indices[o + 2] = b1;
    indices[o + 3] = t0; indices[o + 4] = b1; indices[o + 5] = t1;
  }
  return { positions, normals, uvs, indices };
}

function buildIcosphereMesh({ radius = 1 } = {}) {
  // Minimal icosahedron (12 verts, 20 faces). Sufficient for a sensor
  // "eye" — the visual is dominated by the emissive material, not the
  // silhouette.
  const t = (1 + Math.sqrt(5)) / 2;
  const verts = [
    [-1,  t,  0], [ 1,  t,  0], [-1, -t,  0], [ 1, -t,  0],
    [ 0, -1,  t], [ 0,  1,  t], [ 0, -1, -t], [ 0,  1, -t],
    [ t,  0, -1], [ t,  0,  1], [-t,  0, -1], [-t,  0,  1],
  ];
  const positions = new Float32Array(verts.length * 3);
  const normals = new Float32Array(verts.length * 3);
  const uvs = new Float32Array(verts.length * 2);
  for (let i = 0; i < verts.length; i++) {
    const len = Math.hypot(verts[i][0], verts[i][1], verts[i][2]);
    positions[(i * 3) + 0] = (verts[i][0] / len) * radius;
    positions[(i * 3) + 1] = (verts[i][1] / len) * radius;
    positions[(i * 3) + 2] = (verts[i][2] / len) * radius;
    normals[(i * 3) + 0] = verts[i][0] / len;
    normals[(i * 3) + 1] = verts[i][1] / len;
    normals[(i * 3) + 2] = verts[i][2] / len;
    uvs[(i * 2) + 0] = 0.5 + Math.atan2(verts[i][2], verts[i][0]) / (2 * Math.PI);
    uvs[(i * 2) + 1] = 0.5 - Math.asin(verts[i][1] / len) / Math.PI;
  }
  const faces = [
    [0, 11, 5], [0, 5, 1], [0, 1, 7], [0, 7, 10], [0, 10, 11],
    [1, 5, 9], [5, 11, 4], [11, 10, 2], [10, 7, 6], [7, 1, 8],
    [3, 9, 4], [3, 4, 2], [3, 2, 6], [3, 6, 8], [3, 8, 9],
    [4, 9, 5], [2, 4, 11], [6, 2, 10], [8, 6, 7], [9, 8, 1],
  ];
  const indices = new Uint16Array(faces.length * 3);
  for (let i = 0; i < faces.length; i++) {
    indices[(i * 3) + 0] = faces[i][0];
    indices[(i * 3) + 1] = faces[i][1];
    indices[(i * 3) + 2] = faces[i][2];
  }
  return { positions, normals, uvs, indices };
}

// ----- Animation helpers -----
//
// Build a glTF sampler (input/output/accessors) + channel from a small
// JS description. We only animate Translation / Rotation channels; scale
// stays identity.

function timeArray(keyframes) {
  // keyframes: [{ t, ... }]
  const out = new Float32Array(keyframes.length);
  for (let i = 0; i < keyframes.length; i++) out[i] = keyframes[i].t;
  return out;
}

function vec3Array(keyframes, pick) {
  const out = new Float32Array(keyframes.length * 3);
  for (let i = 0; i < keyframes.length; i++) {
    const v = pick(keyframes[i]);
    out[(i * 3) + 0] = v[0]; out[(i * 3) + 1] = v[1]; out[(i * 3) + 2] = v[2];
  }
  return out;
}

function quatArray(keyframes, pick) {
  const out = new Float32Array(keyframes.length * 4);
  for (let i = 0; i < keyframes.length; i++) {
    const q = pick(keyframes[i]);
    out[(i * 4) + 0] = q[0]; out[(i * 4) + 1] = q[1];
    out[(i * 4) + 2] = q[2]; out[(i * 4) + 3] = q[3];
  }
  return out;
}

// Quaternion from axis-angle
function quatFromAxisAngle(axis, angle) {
  const half = angle / 2;
  const s = Math.sin(half);
  return [axis[0] * s, axis[1] * s, axis[2] * s, Math.cos(half)];
}

function quatIdentity() {
  return [0, 0, 0, 1];
}

// ----- Build the avatar document -----
async function buildAvatarDocument() {
  const doc = new Document();
  doc.createBuffer();

  // Register only the extensions we KNOW we'll use at write time.
  // ALL_EXTENSIONS would try to emit Draco (needs encoder) and Meshopt
  // (needs encoder) — those are applied later in `gltf:optimize`.
  // We use the default pbrMetallicRoughness material, so no extensions
  // need to be pre-declared here.
  void doc;

  // ---- Textures (PBR maps) ----
  const albedoImg = await encodeJPEG(buildAlbedo({ width: 256, height: 256 }), 256, 256, 80);
  const normalImg = await encodeJPEG(buildNormal({ width: 256, height: 256 }), 256, 256, 80);
  const mrImg = await encodeJPEG(buildMetallicRoughness({ width: 256, height: 256 }), 256, 256, 80);

  const albedoTex = doc.createTexture("avatar_albedo").setImage(albedoImg).setMimeType("image/jpeg");
  const normalTex = doc.createTexture("avatar_normal").setImage(normalImg).setMimeType("image/jpeg");
  const mrTex = doc.createTexture("avatar_metallicRoughness").setImage(mrImg).setMimeType("image/jpeg");

  // ---- Material (single PBR) ----
  const material = doc.createMaterial("avatar_body")
    .setBaseColorFactor([0.85, 0.85, 0.85, 1.0])
    .setMetallicFactor(0.18)
    .setRoughnessFactor(0.62)
    .setBaseColorTexture(albedoTex)
    .setNormalTexture(normalTex)
    .setMetallicRoughnessTexture(mrTex);

  // ---- Geometries → accessors → primitives → meshes ----
  const chassis = buildBoxMesh({ sx: 1.2, sy: 0.32, sz: 0.72 });
  const wheel = buildCylinderMesh({ radius: 0.18, height: 0.10, segments: 16 });
  const sensor = buildIcosphereMesh({ radius: 0.13 });
  const antenna = buildCylinderMesh({ radius: 0.015, height: 0.20, segments: 6 });

  // Generic helper to push a CPU mesh into a glTF primitive.
  function attachPrimitive(mesh, name, indicesType = "Uint16") {
    const buffer = doc.getRoot().listBuffers()[0];

    const posAcc = doc.createAccessor(name + "_positions")
      .setType("VEC3")
      .setArray(mesh.positions)
      .setBuffer(buffer);
    const normAcc = doc.createAccessor(name + "_normals")
      .setType("VEC3")
      .setArray(mesh.normals)
      .setBuffer(buffer);
    const uvAcc = doc.createAccessor(name + "_uvs")
      .setType("VEC2")
      .setArray(mesh.uvs)
      .setBuffer(buffer);
    const idxAcc = doc.createAccessor(name + "_indices")
      .setType("SCALAR")
      .setArray(mesh.indices)
      .setBuffer(buffer);

    const prim = doc.createPrimitive()
      .setAttribute("POSITION", posAcc)
      .setAttribute("NORMAL", normAcc)
      .setAttribute("TEXCOORD_0", uvAcc)
      .setIndices(idxAcc);

    const gltfMesh = doc.createMesh(name).addPrimitive(prim.setMaterial(material));
    return gltfMesh;
  }

  const chassisMesh = attachPrimitive(chassis, "chassis");
  const wheelMesh = attachPrimitive(wheel, "wheel");
  const sensorMesh = attachPrimitive(sensor, "sensor");
  const antennaMesh = attachPrimitive(antenna, "antenna");

  // ---- Node hierarchy ----
  // Root
  // ├─ chassis
  // ├─ sensor (offset forward on +Z, animated yaw)
  // ├─ antenna (top of chassis)
  // └─ wheel_FL, wheel_FR, wheel_RL, wheel_RR  (animated X-rotation)
  const rootNode = doc.createNode("avatar_root");

  const chassisNode = doc.createNode("chassis")
    .setTranslation([0, 0.18, 0])
    .setMesh(chassisMesh);
  rootNode.addChild(chassisNode);

  const sensorNode = doc.createNode("sensor")
    .setTranslation([0, 0.40, 0.30])
    .setMesh(sensorMesh);
  rootNode.addChild(sensorNode);

  const antennaNode = doc.createNode("antenna")
    .setTranslation([0, 0.45, 0])
    .setMesh(antennaMesh);
  rootNode.addChild(antennaNode);

  // Wheel local offsets: y=0.10 (axle height), z=±0.31 (chassis half), x=±0.50
  // The wheels are oriented so their axis (cylinder Y) aligns with the
  // chassis X axis. We achieve this with a 90° Z-axis rotation; then the
  // "spin" channel rotates around the cylinder's local Y, which after the
  // parent transform becomes the world X axis (forward rolling).
  function makeWheelNode(name, x, z) {
    const node = doc.createNode(name)
      .setTranslation([x, 0.10, z])
      .setRotation(quatFromAxisAngle([0, 0, 1], Math.PI / 2))
      .setMesh(wheelMesh);
    rootNode.addChild(node);
    return node;
  }
  const wheelFL = makeWheelNode("wheel_FL", +0.50, +0.31);
  const wheelFR = makeWheelNode("wheel_FR", -0.50, +0.31);
  const wheelRL = makeWheelNode("wheel_RL", +0.50, -0.31);
  const wheelRR = makeWheelNode("wheel_RR", -0.50, -0.31);

  // ---- Scene ----
  doc.createScene("avatar_scene").addChild(rootNode);

  // ---- Animations ----
  // Each animation = 1 sampler (input/output accessors) + N channels.
  // Channel target = node + path (translation / rotation).

  function makeAnimationSampler(name, inputArr, outputArr, interpolation = "LINEAR") {
    const buffer = doc.getRoot().listBuffers()[0];
    const inputAcc = doc.createAccessor(name + "_input")
      .setType("SCALAR")
      .setArray(inputArr)
      .setBuffer(buffer);
    const outputAcc = doc.createAccessor(name + "_output")
      .setType(outputArr.length / inputArr.length === 4 ? "VEC4" : "VEC3")
      .setArray(outputArr)
      .setBuffer(buffer);
    return doc.createAnimationSampler()
      .setInput(inputAcc)
      .setOutput(outputAcc)
      .setInterpolation(interpolation);
  }

  function addTranslationChannel(anim, name, node, keyframes) {
    const input = timeArray(keyframes);
    const output = vec3Array(keyframes, (k) => k.translation);
    const sampler = makeAnimationSampler(name, input, output);
    anim.addSampler(sampler);
    anim.addChannel(doc.createAnimationChannel()
      .setSampler(sampler)
      .setTargetNode(node)
      .setTargetPath("translation"));
  }

  function addRotationChannel(anim, name, node, keyframes) {
    const input = timeArray(keyframes);
    const output = quatArray(keyframes, (k) => k.rotation);
    const sampler = makeAnimationSampler(name, input, output);
    anim.addSampler(sampler);
    anim.addChannel(doc.createAnimationChannel()
      .setSampler(sampler)
      .setTargetNode(node)
      .setTargetPath("rotation"));
  }

  // Idle: chassis bobs ±0.02 in Y, sensor yaws ±25° in Y. 2 s loop.
  const idleKeyframesChassis = [
    { t: 0.0,  translation: [0, 0.18, 0],    rotation: quatIdentity() },
    { t: 0.5,  translation: [0, 0.20, 0],    rotation: quatIdentity() },
    { t: 1.0,  translation: [0, 0.18, 0],    rotation: quatIdentity() },
    { t: 1.5,  translation: [0, 0.16, 0],    rotation: quatIdentity() },
    { t: 2.0,  translation: [0, 0.18, 0],    rotation: quatIdentity() },
  ];
  const idleKeyframesSensor = [
    { t: 0.0,  translation: [0, 0.40, 0.30], rotation: quatFromAxisAngle([0, 1, 0], -25 * Math.PI / 180) },
    { t: 1.0,  translation: [0, 0.40, 0.30], rotation: quatFromAxisAngle([0, 1, 0],  25 * Math.PI / 180) },
    { t: 2.0,  translation: [0, 0.40, 0.30], rotation: quatFromAxisAngle([0, 1, 0], -25 * Math.PI / 180) },
  ];

  const idleAnim = doc.createAnimation("idle");
  addTranslationChannel(idleAnim, "idle_chassis_trans", chassisNode, idleKeyframesChassis);
  addRotationChannel(idleAnim, "idle_sensor_rot", sensorNode, idleKeyframesSensor);

  // Helper: build a wheel-spin animation that drives 4 wheels.
  // `multiplier` lets drive-fwd use +1.0, drive-back -1.0, turn-left
  // (+0.4 on L, +1.0 on R), turn-right (-1.0 on L, -0.4 on R) so each
  // wheel rotates at its own rate per channel.
  function makeWheelSpinAnimation(name, multipliers, durationSec) {
    // Each wheel channel has its own kf array (constant rate over time).
    const anim = doc.createAnimation(name);
    const kfCount = 5;
    const ratePerWheel = (node) => {
      // Wheel spins at 2π/sec × multiplier → so over `durationSec` we
      // accumulate `2π * durationSec * multiplier` radians of rotation.
      const totalAngle = 2 * Math.PI * durationSec * multipliers[node.getName()];
      const kfs = [];
      for (let i = 0; i < kfCount; i++) {
        const t = (i / (kfCount - 1)) * durationSec;
        const angle = (totalAngle * i) / (kfCount - 1);
        kfs.push({
          t,
          translation: node.getTranslation(),
          rotation: quatFromAxisAngle([0, 1, 0], angle),
        });
      }
      return kfs;
    };
    for (const w of [wheelFL, wheelFR, wheelRL, wheelRR]) {
      addRotationChannel(anim, `${name}_${w.getName()}_rot`, w, ratePerWheel(w));
    }
    return anim;
  }

  // 1 s duration each — fast enough for the e2e animation mixer to land
  // on a stable frame mid-cycle, slow enough that no frame lands on
  // t=0 boundary.
  makeWheelSpinAnimation("drive-forward", { wheel_FL: +1.0, wheel_FR: +1.0, wheel_RL: +1.0, wheel_RR: +1.0 }, 1.0);
  makeWheelSpinAnimation("drive-backward", { wheel_FL: -1.0, wheel_FR: -1.0, wheel_RL: -1.0, wheel_RR: -1.0 }, 1.0);
  // turn-left: left wheels slow, right wheels fast → pivot left.
  makeWheelSpinAnimation("turn-left", { wheel_FL: +0.4, wheel_FR: +1.0, wheel_RL: +0.4, wheel_RR: +1.0 }, 1.0);
  // turn-right: opposite.
  makeWheelSpinAnimation("turn-right", { wheel_FL: -1.0, wheel_FR: -0.4, wheel_RL: -1.0, wheel_RR: -0.4 }, 1.0);

  return doc;
}

// ----- Write GLB -----
async function main() {
  const doc = await buildAvatarDocument();

  // Apply the geometry compression pass (Draco + Meshopt) directly here.
  // We deliberately skip `prune()` — it incorrectly disposes the packed
  // metallic-roughness texture even when the material has it bound. The
  // textures pass (KTX2) happens in `gltf-optimize-ktx2.mjs` as a
  // separate concern.
  if (MeshoptDecoder?.ready) await MeshoptDecoder.ready;
  if (MeshoptEncoder?.ready) await MeshoptEncoder.ready;
  const io = new NodeIO()
    .registerExtensions(ALL_EXTENSIONS)
    .registerDependencies({
      "draco3d.encoder": await draco3d.createEncoderModule(),
      "draco3d.decoder": await draco3d.createDecoderModule(),
      "meshopt.encoder": MeshoptEncoder,
      "meshopt.decoder": MeshoptDecoder,
    });

  await doc.transform(
    dedup(),
    resample(),
    draco({ method: "edgebreaker", encodeSpeed: 5, decodeSpeed: 5 }),
    // NOTE: pass `false` for the second arg to skip meshopt's instancing
    // step — it would otherwise split each shared mesh (4 wheels, 1
    // antenna) into separate anonymous Node entries and break the named
    // animation targets ("wheel_FL", "wheel_FR", …) we rely on for
    // AnimationMixer.
    meshopt({ encoder: MeshoptEncoder }, false),
  );

  const glb = await io.writeBinary(doc);

  await mkdir(dirname(outPath), { recursive: true });
  await writeFile(outPath, glb);

  // Also write a `<name>.optimized.glb` copy so the conventional
  // `npm run gltf:verify` pipeline + the KTX2 pass
  // (scripts/gltf-optimize-ktx2.mjs) see it. The build-avatar step
  // already runs Draco+Meshopt, so this is purely a naming convention
  // for downstream tooling.
  const optimizedPath = outPath.replace(/\.glb$/i, ".optimized.glb");
  if (optimizedPath !== outPath) {
    await writeFile(optimizedPath, glb);
  }

  const sizeKB = (glb.byteLength / 1024).toFixed(1);
  console.log(`[build-avatar] wrote ${outPath} (${sizeKB} KB) [Draco + Meshopt applied; textures stay JPEG for the KTX2 pass]`);
  console.log(`[build-avatar] also wrote ${optimizedPath} for downstream tooling (gltf:verify + KTX2 pass)`);

  // Quick summary of what we shipped, for the dev reading the terminal.
  const root = doc.getRoot();
  const meshes = root.listMeshes().length;
  const nodes = root.listNodes().length;
  const materials = root.listMaterials().length;
  const animations = root.listAnimations();
  const textures = root.listTextures().length;
  console.log(`[build-avatar] meshes=${meshes} nodes=${nodes} materials=${materials} textures=${textures} animations=${animations.length}`);
  for (const a of animations) {
    const dur = a.listSamplers().reduce((mx, s) => {
      const input = s.getInput().getArray();
      return Math.max(mx, input[input.length - 1]);
    }, 0);
    console.log(`  - "${a.getName()}" duration=${dur.toFixed(2)}s channels=${a.listChannels().length}`);
  }
}

main().catch((err) => {
  console.error("[build-avatar] FATAL:", err);
  process.exit(1);
});
