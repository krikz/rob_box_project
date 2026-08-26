#!/usr/bin/env node
// build_bridge_assets.mjs — синтез low-poly sci-fi Bridge scene для rob_box_quest/webxr_client.
// Все модели — собственные (CC0), собраны из three.js примитивов + @gltf-transform.
// Экспорт: 3 glb файла (bridge_floor, bridge_walls, bridge_props) в assets/raw/env/.
//
// Бюджет raw (до Draco, Phase 2.0 pipeline вне scope):
//   bridge_floor.glb   ≤ 120 KB
//   bridge_walls.glb   ≤ 180 KB
//   bridge_props.glb   ≤ 250 KB
//
// Дизайн (Senior UI/UX Designer):
//   - Hex-grid floor 12×12м, dark metal с emissive cyan lines (sci-fi command bridge)
//   - Walls 4 шт: back wall + две side walls + front viewport, панели + console strips
//   - Props: captain chair + main console (изогнутый) + 4 side terminals + 2 holo-projectors

import * as THREE from "three";
import { Document, NodeIO } from "@gltf-transform/core";
import { ALL_EXTENSIONS } from "@gltf-transform/extensions";
import { promises as fs } from "node:fs";
import path from "node:path";

// Phase 2.1: build_bridge_assets.mjs outputs to public/assets/bridge/ so Vite
// can serve .glb + .hdr at runtime (/assets/bridge/...) без отдельной
// copy-стадии. Старая схема "assets/raw/env" — это pre-public staging,
// нам больше не нужна (Phase 2.0 KTX2-pipeline вне scope).
const OUT_DIR = path.resolve(
  process.cwd(),
  "public/assets/bridge",
);
const BUDGETS = { floor: 120 * 1024, walls: 180 * 1024, props: 250 * 1024 };

// --- палитра (sci-fi command bridge, dark + cyan accents) ---
const COL = {
  floorBase: 0x14181f,    // very dark gray-blue
  floorEdge: 0x2ec27e,    // emissive cyan-green
  floorPanel: 0x1a2230,   // panel inlay
  wallBase: 0x181c25,     // dark wall
  wallPanel: 0x222831,    // wall panel
  wallViewport: 0x0a0d11,  // viewport (very dark, simulating space)
  wallEdge: 0x2ec27e,     // emissive edge
  console: 0x1f242e,      // console dark
  consoleStrip: 0x2ec27e, // emissive strip
  consoleScreen: 0x0a4d4d,// screen color (emissive)
  chair: 0x0e1116,        // chair base
  chairCush: 0x1a2230,    // chair cushion
  chairEdge: 0x2ec27e,    // chair accent
  holo: 0x44ddff,         // hologram (emissive cyan)
};

// --- low-poly hex tile geometry (one tile, ~12 verts) ---
function makeHexTileGeometry(radius, height) {
  const shape = new THREE.Shape();
  for (let i = 0; i < 6; i++) {
    const a = (Math.PI / 3) * i;
    const x = radius * Math.cos(a);
    const y = radius * Math.sin(a);
    if (i === 0) shape.moveTo(x, y); else shape.lineTo(x, y);
  }
  shape.closePath();
  const geom = new THREE.ExtrudeGeometry(shape, {
    depth: height,
    bevelEnabled: false,
    curveSegments: 1,
    steps: 1,
  });
  geom.rotateX(-Math.PI / 2);
  geom.translate(0, -height / 2, 0);
  return geom;
}

// --- floor: 3 merged meshes (base, edges, panel) ---
// Объединяем все плитки одного типа в один BufferGeometry —
// сокращает overhead с 72 mesh до 3 (×overhead ~ 100 bytes per mesh в glb).
function buildFloor() {
  const group = new THREE.Group();
  group.name = "bridge_floor";
  const r = 0.6;
  const h = 0.05;
  const w = Math.sqrt(3) * r;
  const cols = 6;
  const rows = 6;

  // Соберём tile геометрии в массивы (с world transform применённым)
  const baseGeoms = [];
  const edgeGeoms = [];
  const panelGeoms = [];

  for (let row = 0; row < rows; row++) {
    for (let col = 0; col < cols; col++) {
      const offsetX = (row % 2) * (w / 2);
      const x = col * w + offsetX - (cols * w) / 2;
      const z = row * (1.5 * r) - (rows * 1.5 * r) / 2;

      const baseG = makeHexTileGeometry(r, h);
      baseG.translate(x, 0, z);
      baseGeoms.push(baseG);

      if ((row + col) % 2 === 0) {
        const edgeG = makeHexTileGeometry(r * 0.95, h * 0.5);
        edgeG.translate(x, h + 0.001, z);
        edgeGeoms.push(edgeG);
      }

      if (row === Math.floor(rows / 2) && col === Math.floor(cols / 2)) {
        const panelG = makeHexTileGeometry(r * 0.5, h * 0.6);
        panelG.translate(x, h + 0.002, z);
        panelGeoms.push(panelG);
      }
    }
  }

  function mergeGeoms(geoms) {
    if (geoms.length === 0) return null;
    const merged = new THREE.BufferGeometry();
    // Простое merge: собираем position и index массивы
    let totalVerts = 0;
    let totalIndices = 0;
    for (const g of geoms) {
      g.computeVertexNormals();
      totalVerts += g.attributes.position.count;
      totalIndices += g.index ? g.index.count : g.attributes.position.count;
    }
    const positions = new Float32Array(totalVerts * 3);
    const normals = new Float32Array(totalVerts * 3);
    const indices = new Uint32Array(totalIndices);
    let vertOff = 0;
    let idxOff = 0;
    for (const g of geoms) {
      const pos = g.attributes.position.array;
      const norm = g.attributes.normal.array;
      positions.set(pos, vertOff * 3);
      normals.set(norm, vertOff * 3);
      if (g.index) {
        for (let i = 0; i < g.index.count; i++) {
          indices[idxOff + i] = g.index.array[i] + vertOff;
        }
        idxOff += g.index.count;
      } else {
        for (let i = 0; i < g.attributes.position.count; i++) {
          indices[idxOff + i] = vertOff + i;
        }
        idxOff += g.attributes.position.count;
      }
      vertOff += g.attributes.position.count;
    }
    merged.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    merged.setAttribute("normal", new THREE.BufferAttribute(normals, 3));
    merged.setIndex(new THREE.BufferAttribute(indices, 1));
    return merged;
  }

  const baseMerged = mergeGeoms(baseGeoms);
  if (baseMerged) {
    group.add(new THREE.Mesh(baseMerged, new THREE.MeshStandardMaterial({
      color: COL.floorBase, roughness: 0.7, metalness: 0.6, flatShading: true,
    })));
  }
  const edgeMerged = mergeGeoms(edgeGeoms);
  if (edgeMerged) {
    group.add(new THREE.Mesh(edgeMerged, new THREE.MeshStandardMaterial({
      color: COL.floorEdge, emissive: COL.floorEdge, emissiveIntensity: 0.8,
      roughness: 0.4, metalness: 0.7, flatShading: true,
    })));
  }
  const panelMerged = mergeGeoms(panelGeoms);
  if (panelMerged) {
    group.add(new THREE.Mesh(panelMerged, new THREE.MeshStandardMaterial({
      color: COL.floorPanel, roughness: 0.6, metalness: 0.5, flatShading: true,
    })));
  }

  return group;
}

// --- walls: 4 стены ---
function makeWallPanel(width, height, color) {
  const geom = new THREE.BoxGeometry(width, height, 0.05);
  return new THREE.Mesh(geom, new THREE.MeshStandardMaterial({
    color, roughness: 0.85, metalness: 0.4, flatShading: true,
  }));
}
function makeConsoleStrip(width, height, color) {
  const geom = new THREE.BoxGeometry(width, height, 0.08);
  return new THREE.Mesh(geom, new THREE.MeshStandardMaterial({
    color, roughness: 0.5, metalness: 0.7, emissive: color, emissiveIntensity: 0.7,
  }));
}
function makeViewport(width, height, color) {
  const geom = new THREE.BoxGeometry(width, height, 0.04);
  return new THREE.Mesh(geom, new THREE.MeshStandardMaterial({
    color, roughness: 0.2, metalness: 0.3, emissive: color, emissiveIntensity: 0.1,
  }));
}
function buildWalls() {
  const group = new THREE.Group();
  group.name = "bridge_walls";
  const roomW = 7;
  const roomD = 8;
  const wallH = 3;

  // back wall (за спиной капитана) — с большим viewport
  const backWall = makeWallPanel(roomW, wallH, COL.wallBase);
  backWall.position.set(0, wallH / 2, -roomD / 2);
  group.add(backWall);
  const backViewport = makeViewport(roomW * 0.7, wallH * 0.5, COL.wallViewport);
  backViewport.position.set(0, wallH * 0.55, -roomD / 2 + 0.03);
  group.add(backViewport);
  const backStripL = makeConsoleStrip(0.1, wallH * 0.7, COL.consoleStrip);
  backStripL.position.set(-roomW * 0.42, wallH * 0.55, -roomD / 2 + 0.06);
  group.add(backStripL);
  const backStripR = makeConsoleStrip(0.1, wallH * 0.7, COL.consoleStrip);
  backStripR.position.set(roomW * 0.42, wallH * 0.55, -roomD / 2 + 0.06);
  group.add(backStripR);

  // front wall (перед капитаном)
  const frontWall = makeWallPanel(roomW, wallH, COL.wallBase);
  frontWall.position.set(0, wallH / 2, roomD / 2);
  group.add(frontWall);
  for (let i = 0; i < 4; i++) {
    const panel = makeWallPanel(roomW * 0.18, wallH * 0.25, COL.wallPanel);
    panel.position.set(-roomW * 0.36 + i * roomW * 0.24, wallH * 0.65, roomD / 2 + 0.04);
    group.add(panel);
  }
  for (let i = 0; i < 3; i++) {
    const strip = makeConsoleStrip(roomW * 0.25, 0.08, COL.consoleStrip);
    strip.position.set(-roomW * 0.3 + i * roomW * 0.3, wallH * 0.18, roomD / 2 + 0.06);
    group.add(strip);
  }

  // left wall
  const leftWall = makeWallPanel(roomD * 0.95, wallH, COL.wallBase);
  leftWall.position.set(-roomW / 2 - 0.02, wallH / 2, 0);
  leftWall.rotation.y = Math.PI / 2;
  group.add(leftWall);
  const leftPanel = makeWallPanel(1.5, 1.2, COL.wallPanel);
  leftPanel.position.set(-roomW / 2 + 0.04, 1.6, -1.5);
  leftPanel.rotation.y = Math.PI / 2;
  group.add(leftPanel);
  const leftViewport = makeViewport(1.8, 1.0, COL.wallViewport);
  leftViewport.position.set(-roomW / 2 + 0.04, 1.8, 1.5);
  leftViewport.rotation.y = Math.PI / 2;
  group.add(leftViewport);

  // right wall
  const rightWall = makeWallPanel(roomD * 0.95, wallH, COL.wallBase);
  rightWall.position.set(roomW / 2 + 0.02, wallH / 2, 0);
  rightWall.rotation.y = -Math.PI / 2;
  group.add(rightWall);
  const rightPanel = makeWallPanel(1.5, 1.2, COL.wallPanel);
  rightPanel.position.set(roomW / 2 - 0.04, 1.6, 1.5);
  rightPanel.rotation.y = -Math.PI / 2;
  group.add(rightPanel);
  const rightViewport = makeViewport(1.8, 1.0, COL.wallViewport);
  rightViewport.position.set(roomW / 2 - 0.04, 1.8, -1.5);
  rightViewport.rotation.y = -Math.PI / 2;
  group.add(rightViewport);

  return group;
}

// --- props ---
function makeChairBase() {
  const group = new THREE.Group();
  const base = new THREE.Mesh(
    new THREE.CylinderGeometry(0.35, 0.4, 0.15, 8),
    new THREE.MeshStandardMaterial({
      color: COL.chair, roughness: 0.6, metalness: 0.6, flatShading: true,
    })
  );
  base.position.y = 0.075;
  group.add(base);
  const pillar = new THREE.Mesh(
    new THREE.CylinderGeometry(0.08, 0.08, 0.5, 6),
    new THREE.MeshStandardMaterial({
      color: COL.chair, roughness: 0.5, metalness: 0.8, flatShading: true,
    })
  );
  pillar.position.y = 0.4;
  group.add(pillar);
  const seat = new THREE.Mesh(
    new THREE.BoxGeometry(0.5, 0.1, 0.5),
    new THREE.MeshStandardMaterial({
      color: COL.chairCush, roughness: 0.9, metalness: 0.1, flatShading: true,
    })
  );
  seat.position.y = 0.85;
  group.add(seat);
  const back = new THREE.Mesh(
    new THREE.BoxGeometry(0.5, 0.7, 0.1),
    new THREE.MeshStandardMaterial({
      color: COL.chairCush, roughness: 0.9, metalness: 0.1, flatShading: true,
    })
  );
  back.position.set(0, 1.25, -0.25);
  group.add(back);
  const accent = new THREE.Mesh(
    new THREE.BoxGeometry(0.45, 0.04, 0.11),
    new THREE.MeshStandardMaterial({
      color: COL.chairEdge, emissive: COL.chairEdge, emissiveIntensity: 0.8,
      roughness: 0.3, metalness: 0.5, flatShading: true,
    })
  );
  accent.position.set(0, 1.55, -0.25);
  group.add(accent);
  for (const side of [-1, 1]) {
    const arm = new THREE.Mesh(
      new THREE.BoxGeometry(0.08, 0.08, 0.4),
      new THREE.MeshStandardMaterial({
        color: COL.chair, roughness: 0.5, metalness: 0.7, flatShading: true,
      })
    );
    arm.position.set(side * 0.27, 1.0, -0.05);
    group.add(arm);
  }
  return group;
}

function makeMainConsole() {
  const group = new THREE.Group();
  const sectors = [
    { x: -1.3, rot: 0.35 },
    { x: 0,    rot: 0 },
    { x: 1.3,  rot: -0.35 },
  ];
  for (const s of sectors) {
    const mesh = new THREE.Mesh(
      new THREE.BoxGeometry(1.2, 0.7, 0.5),
      new THREE.MeshStandardMaterial({
        color: COL.console, roughness: 0.7, metalness: 0.5, flatShading: true,
      })
    );
    mesh.position.set(s.x, 0.5, 1.8);
    mesh.rotation.y = s.rot;
    group.add(mesh);
    const screen = new THREE.Mesh(
      new THREE.BoxGeometry(1.0, 0.4, 0.05),
      new THREE.MeshStandardMaterial({
        color: COL.consoleScreen, emissive: COL.consoleScreen, emissiveIntensity: 0.6,
        roughness: 0.3, metalness: 0.2, flatShading: true,
      })
    );
    screen.position.set(s.x, 0.7, 1.8 + Math.cos(s.rot) * 0.27);
    screen.rotation.x = -0.3;
    screen.rotation.y = s.rot;
    group.add(screen);
    const strip = new THREE.Mesh(
      new THREE.BoxGeometry(1.1, 0.04, 0.05),
      new THREE.MeshStandardMaterial({
        color: COL.consoleStrip, emissive: COL.consoleStrip, emissiveIntensity: 0.9,
        roughness: 0.3, metalness: 0.7, flatShading: true,
      })
    );
    strip.position.set(s.x, 0.88, 1.8 + Math.cos(s.rot) * 0.27);
    strip.rotation.x = -0.3;
    strip.rotation.y = s.rot;
    group.add(strip);
  }
  return group;
}

function makeSideTerminal(x, z, rotY = 0) {
  const group = new THREE.Group();
  const stand = new THREE.Mesh(
    new THREE.CylinderGeometry(0.15, 0.2, 1.0, 6),
    new THREE.MeshStandardMaterial({
      color: COL.console, roughness: 0.6, metalness: 0.6, flatShading: true,
    })
  );
  stand.position.y = 0.5;
  group.add(stand);
  const screen = new THREE.Mesh(
    new THREE.BoxGeometry(0.5, 0.35, 0.04),
    new THREE.MeshStandardMaterial({
      color: COL.consoleScreen, emissive: COL.consoleScreen, emissiveIntensity: 0.5,
      roughness: 0.3, metalness: 0.2, flatShading: true,
    })
  );
  screen.position.y = 1.25;
  screen.rotation.x = -0.25;
  group.add(screen);
  const bezel = new THREE.Mesh(
    new THREE.BoxGeometry(0.55, 0.4, 0.04),
    new THREE.MeshStandardMaterial({
      color: COL.console, roughness: 0.5, metalness: 0.7, flatShading: true,
    })
  );
  bezel.position.y = 1.25;
  bezel.position.z = -0.015;
  bezel.rotation.x = -0.25;
  group.add(bezel);
  group.position.set(x, 0, z);
  group.rotation.y = rotY;
  return group;
}

function makeHoloProjector(x, z) {
  const group = new THREE.Group();
  const base = new THREE.Mesh(
    new THREE.CylinderGeometry(0.15, 0.2, 0.2, 8),
    new THREE.MeshStandardMaterial({
      color: COL.console, roughness: 0.5, metalness: 0.8, flatShading: true,
    })
  );
  base.position.y = 0.1;
  group.add(base);
  const emitter = new THREE.Mesh(
    new THREE.CylinderGeometry(0.08, 0.08, 0.1, 6),
    new THREE.MeshStandardMaterial({
      color: COL.holo, emissive: COL.holo, emissiveIntensity: 1.5,
      roughness: 0.2, metalness: 0.3, flatShading: true,
    })
  );
  emitter.position.y = 0.25;
  group.add(emitter);
  const holoDisc = new THREE.Mesh(
    new THREE.CylinderGeometry(0.45, 0.05, 0.6, 6, 1, true),
    new THREE.MeshStandardMaterial({
      color: COL.holo, emissive: COL.holo, emissiveIntensity: 1.0,
      roughness: 0.2, metalness: 0.0,
      transparent: true, opacity: 0.4, side: THREE.DoubleSide,
      flatShading: true,
    })
  );
  holoDisc.position.y = 0.65;
  group.add(holoDisc);
  group.position.set(x, 0, z);
  return group;
}

function buildProps() {
  const group = new THREE.Group();
  group.name = "bridge_props";
  const chair = makeChairBase();
  chair.position.set(0, 0, 0.6);
  chair.rotation.y = Math.PI;
  group.add(chair);
  const console = makeMainConsole();
  group.add(console);
  group.add(makeSideTerminal(-3.2, 0.5, Math.PI / 2));
  group.add(makeSideTerminal(-3.2, -0.5, Math.PI / 2));
  group.add(makeSideTerminal(3.2, 0.5, -Math.PI / 2));
  group.add(makeSideTerminal(3.2, -0.5, -Math.PI / 2));
  group.add(makeHoloProjector(-1.0, -1.8));
  group.add(makeHoloProjector(1.0, -1.8));
  return group;
}

// --- экспорт three.js Group → glb ---
async function exportGroupToGLB(group, outputPath) {
  const doc = new Document();
  const buf = doc.createBuffer();

  const meshList = [];
  group.traverse((o) => { if (o.isMesh) meshList.push(o); });

  // Уникальные материалы
  const matMap = new Map();
  for (const m of meshList) {
    const mat = m.material;
    const emissiveHex = mat.emissive ? mat.emissive.getHexString() : "0";
    const key = `${mat.color.getHexString()}_${mat.metalness}_${mat.roughness}_${emissiveHex}_${mat.emissiveIntensity ?? 0}_${mat.transparent ? 1 : 0}_${mat.opacity ?? 1}`;
    if (!matMap.has(key)) matMap.set(key, mat);
  }

  const materials = [];
  for (const mat of matMap.values()) {
    const m = doc.createMaterial(mat.name || "mat")
      .setBaseColorFactor([mat.color.r, mat.color.g, mat.color.b, mat.transparent ? (mat.opacity ?? 0.5) : 1])
      .setMetallicFactor(mat.metalness ?? 0)
      .setRoughnessFactor(mat.roughness ?? 0.5);
    if (mat.emissive) {
      m.setEmissiveFactor([mat.emissive.r, mat.emissive.g, mat.emissive.b]);
    }
    if (mat.transparent) {
      m.setAlphaMode("BLEND");
    }
    materials.push(m);
  }

  for (const mesh of meshList) {
    const geom = mesh.geometry;
    const posAttr = geom.getAttribute("position");
    const normAttr = geom.getAttribute("normal");
    if (!posAttr) continue;

    // Bake world transform в геометрию
    geom.applyMatrix4(mesh.matrixWorld);

    const posAcc = doc.createAccessor()
      .setType("VEC3")
      .setArray(new Float32Array(posAttr.array))
      .setBuffer(buf);
    const meshPrim = doc.createPrimitive()
      .setAttribute("POSITION", posAcc);
    if (normAttr) {
      const normAcc = doc.createAccessor()
        .setType("VEC3")
        .setArray(new Float32Array(normAttr.array))
        .setBuffer(buf);
      meshPrim.setAttribute("NORMAL", normAcc);
    }

    const mat = mesh.material;
    const emissiveHex = mat.emissive ? mat.emissive.getHexString() : "0";
    const key = `${mat.color.getHexString()}_${mat.metalness}_${mat.roughness}_${emissiveHex}_${mat.emissiveIntensity ?? 0}_${mat.transparent ? 1 : 0}_${mat.opacity ?? 1}`;
    const matIdx = materials.indexOf(matMap.get(key));
    meshPrim.setMaterial(materials[matIdx]);

    if (geom.index) {
      const idxAcc = doc.createAccessor()
        .setType("SCALAR")
        .setArray(new Uint32Array(geom.index.array))
        .setBuffer(buf);
      meshPrim.setIndices(idxAcc);
    }

    const m = doc.createMesh(mesh.name || "mesh").addPrimitive(meshPrim);
    const n = doc.createNode(mesh.name || "node").setMesh(m);
    doc.createScene(group.name).addChild(n);
  }

  const io = new NodeIO().registerExtensions(ALL_EXTENSIONS);
  const glb = await io.writeBinary(doc);
  await fs.writeFile(outputPath, glb);
  return { size: glb.byteLength, meshes: meshList.length, materials: materials.length };
}

// --- main ---
async function main() {
  console.log("[bridge_assets] синтез low-poly sci-fi Bridge scene...");
  console.log(`OUT_DIR = ${OUT_DIR}`);
  await fs.mkdir(OUT_DIR, { recursive: true });

  const tasks = [
    { name: "bridge_floor.glb", group: buildFloor(), budget: BUDGETS.floor },
    { name: "bridge_walls.glb", group: buildWalls(), budget: BUDGETS.walls },
    { name: "bridge_props.glb", group: buildProps(), budget: BUDGETS.props },
  ];

  const results = [];
  for (const t of tasks) {
    const outPath = path.join(OUT_DIR, t.name);
    const r = await exportGroupToGLB(t.group, outPath);
    const kb = (r.size / 1024).toFixed(1);
    const budgetKb = (t.budget / 1024).toFixed(0);
    const ok = r.size <= t.budget ? "OK" : "OVER";
    console.log(`  ${t.name}: ${kb} KB / budget ${budgetKb} KB [${ok}] — ${r.meshes} meshes, ${r.materials} mats`);
    results.push({ name: t.name, size: r.size, budget: t.budget, ok: r.size <= t.budget });
  }

  const failed = results.filter((r) => !r.ok);
  if (failed.length > 0) {
    console.error(`[FAIL] ${failed.length} файлов превысили budget:`);
    failed.forEach((f) => console.error(`  ${f.name}: ${(f.size / 1024).toFixed(1)} KB / ${(f.budget / 1024).toFixed(0)} KB`));
    process.exit(1);
  }

  console.log("[bridge_assets] all 3 glb экспортированы, в budget.");
}

main().catch((e) => { console.error(e); process.exit(1); });
