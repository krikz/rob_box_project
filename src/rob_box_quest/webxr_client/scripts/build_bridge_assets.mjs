#!/usr/bin/env node
// build_bridge_assets.mjs — синтез low-poly sci-fi Bridge scene для
// rob_box_quest/webxr_client (Phase 2.1, kanban t_0bd54b80, issue #1677).
//
// Все модели — собственные (CC0), собраны из three.js примитивов +
// @gltf-transform/core. Raw glb экспортируются в `public/models/environment/_raw/`
// (gitignored); затем Phase 2.0 pipeline (`npm run gltf:optimize`) пережмёт их
// в `.optimized.glb` с Draco + Meshopt (см. ADR-0032 §3.1).
//
// Outputs:
//   public/models/environment/_raw/bridge_floor.glb    (≤ 120 KB raw)
//   public/models/environment/_raw/bridge_walls.glb    (≤ 180 KB raw)
//   public/models/environment/_raw/bridge_props.glb    (≤ 250 KB raw)
//   public/models/environment/_raw/bridge_nav.glb      (≤ 30 KB raw, AABB маркеры)
//   public/models/environment/_raw/bridge_occluders.glb(≤ 20 KB raw, прозрачные плоскости)
//
// После `gltf:optimize` все файлы должны пройти `gltf:verify` (≤ 2 MB total).
//
// Дополнительно генерируется bridge_scene_meta.json в
// public/models/environment/_raw/ (для runtime / tests / docs):
//   {
//     "design": { ... },             // описание стиля, освещения, palette
//     "safe_walk_area": { aabb: ... }, // AABB зоны, где безопасно ходить
//     "nav_points": [ { id, position, kind, label } ],
//     "occluders":   [ { id, position, size, normal, label } ],
//     "walls":       [ ... ],
//     "props":       [ ... ],
//     "budgets_raw_bytes": { floor, walls, props, nav, occluders }
//   }
//
// Design (Senior UI/UX):
//   - Hex-grid floor 7×8м, dark metal + emissive cyan lines (sci-fi command bridge)
//   - Walls 4 шт + viewports + console strips
//   - Props: captain chair + curved main console (3 sectors) + 4 side terminals + 2 holo-projectors
//   - Nav-points: 7 точек (центр + консоль + side terminals + 2 входа в зону ходьбы)
//   - Occluders: 4 плоскости рядом со стенами — при hide-UI панели уходят за стену
//
// CC0: собственный код, без внешних моделей. HDR `bridge_env_1k.hdr` — CC0 Poly Haven
// (см. CREDITS.md).

import * as THREE from "three";
import { Document, NodeIO } from "@gltf-transform/core";
import { ALL_EXTENSIONS } from "@gltf-transform/extensions";
import { promises as fs } from "node:fs";
import path from "node:path";

const OUT_DIR = path.resolve(
  process.cwd(),
  "public/models/environment/_raw",
);
const META_OUT = path.join(OUT_DIR, "bridge_scene_meta.json");

const BUDGETS = {
  floor: 120 * 1024,
  walls: 180 * 1024,
  props: 250 * 1024,
  nav: 30 * 1024,
  occluders: 20 * 1024,
};

// --- палитра (sci-fi command bridge, dark + cyan accents) ---
const COL = {
  floorBase: 0x14181f,
  floorEdge: 0x2ec27e,
  floorPanel: 0x1a2230,
  wallBase: 0x181c25,
  wallPanel: 0x222831,
  wallViewport: 0x0a0d11,
  wallEdge: 0x2ec27e,
  console: 0x1f242e,
  consoleStrip: 0x2ec27e,
  consoleScreen: 0x0a4d4d,
  chair: 0x0e1116,
  chairCush: 0x1a2230,
  chairEdge: 0x2ec27e,
  holo: 0x44ddff,
  // nav-points (видимые в editor, runtime-скрытые через opacity=0)
  navOrigin: 0x44ddff,
  navConsole: 0xffd24a,
  navTerminal: 0xff7e6b,
  navEntry: 0xb388ff,
  // occluders (полупрозрачные cyan, в runtime opacity=0)
  occluder: 0x44ddff,
};

// --- low-poly hex tile geometry ---
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

function mergeGeoms(geoms) {
  if (geoms.length === 0) return null;
  const merged = new THREE.BufferGeometry();
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

// Room dimensions (m) — обязательно соблюдаются в floor/walls/props/nav/occluders,
// чтобы safe-walk-area действительно была внутри walls.
const ROOM_W = 7;   // x-axis (left-right)
const ROOM_D = 8;   // z-axis (front-back)
const ROOM_H = 3;   // y-axis (height)

// ===== FLOOR =====
function buildFloor() {
  const group = new THREE.Group();
  group.name = "bridge_floor";
  const r = 0.6;
  const h = 0.05;
  const w = Math.sqrt(3) * r;
  const cols = 6;
  const rows = 6;

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

// ===== WALLS =====
function makeWallPanel(width, height, color) {
  return new THREE.Mesh(
    new THREE.BoxGeometry(width, height, 0.05),
    new THREE.MeshStandardMaterial({ color, roughness: 0.85, metalness: 0.4, flatShading: true })
  );
}
function makeConsoleStrip(width, height, color) {
  return new THREE.Mesh(
    new THREE.BoxGeometry(width, height, 0.08),
    new THREE.MeshStandardMaterial({
      color, roughness: 0.5, metalness: 0.7, emissive: color, emissiveIntensity: 0.7, flatShading: true,
    })
  );
}
function makeViewport(width, height, color) {
  return new THREE.Mesh(
    new THREE.BoxGeometry(width, height, 0.04),
    new THREE.MeshStandardMaterial({
      color, roughness: 0.2, metalness: 0.3, emissive: color, emissiveIntensity: 0.1, flatShading: true,
    })
  );
}

function buildWalls() {
  const group = new THREE.Group();
  group.name = "bridge_walls";

  // back wall (за спиной капитана)
  const backWall = makeWallPanel(ROOM_W, ROOM_H, COL.wallBase);
  backWall.position.set(0, ROOM_H / 2, -ROOM_D / 2);
  group.add(backWall);
  const backViewport = makeViewport(ROOM_W * 0.7, ROOM_H * 0.5, COL.wallViewport);
  backViewport.position.set(0, ROOM_H * 0.55, -ROOM_D / 2 + 0.03);
  group.add(backViewport);
  const backStripL = makeConsoleStrip(0.1, ROOM_H * 0.7, COL.consoleStrip);
  backStripL.position.set(-ROOM_W * 0.42, ROOM_H * 0.55, -ROOM_D / 2 + 0.06);
  group.add(backStripL);
  const backStripR = makeConsoleStrip(0.1, ROOM_H * 0.7, COL.consoleStrip);
  backStripR.position.set(ROOM_W * 0.42, ROOM_H * 0.55, -ROOM_D / 2 + 0.06);
  group.add(backStripR);

  // front wall (перед капитаном)
  const frontWall = makeWallPanel(ROOM_W, ROOM_H, COL.wallBase);
  frontWall.position.set(0, ROOM_H / 2, ROOM_D / 2);
  group.add(frontWall);
  for (let i = 0; i < 4; i++) {
    const panel = makeWallPanel(ROOM_W * 0.18, ROOM_H * 0.25, COL.wallPanel);
    panel.position.set(-ROOM_W * 0.36 + i * ROOM_W * 0.24, ROOM_H * 0.65, ROOM_D / 2 + 0.04);
    group.add(panel);
  }
  for (let i = 0; i < 3; i++) {
    const strip = makeConsoleStrip(ROOM_W * 0.25, 0.08, COL.consoleStrip);
    strip.position.set(-ROOM_W * 0.3 + i * ROOM_W * 0.3, ROOM_H * 0.18, ROOM_D / 2 + 0.06);
    group.add(strip);
  }

  // left wall
  const leftWall = makeWallPanel(ROOM_D * 0.95, ROOM_H, COL.wallBase);
  leftWall.position.set(-ROOM_W / 2 - 0.02, ROOM_H / 2, 0);
  leftWall.rotation.y = Math.PI / 2;
  group.add(leftWall);
  const leftPanel = makeWallPanel(1.5, 1.2, COL.wallPanel);
  leftPanel.position.set(-ROOM_W / 2 + 0.04, 1.6, -1.5);
  leftPanel.rotation.y = Math.PI / 2;
  group.add(leftPanel);
  const leftViewport = makeViewport(1.8, 1.0, COL.wallViewport);
  leftViewport.position.set(-ROOM_W / 2 + 0.04, 1.8, 1.5);
  leftViewport.rotation.y = Math.PI / 2;
  group.add(leftViewport);

  // right wall
  const rightWall = makeWallPanel(ROOM_D * 0.95, ROOM_H, COL.wallBase);
  rightWall.position.set(ROOM_W / 2 + 0.02, ROOM_H / 2, 0);
  rightWall.rotation.y = -Math.PI / 2;
  group.add(rightWall);
  const rightPanel = makeWallPanel(1.5, 1.2, COL.wallPanel);
  rightPanel.position.set(ROOM_W / 2 - 0.04, 1.6, 1.5);
  rightPanel.rotation.y = -Math.PI / 2;
  group.add(rightPanel);
  const rightViewport = makeViewport(1.8, 1.0, COL.wallViewport);
  rightViewport.position.set(ROOM_W / 2 - 0.04, 1.8, -1.5);
  rightViewport.rotation.y = -Math.PI / 2;
  group.add(rightViewport);

  return group;
}

// ===== PROPS =====
function makeChairBase() {
  const group = new THREE.Group();
  const base = new THREE.Mesh(
    new THREE.CylinderGeometry(0.35, 0.4, 0.15, 8),
    new THREE.MeshStandardMaterial({ color: COL.chair, roughness: 0.6, metalness: 0.6, flatShading: true })
  );
  base.position.y = 0.075;
  group.add(base);
  const pillar = new THREE.Mesh(
    new THREE.CylinderGeometry(0.08, 0.08, 0.5, 6),
    new THREE.MeshStandardMaterial({ color: COL.chair, roughness: 0.5, metalness: 0.8, flatShading: true })
  );
  pillar.position.y = 0.4;
  group.add(pillar);
  const seat = new THREE.Mesh(
    new THREE.BoxGeometry(0.5, 0.1, 0.5),
    new THREE.MeshStandardMaterial({ color: COL.chairCush, roughness: 0.9, metalness: 0.1, flatShading: true })
  );
  seat.position.y = 0.85;
  group.add(seat);
  const back = new THREE.Mesh(
    new THREE.BoxGeometry(0.5, 0.7, 0.1),
    new THREE.MeshStandardMaterial({ color: COL.chairCush, roughness: 0.9, metalness: 0.1, flatShading: true })
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
      new THREE.MeshStandardMaterial({ color: COL.chair, roughness: 0.5, metalness: 0.7, flatShading: true })
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
      new THREE.MeshStandardMaterial({ color: COL.console, roughness: 0.7, metalness: 0.5, flatShading: true })
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
    new THREE.MeshStandardMaterial({ color: COL.console, roughness: 0.6, metalness: 0.6, flatShading: true })
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
    new THREE.MeshStandardMaterial({ color: COL.console, roughness: 0.5, metalness: 0.7, flatShading: true })
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
    new THREE.MeshStandardMaterial({ color: COL.console, roughness: 0.5, metalness: 0.8, flatShading: true })
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
  group.add(makeMainConsole());
  group.add(makeSideTerminal(-3.2, 0.5, Math.PI / 2));
  group.add(makeSideTerminal(-3.2, -0.5, Math.PI / 2));
  group.add(makeSideTerminal(3.2, 0.5, -Math.PI / 2));
  group.add(makeSideTerminal(3.2, -0.5, -Math.PI / 2));
  group.add(makeHoloProjector(-1.0, -1.8));
  group.add(makeHoloProjector(1.0, -1.8));
  return group;
}

// ===== NAV-POINTS (AABB маркеры для safe-walk-area / XR-сессии) =====
// Эти mesh'ы — редакторские маркеры: видны в dev (opacity 0.6), скрыты в
// production (opacity 0). Runtime использует их bounding-box для navmesh
// и как "safe teleport" якоря для XR-сессии.
function buildNav() {
  const group = new THREE.Group();
  group.name = "bridge_nav";
  const defs = [
    // id, position, size (xyz), kind, label
    { id: "origin",     pos: [0, 0.05, 0],         size: [0.2, 0.1, 0.2],  kind: "origin",  color: COL.navOrigin,  label: "user spawn (0,0,0)" },
    { id: "console-c",  pos: [0, 0.5, 1.8],       size: [3.6, 1.0, 0.6],  kind: "console",  color: COL.navConsole,  label: "main console area" },
    { id: "terminal-l", pos: [-3.2, 0.5, 0.5],     size: [0.7, 1.0, 0.5],  kind: "terminal", color: COL.navTerminal, label: "left side terminal" },
    { id: "terminal-r", pos: [3.2, 0.5, 0.5],      size: [0.7, 1.0, 0.5],  kind: "terminal", color: COL.navTerminal, label: "right side terminal" },
    { id: "entry-back", pos: [0, 0.05, -3.5],      size: [1.4, 0.1, 0.6],  kind: "entry",    color: COL.navEntry,    label: "back entry (teleport anchor)" },
    { id: "entry-front", pos: [0, 0.05, 3.5],      size: [1.4, 0.1, 0.6],  kind: "entry",    color: COL.navEntry,    label: "front entry (teleport anchor)" },
    { id: "holo-left",  pos: [-1.0, 0.5, -1.8],    size: [0.6, 1.2, 0.6],  kind: "holo",     color: COL.navOrigin,   label: "left holo-projector zone" },
    { id: "holo-right", pos: [1.0, 0.5, -1.8],     size: [0.6, 1.2, 0.6],  kind: "holo",     color: COL.navOrigin,   label: "right holo-projector zone" },
  ];
  const navPoints = [];
  for (const d of defs) {
    const m = new THREE.Mesh(
      new THREE.BoxGeometry(d.size[0], d.size[1], d.size[2]),
      new THREE.MeshStandardMaterial({
        color: d.color, emissive: d.color, emissiveIntensity: 0.4,
        transparent: true, opacity: 0.6, roughness: 0.5, metalness: 0.1,
        depthWrite: false,
        flatShading: true,
      })
    );
    m.position.set(d.pos[0], d.pos[1], d.pos[2]);
    m.userData.navId = d.id;
    m.userData.kind = d.kind;
    m.userData.label = d.label;
    m.userData.aabb = {
      min: [d.pos[0] - d.size[0] / 2, d.pos[1] - d.size[1] / 2, d.pos[2] - d.size[2] / 2],
      max: [d.pos[0] + d.size[0] / 2, d.pos[1] + d.size[1] / 2, d.pos[2] + d.size[2] / 2],
    };
    group.add(m);
    navPoints.push({ id: d.id, position: d.pos, size: d.size, kind: d.kind, label: d.label });
  }
  group.userData.navPoints = navPoints;
  return group;
}

// ===== OCCLUDERS (плоскости рядом со стенами для скрытия UI-панелей) =====
// Эти плоскости имеют material.colorWrite=false в runtime — они
// реально рендерятся в depth-buffer, но не пишут в color buffer, поэтому
// UI-панели за ними корректно скрываются стенами.
function buildOccluders() {
  const group = new THREE.Group();
  group.name = "bridge_occluders";
  const defs = [
    // id, position, size, normal (rotation Y in radians)
    { id: "occ-back",    pos: [0, ROOM_H / 2, -ROOM_D / 2 + 0.04], size: [ROOM_W + 0.4, ROOM_H + 0.2, 0.01], rotY: 0,                label: "back wall occluder" },
    { id: "occ-front",   pos: [0, ROOM_H / 2, ROOM_D / 2 - 0.04],  size: [ROOM_W + 0.4, ROOM_H + 0.2, 0.01], rotY: 0,                label: "front wall occluder" },
    { id: "occ-left",    pos: [-ROOM_W / 2 + 0.04, ROOM_H / 2, 0], size: [ROOM_D + 0.4, ROOM_H + 0.2, 0.01], rotY: Math.PI / 2,      label: "left wall occluder" },
    { id: "occ-right",   pos: [ROOM_W / 2 - 0.04, ROOM_H / 2, 0],  size: [ROOM_D + 0.4, ROOM_H + 0.2, 0.01], rotY: -Math.PI / 2,     label: "right wall occluder" },
  ];
  const occluders = [];
  for (const d of defs) {
    const m = new THREE.Mesh(
      new THREE.BoxGeometry(d.size[0], d.size[1], d.size[2]),
      new THREE.MeshBasicMaterial({
        color: COL.occluder,
        transparent: true, opacity: 0.15,
        side: THREE.DoubleSide,
        depthWrite: true,
      })
    );
    m.position.set(d.pos[0], d.pos[1], d.pos[2]);
    m.rotation.y = d.rotY;
    m.userData.occluderId = d.id;
    m.userData.label = d.label;
    m.userData.normal = [0, 0, d.rotY === 0 ? (d.id === "occ-back" ? -1 : 1) : 0];
    m.userData.aabb = {
      min: [d.pos[0] - d.size[0] / 2, d.pos[1] - d.size[1] / 2, d.pos[2] - d.size[2] / 2],
      max: [d.pos[0] + d.size[0] / 2, d.pos[1] + d.size[1] / 2, d.pos[2] + d.size[2] / 2],
    };
    group.add(m);
    occluders.push({ id: d.id, position: d.pos, size: d.size, normal: m.userData.normal, label: d.label });
  }
  group.userData.occluders = occluders;
  return group;
}

// ===== EXPORT three.js Group → glb =====
async function exportGroupToGLB(group, outputPath) {
  const doc = new Document();
  const buf = doc.createBuffer();

  const meshList = [];
  group.traverse((o) => { if (o.isMesh) meshList.push(o); });

  // КРИТИЧНО: перед запеканием world-трансформа надо явно обновить
  // matrixWorld всей иерархии. Без этого mesh.matrixWorld — identity
  // (нода не рендерилась), и `geom.applyMatrix4(mesh.matrixWorld)` теряет
  // позиции wall/props (они коллапсируют в origin — «стена сквозь
  // спавн-точку»). Floor не страдал, т.к. его позиции запечены в
  // геометрию через geometry.translate().
  group.updateMatrixWorld(true);

  // Уникальные материалы
  const matMap = new Map();
  for (const m of meshList) {
    const mat = m.material;
    const emissiveHex = mat.emissive ? mat.emissive.getHexString() : "0";
    const key = `${mat.color.getHexString()}_${mat.metalness ?? 0}_${mat.roughness ?? 0.5}_${emissiveHex}_${mat.emissiveIntensity ?? 0}_${mat.transparent ? 1 : 0}_${mat.opacity ?? 1}`;
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
    const key = `${mat.color.getHexString()}_${mat.metalness ?? 0}_${mat.roughness ?? 0.5}_${emissiveHex}_${mat.emissiveIntensity ?? 0}_${mat.transparent ? 1 : 0}_${mat.opacity ?? 1}`;
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
    if (mesh.userData?.navId) {
      n.setExtras({ navId: mesh.userData.navId, kind: mesh.userData.kind, label: mesh.userData.label });
    } else if (mesh.userData?.occluderId) {
      n.setExtras({ occluderId: mesh.userData.occluderId, label: mesh.userData.label });
    }
    doc.createScene(group.name).addChild(n);
  }

  const io = new NodeIO().registerExtensions(ALL_EXTENSIONS);
  const glb = await io.writeBinary(doc);
  await fs.writeFile(outputPath, glb);
  return { size: glb.byteLength, meshes: meshList.length, materials: materials.length };
}

// ===== MAIN =====
async function main() {
  console.log("[bridge_assets] синтез low-poly sci-fi Bridge scene (Phase 2.1)...");
  console.log(`OUT_DIR = ${OUT_DIR}`);
  await fs.mkdir(OUT_DIR, { recursive: true });

  const tasks = [
    { name: "bridge_floor.glb",     group: buildFloor(),     budget: BUDGETS.floor },
    { name: "bridge_walls.glb",     group: buildWalls(),     budget: BUDGETS.walls },
    { name: "bridge_props.glb",     group: buildProps(),     budget: BUDGETS.props },
    { name: "bridge_nav.glb",       group: buildNav(),       budget: BUDGETS.nav },
    { name: "bridge_occluders.glb", group: buildOccluders(), budget: BUDGETS.occluders },
  ];

  const navGroup = tasks.find((t) => t.name === "bridge_nav.glb").group;
  const occGroup = tasks.find((t) => t.name === "bridge_occluders.glb").group;

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

  // ---- meta JSON ----
  // safe_walk_area: AABB на полу (y=0), внутри walls, исключая zone у
  // main console (z>1.5) и side terminals (|x|>3.0).
  const safeWalkArea = {
    min: [-ROOM_W / 2 + 0.3, 0.0, -ROOM_D / 2 + 0.3],
    max: [ROOM_W / 2 - 0.3, 2.5, ROOM_D / 2 - 0.3],
    note: "polygon AABB внутри стен; не включает зону main console (z >= 1.5) и side terminals (|x| >= 3.0). Размер: ~6.4×2.5×7.4м — комфортно для room-scale VR с центром в (0, 0, 0).",
  };
  const meta = {
    design: {
      style: "low-poly sci-fi Captain Bridge",
      palette: Object.fromEntries(Object.entries(COL).map(([k, v]) => [k, "0x" + v.toString(16).padStart(6, "0")])),
      lighting_plan: {
        ambient: { color: "0xffffff", intensity: 0.6 },
        directional: { color: "0xffffff", intensity: 0.4, position: [2, 4, 1] },
        ibl: { source: "bridge_env_1k.hdr", format: "Radiance HDR (Poly Haven CC0)", target_ktx2_after_phase_2_0: "≤ 400 KB" },
      },
      dimensions_m: { width: ROOM_W, depth: ROOM_D, height: ROOM_H },
      ceiling: "open (room-scale VR)",
    },
    safe_walk_area: safeWalkArea,
    nav_points: navGroup.userData.navPoints,
    occluders: occGroup.userData.occluders,
    budgets_raw_bytes: BUDGETS,
    files: results,
  };
  await fs.writeFile(META_OUT, JSON.stringify(meta, null, 2) + "\n");
  console.log(`  bridge_scene_meta.json: written (${Object.keys(meta).length} top-level keys, ${navGroup.userData.navPoints.length} nav-points, ${occGroup.userData.occluders.length} occluders)`);

  const failed = results.filter((r) => !r.ok);
  if (failed.length > 0) {
    console.error(`[FAIL] ${failed.length} файлов превысили budget:`);
    failed.forEach((f) => console.error(`  ${f.name}: ${(f.size / 1024).toFixed(1)} KB / ${(f.budget / 1024).toFixed(0)} KB`));
    process.exit(1);
  }

  console.log("[bridge_assets] all 5 glb экспортированы, в budget. Следующий шаг: npm run gltf:optimize");
}

main().catch((e) => { console.error(e); process.exit(1); });