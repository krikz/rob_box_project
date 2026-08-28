# `@rob-box/quest-webxr-client`

Meta Quest / WebXR Captain Bridge client for `rob_box_quest`.
Phase 2+ stack per [ADR-0032](../../../docs/adr/0032-meta-quest-webxr-stack-and-assets.md):
Three.js r170 + native WebXR + glTF 2.0 asset pipeline (Draco + Meshopt + KTX2).

This README covers the **asset pipeline** (Phase 2.0). For application code
(rendering, teleop, XR bootstrap) see `src/` and the ADR.

---

## Asset pipeline (CC0-only)

All 3D assets committed under `public/models/` MUST be CC0 (or CC-BY with a
row in `CREDITS.md`) AND MUST be **optimized**. Raw glTF source files
(`.glb`, `.gltf` produced by Blender / Quaternius / Khronos Sample Assets)
are forbidden in the tree — the CI guard `npm run gltf:verify` rejects any
`.glb` that lacks `KHR_draco_mesh_compression` + `EXT_meshopt_compression`.

Why: per ADR-0032 §3.2 we have hard size budgets (environment ≤ 2 MB,
avatar ≤ 500 KB, panel ≤ 150 KB). Uncompressed glTF bloats past these
budgets and forces 3G/Meta-Quest bandwidth choices we'd rather not make.

### Tooling

| Script                   | Purpose                                                                                |
| ------------------------ | -------------------------------------------------------------------------------------- |
| `npm run gltf:optimize`  | Walks `public/models/` and emits `<name>.optimized.glb` (Draco + Meshopt, optional WebP)|
| `npm run gltf:verify`    | CI guard — fails on any `.glb` missing required extensions or over budget               |
| `npm run build:avatar`   | Programmatically synthesises the 4-wheeled robot avatar (CC0) — Phase 2.1. See [§Avatar](#avatar-phase-21) below. |

### Pipeline transforms (in order)

1. `dedup()` — drop duplicate vertices / texture data.
2. `prune()` — drop unused nodes / textures / materials.
3. `resample()` — lossless animation frame resample.
4. `draco({...})` — `KHR_draco_mesh_compression` (static meshes).
5. `meshopt({...})` — `EXT_meshopt_compression` (geometry + animation tracks).
6. `textureCompress()` *(opt-in via `sharp`)* — WebP texture compression.

`KHR_texture_basisu` (KTX2 / Basis) is **not** emitted by default. The full
KTX2 path requires the native `ktx` binary from KTX-Software v4.3.0+ and is
opt-in; until then `gltf-verify` reports it as a warning (not an error). See
ADR-0032 §3.2 for the rationale.

### Per-category size budget

Driven by the first path segment under `public/models/`:

| Category       | Budget  | Example                         |
| -------------- | ------- | ------------------------------- |
| `environment/` | ≤ 2 MB  | `public/models/environment/...` |
| `avatar/`      | ≤ 500 KB| `public/models/avatar/...`      |
| `panel/`       | ≤ 150 KB| `public/models/panel/...`       |
| `texture/`     | ≤ 5 MB  | `public/models/texture/...`     |
| `hdr/`         | ≤ 600 KB| `public/models/hdr/...`         |
| (unclassified) | ≤ 150 KB| default = strictest (fail loud) |

Files that fall outside a recognised category default to the panel budget;
add a directory under `public/models/` if you need a looser limit.

### Adding a new asset

1. Drop the CC0 source (or hand-authored) `.glb` into the matching
   `public/models/<category>/` directory — **but mark it `.gitignore`d**;
   the `.gitignore` already excludes `public/models/**/*.glb` except
   `*.optimized.glb`, so you don't need to do anything extra.
2. Run `npm run gltf:optimize` — it writes
   `public/models/<category>/<name>.optimized.glb`.
3. Commit the `.optimized.glb` + (if new) a `CREDITS.md` row.
4. PR — CI runs `gltf:verify` automatically.

If you need to commit an asset that legitimately must stay raw (e.g. a
source-of-truth mesh exported by an artist), add it to `.gitignore`
allowlist with a comment explaining why and link the ADR that authorises
the exception.

---

## Captain Bridge environment (Phase 2.1, kanban t_0bd54b80)

Five `.glb` files + one HDR live under `public/models/environment/`,
representing the Captain Bridge scene that hosts hand-tracking + UI
panels.

| File                          | Optimized size | Purpose                                                                |
| ----------------------------- | -------------- | ---------------------------------------------------------------------- |
| `bridge_floor.optimized.glb`  |  22 KB         | Hex-grid floor (6×6 tiles, dark metal + emissive cyan edges).          |
| `bridge_walls.optimized.glb`  |  12 KB         | 4 walls + viewports + console strips.                                  |
| `bridge_props.optimized.glb`  |  24 KB         | Captain chair + main console + side terminals + holo-projectors.       |
| `bridge_nav.optimized.glb`    |   8 KB         | 8 AABB markers for `safe_walk_area` + XR teleport anchors.             |
| `bridge_occluders.optimized.glb` |  4 KB      | 4 wall-coincident planes for hiding UI panels behind walls.            |
| `bridge_scene_meta.json`      |   5 KB         | Runtime metadata (design, safe_walk_area, nav_points, occluders).      |
| `hdr/bridge_env_1k.hdr`       | 1.6 MB         | Radiance HDR for IBL on metallic parts (Poly Haven CC0).               |

**Total GLB payload: 70 KB** — 3.4% of the 2 MB `environment/` budget per
ADR-0032 §3.2.

### Why synthesized meshes (not Quaternius packs)

The Quaternius sci-fi packs are CC0 but distributed via itch.io
pay-what-you-want and Google Drive share-link — neither automatable
from this environment. The bridge is therefore synthesized from
three.js primitives via `scripts/build_bridge_assets.mjs`, with the
exact dimensions and palette documented in `bridge_scene_meta.json`.
If товарищ Шифу wants the Quaternius-style models specifically, the
swap is mechanical: drop the Quaternius `.glb` files into
`public/models/environment/_raw/` and re-run `npm run gltf:optimize`.

### Regenerating the environment

```bash
# 1. Re-synthesize raw GLB (writes to public/models/environment/_raw/)
node scripts/build_bridge_assets.mjs

# 2. Run the Phase 2.0 pipeline → *.optimized.glb in public/models/environment/
npm run gltf:optimize

# 3. CI guard — must PASS before commit
npm run gltf:verify
```

`scripts/build_bridge_assets.mjs` also writes `bridge_scene_meta.json`
which carries the `safe_walk_area` AABB, the 8 nav-points, and the 4
occluder definitions consumed at runtime by `src/scene/bridge_assets.ts`.

### Runtime integration

`src/scene/bridge_assets.ts` exports `loadBridgeAssets(scene, renderer, opts)`
which loads all 5 GLB + the HDR via `GLTFLoader + DRACOLoader +
MeshoptDecoder + RGBELoader + PMREMGenerator`. The returned
`BridgeAssetHandle` exposes `navPoints`, `occluders`, and `safeWalkArea`
to the rest of the app — the XR layer can use the `entry`-tagged
nav-points as teleport anchors, and the panel manager can use the
occluder AABBs to clip panels against walls.

`src/scene/captain_bridge.ts` integrates the loader via
`bridge.loadEnvironment()` — fail-soft, so a missing GLB does not
break the procedural fallback floor + grid.

### Lighting & IBL

- Ambient: `0xffffff` × 0.6.
- Directional: `0xffffff` × 0.4, position `(2, 4, 1)`.
- IBL: `hdr/bridge_env_1k.hdr` via `THREE.PMREMGenerator` →
  `scene.environment`. `scene.background` is intentionally NOT set
  (bridge walls/viewports provide the dark interior look). KTX2 path
  for the HDR is opt-in (Phase 2.0); see *KTX2 / Basis pipeline* below.

---

## Avatar (Phase 2.1)

The avatar is a low-poly 4-wheeled robot synthesised programmatically by
[`scripts/build-avatar.mjs`](scripts/build-avatar.mjs). All geometry is
original code (Three.js `BoxGeometry` + `CylinderGeometry` + `SphereGeometry`)
authored as part of `rob_box_quest`, released under **CC0** — no external
mesh data or textures are referenced. See [`public/models/avatar/CREDITS.md`](public/models/avatar/CREDITS.md)
for the full attribution.

Output artefact: [`public/models/avatar/avatar.glb`](public/models/avatar/avatar.glb)
— ~19 KB after Draco + Meshopt (3.8 % of the 500 KB budget per
[ADR-0032 §3.2](../../../docs/adr/0032-meta-quest-webxr-stack-and-assets.md)).
`avatar.optimized.glb` is the direct pipeline output and is byte-identical
to `avatar.glb` at commit time (copied by `build-avatar.mjs --publish` so
the runtime can drop the `.optimized.` suffix).

### Geometry

- Chassis: `BoxGeometry 0.6 × 0.18 × 0.8` (light-grey-blue PBR, metallic 0.6 / roughness 0.45)
- 4 wheels: `CylinderGeometry R=0.16, w=0.12, 16 seg` (dark rubber)
- Head: `BoxGeometry 0.28 × 0.18 × 0.18` (teal sensor head, emissive cyan trim)
- Mast + tip: thin cylinder + emissive red sphere LED
- Total: 9 meshes, 6 PBR materials, 9 nodes, ≈700 triangles before optimisation

### Animations (5 clips, all LOOP, LINEAR interpolation)

| Name             | Duration | Channels | Notes                                                  |
| ---------------- | -------- | -------- | ------------------------------------------------------ |
| `idle`           | 3.0 s    | 2        | chassis bob ±1 cm (sin) + head yaw scan ±10° (cos)     |
| `drive_forward`  | 1.0 s    | 4        | all wheels at +π rad/s around the X axis               |
| `drive_backward` | 1.0 s    | 4        | all wheels at −π rad/s                                  |
| `turn_left`      | 1.0 s    | 4        | front ×1.5 speed, rear ×0.5 speed                      |
| `turn_right`     | 1.0 s    | 4        | mirror of `turn_left`                                   |

### Rebuild

```
npm run build:avatar         # raw → public/models/avatar/_raw/avatar.glb (~39 KB)
npm run gltf:optimize        # Draco + Meshopt → _raw/avatar.optimized.glb (~19 KB)
node scripts/build-avatar.mjs --publish   # move optimized → public/models/avatar/, drop raw
cp public/models/avatar/avatar.optimized.glb public/models/avatar/avatar.glb
npm run gltf:verify          # extension + budget guard
npm test                     # unit tests (4 gltf-pipeline + 6 avatar-pipeline + ...)
```

### Runtime loading

```ts
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { DRACOLoader } from 'three/examples/jsm/loaders/DRACOLoader.js';
import { MeshoptDecoder } from 'meshoptimizer';

const loader = new GLTFLoader();
loader.setDRACOLoader(new DRACOLoader().setDecoderPath('/draco/'));
loader.setMeshoptDecoder(MeshoptDecoder);

const gltf = await loader.loadAsync('/models/avatar/avatar.glb');
const mixer = new THREE.AnimationMixer(gltf.scene);
const idle = mixer.clipAction(gltf.animations.find(a => a.name === 'idle'));
idle.play();
```

---

## KTX2 / Basis pipeline (opt-in)

To enable full KTX2 texture compression (`KHR_texture_basisu`):

1. Install [KTX-Software v4.3.0+](https://github.com/KhronosGroup/KTX-Software)
   (`ktx` CLI must be on `$PATH`).
2. Run `npx @gltf-transform/cli etc1s public/models/in.glb --texture-compressor ktx2 --out public/models/in.optimized.glb`.
3. `npm run gltf:verify` — the warning disappears, hard-error budget holds.

KTX-Software is a native binary and is not currently bundled in the GitHub
Actions runners; we ship WebP-via-sharp as the default progressive
enhancement.

---

## Tests

- `npm run test` — vitest unit tests, including `tests/gltf_pipeline.test.ts`
  which round-trips the committed `Duck.optimized.glb` through
  `gltf-transform` + Draco + Meshopt decoders and asserts the optimizer
  actually emitted the required extensions.
- `npm run typecheck` — `tsc --noEmit` over `src/` and `tests/`.

The Three.js browser loader path (`GLTFLoader + DRACOLoader + MeshoptDecoder`)
is exercised at runtime in the WebXR client on the Quest device — the unit
test validates the artifact, not the browser path (jsdom has no
`new Worker(...)`).

---

## References

- ADR-0032 — Meta Quest / WebXR stack and assets
  ([../../../docs/adr/0032-meta-quest-webxr-stack-and-assets.md](../../../docs/adr/0032-meta-quest-webxr-stack-and-assets.md))
- Research note — WebXR best practices ([../../../docs/research/2026-08-26-meta-quest-webxr-best-practices.md](../../../docs/research/2026-08-26-meta-quest-webxr-best-practices.md))
- [gltf-transform docs](https://gltf-transform.dev/)
- [Three.js GLTFLoader](https://threejs.org/docs/#examples/en/loaders/GLTFLoader)
- [Khronos KTX-Software](https://github.com/KhronosGroup/KTX-Software)
