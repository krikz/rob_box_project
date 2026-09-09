# WebXR Captain Bridge — asset attributions

All `.glb` / `.ktx2` assets committed under `public/models/` MUST be
optimized via `npm run gltf:optimize` (Draco + Meshopt; optionally KTX2).
Raw glTF source files MUST NOT be committed — see `../README.md` and
`.gitignore` for the contract enforced by `npm run gltf:verify`.

## Duck (smoke-test reference asset)

- Source: Khronos Group glTF 2.0 Sample Assets — `Duck.glb`
  https://github.com/KhronosGroup/glTF-Sample-Assets/tree/main/Models/Duck
- License: CC0 1.0 Universal (public-domain dedication)
  https://creativecommons.org/publicdomain/zero/1.0/
- Used as a smoke-test target for the asset pipeline only
  (`tests/gltf_pipeline.test.ts`). The committed `Duck.optimized.glb` is
  regenerated locally from a freshly downloaded CC0 source via
  `npm run gltf:optimize` — it is not redistributed as a sample; the
  pipeline round-trip is the point.

## Captain Bridge environment (Phase 2.1, kanban t_0bd54b80, issue #1677)

The Captain Bridge is the immersive "space bridge" room that hosts
hand-tracking + UI panels. Five committed `.glb`
assets live under `public/models/environment/` and a single HDR for IBL
under `public/models/environment/hdr/`. All assets are CC0.

| File                          | Size (optimized) | Purpose                                                                                    |
| ----------------------------- | ---------------- | ------------------------------------------------------------------------------------------ |
| `bridge_floor.optimized.glb`  | 22.0 KB          | Hex-grid floor (6×6 tiles, radius 0.6m, emissive cyan edges, central panel inlay).         |
| `bridge_walls.optimized.glb`  | 11.9 KB          | Back/front/left/right walls + 2 viewports + console strips + wall panels.                  |
| `bridge_props.optimized.glb`  | 24.3 KB          | Captain chair + curved main console (3 sectors) + 4 side terminals + 2 holo-projectors.   |
| `bridge_nav.optimized.glb`    |  8.1 KB          | 8 AABB markers for `safe_walk_area` + XR teleport anchors (origin, console, terminals, entries, holo zones). |
| `bridge_occluders.optimized.glb` | 4.1 KB       | 4 semi-transparent planes coincident with walls — depth-write only, used to hide UI panels behind walls at runtime. |
| `bridge_scene_meta.json`      |  5.1 KB          | Structured scene metadata: design palette, lighting plan, `safe_walk_area` AABB, nav-points, occluders, raw-size budgets. Loaded at runtime to drive navmesh / XR teleport anchors / UI panel placement. |
| `hdr/bridge_env_1k.hdr`       | 1.63 MB          | Radiance HDR for IBL on metallic bridge parts. **Not** compressed — Phase 2.0 KTX2 path is opt-in; HDR lives outside the glTF ≤ 2 MB budget per ADR-0032 §3.2. |

**Total committed `.glb` size: 70.4 KB** (3.4% of the 2 MB `environment/`
budget per ADR-0032 §3.2).

### Sources

| Asset                         | Source                                                                                          | License |
| ----------------------------- | ------------------------------------------------------------------------------------------------ | ------- |
| `bridge_floor/walls/props/nav/occluders/*.glb` | Synthesized procedurally from three.js primitives (BoxGeometry / ExtrudeGeometry / CylinderGeometry / PlaneGeometry) via `scripts/build_bridge_assets.mjs`. | **CC0 (own code, no third-party meshes)** |
| `hdr/bridge_env_1k.hdr`       | Poly Haven — "Cayley Interior" (1K indoor studio interior HDRI). https://polyhaven.org/a/cayley_interior | **CC0 1.0 Universal (public-domain dedication)** |

### Why synthesized meshes instead of Quaternius packs

The original brief referenced Quaternius ("Room Kit", "Sci-Fi Props",
"Ultimate Space Kit") as the CC0 source for sci-fi environment assets.
Quaternius distributes those packs via:
  (a) itch.io pay-what-you-want — requires browser-flow with a button
      click that headless tooling cannot drive;
  (b) Google Drive share-link — requires OAuth or a manual download.

Both paths are not automatable from this environment without manual
intervention from товарищ Шифу. KhronosGroup/glTF-Sample-Assets were
considered as a fallback but contain only demo models (Duck, Box,
Lantern), not environment kits.

**Decision:** synthesize the low-poly sci-fi scene from three.js
primitives, so that:
  * the pipeline stays CC0 without external dependencies;
  * raw sizes fit per-file budgets (≤ 120 / 180 / 250 / 30 / 20 KB);
  * the visual language matches the Captain Bridge intent
    (hex-grid floor, dark-metal + emissive cyan accents, curved
    main console, captain chair, holo-projectors);
  * assets are regenerable from a single script
    (`scripts/build_bridge_assets.mjs`) — no hidden binary blobs.

If товарищ Шифу wants the Quaternius-style models specifically, the
swap is mechanical: drop the Quaternius `.glb` files into
`public/models/environment/_raw/` (gitignored) and re-run
`npm run gltf:optimize` — the optimized outputs will replace the
synthesized ones without changing the loader contract in
`src/scene/bridge_assets.ts`.

### Lighting plan (design)

- Ambient: `0xffffff` × 0.6 (cheap floor read).
- Directional: `0xffffff` × 0.4, position `(2, 4, 1)` (rim definition on
  props, soft shadow direction).
- IBL: `bridge_env_1k.hdr` via `THREE.PMREMGenerator.fromEquirectangular`,
  applied as `scene.environment` so that metalness ≥ 0.5 parts
  (floor edges, console strips, chair accent, holo-projector) actually
  reflect. After Phase 2.0 KTX2 path becomes the default, swap to
  `KTX2Loader` + ASTC/ETC2 to halve VRAM.
- The Captain Bridge scene does NOT set `scene.background` — the
  bridge walls/viewports provide the dark interior look; the HDR is
  only for reflection, not backdrop.

### Safe-walk-area (navmesh / XR teleport anchors)

The 8 AABB markers in `bridge_nav.optimized.glb` plus the
`safe_walk_area` block in `bridge_scene_meta.json` define where it's
safe to walk in room-scale VR. Run-time: read `nav_points` from the
JSON, walk their AABBs; treat points tagged `kind: "entry"` as
explicit teleport anchors (back / front of the room).

## Adding non-CC0 assets

When a non-CC0 source is added (e.g. CC-BY), append a row with author,
license, and source URL — never silently include a third-party asset.