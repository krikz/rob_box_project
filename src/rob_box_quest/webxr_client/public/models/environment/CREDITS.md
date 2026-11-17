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
  (`tests/gltf_pipeline.test.ts`). Lives at `tests/fixtures/Duck.optimized.glb`
  — not under `public/models/`, since it's a test fixture, not a shipped
  scene asset (moved there in the #3044-#3048 review pass so
  `gltf-verify.mjs` / `bridge_environment.test.ts` don't have to special-case
  it). The committed file is regenerated locally from a freshly downloaded
  CC0 source via `npm run gltf:optimize` — it is not redistributed as a
  sample; the pipeline round-trip is the point.

## Captain Bridge environment (Phase 2.1, kanban t_0bd54b80, issue #1677)

The Captain Bridge is the immersive "space bridge" room that hosts
hand-tracking + UI panels. Eight committed `.glb`
assets live under `public/models/environment/` and a single HDR for IBL
under `public/models/environment/hdr/`. Five are synthesized (CC0 own code)
and three are Tripo3D hero props (AI-generated, see §Hero props below).

| File                          | Size (optimized) | Purpose                                                                                    |
| ----------------------------- | ---------------- | ------------------------------------------------------------------------------------------ |
| `bridge_floor.optimized.glb`  | 22.0 KB          | Hex-grid floor (6×6 tiles, radius 0.6m, emissive cyan edges, central panel inlay).         |
| `bridge_walls.optimized.glb`  | 20.9 KB          | Back/front/left/right walls + 2 viewports + console strips + wall panels.                  |
| `bridge_props.optimized.glb`  | 45.9 KB          | Captain chair + curved main console (3 sectors) + 4 side terminals (голо-проекторы убраны 30.09.2026). |
| `bridge_nav.optimized.glb`    | 10.3 KB          | 6 AABB markers for `safe_walk_area` + XR teleport anchors (origin, console, terminals, entries). |
| `bridge_occluders.optimized.glb` | 6.2 KB       | 4 semi-transparent planes coincident with walls — depth-write only, used to hide UI panels behind walls at runtime. |
| `bridge_platform.optimized.glb` | 499.7 KB       | Tripo3D hero prop — central hex pedestal under the operator (spawn `(0,0,0)`). See §Hero props. |
| `bridge_screen.optimized.glb`   | 244.2 KB       | Tripo3D hero prop — universal 16:9 screen frame; reused for main screen, TARS wings, ceiling and floating panels. See §Hero props. |
| `bridge_scene_meta.json`      |  5.1 KB          | Structured scene metadata: design palette, lighting plan, `safe_walk_area` AABB, nav-points, occluders, raw-size budgets. Loaded at runtime to drive navmesh / XR teleport anchors / UI panel placement. |
| `hdr/bridge_env_1k.hdr`       | 1.63 MB          | Radiance HDR for IBL on metallic bridge parts. **Not** compressed — Phase 2.0 KTX2 path is opt-in; HDR lives outside the glTF ≤ 2 MB budget per ADR-0032 §3.2. |

**Total committed `.glb` size: 0.82 MB** (7 files, ~41% of the 2 MB
`environment/` budget per ADR-0032 §3.2). Breakdown: 5 synthesized `.glb`
= 105.2 KB (CC0 own code) + 2 Tripo3D hero props = 743.9 KB
(AI-generated, see §Hero props). Verified via `npm run gltf:verify`
(assets compliant, including the Duck smoke-test); sizes are
post-`gltf-transform` (Draco + Meshopt) and re-derivable from
`du -bc public/models/environment/*.optimized.glb`.

### Sources

| Asset                         | Source                                                                                          | License |
| ----------------------------- | ------------------------------------------------------------------------------------------------ | ------- |
| `bridge_floor/walls/props/nav/occluders/*.glb` | Synthesized procedurally from three.js primitives (BoxGeometry / ExtrudeGeometry / CylinderGeometry / PlaneGeometry) via `scripts/build_bridge_assets.mjs`. | **CC0 (own code, no third-party meshes)** |
| `bridge_platform/screen/*.glb` | AI-generated via Tripo3D (image-to-3D) from Qwen-generated reference images. Source `.glb` live in `_raw/` (gitignored); committed `.optimized.glb` are produced by `npm run gltf:optimize`. | **AI-generated — license status unsettled** (see §Hero props; flag before any public redistribution) |
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

The 6 AABB markers in `bridge_nav.optimized.glb` plus the
`safe_walk_area` block in `bridge_scene_meta.json` define where it's
safe to walk in room-scale VR. Run-time: read `nav_points` from the
JSON, walk their AABBs; treat points tagged `kind: "entry"` as
explicit teleport anchors (back / front of the room).

## Hero props (Tripo3D, 2026-09-26)

Two hero props augment the procedural `bridge_props` scene. Generated via
Tripo3D (image-to-3D) from Qwen-generated reference images. No third-party
meshes: source `.glb` live in `_raw/` (gitignored); committed `.optimized.glb`
are produced by `npm run gltf:optimize`.

| File | Purpose |
| ------------------------------- | -------------------------------------------------------------------------------------------------- |
| `bridge_platform.optimized.glb` | Central hex pedestal under the operator (spawn `(0,0,0)`). |
| `bridge_screen.optimized.glb`   | Universal 16:9 screen frame — reused for main screen, TARS wings, ceiling and floating panels. |

**License note:** AI-generated (Tripo3D from Qwen reference images), no
third-party meshes/textures. License status of AI-generated content is not yet
settled — flag before any public redistribution.

## Adding non-CC0 assets

When a non-CC0 source is added (e.g. CC-BY), append a row with author,
license, and source URL — never silently include a third-party asset.

## AI-generated assets

AI-generated assets (mesh/texture/audio produced by generative models
such as Tripo3D, MiniMax image, MiniMax TTS, voice cloning) are **NOT
CC0** by default and the redistribution status of AI-only output is
unsettled across jurisdictions. Per ADR-0136 each such asset MUST:

1. Be tracked in the per-area meta tracking issue with label
   `ai-asset` (currently: meta-issue for `rob_box_quest` environment
   assets — [tracking issue #3051](https://github.com/krikz/rob_box_project/issues/3051)).
2. Appear in this CREDITS.md with: generator name, generation date,
   tracking-issue link, current license status (`pending` / `yes` /
   `removed`), owner (товарищ Шифу).
3. Have a `<!-- ai-asset-policy: yes|pending|removed -->` trailer on
   the commit that introduced it.

**`pending` assets MUST NOT be included in public releases** (Docker
Hub tags, GitHub releases, web deploys). See
`docs/process/ai-generated-asset-handling.md` for the operational
procedure.
> 30.09.2026: голо-проекторы (`bridge_holo_projector.optimized.glb` и процедурные проекторы в `bridge_props`) убраны по решению Шифу — мешали обзору главного экрана.
