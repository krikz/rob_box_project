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