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

## Avatar / Bridge / panel assets

See `public/models/avatar/CREDITS.md` (Phase 2.1, t_1fa6e505) and
`public/models/environment/CREDITS.md` (Phase 2.1, t_0bd54b80) when those
cards land. CC0 sources planned:

- Quaternius Ultimate Animated Character Pack (CC0) — avatar base.
- Quaternius Room Kit / sci-fi props (CC0) — Bridge environment.
- Poly Haven HDRIs (CC0) — IBL environment maps.

When a non-CC0 source is added (e.g. CC-BY), append a row with author,
license, and source URL — never silently include a third-party asset.