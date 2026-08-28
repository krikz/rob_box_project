# Avatar — CC0 attribution

This directory ships the avatar GLB for the Meta Quest WebXR Captain Bridge
(`src/rob_box_quest/webxr_client`, ADR-0032 §3.2 — avatar ≤ 500 KB).

## License

**CC0 1.0 Universal — Public Domain Dedication**
<https://creativecommons.org/publicdomain/zero/1.0/>

## Source

`avatar.glb` (raw) is produced programmatically by
[`scripts/build-avatar.mjs`](../../scripts/build-avatar.mjs). The script
synthesises a low-poly 4-wheeled robot from Three.js primitive geometries
(BoxGeometry + CylinderGeometry + SphereGeometry) and emits it via
`@gltf-transform/core` with PBR materials (baseColor + metallic + roughness)
and 5 procedural animations:

  - `idle`            (3.0 s) — chassis bob + sensor head yaw scan
  - `drive_forward`   (1.0 s) — all wheels at +π rad/s around the X axis
  - `drive_backward`  (1.0 s) — all wheels at −π rad/s
  - `turn_left`       (1.0 s) — front wheels ×1.5 speed, rear ×0.5
  - `turn_right`      (1.0 s) — mirror of `turn_left`

No external mesh data, textures, or models are referenced. The geometry is
entirely original code authored as part of rob_box_quest, released to the
public domain under CC0.

## Optimised artefact

This directory commits the avatar under two names:

  - `avatar.optimized.glb` — the direct output of the Phase 2.0 glTF asset
    pipeline ([`scripts/gltf-optimize.mjs`](../../scripts/gltf-optimize.mjs)).
    Carries:
      - `KHR_draco_mesh_compression`     (geometry, Draco edgebreaker)
      - `EXT_meshopt_compression`        (geometry + animation, Meshopt)
      - `KHR_mesh_quantization`          (16-bit vertex/index quantization)
  - `avatar.glb` — the canonical name that the WebXR runtime loads. Same
    bytes as `avatar.optimized.glb`; copied by the build script (step 4
    below) so the runtime can address the asset without the `.optimized.`
    suffix. The two files are byte-identical at commit time.

Note on `KHR_texture_basisu` (KTX2/Basis): the avatar has **zero textures**
(only PBR baseColor/metallic/roughness factors, no albedo/normal maps),
so KTX2 would be a no-op for this asset. `gltf:verify` emits a warning
for missing `KHR_texture_basisu` rather than failing the gate; the
opt-in KTX-Software `ktx` CLI is documented in README §KTX2 / Basis.

The raw `avatar.glb` lives in `_raw/` (gitignored) and is **not** committed
to the repository — only the optimised artifacts are.

## Rebuild

```
# 1. synthesise raw .glb in _raw/
node scripts/build-avatar.mjs

# 2. run Phase 2.0 pipeline (Draco + Meshopt) → _raw/avatar.optimized.glb
npm run gltf:optimize

# 3. move optimised → public/models/avatar/, delete raw
node scripts/build-avatar.mjs --publish

# 4. publish canonical avatar.glb (same bytes as avatar.optimized.glb)
cp public/models/avatar/avatar.optimized.glb public/models/avatar/avatar.glb

# 5. verify (size budget + extension contract)
npm run gltf:verify
```

## References

- Kanban: t_1fa6e505 (Phase 2.1 Avatar)
- Issue: GitHub #1677 (Phase 2+ Captain Bridge)
- ADR-0032: <https://github.com/krikz/rob_box_project/blob/develop/docs/adr/0032-meta-quest-webxr-stack-and-assets.md>
- glTF 2.0 spec: <https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html>