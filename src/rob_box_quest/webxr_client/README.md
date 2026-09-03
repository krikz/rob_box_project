# `@rob-box/quest-webxr-client`

Meta Quest / WebXR Captain Bridge client for `rob_box_quest`.
Phase 2+ stack per [ADR-0032](../../../docs/adr/0032-meta-quest-webxr-stack-and-assets.md):
Three.js r170 + native WebXR + glTF 2.0 asset pipeline (Draco + Meshopt + KTX2).

The README has two parts:

1. **[Asset pipeline](#asset-pipeline-cc0-only)** (Phase 2.0) — glTF optimization + CI guard.
2. **[Captain Bridge application](#captain-bridge-application-phase-21)** (Phase 2.1+) — render loop,
   teleop, voice, **UX overlays** (Phase 2.3).

For deep architecture see [`docs/architecture/captain-bridge.md`](../../../docs/architecture/captain-bridge.md).
WSS protocol contract: [`docs/architecture/meta-quest-api.md`](../../../docs/architecture/meta-quest-api.md).

---

## Captain Bridge application (Phase 2.1+)

### What's inside

- **Two-mode entry**: PIN form → if browser supports `immersive-vr`, auto-enter
  WebXR; otherwise stay in desktop fallback (WASD + 2D render).
- **Scene graph**: Captain Bridge environment (5 CC0 GLB + HDR, ~70 KB total),
  main wall-screen for the front camera, side video panels at ±75°
  (`camera_oak_depth`, `camera_ceiling`), LiDAR overlay, ARM + status HUDs.
- **Voice pipeline panel**: always-visible 3D panel on the right (+105°),
  showing `voice → STT → LLM → TTS → speaker` with per-stage toggles, six
  style presets + output language, and a TTS voice button opening the TTS
  picker. Only the PIN form is HTML — all panels live in the 3D scene.
- **Teleop**: XR controllers (oculus-touch-v2 mapping) — left stick movement,
  right stick click arm/disarm toggle, B/Y emergency stop. Desktop fallback
  WASD + Space + E. Arm-state visible in HUD on the front wall.
- **Voice PTT**: left grip = robot_voice (STT→LLM→TTS via supervisor),
  right grip = radio (passthrough to robot speaker). Microphone is shared
  between modes; PTT is edge-triggered.
- **Status HUD** (Wave 3.A / R8): battery (percent, or volts when only VESC
  voltage is available), Wi-Fi RSSI, speed, ping/pong RTT, robot mode.
- **WSS protocol**: `robbox-quest-v1` subprotocol. Implemented: streams
  (video / LiDAR / `robot_status`), teleop, `voice_mode`, voice PTT,
  `stream_list` / `stream_select`, ping/pong. **Not implemented** (types
  only in `wire/messages.ts`): TTS picker (`list_voices` / `set_voice` /
  `preview_voice`), `set_panel_topic`, `admin_logs` — see
  `docs/plans/2026-08-30-captain-bridge-feature-audit.md`.

### Phase 2.3 UX overlays (NEW)

Three DOM-overlay modules in `src/ui/`:

| Overlay | When | Trigger |
|---|---|---|
| `loading_screen` | пока грузятся CC0 GLB/HDR | автоматически на bootstrap, `loading.watch(promise)` |
| `error_overlay` | WS disconnect > 5s (с watchdog), или `closed` | `errorOverlay.show(headline, detail)` / `watchdog.markDisconnected()` |
| `help_overlay` | список горячих клавиш (Desktop / WebXR / Global) | H key или клик по `?`-кнопке в HUD |
| `mode_manager` | клиентский observable-стор UI-state | синхронизируется с arm-stikom и voice-режимом в `main.ts` |

Подробности API и поведения — в [captain-bridge.md §7](../../../docs/architecture/captain-bridge.md#7-phase-23-ux-overlays-new).

### Default layout

Экран-стена впереди + боковые panels на радиусе 2.0 м:

| Где | Topic | Назначение |
|---|---|---|
| стена (z=-3.9) | `camera_rear` | фронтальная OAK-D color (через ROS) |
| -75° | `camera_oak_depth` | OAK-D depth (depthai) |
| +75° | `camera_ceiling` | потолочная USB-камера |

`camera_oak_color` (0x1003) не дублируется на панель — это тот же сенсор,
что и на экране-стене.

Camera default: `pos=(0, 1.6, 0)`, `look forward` (в сторону main screen).

### Указатель (луч)

Один слой на все интерактивные объекты сцены (`src/interaction/`):

| Где | Луч | Выбор |
|---|---|---|
| Desktop | из камеры через курсор | ЛКМ |
| WebXR | `targetRaySpace` контроллера (правая рука приоритетнее) | trigger (кнопка 0) |

Trigger свободен намеренно: grip'ы заняты голосом (рация / робот-голос),
клик стика — arm/disarm, B/Y — emergency.

Поведение: наведение подсвечивает панель, короткое нажатие — выбор,
нажатие с уводом луча — перетаскивание. Панель катается **по сфере**
вокруг оператора: расстояние не меняется, `facing` всегда смотрит в
центр, высота ограничена 0.8–2.6 м. Потеря луча (курсор ушёл с канваса,
трекинг пропал) корректно завершает драг, а не оставляет панель
приклеенной.

Сюда же будут регистрироваться кнопки панели режимов супервизора и клик
по карте — `bridge.pointer.addTarget({ id, object, draggable })`.

### Hotkeys

| Key | Action | Mode |
|---|---|---|
| WASD | Movement | Desktop |
| Space | Boost (×1.5) | Desktop |
| E | Emergency stop | Desktop |
| L stick | Move (forward/back/strafe) | WebXR |
| R stick click | Arm / disarm toggle | WebXR |
| L grip | Voice: radio (рация) | WebXR |
| R grip | Voice: robot_voice (STT→LLM→TTS) | WebXR |
| B / Y | Emergency stop | WebXR |
| H | Показать / скрыть help overlay | Global |
| Esc | Закрыть overlay / exit VR | Global |

Press H in the client to see this list interactively.

### Tooltips

- **Status badge** (`#status`) — `title="WebSocket connection to robot"`.
- **Help-toggle button** (`#help-toggle`) — `title="Показать / скрыть горячие клавиши (H)"`.

### Tests

142 vitest unit tests across 14 files:

- `panel_manager.test.ts` — pure-data panel state.
- `video_panel.test.ts` *(если добавится в Phase 3)*.
- `bridge_environment.test.ts` — CC0 GLB round-trip.
- `teleop_fsm.test.ts` — arm/emergency FSM.
- `xr_teleop.test.ts`, `xr_bootstrap.test.ts` — XR controllers + session.
- `voice_capture.test.ts` — Mic PCM capture.
- `connection.test.ts`, `protocol.test.ts` — WSS handshake.
- `lidar_payload.test.ts` — LiDAR wire format.
- `gltf_pipeline.test.ts` — Duck.optimized.glb round-trip.
- **Phase 2.3 NEW**: `loading_screen.test.ts`, `error_overlay.test.ts`,
  `help_overlay.test.ts`, `mode_manager.test.ts`.

```bash
npm test                 # 142 tests, ~10 s
npm run typecheck        # tsc --noEmit
npm run gltf:verify      # CI guard for committed GLB
npm run build            # vite build (dist/, ~118 KB JS)
```

---

## Asset pipeline (CC0-only)

All 3D assets committed under `public/models/` MUST be CC0 (or CC-BY with a
row in `CREDITS.md`) AND MUST be **optimized**. Raw glTF source files
(`.glb`, `.gltf` produced by Blender / Quaternius / Khronos Sample Assets)
are forbidden in the tree — the CI guard `npm run gltf:verify` rejects any
`.glb` that lacks `KHR_draco_mesh_compression` + `EXT_meshopt_compression`.

Why: per ADR-0032 §3.2 we have hard size budgets (environment ≤ 2 MB,
panel ≤ 150 KB). Uncompressed glTF bloats past these
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
|source-of-truth mesh exported by an artist), add it to `.gitignore`
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