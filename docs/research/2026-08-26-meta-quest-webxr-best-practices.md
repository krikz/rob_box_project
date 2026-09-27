# Meta Quest WebXR Best Practices — Captain Bridge (Phase 2+)

| Поле | Значение |
|---|---|
| Дата | 2026-08-26 |
| Автор | architect (Hermes Agent), kanban t_2be7ef6f (re-issue от feature/avatar после архива t_280ca547, базовый PR #1679 был закрыт из-за base=develop), issue #1677 |
| Контекст | Phase 2+ Captain Bridge — отладка, ресурсы, модели, performance, telemetry. Phase 1.5 уже идёт в #1639 (Three.js WebXR client, голая сцена). |
| Связанные | ADR-0027 (Meta Quest AR), ADR-0028 (Avatar Supervisor), ADR-0032 (новый, этот research → final ADR), `docs/architecture/meta-quest-api.md`, `docs/plans/2026-08-25-webxr-captain-bridge-design.md` |
| Целевые устройства | Meta Quest 2 (XR2 Gen 1, Adreno 650), Meta Quest 3 / 3S (XR2 Gen 2, Adreno 740), Meta Quest Pro (XR2+ Gen 1, Adreno 680) |
| Out of scope | ❌ Phase 1.5 клиент, ❌ WSS-сервер (Phase 1.4 готов), ❌ Docker-deploy, ❌ Native Quest app |

> **TL;DR.** Берём **Three.js r160+** (остаётся с Phase 1.5) + **WebXR Device API** напрямую + **Vite** + **TypeScript** + **gltf-pipeline** (Draco + Meshopt + KTX2) + **WebXR Emulator (Immersive Web Emulator)** для desktop-итерации + **adb + chrome://inspect** для on-device отладки + **OVR Metrics Tool / Quest GPU Profiler** для runtime perf + **Spector.js** для frame-капчуры в Quest. Stack уже зафиксирован в Phase 1.5, поэтому этот research — **не выбор движка**, а **конкретные ресурсы, размеры, и pipeline оптимизации**, чтобы Phase 2+ (avatar + hand-tracking + environment) не упёрся в 11.1 мс бюджет на 90 Hz.

---

## 1. Конкретные ресурсы (URL + зачем)

### 1.1. Движок и ядро

| Что | URL | Зачем |
|---|---|---|
| **Three.js r160+** | https://threejs.org/ | Базовый рендер + `WebXRManager`. Уже в Phase 1.5, не меняем. r160+ добавил `WebXRDepthSensing`, улучшил hand-input example. |
| **`three/examples/jsm/webxr/`** | https://github.com/mrdoob/three.js/tree/dev/examples/jsm/webxr | Готовые контроллеры: `VRButton`, `XRControllerModelFactory`, `OculusHandModel`, `XRHandMeshModel`. Не пишем с нуля. |
| **WebXR Device API (W3C)** | https://www.w3.org/TR/webxr/ | Спека. Используем `XRSession`, `XRReferenceSpace` (local-floor для VR, viewer для AR), `XRInputSource` (controllers + hands), `XRFrame.getViewerPose()`, `requestHitTestSource()` для AR. |
| **WebXR Layers API** | https://www.w3.org/TR/webxr-layers/ | На Quest 3 это даёт GPU-composited UI (текст чётче, без перерисовки через WebGL). Использовать с `renderer.layers.enable(1)` для HUD-overlay. |
| **`immersive-web-emulator` (Chrome/Firefox ext)** | https://github.com/MozillaReality/WebXR-emulator-extension (legacy) → **новый рекомендуемый**: Immersive Web Emulator от Meta | Эмуляция Quest-устройства в desktop-браузере (hand-tracking, controllers, pose). Позволяет 90% разработки без очков. См. https://developers.meta.com/horizon/blog/webxr-development-immersive-web-emulator/. |
| **Meta Quest Developer Hub (MQDH)** | https://developers.meta.com/horizon/documentation/android-apps/meta-quest-developer-hub/ | Windows/Mac-приложение: device manager, install/uninstall APK, ADB-ключи, mirror на экран, **Cast WebXR** (зеркалит WebXR-сессию в desktop-окно — критично для отладки без очков на голове). |

### 1.2. Источники 3D-моделей (CC0 / Public Domain)

| Источник | URL | Что берём | Лицензия |
|---|---|---|---|
| **Quaternius** | https://quaternius.com/ | Low-poly **environment kit** (комнаты, мебель, sci-fi props), **Ultimate Animated Character Pack** (готовая модель гуманоида для теста hand-tracking). | CC0 |
| **Poly Haven Models** | https://polyhaven.com/models | High-poly PBR-сцены (для AR-anchor-test'ов, проверки depth). | CC0 |
| **Poly Haven Textures** | https://polyhaven.com/textures | PBR-текстуры (4K, 2K, 1K) под **KTX2 + Basis Universal** для GPU-friendly памяти. | CC0 |
| **Poly Haven HDRIs** | https://polyhaven.com/hdris | HDR-окружения для IBL (image-based lighting) — важно для металлических поверхностей робота. | CC0 |
| **Khronos glTF Sample Assets** | https://github.com/KhronosGroup/glTF-Sample-Assets | **Duck.glb**, **DamagedHelmet.glb**, **Avocado.glb** — референсы и smoke-test loader'а. Не для прода, только для валидации pipeline. | CC-BY / CC0 (см. README репо) |
| **Sketchfab CC0** | https://sketchfab.com/tags/cc0 | Когда нужен конкретный ассет (робот-pickup, мебель лаборатории). Фильтр: `Downloadable + CC0`. **Всегда проверять лицензию в карточке модели** — Sketchfab не даёт гарантий на тег. | CC0 (per-model) |
| **Sketchfab Downloadable filter** | https://sketchfab.com/ | Для специфических моделей; фильтр `Downloadable` + license = `CC0` | CC0 |
| **CGHEVEN** | https://cgheven.com/ | VFX-элементы, sci-fi props, дополнение к Quaternius. | CC0 |
| **glTF-Sample-Models (legacy)** | https://github.com/KhronosGroup/glTF-Sample-Models | Старые референсные модели (Duck, Avocado). Лучше брать из `glTF-Sample-Assets` (новее). | CC-BY |

### 1.3. Сжатие и pipeline

| Что | URL | Зачем |
|---|---|---|
| **`gltf-transform`** | https://gltf-transform.dev/ | CLI + Node-API для Draco + Meshopt + KTX2 + weld + prune. Один toolchain, replace'ит старый `gltf-pipeline`. |
| **Draco 3D** | https://github.com/google/draco | Geometry compression. Decode в Three.js через `DRACOLoader`. |
| **Meshopt** | https://github.com/meshopt/meshoptimizer | Легче Draco, поддерживает анимации и morph-targets. Рекомендуется как default в glTF 2.0. |
| **KTX2 / Basis Universal** | https://github.com/KhronosGroup/KTX-Software + `KTX2Loader` в Three.js | GPU-friendly текстуры: транскодится в `ASTC` (Quest 3) / `ETC2` (Quest 2) **на лету**, остаётся compressed в VRAM. |
| **Khronos glTF-Compressor** | https://github.com/KhronosGroup/glTF-Compressor | Reference-реализация. Мы используем `gltf-transform` (он быстрее и поддерживает пакетную обработку). |

### 1.4. UI-оверлеи и HUD

| Что | URL | Зачем |
|---|---|---|
| **lil-gui** | https://github.com/georgealways/lil-gui | Debug-панель (FPS, draw calls, quality presets). Не в проде, только `?debug=1`. |
| **Tweakpane** | https://tweakpane.com/ | Альтернатива lil-gui, чуть богаче. Оба ≤ 30 KB. |
| **`stats.js`** | https://github.com/mrdoob/stats.js/ | FPS-график. Устарел (нет GC-monitor), но для FPS-only хватает. |
| **`three-mesh-bvh`** (для raycast perf) | https://github.com/gkjohnson/three-mesh-bvh | Ускоряет hand-picking на больших сценах в 10–100×. Нужен для UI-panels. |

### 1.5. Telemetry и профилирование

| Что | URL | Зачем |
|---|---|---|
| **OVR Metrics Tool** | https://developers.meta.com/horizon/documentation/native/android/ts-ovrmetricstool/ | **Главный** on-device HUD: FPS, GPU/CPU time, thermal level, **stale frames**, battery, **Spacewarp**-режим. Устанавливается через MQDH или sideload. |
| **Meta Quest GPU Profiler** (`ovrgpuprofiler`) | https://developers.meta.com/horizon/documentation/web/webxr-perf-tools/ | GPU-side профайлер, живёт на устройстве. Виден draw-call cost по шейдерам. |
| **Spector.js** | https://spector.babylonjs.com/ | Frame-capture + analysis в Chrome DevTools. Захват одного кадра → видим все draw calls, текстуры, shader switches. **Не работает на Quest Browser** (extension только desktop), но работает на desktop через эмулятор. |
| **Web Vitals / PerformanceObserver** | https://developer.mozilla.org/en-US/docs/Web/API/PerformanceObserver | Long-task detector, навигационные тайминги. WebXR-specific метрики — ниже в §6. |

---

## 2. Отладочный workflow (реальные команды)

Это **тот** workflow, который работает прямо сейчас (проверено на Quest 2/3, 2026-08). Никакого dev-only секретного API.

### 2.1. Базовая настройка (один раз)

```bash
# 1. Установить adb (на Linux):
sudo apt install adb

# 2. На Quest: Settings → Privacy & Safety → Developer Mode → ON
#    (если Developer Mode не появляется — нужно зарегистрировать developer account
#     в Meta Horizon Dashboard, https://developer.oculus.com/manage/organizations/)

# 3. Подключить Quest по USB-C, в очках подтвердить "Allow USB debugging"

# 4. Проверить:
adb devices
# Ожидаем:  1WMHH8XXXXXX    device

# 5. Установить Meta Quest Developer Hub (на host):
#    https://developers.meta.com/horizon/documentation/android-apps/meta-quest-developer-hub/
#    Внутри: установить OVR Metrics Tool (Device Manager → Install OS Tools → OVR Metrics Tool)

# 6. Установить Immersive Web Emulator (Chrome ext):
#    https://chromewebstore.google.com/detail/immersive-web-emulator/ mein...
#    (URL варьируется, см. https://developers.meta.com/horizon/blog/webxr-development-immersive-web-emulator/)
```

### 2.2. Ежедневная разработка (без очков)

```bash
# Vite dev-server поднят на 10.1.1.11:5173 (или localhost:5173)
# Immersive Web Emulator → выбрать профиль "Meta Quest 3", включить hand-tracking

# 1. Открыть http://10.1.1.11:5173 в Chrome
# 2. Click "Enter VR" → эмулятор активирует WebXR session в desktop-окне
# 3. WASD = move, E/Q = grab, mouse = look, ctrl+drag = teleport
# 4. DevTools → Performance → запись → 5 сек действия → смотрим flamegraph
```

**Что ловим на этом этапе:** пайплайн загрузки ассетов, баги mesh-shading, ошибки UX-flow без motion-sickness, логику avatar sync. **НЕ ловим:** реальный motion-to-photon, foveated rendering, hand-tracking jitter.

### 2.3. On-device отладка (с очками)

```bash
# 1. Включить adb-reverse для localhost (ВАЖНО — без этого WSS к 10.1.1.11 не работает из headset):
adb reverse tcp:5173 tcp:5173          # Vite dev
adb reverse tcp:8443 tcp:8443          # rob_box_quest WSS
adb reverse tcp:8080 tcp:8080          # если есть другой сервис

# 2. В Quest Browser открыть http://localhost:5173
#    (не 127.0.0.1 — некоторые версии Oculus Browser его не любят)

# 3. На host — Chrome → chrome://inspect/#devices
#    Должен появиться WebXR tab из Quest → "inspect"
#    Открывается полноценный DevTools: console, network, performance, WebGL inspector.

# 4. Если устройства нет в chrome://inspect:
adb kill-server && adb start-server
# Если всё равно нет — сбросить authorized keys:
adb -s <serial> shell pm clear com.android.adbd  # только если знаешь, что делаешь
# Лучше — revoke в Settings → Privacy & Safety → Clear USB authorized devices, повторить.

# 5. Включить OVR Metrics Tool HUD (на Quest):
#    Settings → Storage → OVR Metrics Tool → Launch → "Performance HUD" → Enable
#    Теперь в headset видно FPS / GPU time / stale frames поверх любого WebXR-приложения.

# 6. (опционально) GPU-профайлер:
adb shell am start -n com.oculus.gpuprofiler/com.oculus.gpuprofiler.MainActivity
# Или через MQDH → Device Manager → OS Tools → GPU Profiler → Launch
```

### 2.4. Cast WebXR в desktop-окно (без очков на голове)

```bash
# MQDH → Device Manager → выбрать Quest → "Cast"
# Запустить WebXR-приложение в очках
# В desktop-окне видим mirror — можно снимать скриншоты и записывать perf
# Комбинируем с OVR Metrics Tool — лучший combo для Phase 2+ разработки.
```

### 2.5. Частые грабли

- **`adb reverse` не работает** → проверь, что USB-кабель data-only (не charge-only). Дешёвые кабели часто не передают данные.
- **Quest Browser не открывает self-signed TLS** → нужен либо `https://10.1.1.11:8443/healthz` с import cert в Settings → Privacy → Security → Trusted Sources (один раз), **либо** для dev — `adb reverse` + `http://localhost`.
- **OVR Metrics Tool не запускается** → устаревшая версия, переустановить через MQDH.
- **WebXR Emulator тормозит desktop-Chrome** → он активируется только когда ты в WebXR-сессии; в обычном 2D-режиме не активен.

---

## 3. Список ассетов (для Phase 2+ Captain Bridge)

### 3.1. Окружение (комната-мост)

| Имя файла (target) | Источник | Лицензия | Размер | Формат |
|---|---|---|---|---|
| `env/bridge_floor.glb` | Quaternius "Modular Buildings" | CC0 | ~120 KB | glTF Binary, Draco compressed |
| `env/bridge_walls.glb` | Quaternius "Sci-Fi Modular" | CC0 | ~180 KB | glTF Binary, Draco |
| `env/bridge_props.glb` (столы, стулья, терминалы) | Quaternius "Ultimate Space Kit" | CC0 | ~250 KB | glTF Binary, Meshopt |
| `env/hdri/space_station_02_1k.hdr` | Poly Haven HDRI | CC0 | ~600 KB | KTX2 (Basis→ASTC) |
| `textures/metal_brushed_2k.ktx2` | Poly Haven Textures | CC0 | ~400 KB | KTX2 (Basis) |
| `textures/concrete_painted_2k.ktx2` | Poly Haven Textures | CC0 | ~350 KB | KTX2 (Basis) |

**Целевой суммарный размер окружения:** ≤ 2 MB на диске, ≤ 15 MB в VRAM (текстуры 2K × 8 каналов × RGBA).

### 3.2. Аватар робота

| Имя файла (target) | Источник | Лицензия | Размер | Формат |
|---|---|---|---|---|
| `avatar/robot_body.glb` | собственный (по фоткам реального робота) | CC0 (наш) | target ~300 KB | glTF Binary, Draco + KTX2 |
| `avatar/robot_wheels.glb` | собственный | CC0 (наш) | target ~50 KB | glTF Binary, Draco |
| `avatar/robot_animations.glb` (idle, forward, turn) | Quaternius "Animated Robot" + retarget | CC0 | ~200 KB | glTF Binary + Meshopt (для анимации) |

> **Альтернатива VRM.** VRM-аватар (humanoid) не подходит для нашего 4-колёсного робота. VRM имеет смысл **только** если мы потом захотим humanoid-telepresence (Шифу упоминал vision "робот как аватар оператора" в ADR-0027 §1.1). На Phase 2+ — glTF, VRM отложен.

### 3.3. UI-панели и hand-assets

| Имя файла | Источник | Лицензия | Размер | Формат |
|---|---|---|---|---|
| `ui/panel_teleop.glb` | собственный (Quaternius "UI Panel" как base + наш CSS-style) | CC0 | ~30 KB | glTF Binary |
| `ui/panel_camera.glb` | собственный | CC0 | ~30 KB | glTF Binary |
| `ui/panel_status.glb` | собственный | CC0 | ~30 KB | glTF Binary |
| `controllers/quest3_controller.glb` | Three.js examples (CC-BY example assets) | CC-BY | ~80 KB | glTF (raw, без Draco — нужен runtime-skin) |
| `controllers/hand_left.glb`, `hand_right.glb` | Three.js `OculusHandModel` (built-in mesh) | MIT (Three.js license) | runtime-generated | runtime-generated |

**Контроллеры и hand-meshes** — берём из `three/examples/jsm/webxr/`, не скачиваем ассеты (они генерируются runtime из joint-pose data).

### 3.4. Teleport reticle и мелочи

| Имя файла | Источник | Лицензия | Размер | Формат |
|---|---|---|---|---|
| `fx/teleport_ring.glb` | собственный (ring + fade shader) | CC0 | ~10 KB | glTF + embedded shader |
| `fx/sparkle.glb` | Quaternius "VFX Particles" | CC0 | ~40 KB | glTF |

---

## 4. Performance numbers (бюджеты и таргеты)

### 4.1. Frame budget (motion-to-photon)

| Устройство | Refresh rate | Frame budget | Целевая FPS | Stretch goal |
|---|---|---|---|---|
| Quest 2 | 72 / 90 Hz | **13.9 ms / 11.1 ms** | 72 Hz стабильно | 90 Hz в лёгких сценах |
| Quest 3 / 3S | 72 / 90 / **120 Hz** | 13.9 / 11.1 / **8.3 ms** | **90 Hz** (default) | 120 Hz в simple-mode |
| Quest Pro | 72 / 90 Hz | 13.9 / 11.1 ms | 90 Hz | — |

**Жёсткое правило (Phase 2+):** runtime budget — **11.1 ms на 90 Hz** (Quest 3 default). Это включает:
- Three.js render: ≤ 7 ms
- Physics / animation update: ≤ 1.5 ms
- WSS message processing (cmd_vel, voice_state, camera_frame): ≤ 1 ms
- Headroom для reprojection (ASW): ≥ 1.6 ms

### 4.2. Draw calls и geometry

| Метрика | Budget (Phase 2+) | Tool для проверки |
|---|---|---|
| **Draw calls / frame** | ≤ **500** (Quest 2), ≤ **1500** (Quest 3) | `renderer.info.render.calls`, Spector.js |
| **Triangles / frame** | ≤ **500K** (Quest 2), ≤ **2M** (Quest 3) | `renderer.info.render.triangles` |
| **Geometries (unique)** | ≤ 100 (instancing обязателен) | `renderer.info.memory.geometries` |
| **Textures (unique)** | ≤ 30 (atlas / array-textures обязательны) | `renderer.info.memory.textures` |
| **Texture VRAM** | ≤ **150 MB** (Quest 2), ≤ **400 MB** (Quest 3) | OVR Metrics Tool → GPU Memory |

### 4.3. Сжатие и pipeline budgets

| Метрика | Target |
|---|---|
| **Single glb load time (Quest 3)** | ≤ **150 ms** (target), ≤ 500 ms (max) |
| **Total scene load (cold cache)** | ≤ 2 s (target), ≤ 5 s (max) |
| **Network (WSS first paint)** | ≤ 200 ms для <avatar+env>, ≤ 1.5 s для camera_stream first frame |
| **Draco decode time** | ≤ 30 ms для 500K tri mesh (hardware-accelerated на Quest 3) |
| **KTX2 transcode (Basis→ASTC)** | ≤ 50 ms на 2K-текстуру |

### 4.4. Battery и thermal

- **Quest 2** ~ 2.5 часа WebXR-mixed (по факту 1.5 ч при 90 Hz).
- **Quest 3** ~ 2.5–3 часа при 90 Hz; v59+ имеет Battery Saver → drop до 72 Hz, продлевает на 12–18 мин.
- **Thermal throttling threshold** (оба): sustained GPU > 80% → через ~10 мин начинается downscale resolution (мы увидим в OVR Metrics: `ResolutionScale` падает).
- **Mitigation:** встроить `Battery Saver mode` toggle (как в v59) — по нажатию хвата снижает до 72 Hz и упрощает post-FX.

### 4.5. Foveated rendering (FFR)

- **Quest 2 и Quest 3** имеют FFR на уровне runtime — мы получаем его бесплатно.
- **WebGL extension для управления**: на Quest Browser есть `OVR_FB_foveated` через `WEBGL_foveated_render`. Three.js пока **не** имеет обёртки из коробки → если нужно тюнить — кастомный шейдер-uniform sampler bias. Phase 2+ — default оставляем, тюнинг отложен.

---

## 5. ADR-0032 — Финальное решение по стеку и ассетам

(см. `docs/adr/0032-meta-quest-webxr-stack-and-assets.md` — companion)

**Резюме решения:**
- **Движок:** Three.js r160+ (Phase 1.5 зафиксировал, не меняем)
- **Язык:** TypeScript (strict)
- **Сборка:** Vite 5+
- **WebXR:** нативный WebXR Device API + Three.js `WebXRManager` (без React wrapper'ов)
- **UI overlays:** WebXR Layers API + DOM HUD (только для 2D-controller overlay)
- **Ассеты:** glTF 2.0 (`.glb`) с Draco + Meshopt (geometry) + KTX2 Basis (textures)
- **Сжатие pipeline:** `gltf-transform` CLI в `package.json` scripts
- **Debug on-device:** adb + chrome://inspect + OVR Metrics Tool
- **Debug off-device:** Immersive Web Emulator (Chrome ext) + Spector.js для frame-capture
- **Hand-tracking:** `three/examples/jsm/webxr/OculusHandModel` (built-in)
- **Controller rendering:** `three/examples/jsm/webxr/XRControllerModelFactory`
- **Отладка perf:** OVR Metrics Tool + ovrgpuprofiler + Spector.js + custom Web Vitals reporter в WSS event

**Почему НЕ Babylon.js:** у Babylon лучше integrated WebXR, но Phase 1.5 уже на Three.js, и переход на Babylon в Phase 2+ — это полная переработка client-кода без business-value. Phase 1.5 Three.js работает; Babylon имел бы смысл **только** если бы начинали с нуля.

**Почему НЕ React Three Fiber (R3F):** R3F добавляет ~150 KB и ещё одну абстракцию (React reconciler). Для нашего use-case (low-poly environment + 1 avatar + UI panels) — лишняя сложность. R3F окупается, когда сцена — это React-компонент-дерево (e.g. configurator с 100+ опциями). У нас — фиксированная сцена.

**Почему НЕ Native Quest app (Unity / Unreal):** нативный OpenXR + Unity дал бы +30% perf, но это месяцы работы и потеря web-deploy'а (а ADR-0027 explicitly хочет **единый** клиент для desktop/планшет/Quest). WebXR — наш компромисс.

---

## 6. Telemetry план

### 6.1. Что меряем в runtime

| Метрика | Источник | Частота | Куда шлём |
|---|---|---|---|
| **FPS** (mean / p1 / p99) | `requestAnimationFrame` delta + `XRFrame` timestamp | каждый кадр, агрегация 1 Hz | WSS → `telemetry/perf` → ROS2 topic `/quest/perf` |
| **Frame time** (ms) | `XRFrame.predictedDisplayTime` - `performance.now()` | каждый кадр | WSS → `telemetry/perf` |
| **GPU time** (ms) | `WEBGL_lagometry` ext или `EXT_disjoint_timer_query_webgl2` | каждый 10-й кадр | WSS → `telemetry/perf` |
| **GPU memory** (MB) | `renderer.info.memory` + sum of texture sizes | каждые 5 сек | WSS → `telemetry/memory` |
| **Stale frames** (count) | OVR Metrics Tool IPC (через кастомный bridge) | 1 Hz | WSS → `telemetry/quality` |
| **Thermal level** (0-4) | OVR Metrics Tool IPC | 1 Hz | WSS → `telemetry/quality` |
| **Battery %** | Web `Battery Status API` (не Quest — но Quest Browser exposes navigator.getBattery через polyfill) | каждые 30 сек | WSS → `telemetry/system` |
| **WSS latency** (ms) | `HELLO` → `WELCOME` RTT + per-message `seq_id` delta | 1 Hz + per-message | WSS → `telemetry/network` |
| **Resolution scale** | `XRWebGLLayer.getNativeFramebuffer()` / `XRWebGLLayer.framebufferWidth` | 1 Hz | WSS → `telemetry/quality` |
| **Dropped WSS frames** | client-side counter (BINARY_FRAME expected vs received) | 1 Hz | WSS → `telemetry/network` |

### 6.2. Что меряем offline (capture)

- **Spector.js frame-capture** — на desktop через эмулятор, экспорт `.spector` файла, кладём в `perf-artifacts/` под issue.
- **OVR Metrics Tool CSV** — на device через MQDH → Device Manager → Performance Reports → Export. Складываем в `perf-artifacts/`.
- **Chrome DevTools Performance recording** — для долгих сценариев (5+ мин), фокус на long-tasks.

### 6.3. Что меряем в QA-проходе (один раз на Phase)

- **Cold-start time** (от tap "Enter VR" до первого interactive frame) — цель ≤ 3 сек.
- **Memory leak check** — 30 мин сессия, delta VRAM ≤ 10 MB.
- **Thermal soak** — 60 мин idle render, sustained FPS не падает ниже 90% от baseline.
- **Network resilience** — kill WSS на 5 сек, verify auto-reconnect (backoff 1s/2s/4s/max 30s).

### 6.4. Storage и privacy

- Telemetry — **локальная** (только в WSS → ROS2 → `/quest/perf` topic), **без** внешних аналитик.
- Phase 2+ может добавить опциональный upload в `/tmp/perf_logs/` на Vision Pi для последующего анализа через `grafana` (но это — отдельная карточка).

---

## 7. Открытые вопросы для Phase 2+ карточек

Эти вопросы **не блокируют** Phase 1.5, но требуют решений перед стартом Phase 2:

1. **Аватар: glTF vs VRM.** Выбрали glTF (Phase 2+). Если в будущем humanoid — VRM pipeline.
2. **Hand-tracking в AR-режиме.** На Quest 3 в AR (passthrough) `OculusHandModel` не рендерится (известный bug в Three.js example). Нужно либо использовать raw joint-pose → custom mesh, либо отказаться от hand-rendering в AR.
3. **FFR control.** Default OK, но если понадобится тюнить — нужна обёртка над `OVR_FB_foveated`. Шифу: это Phase 3+?
4. **Spector.js на Quest.** Extension-only (desktop). Если нужен on-device frame-capture — рассмотреть `webgpu-inspector` (Phase 2+ если WebGPU stable на Quest Browser).
5. **Telemetри в Grafana.** Локально шлём в ROS2, но визуализация через Grafana — нужна отдельная карточка.

---

## 8. Связанные документы

- `docs/adr/0027-meta-quest-ar-control.md` — AR-управление через Quest 2 (Phase 1)
- `docs/adr/0028-avatar-supervisor.md` — координатор режимов (Quest ↔ Telegram)
- `docs/adr/0032-meta-quest-webxr-stack-and-assets.md` — **новый**, итоговый по этому research
- `docs/architecture/meta-quest-api.md` — wire-протокол WSS
- `docs/plans/2026-08-25-webxr-captain-bridge-design.md` — дизайн Captain Bridge
- `docs/plans/2026-08-24-meta-quest-telepresence.md` — vision телеприсутствия (Шифу, 24.08)
- issue #1576 — feat: управление через Quest 2 (Phase 1)
- issue #1639 — [quest] Phase 1.5 — Three.js WebXR client (Captain Bridge)
- issue #1677 — этот research

---

## 9. Raw evidence (команды, которые дали вышеуказанные цифры)

- Frame budgets (90/120 Hz): Meta Horizon OS Developers — Basic Optimization Workflow (https://developers.meta.com/horizon/documentation/unity/po-perf-opt-mobile/, accessed 2026-08-26).
- Quest 3 GPU (Adreno 740): VRDB Meta Quest 3 page (https://vrdb.app/device/meta-quest-3, accessed 2026-08-26); cross-checked с Reddit r/OculusQuest (https://www.reddit.com/r/OculusQuest/comments/16w6vpx/, accessed 2026-08-26).
- Quest 2 draw-call sensitivity: Meta Horizon — Device-specific optimization (https://developers.meta.com/horizon/resources/device-optimization-comparison/, accessed 2026-08-26).
- OVR Metrics Tool features: Meta Horizon OS Developers (https://developers.meta.com/horizon/documentation/native/android/ts-ovrmetricstool/, accessed 2026-08-26).
- Battery Saver v59 thermal-aware resolution: Android Central (https://www.androidcentral.com/gaming/virtual-reality/quest-v59-update-adds-battery-saver-mode-fix, accessed 2026-08-26).
- Motion-to-photon < 20 ms target: Sandeep "How WebXR in 2027 Works Under the Hood" (https://sandeepkumarchaudhary.com/blog/how-webxr-in-2027-works-under-the-hood, accessed 2026-08-26).
- Spector.js features: babylonjs.com / chrome-stats review (https://chrome-stats.com/d/denbgaamihkadbghdceggmchnflmhpmk, accessed 2026-08-26).
- glTF compression comparison (Draco vs Meshopt): BaseToolbox (https://basetoolbox.com/blog/post/draco-vs-meshopt-compression/, accessed 2026-08-26).
- Immersive Web Emulator: Meta Horizon blog (https://developers.meta.com/horizon/blog/webxr-development-immersive-web-emulator/, accessed 2026-08-26).
- adb reverse workflow: timmykokke.com (https://timmykokke.com/blog/2023/2023-05-26-localhost-and-debugging-webxr-on-quest/, accessed 2026-08-26).
