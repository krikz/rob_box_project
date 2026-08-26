# ADR-0032: Meta Quest WebXR client — финальный стек и ассет-pipeline

| Поле | Значение |
|---|---|
| Статус | **Accepted** (финальный по research #1677, заменяет draft в ADR-0027 §3 / §4 для Phase 2+ Captain Bridge) |
| Дата | 2026-08-26 |
| Автор | architect (Hermes Agent), kanban t_280ca547, issue #1677 |
| Контекст | Phase 2+ Captain Bridge требует avatar робота, hand-tracking, UI panels, environment — после Phase 1.5 (минимальный Three.js WebXR клиент из #1639). Нужно зафиксировать стек (Three.js vs Babylon / R3F), asset pipeline (glTF + Draco + Meshopt + KTX2), debug workflow, perf targets до начала implementation. |
| Затрагивает | (будущее) новый client-package `rob_box_quest_client` или доработка существующего в Phase 1.5; `docker/vision/` сервис `rob_box_quest` (уже частично в Phase 1.4–1.6); новый pipeline `gltf-transform` в `package.json`; WSS-схема telemetry (см. `docs/architecture/meta-quest-api.md`) |
| Родители | ADR-0018 (honesty), ADR-0027 (Meta Quest AR — Phase 1), ADR-0028 (Avatar Supervisor), ADR-0030 (ADR-numbering SOT) |
| Связанные | `docs/research/2026-08-26-meta-quest-webxr-best-practices.md` (детальный research); issue #1576, #1639, #1677; `docs/architecture/meta-quest-api.md`; `docs/plans/2026-08-25-webxr-captain-bridge-design.md` |
| Целевые устройства | Meta Quest 2 / 3 / 3S / Pro |
| Заменяет | Черновые предположения в ADR-0027 §3.2 (stack), §4.2 (assets) — для Phase 2+ |

> **TL;DR.** Phase 2+ Captain Bridge строится на **Three.js r160+ + TypeScript + Vite + нативный WebXR Device API + glTF 2.0 (Draco + Meshopt + KTX2)**. Stack уже зафиксирован Phase 1.5 (Three.js) — этот ADR **подтверждает** его и добавляет: (а) **asset-pipeline** через `gltf-transform`, (б) **debug workflow** (Immersive Web Emulator + adb + chrome://inspect + OVR Metrics Tool), (в) **performance budget** (11.1 ms / 90 Hz, ≤ 500 draw calls на Quest 2, ≤ 1500 на Quest 3), (г) **telemetry-план** (FPS, GPU time, thermal, WSS latency → WSS → ROS2). Никакого Babylon / R3F / Native. Аргументы — §3.

---

## 1. Контекст и проблема

### 1.1. Где мы сейчас (Phase 1.5 → Phase 2+)

**Phase 1 (ADR-0027, готов):** TLS-шлюз `rob_box_quest` на Vision Pi, WSS-контракт, PIN-auth, бинарные потоки (camera + LiDAR) — без 3D-сцены. Только flat HTML + JS-команды teleop.

**Phase 1.5 (issue #1639, в работе):** минимальный Three.js WebXR клиент. Пустая сцена, базовый hand/controller-render, подключение к WSS из Phase 1. Это **доказательство концепции** Three.js + WebXR на реальном Quest.

**Phase 2+ Captain Bridge (цель этого ADR):** полноценная 3D-сцена с:
- аватаром робота (glTF, наша модель 4-колёсной платформы),
- hand-tracking / controller-render для teleop,
- UI panels (teleop, camera streams, status),
- environment (комната-мост, room-scale VR),
- AR passthrough overlay (LiDAR-точки, depth),
- голосовым вводом через Quest microphone (ADR-0027 §3.4).

**Проблема:** без фиксации стека и asset-pipeline разработка Phase 2+ уйдёт в:
1. споры «а почему не Babylon / R3F»,
2. тяжёлые модели (raw glTF, без Draco → 50 MB сцена → 30+ сек загрузка на Quest),
3. неработающую on-device отладку (потому что нет adb-reverse / нет OVR Metrics Tool),
4. отсутствие perf-budget'а → разрабы не знают, что «90 FPS» — это 11.1 ms.

### 1.2. Требования

- **R1.** Работать на Quest 2 (Adreno 650), Quest 3/3S (Adreno 740), Quest Pro (Adreno 680).
- **R2.** Sustained 90 Hz на Quest 3, 72 Hz на Quest 2 — без thermal throttling в течение 30 мин.
- **R3.** Cold-start ≤ 3 сек до первого interactive frame.
- **R4.** Total VRAM ≤ 150 MB (Quest 2) / ≤ 400 MB (Quest 3).
- **R5.** Работать в обычном браузере (desktop/планшет) — тот же клиент (ADR-0027 §1.1).
- **R6.** Поддержка hand-tracking и controllers.
- **R7.** Battery-friendly: при 90 Hz ≥ 2 часа работы (или честная индикация «Battery Saver»).
- **R8.** Dev-friendly: 90% разработки без очков (Immersive Web Emulator), on-device отладка через Chrome DevTools.
- **R9.** Все ассеты — **CC0** (или собственные).
- **R10.** Asset pipeline — reproducible (одна команда CI перегенерит ассеты).

---

## 2. Рассмотренные альтернативы

### 2.1. Движок: Three.js (выбран) vs Babylon.js vs R3F vs Native

| Критерий | Three.js r160+ | Babylon.js 7+ | React Three Fiber (R3F) | Native (Unity+OpenXR) |
|---|---|---|---|---|
| WebXR support | Хороший (`WebXRManager`, hands, controllers) | **Отличный** (встроенный XR MVM, hand physics) | Хороший (через Three.js) | N/A (не web) |
| Размер bundle | ~180 KB core + tree-shake | ~400 KB core | ~330 KB (Three + R3F) | N/A |
| Phase 1.5 уже использует | ✅ Да | ❌ Нет | ❌ Нет | ❌ Нет |
| Экосистема (Three examples, glTF-loader, KTX2) | ✅ Самая большая | ✅ Хорошая | ✅ Через Three.js | — |
| Профилирование (Spector.js) | ✅ Работает | ✅ Работает | ✅ Работает | ⚠️ Только native tools |
| Honesty-стоимость перехода | — | Высокая (переписать Phase 1.5) | Средняя (обернуть в React) | **Очень высокая** (месяцы) |
| Web-deploy (тот же код в desktop browser) | ✅ | ✅ | ✅ | ❌ |

**Решение:** **Three.js r160+**. Phase 1.5 уже на нём, переход на Babylon в Phase 2+ — переписывание без business-value. R3F даёт +150 KB и React-overhead для сцены из 1 аватара + 5 UI-panels — не окупается. Native (Unity/OpenXR) дал бы +30% perf, но ADR-0027 §1.1 explicitly требует **единый** web-клиент для desktop/планшет/Quest. Компромисс — WebXR.

### 2.2. Asset format: glTF + Draco + Meshopt + KTX2 (выбран) vs raw glTF vs FBX/OBJ vs VRM

| Формат | Поддержка Three.js | Размер (на типичной сцене) | Pipeline |
|---|---|---|---|
| **glTF 2.0 binary (.glb) + Draco + Meshopt + KTX2** | ✅ Нативный (`GLTFLoader` + `DRACOLoader` + `KTX2Loader`) | 100 KB – 1 MB | `gltf-transform` CLI |
| glTF raw (no compression) | ✅ | 5–50 MB | Нет (нужны external tools) |
| FBX / OBJ | ⚠️ Через конвертеры | 10–100 MB | Blender export → glTF |
| VRM | ✅ (плагин `@pixiv/three-vrm`) | 1–10 MB | VRM-1.0 spec |

**Решение:** **glTF 2.0 + Draco + Meshopt (geometry) + KTX2/Basis (textures)**. Draco для статической геометрии, Meshopt — для анимированных (meshopt сохраняет animation tracks). KTX2/Basis транскодится в `ASTC` (Quest 3 GPU-native) или `ETC2` (Quest 2) → остаётся compressed в VRAM. Pipeline — `gltf-transform` в `package.json` scripts (`pnpm run assets:optimize`). VRM — отложен до Phase humanoid (если Шифу захочет).

### 2.3. Hand-tracking: built-in OculusHandModel (выбран) vs custom joint→mesh vs сторонние библиотеки

| Подход | Pros | Cons |
|---|---|---|
| **`three/examples/jsm/webxr/OculusHandModel`** (built-in) | Готовые mesh'ы, MIT (Three.js), работает out-of-box | Не рендерится в AR-mode на Quest 3 (известный bug) |
| **Custom joint→mesh** | Полный контроль, можно в AR | Нужно самим делать skinning, IK |
| **`@pmndrs/hand-tracking` (zustand-based)** | Декларативный | Overkill, для R3F |

**Решение:** **Built-in `OculusHandModel` для VR-mode**, **отключить hand-rendering в AR-mode** (только controllers, или custom joint-trail как fallback). Это явно фиксируем в WSS frame subscription: Phase 2+ `client_capabilities` поле → сервер знает, что AR-клиент хочет raw joint-pose.

### 2.4. Debug workflow: Immersive Web Emulator + adb + chrome://inspect (выбран) vs MQDH-only vs SideQuest

**Решение:** **Immersive Web Emulator** (Chrome/Firefox ext, Meta-рекомендуется с 2024) для desktop-разработки без очков. **`adb + chrome://inspect`** для on-device (стандартный Android debug). **MQDH** для device manager и OVR Metrics Tool install. SideQuest НЕ используем (только для sideload APK, что нам не нужно — у нас WebXR, не APK).

### 2.5. Telemetry: OVR Metrics Tool + custom WSS reporter (выбран) vs полностью на стороне клиента

**Решение:** **Гибрид.** OVR Metrics Tool — для on-device debug-сессий (sideload через MQDH, показывает HUD поверх). **WSS reporter** — для runtime telemetry в production: FPS, GPU time (через `EXT_disjoint_timer_query_webgl2`), WSS latency, thermal level (через OVR Metrics IPC bridge), resolution scale → шлём в WSS event `telemetry/perf` → ROS2 topic `/quest/perf` → (опционально) Grafana.

---

## 3. Решение (детально)

### 3.1. Stack

```
Three.js r160+
  + TypeScript (strict)
  + Vite 5+
  + WebXR Device API (нативный, без wrapper'ов)
  + @types/webxr
  + three/examples/jsm/webxr/{VRButton, XRControllerModelFactory, OculusHandModel, XRHandMeshModel}

Asset pipeline (CI-side, не runtime):
  gltf-transform CLI
    ├─ Draco compress geometry
    ├─ Meshopt compress (animations)
    └─ KTX2/Basis compress textures

Debug tools (dev-only, не в проде):
  + Immersive Web Emulator (Chrome ext)
  + Spector.js (через npm dev-dep, не в проде)
  + lil-gui (через ?debug=1 query)
  + adb + chrome://inspect (on-device)
  + OVR Metrics Tool (sideload через MQDH)
```

### 3.2. Asset budget (Phase 2+)

| Категория | Файлов | Размер на диске | VRAM (после transcode) |
|---|---|---|---|
| Environment (bridge + props) | 4–6 glb | ≤ 2 MB | ≤ 15 MB |
| Avatar робот | 2–3 glb | ≤ 500 KB | ≤ 5 MB |
| UI panels | 3–4 glb | ≤ 150 KB | ≤ 2 MB |
| Textures (KTX2) | 10–15 | ≤ 5 MB | ≤ 30 MB |
| HDR env (KTX2) | 1 | ≤ 600 KB | ≤ 4 MB |
| **Total** | ~25 assets | **≤ 8 MB** | **≤ 60 MB** (с запасом до 150/400 MB бюджета) |

### 3.3. Performance budget

| Метрика | Quest 2 budget | Quest 3 budget |
|---|---|---|
| Sustained FPS | 72 Hz | 90 Hz |
| Frame time | ≤ 13.9 ms | ≤ 11.1 ms |
| Draw calls / frame | ≤ 500 | ≤ 1500 |
| Triangles / frame | ≤ 500K | ≤ 2M |
| VRAM | ≤ 150 MB | ≤ 400 MB |
| Cold-start time | ≤ 4 s | ≤ 3 s |

См. детали — `docs/research/2026-08-26-meta-quest-webxr-best-practices.md` §4.

### 3.4. Debug workflow (TL;DR; полный — §2 research)

1. Desktop-разработка: Immersive Web Emulator + Chrome DevTools.
2. On-device: `adb reverse tcp:5173 tcp:5173`, открыть `http://localhost:5173` в Quest Browser, `chrome://inspect` на host.
3. Performance: OVR Metrics Tool (HUD) + ovrgpuprofiler + Spector.js (только desktop).
4. Network resilience: kill WSS на 5 сек, проверить auto-reconnect.

### 3.5. Telemetry

| Что | Как | Куда |
|---|---|---|
| FPS / frame time | `XRFrame` + `performance.now()` | WSS → ROS2 `/quest/perf` |
| GPU time | `EXT_disjoint_timer_query_webgl2` (каждые 10 кадров) | WSS |
| Stale frames / thermal / battery | OVR Metrics Tool IPC bridge | WSS |
| WSS latency | client-side `HELLO` RTT + per-message seq_id | WSS / self-log |
| Resolution scale | `XRWebGLLayer.getNativeFramebuffer()` | WSS |

См. детали — research §6.

---

## 4. Trade-offs (явные)

### 4.1. Что мы теряем, выбирая Three.js вместо Babylon

- **Hand physics в Babylon** (XR MVM hand physics) — не используем, hand-tracking только для input.
- **Built-in XR-менеджер Babylon** — у Three.js `WebXRManager` требует ~30 строк boilerplate. Принимаем, это разовая плата.
- **Babylon GUI 3D** — у Three.js эквивалент — отдельная HTML-overlay или custom shader-quad. Принимаем.

### 4.2. Что мы теряем, выбирая WebXR вместо Native

- **+30% perf** — даёт Unity+OpenXR (ниже GC-паузы, прямой доступ к Vulkan). Не критично для нашего use-case (4-колёсная платформа + 3D-сцена, не AAA-игра).
- **Native hand physics** — мы используем XR Hands API (joints), физику делаем сами. Принимаем.
- **Passthrough качество** — WebXR passthrough на Quest 3 хуже native (нельзя depth-test без `XRDepthSensing`). Phase 3+ если понадобится.

### 4.3. Что мы теряем, выбирая glTF вместо VRM

- **Humanoid avatar pipeline** — VRM имеет встроенные blend shapes, spring bones, eye tracking. На Phase 2+ нам не нужно (аватар — 4-колёсный робот). Если Шифу захочет humanoid (vision из ADR-0027 §1.1) — отдельный ADR добавит VRM pipeline.

### 4.4. Что мы НЕ делаем (и почему)

- ❌ **Native Quest app (Unity / Unreal)** — ADR-0027 §1.1 зафиксировал web-only клиент.
- ❌ **React Three Fiber** — лишний слой для фиксированной сцены.
- ❌ **Babylon.js** — переписывать Phase 1.5 без business-value.
- ❌ **Foveated rendering custom-tuning** — FFR работает по умолчанию, тюнинг отложен до Phase 3+ (нужен кастомный шейдер-uniform, отдельная карточка).
- ❌ **WebGPU** — на Quest Browser пока experimental, Three.js WebGPURenderer работает, но не даёт значимого perf-буста vs WebGL2 + Draco + KTX2. Отложено до стабилизации.

---

## 5. План rollout

### 5.1. Phase 2.0 (после Phase 1.5 готов)

1. **Подготовка pipeline** — добавить `gltf-transform` в `package.json`, скрипт `pnpm run assets:build`, CI-job `assets:verify` (проверка, что все `.glb` ≤ budget).
2. **Hand-tracking / controllers** — Three.js examples как base, привязка к WSS commands.
3. **UI panels** — WebXR Layers API для текста, glTF panels для рамок.
4. **Telemetry** — WSS reporter в client, ROS2 subscriber на `/quest/perf`.

### 5.2. Phase 2.1+

5. **Avatar робота** — модель по фото, glTF-pipeline через `gltf-transform`, animations (idle, forward, turn).
6. **Environment** — bridge-room по Quaternius + Poly Haven.
7. **AR passthrough overlay** — LiDAR-точки + depth-test (если Quest Browser поддержит `XRDepthSensing`).

### 5.3. Phase 3+

8. **FFR tuning** — кастомный шейдер-uniform если понадобится.
9. **VRM avatar pipeline** — если humanoid.
10. **Grafana dashboard** — `/quest/perf` → visualisation.

---

## 6. Acceptance checklist

- [x] Стек зафиксирован (Three.js r160+ + TS + Vite).
- [x] Asset format зафиксирован (glTF 2.0 + Draco + Meshopt + KTX2).
- [x] Performance budget зафиксирован (11.1 ms / 90 Hz Quest 3, ≤ 500 draw calls Quest 2).
- [x] Debug workflow описан (Immersive Web Emulator + adb + OVR Metrics Tool).
- [x] Telemetry план описан (WSS → ROS2 `/quest/perf`).
- [x] Альтернативы явно рассмотрены (Babylon, R3F, Native, VRM, raw glTF).
- [x] Trade-offs задокументированы.
- [ ] Phase 2+ карточки созданы в kanban (см. ниже).
- [ ] Phase 1.5 завершён и merged (зависимость).

---

## 7. Связанные документы

- `docs/research/2026-08-26-meta-quest-webxr-best-practices.md` — **этот research**, детали §1–§6
- `docs/adr/0027-meta-quest-ar-control.md` — Phase 1 (WSS + auth + protocol)
- `docs/adr/0028-avatar-supervisor.md` — координатор режимов (Quest ↔ Telegram)
- `docs/adr/0030-adr-numbering-sot.md` — почему ADR-0032 (а не 0027.1)
- `docs/architecture/meta-quest-api.md` — wire-протокол WSS (включая telemetry event'ы)
- `docs/plans/2026-08-25-webxr-captain-bridge-design.md` — общий design Captain Bridge
- `docs/plans/2026-08-24-meta-quest-telepresence.md` — vision телеприсутствия
- issue #1576 — Phase 1 (Quest 2 teleop)
- issue #1639 — Phase 1.5 (Three.js WebXR client)
- issue #1677 — research для этого ADR

---

## 8. Honest gaps (что мы НЕ знаем)

- **Quest Browser FPS для WebGL2 vs WebGPU** — не измерено, нужно бенчмарк на реальном устройстве (Phase 2.0 acceptance test).
- **OVR Metrics Tool IPC bridge в WebXR** — формально есть в `WEBXR_perf` (?), но реальный API нужно проверить на устройстве (Phase 2.0 spike).
- **`XRDepthSensing` support на Quest Browser** — по спецификации должен быть, в реальности — тестировать (Phase 2.1).
- **Battery drain для 90 Hz vs 72 Hz** — цифры от v59 update (12–18 мин savings) — для native apps, для WebXR может отличаться (Phase 2.0 measurement).
- **`OculusHandModel` AR-mode bug** — обходные пути не проверены на реальном Quest 3 в AR (Phase 2.1 spike).
