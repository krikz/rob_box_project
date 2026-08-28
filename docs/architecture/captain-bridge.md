# Captain Bridge — Architecture Overview

> Сцена «капитанский мостик» — основное 3D-окружение Meta Quest / WebXR
> клиента. Phase 2 покрывает полную картину: environment + panels +
> teleop + voice + error/help/loading UX.
>
> Документ — high-level overview. Детали реализации смотрите в коде и в
> смежных документах (`meta-quest-api.md`, `minimax-tts-architecture.md`,
> ADR-0027, ADR-0028, ADR-0032).

---

## 1. Что такое Captain Bridge

Captain Bridge — это место, откуда оператор управляет роботом. Метафора:
кресло пилота, перед ним большая стена-экран с фронтальной камерой
робота и LiDAR-overhead, вокруг — 4 floating panels с другими
видео-камерами (rear, OAK-D color/depth, ceiling).

Двухрежимный вход (по наличию WebXR):

| Режим | Как зайти | Что доступно |
|---|---|---|
| **Desktop fallback** | Браузер без `immersive-vr` (Chromium dev-tools, мобильный) | 2D-рендер, WASD + Space + E |
| **WebXR / VR** | `requestSession("immersive-vr")` в submit-handler | Stereo, XR controllers, hand tracking, voice capture |

Оба режима используют одну и ту же WebSocket-сессию и одну и ту же
`PIN`-аутентификацию. В VR-режиме поверх сцены накладывается WebXR-стерео
рендер, и `window.requestAnimationFrame` заменяется на
`session.requestAnimationFrame` (см. §6).

---

## 2. Module layout

```
src/
├── entry.ts                    HTML-DOM bootstrap
├── main.ts                     Lifecycle, overlays, scene assembly
├── xr_bootstrap.ts             WebXR session manager (request/end, events)
├── scene/
│   ├── captain_bridge.ts       Scene graph (lights, camera, panels, main screen)
│   ├── panel_manager.ts        Pure-data panel state (positions, sizes, topics)
│   ├── video_panel.ts          VideoPanel (Three.js mesh + JPEG ingestion)
│   ├── lidar_overlay.ts        LiDAR points on the floor
│   ├── lidar_payload.ts        LiDAR wire-format decoder
│   └── bridge_assets.ts        CC0 GLB + HDR loader (DRACO + KTX2 + Meshopt)
├── input/
│   ├── teleop_fsm.ts           FSM: idle → armed → emergency
│   ├── teleop_config.ts        XR button indices (oculus-touch-v2)
│   ├── desktop_teleop.ts       WASD/Space/E → teleop_fsm
│   ├── xr_teleop.ts            XR controllers → teleop_fsm
│   └── voice_capture.ts        Mic PCM 16 kHz → WSS VOICE_AUDIO
├── wire/
│   ├── connection.ts           WSS client (HELLO/WELCOME/subscribe/binary)
│   ├── protocol.ts             Binary frame codec
│   └── messages.ts             JSON_CMD/JSON_EVENT TypeScript types
└── ui/
    ├── loading_screen.ts       Async-asset loading overlay
    ├── error_overlay.ts        Disconnect-watchdog overlay (>5s)
    ├── help_overlay.ts         H-key hotkey cheat sheet
    └── mode_manager.ts         Client UI-state store (voice mode / armed)
```

Каждый модуль — **pure logic** (тестируется без Three.js / DOM), либо
**Three.js scene-graph** (тестируется с jsdom + mock-Three). `main.ts`
склеивает всё в один bootstrap-pipeline.

---

## 3. Lifecycle / Boot order

```
window.DOMContentLoaded
    ↓
entry.ts: find DOM elements (canvas, pin-overlay, status, help-toggle, body)
    ↓
main.ts bootstrap(opts)
    │
    ├─ Создать Phase 2.3 overlays:
    │   • loading_screen (на body)
    │   • error_overlay + DisconnectWatchdog (5 s threshold)
    │   • help_overlay (H key + help-toggle button)
    │   • mode_manager (UI-state стор)
    │
    ├─ CaptainBridge.createCaptainBridge({ canvas, enableXr: true })
    │   • THREE.WebGLRenderer
    │   • Scene: ambient + dir light, fog, floor, grid
    │   • PerspectiveCamera (70°, pos (0, 1.6, 0))
    │   • PanelManager.resetLayout() — 4 panels на полукруге
    │   • mainScreen (front wall screen для MAIN_SCREEN_TOPIC)
    │
    ├─ bridge.loadEnvironment()             ← Promise<BridgeAssetHandle>
    │     loading.watch(promise, "Loading environment…")
    │     (success → loading.hide(); fail → loading.fail(reason))
    │
    ├─ TeleopFSM, DesktopTeleop, XrBootstrap, VoiceCapture
    │   • register XR controllers (когда ready)
    │
    ├─ World rAF loop: teleop loop + render loop
    │
    └─ PIN form submit handler
        ├─ validate 6-digit PIN
        ├─ Connection.open()      ← onStateChange('connected' → watchdog.markConnected)
        └─ autoEnterVr()          ← requestSession в user-activation submit
```

LIFECYCLE invariants:

1. **Overlays создаются ДО loadEnvironment** — иначе loading-screen не успеет
   перекрыть canvas.
2. **PIN submit → Connection.open + autoEnterVr атомарны** — WebXR
   `requestSession` требует user-activation ЖЕСТКО в submit-handler,
   иначе браузер отклонит (Quest Browser 2026+).
3. **`session.requestAnimationFrame` обязателен в VR** — `window.rAF`
   заморожен immersive-сессией.
4. **`exitVr()` сбрасывает arm/voice state** — иначе grip-стейк останется
   «зажатым» после выхода из VR.

---

## 4. Scene graph

```
THREE.Scene
├── AmbientLight (0xffffff, 0.6)
├── DirectionalLight (0xffffff, 0.4, pos (2,4,1))
├── Floor (MeshStandardMaterial, 20x20, color #14181f)
├── GridHelper (20x20, color #444a52/#2a2f36)
├── CC0 environment (Phase 2.1)
│   ├── bridge_floor.optimized.glb
│   ├── bridge_walls.optimized.glb
│   ├── bridge_props.optimized.glb
│   ├── bridge_nav.optimized.glb         (navmesh / walkable regions)
│   ├── bridge_occluders.optimized.glb   (occluder meshes)
│   └── bridge_env_1k.hdr                (IBL)
├── PanelManager.panels: 4× VideoPanel
│   ├── angle -60° from "forward"  → camera_rear
│   ├── angle -20°                 → camera_oak_color
│   ├── angle +20°                 → camera_oak_depth
│   └── angle +60°                 → camera_ceiling
├── mainScreen: VideoPanel (front wall, MAIN_SCREEN_TOPIC = camera_rear)
├── LiDAR overlay (THREE.Points на плоскости пола)
├── VR controller visuals (только в WebXR)
└── Arm-state HUD (text-sprite на стене)
```

Camera: `PerspectiveCamera(70°, aspect, 0.05, 50)` в позиции
`(0, 1.6, 0)` — высота глаз ~1.6 м. Y-offset для panels — та же высота
(см. `panel_manager.ts: panelYOffset = 1.6`).

---

## 5. State machines

### 5.1 Connection lifecycle (`wire/connection.ts`)

```
   ┌─ idle ─┐
   │        ↓
   │   connecting ─── HELLO sent
   │        ↓
   │   authenticating ── wait WELCOME
   │     ↓          ↓
   │  connected   auth_failed (close socket, PIN overlay reopens)
   │     ↓
   │  reconnecting (exponential backoff 1→30 s)
   │     ↓         ↓
   │  connected   closed (финал)
   ↓
  disposed
```

Each transition emits `onStateChange`. Phase 2.3 wires `error_overlay`
через `DisconnectWatchdog`: при `reconnecting` стартует 5-секундный
таймер, по истечении — `error_overlay.show("Connection lost", "{N}s…")`.
При восстановлении (`connected`) — `error_overlay.dismiss()`.

### 5.2 Teleop FSM (`input/teleop_fsm.ts`)

```
idle ─[deadman=true]→ armed ─[tick]→ emits teleop_twist @ 30 Hz
                                 ↓
                          triggerEmergency()
                                 ↓
                          emergency (one-shot)
                                 ↓
                          armed (auto-release after 1 s)
```

`deadman` — флаг, который должен быть `true` на каждом `teleop_twist`,
иначе сервер игнорирует пакет (страховка от «отпустил grip, пакет
пришёл позже»).

### 5.3 Voice mode state (`ui/mode_manager.ts`)

```
       ┌───────── off ─────────┐
       ↓                       ↑
   radio ──────→ robot_voice ──┘
  (правый grip)   (левый grip → STT → LLM → TTS)
```

Управляется XR-контроллерами (grip-кнопки). Приоритет: `robot_voice`
> `radio` если оба зажаты (робот-голос интереснее с т.з. demo-flow).
Edge-triggered — клиент шлёт `voice_ptt_start{mode}` / `voice_ptt_stop{mode}`.

Перед `voice_ptt_start{robot_voice}` клиент обязан отправить
`voice_mode{mode:"ttts_proxy"}` — иначе supervisor не переключит
`voice_input_mode=respeaker` → STT не услышит (ADR-0028 §5).

### 5.4 Panel manager (`scene/panel_manager.ts`)

Не state-machine в строгом смысле — pure-data store:

```
resetLayout() → создать 4 panels на полукруге (дефолтные topics)
createPanel(topic, pos, facing) → новый panel
movePanel(id, pos) → обновить position (с clamping)
resizePanel(id, size) → обновить size
deletePanel(id) → удалить
setTopic(id, topic) → поменять topic (триггерит set_panel_topic по WSS)
list() / get(id) → queries
```

Persistence: Phase 2.3 пока НЕ сохраняет layout в localStorage
(был дропнут в `d6548abb refactor(quest): drop lil-gui and orphaned
voice/layout modules`). Каждый вход — дефолтный 4-panel layout.
Phase 3 добавит localStorage persistence (`rob_box_quest.panel_layout.v1`).

---

## 6. WebXR session lifecycle

```
PIN submit (user-activation!)
    ↓
xr.isSupported("immersive-vr")  ← Promise<boolean>
    ↓ true
xr.requestSession("immersive-vr")  ← Promise<XRSession>
    ↓
xr.bindSession(session)
session.addEventListener("inputsourceschange", refreshCache)
bridge.attachXrSession(session)  ← stereo renderer, controllers overlay
    ↓
session.requestAnimationFrame(xrFrame)  ← teleop + render loop
    ↓ user presses Meta button / endSession()
xr.endSession()
    ↓ finally
session.cancelAnimationFrame(xrRafId)
modeManager.setTeleopState("disarmed")
applyVoicePtt(false, false)
xrTeleopHandle.destroy()
```

Invariants:
- В immersive-vr `window.requestAnimationFrame` НЕ работает. Используется
  только `session.requestAnimationFrame`.
- `inputsourceschange` событие обязательно для refresh контроллеров —
  иначе после добавления/удаления controller он «застревает».
- `xr.endSession()` ВСЕГДА в try/finally, иначе контроллер-стейк не
  очистится и grip останется «зажатым».

---

## 7. Phase 2.3 UX overlays (NEW)

Phase 2.3 добавил три overlay-модуля + один client-side state store:

### 7.1 Loading screen

Показывается с момента bootstrap до завершения `bridge.loadEnvironment()`
(CC0 GLB + HDR). Содержит spinner + текст. `minVisibleMs=250` чтобы не
мигать на быстром кеше. При ошибке загрузки — текст становится красным,
spinner останавливается.

### 7.2 Error overlay + Disconnect watchdog

`error_overlay` — UI-карточка с headline + detail + dismiss button.
`DisconnectWatchdog` — обёртка, которая показывает overlay через **5 секунд**
после `markDisconnected()`. До этого пользователь не видит UI-тревоги —
только `status: RECONNECTING…`. Если reconnect успешен за 5 с —
overlay вообще не появляется. Это снижает «flapping» тревогу при
кратковременных сетевых разрывах.

При `closed` (финал, не reconnect) — overlay показывается сразу, без
5-секундного порога.

### 7.3 Help overlay (H key)

Список горячих клавиш, сгруппированный по `Desktop` / `WebXR` / `Global`.
Тогглится клавишей `H` или кликом по `?`-кнопке в HUD. Закрывается
повторным `H`, `Escape`, или кликом вне карточки.

Игнорирует `H`/`Esc` когда фокус в `<input>` / `<textarea>` /
`contenteditable` — иначе H печаталась бы в PIN-инпуте.

### 7.4 Mode manager (client UI-state)

Маленький observable-стор:
- `voiceMode: "off" | "radio" | "robot_voice"`
- `teleopState: "disarmed" | "armed"`
- `currentVoice: string | null`
- `currentPreset: VoicePreset | null`

Используется в `main.ts` для синхронизации UI-state с реальным
arm-стейтом и voice-режимом. **Сейчас** не показывается в HUD напрямую
(Phase 2.3 не делал voice picker UI — голос управляется XR-grip'ами).
**В Phase 3** будет использоваться для отображения текущего голоса
и подсветки активной кнопки voice picker.

---

## 8. WSS protocol (high level)

Captain Bridge использует subprotocol `robbox-quest-v1`. Полная спецификация
— [meta-quest-api.md](./meta-quest-api.md). Главные эндпоинты:

| Категория | Команды | События |
|---|---|---|
| **Стримы** | `subscribe`, `unsubscribe`, `stream_list` | `subscribe_ack`, `subscribe_nack`, `stream_list` |
| **Видео** | `set_panel_topic`, `stream_select` | `stream_select_ack` |
| **Teleop** | `teleop_twist`, `stop_emergency` | `safety_stop` |
| **Голос** | `voice_mode`, `voice_ptt_start`/`_stop`, `set_voice`, `list_voices`, `preview_voice` | `voice_mode_ack`, `voice_state`, `voice_list`, `voice_set_ack`/`_nack`, `preview_voice_audio`/`_done`/`_error` |
| **System** | `ping`, `admin_logs`, `admin_logs_stop` | `pong`, `heartbeat`, `robot_alert`, `admin_logs_chunk`/`_end` |

Голосовой канал (`voice_ptt_start`) публикуется в бинарный поток
`VOICE_AUDIO` (PCM 16 kHz int16) — см. `wire/protocol.ts`.

Phase 2 §4 (TTS picker) — клиент посылает `list_voices` при подключении,
получает `voice_list`, рисует UI dropdown (Phase 3 ещё не сделан).
`set_voice` / `preview_voice` — кнопки в picker.

---

## 9. Acceptance checklist (Phase 2.3)

- [x] Bridge environment загружается (CC0 GLB + HDR, 5 файлов ≤ 130 KB total)
- [x] Layout reset создаёт 4 panels на полукруге, facing inward
- [x] Camera в (0, 1.6, 0), looking forward (no lookAt — задаётся через panel facing)
- [x] WebXR auto-entry после PIN submit (immersive-vr)
- [x] Desktop fallback (WASD + Space + E)
- [x] XR controllers — left stick movement, right stick arm toggle, B/Y emergency
- [x] Voice PTT: left grip = robot_voice (STT→LLM→TTS), right grip = radio (passthrough)
- [x] Loading screen пока грузятся ассеты (~<500 ms на dev-машине)
- [x] Error overlay при disconnect > 5s, auto-hide при восстановлении
- [x] Help overlay по H — список горячих клавиш
- [x] Tooltips на status + help-toggle кнопках (HTML `title` attr)
- [x] `npm run build` + `npm test` зелёные (142/142 tests, build < 600 KB JS)
- [x] `npm run gltf:verify` PASS (6/6 assets compliant, KTX2 — warning only)
- [ ] FPS ≥ 60 на Meta Quest 2 — best effort, метрика в Phase 3 (telemetry)
- [ ] LocalStorage layout persistence — Phase 3

---

## 10. См. также

- [meta-quest-api.md](./meta-quest-api.md) — WSS protocol контракт
- ADR-0027 — Meta Quest AR control design
- ADR-0028 — Avatar Supervisor FSM
- ADR-0032 — Meta Quest WebXR stack + assets
- `src/rob_box_quest/webxr_client/README.md` — client code quickstart
- `docs/e2e/captain-bridge-phase2-checklist.md` — manual E2E checklist
- `docs/research/2026-08-26-meta-quest-webxr-best-practices.md` — research
