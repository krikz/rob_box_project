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
робота и LiDAR-overhead, по бокам — floating panels с остальными
видео-камерами (OAK-D depth, ceiling), на стене — ARM- и status-индикаторы.

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
│   ├── status_hud.ts           robot_status HUD (BAT/WIFI/SPD/RTT/MODE)
│   ├── tts_picker_menu.ts      TTS picker: 3D-меню голосов (AV-27)
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
│   ├── msgpack.ts              Минимальный msgpack-декодер (robot_status)
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
    │   • PanelManager.resetLayout() — боковые panels (±75°)
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
├── PanelManager.panels: 2× VideoPanel (Wave 3.A)
│   ├── angle -75° from "forward"  → camera_oak_depth
│   └── angle +75°                 → camera_ceiling
├── mainScreen: VideoPanel (front wall, MAIN_SCREEN_TOPIC = camera_rear)
├── LiDAR overlay (Group в центре робота: Points на плоскости луча
│      y=0.4765 м + вертикальный «занавес» до пола)
├── VR controller visuals (только в WebXR)
├── Arm-state HUD (text-sprite справа вверху на стене)
├── Status HUD (text-sprite слева вверху: BAT / WIFI / SPD / RTT / MODE)
├── TTS picker launch tab (AV-27: плашка VOICE, (-1.35, 0.95, -3.85))
└── TTS picker menu (AV-27: header + строки голосов + APPLY/STOP/CLOSE + footer,
       renderOrder 20 — тот же слой глубины, что у stream_menu)
```

LiDAR строится от начала координат сцены — это `base_link` робота, то есть
пол ровно под оператором. Точки лежат на реальной высоте плоскости луча N10
(0.4765 м, `rob_box.xacro:338`), с учётом смещения лидара на 0.17 м назад.
ROS→сцена: «вперёд робота» (+x REP-103) = −Z сцены, «влево» (+y) = −X.
Точки рисуются без тумана и поверх геометрии комнаты (`alwaysVisible`),
потому что скан достаёт до 10 м, а виртуальная комната ~7×8 м.

`camera_oak_color` (0x1003) на панель не выводится: это тот же сенсор, что
и на экране-стене (`camera_rear` = 0x1001 через ROS). Углы ±75° выбраны,
чтобы панели не перекрывали экран-стену во фронтальном секторе.

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
resetLayout() → создать panels по углам из опции angles (дефолтные topics)
createPanel(topic, pos, facing) → новый panel
movePanel(id, pos) → обновить position (с clamping)
resizePanel(id, size) → обновить size
deletePanel(id) → удалить
setTopic(id, topic) → поменять topic (триггерит set_panel_topic по WSS)
list() / get(id) → queries
```

Persistence: layout в localStorage НЕ сохраняется (дропнут в `d6548abb
refactor(quest): drop lil-gui and orphaned voice/layout modules`). Каждый
вход — дефолтная раскладка. Wave 3.B добавит persistence
(`rob_box_quest.panel_layout.v1`) и drag/resize панелей.

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

### 7.4 Status HUD (Wave 3.A, R8)

Спрайт слева вверху на стене — зеркально ARM-индикатору. Пять строк:

| Строка | Источник | Нет источника |
|---|---|---|
| `BAT` | `robot_status.battery_pct`, иначе `battery_v` (вольты) | `—` |
| `WIFI` | `robot_status.wifi_rssi` (`/proc/net/wireless` на Vision Pi) | `—` |
| `SPD` | `robot_status.vel_linear` (из `/odom`) | `—` |
| `RTT` | client ping → server pong (эхо `ts_ms`) | `—` до первого pong |
| `MODE` | `emergency` / `teleop_active` / `idle` | `—` |

Цвет значения: зелёный / жёлтый (WIFI ≤ -75 dBm, RTT ≥ 200 мс) /
красный (BAT ≤ 20%, RTT ≥ 400 мс, MODE=emergency) / серый (нет источника).
Прочерк вместо нуля намеренно: «0%» и «нет данных о заряде» — разные вещи.

Формат строк — чистая функция `formatStatusLines` (`scene/status_hud.ts`),
разбор payload — `parseRobotStatus` поверх минимального msgpack-декодера
(`wire/msgpack.ts`, без runtime-зависимостей).

### 7.5 Mode manager (client UI-state)

Маленький observable-стор:
- `voiceMode: "off" | "radio" | "robot_voice"`
- `teleopState: "disarmed" | "armed"`
- `currentVoice: string | null`
- `currentPreset: VoicePreset | null`

Используется в `main.ts` для синхронизации UI-state с реальным
arm-стейтом и voice-режимом. `currentVoice` / `currentPreset` наполняются
из серверных `voice_list.active_voice` и `voice_set_ack` (AV-27) — клиент
своё значение не выдумывает; до ответа сервера там `null`, и picker
показывает прочерк.

### 7.6 TTS picker (AV-27, 3D-меню выбора голоса)

Точка входа — плашка **VOICE** на слое указателя (левее экрана-стены). В VR
клавиатуры нет, поэтому вход обязан быть кликабельным объектом; на десктопе
дополнительно работает клавиша **V**.

Пять состояний (`state/tts_picker_state.ts`, чистый редьюсер):

| Состояние | Когда | Что видно |
|---|---|---|
| `loading` | открыли меню → отправили `list_voices` | плашка `◐ loading voices…`, footer `waiting for voice_list…` |
| `empty` | сервер ответил `{voices: []}` | дословно `Provider does not expose a voice list` — голоса НЕ выдумываем |
| `ready` | список пришёл | строки `{display_name · language}` + `{provider · gender · presets}`, активный голос подсвечен `ACTIVE` |
| previewing | нажали `PREVIEW` строки | footer `PREVIEW <voice>: chunk N/total…` + активная кнопка `STOP` |
| applying | нажали `APPLY` | header `APPLYING <voice>…`, строки и кнопки залочены до `voice_set_ack`/`_nack` |

Отрисовка — `scene/tts_picker_menu.ts`: каждая интерактивная зона это
отдельный меш с canvas-текстурой и целью указателя `tts:*` (`voice:<id>`,
`preview:<id>`, `apply`, `stop`, `close`, `launch`) — тот же приём, что в
`stream_menu.ts`, включая `renderOrder = 20`. Мёртвые кнопки (`APPLY` без
выбора, `STOP` без preview) в PointerSystem не регистрируются: кнопка,
которая ловит луч и ничего не делает, обманывает оператора.

Аудио preview — `ui/preview_audio_sink.ts`. Сервер шлёт
`JSON_EVENT{preview_voice_audio, request_id, content_type, seq, total}`, а
следом `BINARY_FRAME` **со `stream_id = 0`** (control): у него нет топика в
`subscribe_ack`, поэтому `main.ts` роутит нулевой stream в sink, а не в
видео-панели. Чанки копятся под `request_id` (лимит 4 МБ), на
`preview_voice_done` склеиваются и играются через `decodeAudioData` +
`AudioBufferSourceNode` (в immersive-vr автоплей media-элементов режется,
WebAudio — единственный надёжный путь). `preview_voice_error` показывается
инлайном как `ERROR: <reason>`.

Разрыв связи → `disconnected`: активный голос обнуляется, picker уходит в
`loading`, preview обрывается. После реконнекта `list_voices` уходит
заново, если меню открыто (провайдер мог смениться, а старый список — не факт).

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

**TTS picker (`list_voices` / `set_voice` / `preview_voice`)** реализован:
сервер — AV-27 (`ws_server.py` + supervisor + `tts_voice_registry`), клиент —
`state/tts_picker_state.ts` + `scene/tts_picker_menu.ts` + `ui/preview_audio_sink.ts`
(см. §7.6). Синтез самого preview на сервере в MVP ещё не сделан — сервер
честно отвечает `preview_voice_error{preview_synthesis_not_implemented_in_mvp}`,
и picker показывает это как `ERROR: …`, а не как «сыграло».
**`set_panel_topic` НЕ реализован** — в клиенте есть только TypeScript-тип
(`wire/messages.ts`), сервер этой команды не знает.
`voice_state` (0x1202) есть в registry, но публикатора на сервере нет.

---

## 9. Acceptance checklist (Phase 2.3)

- [x] Bridge environment загружается (CC0 GLB + HDR, 5 файлов ≤ 130 KB total)
- [x] Layout reset создаёт боковые panels (Wave 3.A: 2 шт, ±75°), facing inward
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
- [x] Status HUD: battery / Wi-Fi / speed / RTT / mode (Wave 3.A, R8)
- [ ] LocalStorage layout persistence — Wave 3.B

---

## 10. См. также

- [meta-quest-api.md](./meta-quest-api.md) — WSS protocol контракт
- ADR-0027 — Meta Quest AR control design
- ADR-0028 — Avatar Supervisor FSM
- ADR-0032 — Meta Quest WebXR stack + assets
- `src/rob_box_quest/webxr_client/README.md` — client code quickstart
- `docs/e2e/captain-bridge-phase2-checklist.md` — manual E2E checklist
- `docs/research/2026-08-26-meta-quest-webxr-best-practices.md` — research
