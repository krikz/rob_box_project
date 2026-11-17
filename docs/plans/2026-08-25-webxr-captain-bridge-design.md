# rob_box_quest WebXR client — Captain Bridge (Phase 1.5 design)

> **Companion:** реализует Phase 1.5 из
> [2026-08-24-meta-quest-telepresence.md](./2026-08-24-meta-quest-telepresence.md#phase-15--веб-клиент-threejs--webxr).
>
> **For Claude:** REQUIRED SUB-SKILL после approval — `superpowers:writing-plans`.
>
> **Дата:** 25.08.2026 · **Ветка:** `feature/avatar` · **Статус:** дизайн согласован с Шифу

## 1. Концепция: «капитанский мостик»

Вместо одной видео-плоскости перед камерой — пользователь стоит в центре 3D-сцены
(Captain Bridge) и расставляет вокруг себя **floating panels**, каждая из которых
показывает свой видео-поток с робота. На полу — LiDAR-облако точек. Управление
роботом — с физических контроллеров Quest (или WASD/Space на desktop).

Сравнение с базовым планом 1.5:

| Аспект | План 1.5 (baseline) | Captain Bridge (этот дизайн) |
|---|---|---|
| Видео | Full-screen `PlaneGeometry` перед камерой | 4 floating `Plane`, расставлены полукругом, draggable |
| LiDAR | На полу как Points | То же |
| Stream select | (Phase 2 / R10) | **Сразу:** lil-gui → add/move/close panels |
| Teleop | HUD overlay (grip, B) | Те же контроллеры Quest, но HUD-overlay внизу |
| Scope | Минимум | +1 фича (panels), та же сложность |

## 2. Архитектура

```
src/rob_box_quest/webxr_client/
├── package.json              # vite, three, @types/three, lil-gui
├── package-lock.json         # в репо (воспроизводимая сборка)
├── tsconfig.json
├── vite.config.ts            # base='/quest/' для Caddy
├── index.html                # <canvas> + <div id="hud">
├── src/
│   ├── main.ts               # bootstrap → wire.connection → scene
│   ├── wire/
│   │   ├── protocol.ts       # frame codec (mirror Python frame.py)
│   │   ├── connection.ts     # WSS, HELLO/WELCOME, heartbeat
│   │   └── messages.ts       # SUBSCRIBE/UNSUBSCRIBE/teleop/stop_emergency
│   ├── scene/
│   │   ├── captain_bridge.ts # scene, lights, ground grid
│   │   ├── video_panel.ts    # floating Plane + CanvasTexture (JPEG)
│   │   ├── lidar_overlay.ts  # THREE.Points на полу
│   │   └── panel_manager.ts  # add/move/close panels
│   ├── input/
│   │   ├── xr_teleop.ts      # Quest controllers (thumbstick+grip+B)
│   │   └── desktop_teleop.ts # WASD/Space/E fallback
│   └── ui/
│       ├── hud.ts            # status, PIN form, CONNECTION LOST
│       └── stream_select.ts  # lil-gui: добавить panel, выбор стрима
├── tests/
│   └── playwright/           # Phase 1.7 e2e (отдельная карточка)
└── .gitignore                # + dist/, node_modules/

dist/                         # build artifact → в .gitignore
```

## 3. Рендер-потоки

| Стрим | Three.js объект | Обновление |
|---|---|---|
| `camera_rear`, `camera_oak_color`, `camera_oak_depth`, `camera_ceiling` | `THREE.Plane + CanvasTexture` на **floating panel** | JPEG → `Image` → `texture.image = img` → `needsUpdate=true`. Drop-oldest, если GPU занят. |
| `lidar_2d` | `THREE.Points` на полу сцены (y=0) | `BufferGeometry.setAttribute('position', Float32Array)` + vertex colors по дистанции (HSL: близко=красный, далеко=синий). |

Дефолтная раскладка: 4 панели полукругом вокруг пользователя (radius 2 м, углы
±60°, ±20° по горизонту, высота глаз ±0.2 м), каждая 1.2×0.7 м. Layout
reset → возвращает дефолт.

Drag (desktop — мышь, XR — controller point + trigger):
1. Raycaster попадает в panel → фокус → рамка подсветки.
2. Drag — перемещение в плоскости камеры (XZ) с учётом расстояния от камеры.
3. Release — фиксация позиции.

## 4. stream_select UI

`lil-gui` справа-сверху (desktop, ~250 px) или WebXR DOM-overlay (Quest, если
поддерживается; иначе — fallback на 3D-sprite с raycaster):

- **Add Panel** → dropdown со списком стримов из `STREAM_CATALOG`
  (`camera_rear`, `camera_oak_color`, `camera_oak_depth`, `camera_ceiling`,
  `lidar_2d` опционально) → создаёт новую floating panel.
- Для каждой panel: список с кнопками **✕ Close** и **Switch Stream**.
- **Layout reset** → возвращает дефолтное расположение.
- **Connection status** (CONNECTED / LOST / RECONNECTING, цветной индикатор).
- **PIN form** — отдельный overlay на входе до HELLO.

## 5. Wire-протокол (клиент)

Зеркало `protocol/frame.py`:

```typescript
enum FrameType { HELLO=0x01, WELCOME=0x02, SUBSCRIBE=0x03, UNSUBSCRIBE=0x04,
                 BINARY_FRAME=0x10, JSON_CMD=0x11, JSON_EVENT=0x12,
                 GOODBYE=0x20, ERROR=0xFF }

encodeFrame(type, streamId, payload) → ArrayBuffer   // [1B type][4B sid LE][LEB128 len][payload]
decodeFrame(buf) → { type, streamId, payload }
```

Контракт SUBSCRIBE → BINARY_FRAME:
- `SUBSCRIBE{topic, quality}` (JSON_CMD=0x11) — стрим `0x0001..0x0FFF` (client-initiated).
- Сервер отвечает: `BINARY_FRAME(0x10)` со `stream_id` из `0x1000..0xFFFF`,
  payload начинается с 4-байтового topic_id (LE uint32) + raw data.
- Topic-tag соответствует `STREAM_CATALOG` (`meta-quest-api.md` §4):
  `0x1001` camera_rear, `0x1003` oak_color, `0x1004` oak_depth, `0x1005` ceiling,
  `0x1101` lidar_2d.

Teleop (30 Hz):
```json
{ "cmd":"teleop_twist", "ts_ms":..., "seq":N, "linear":{"x":0.5}, "angular":{"z":0.3}, "deadman":true }
```
`deadman=false` → сервер игнорирует (страховка от застрявшего пакета после отпускания grip).

Emergency:
```json
{ "cmd":"stop_emergency", "ts_ms":..., "source":"controller_b"|"ui_button"|"client_lost" }
```

Heartbeat: клиент шлёт JSON_CMD `{"cmd":"ping","ts_ms":...}` каждые 200 мс;
отсутствие > 600 мс → сервер рвёт сокет (watchdog).

## 6. Teleop

**XR (Quest controllers):**
| Вход | Действие |
|---|---|
| Left thumbstick Y | linear.x = stick.y * MAX_LINEAR (0.5 м/с) |
| Left thumbstick X | angular.z = stick.x * MAX_ANGULAR (1.0 рад/с) |
| **Grip** (любой) | deadman=true пока зажат |
| **B button** (любой) | stop_emergency (edge-triggered, debounce 300 мс) |

**Desktop fallback** (активируется автоматически если нет XR):
| Клавиша | Действие |
|---|---|
| W/S | linear ± |
| A/D | angular ± |
| Space | deadman hold |
| E | emergency stop |

Throttle 30 Hz, монотонный `seq`. На отпускании grip → следующий фрейм
`deadman:false`, через 100 мс — `stop` (twist=0) для гарантии safe-stop.

## 7. Сборка и Docker

- `vite.config.ts`: `base: '/quest/'` (для Caddy reverse-proxy)
- `npm run build` → `src/rob_box_quest/webxr_client/dist/`
- `dist/` в `.gitignore`, `package-lock.json` коммитится
- Dockerfile (Phase 1.6 — отдельная карточка):
  ```Dockerfile
  FROM node:20-alpine AS builder
  WORKDIR /src
  COPY webxr_client/package*.json ./
  RUN npm ci
  COPY webxr_client/ ./
  RUN npm run build
  ```

## 8. Out of scope (явно)

- ❌ H.264 — Phase 2 (tech-debt если JPEG latency > 200 мс, см. ADR-0027 §4.3)
- ❌ Person detections (`person.ts` = заглушка)
- ❌ Layout persistence (localStorage) — nice-to-have
- ❌ Multi-user / spatial audio — Phase 3
- ❌ mTLS / TOTP — Phase 3

## 9. Definition of Done Phase 1.5

- [ ] `npm ci && npm run build` завершается без ошибок
- [ ] `tsc --noEmit` чисто
- [ ] В desktop-браузере открывается страница с PIN-формой
- [ ] После ввода PIN → 4 панели с видео-потоками + LiDAR на полу
- [ ] `stream_select` (lil-gui): Add Panel работает, Close работает
- [ ] WASD/Space/E двигают робота (Phase 1.7 e2e — telemetry от ros)
- [ ] Сборка артефакта ≤ 1.5 MB gzipped (lighthouse-style sanity)
- [ ] Нет CDN-зависимостей в runtime (всё через npm + importmap local)

## 10. Открытые вопросы / future work

- Drag в XR: начальная реализация — controller point + trigger (простая).
  Альтернатива — hand tracking + pinch (Phase 2).
- HUD-overlay в Quest без WebXR DOM-overlay → 3D-sprite с raycaster.
- Голосовое управление panels («открой заднюю камеру») — Phase 2.

## 11. Решения (записано для следующей сессии)

| # | Решение | Почему |
|---|---|---|
| 1 | Vite + TypeScript | HMR в dev, типы, tree-shaking |
| 2 | 4 floating panels + draggable | «Капитанский мостик» — Шифу |
| 3 | JPEG в CanvasTexture через `<img>` | Проще, чем `<video>` для покадрового JPEG |
| 4 | lil-gui для stream_select | ~5 KB, хорошо знаком, легко кастомизируется |
| 5 | Teleop от физических контроллеров Quest | Шифу; desktop WASD — fallback |
| 6 | dist/ в .gitignore, multi-stage Docker | Не раздуваем git, воспроизводимо |
| 7 | out of scope: H.264, person detections, persistence | YAGNI, Phase 2+ |

## 12. Связанные документы

- [meta-quest-telepresence.md](./2026-08-24-meta-quest-telepresence.md) — общий план Phase 1
- [meta-quest-api.md](../architecture/meta-quest-api.md) — wire-протокол (source of truth)
- [ADR-0027](../adr/0027-meta-quest-ar-control.md) — решения по стеку
- [ADR-0028](../adr/0028-avatar-supervisor.md) — FSM режимов супервизора (Phase 1.6+)
