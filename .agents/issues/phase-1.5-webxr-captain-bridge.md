## Phase 1.5 — Three.js WebXR client (Captain Bridge)

### Источник истины
- Дизайн: `docs/plans/2026-08-25-webxr-captain-bridge-design.md` (согласован с Шифу 25.08.2026)
- Контракт сервера: `docs/architecture/meta-quest-api.md` §1-§7
- ADR: `docs/adr/0027-meta-quest-ar-control.md` §3.1, §3.3, §4.2
- Phase 1.4 (сервер): commit `b720c8af feat(quest): camera (JPEG, bypass-ROS) + lidar (2D) + stream_select`

### Контекст
Phase 1.1–1.4 (Python-сервер `rob_box_quest`) готовы. Сервер уже отдаёт:
- JPEG-кадры камер через WSS `BINARY_FRAME(stream_id, jpeg_bytes)` (без 4-byte topic-id префикса — реальная схема расходится с api.md §4, см. замечание ниже)
- 2D LiDAR через тот же WSS
- Handshake HELLO/WELCOME/SUBSCRIBE → ack со stream_id

Phase 1.5 — **только клиент**. Сервер и Docker-деплой не трогаем.

### Что строим
Three.js + TypeScript проект `src/rob_box_quest/webxr_client/`:

- **Captain Bridge** сцена: пользователь стоит в центре, вокруг расставлены 4 floating panels с видео-потоками (полукруг, radius 2 м), LiDAR-points на полу, draggable
- **Wire-протокол клиент**: зеркало `protocol/frame.py` — HELLO/WELCOME/SUBSCRIBE/UNSUBSCRIBE/JSON_CMD(teleop_twist, stop_emergency)/JSON_EVENT(ping)
- **Рендер JPEG**: `<img>` → `CanvasTexture` → `Plane` (drop-oldest если GPU занят)
- **stream_select UI** (`lil-gui`): Add Panel / Close / Layout reset / Connection status / PIN form
- **Teleop**: XR-контроллеры Quest (thumbstick + grip + B), desktop fallback WASD/Space/E, 30 Hz throttle, монотонный `seq`, deadman
- **Watchdog**: клиент шлёт `JSON_EVENT{type:"ping"}` каждые 500 мс; 3 подряд heartbeat отсутствуют → UI "CONNECTION LOST", grip отпускается

### Стек
- TypeScript 5.x, Vite 5.x, Three.js r170+, lil-gui
- Vitest для unit-тестов (codec, teleop FSM, panel state)
- `tsc --noEmit` + `npm run build` + `npm run test` в CI

### Out of scope
- ❌ H.264 — Phase 2 (tech-debt если JPEG latency > 200 мс)
- ❌ Docker/Caddy/деплой — Phase 1.6 (отдельная карточка)
- ❌ Hand-tracking pinch-grab — Phase 2
- ❌ Layout persistence (localStorage) — nice-to-have
- ❌ E2E (Playwright) — Phase 1.7

### Definition of Done
- [ ] `npm ci` и `npm run build` без ошибок
- [ ] `tsc --noEmit` чисто
- [ ] `npm test` ≥ 8 unit-тестов (codec round-trip, teleop FSM, panel manager, lidar payload parse) — все зелёные
- [ ] В desktop-браузере открывается страница с PIN-формой
- [ ] После ввода PIN → 4 panels полукругом, LiDAR-points на полу
- [ ] `stream_select` (lil-gui): Add Panel работает, Close работает, Layout reset возвращает дефолт
- [ ] WASD/Space/E шлют teleop_twist 30 Hz (verified через mock-сервер test_ws_server.py)
- [ ] Сборка `dist/` ≤ 1.5 MB gzipped
- [ ] Нет CDN-зависимостей в runtime (всё через npm + importmap local)
- [ ] Commit: `feat(quest): webxr_client Captain Bridge (TS/Three.js)` + `test(quest): unit tests for codec/teleop/panels`

### Расхождение с api.md §4 (зафиксировать)
api.md обещает 4-байтовый topic_id в начале payload BINARY_FRAME. Реальный сервер (Phase 1.4, commit b720c8af) шлёт чистый JPEG без префикса — клиент роутит по `stream_id` из заголовка фрейма + маппингу из `subscribe_ack`. Это зафиксировать в комментарии `protocol.ts` и TODO для future-proof (если api.md будет приведён в соответствие).

### Acceptance (raw-evidence обязателен, ADR-0018)
- [ ] `npm run build` output: hash + размеры chunks
- [ ] `npm test -- --reporter=verbose` — полный вывод
- [ ] `tsc --noEmit` — exit code 0
- [ ] `npx vite-bundle-visualizer` или аналог: ни один chunk > 500 KB
- [ ] Скриншот desktop-сцены с 4 panels (через `npm run dev` + локальный mock-сервер)