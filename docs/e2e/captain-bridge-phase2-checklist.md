# Captain Bridge Phase 2 E2E Checklist (manual)

> Ручной чек-лист для проверки Phase 2 фич Captain Bridge перед релизом.
> Автоматические e2e (Playwright) пока не покрывают WebXR — этот чек-лист
> обязателен перед слиянием feature/avatar в main.
>
> Каждый шаг помечен `[headless]` (можно проверить в Chromium dev-tools
> без VR-шлема) или `[on-device]` (нужен реальный Quest 2/3).

**Контекст:** клиент `src/rob_box_quest/webxr_client/` (Phase 2.1+),
WebSocket сервер `rob_box_quest` (Phase 1.4), supervisor `avatar_supervisor`
(Phase 2, ADR-0028).

---

## Предусловия

- [ ] Бэкенд Quest Pi работает (`docker ps | grep quest` показывает
      контейнеры `caddy`, `vision_quest`, `voice_assistant`).
- [ ] PIN виден в логах: `docker logs vision_quest 2>&1 | grep -i "pin\|session"`
      (PIN ротируется при перезапуске контейнера).
- [ ] ROS 2 окружение доступно (если on-device тест): `ros2 topic list`
      на Pi показывает `/camera/camera/color/image_raw`, `/scan` и т.п.

---

## 1. Bootstrap & environment (Phase 2.1)

- [ ] Открыть `https://<quest-pi>:8443/quest` в Chromium / Quest Browser.
- [ ] Появляется **PIN overlay** (форма с 6-значным полем).
- [ ] Подсказка с горячими клавишами доступна по клавише **H** (виден
      `?`-button в правом верхнем углу).

### Loading screen

- [ ] После ввода правильного PIN и нажатия Enter **loading screen**
      появляется на ~200-500 ms пока грузятся CC0 GLB.
- [ ] В консоли браузера видны только `[quest]` логи (нет ошибок).
- [ ] `loading-screen__spinner` крутится (CSS animation `loading-screen-spin`).
- [ ] Если GLB не загрузился (например, отключить интернет) — текст
      становится красным, спиннер останавливается (`loading-screen--failed`).

### WebXR auto-entry `[on-device]`

- [ ] На Quest 2/3: после PIN submit клиент СРАЗУ запрашивает immersive-vr
      session (виден VR-оверлей).
- [ ] Если нажать «отмена» в system-prompt Quest Browser — клиент
      остаётся в desktop fallback (2D рендер).

### Desktop fallback `[headless]`

- [ ] В Chromium dev-tools (без WebXR) клиент НЕ запрашивает VR session.
- [ ] В консоли: `[xr] isSupported('immersive-vr') → false` (или similar).
- [ ] Отображается 3D сцена с procedural fallback floor (если GLB упали).

---

## 2. Default layout (4 panels)

- [ ] После успешного bootstrap видно **4 panels** в виде полукруга:
      -60°, -20°, +20°, +60° (от направления "вперёд").
- [ ] Каждый panel имеет default topic:
      - `camera_rear` (-60°),
      - `camera_oak_color` (-20°),
      - `camera_oak_depth` (+20°),
      - `camera_ceiling` (+60°).
- [ ] Panels повёрнуты лицом к пользователю (facing = -position direction).
- [ ] Camera на позиции `(0, 1.6, 0)`, смотрит "вперёд" (к main screen).
- [ ] Main screen (большая стена впереди) показывает `camera_rear` (front).

### WSS status

- [ ] Status badge слева вверху: `CONNECTING…` → `CONNECTED` (зелёный).
- [ ] При успешном WELCOME error overlay НЕ появляется.
- [ ] Если WSS недоступен: status → `CLOSED`, error overlay показывает
      `Disconnected / Connection closed by server` через 5 секунд.

---

## 3. Teleop (WebXR)

- [ ] `[on-device]` Левый стик вперёд → робот едет вперёд (в ROS видно
      `/cmd_vel` со скоростью ~0.5 м/с).
- [ ] `[on-device]` Левый стик влево/вправо → strafe (в ROS `linear.y`).
- [ ] `[on-device]` Правый стик click → ARM (на стене загорается "ARM").
- [ ] `[on-device]` Повторный click → DISARM (HUD отражает).
- [ ] `[on-device]` B или Y → emergency stop (робот тормозит, в ROS
      `/safety/emergency_stop`).
- [ ] `[on-device]` Boost (Space в desktop) ускоряет teleop ×1.5.

### Deadman

- [ ] Если отпустить arm (DISARM) — робот останавливается через
      ≤ 200 ms (deadman fail-safe).
- [ ] `teleop_twist` уходит с `deadman: true` каждые 33 ms (30 Hz).

### Desktop fallback `[headless]`

- [ ] W → движение вперёд.
- [ ] S → назад.
- [ ] A/D → strafe.
- [ ] Space → boost (×1.5).
- [ ] E → emergency stop.

---

## 4. Voice PTT (WebXR)

- [ ] `[on-device]` Левый grip зажат → в ROS `/audio/quest_in` идёт
      PCM 16 kHz (можно проверить `ros2 topic echo /audio/quest_in`).
- [ ] В логах Quest клиента: `voice_ptt_start{mode: "radio"}`.
- [ ] `[on-device]` Правый grip → `voice_ptt_start{mode: "robot_voice"}`.
      ДО этого клиент шлёт `voice_mode{mode: "ttts_proxy"}`.
- [ ] На роботе supervisor переключает `voice_input_mode` (ADR-0028 §5).
- [ ] STT → LLM → TTS: голос оператора транскрибируется, LLM отвечает,
      TTS говорит ответом голосом робота.
- [ ] Если оба grip отпущены — клиент шлёт `voice_ptt_stop`.
- [ ] `voice_mode` ack приходит (`voice_mode_ack{mode: "ttts_proxy"}`).

---

## 5. UX overlays (Phase 2.3)

### Loading screen (повторно)

- [ ] Hard-refresh страницы (`Ctrl+Shift+R`) — overlay появляется
      минимум на ~250 ms (даже если ассеты в кеше браузера).
- [ ] Нет мигания overlay (минимальный порог `minVisibleMs=250`).

### Error overlay

- [ ] Симулировать WS разрыв: `docker restart vision_quest`.
- [ ] Status badge → `RECONNECTING…` (жёлтый).
- [ ] Через 5 секунд → error overlay появляется сверху по центру:
      `Connection lost / No response from server for 5s`.
- [ ] Detail обновляется каждую секунду.
- [ ] Когда WSS восстановится — overlay исчезает автоматически.
- [ ] Если нажать `×` на overlay — закрывается, но через 5s нового
      disconnect появится снова.

### Help overlay

- [ ] Нажать **H** в desktop mode → появляется список hotkeys.
- [ ] Ещё раз **H** → скрывается.
- [ ] Кликнуть по `?`-кнопке в HUD → overlay открывается.
- [ ] **Esc** → закрывает overlay.
- [ ] Клик вне карточки (на затемнённый фон) → закрывается.
- [ ] Если фокус в PIN-input (на старте) — нажатие **H** НЕ открывает
      overlay (а печатает в input).
- [ ] Hotkeys сгруппированы: Desktop, WebXR, Global.
- [ ] Каждая клавиша отображается в `<kbd>` элементе.

### Mode manager (внутреннее)

- [ ] Открыть dev-tools, набрать в консоли:
      ```js
      // Доступ к mode_manager — нет публичного API, но можно проверить
      // через HUD arm-state на стене: при DISARM — "DISARM", при ARM — "ARM".
      ```
- [ ] При arm toggle в HUD: `"ARM"` ↔ `"DISARM"` отражает реальный стейт.
- [ ] При grip → voice: в логах клиента `voice_ptt_start{mode: "..."}`.

---

## 6. Tooltips

- [ ] Hover на status badge (desktop mode) — нативный tooltip
      "WebSocket connection to robot".
- [ ] Hover на `?`-кнопке — "Показать / скрыть горячие клавиши (H)".
- [ ] Tooltips видны на мобильных через long-press (нативное поведение).

---

## 7. CI green

- [ ] `npm run typecheck` — exit 0, no errors.
- [ ] `npm test` — 142/142 PASS.
- [ ] `npm run gltf:verify` — 6/6 assets compliant.
- [ ] `npm run build` — exit 0, размер JS ≤ 600 KB (warning), CSS ≤ 10 KB.
- [ ] PR `z-frontend/<card-id>-phase2-integration` → `feature/avatar`.
- [ ] CI workflow `G: WebXR glTF Pipeline Verify` зелёный на PR.

---

## 8. Manual rejection criteria

PR блокируется, если:

- [ ] Любой из шагов выше failed (e2e или unit).
- [ ] FPS на Quest 2 < 30 в любой момент сессии (замерить через
      `chrome://inspect` или OVR Metrics Tool).
- [ ] Console errors (только warnings от Quest Browser не считаются).
- [ ] CI workflows красные.
- [ ] В `main.ts` остались закомментированные блоки кода (cleanup).

---

## 9. Связанные документы

- [captain-bridge.md](../architecture/captain-bridge.md) — architecture overview
- [meta-quest-api.md](../architecture/meta-quest-api.md) — WSS protocol контракт
- [ADR-0027](../adr/0027-meta-quest-webxr-ar-control.md) — Meta Quest AR control
- [ADR-0028](../adr/0028-avatar-supervisor.md) — Avatar Supervisor FSM
- [ADR-0032](../adr/0032-meta-quest-webxr-stack-and-assets.md) — Stack + assets
- [WebXR best practices research](../research/2026-08-26-meta-quest-webxr-best-practices.md)
