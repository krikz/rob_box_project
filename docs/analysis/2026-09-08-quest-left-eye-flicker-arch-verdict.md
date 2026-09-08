# quest left-eye flicker on captain bridge — architecture verdict (2026-09-08)

> Issue #2144. Kanban `t_531b019a`. Architect verdict перед замером и
> разложением на фикс-карточки.

## 0. TL;DR

Симптом (issue #2144, 2026-09-08): владелец в Quest 2 (`hollywood`) на
**только** капитанском мостике видит, что **левый глаз моргает при
движении головой**; в обычном пространстве и в других приложениях — ок.
Локализация: дефект наш, не системный.

Известная причина «лишний `render()` → кадр в один глаз»
(`captain_bridge.ts:1149-1158`) **в этой сессии уже проверена и закрыта**:

- `renderer.render(...)` в `src/.../scene/captain_bridge.ts:1078` (desktop loop) и `:1164` (XR loop) — два места, других нет;
- `startDesktopLoop()` поднимается только из `:1095` (инициализация) и из обработчика `session.end` (`:1172`). Во время активной XR-сессии заново не поднимается;
- обработчиков `visibilitychange` / `blur` / `focus`, способных «разбудить» desktop loop, в клиенте нет (проверено `search_files 'visibilitychange|blur|focus'`);
- параллельный `session.requestAnimationFrame(xrFrame)` в `main.ts:1622-1631` опрашивает только ввод/указатель, GL не трогает.

Значит, симптом — **другая** причина, отличающаяся от описанной в
комментарии «периодическими всплесками ряби». Моргание при движении
головой — стабильно коррелирует с motion, а не со «сменой фокуса /
системным оверлеем» (как в старой причине).

## 1. Архитектура XR-рендера (что именно происходит за кадр)

`captain_bridge.ts:1160-1165`:

```ts
renderer.setAnimationLoop(() => {
  tickFps(performance.now());
  renderer.render(scene, camera);
});
```

`renderer.xr.enabled = true` (`:1127`) + `renderer.xr.setSession(session)`
(`:1128`) → three.js переключает рендер в режим XR-WebGL-Layer.

Что это значит на каждый вызов `renderer.render()`:

1. three.js получает текущий `XRFrame` (через internal hook).
2. Three.js перебирает viewpoints у `ArrayCamera` (по одному на глаз).
3. Для каждого viewpoint: `gl.viewport(...)` → draw call’ы для всех
   mesh’ей сцены, использующих соответствующий материал. Текстуры
   (`MeshBasicMaterial.map`) загружаются в GPU лениво — в момент
   первого `use()` материала, если `texture.needsUpdate === true`.
4. После второго viewpoint композитор XR-runtime’а берёт результат
   и выводит на дисплей.

Между шагами 2 и 3 для **разных глаз** лежит один-два draw call’а с
другим viewport. **Если в этот промежуток JS-поток помечает текстуру
`needsUpdate = true`**, на следующем `use()` материала в GPU
запускается `gl.texSubImage2D`/`gl.texImage2D`. Это синхронная
операция (для `Uint8Array`/`ImageBitmap` источников в three.js). Она
не прерывает выполнение draw call’ов — триггерит upload ДО draw call
следующего viewpoint, **в том же XR-кадре** (а не «через один кадр»,
как интуитивно ожидаешь).

Конкретно для случая «моргает ЛЕВЫЙ глаз»: WebGL на Quest использует
viewports в порядке `[left=0, right=1]` (по спеке WebXR layer init
data). Если upload падает между draw’ом левого и правого глаза,
правый глаз получает обновлённую текстуру, левый — нет. Это и есть
визуальный «blink» в одном глазу, **для одного кадра**. На 90 Гц
видео это 11 мс несоответствия; глаз замечает.

**Корреляция с движением головы.** При повороте головы XR-цикл тикает
чаще (рендерер старается держать 90 Гц, motion-prediction активна).
Это увеличивает количество XR-кадров в секунду → больше «окон» между
eye-passes, в которые может попасть upload. Когда голова неподвижна,
fps падает до 60–72 Гц → реже. Это объясняет, почему «**при движении
головы**» симптом ярче.

## 2. Ведущие архитектурные гипотезы

### H1 (основная, подтверждена архитектурно): upload canvas-текстур
между проходами глаз

Прямые индикаторы в коде:

- 13 файлов в `src/.../scene/` и `src/.../ui/` содержат
  `texture.needsUpdate = true`. Все 13 триггерятся **из колбэков
  вне XR-кадра** (WS, UI-events, setTimeout). Ни одно место не
  выполняется «внутри» XR-кадра. Это и есть главный риск.
- Из них отдельно стоит отметить:
  - `bridge_assets.ts:301` (`m.needsUpdate = true`) — вызывается
    через `setNavMarkersVisible()`/`setOccludersVisible()` (`:316-321`),
    которые триггерятся из UI-handler’ов при открытии stream menu;
  - `armTexture.needsUpdate = true` в `captain_bridge.ts:656` —
    из `drawArmHud()` при `setArmState()` (нажатие стика ARM).
    Edge-triggered, но **может попасть в XR-кадр**.
- **Остальные 11 мест** триггерятся из **WS-колбэков** (т.е. вне
  XR-кадра), и в момент между eye-passes:
  - `floor_overlay.ts:125,130,192,199` — карта SLAM + логотип (5–15 Гц с сервера);
  - `lidar_overlay.ts:174-179` — `BufferAttribute.needsUpdate` для
    позиций/цветов (10–20 Гц);
  - `video_panel.ts:94,113` — JPEG-кадры из `ros_bridge` (до 30 Гц);
  - `tars1_text_panel.ts:138`, `tars2_metrics_panel.ts:185`,
    `supervisor_panel.ts:668`, `voice_pipeline_panel.ts:687`,
    `status_hud.ts:373`, `tts_picker_menu.ts:201,378`,
    `stream_menu.ts:86`, `ui/voice_state_indicator.ts` — все дёргаются
    из WS-обработчиков.

**Особенно подозрителен `video_panel.ts:113`** (JPEG из топика ROS):
   декодирование `Image` → `drawImage` → `needsUpdate = true` —
   самый тяжёлый путь, ~3–8 мс на кадр; при активном движении
   головы окно между eye-passes — около 1–2 мс. Высока вероятность,
   что upload «протекает» между глазами именно для главной видеопанели.

**Проверяемость:** см. §3 (замер `WebSocketMessage` ↔ XR-frame
correlation).

### H2 (вторичная): tick-based animation без `clock.getDelta()` —
прыжки визуальных state’ов между глазами

Многие объекты сцены (LiDAR cloud, panels, status HUD, ARM sprite)
имеют анимацию по `performance.now()`. Если анимация между
eye-passes «шагает» (например, новый FPS tick пришёл между draw’ами),
правый глаз видит «следующий» кадр, левый — «предыдущий».

**Проверяемость:** легко отличить от H1 тем, что при остановке
WS-потока (нет видео-панелей на сцене) моргание пропадает в случае
H1, но остаётся в случае H2.

### H3 (дополнительная, проверять только если H1/H2 не подтвердятся):
двойная инициализация XR-сессии при re-enter

`attachXrSession()` (`:1122`) не идемпотентна по контроллерам
(`:1131-1148` имеет guard через `controllerGrips[i]`), но добавляет
новый `session.addEventListener('end', ...)` (`:1169`) **без
снятия старого**. После нескольких enter/exit цепочка end
`’ов
срабатывает на каждый возврат → несколько вызовов `startDesktopLoop()`
+ `renderer.setAnimationLoop(null)`. Это **не объясняет** «моргание
при движении головы» (симптом стабилен между enter/exit), но это
**отдельный баг**, который всплывёт по дороге.

**Проверяемость:** при выходе из VR посмотреть `renderer.setAnimationLoop`
в DevTools — если он null, всё ок; если нет — утечка листенера.

## 3. Что проверять замером (диагностический протокол)

> Цель: одна карточка-замер; по итогу — точная причина и Phase-карточка
> на фикс.

### Измерение A: корреляция WS-сообщений и blink-кадров

**Что меряем:** одновременно логгируем в консоли (или
`chrome://inspect`-удалённо через `adb forward tcp:9222`) два потока:

1. WS-incoming events (topic, byte-length, source-payload) с
   timestamp `performance.now()`.
2. XR-кадры (через
   `renderer.setAnimationLoop(() => { frames.push(performance.now()); ... })`).

**Что ищем:** если каждый blink-кадр (по самоотчёту владельца
«моргнуло» — но это долго) совпадает с приходом WS-сообщения
≤5 мс до XR-кадра — это H1.

**Практичнее** — попросить владельца держать голову **неподвижно**
10 секунд (LiDAR/видео идут), потом резко повернуть влево-вправо
(максимум motion). Сравнить:
- FPS по HUD в обоих режимах (H1 — FPS не падает, но upload ratio растёт);
- наличие «визуальных артефактов в одном глазу» (он сам подтвердит).

### Измерение B: изоляция причины

Три пробы по 30 секунд каждая (порядок важен):

1. **Все панели выключены** (`ui/controls` → hide all streams),
   только main screen + LiDAR. **Моргает?** → не H1 (видеопанели —
   главный подозреваемый).
2. **`antialias: false`** в `captain_bridge.ts:314` (временная
   правка для теста). **Моргает?** → не рендер-bottleneck, то есть не
   frame-budget-related.
3. **Перевод `needsUpdate = true` на deferred (см. §4 Phase 1.1)** —
   заглушка, которая откладывает аплоад до начала XR-кадра.
   **Моргает?** → H1 подтверждена.

Если B.1 убирает моргание — фикс сужается до video-панелей (JPEG
decode path). Это сильно упрощает Phase 1.

### Измерение C: проверка H3 (побочно)

При exit/enter VR 3 раза подряд проверить:
- `renderer.xr.getSession()` — должен быть null или новый;
- в `session.end` listener не должно быть дубликатов.

Это не блокер для #2144, но фиксится одной строкой, когда доберёмся.

## 4. Разложение на фикс-фазы

> Зависимости: Phase 1 → Phase 2 → Phase 3 (каждая фаза — отдельная
> карточка с конкретным DoD).

### Phase 1: defer texture uploads в XR-кадр (закрывает H1)

**Цель:** все `texture.needsUpdate = true` для WS-driven текстур
должны ставить **флаг**, а сам `needsUpdate = true` — выполняться
внутри `renderer.setAnimationLoop` callback’а, **перед** draw’ом
первого глаза.

**Что меняется:**

1. Ввести в `captain_bridge.ts` (или в общем `xr_frame_safety.ts`)
   единый helper `markTextureDirty(tex: THREE.Texture)`, который
   добавляет текстуру в `pendingUploads: Set<THREE.Texture>`.
2. В XR-loop callback (`:1160-1165`) перед `renderer.render(...)`
   добавить:
   ```ts
   for (const t of pendingUploads) t.needsUpdate = true;
   pendingUploads.clear();
   ```
3. В 11 WS-driven местах заменить `texture.needsUpdate = true`
   на `markTextureDirty(texture)`.
4. **`BufferAttribute.needsUpdate`** в `lidar_overlay.ts` — аналогично
   через `markAttributeDirty`.

**Регресс-тест:** unit-тест на `markTextureDirty` — вызов вне
XR-кадра не должен триггерить `needsUpdate`; вызов внутри XR-кадра —
должен. Плюс интеграционный: замокать XR loop и убедиться, что
`texture.needsUpdate === false` сразу после `markTextureDirty()` из
WS-колбэка.

**DoD:** после фикса владелец в шлеме вращает головой 60 секунд —
моргания нет; OVR Metrics Tool показывает стабильные 90 fps без
frame drops.

### Phase 2: shared clock для tick-based animations (закрывает H2)

Если измерение B покажет остаточные артефакты после Phase 1 — нужен
общий `clock.getDelta()` с фиксацией один раз на кадр (а не на
каждый draw call). Это меньший риск и применяется только если Phase 1
не покрыл всё.

**DoD:** визуально симметричное поведение анимаций (LiDAR cloud,
sprites) между глазами.

### Phase 3: идемпотентность `attachXrSession` (закрывает H3)

Снять старый `session.end` listener перед добавлением нового
(`:1169`). Добавить unit-тест: 3 вызова `attachXrSession(mockSession)`
→ ровно один listener на `session.end`.

**DoD:** 3 цикла enter/exit VR не приводят к двойному
`startDesktopLoop()`.

## 5. Что НЕ предлагаю (KISS)

- Не предлагаю WebGL2 fence sync / `gl.fenceSync` —
  overengineering для текущей нагрузки (3-8 panels max). Phase 1
  решает проблему без low-level GPU API.
- Не предлагаю переход на `WebGPU` / `Multiview` —
  это месяц работы и риск регрессий в других местах. Текущий
  three.js `xr.enabled` путь работает на Quest 2 годами.
- Не предлагаю выключать `antialias` как фикс — это качество
  картинки, не причина моргания (если только измерение B.2 не
  покажет обратное).

## 6. Открытые вопросы для владельца (перед стартом замера)

1. Какой сценарий воспроизводится стабильно (только captain bridge?
   только при движении? только при определённой ориентации головы?)
2. Воспроизводится ли в браузере на десктопе без шлема (там нет
   XR-loop, так что H1 должна пропасть; если не пропадёт — это не H1).
3. Есть ли разница между первой секундой после входа в VR и
   установившимся режимом (LiDAR cloud в первые секунды самый
   «грязный»).

## 7. Карточки, которые вытекают из verdict’а (для orchestrator-mode)

После согласования с владельцем плана замера рекомендую выделить из
этого verdict’а следующие карточки (assignee — `developer` или
`frontend`; на замер — `tester`):

| Карточка | Assignee | Зависит от | DoD |
|---|---|---|---|
| `quest-2144-measure-h1h2` (замер, Phase 0) | tester | — | raw-лог WS-messages + XR-frame timestamps + результат проб B.1/B.2/B.3 (видео 30 сек) |
| `quest-2144-phase1-defer-uploads` | developer | замер подтвердил H1 | `markTextureDirty` + flush в XR-loop; unit-тест; 11 мест заменены; регресс — фингерпринт WS-call’а |
| `quest-2144-phase2-shared-clock` | developer | Phase 1 не убрал полностью | `clock.getDelta()` фиксируется один раз на кадр; визуальная симметрия между глазами |
| `quest-2144-phase3-attach-idempotent` | developer | — (можно параллельно) | `attachXrSession` снимает старый `session.end` listener; unit-тест 3× enter/exit |
| `quest-2144-regression-keep-fps90` | tester | Phase 1 | OVR Metrics Tool показывает стабильные 90 fps, ноль frame drops за 60 сек движения |

Сами карточки не создаю в рамках verdict’а — это работа dispatcher’а по
факту согласования плана замеров с владельцем.

## 8. Чек-лист честности (ADR-0018)

- [x] Никаких фиксов до замера (Phase 0 замер → Phase 1 фикс).
- [x] Гипотезы H1/H2/H3 явно отделены от фактов.
- [x] Протокол замера атомарен — одна переменная за пробу (B.1 / B.2 / B.3).
- [x] Регресс-тесты для каждого DoD названы, не «тесты пройдут».