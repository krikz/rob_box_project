# ADR-0072: убрать плитку VOICE с главного экрана мостика (single-source-of-truth через voice_pipeline_panel)

- **Status:** proposed
- **Date:** 2026-09-08
- **Issue:** [#2141](https://github.com/krikz/rob_box_project/issues/2141)
- **Related:** [#2138](https://github.com/krikz/rob_box_project/issues/2138) (выбор голоса не доезжает по проводу)
- **Author:** architect
- **Component:** rob_box_quest / webxr_client

## Контекст

В сцене сейчас **две точки входа** в один и тот же TTS picker:

1. **Launcher на основном экране мостика** — `tts_picker_menu.ts:188-203` (плитка `▶ VOICE`, всегда видна, всегда кликабельна). Регистрируется в `captain_bridge.ts:949-953, 956-959`, обрабатывается как `tts:launch` → `ttsPicker.show(...)`.
2. **Кнопка в голосовой панели слева** — `voice_pipeline_panel.ts:43` (`TTS_TARGET_ID = "vpl:tts"`) → та же `handleTtsTarget({ kind: "launch" })` в `main.ts:760+`.

Оба пути открывают **одно** и то же меню. Сам picker вызывает `set_voice` по проводу, в нём `voice_id: ""` — это баг #2138 (отдельная карточка, backend-воркер).

На главном экране плитка занимает место **между картой лидара и видеостеной** — то есть место, ценно для пилотирования, а голосовой выбор там не к месту. Владелец в шлеме 2026-09-08: «нахуя она там нужна?»

## Решение

**Удалить дубль.** Канонический путь к TTS picker'у — кнопка `vpl:tts` в `voice_pipeline_panel`, рядом со STT/LLM/пресетами/языком. Это та же группа настроек голоса, и там же оператор уже находится, когда хочет что-то поменять. Launcher на основном экране — лишний.

## Trade-offs

| Аспект | Оставляем launcher | Убираем (выбрано) |
|---|---|---|
| Дублирование входа | есть — два клика открывают одно меню | нет |
| Место на основном экране | занято в зоне пилотирования | свободно для карты/видео |
| Время внесения правки | 0 | ~30 мин |
| Зависимость от #2138 | нет (сам launcher работает, меню внутри — нет) | нет (то же) |
| Регресс при поиске «где настройки голоса» | — | придётся обновить 0 строк документации: кнопка уже описана в `voice_pipeline_panel.ts:11` |

## Что меняется в коде

1. `tts_picker_menu.ts:188-203` — убрать `launchGroup`, `launchTile`, `drawLaunch`, `LAUNCH_TARGET_ID` в `launchTarget()` / `launchObject` / dispose. Если `LAUNCH_TARGET_ID` и `parseTtsTargetId("launch")` больше никто не использует — удалить и их (`state/tts_picker_state.ts:301, 315`).
2. `captain_bridge.ts:949-959` — убрать `scene.add(ttsPicker.launchObject)` и регистрацию цели `tts:launch`. Вызов `ttsPicker.show()` остаётся — он нужен из `voice_pipeline_panel` через ту же `handleTtsTarget`.
3. **Ничего не трогать** в `voice_pipeline_panel.ts` и `main.ts` — путь через `vpl:tts` уже работает и остаётся единственным.
4. Никаких правок в `ws_server.py` / `supervisor_node.py` / `tts_voice_registry.py` — это контракт по #2138, не наша задача.

## Когда включать обратно

Если когда-нибудь понадобится быстрый доступ к picker'у с основного экрана (вне голосовой панели) — добавить как **отдельную** карточку с обоснованием. До тех пор `voice_pipeline_panel` остаётся единственным entry point.

## Что не входит

- Само меню picker'а и его UI — не трогаем.
- Правки клиент→сервер по `voice_id` — это #2138.
- Пресеты речи / язык — работают, не трогаем.

## Определение готовности (для воркера)

- [ ] `git grep -n 'launchObject\|launchTile\|drawLaunch\|tts:launch\|LAUNCH_TARGET_ID\|launchTarget' src/rob_box_quest/webxr_client/src` — пусто
- [ ] `git grep -n '"launch"' src/rob_box_quest/webxr_client/src/state/tts_picker_state.ts` — пусто
- [ ] CI зелёный (`L-Build Vision Pi Services` на runner-8, `Unit Tests (ROS2 Humble)`, `Python Code Quality`, `Lint`, `Shell`, `YAML`)
- [ ] В шлеме на главном экране мостика плитки `▶ VOICE` нет (raw-скриншот через e2e-process)
- [ ] Клик по `vpl:tts` в голосовой панели слева открывает picker как и раньше
- [ ] PR в `develop`, `Closes #2141` в description, ожидает мержа от товарища Шифу

## Hotspot

`src/rob_box_quest/webxr_client/src/scene/tts_picker_menu.ts` и `scene/captain_bridge.ts` — оба файла пересекаются с другими активными карточками (#2138 backend, #2129 process, #2118 lint-strict). Если воркер собирает диф больше ~30 строк — это сигнал, что он залез не туда. Правка должна быть **минимальной**: убрать launcher, оставить picker и его цели.

## Альтернативы (рассмотрены и отклонены)

- **B. Перенести launcher в voice_pipeline_panel как «отдельную вкладку».** Это та же операция, что A, только словами «вкладка» вместо «убрать». B не меняет сути — мы всё равно оставляем единственную точку входа. A проще сформулировать.
- **C. Оставить launcher + добавить badge «не работает до #2138».** Дизайн-clutter, врёт оператору («выглядит рабочим» — ровно то, что владелец и просил убрать).