# BRIEF для воркера — Scheduler segments / MERGE в `dialogue_node`

> **Это стартовый пакет.** Прочитай его целиком ДО первой правки кода.
> Он самодостаточен: контекста прошлой сессии у тебя нет и не нужно.

**Задача:** `docs/plans/2026-08-28-scheduler-segments-merge-plan.md` — фазы S1…S8.
**Дизайн:** `docs/design/SCHEDULER_DESIGN.md` (§2, §4, §4.7, §6.5, §7).
**Правила репо:** `AGENTS.md` — прочитай, они сильнее твоей «оптимизации под пользователя».

---

## 1. Правила, которые нарушают чаще всего

Из `AGENTS.md` (ADR-0018, «Честный FAIL лучше красивого PASS»):

- **Не пиши «проверил, работает», если не прогнал.** Прикладывай raw-вывод `pytest -v`.
- **Не ставь `e2e-done` без реального e2e.** E2E тут прогоняется на железе, не на твоей машине.
- **Не мёржи PR.** Мёржит только товарищ Шифу.
- **Не фикси попутные баги руками** — заводи отдельную задачу.
- Обращения в диалоге: владелец — **товарищ Шифу**, старший воркер — **шисюн**, младший — **шиди**.
- Если что-то не готово — скажи «не готово, осталось: …». Это правильный ответ, а не провал.

---

## 2. Окружение: что реально запускается на этой машине

Проверено прогонами 2026-08-28 на Windows-воркстейшене. **Не гадай — вот факты.**

### ✅ Запускается локально

Чистый Python, без ROS. Все команды — **из каталога `src/rob_box_voice`**:

```bash
cd src/rob_box_voice && PYTHONUTF8=1 python -m pytest test/test_task_scheduler.py test/test_tool_executor.py test/test_speculative_pre_gen.py test/test_segment_estimator.py test/test_scheduler_event_bus.py -q
```

Проверенный результат на `develop` (`610bf10e`): **80 passed**. Это твой зелёный ориентир для фаз S2–S6.

Тесты `dialogue_node` с замоканным rclpy (`test/unit/node/conftest.py` подменяет ROS2 в `sys.modules`):

```bash
cd src/rob_box_voice && PYTHONUTF8=1 python -m pytest test/unit/node -q
```

### ❌ НЕ запускается локально

- `test/test_dialogue_shell.py` → `ModuleNotFoundError: No module named 'rcl_interfaces'`. Нужен настоящий ROS 2 Humble.
- `test/unit/sound/` → `ImportError: cannot import name 'DurabilityPolicy'` (частичный stub rclpy).
- `test/unit/core/test_voice_settings.py` → collection error `KeyError: ~TContext`.
- Любой e2e из `docker/vision/test/scenario_runner/` — только на роботе.

**Вывод для планирования:** новые тесты на `dialogue_node` пиши в `test/unit/node/`,
а **не** в `test/test_dialogue_shell.py`. Готовый образец ровно такой фичи —
`test/unit/node/test_speech_backlog_accumulator.py` (аккумулятор приземлялся так же).

### ⚠️ Две ловушки окружения

1. **`PYTHONUTF8=1` обязателен.** Без него часть тестов падает с
   `UnicodeDecodeError: 'charmap' codec` — они делают `Path.read_text()` без
   `encoding=`, а Windows-локаль cp1252 давится кириллицей. Это не твой баг.
2. **Не запускай pytest из корня репо.** Корневой `pytest.ini` жёстко задаёт
   `testpaths = src/rob_box_harness/test`, `--cov=rob_box_harness` и
   `--cov-fail-under=85` → прогон voice-тестов оттуда упадёт по coverage,
   даже если все тесты зелёные.

---

## 3. Красная базовая линия — ЭТО НЕ ТЫ СЛОМАЛ

Прогон `develop` (`610bf10e`) **до любых правок**:

```bash
cd src/rob_box_voice && PYTHONUTF8=1 python -m pytest test/unit -q \
  --ignore=test/unit/core/test_voice_settings.py --ignore=test/unit/sound
```

```
50 failed, 1408 passed, 11 skipped, 39 warnings in 57.87s
```

Раскладка падений по файлам:

| Падений | Файл | Похоже на |
|---|---|---|
| 28 | `test/unit/skills/test_music_skill_generated_music.py` | `generate_music` отключён 20.08 (`mcp_server.py:704`), тесты не обновлены |
| 7 | `test/unit/test_issue_1392_v3_tool_order.py` | то же — порядок тулов с `generate_music` |
| 4 | `test/unit/test_issue_1219_set_voice_rule.py` | ассерты на текст промпта, промпт разъехался |
| 2 | `test/unit/node/test_dialogue_node.py` | `AttributeError: '_active_tg_chat_id'` — тест строит неполную ноду |
| 2 | `test/unit/node/test_startup_greeting_flow.py` | — |
| 2 | `test/unit/core/test_music_runtime_assets.py` | — |
| 1 | `test/unit/node/test_command_intent_gate.py` | — |
| 1 | `test/unit/node/test_dialogue_node_imports.py` | — |
| 1 | `test/unit/node/test_faq_event_mode.py` | — |
| 1 | `test/unit/tts/test_provider_chain.py` | — |
| 1 | `test/unit/tts/test_yandex_chunking.py` | — |

**Что с этим делать:**

- **НЕ чинить.** Это вне твоей задачи (`AGENTS.md`: не фиксить попутное руками).
- **Зафиксировать до старта:** прогони команду выше сам, сохрани вывод. Твой
  критерий — «было 50 упавших, стало 50 тех же». Любой **новый** упавший — твой.
- Если считаешь, что какое-то падение стоит починить — заведи **отдельную**
  задачу, не тащи в этот PR.
- ⚠️ Два падения (`test_dialogue_node.py`) — в том самом файле, который ты
  будешь трогать. Не спутай их со своими.

---

## 4. Что уже сделано и трогать НЕ надо

`SCHEDULER_DESIGN.md §11.6` фиксирует: фазы 1–4 приземлены, W7a–c приземлены.
Проверено по коду:

| Компонент | Файл | Не трогать |
|---|---|---|
| `TaskScheduler` + FIFO-каналы | `scheduler/task_scheduler.py` | расширяем, не переписываем |
| `SchedulerToolExecutor` | `scheduler/tool_executor.py` | расширяем |
| EventBus | `scheduler/event_bus.py` | ✅ готов |
| Reflex-слой | `scheduler/reflex.py` | ✅ готов, не наша тема |
| Эстиматоры + pre-gen | `estimator.py`, `quality.py`, `pre_gen.py`, `speculative_executor.py` | ✅ готовы |
| Acceptance-гейт | `rob_box_harness/core/acceptance.py` | ❌ **не наша фаза** |
| `/harness/task_events` publisher | `dialogue_node.py:415` | ✅ есть, переиспользуй |
| `[ACTIVE TASKS]` блок | `tool_executor.py:223` | рядом добавляем `[SEGMENT PLAN]` |

**Не перепутай:** `scheduler/decision.py` (`DecisionCoordinator`, `DecisionPlan`) —
это контракт «планировщик ↔ исполнитель шагов», **не** классификатор
MERGE/REPLACE. Он существует и работает; свой `quick_decide` пиши отдельным
модулем, не пытайся встроиться в него.

---

## 5. Три вещи, на которых ты почти наверняка споткнёшься

### R1 — `_cancel_run` делает четыре дела разом

`dialogue_node.py:4555-4566`:
```python
self._run_cancelled = True
task.cancel()                          # отменить LLM-турн
self._tts_control_pub.publish("STOP")  # заглушить TTS      ← это надо убрать
self._effects.release_all_tts()        # отпустить awaiter'ы ← это НАДО оставить
self._effects.clear_sound_event()
```

Если наивно убрать только `STOP`, не разделив функцию, — `speak_helpers._tts_events`
(`core/speak_helpers.py:292,320`) залипнут, и робот замолкнет **навсегда**.
Тест «awaiter'ы отпущены, STOP не отправлен» — обязателен (Task 1.2).

### R2 — `update()` гоняется с `_pump`

`_Channel.remove` (`task_scheduler.py:359`) перестраивает очередь из снапшота.
Между снятием снапшота и заливкой `_pump` (`:319`) может забрать голову.
Итог — сегмент исполнится дважды или потеряется. Конкурентный тест обязателен (Task 3.2).

### R3 — музыкальные стартеры вне планировщика

`tool_executor.py:49-52`: `execute_music_code` / `set_vibe_preset` / `load_track`
**намеренно** обходят scheduler («party regression, live 19.08» — LLM должна
видеть настоящий результат, иначе DJ-сет разваливается).

Значит MERGE работает по **вокалу** (`speak_text`), но не по биту. Это осознанное
ограничение, а не баг. **Не чини это в рамках задачи.**

---

## 6. Порядок работы

```
S1  флаг barge_in_policy + разделить _cancel_run   ← начни отсюда, эффект наибольший
S2  group_id / seg_idx у SchedulerTask
S3  TaskScheduler.update() + инвариант §2.3
S4  quick_decide (правила; второй LLM НЕТ — ревизия v5)
S5  [SEGMENT PLAN] в контекст LLM
S6  MCP-тул task_delta
S7  очередь pending user-messages
S8  e2e на железе  ← ТЫ ЕГО НЕ ПРОГОНИШЬ, готовь и передавай Шифу
```

- Каждая фаза = **отдельный PR** (ADR-0013).
- Всё под флагом `barge_in_policy`, дефолт `replace` = сегодняшнее поведение.
- Порядок внутри задачи: **сначала тест, потом код** (см. формат плана).
- S9–S12 в этот заход не берём.

**Где остановиться:** доведи S1–S7 до зелёных unit-тестов и открой PR.
S8 (e2e) требует железа — подготовь сценарий и явно скажи: «e2e не прогонял,
нужен стенд». Это правильный ответ, а не отговорка.

---

## 7. Чек-лист перед словом «готово»

Из `AGENTS.md` («минимальный контракт воркера»):

- [ ] Приложил raw-вывод `pytest` — с командой и полным хвостом.
- [ ] Показал базовую линию ДО правок и результат ПОСЛЕ (было 50 упавших → стало 50 тех же).
- [ ] Указал конкретные `файл:строка`, а не «поправил».
- [ ] `e2e-done` НЕ ставил — e2e на этой машине не прогоняется.
- [ ] Проверил, что `barge_in_policy=replace` полностью воспроизводит старое поведение (регресс).
- [ ] Не мёржил PR.
