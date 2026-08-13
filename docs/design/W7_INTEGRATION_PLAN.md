# W7 — План интеграции scheduler в dialogue_node (issue #968)

Статус: **ПЛАН (готов к исполнению developer/backend)**
Версия: 1.0 (2026-08-13)
Автор: architect (kanban t_6a4d501b)
Связано: issue #968, `docs/design/SCHEDULER_DESIGN.md` (v6, PR #1180), `docs/design/CHILD_TASKS_PROPOSAL.md`

---

## 0. Зачем этот документ

Issue #968 переоткрыт пользователем: целевой сценарий «комар + енот» **не работает**
(комментарий 2026-08-13: «Он сейчас не работает!!»). Дизайн-документ доведён до v6
и смержен (PR #1180), фазы 1–4 scheduler'а **приземлены в develop как библиотека**,
но **W7 (интеграция в живой путь dialogue_node) не выполнена** — поэтому поведение
на железе не изменилось: LLM по-прежнему вызывает инструменты fire-and-forget.

Этот документ — код-точный план W7: что где врезать, какие костыли снять, как
проверить. После его реализации целевой сценарий из issue должен работать.

---

## 1. Диагноз (проверено по коду develop, 2026-08-13)

### 1.1 Что уже приземлено (фазы 1–4)

| Компонент | Файл | Статус |
|-----------|------|--------|
| TaskScheduler (FIFO voice/music/anim, submit/cancel/wait_all/channel_status) | `src/rob_box_voice/rob_box_voice/scheduler/task_scheduler.py` | ✅ есть (commit `83947d0f`, `9026b3ec`) |
| AcceptanceGate + PendingSegment (AWAITING_CONFIRMATION) | `src/rob_box_harness/rob_box_harness/core/acceptance.py`, `confirmation_policy.py` | ✅ есть, **встроен** в dialog_core tool loop (`74da1902`) |
| EventBus (bounded, backpressure) | `scheduler/event_bus.py` | ✅ есть (`e5e4eec2`) |
| DecisionCoordinator (two-tier decision) | `scheduler/decision.py` | ✅ есть (`ceb3ea43`) |
| ReflexLayer (STOP/direction/debounce/metrics) | `scheduler/reflex.py` | ✅ есть (`138a8f6d`, `653ab60b`) |
| SegmentEstimator / EstimatorQualityTracker / SpeculativePreGenerator / SpeculativeStepExecutor | `scheduler/estimator.py`, `quality.py`, `pre_gen.py`, `speculative_executor.py` | ✅ есть (`4efab62c`, `b1983b83`, `d7356b20`, `5c1c2141`, `806eb146`) |
| Action server (HTTP+JSON, PASTE shadow) | `src/rob_box_voice/rob_box_voice/action_server/` | ✅ есть |

Тесты: `src/rob_box_voice/test/test_task_scheduler.py` (25), `test_scheduler_event_bus.py` (6),
`test_task_scheduler_integration.py` (4) — всего 35 тестов.

### 1.2 Чего НЕТ (корень жалобы)

**W7 — подключение scheduler к живому пути не сделано:**

1. `dialogue_node.py` **не импортирует** `rob_box_voice.scheduler` нигде.
2. Живой tool loop — `DialogCore._run_with_tools` (`dialog_core.py:414–531`): каждый
   `tool_call` из ответа LLM исполняется **напрямую**:
   ```python
   # dialog_core.py:515
   tool_result = await self._tools.execute(call)
   ```
   Все tool_calls одного батча исполняются последовательно, **без ре-ордеринга
   по зависимостям** — это и есть INSIGHT #1 товарища Шифу (гонка внутри одного
   ответа LLM: `speak_text` + `stop_music` в одном батче → stop_music улетает на
   железо через миллисекунды, TTS ещё не синтезирован).
3. Костыли из #935/#980/#992 всё ещё в `dialogue_node.py`:
   `_pending_music_cleanup` (строки ~411–419, 1430–1436, 1882–1935),
   `_publish_music_cleanup`, deferred cleanup по `batch_complete` — каждый лечит
   симптом, а не корень.

### 1.3 Точка врезки (одна, общая)

Единственная точка, через которую проходят ВСЕ tool_call'ы LLM в живом пути:

```
dialogue_node._run_turn (dialogue_node.py:1750)
  → self._core.process_input (dialogue_node.py:1837)
    → DialogCore._run_with_tools (dialog_core.py:414)
      → for call in response.tool_calls:          # dialog_core.py:457
          if acceptance_gate: ...                  # dialog_core.py:480  (уже есть)
          tool_result = await self._tools.execute(call)   # dialog_core.py:515 ← ВРЕЗКА
```

Рядом с `_acceptance_gate` (уже встроен, строки 480–514) добавляется второй hook —
`task_scheduler`. AcceptanceGate решает «можно ли исполнять» (AWAITING), scheduler —
«в каком порядке и на каком канале».

---

## 2. Целевая архитектура W7

```
LLM tool_call (батч)
  → DialogCore._run_with_tools
    → W7a: ре-ордеринг батча по зависимостям (speak → [TTS done] → stop_music)
    → W7b: SchedulerToolExecutor.submit(call)
        ├─ voice-канал:  speak_text → /voice/tts/request (FIFO, строго последовательно)
        ├─ music-канал:  execute_music_code / set_vibe_preset / load_track / stop_music
        ├─ anim-канал:   play_animation
        └─ bypass:       memory_*, search_*, get_* (INSTANT, без очереди — как сейчас)
    → W7c: task_events publisher → /harness/task_events + [ACTIVE TASKS] в LLM-контекст
    → W7d: снять костыли (_pending_music_cleanup, deferred cleanup, debounce)
```

**Принцип (из issue, раздел «ИТОГОВАЯ МОДЕЛЬ»):** LLM остаётся свободной — она
шлёт tool_call'ы мгновенно (fire-and-forget сохраняется). Планировщик владеет
каналами: `stop_music`, пришедший раньше времени, встаёт в очередь и исполняется
в правильный момент; `speak_text` дополняет FIFO-очередь. Деструктивные тулзы
(`stop_music`, REPLACE) исполняются **на естественной границе**, не мгновенно.

---

## 3. Шаги W7 (в порядке исполнения)

### W7a — Ре-ордеринг tool_calls ВНУТРИ одного батча (dialog_core.py)

**Файл:** `src/rob_box_harness/rob_box_harness/core/dialog_core.py`, метод
`_run_with_tools`, блок `for call in response.tool_calls:` (строки ~457–522).

**Что сделать:** перед исполнением батча отсортировать `response.tool_calls`
по топологическому порядку зависимостей:

1. **`stop_music` / `stop_navigation` (деструктивные) — в КОНЕЦ батча.** Если в
   батче есть `speak_text`, они не должны исполняться раньше.
2. **`execute_music_code` / `set_vibe_preset` / `load_track` — перед `speak_text`**
   (музыка должна стартовать до/вместе с речью; уже есть прелюдия в дизайне).
3. Остальные тулзы — в исходном порядке (они независимы, порядок не важен).

Реализация — чистая функция `_order_tool_calls(calls) -> list`, покрытая unit-тестом:
- `[speak_text, stop_music]` → `[speak_text, stop_music]` (порядок сохранён, но
  stop_music получает флаг `defer_until_voice_drained`);
- `[execute_music_code, speak_text, stop_music]` → `[execute_music_code, speak_text, stop_music]`;
- независимые `[memory_save, speak_text]` → порядок не меняется.

**Важно:** ре-ордеринг НЕ меняет семантику для LLM — tool-результаты возвращаются
в исходном порядке (по `tool_call_id`), чтобы история сообщений осталась валидной
для OpenAI-формата.

**Acceptance:** unit-тест на `_order_tool_calls`; e2e v36-симптом (stop_music через
3с после начала рэпа) больше не воспроизводится на уровне порядка вызовов.

### W7b — SchedulerToolExecutor (новый модуль, rob_box_voice)

**Новый файл:** `src/rob_box_voice/rob_box_voice/scheduler/tool_executor.py`
(или `integration.py` — по согласованию с developer).

**Что делает:** реализует порт `ToolProvider` (`discover`/`execute` — интерфейс из
`rob_box_core/ports.py`, которым уже пользуется dialog_core через
`LLMToolCallAdapter`/`ROSMCPToolProvider`), но внутри маршрутизирует вызовы через
`TaskScheduler`:

| Tool | Канал | Executor (TaskExecutor) |
|------|-------|-------------------------|
| `speak_text` | VOICE | publish `/voice/tts/request` (как `SpeakTextTool.execute` сейчас) |
| `execute_music_code`, `set_vibe_preset`, `load_track` | MUSIC | publish `/voice/music/...` (как `MusicTool.execute`) |
| `stop_music` | MUSIC | отложенный: исполняется после drain voice-очереди (см. W7a) |
| `play_animation` | ANIM | publish `/voice/animation/request` |
| `memory_context`, `search_*`, `get_*`, `estimate_tts_duration` и пр. | bypass | прямой вызов (INSTANT, как сейчас) |

**Ключевой контракт:** `execute(call)` **не ждёт** завершения (кроме INSTANT-тулов) —
возвращает `LLMToolResult(status="queued", task_id=...)` сразу, как и требует
«LLM свободна» из итоговой модели. Реальный результат (success/error) доставляется
через feedback events (W7c) и в следующий tool-result при необходимости.

**Где врезать:** `dialogue_node.py:_build_tool_provider()` (строка ~842) — обернуть
текущий `ROSMCPToolProvider` в `SchedulerToolExecutor`; либо передать scheduler
отдельным аргументом в `DialogCore` рядом с `acceptance_gate`
(`dialog_core.py:134`, `194`).

**Acceptance:**
- unit: `speak_text` → task на voice-канале, `stop_music` → отложен до drain;
- unit: два подряд `speak_text` за < 50мс каждый (submit не блокирует);
- integration (расширить `test_task_scheduler_integration.py`): сценарий
  «комар + енот» на фейковом side-effect bus — куплет 1 звучит, MERGE переписывает
  PENDING-сегменты, куплет 2 про енота без паузы.

### W7c — Feedback events (task_events + LLM-контекст)

**Новое:** publisher `/harness/task_events` в `dialogue_node.py` (решение Q3 из v6:
publisher внутри dialogue_node, отдельный node не нужен — SCHEDULER_DESIGN §14.1).

События (формат из issue):
```
task.created(id, type, params)
task.started(id)
task.segment_started(id, seg_idx)
task.segment_completed(id, seg_idx)
task.updated(id, delta)
task.completed(id)
task.cancelled(id, reason)
```

**В LLM-контекст:** `dialogue_node._build_dynamic_system_context()`
(строка ~1624) — добавить блоки:
```
[ACTIVE TASKS]
- id=t_001, type=sing, topics=["комар"], progress=0.4,
  current="куплет 2/4", eta=45s, channels={music: playing, voice: speaking}

[PENDING EVENTS]
- battery_critical (source=hermes, priority=high): "батарея 12%..."
```
Источник: `TaskScheduler.channel_status()` + очередь событий `EventBus`.

**Acceptance:**
- события видны в `/harness/task_events` (ros2 topic echo);
- перед каждым ходом LLM в контексте есть блок `[ACTIVE TASKS]` + `[PENDING EVENTS]`;
- пост-амбл «Готово! Зачитал тебе...» исчезает при активном voice-канале
  (критерий из INSIGHT #8: LLM видит `voice: speaking` → не добавляет комментарий).

### W7d — Снять костыли

После того как scheduler владеет каналами (W7b) и есть честный сигнал
`/voice/tts/batch_complete` (уже существует, `dialogue_node.py:385`, `tts_node.py`),
удалить/выключить:

- `_pending_music_cleanup` и всю логику deferred cleanup по `batch_complete`
  (`dialogue_node.py`: ~411–419, 1430–1436, 1882–1935);
- `_publish_music_cleanup(reason=...)` как «костыль на таймере» — cleanup теперь
  делает scheduler (REPLACE/стоп на естественной границе);
- 3-секундный debounce в `_on_tts_finished` (если остался) — заменяется на
  batch_complete + канальное состояние.

**Acceptance:** grep по `dialogue_node.py`: `_pending_music_cleanup` пуст;
`music_cleanup` публикуется только из scheduler-пути.

---

## 4. Порядок и зависимости

```
W7a (ре-ордеринг батча, dialog_core)   — можно первым, изолирован, unit-тест
W7b (SchedulerToolExecutor)            — после W7a; ядро интеграции
W7c (feedback events)                  — после W7b (нужны task_id из scheduler)
W7d (снять костыли)                    — последним, после зелёного e2e
```

W7a можно делать отдельным PR (маленький, чинит INSIGHT #1 уже на уровне порядка).
W7b+W7c+W7d — один PR (интеграция).

---

## 5. Acceptance criteria (из issue #968)

- [ ] `stop_music()` не может исполниться раньше конца текущего TTS-чанка
      (порядок гарантирован планировщиком — W7a + W7b)
- [ ] Пользователь может прервать песню командой — новый сегмент доигрывает ≤4с
      и плавно затухает (REPLACE на естественной границе)
- [ ] «и ещё про енота» во время куплета 1 → куплет 2 переписывается без паузы
      и рестарта (MERGE, PENDING-only — инвариант из issue)
- [ ] Перед каждым ходом LLM в контексте есть «Сейчас исполняется/Прогресс/Каналы»
      + [PENDING EVENTS]
- [ ] Решение по новому вводу (MERGE/REPLACE/QUEUE/IGNORE/CLARIFY) < 800мс
      (уровень 1 — правила < 50мс; уровень 2 — лёгкая LLM)
- [ ] e2e-прогон: рэп 4+ чанков с музыкой до конца, stop_music строго после
      последнего чанка
- [ ] Feedback events публикуются в /harness/task_events и видны в мониторинге
- [ ] Edge case «сетевой таймаут API» — seg скипается, задача продолжается
- [ ] **Инвариант сегментов (unit):** update() модифицирует ТОЛЬКО PENDING,
      ACTIVE не затрагивается
- [ ] **Два MERGE подряд за < 2с (unit):** оба применяются, без конфликта
- [ ] **Приоритеты событий (unit):** critical (obstacle) прерывает песню на границе
      такта, high (battery) вплетается в PENDING без прерывания
- [ ] **e2e «батарея»:** поёт → battery_critical → предупреждение в песне → финал →
      сообщение об уходе на базу (без паузы и рестарта)

---

## 6. e2e-контракт (для e2e-process, после merge)

Голосовая команда уже есть в репо: `.github/e2e/voice_commands/rabot_spoy_pro_kotika.ogg`
(«робот, спой про котика») — подходит как базовый сценарий «спой»; для проверки
MERGE-сценария «комар + енот» используется фраза с wake word и двумя топиками
(см. `.github/e2e/VOICE_COMMANDS_RESEARCH.md` — какие .ogg уже проверены).

Параметры прогона (ориентир):
```
voice_text: "Робот, спой песню про комара и про енота"
volume: 150
record_seconds: 90
llm: deepseek (текущий fallback-провайдер)
tts: yandex
stt: yandex
```
Критерий e2e: рэп/песня с музыкой играет до конца (4+ чанка), `stop_music`
публикуется строго после `tts/batch_complete` последнего чанка; нет пост-амбла
«Готово!» поверх трека.

---

## 7. Риски и открытые вопросы

| Риск | Митигация |
|------|-----------|
| Ре-ордеринг батча сломает историю сообщений (tool-result не в том порядке) | возвращать результаты по `tool_call_id` в исходном порядке |
| `TaskScheduler` создан в `rob_box_voice`, а tool loop — в `rob_box_harness` (слой-зависимость) | scheduler передаётся в DialogCore портом (как acceptance_gate); rob_box_harness не импортирует rob_box_voice — адаптер живёт в dialogue_node |
| stop_music «в очереди» вместо мгновенного — пользователь ждёт | stop_music исполняется на границе voice-чанка (≤4с), критичные события (obstacle) прерывают через ReflexLayer |
| EventBus уже занят под system events (Phase 2) | не путать: SchedulerEventBus (Phase 2) для /battery/*, /obstacle/*; task_events — отдельный publisher |
| Q10 (reflex-events транспорт: ROS topic vs прямой вызов) всё ещё открыт | не блокирует W7a/W7b; затрагивает только ReflexLayer-интеграцию (Phase 2.5) |

---

## 8. Что НЕ входит в W7

- Двухуровневое quick-decide (уровень 2, лёгкая LLM < 800мс) — модуль
  `scheduler/decision.py` уже есть, подключение классификатора к новому вводу —
  отдельная карточка (Phase 2).
- ReflexLayer-интеграция с command_node (Phase 2.5, блокер Q10).
- SegmentEstimator/speculative pre-gen в живом цикле (Phase 3) — модули есть,
  подключение к W7b — следующая итерация.
- Action server в tts_node (Phase 4) — приземлён как sidecar, перевод живого
  пути на action-интерфейс — отдельная карточка.
