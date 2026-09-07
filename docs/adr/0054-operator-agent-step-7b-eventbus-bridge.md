# ADR-0054: `ReflexLayer` ↔ `command_node` через внутрипроцессную `EventBus`

| Поле | Значение |
|---|---|
| Статус | Proposed (после merge → Accepted) |
| Дата | 2026-09-07 |
| Автор | architect по карточке `#1997` (operator-agent 07b, шаг 7б из целевой §13) |
| Контекст | Закрывает шаг 7б целевой миграции `docs/architecture/target-operator-agent-and-dialogue.md` §13: подключить `ReflexLayer` (656 LOC, `scheduler/reflex.py`) к шине, подписать его на топик `command_node` и публиковать `ReflexEvent` на `/reflex/events`. Шина остаётся внутрипроцессной (тащить её в ROS = заводить второй `/mcp/tools` для внутренних событий). Разблокирует рефлексные сценарии «стой!» / «направо» из `docs/design/SCHEDULER_DESIGN.md` §8.10.6. |
| Затрагивает | `src/rob_box_voice/rob_box_voice/command_node.py` (новый ROS-параметр `enable_reflex_layer`, фоновый поток с asyncio-loop, публикация `EventEnvelope`); `src/rob_box_voice/rob_box_voice/scheduler/reflex.py` (`attach()`/`detach()`/`aclose()`, `_consume()` consumer-loop, `_envelope_to_command()` reconstructor, `SUBSCRIBE_TOPIC`); `src/rob_box_voice/test/unit/node/command_reflex_bridge/` (новый test pkg: envelope-shape 12 тестов, live round-trip 4 теста, disabled no-op 3 теста, pure envelope helper 4 теста) |
| Родители | ADR-0051 (агентский цикл оператора), ADR-0018 (честность — ADR должен появиться до merge, как и для 5а/6), `docs/architecture/target-operator-agent-and-dialogue.md` §8а.1 (таблица «статус»), §8а.4 (порядок включения), §13 шаг 7б, `docs/design/SCHEDULER_DESIGN.md` §8.10.4 (минимальная интеграция), §1 «Чего НЕ хотим» (scheduler не на ROS-топике) |
| Блок-зависимости | Зависит от шага 07 (`EventBus` + отмена — `scheduler/event_bus.py` и `task_scheduler.py` уже в develop, см. §1.1). Разблокирует рефлексные сценарии (e2e-проверки «стой!» во время песни / «направо» во время езды) и шаг 08 (срез на транспорте — без reflex-events LLM-feedback остаётся без прерываний). |

> **TL;DR.** `ReflexLayer` подписывается на `EventBus` (внутрипроцессную, async) по
> топику `/command/parsed`, на который `command_node` публикует JSON-friendly
> dict с полями `intent`/`text`/`entities`/`confidence`. После классификации
> `ReflexLayer` публикует `ReflexEvent` на `/reflex/events`. Существующий путь
> `command_node → Nav2` не задевается: мост opt-in через ROS-параметр
> `enable_reflex_layer` (default `false`), фоновый поток с asyncio-loop владеет
> шиной; rclpy-executor шлёт опубликованные команды через
> `asyncio.run_coroutine_threadsafe` (тот же паттерн, что в
> `RobustAudioBridge` / `audio_playback_manager.py`). 23 новых unit-теста
> покрывают три DoD из issue #1997.

---

## 1. Контекст и бизнес-проблема

`ReflexLayer` (`src/rob_box_voice/rob_box_voice/scheduler/reflex.py`, 656 LOC)
написан и приземлён в develop, но **не подключён**: в `command_node.py` слова
`reflex` нет ни разу (`SCHEDULER_DESIGN §11.6`). Целевая
(`target-operator-agent-and-dialogue.md §2.5`) и `SCHEDULER_DESIGN §8.10.4`
требуют подключить слой к шине, чтобы рефлексные сценарии из §8.10.6 («Спой
песню → стой!», «Едь на кухню → направо») вошли в прод. Без моста регулярки
типа «стоп» работают только в Nav2-пути (`handle_stop` через `CancelGoal`),
а `cancel-all` через `TaskScheduler` и приоритетная врезка в TTS-очередь —
не активны.

Главное архитектурное ограничение (`SCHEDULER_DESIGN.md §1 «Чего НЕ хотим»):
**scheduler не живёт на ROS-топике**. То есть:

- ❌ `/reflex/events` как ROS-топик + DDS round-trip (+2–5 мс в `network_mode: host`).
- ❌ Дублирование внутренних событий через `/mcp/tools` (это tool-API, не event-API).
- ✅ Внутрипроцессная `EventBus` (asyncio), `command_node` шлёт из rclpy-callback
  через `asyncio.run_coroutine_threadsafe`.

### 1.1 Что уже сделано в смежных карточках

| Что сделано | Где | Карточка |
|---|---|---|
| `EventBus` + `EventEnvelope` + `BackpressurePolicy` (BLOCK/DROP_OLDEST/RAISE) | `scheduler/event_bus.py` (189 LOC) | шаг 07 (уже в develop) |
| `TaskScheduler` + `SchedulerTask` + метрики | `scheduler/task_scheduler.py` | шаг 07 |
| `ReflexLayer` (классификация, debounce, метрики, history) | `scheduler/reflex.py` 656 LOC | #1993 (уже в develop) |
| `EventBus.subscribe` с `BLOCK` backpressure и bounded queue | `event_bus.py:145-160` | шаг 07 |

Это значит, что карточка **не пишет шину** — только мост.

### 1.2 Что осталось (эта карточка)

1. `ReflexLayer.attach(bus)` — подписка на `/command/parsed` + asyncio-consumer
   loop + `handle(envelope)` для каждого опубликованного envelope.
2. `ReflexLayer.detach()` / `aclose()` — отмена consumer-task и закрытие
   subscription (sync + async variants для удобства тестов и teardown).
3. `ReflexLayer._envelope_to_command(envelope)` — reconstructor: из
   JSON-friendly dict обратно в duck-typed command (intent.value / text /
   entities / confidence). Чистая статика — тестируется отдельно.
4. `CommandNode` — параметр `enable_reflex_layer` (default `false`), фоновый
   daemon-поток с `asyncio.new_event_loop()`, инициализация шины +
   `TaskScheduler.start()` + `ReflexLayer(scheduler, bus).attach(bus)`,
   публикация `EventEnvelope(topic=SUBSCRIBE_TOPIC, payload=...)` из
   `stt_callback` через `asyncio.run_coroutine_threadsafe`.
5. `destroy_node()` идемпотентно гасит поток и закрывает шину.
6. `CommandNode.build_parsed_envelope(command)` — pure static helper для
   тестируемости wire-контракта.

## 2. Принятое решение

### 2.1 Wire-контракт `/command/parsed`

| Поле | Тип | Контракт |
|---|---|---|
| `intent` | `str` | Обязательное. Значение из `IntentType.value` (`navigate`, `stop`, `follow`, `status`, `map`, `vision`, `unknown`). |
| `text` | `str` | Оригинальная фраза после `parse()`. По умолчанию `""`. |
| `entities` | `dict[str, Any]` | Извлечённые сущности. Всегда маппинг (даже пустой) — `_envelope_to_command` отбрасывает envelope с `entities=None`. |
| `confidence` | `float` | Уверенность парсера. Default `0.0`. |

Envelope оборачивает payload в `EventEnvelope(topic="/command/parsed",
event_id=uuid4(), priority="normal", created_at=time.monotonic())`.
См. `build_parsed_envelope()` в `command_node.py:226–260`.

### 2.2 Архитектура потока

```
rclpy executor thread                asyncio loop (daemon thread)
        │                                       │
        ▼                                       │
  stt_callback(msg)                             │
        │                                       │
        ├─► self.command_parser.parse(text)      │
        │                                       │
        ├─► self._publish_parsed_async(command)─┼──► EventBus.publish(envelope)
        │                                       │           │
        ├─► self.publish_intent(command)        │           ▼
        │       (Nav2 / dialogue — как раньше)  │     /command/parsed subscriber
        │                                       │           │
        │                                       │           ▼
        │                                       │     ReflexLayer._consume()
        │                                       │           │
        │                                       │           ▼
        │                                       │     ReflexLayer.handle(cmd)
        │                                       │           │
        │                                       │           ▼
        │                                       │     EventBus.publish(/reflex/events)
        └───────────────────────────────────────┘
```

Ключевые инварианты:

1. **Шина живёт на отдельном daemon-потоке** (`command-node-reflex-bus`).
   `rclpy` executor не владеет asyncio loop, поэтому вкладывать шину в rclpy
   нельзя. Тот же паттерн, что `RobustAudioBridge` в `audio_playback_manager.py`.
2. **Один event-loop owner**: шина, scheduler, layer создаются в
   `_bus_thread_main` через `asyncio.new_event_loop()`. rclpy-callback шлёт
   в этот loop через `asyncio.run_coroutine_threadsafe`.
3. **Default OFF**: `enable_reflex_layer=false` → шина не поднимается,
   `destroy_node()` идемпотентен, существующие харнесы и e2e не задеты.
4. **Lazy import**: `from rob_box_voice.scheduler import EventBus, ReflexLayer,
   TaskScheduler` — внутри `_start_reflex_bridge()`, чтобы юнит-тесты без
   моста не платили import-cost scheduler-пакета.
5. **`aclose()` отдельно от `detach()`**: `detach()` — sync (нужен из
   rclpy-callback для shutdown), `aclose()` — async (для тестов с
   `await layer.aclose()`).
6. **Ошибка handle() не убивает consumer-loop**: `_consume` ловит
   `Exception` (но не `CancelledError`) и логирует warning. Один плохой
   envelope не должен убить рефлексный путь.

### 2.3 Альтернативы, которые отклонены

| Альтернатива | Почему нет |
|---|---|
| ROS-топик `/command/parsed` через DDS | `SCHEDULER_DESIGN.md §1 «Чего НЕ хотим»` — scheduler не на ROS-топике. +2–5 мс latency в `network_mode: host`, удвоение tool-API. |
| Прямой вызов `ReflexLayer.handle()` из rclpy-callback | Требует общего loop (сейчас loop живёт в `RobustAudioBridge`, не в `command_node`). Создание loop на rclpy main-thread ломает `executor.spin()`. |
| Топик `/reflex/events` через DDS вместо шины | `ReflexLayer` намеренно ничего не знает про ROS. Шина — единая точка, через которую ещё `TaskScheduler` шлёт cancel-events (шаг 07). |
| `command_node` как полноценный ROS-publisher вместо локальной шины | Дублирование с `/reflex/events` для downstream-наблюдения. Шина + явная подписка на `ReflexLayer.TOPIC` через `bus.subscribe` в тестах уже даёт observability без ROS. |

### 2.4 Совместимость с `disable_reflex_layer` / существующим Nav2-путём

`enable_reflex_layer=false` — это **дефолт** и рекомендованная конфигурация
для уже работающих харнесов. Поведение:

- `_event_bus = None`, `_reflex_layer = None`, фоновый поток не создаётся.
- `stt_callback` вызывает `_publish_parsed_async()`, который сразу выходит
  (`if self._event_bus is None: return`).
- `destroy_node()` — no-op для шины (см. `test_destroy_node_is_idempotent_when_bridge_was_never_started`).

Существующие тесты `command_node` (`test/unit/node/`) собирают ноду без
параметра `enable_reflex_layer` → дефолт `false` → мост не активен → Nav2 и
диалог работают байт-в-байт как раньше. Это подтверждено тестом
`test_no_background_thread_is_created_when_disabled`.

### 2.5 Что НЕ в этой карточке

- Сама шина и cancel-механика (шаг 07, уже в develop).
- Спекулятивная генерация (`docs/design/SCHEDULER_DESIGN §11 шаг 13`) — отдельная карточка.
- ROS-топик `/reflex/events` для observability — отдельная карточка, если
  потребуется. Сейчас шина + `bus.subscribe(ReflexLayer.TOPIC)` уже дают
  наблюдаемость для тестов и, при желании, для мониторинга в одном процессе.
- LLM-фидбек `[REFLEX EVENTS]` блок (`SCHEDULER_DESIGN §14 Q6`) — закрыт
  как «да, всегда», но реализация — в `feedback_formatter`, отдельная
  карточка после этого шага.

## 3. Definition of Done (проверяемые факты)

- [x] `git grep -i reflex src/rob_box_voice/rob_box_voice/command_node.py`
      возвращает ≥10 вхождений (параметр, поток, публикация, импорты).
- [x] `ReflexEvent` публикуется при команде навигации — тест
      `test_attach_consumes_navigate_command_and_emits_reflex_event`:
      live round-trip `bus.publish(/command/parsed) → /reflex/events`.
- [x] Обычный путь не деградировал: `enable_reflex_layer=false` → bridge no-op,
      Nav2-путь неизменен (тесты `test_disabled_noop.py` +
      `test_no_background_thread_is_created_when_disabled`).
- [x] 23 новых unit-теста + 43 существующих `ReflexLayer` теста = **66/66 PASS**:
      ```
      pytest src/rob_box_voice/test/test_reflex_layer.py \
             src/rob_box_voice/test/unit/node/command_reflex_bridge/ -v
      ============================== 66 passed in 1.32s ==============================
      ```

## 4. Как проверить локально

```bash
# unit-тесты моста (4 + 12 + 3 + 4 = 23)
pytest src/rob_box_voice/test/unit/node/command_reflex_bridge/ -v

# Регрессия ReflexLayer
pytest src/rob_box_voice/test/test_reflex_layer.py -v

# Линт (CC-бюджет должен быть ОК — ни один существующий файл не вырос за порог)
python3 scripts/lint/cc_budget.py

# grep-чек на DoD #1
git grep -i reflex src/rob_box_voice/rob_box_voice/command_node.py | wc -l  # ≥10
```

## 5. Риски и компромиссы

- **Daemon-поток на процесс command_node**: при kill -9 поток не
  завершается gracefully (daemon=True). При штатном `destroy_node()`
  поток корректно глушится через `loop.call_soon_threadsafe(loop.stop)` +
  отмена pending tasks + `loop.close()`. Worst case — утечка одного
  asyncio-loop в zombie-процессе, что не хуже уже существующего
  `RobustAudioBridge`.
- **Топик `command_node` ↔ шина = два разных API**: если будущая рефакторинг
  переедет на полноценный ROS-топик, придётся обновить только `build_parsed_envelope`
  + добавить subscription с `ROSMCPToolProvider`. Wire-контракт dict-а стабилен.
- **`_envelope_to_command` duck-types**: вместо реального `Command` dataclass
  консьюмер собирает мини-shim. Это экономит импорт `Command` в scheduler (а он
  сейчас в `core/`, не в `scheduler/`). Минус — теряем type-check на стороне
  consumer; плюс — модуль scheduler остаётся автономным (родитель ADR-0051 §2.2).
- **Backpressure = BLOCK**: при забитом слое (нереально для 5 events/hour,
  SCHEDULER_DESIGN §14 Q6) parser блокируется. Это сознательное решение —
  лучше замедлить STT, чем дропать reflex-events.

## 6. Связанные ADR

- ADR-0051 (родитель) — агентский цикл оператора.
- ADR-0054 (sibling — 0054-dialogue-control-pause-resume.md) — пауза личности
  через `/dialogue/control`. Это другой шаг (6), но оба в одной миграции
  operator-agent и используют общий принцип «единственный канал = один топик».
- ADR-0054 (sibling — 0054-operator-agent-step-5a-wake-stream.md) — wake-stream
  от Quest. Это другой шаг (5а); здесь шаг 7б.
- ADR-0013 (incremental delivery) — поэтому `enable_reflex_layer=false` в дефолте,
  а не «выкатываем сразу на прод».
- ADR-0018 (honest FAIL) — DoD требует raw pytest-вывод и grep-чеки; без них
  шаг 7б не считается закрытым.

> **Честный FAIL.** Это архитектурная + имплементационная карточка:
> код, тесты, ADR. Сам по себе ADR без кода — не закрытие шага. Код
> ревьюится разработчиком перед merge; e2e (live голосом «стой!» во время
> песни / «направо» во время езды) — отдельный шаг, не здесь.
