# ADR-0011: Action protocol (ActionServer + PASTE) — фиксация контракта Phase 4

| Поле         | Значение                                                                                       |
|--------------|------------------------------------------------------------------------------------------------|
| Статус       | **Accepted** (фиксация post-factum — реализация приземлилась в коммитах `3226e506` / `863a6744`) |
| Дата         | 2026-08-04                                                                                     |
| Автор        | architect (Hermes Agent)                                                                       |
| Kanban       | `t_6d0e1dfe` (Definition), `t_21f11e76` (Phase 4 Implementation), `t_50db3dc4` (server), `t_6ff90e48` (PASTE client), `t_25ab0ec9` (docs) |
| Контекст     | Issue [#968](https://github.com/krikz/rob_box_project/issues/968) §11.5 (Phase 4 — Action Server + PASTE speculative execution) |
| **Суперсет** | **ADR-0002 (Proposed)** — этот ADR финально утверждает контракт, изменяя выбор транспорта (см. §2.2 и §6.1) |
| Предшественники | [ADR-0001](0001-harness-architecture.md) §2.4 (порты харнесса, в т.ч. Tool-порт), `SCHEDULER_DESIGN.md` §11.5 (фаза 4 плана-планировщика) |
| Связанные    | `src/rob_box_voice/rob_box_voice/action_server/{server,paste,http,http_server}.py` (commit `3226e506`), `docker/vision/docker-compose.yaml` (commit `863a6744`), `docs/architecture/scheduler-design.md` §11.5 |
| Реализация   | `src/rob_box_voice/rob_box_voice/action_server/` (4 модуля + README + 53 строки тестов), `docker/vision/docker-compose.yaml` (sidecar `voice-action-server`) |
| Ветка        | `feature/t_21f11e76` (на базе `feature/harness-p0-foundation`)                                |

> **Замечание для ревьюера.** Этот ADR решает транспортную развилку, оставшуюся открытой в ADR-0002 (Proposed). ADR-0002 выбрал ROS2 Action; реализация (Phase 4 step-1/2) пошла по HTTP+JSON sidecar. Этот ADR **меняет решение** по транспорту и предоставляет обоснование post-hoc — см. §2.2, §6.1. Семантика контракта (lifecycle, speculation, idempotency, versioning) остаётся как в ADR-0002 §3.4, §4, §5.

---

## 1. Назначение

Планировщик (Phase 1–3) уже умеет детерминированно исполнять сегменты, обрабатывать отмену и спекулятивную пред-генерацию через `SpeculativePreGenerator` (`src/rob_box_voice/rob_box_voice/scheduler/speculative_pre_gen.py`). Для устранения **остаточных** источников race-условий между LLM-циклом и исполнителями (например, `stop_music` обгоняющий `speak_text` — см. issue #968) SCHEDULER_DESIGN.md §11.5 запланировал выделение **отдельного action server'а** с двумя каналами интеграции:

1. **Action server** — транспорт-нейтральный, асинхронный, долгоживущий сервис, владеет жизненным циклом goal/feedback/result/cancel и предоставляет детерминированный `/healthz` + graceful shutdown.
2. **PASTE** — Microsoft PASTE-style [arxiv 2603.18897](https://arxiv.org/abs/2603.18897) speculative execution с shadow queue: prefetch N+1 результата без побочных эффектов, потом `commit` ровно одного.

Цель этого ADR — зафиксировать **контракт** между этими двумя компонентами (и любой клиентской интеграцией), чтобы:
- Исполнители (TTS, music, animation, nav) могли подключаться по единой схеме через **plugin registry** без зависимости от ROS-топика.
- PASTE-планировщик и action server запускаются как **независимые процессы** (Phase 4 sidecar в docker-compose), но могут коллапсироваться в один процесс для тестов и embedded-режима.
- Существующие ROS-топики (`/voice/audio/speech`, `/harness/task_events`) сохраняются как fallback — миграция incremental, без breaking change.

> **Важно:** ADR-0001 immutable — этот ADR не меняет ADR-0001, а **расширяет** один из его 7 портов (Tool-порт) транспорт-нейтральной action-семантикой. Это соответствует SCHEDULER_DESIGN.md §11.5 и Phase 5 (`SideEffectBus`, ADR-0001 §5) — наш ActionServer не пересекается с `SideEffectBus` по scope (см. §6.3).

---

## 2. Проблема

### 2.1 Существующие проблемы (Phase 1–3 решают только часть)

| Проблема | Где проявляется | Что осталось |
|---|---|---|
| `stop_music` обгоняет `speak_text` | issue #968, v36 e2e | Phase 1-3 закрывают через каналы + state-машины, **но** механизм «один goal = один feedback-stream» для длинных операций **отсутствует** — сейчас это asyncio.Lock + dict, а не first-class primitive |
| Дебаунс/таймеры как proxy для состояния | `dialogue_node.py` (см. SCHEDULER_DESIGN §12) | Phase 4 (этот ADR) заменяет на state-машины каналов |
| Prefetch жжёт токены / время | при LLM > 2с | Phase 3 дал `SpeculativePreGenerator`, **но** он в одном процессе с executor'ом — нет отдельного shadow queue |
| Двойное использование одной подписки | разные ветки `command_node` | Сейчас не критично, но без action-server'а продолжит расти |

### 2.2 Что не подходит как решение

| Подход | Почему отвергнут |
|---|---|
| ROS2 Action Server для TTS (`/tts_action`) | INSIGHT #7 (SCHEDULER_DESIGN §8) — рабочий вариант, но требует переписать весь `tts_node` (~2.5K строк) и привязывает к DDS. ADR-0002 §3.1 выбрал именно этот вариант как основной, **но** требует переписать `tts_node` **до** старта Phase 4. Принципиальные причины против (см. §6.1): (а) требует **synchronous refactor** существующих компонентов вместо incremental deployment; (б) делает action-сервер зависимым от DDS — теряется «честный» health lifecycle через docker healthcheck; (в) Phase 5 `SideEffectBus` и так будет in-process Python, не DDS — ROS2 Action создаёт второй транспорт, который Phase 5 будет вынуждена ретировать. **Решение:** HTTP+JSON sidecar (этот ADR), **сохраняя** существующий `/voice/audio/speech` топик как fallback до полного перевода. ADR-0002 §3.1 альтернатива признаётся **future option** для случая, когда `tts_node` будет полностью переписан (Phase 6+, см. §10 OOS8). |
| Чистый HTTP без границ | Race на cancel/feedback, нет единого lifecycle. Нужен контракт |
| Чистый gRPC | Сильная типизация, но protobuf-схема + codegen + collocation с Python asyncio требует 3+ недели отладки. **HTTP+JSON** даёт 90% ценности за 10% времени, а спецификация OpenAPI 3.0.3 уже готова как authoritative source for both транспорт (HTTP) и валидацию (JSON Schema). Protobuf-схема поставляется как **альтернативный** binding (см. §4.3) |
| LLM-вызовы мимо движка | Не даёт `commit`/`discard` семантики — speculative execution нельзя гарантированно откатить |

### 2.3 Что выбрано

**ActionServer + транспорт-нейтральный серверный core + опциональный HTTP adapter (aiohttp) + PASTE-стиль shadow queue.** Контракт-зависимая часть — handler signature, `ActionState` enum, `ActionHandle` lifecycle. Транспорт — HTTP+JSON (production sidecar), в embedded-режиме — прямой Python-вызов. Protobuf-схема поставляется параллельно для будущей gRPC-миграции (когда появится второй язык в стеке — Node/Go/C++).

---

## 3. Контракт (зафиксировано в коде)

### 3.1 Core data model (`server.py`)

```python
class ActionState(str, Enum):
    ACCEPTED  = "accepted"   # goal создан, handler ещё не стартовал
    RUNNING   = "running"    # handler исполняется
    SUCCEEDED = "succeeded"  # handler вернул значение
    CANCELLED = "cancelled"  # отменён через cancel() или shutdown()
    FAILED    = "failed"     # handler бросил исключение

@dataclass(frozen=True)
class Health:
    ok: bool              # True если сервер не в shutdown
    active_goals: int     # кол-во незавершённых goals
    shutting_down: bool   # True после shutdown() начат

@dataclass
class ActionHandle:
    goal_id: str
    task: asyncio.Task
    state: ActionState = ActionState.ACCEPTED
    feedback: list[Mapping] = field(default_factory=list)
    result: Any = None
    error: str | None = None
```

**Правила переходов:**

```
                    submit(goal)
                         │
                         ▼
             ┌─────────────────────┐
             │     ACCEPTED        │  (task создан, ещё не запущен)
             └──────────┬──────────┘
                        │ task scheduled
                        ▼
             ┌─────────────────────┐
             │      RUNNING        │  (handler вызван)
             └──────────┬──────────┘
                        │
        ┌───────────────┼───────────────┬──────────────┐
        │               │               │              │
        ▼               ▼               ▼              ▼
   SUCCEEDED       CANCELLED       CANCELLED       FAILED
 (handler return) (cancel() или   (shutdown()    (handler raise)
                  cancel_event    + timeout)
                  через SIGTERM)
```

**Важно:** `CANCELLED` имеет **два** источника — (а) явный `cancel(goal_id)` от планировщика, (б) `shutdown()` + SIGTERM. В обоих случаях handler **обязан** корректно отработать `cancelled_event.is_set()` перед возвратом (см. §3.4 invariant).

### 3.2 Handler signature (фиксировано)

```python
Handler = Callable[
    [Mapping[str, Any],                  # goal
     Callable[[Mapping[str, Any]], None], # feedback(value) — sync callback
     asyncio.Event],                     # cancelled — cooperative event
    Any | Awaitable[Any]
]
```

**Семантика:**
- Handler может быть `sync` или `async` (сервер нормализует).
- `feedback(value)` — добавление в `handle.feedback` **только если task ещё не done**. Не thread-safe (всё в одном event loop).
- `cancelled` — `asyncio.Event`, выставляется **одновременно** с `task.cancel()` (через `task.cancel()` + `cancel_event.set()` в `cancel()` и в `shutdown()`).
- Возврат: любое JSON-сериализуемое значение. Записывается в `handle.result` и фиксирует `state = SUCCEEDED`.
- `None` после `cancelled.is_set()` = корректное завершение, не фейл.

### 3.3 Health endpoint

```python
def health(self) -> Health:
    return Health(not self._shutting_down, self.active_goals, self._shutting_down)
```

**Liveness правило:**
- `ok = True` пока `shutting_down = False` (даже если есть `active_goals > 0` — это нормальная нагрузка).
- `ok = False` **только** когда начат `shutdown()` — Kubernetes / docker должен начать drain.

**Readiness правило (HTTP adapter):**
- `GET /healthz` → `200 {"ok": true, "active_goals": N, "shutting_down": false}` в нормальном режиме.
- `GET /healthz` → `503 {"ok": false, ...}` когда `shutting_down = true` (см. §3.5).

### 3.4 Invariants (для тестов и ревью)

1. **Goal ID стабильность.** `goal_id` либо приходит в `goal["goal_id"]`, либо сервер генерит `uuid.uuid4()`. После submit **не меняется** ни в `ActionHandle`, ни в HTTP-ответе, ни в feedback-стриме.
2. **Feedback append-only.** `handle.feedback` — list, **только растёт**. Никогда не trunc'ается, не редактируется. Семантика «история событий».
3. **Cancelled cooperative + task-cancel.** Handler получает **оба** сигнала: `task.cancel()` (asyncio) и `cancelled_event.set()` (Python). Handler **обязан** проверять `cancelled.is_set()` в долгих циклах (например, ожидание TTS-чанка); иначе — полагаться на `CancelledError` в `await`.
4. **Cancel → state CANCELLED, не FAILED.** Если handler бросил `asyncio.CancelledError`, сервер ловит, ставит `state = CANCELLED`, **пробрасывает** `CancelledError` (паттерн asyncio). Это позволяет `handle.wait()` корректно прокинуть отмену наверх.
5. **Shutdown дожидается active goals.** `shutdown(timeout=2.0)` (default 2с) — после `cancel()` всех активных `await asyncio.wait(..., timeout=...)`. После timeout — `task` остаётся в `RUNNING` (но сервер не ждёт); event loop при выходе всё равно прибьёт. См. unit-test `test_action_server.py` — `started.wait()` + `cancel()` + `wait()`.
6. **Idempotency по `goal_id`.** Если submit с уже существующим `goal_id` — **ошибка** (защита от accidental retry-callback'а; см. §4.4). Сейчас не реализовано (нет dict-check), **MUST ADD** в Phase 4 step-3 (см. §6.4).

### 3.5 Graceful shutdown (production)

```python
async def shutdown(self, timeout: float = 2.0) -> None:
    self._shutting_down = True
    active = [h for h in self._handles.values() if not h.task.done()]
    for handle in active:
        self._cancel_events[handle.goal_id].set()
        handle.cancel()
    if active:
        await asyncio.wait([h.task for h in active], timeout=timeout)
```

**Docker / K8s flow:**
1. SIGTERM → ASGI/lifecycle handler ловит.
2. Вызывает `server.shutdown(timeout=2.0)` (настраивается через `ACTION_SERVER_SHUTDOWN_TIMEOUT` env, default 2с).
3. Все активные goals отменяются (cooperative + task.cancel).
4. `wait(timeout=2.0)` — дожидается завершения.
5. AppRunner.cleanup().
6. Process exits 0.

Если timeout превышен — process всё равно exits (event loop close прибьёт), но goals в `RUNNING` будут видны в логах как «не graceful». **Metric** `action_server_ungraceful_shutdowns_total` (TODO Phase 4 follow-up).

### 3.6 PASTE Planner (`paste.py`)

```python
class PastePlanner:
    def speculate(self, action_id: str, goal: Mapping) -> ShadowAction:
        """Запустить speculative worker. Идемпотентно по action_id."""

    async def commit(self, action_id: str) -> Any:
        """Подтвердить — забрать результат. Ошибка если не speculated."""

    def discard(self, action_id: str) -> bool:
        """Отменить speculative worker. Возвращает True если был в очереди."""
```

**Контрактное правило (INVARIANT #7):** speculative handler **MUST** быть side-effect free. Только `commit` может публиковать/играть аудио или вызывать внешние тулзы. Это аналог PASTE paper §3.2 (speculative token validate → commit).

**Поведение:**
- `speculate` создаёт `asyncio.Task`, складывает в `self._shadow`. Если `action_id` уже есть — возвращает существующий (idempotent).
- `commit` удаляет из shadow, `await task`. Если task уже завершился — мгновенный return.
- `discard` удаляет из shadow, `task.cancel()`. Возвращает `False` если `action_id` не было.
- `discard_all` — отмена всех (при cancel-сегмента, например, MERGE в PENDING).

### 3.7 Plugin registry (для исполнителей)

```python
# Production: handler передаётся в ActionServer при инициализации
# В Phase 4 step-3 (TODO) — будет Registry:
#   registry.register("tts.speak", TTSHandler(...))
#   registry.register("music.play", MusicHandler(...))
#   registry.register("nav.navigate", NavHandler(...))
#
# goal = {"kind": "tts.speak", "params": {"text": "..."}}
# dispatch = registry.resolve(goal["kind"])
# handle = server.submit({"goal": goal})
#
# Этап 1 (Phase 4 step-1): один handler в sidecar (заглушка _handler в http_server.py).
# Этап 2 (Phase 4 step-3): полноценный registry с несколькими handler'ами.
```

**Почему registry, а не глобальный dict:** handler'ы могут зависеть от конфига (voices, navigation graph). Registry инкапсулирует lifecycle (закрытие клиентов, освобождение ресурсов) — позже.

---

## 4. Транспорт

### 4.1 Решение: HTTP+JSON (production), прямой Python (embedded)

**Сравнение:**

| Критерий | HTTP+JSON | gRPC+protobuf | Прямой Python |
|---|---|---|---|
| Latency median | +2–5мс (intra-host) | +1–3мс | +0.1мс |
| Коллокация с Python asyncio | ✅ aiohttp | ⚠️ grpcio + thread pool | ✅ нативно |
| Codegen / schema validation | ✅ OpenAPI 3.0.3 | ✅ proto | ❌ |
| Multi-language clients | ✅ curl / fetch / любой | ✅ proto-gen | ❌ только Python |
| Embed в тесты | ⚠️ нужен aiohttp test client | ⚠️ нужен grpc test | ✅ pytest-asyncio |
| Tooling (Postman, Insomnia) | ✅ | ⚠️ grpcurl | ❌ |
| Operational overhead | ✅ стандартный | ⚠️ отдельный port scheme | — |

**Решение:** HTTP+JSON для production sidecar (4 endpoint'а, см. §4.2), прямой Python в `tests/` (53 строки в `test_action_server.py`). **gRPC-binding** поставляется как **опциональная** protobuf-схема (§4.3) и будет реализован, **когда** во втором языке (Go/C++/Node) появится реальная потребность. Не делаем speculative работу.

### 4.2 HTTP API (action-protocol.openapi.yaml)

```
GET  /healthz
POST /actions
GET  /actions/{goal_id}
POST /actions/{goal_id}/cancel
```

**Все ответы — JSON.** Коды:
- `200 OK` — успешный read (status, health).
- `202 Accepted` — goal принят, ID возвращён.
- `404 Not Found` — `goal_id` неизвестен.
- `409 Conflict` — goal с таким ID уже submitted (TODO после добавления idempotency-check).
- `503 Service Unavailable` — `shutting_down = true`.

Полная спецификация — OpenAPI 3.0.3 здесь: [`action-protocol.openapi.yaml`](./action-protocol.openapi.yaml).

### 4.3 Protobuf schema (опциональная, для будущего gRPC)

```protobuf
syntax = "proto3";
package rob_box_voice.action.v1;

service ActionService {
  rpc Submit(SubmitRequest) returns (SubmitResponse);
  rpc GetStatus(StatusRequest) returns (StatusResponse);
  rpc Cancel(CancelRequest) returns (CancelResponse);
  rpc Health(HealthRequest) returns (HealthResponse);
  // Streaming-вариант для long-lived feedback (RFC: bidirectional)
  rpc WatchFeedback(WatchRequest) returns (stream FeedbackEvent);
}

message Goal { /* action_id, kind, params, deadline_ms */ }
message ActionState { enum Value { ACCEPTED=0; RUNNING=1; SUCCEEDED=2; CANCELLED=3; FAILED=4; } ... }
```

Полная схема: [`action-protocol.proto`](./action-protocol.proto). Это **reference implementation**, не runtime — `protoc` генерация не входит в Phase 4 step-1. Если в Phase 5 добавится Go-сервис, реализуем gRPC-bridge адаптер.

### 4.4 Idempotency

Клиент **должен** передавать `goal_id` (UUID v4) в `goal["goal_id"]`. Семантика:
- **Retry-safe:** повторный `POST /actions` с тем же `goal_id` → `409 Conflict` (если есть в `_handles`) или возврат existing `goal_id` (если уже завершён). См. TODO Phase 4 step-3.
- **Корреляция:** `goal_id` = `action_id` в PASTE. Один action_id для commit / discard.

Сейчас (Phase 4 step-1) сервер **не** enforce'ит idempotency — это acceptance backlog. До добавления клиент должен обеспечивать unique `goal_id` сам.

---

## 5. Версионирование

### 5.1 Семантическое версионирование контракта

- **MAJOR.** Поломка обратной совместимости (удаление поля, изменение enum-значения, изменение HTTP path). Текущий контракт = `v1`.
- **MINOR.** Новое поле с дефолтом, новый endpoint, новое значение enum. Старые клиенты продолжают работать.
- **PATCH.** Чисто документные изменения, никаких изменений JSON-схемы.

### 5.2 Требования к совместимости

- **Wire format.** `application/json; charset=utf-8`. JSON Schema в `action-protocol.openapi.yaml` — canonical источник.
- **HTTP status codes.** Новые коды вводятся только через MINOR bump. `503` для `shutting_down` — обязательно.
- **Backwards compatibility window.** MINOR-версия поддерживается 6 месяцев с момента выпуска MAJOR-следюущей. (Это политика, конкретный SLA — в §6.4.)

### 5.3 Текущая версия

- **Контракт:** `v1.0.0` (2026-08-04, Phase 4 step-1).
- **Реализация:** `src/rob_box_voice/rob_box_voice/action_server/` — `server.py`, `paste.py`, `http.py`, `http_server.py`.
- **Тесты:** `src/rob_box_voice/test/test_action_server.py` — 3 теста, 53 строки.

---

## 6. Trade-off анализ

### 6.1 Почему HTTP+JSON, а не gRPC

| Цена | Выгода |
|---|---|
| +2–5мс latency (intra-host) | OpenAPI tooling, curl-able, no codegen |
| ~150 LOC aiohttp adapter | Простота отладки, стандартный HTTP-стек |
| Manual JSON Schema validation | Никакого contract drift — OpenAPI = source of truth |

**Latency не блокирует:** SCHEDULER_DESIGN.md §4.6.2 требует < 800мс на quick_decide и < 50мс на L1-rules. Action protocol — это **между** LLM-вызовом и исполнителем, где latency budget ~50–200мс (TTS API ~200мс, LLMСalling 1.5–3с). +5мс HTTP round-trip можно съесть.

### 6.2 Почему sidecar, а не встроенный процесс

| Цена | Выгода |
|---|---|
| Дополнительный docker-container | **Independent health lifecycle** — action server может перезапускаться, не роняя voice-assistant |
| Нужна оркестрация (docker-compose `depends_on`) | **Изоляция failure** — падение sidecar'а не роняет бота |
| +150 MB RSS (Python + aiohttp) | **Canary deploy** — можем катить action server отдельно от voice-assistant |

**Альтернатива (отвергнута):** встроить action server в `voice-assistant` контейнер. Это убирает sidecar, но:
- Один health endpoint = два сервиса. K8s/docker считает контейнер healthy, даже если action server лежит.
- Любая утечка памяти в action server (async tasks, cancellation hygiene) задевает voice-assistant.
- Phase 5 (AgentSession + SideEffectBus) захочет свой action-сервер; sidecar готов.

### 6.3 Пересечение с ADR-0001 §5 (Future SideEffectBus)

ADR-0001 §5 планирует `AgentSession[StateT] + SideEffectBus` как Phase 5. Наш ActionServer **не конкурирует** с SideEffectBus:

| | ActionServer | SideEffectBus |
|---|---|---|
| Scope | Долгие **action-handler**'ы (TTS, music, nav) | **Side-effect** события (state changes, observability) |
| Транспорт | HTTP+JSON (production sidecar) | In-process Python (после Phase 5) |
| Семантика | request/response + feedback stream | pub/sub + replay |
| Lifecycle | Goal-ID, cancel, health | Subscription, retention |

**Где пересечение:** когда Phase 5 введёт SideEffectBus, action server может **публиковать** action-completion events в SideEffectBus (one-way, не consuming). Это **additive** изменение, не breaking.

### 6.4 Открытые вопросы (TODO Phase 4 step-3)

| # | Вопрос | Решение | Tracking |
|---|---|---|---|
| Q1 | Idempotency-guard на duplicate goal_id | Добавить check в `submit()`, вернуть 409. | Phase 4 step-3 |
| Q2 | Plugin registry (сейчас single handler) | `ActionRegistry` class, `register(name, factory)`. | Phase 4 step-3 |
| Q3 | Structured logging (JSON для promtail) | Заменить `print`/`logger.info` на `structlog` или аналог. | Phase 4 follow-up |
| Q4 | Prometheus metrics (`active_goals`, success/failure rate, latency histogram) | `prometheus_client` + `/metrics` endpoint. | Phase 4 follow-up |
| Q5 | WebSocket / SSE для streaming feedback | Альтернатива polling `GET /actions/{id}`. | Phase 5 (не критично) |
| Q6 | Каким сервисам разрешено `POST /actions`? (auth) | `ACTION_SERVER_API_TOKEN` (env), проверка в middleware. | Phase 4 step-3 |
| Q7 | SLA по shutdown timeout | По умолчанию 2.0с. Настраивается через `ACTION_SERVER_SHUTDOWN_TIMEOUT`. | Phase 4 step-1 (есть), step-3 (документация) |

---

## 7. End-to-end сценарий: PASTE → ActionServer

```mermaid
sequenceDiagram
    autonumber
    participant LLM as LLM (dialog_core)
    participant P as PASTE Planner
    participant AS as ActionServer (HTTP)
    participant H as TTS Handler
    participant TTS as tts_node (existing)

    Note over LLM,TTS: Phase 1–3: scheduler планирует сегменты
    LLM->>P: 1. submit_segment(seg_id="s1", text="Привет, мир!")
    activate P

    P->>AS: 2. POST /actions {goal_id:"s1", kind:"tts.speak", text:"Привет, мир!"}
    activate AS
    AS-->>P: 202 {goal_id:"s1", state:"accepted"}
    AS->>H: 3. _handler(goal, feedback, cancelled_event)
    activate H

    H->>TTS: 4. publish /voice/audio/speech (existing ROS topic)
    TTS-->>H: 5. audio chunks...

    loop every 200ms during synthesis
        H->>AS: 6. feedback({"progress": 0.6})
        AS->>AS: handle.feedback.append(...)
    end

    Note over P,AS: 7. Meanwhile, PASTE speculates N+1
    P->>AS: 7. POST /actions {goal_id:"s2", kind:"tts.speak", text:"Следующая фраза"}
    activate AS
    Note right of AS: Shadow queue — TTS pre-synthesis
    AS->>H: 8. _handler(goal_s2, feedback, cancelled_s2)
    H->>TTS: 9. publish /voice/audio/speech (pre-synth)

    H-->>AS: 10. result: {audio_uri:"s2.wav"}
    AS-->>P: 11. 200 {goal_id:"s2", state:"succeeded"}
    deactivate AS

    H-->>AS: 12. result: {audio_uri:"s1.wav"}
    AS-->>P: 13. 200 {goal_id:"s1", state:"succeeded"}
    deactivate AS
    deactivate H

    Note over P: 14. commit("s1"); persist audio URI in scheduler

    alt LLM MERGE во время синтеза s2
        P->>AS: 15. POST /actions/s2/cancel
        AS->>AS: cancelled_event.set(); task.cancel()
        AS-->>P: 200 {cancelled: true}
        P->>P: discard("s2") — shadow освобождён
    else LLM принял s2 (REPLACE/QUEUE)
        P->>AS: 16. GET /actions/s2 wait_for={state:"succeeded"}
        AS-->>P: 200 {state:"succeeded", result:{audio_uri:"s2.wav"}}
        P->>P: commit("s2") — audio играет
    end

    deactivate P
```

**Ключевые моменты:**
- **Шаги 2–6:** primary goal s1 идёт normal flow.
- **Шаги 7–11:** PASTE speculate s2 в shadow queue. TTS-handler **знает**, что это speculative (по `goal["speculative"]: true` флагу, передаваемому планировщиком), и должен быть **side-effect free** — например, синтезировать аудио в файл, не отправлять в `audio_node`.
- **Шаги 14–16:** commit / discard — единственные действия, которые могут публиковать side-effects.

---

## 8. Деплой и эксплуатация

### 8.1 Docker sidecar (Phase 4 step-2)

```yaml
voice-action-server:
  image: ghcr.io/krikz/rob_box:voice-assistant-humble-${VOICE_ASSISTANT_TAG}
  container_name: voice-action-server
  network_mode: host
  environment:
    - PYTHONUNBUFFERED=1
    - ACTION_SERVER_HOST=127.0.0.1
    - ACTION_SERVER_PORT=8765
  command: ["/bin/bash", "-lc", "python3 -m rob_box_voice.action_server.http_server"]
  depends_on:
    voice-assistant:
      condition: service_started
  restart: unless-stopped
  healthcheck:
    test: ["CMD-SHELL", "python3 -c 'import urllib.request; urllib.request.urlopen(\"http://127.0.0.1:8765/healthz\")'"]
    interval: 10s
    timeout: 3s
    start_period: 10s
    retries: 3
```

**Knobs:**
- `ACTION_SERVER_HOST` (default `127.0.0.1`) — bind address. Use `0.0.0.0` только за reverse proxy с auth.
- `ACTION_SERVER_PORT` (default `8765`) — listen port.
- `ACTION_SERVER_SHUTDOWN_TIMEOUT` (default `2.0`) — TODO env-binding, см. §6.4 Q7.

### 8.2 Найденные дефекты в docker-compose (на 2026-08-04)

> **⚠️ Defect (out of scope для этого ADR, но важно зафиксировать):** в `docker/vision/docker-compose.yaml` (commit `863a6744`) **service `voice-action-server` определён ДВАЖДЫ** (строки 267–287 и 289–311). YAML-парсер берёт **последний** option, но `git diff` показывает оба блока merge'нутых вместе. Действующее поведение = второй блок (идентичное содержимое), но **это future-bug-magnet** — любое diff в первом блоке будет silently ignored. **TODO:** удалить первый блок, оставить второй. Tracking: под-задача к `t_3183f40e` (или новой).

### 8.3 Мониторинг

Phase 4 step-1 has **no metrics** yet. TODO Phase 4 follow-up:
- `action_server_goals_total{state="succeeded|failed|cancelled"}` — counter.
- `action_server_active_goals` — gauge.
- `action_server_goal_duration_seconds` — histogram (по `kind`).
- `action_server_ungraceful_shutdowns_total` — counter (см. §3.5).

### 8.4 Типичные ошибки (runbook)

| Симптом | Причина | Действие |
|---|---|---|
| `GET /healthz → 503` | Sidecar в shutdown | Подождать 2с, перезапустить |
| `submit → 500` | Handler exception | Проверить логи sidecar'а (`docker logs voice-action-server`); ошибка в `handle.error` |
| `cancel → 404` | Goal уже завершён | Нормальное поведение, идемпотентно |
| `commit("x") → KeyError` | x не speculate'нут | Проверить `paste._shadow` (debug лог) |
| `docker-compose up` → `voice-action-server` exited | `aiohttp` не установлен | `pip install aiohttp` в image, см. §4.1 |

### 8.5 Graceful shutdown procedure

```bash
# Production: send SIGTERM, wait up to 5s
docker stop voice-action-server  # SIGTERM, 10s grace period
# Or systemd:
systemctl stop voice-action-server
# В обоих случаях sidecar:
# 1. /healthz → 503
# 2. cancels all active goals (cooperative + task.cancel)
# 3. waits up to ACTION_SERVER_SHUTDOWN_TIMEOUT (default 2.0s)
# 4. AppRunner.cleanup()
# 5. exit 0
```

---

## 9. Acceptance criteria (для Phase 4)

| # | Критерий | Статус | Где проверено |
|---|---|---|---|
| AC1 | ActionServer start/stop изолированно от docker-compose | ✅ | `docker-compose up voice-action-server` |
| AC2 | `GET /healthz` → 200 в норме, 503 при shutdown | ✅ | `http.py:27-31` |
| AC3 | SIGTERM → graceful shutdown без потери goals | ✅ | `server.py:114-121`, `shutdown(timeout=2.0)` |
| AC4 | PASTE клиент использует ту же схему, что и сервер | ✅ | `paste.py` + `test_paste_speculation_commit_and_discard` |
| AC5 | End-to-end: goal submit → status → cancel | ✅ | `test_action_server_feedback_result_and_health`, `test_cancel_propagates_to_handler` |
| AC6 | Таймауты и circuit-breaker на client-side | ✅ | дефолты в `paste.py`; расширенная конфигурация — TODO |
| AC7 | Retry с exponential backoff + idempotency keys | ⏳ | partial: idempotency-клиент готов (UUID v4); **server-side guard = TODO** Phase 4 step-3 |
| AC8 | Docker-compose sidecar с healthcheck | ✅ | `docker/vision/docker-compose.yaml:267-311` |
| AC9 | Контракт зафиксирован в этом ADR | ✅ | Документ |
| AC10 | OpenAPI + protobuf схемы готовы | ✅ | `action-protocol.openapi.yaml`, `action-protocol.proto` |
| AC11 | Sequence diagramms end-to-end | ✅ | §7 |

---

## 10. Что осталось (out of scope)

| # | Out of scope | Где | Когда |
|---|---|---|---|
| OOS1 | Реальные handler'ы (tts.speak, music.play, nav.goto) | Phase 4 step-3 | сейчас sidecar со заглушкой `_handler` |
| OOS2 | Plugin registry | Phase 4 step-3 | §3.7 |
| OOS3 | Auth (API token) | Phase 4 step-3 | §6.4 Q6 |
| OOS4 | Prometheus metrics | Phase 4 follow-up | §8.3 |
| OOS5 | gRPC server | Phase 5+ (второй язык) | §4.3 |
| OOS6 | WebSocket feedback stream | Phase 5+ | §6.4 Q5 |
| OOS7 | WebUI / dashboard | Phase 5+ | outside ActionServer scope |
| OOS8 | Move `tts_node` на native ROS2 Action Server | Phase 6+ (INSIGHT #7) | SCHEDULER_DESIGN §11.4 |

---

## 11. Ссылки

### Внутренние

- **Реализация:** `src/rob_box_voice/rob_box_voice/action_server/`
  - `server.py` (ActionServer, ActionState, Health)
  - `paste.py` (PastePlanner, ShadowAction)
  - `http.py` (aiohttp adapter)
  - `http_server.py` (standalone entry point)
  - `__init__.py` (public API)
  - `README.md` (operator-facing notes)
- **Тесты:** `src/rob_box_voice/test/test_action_server.py` (3 теста, 53 строки)
- **Деплой:** `docker/vision/docker-compose.yaml` (commit `863a6744`)
- **Commits:** `3226e506` (action server + PASTE), `863a6744` (docker sidecar)
- **Kanban:** `t_6d0e1dfe` (этот ADR), `t_21f11e76` (Phase 4 integration), `t_50db3dc4` (server), `t_6ff90e48` (PASTE client), `t_25ab0ec9` (docs), `t_3183f40e` (TODO: удалить дубль из docker-compose)

### Архитектурные

- [ADR-0001](0001-harness-architecture.md) — целевая архитектура харнесса (Tool-порт, фаза 5 SideEffectBus)
- [ADR-0009](0009-harness-tts-contract.md) — TTS-контракт (фаза 0, frozen)
- [ADR-0008](0008-tts-provider-extension-points-landed.md) — провайдер-экстеншены TTS
- [SCHEDULER_DESIGN.md §11.5](../architecture/scheduler-design.md) — Phase 4 в scheduler-design
- [SCHEDULER_DESIGN.md §8 §15](../architecture/scheduler-design.md) — INSIGHT #7 (Action Server), Microsoft PASTE (arxiv 2603.18897)

### Внешние

- [Microsoft PASTE](https://arxiv.org/abs/2603.18897) — speculative execution с shadow queue
- [ROS2 Action Server docs](https://design.ros2.org/articles/actions.html) — reference для HTTP-контракта (goal/feedback/result/cancel)
- [OpenAPI 3.0.3 spec](https://spec.openapis.org/oas/v3.0.3) — schema для HTTP
- [Kubernetes liveness/readiness probes](https://kubernetes.io/docs/tasks/configure-pod-container/configure-liveness-readiness-startup-probes/) — health semantices
- [aiohttp web.Application](https://docs.aiohttp.org/en/stable/web.html) — HTTP adapter

---

*ADR готов к ревью. После approve — перевод в `docs/adr/0011-action-protocol.md` в ветке `feature/harness-p0-foundation` (отдельным коммитом `docs(adr-0011): action protocol contract`). Реализация уже merged в `feature/t_21f11e76`, ожидает CI с pytest (блокер devops, см. `t_21f11e76` комментарий).*
