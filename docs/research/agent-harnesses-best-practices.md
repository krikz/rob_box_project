# Best practices харнесов в агентских системах

**Дата:** 2026-07-20
**Статус:** Draft (v1)
**Автор:** architect (Hermes Agent)
**Контекст:** Kanban-задача `t_e23474f4` (parent для задач `t_6c0600d7`, `t_ace66f51`).
**Связанные документы:** `docs/adr/0001-harness-architecture.md`, `docs/refactoring-plan.md`, `src/rob_box_core/README.md`, `src/rob_box_llm/README.md`.
**Назначение:** справочник паттернов, который будут использовать следующие этапы планирования (P1/P2). Документ описывает **как устроены современные харнесы**, а не как мы их реализуем — наша реализация зафиксирована в ADR-0001.

---

## 0. Что мы понимаем под «харнесом»

«Harness» (a.k.a. agent runtime / orchestrator / runner) — это **тонкая обёртка вокруг LLM-агента**, которая:

1. Владеет **жизненным циклом** сессии (start → loop → stop).
2. Держит **контракт** внешних зависимостей (LLM, инструменты, транспорт, память) через **порты**.
3. Маршрутизирует **события** и **побочные эффекты** через единый механизм (а не через ad-hoc вызовы).
4. Даёт **observability и testability**: snapshot, replay, hooks, deterministic clocks.

Граница между «харнесом» и «фреймворком» условна: Hermes Agent — это полноценный CLI-фреймворк, но его внутренний `run_agent.py` + `tools/registry.py` играют роль харнеса для одной сессии. LangGraph/CrewAI — это харнесы с акцентом на граф. OpenAI Agents SDK — компактный харнес вокруг одной «Runner»-петли.

---

## 1. Целевая «форма» харнеса (как она выглядит у всех референсов)

Несмотря на разные стили, у всех зрелых харнесов есть одинаковый **скелет**:

```
┌──────────────────────────────────────────────────────────────────┐
│  HARNESS                                                          │
│  ┌────────────────┐  ┌─────────────────┐  ┌──────────────────┐   │
│  │ State container│  │ Event / message │  │ Side-effect bus  │   │
│  │ (TypedDict /   │  │ bus (chunk /    │  │ (dispatcher /    │   │
│  │  dataclass)    │  │  tool_call)     │  │  joiner)         │   │
│  └───────┬────────┘  └────────┬────────┘  └─────────┬────────┘   │
│          └────────────────────┴──────────────────────┘            │
│                              │                                   │
│                              ▼                                   │
│                     ┌────────────────────┐                       │
│                     │ Agent loop / node  │                       │
│                     │ (decide → call     │                       │
│                     │  tool → emit)      │                       │
│                     └─────────┬──────────┘                       │
│                               │ uses                             │
│                               ▼                                  │
│  PORTS (swappable adapters):                                     │
│    • LLMProvider  (DeepSeek / MiMo / OpenAI / mock)              │
│    • ToolExecutor (MCP / local skill / fake)                      │
│    • MemoryStore  (SQLite / Redis / in-mem)                      │
│    • Transport    (ROS2 / TG / Slack / keyboard)                 │
│    • Clock        (system / mock)                                │
└──────────────────────────────────────────────────────────────────┘
```

**Главная идея**: доменная логика не должна знать, *где* она запущена. Тест меняет `Transport` на fake и проверяет только маршрутизацию; интеграция подменяет `LLMProvider` на реальный DeepSeek.

---

## 2. Референсы (сводный разбор)

Для каждой системы — короткий профиль по **одному формату**: жизненный цикл ноды/агента, контекст, маршрутизация, observability, extensibility.

### 2.1 Hermes Agent (Nous Research) — внешний референс для паттернов

Hermes Agent — сам «продукт», внутри которого мы запускаемся; его собственный харнес — это образец «тонкого runner-а над LLM + tools».

| Аспект | Что у Hermes | Полезный паттерн |
|--------|--------------|------------------|
| **Жизненный цикл** | Один вызов `run_agent.py` = одна сессия; loop «model → tool → model», пока не вернётся `final_output` или не сработает gate. | **Single-runner model**: один явный `Runner` отвечает за весь loop. Не «класс-агент, который сам себя гоняет». |
| **Контекст** | `tools/registry.py` — централизованный реестр с self-registration при импорте; 70+ инструментов в 28+ toolsets. | **Capability registry + import-time registration**: skill/tool регистрируется модулем-импортом, runner-у остаётся только вызвать `registry.get(name)`. |
| **Маршрутизация** | `run_agent.py` интерсептит «agent-level» tools (todo, memory) до `handle_function_call()`; остальное — через реестр. | **Pre-/post-hooks на tool execution**: спец-инструменты могут быть обработаны не как функции, а как действия runner-а. |
| **Observability** | Трейсы сессий (ShareGPT-формат для обучения), JSON run logs, опциональный ACP-editor. | **Сериализуемая session-история** — снапшот нужен не только для тестов, но и для трейнинга и UI. |
| **Extensibility** | Плагины 4-х типов (tools, memory, context engine, gateway hooks). Single-select для memory/context, multi-select для tools/gateway hooks. | **Plugin slots с явной семантикой** (single vs multi). Не «всё один большой extension point». |
| **Provider abstraction** | `runtime_provider` — единый resolver (provider, model) → (api_mode, api_key, base_url); 18+ провайдеров. | **Разделить «как подключиться» (provider) и «как вызвать» (port contract)**. |
| **Threaded bridge** | `gateway/` — мост между внешним миром (TG, Discord) и внутренним asyncio-loop; hooks на входящие/исходящие. | **Именованный мост** «внешний loop ↔ внутренний runner», а не прямое обращение к LLM из обработчика платформы. |

**Что мы у Hermes уже взяли** (см. ADR-0001): capability registry, lifecycle hooks, provider abstraction, side-effect wrapper, мост «platform loop ↔ runner».

**Источники:** Hermes AGENTS.md (`tools/registry.py`, `run_agent.py`), docs/developer-guide/architecture.

### 2.2 OpenAI Agents SDK — компактный «Runner-loop» харнес

Это харнес, на котором **уже построен** наш `dialogue_node.py` (через `agents` SDK). Берём как образец «простого runner-а».

| Аспект | Что у OpenAI Agents SDK |
|--------|-------------------------|
| **Жизненный цикл** | `Runner.run(agent, input)` → loop: вызвать model → если есть tool_call, выполнить и подать результат обратно → повторять. Два терминальных исключения: `MaxTurnsExceeded`, `GuardrailTripwireTriggered`. |
| **Контекст** | `input` — список сообщений; `RunContext` — кастомный контекстный dataclass, который пробрасывается во все tools/hooks. |
| **Маршрутизация** | **Handoffs** — модель может вернуть «передать управление агенту X», и runner продолжает loop уже от имени X. Внутри одного run можно менять «текущего агента». |
| **Observability** | Built-in tracing (экспорт в OpenAI dashboard или сторонний backend), structured `RunResult` с историей. |
| **Extensibility** | `function_tool` decorator + JSON-schema tools + `input_guardrails` / `output_guardrails` / `tool_guardrails`. |
| **Retry/error** | SDK не навязывает retry; пользователь решает через `tool` decorator или wrapper. |

**Полезное для нас**:
- **«Runner-loop» как отдельная сущность** — не класс-агент, а функция `Runner.run`. Это снимает дилемму «у нас DialogNode и TelegramNode — два разных loop-а» (на самом деле loop один, адаптеры разные).
- **Handoffs как способ делегировать между skill-ами**, не делая skill самостоятельным агентом со своим loop-ом.
- **Guardrails в три слота** (input/output/tool) — точно наш случай для wake-word gate и «помолчи».
- **Структурированное исключение при исчерпании шагов** (`MaxTurnsExceeded`) — даёт детерминированный лимит вместо «висим и надеемся».

**Антипаттерн (явный в SDK)**: SDK не имеет checkpoint/replay из коробки; для продакшна нужен свой wrapper.

**Источники:** `openai.github.io/openai-agents-python/running_agents/`, `ref/run/`, `guides/guardrails/`.

### 2.3 LangGraph — «граф состояний» харнес

| Аспект | Что у LangGraph |
|--------|-----------------|
| **Жизненный цикл** | `StateGraph` компилируется в runnable; `invoke` / `stream` / `astream_events` запускают прогон. На каждом node — checkpoint. |
| **Контекст** | `State` — `TypedDict` с reducer'ами (`Annotated[list[str], operator.add]`). Каждый node возвращает **частичный** update, reducer мерджит. |
| **Маршрутизация** | Условные edges (`add_conditional_edges`) → маршрут выбирается функцией от state. `Command(goto=...)` — императивный переход из ноды. |
| **Observability** | Checkpointer (`InMemorySaver`, `PostgresSaver`) сохраняет state после каждой ноды; `get_state_history()` — time-travel. |
| **Extensibility** | `interrupt()` — точка приостановки для human-in-the-loop; после resume — `Command(resume=...)`. |
| **Retry/error** | Внутри ноды — try/except; на уровне графа — `retry_policy=RetryPolicy(max_attempts=3)`. |

**Полезное для нас**:
- **Идея частичных апдейтов state + reducer'ов** — переехала в наш `MemoryStore`/`DialogueStateMachine` (state не перезаписывается целиком, апдейт точечный).
- **`interrupt()` для human-in-the-loop** — это и есть наш wake-word gate / команда «помолчи / не отвечай»: модель сгенерила ответ, но runner ждёт подтверждения, прежде чем публиковать.
- **Checkpoint per node** — модель того, как мы делаем snapshot в `AgentSession`: после каждой ноды сохраняем, можно откатить.

**Что у LangGraph НЕ подходит нам**:
- **Граф как «главный» artifact** — у нас request-response со стримом, а не DAG. Если затянуть граф, получится лишний слой конфигурации.
- **Reducer'ы через `Annotated[…, operator.add]`** — элегантно, но Python-typing-тяжело; мы берём идею, но реализуем через явные `set_state_field` / reducer-методы, чтобы не зависеть от `typing_extensions`.

**Источники:** `docs.langchain.com/oss/python/langgraph/interrupts`, `eastondev.com/blog/en/posts/ai/20260424-langgraph-agent-architecture/`, `kalviumlabs.ai/blog/langgraph-in-production-stateful-multi-step-agents/`.

### 2.4 Microsoft AutoGen — «распределённый runtime с сообщениями»

| Аспект | Что у AutoGen |
|--------|---------------|
| **Жизненный цикл** | Agent регистрируется в `AgentRuntime`; runtime владеет lifecycle (publish, subscribe, shutdown). `AssistantAgent.on_messages()` — основной обработчик. |
| **Контекст** | `MessageContext` (sender, topic_id, cancellation_token); сообщения — typed (`TextMessage`, `ToolCallMessage`...). |
| **Маршрутизация** | Topic-based pub/sub; `RoutedAgent` подписывается на типы сообщений; `GroupChat` orchestration выбирает «кто говорит следующим» (round-robin, prompt-based, custom). |
| **Observability** | Cancellation tokens, structured traces; runtime logs (gRPC / single-process). |
| **Extensibility** | Локальный (`SingleThreadedAgentRuntime`) или распределённый (`GrpcAgentRuntime`) — переключается. |
| **Retry/error** | `max_tool_iterations` лимитирует внутренние итерации; `cancellation_token` останавливает кооперативно. |

**Полезное для нас**:
- **Разделение «agent как код» vs «runtime как инфраструктура»**: у нас `AgentSession` — это код, ROS2-нода — runtime. Сейчас они перемешаны, и это источник проблемы #4 из ADR (side-effects в ROS2).
- **Cooperative cancellation через token** — если пользователь в TG нажал /cancel, нужно уметь остановить текущий LLM-stream, не убивая процесс. Прямо переносим как `CancellationToken` в `AgentSession`.
- **`max_tool_iterations`** — защита от бесконечного tool-loop; у нас должна быть аналогичная в `ToolExecutor`.

**Что у AutoGen НЕ подходит нам**:
- **Distributed gRPC runtime** — overkill для одного робота на RPi. Но идею «runtime != agent» берём.
- **Topic-based pub/sub как основа** — у нас ROS2-топики и так дают pub/sub; ещё один слой тем избыточен.

**Источники:** `microsoft.github.io/autogen/stable/user-guide/core-user-guide/framework/agent-and-agent-runtime.html`, `Agents` tutorial.

### 2.5 CrewAI — «роли + процесс» харнес

| Аспект | Что у CrewAI |
|--------|--------------|
| **Жизненный цикл** | `Crew` собирает `Agent`-ов (role, goal, backstory, tools) и `Task`-и; `Process` (sequential / hierarchical / consensual) определяет порядок. `kickoff()` запускает. |
| **Контекст** | `Task.output` передаётся следующему task-у; есть `CrewMemory` (short/long/entity). |
| **Маршрутизация** | В `hierarchical` — manager LLM решает, кому делегировать; в `sequential` — фиксированный порядок. |
| **Observability** | `step_callback`, `task_callback`, опциональный `Langfuse`/`OpenTelemetry`. |
| **Extensibility** | `BaseTool` + `@tool` decorator; `CrewAI Flows` (декларативные state-machine поверх Crew). |
| **Retry/error** | На уровне tool — свой код; на уровне crew — `max_rpm` (rate limit guard). |

**Полезное для нас**:
- **Идея «роль = агент» и «задача = декларативный объект»** — переехала в наш `BaseSkill` + `@function_tool`. У нас role уже не нужна (один робот, один voice), но декларативность задач — берём.
- **`CrewAI Flows`** (новый слой поверх Crew) — пример «state-machine над multi-agent loop». Если у нас появится scenario с разветвлениями (map / backup / silent), это ближайший шаблон.
- **`max_rpm` как глобальный rate-limit guard** — нужно и нам: сейчас при шторме TG-сообщений можем сжечь квоту DeepSeek.

**Антипаттерны CrewAI** (часто упоминаются в сообществе):
- **Verbose-логи по умолчанию** — скрывают реальный ход событий. У нас: structured `Effect`-логи, не «agent_thought: I will now…».
- **«Авто-подбор tools» по описанию роли** — звучит удобно, но в проде даёт неконтролируемые tool-call-ы. У нас: явный `CapabilityRegistry` (какие skill-ы доступны какому каналу).
- **Manager-LLM в hierarchical** — лишний round-trip + лишние токены; для нашего размера задач overhead не оправдан.

**Источники:** `docs.crewai.com/concepts/crews`, `agentscookbook.com/docs/tutorial/crewai/crewai-multi-agent-workflows/`.

### 2.6 Haystack — «pipeline-компоненты + routing»

| Аспект | Что у Haystack |
|--------|-----------------|
| **Жизненный цикл** | Pipeline = DAG из `Component`-ов с `run()` / `async_run()`. `SuperComponent` инкапсулирует под-pipeline. |
| **Контекст** | Data flow через `Document`/`ChatMessage`/custom dataclass; `Component.joiners` собирают ветки. |
| **Маршрутизация** | `ConditionalRouter` (правила на data), `BranchJoiner` (выбрать первый не-None input), `Router` (по эмбеддингам). |
| **Observability** | Tracing через OpenTelemetry; `Pipeline.get_components_graph()` — визуализация. |
| **Extensibility** | `Component` — маленький ABC: `run()` + `async_run()` + `OutputAdapter` для маппинга. |
| **Retry/error** | `ComponentError`; retry на уровне pipeline-runner-а (`max_retries`). |

**Полезное для нас**:
- **`OutputAdapter` как явный маппер `data → effect`** — это наш `Effect[T]` / `SideEffectBus.dispatch(effect)`. У Haystack он декларативный, у нас будет аналогичный tagged-union.
- **`BranchJoiner`** — паттерн «несколько источников эффектов, один получатель». Для нашего `CompositeBus([TTSBus, SoundBus, LEDBus, TelegramBus])` — ровно это.
- **«Маленький ABC»** (`run` / `async_run`) — проще, чем большой `BaseSkill` с десятком методов.

**Что у Haystack НЕ подходит**:
- **DAG-first мышление** — у нас request-response, не DAG. Haystack pipelines длинные (5–10 узлов), у нас цикл короткий (model → tool → model, 1–5 итераций).
- **`Component.run()` без типизированного state** — теряем преимущества reducer'ов LangGraph.

---

## 3. Сводная таблица сравнения

| Аспект | Hermes Agent | OpenAI Agents SDK | LangGraph | AutoGen | CrewAI | Haystack |
|--------|--------------|--------------------|-----------|---------|--------|----------|
| **Loop / Runner** | `run_agent.py` | `Runner.run()` | `StateGraph` + invoke | runtime-hosted | `Crew.kickoff()` | `Pipeline.run()` |
| **Контекст** | `RunContext` + tools-registry | `RunContext` dataclass | `State` + reducer'ы | `MessageContext` + token | Task outputs + Memory | Data flow + `Component` |
| **Маршрутизация** | tools-registry + pre-hooks | handoffs (model-driven) | conditional edges | pub/sub + GroupChat | process type | ConditionalRouter + Joiner |
| **Async-first** | да | да | да | да | частично | да |
| **Human-in-the-loop** | gateway hooks | guardrails | `interrupt()` | runtime-cancel | task_callback | нет |
| **Checkpoint/replay** | session-logs (ShareGPT) | нет | `MemorySaver` / `PostgresSaver` | нет | callbacks | OpenTelemetry trace |
| **Retry policy** | per-tool | нет встроенного | per-node `RetryPolicy` | `max_tool_iterations` | `max_rpm` | `max_retries` |
| **Streaming chunks** | да | да (events) | `astream_events` | да | да | да |
| **Tool abstraction** | `registry.register()` + `check_fn` | `function_tool` | python-функция | `FunctionTool` | `@tool` | `Component` |
| **Extensibility slots** | 4 типа plugins | guardrails + handoffs | reducer + node | runtime + agent type | flows + memory | custom Component |
| **Онтология «skill»** | каждый tool = skill | `@function_tool` | `@tool` decorator | `FunctionTool` | `@tool` | `Component.run` |
| **Side-effect point** | tool returns / hook | tool return | node return | tool return | tool return | `OutputAdapter` |
| **Multi-agent** | через tools/handoffs | через handoffs | через graph | first-class | first-class | через sub-pipelines |
| **Распределённость** | single-process | single-process | single-process | gRPC-runtime | single | single |
| **Нагрузка под которую заточен** | CLI-агент, 1 юзер × 1 сессия | любой request | сложные графы | масштаб runtime | ролевая декомпозиция | RAG-pipeline |

---

## 4. Паттерны, применимые к нам

Выбираем с фильтром: «у нас ~30 RPS, 1 юзер, RPi, ROS2, нужно переиспользовать между Dialog и Telegram, нельзя добавлять distributed runtime».

### 4.1 Применимые паттерны (приоритет ↑)

| # | Паттерн | Где встречается | Что даёт нам | Применяется в фазе |
|---|---------|-----------------|--------------|---------------------|
| **P1** | **Ports & Adapters** (`LLMProvider`, `ToolExecutor`, `MemoryStore`, `SideEffectBus`, `Transport`, `Clock`, `SnapshotStore`) | Hermes, LangGraph, Haystack | Тестируемость без поднятия ROS2/LLM; единая точка подмены | **уже сделано P0** (`rob_box_llm`, `rob_box_core`) |
| **P2** | **Capability registry + import-time registration** | Hermes `tools/registry.py` | Skill-ы регистрируются декларативно; легко скрыть/показать по каналу | **P1** (Skill registry + TelegramCommandRegistry) |
| **P3** | **Runner-loop как отдельная сущность** | OpenAI Agents SDK, Hermes | `AgentSession.on_user_input(...)` — единственная точка входа для всех адаптеров | **P1** (`AgentSession`) |
| **P4** | **Side-effect bus с tagged-union эффектами** | Haystack `OutputAdapter`, Hermes tools | Единственная точка публикации TTS/Sound/LED/TG-reply; NoopBus для тестов | **P1** (`SideEffectBus`, `Effect`) |
| **P5** | **Handoffs между skill-ами** (не между агентами) | OpenAI Agents SDK | Skill может передать управление другому skill-у без отдельного loop | **P1** (через `AgentSession.skill_chain`) |
| **P6** | **Guardrails по слотам** (input/output/tool) | OpenAI Agents SDK | Wake-word gate = input guardrail; «помолчи» = output guardrail; tool guardrail = «не вызывай sound после 23:00» | **P1–P2** (через `hooks`) |
| **P7** | **Cooperative cancellation token** | AutoGen | TG /cancel останавливает stream, не убивая процесс | **P1** (`AgentSession.cancel()`) |
| **P8** | **Snapshot/replay** для тестов и observability | LangGraph checkpointer, Hermes ShareGPT | `AgentSession.snapshot() → JSON`; round-trip через `replay()` | **P2** (snapshot/observability) |
| **P9** | **Threaded bridge «platform loop ↔ runner»** | Hermes gateway, AutoGen runtime | TG-handler не зовёт LLM напрямую — кладёт событие в очередь runner-а | **P1** (`TelegramHarness` transport) |
| **P10** | **`max_tool_iterations` / `max_turns`** | AutoGen, OpenAI Agents SDK | Детерминированный лимит вместо «висим и надеемся» | **P1** (в `AgentSession`) |
| **P11** | **Explicit state machine с transition table** | LangGraph (через reducer'ы), Haystack routing | `DialogueStateMachine` уже сделан; используем как источник правды | **P0 сделано, P1 — wire** |
| **P12** | **In-memory fake для каждого порта** | все референсы | `FakeLLMProvider`, `InMemoryStore`, `MockClock` — уже есть | **уже сделано P0** |
| **P13** | **Streaming chunks + structured events** | Hermes, OpenAI Agents SDK | `LLMChunk` → `Effect` поток; видим токены в TTS без ожидания конца | **P1** (в `AgentSession.stream`) |
| **P14** | **Plugin slots с явной семантикой** (single vs multi) | Hermes | `CapabilityRegistry` показывает/скрывает skill по каналу | **P1–P2** |
| **P15** | **Provider resolver** (provider, model) → (api, key, base_url) | Hermes `runtime_provider` | Уже есть частично в `rob_box_llm` (DeepSeekProvider / MiMoProvider / FakeLLMProvider); в P1 добавить `FallbackProvider` | **P1** |

### 4.2 НЕ применимые паттерны (осознанно отвергаем)

| # | Паттерн | Почему НЕ берём | Когда пересмотреть |
|---|---------|-----------------|---------------------|
| **A1** | **Graph-of-agents как первый класс** (LangGraph `StateGraph`) | У нас request-response, а не DAG; один runner, один loop. Дополнительный слой конфигурации не окупается. | Если появятся сценарии с разветвлениями: «спросить разрешения → исполнить → подтвердить» в одной сессии. |
| **A2** | **Event Sourcing (append-only log событий)** | На текущей нагрузке (1 юзер × 1 робот) — overkill. `MemoryStore` + `Snapshot` дают 90% пользы. | При мульти-юзер / мульти-робот / time-travel отладке. |
| **A3** | **CQRS (отдельные read/write модели)** | ~30 RPS, монолитная БД. Разнесение даст лишний код без выгоды. | При росте нагрузки на порядок. |
| **A4** | **Distributed gRPC runtime** (AutoGen `GrpcAgentRuntime`) | Один RPi, один процесс. Распределённость только мешает. | Вторая физическая платформа робота. |
| **A5** | **Manager-LLM в hierarchical Crew** | Лишний round-trip + лишние токены + непредсказуемость. | Когда у нас будет ≥3 skill-а с реальной конкуренцией за управление. |
| **A6** | **Topic-based pub/sub поверх ROS2** | У ROS2 уже есть топики; свой слой тем — лишний. | Когда ROS2-топики станут узким местом по latency. |
| **A7** | **Авто-подбор tools по описанию роли** | Неконтролируемые tool-call-ы в проде, невозможно тестировать. | Никогда (явный `CapabilityRegistry`). |
| **A8** | **Verbose «agent thought» логи** (CrewAI default) | Скрывают реальный ход событий. | Никогда (structured `Effect`-логи). |

---

## 5. Конкретные рекомендации для следующего этапа планирования

Все рекомендации укладываются в ADR-0001; этот документ лишь указывает, **откуда** взят каждый паттерн и почему именно так.

### 5.1 Для `AgentSession` (P1.1)

1. **Один метод-вход `on_user_input(text, source)` + `on_audio_chunk(...)` + `on_event(event)`** — паттерн P3 (OpenAI Agents SDK). Никакого `process()` — `on_*` явно разделяет типы входа.
2. **`LifecycleHooks`** — `on_start / on_turn_begin / on_tool_call / on_tool_result / on_response_chunk / on_error / on_stop` — паттерн из Hermes (gateway hooks) + AutoGen (message-handler lifecycle). Хуки — это список callable-ов, не subclassing.
3. **`max_turns` параметр** в `on_user_input` — паттерн P10. По умолчанию 5; при превышении — структурированное исключение `MaxTurnsExceeded`.
4. **`CancellationToken`** — паттерн P7 (AutoGen). Хранится в `RunContext`, проверяется в каждой точке loop-а.

### 5.2 Для `SideEffectBus` (P1.1)

1. **`Effect` — frozen-dataclass tagged union**: `Speak(text, ssml=None)`, `PlaySound(name)`, `SetLED(pattern)`, `SendReply(channel, text)`, `Move(vel)`, `Stop()` — паттерн P4 (Haystack `OutputAdapter`).
2. **`CompositeBus([...])` + `NoopBus` + `RecordingBus`** — паттерн Haystack `BranchJoiner` + Hermes test-fakes.
3. **Никаких ad-hoc `publish_*`**: всё через `effects.dispatch(...)` — паттерн «side-effect discipline» (Hermes tools).

### 5.3 Для `ToolExecutor` (P1.2)

1. **`ToolExecutor.execute(call, *, timeout=10.0) → ToolResult`** + async-вариант — паттерн из всех референсов: «у инструмента есть timeout, retry, единый формат ошибки».
2. **`MCPBridgeExecutor`** — обёртка над текущим `MCPBridge` + `LLMToolCallAdapter`. Не выкидываем существующий код, адаптируем под порт.
3. **`LocalSkillExecutor`** — для skill-ов, которые выполняются в Python-коде без MCP.
4. **Лимит `max_tool_iterations`** — паттерн P10.

### 5.4 Для `DialogHarness` (P1.3)

1. **ROS2-callback-и только**: `stt_callback / vad_callback / tts_finished_callback` → переводят ROS2-msg в `AgentSession.on_*`.
2. **Никакой бизнес-логики в ноде** — паттерн «thin ROS2 wrapper» (наша ADR).
3. **Feature flag `USE_HARNESS`** на время миграции — паттерн AutoGen runtime swap.

### 5.5 Для `TelegramHarness` (P1.5 / P2.4)

1. **`TelegramCommandRegistry`** (декларативный) вместо 25 handler-функций — паттерн P2 (Capability registry).
2. **`text_message_handler` → `AgentSession.on_user_input(text, source=TG)`** — паттерн P9 (bridge).
3. **CapabilityRegistry** скрывает skill-ы, нерелевантные TG (например, «поехали на кухню» через TG = плохая идея).
4. **`share_context=true` через opt-in** — паттерн P14 (явный plugin slot), по умолчанию OFF (приватность).

### 5.6 Для `PersistentHarness` (P2.1)

1. **`PersistentHarness` НЕ LLM-агент** — только lifecycle, clock, logger, state-publisher. Никаких `MemoryStore`/`LLMProvider`. Это и есть принцип «agent ≠ runtime» (AutoGen), но применённый к нашей «железной» части.
2. **`StatePublisher`** — единый формат `NodeState` (status, last_error, uptime_sec, metrics) на `/<node>/state_v2`, parallel publishing старого топика для backwards-compat.

### 5.7 Для observability (P2.5)

1. **`AgentSession.snapshot() → JSON`** — паттерн P8 (LangGraph time-travel + Hermes ShareGPT).
2. **Hooks `on_response_chunk` → `/<node>/trace`** — паттерн Hermes structured tracing.
3. **`replay(snapshot)`** для тестов и отладки — воспроизведение без реального LLM.

---

## 6. Что эта документ НЕ покрывает

- Реализация конкретных интерфейсов — это `docs/refactoring-plan.md` (фазы P0–P2).
- Конкретные ошибки текущего кода — `docs/analysis/current-nodes.md` (`t_0f7b815c`, ветка `feature/analysis-current-nodes`).
- Сравнение производительности фреймворков — нам нерелевантно (1 RPi, 1 юзер).
- Distributed runtime / multi-robot — осознанно вне scope.

---

## 7. Ссылки

| # | Источник | Что взяли |
|---|----------|-----------|
| 1 | Hermes Agent AGENTS.md (`tools/registry.py`, `run_agent.py`) | Capability registry, import-time registration, lifecycle hooks, threaded bridge |
| 2 | Hermes Agent docs/developer-guide/architecture | Plugin slots, runtime resolver, ShareGPT-формат трейсов |
| 3 | OpenAI Agents SDK — `running_agents`, `ref/run`, `guides/guardrails` | Runner-loop, handoffs, guardrails, max_turns |
| 4 | LangGraph — `interrupts`, state management, time-travel | Reducer'ы, checkpoint, interrupt() для human-in-the-loop |
| 5 | AutoGen — `agent-and-agent-runtime`, `agents` tutorial | Cooperative cancellation, max_tool_iterations, agent vs runtime |
| 6 | CrewAI — `concepts/crews`, multi-agent workflows | Декларативные Task, max_rpm, flows |
| 7 | Haystack — `conditionalrouter`, joiners, components | OutputAdapter, BranchJoiner, маленький ABC |

Внутренние ссылки: ADR-0001, план рефакторинга, README `rob_box_core`, README `rob_box_llm`.

---

*Готово к ревью. Документ служит входом для `t_6c0600d7` (PersistentHarness design) и `t_ace66f51` (DialogHarness implementation).*
