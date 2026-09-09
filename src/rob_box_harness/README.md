# rob_box_harness — Harness Framework (ADR-0001 P0)

A thin Python framework that gives every rob_box ROS2-adjacent
workflow — **Dialog** (voice), **Persistent** (audio/stt/tts/sound/led/cmd),
**Telegram** (chat bot) — a uniform contract:

* **Ports** — LLM, tools, memory, side-effects, transport, clock.
* **Lifecycle** — `__init__` / `init` / `run` / `teardown`, idempotent
  at every stage, async-context-manager friendly.
* **Registry** — name → builder, with built-in dummy harnesses
  (`echo`, `upper`) for smoke tests.
* **Config** — YAML + `${ENV_VAR}` placeholder interpolation, layered
  files for env overrides.
* **Single entry point** — `run_harness(name, input, config)`.

The framework sits on top of [`rob_box_llm`](../rob_box_llm) and
follows the design spelled out in
[`docs/adr/0001-harness-architecture.md`](../../docs/adr/0001-harness-architecture.md).

## Why a framework at all?

Today, three ROS2 nodes share the same burden: building an LLM
client, talking to MCP tools, remembering the user's history,
replying in TTS / Telegram, handling wake-word gates, retrying
on transient errors. Each node does it differently, with
different test coverage, different bug surfaces.

The harness is the **shared contract** for all of that:

* One **Harness** class, one **lifecycle**, one **state machine**.
* Five **ports** (LLM / Tools / Memory / Effects / Transport)
  wired in via DI; concrete implementations are swappable.
* **Tests** can drop in fakes for every port — no ROS2, no
  network, no Telegram bot.

## Quick start

```python
from rob_box_harness import run_harness_sync

# Run a built-in dummy harness (deterministic, in-memory).
result = run_harness_sync("echo", "hello world")
print(result.output)
# -> "echo: hello world"
```

```python
import asyncio
from rob_box_harness import (
    HarnessConfig, run_harness,
    RecordingBus, get_default_registry, HarnessFactory,
)

# Run async, with a custom side-effect bus.
async def go():
    config = HarnessConfig.from_dict(
        {"harness": {"kind": "echo", "name": "smoke"}}
    )
    bus = RecordingBus()
    reg = get_default_registry()
    HarnessFactory.reset_cache()
    harness = HarnessFactory.create("echo", config, reg)
    harness.effects = bus
    async with harness as h:
        result = await h.run("hello async")
    print(result.output)
    # -> "echo: hello async"
    print([e.text for e in bus.effects])
    # -> ["echo: hello async"]

asyncio.run(go())
```

```yaml
# harness.config.yaml — production-shaped config
harness:
  kind: dialog
  name: dialog_main
  state:
    wake_active: true
    silenced_until: null
llm:
  provider: deepseek
  model: deepseek-chat
  fallback: [minimax, mimo]
  settings:
    temperature: 0.7
    max_tokens: 1024
    timeout_s: 30
tools:
  provider: mcp_bridge
  endpoint: /mcp/execute
memory:
  backend: sqlite
  path: ~/.rob_box/voice.db
effects:
  bus: composite
  composite: [tts, sound, led, telegram_reply]
transport:
  kind: ros2
  topics:
    stt_result: /voice/stt/result
    vad: /audio/vad
    telegram: /telegram/updates
logging:
  level: INFO
  redact: [MINIMAX_API_KEY, DEEPSEEK_API_KEY]
  format: structured
```

```python
import os
from rob_box_harness import load_config

# ${MINIMAX_API_KEY} is resolved from os.environ at load time.
config = load_config("harness.config.yaml")
print(config.llm.provider, config.llm.fallback)
```

## Built-in dummy harnesses

The framework ships with two smoke-test harnesses:

* **`echo`** — round-trips the input through the LLM and returns
  the response verbatim. Triggers a synthetic tool call when the
  user says `"ping"`, exercising the tool-execution path.
* **`upper`** — same as `echo` but uppercases the response.
  Demonstrates that adding a new harness is a 30-line affair.

The two share a single helper in
`rob_box_harness.harnesses._base.run_request_response_loop`; the
only difference is the post-processor (`str.upper` vs identity).

## Architecture (one screen)

```
┌─────────────────────────────────────────────────────┐
│  Adapters (Dialog / Persistent / Telegram nodes)    │
│           │ uses                                    │
│           ▼                                         │
│  Harness (Harness[StateT])                          │
│    • state: StateT                                  │
│    • hooks: LifecycleHooks                          │
│    • llm:     LLMProvider (port)                    │
│    • tools:   ToolProvider  (port)                  │
│    • memory:  MemoryStore   (port)                  │
│    • effects: SideEffectBus  (port)                 │
│    • transport: Transport   (port)                  │
│    • clock:   Clock         (DI for tests)          │
│                                                     │
│  Lifecycle:                                         │
│    __init__  (config only — no I/O)                 │
│      ↓                                              │
│    init()    (idempotent — build ports)             │
│      ↓                                              │
│    run(input) → step(input) → HarnessRunResult      │
│      ↓                                              │
│    teardown() (idempotent — close ports)            │
└─────────────────────────────────────────────────────┘
           │
           ▼
   Single entry point:
   run_harness(name, input, config) -> HarnessRunResult
```

## Public surface

The full list of public symbols is in
`rob_box_harness/__init__.py.__all__` (57 items). Top-level
groups:

| Group           | Highlights |
|-----------------|------------|
| Lifecycle       | `Harness`, `HarnessRunResult`, `LifecycleHooks`, `SessionSnapshot` |
| Config          | `HarnessConfig`, `load_config`, `LLMConfig`, `ToolsConfig`, `MemoryConfig`, `EffectsConfig`, `TransportConfig`, `LoggingConfig` |
| Registry        | `HarnessRegistry`, `HarnessFactory`, `register_builtin_harnesses` |
| Runner          | `run_harness`, `run_harness_sync`, `get_default_registry`, `reset_default_registry` |
| Ports           | `Clock` / `SystemClock` / `MockClock`, `Transport` / `FakeTransport`, `ToolProvider` / `FakeToolProvider`, `MemoryStore` / `InMemoryStore`, `SideEffectBus` / `NoopBus` / `RecordingBus` / `CompositeBus` |
| Built-in prov.  | `DummyLLMProvider`, `HarnessFakeLLMProvider` (= `FakeLLMProvider`) |
| Errors          | `HarnessError`, `ConfigError`, `HarnessNotFoundError`, `HarnessStateError`, `ProviderNotFoundError`, `HookError` |

## Testing

```bash
cd src/rob_box_harness
pip install -e .[dev]
pip install -e ../rob_box_llm
pytest
# 88 tests, ~90% line coverage
```

For strict type checking:

```bash
mypy rob_box_harness/
# Success: no issues found in 20 source files
```

## What this ADR's P0 deliverable does **not** include

* Real Dialog / Persistent / Telegram harnesses — those land in P1.
  The framework is intentionally thin so the per-node work focuses
  on the node-specific transport and side-effects, not the
  lifecycle plumbing.
* Network-backed `LLMProvider`s (DeepSeek / MiniMax / MiMo) — the
  harness accepts any `LLMProvider` from `rob_box_llm`; nothing to
  add here.
* Persistent memory — only `InMemoryStore` ships in P0. SQL / Redis
  implementations will be added when the first real harness needs
  them.
* Real `Transport` (ROS2 / Telegram) — only `FakeTransport` ships
  in P0.

## See also

* `docs/adr/0001-harness-architecture.md` — full architecture
  decision record (the source of truth for this module's design).
* `docs/refactoring-plan.md` — tactical implementation plan that
  this module executes.
* `src/rob_box_llm` — the LLM / TTS provider package that this
  module composes on top of.
