"""rob_box_harness — agent core, config and supporting ports.

The Harness Framework scaffolding (``Harness``, ``HarnessRegistry``/
``HarnessFactory``, ``run_harness``/``run_harness_sync``, the
``Transport`` port and the concrete Dialog/Persistent/Telegram/Echo/
Upper harnesses) was removed as dead code — ADR-0051 §3.2 (closes
ADR-0001 §2.7.1); see issue #1985. Production code runs on
:class:`~rob_box_harness.core.agent_core.AgentCore` instead.

What remains here:

  * :class:`HarnessConfig` / :func:`load_config` — YAML + env config.

Ports (provider contracts):

  * :class:`LLMProvider` — re-exported from ``rob_box_llm``.
  * :class:`ToolProvider` / :class:`FakeToolProvider` — tool execution.
  * :class:`MemoryStore` / :class:`InMemoryStore` — per-scope history.
  * :class:`SideEffectBus` / :class:`NoopBus` / :class:`RecordingBus` /
    :class:`CompositeBus` — fan-out of side-effects.
  * :class:`Clock` / :class:`SystemClock` / :class:`MockClock` — DI
    for time.

Errors live in :mod:`rob_box_harness.errors`. The exit-level
errors are :class:`HarnessError`, :class:`ConfigError`,
:class:`HarnessNotFoundError`, :class:`HarnessStateError`,
:class:`ProviderNotFoundError`, :class:`HookError`.

Public surface organised by concern:

Lifecycle:
    LifecycleHooks, Hook, SessionSnapshot

Config:
    HarnessConfig, LLMConfig, ToolsConfig, MemoryConfig,
    EffectsConfig, TransportConfig, LoggingConfig, load_config

Ports:
    Clock, SystemClock, MockClock,
    ToolProvider, FakeToolProvider, ToolSpec, ToolExecutionError,
    MemoryStore, InMemoryStore, Turn, Fact,
    SideEffectBus, NoopBus, RecordingBus, CompositeBus, Effect,
    EffectContext, LogEffect, EchoEffect
  * :class:`SendReplyEffect` / :class:`SpeakEffect` /
    :class:`PlaySoundEffect` / :class:`SetLEDEffect` /
    :class:`MoveEffect` / :class:`TelegramBus` /
    :class:`TelegramFilteredBus` — telegram-side extensions (P1.4).
  * :class:`SnapshotStore` / :class:`InMemorySnapshotStore` /
    :class:`TelegramUpdate` / :func:`parse_telegram_update` — port
    for camera/frame caches plus update-parsing helper (P1.4).

Built-in providers (for tests / smoke):
    DummyLLMProvider, HarnessFakeLLMProvider (= FakeLLMProvider)

Errors:
    HarnessError, ConfigError, HarnessNotFoundError,
    HarnessStateError, ProviderNotFoundError, HookError
"""

from __future__ import annotations

from rob_box_harness.clock import Clock, MockClock, SystemClock
from rob_box_harness.config import (
    EffectsConfig,
    HarnessConfig,
    HarnessKind,
    LLMConfig,
    LoggingConfig,
    MemoryConfig,
    ToolsConfig,
    TransportConfig,
    load_config,
)
from rob_box_harness.effects import (
    CompositeBus,
    EchoEffect,
    Effect,
    EffectContext,
    LogEffect,
    MoveEffect,
    NoopBus,
    PlaySoundEffect,
    RecordingBus,
    SendReplyEffect,
    SetLEDEffect,
    SideEffectBus,
    SpeakEffect,
    TelegramBus,
    TelegramChannel,
    effect_kind,
    from_dict,
    to_dict,
)
from rob_box_harness.errors import (
    ConfigError,
    HarnessError,
    HarnessNotFoundError,
    HarnessStateError,
    HookError,
    ProviderNotFoundError,
)
from rob_box_harness.health import (
    DEFAULT_HEALTH_TTL_S,
    HealthAwareFallbackLLM,
    HealthCache,
    HealthRecord,
    ProviderStatus,
    TRANSIENT_TTL_S,
    check_deepseek_balance,
    is_auth_failure,
    is_quota_exhausted,
)
from rob_box_harness.lifecycle import Hook, LifecycleHooks
from rob_box_harness.memory import Fact, InMemoryStore, MemoryStore, Turn
from rob_box_harness.providers import DummyLLMProvider, HarnessFakeLLMProvider
from rob_box_harness.snapshot import SessionSnapshot
from rob_box_harness.snapshot_store import (
    InMemorySnapshotStore,
    SnapshotEntry,
    SnapshotStore,
    TelegramUpdate,
    parse_telegram_update,
)
from rob_box_harness.tools import (
    FakeToolProvider,
    ToolExecutionError,
    ToolHandler,
    ToolProvider,
    ToolSpec,
)

__all__ = [
    # Lifecycle
    "LifecycleHooks",
    "Hook",
    "SessionSnapshot",
    # Config
    "HarnessConfig",
    "HarnessKind",
    "LLMConfig",
    "ToolsConfig",
    "MemoryConfig",
    "EffectsConfig",
    "TransportConfig",
    "LoggingConfig",
    "load_config",
    # Ports
    "Clock",
    "SystemClock",
    "MockClock",
    "TelegramUpdate",
    "ToolProvider",
    "FakeToolProvider",
    "ToolSpec",
    "ToolHandler",
    "ToolExecutionError",
    "MemoryStore",
    "InMemoryStore",
    "Turn",
    "Fact",
    "SideEffectBus",
    "NoopBus",
    "RecordingBus",
    "CompositeBus",
    "Effect",
    "EffectContext",
    "LogEffect",
    "EchoEffect",
    "SendReplyEffect",
    "SpeakEffect",
    "PlaySoundEffect",
    "SetLEDEffect",
    "MoveEffect",
    "TelegramBus",
    "TelegramChannel",
    "SnapshotStore",
    "SnapshotEntry",
    "InMemorySnapshotStore",
    "parse_telegram_update",
    # Built-in dummy providers
    "DummyLLMProvider",
    "HarnessFakeLLMProvider",
    # Provider health-check / fallback (issue #1082)
    "ProviderStatus",
    "HealthRecord",
    "HealthCache",
    "HealthAwareFallbackLLM",
    "check_deepseek_balance",
    "is_quota_exhausted",
    "is_auth_failure",
    "DEFAULT_HEALTH_TTL_S",
    "TRANSIENT_TTL_S",
    # Errors
    "HarnessError",
    "ConfigError",
    "HarnessNotFoundError",
    "HarnessStateError",
    "ProviderNotFoundError",
    "HookError",
]

__version__ = "0.1.0"
