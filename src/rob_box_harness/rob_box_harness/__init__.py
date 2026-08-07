"""rob_box_harness — the Harness Framework (ADR-0001 P0).

The framework is the thin layer that sits on top of ``rob_box_llm``
and gives every concrete ROS2-node-adjacent workflow (Dialog,
Persistent, Telegram) a uniform set of contracts:

  * :class:`Harness` — the lifecycle contract.
  * :class:`HarnessConfig` / :func:`load_config` — YAML + env config.
  * :class:`HarnessRegistry` / :class:`HarnessFactory` — name →
    builder registry and cached instantiation.
  * :func:`run_harness` / :func:`run_harness_sync` — the single
    entry point everyone calls.

Ports (provider contracts):

  * :class:`LLMProvider` — re-exported from ``rob_box_llm``.
  * :class:`ToolProvider` / :class:`FakeToolProvider` — tool execution.
  * :class:`MemoryStore` / :class:`InMemoryStore` — per-scope history.
  * :class:`SideEffectBus` / :class:`NoopBus` / :class:`RecordingBus` /
    :class:`CompositeBus` — fan-out of side-effects.
  * :class:`Transport` / :class:`FakeTransport` — input source.
  * :class:`Clock` / :class:`SystemClock` / :class:`MockClock` — DI
    for time.

Errors live in :mod:`rob_box_harness.errors`. The exit-level
errors are :class:`HarnessError`, :class:`ConfigError`,
:class:`HarnessNotFoundError`, :class:`HarnessStateError`,
:class:`ProviderNotFoundError`, :class:`HookError`.

Public surface organised by concern:

Harness & lifecycle:
    Harness, HarnessRunResult, LifecycleHooks, SessionSnapshot

Config:
    HarnessConfig, LLMConfig, ToolsConfig, MemoryConfig,
    EffectsConfig, TransportConfig, LoggingConfig, load_config

Registry & runner:
    HarnessRegistry, HarnessFactory, register_builtin_harnesses,
    run_harness, run_harness_sync, get_default_registry,
    reset_default_registry

Ports:
    Clock, SystemClock, MockClock,
    Transport, FakeTransport, BaseTransport, EventHandler,
    VadEvent, KeyEvent, TelegramUpdate,
    ToolProvider, FakeToolProvider, ToolSpec, ToolExecutionError,
    MemoryStore, InMemoryStore, Turn, Fact,
    SideEffectBus, NoopBus, RecordingBus, CompositeBus, Effect,
    EffectContext, LogEffect, EchoEffect
  * :class:`SendReplyEffect` / :class:`SpeakEffect` /
    :class:`PlaySoundEffect` / :class:`SetLEDEffect` /
    :class:`MoveEffect` / :class:`TelegramBus` /
    :class:`TelegramFilteredBus` — telegram-side extensions (P1.4).
  * :class:`SnapshotStore` / :class:`InMemorySnapshotStore` /
    :func:`parse_telegram_update` — port for camera/frame caches
    plus update-parsing helper (P1.4).

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
from rob_box_harness.harness import Harness, HarnessRunResult
from rob_box_harness.lifecycle import Hook, LifecycleHooks
from rob_box_harness.memory import Fact, InMemoryStore, MemoryStore, Turn
from rob_box_harness.providers import DummyLLMProvider, HarnessFakeLLMProvider
from rob_box_harness.registry import (
    HarnessBuilder,
    HarnessFactory,
    HarnessRegistry,
    register_builtin_harnesses,
)
from rob_box_harness.runner import (
    get_default_registry,
    reset_default_registry,
    run_harness,
    run_harness_sync,
)
from rob_box_harness.snapshot import SessionSnapshot
from rob_box_harness.snapshot_store import (
    InMemorySnapshotStore,
    SnapshotEntry,
    SnapshotStore,
    parse_telegram_update,
)
from rob_box_harness.tools import (
    FakeToolProvider,
    ToolExecutionError,
    ToolHandler,
    ToolProvider,
    ToolSpec,
)
from rob_box_harness.transport import (
    BaseTransport,
    EventHandler,
    FakeTransport,
    KeyEvent,
    TelegramUpdate,
    Transport,
    VadEvent,
)

__all__ = [
    # Harness & lifecycle
    "Harness",
    "HarnessRunResult",
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
    # Registry & runner
    "HarnessBuilder",
    "HarnessRegistry",
    "HarnessFactory",
    "register_builtin_harnesses",
    "run_harness",
    "run_harness_sync",
    "get_default_registry",
    "reset_default_registry",
    # Ports
    "Clock",
    "SystemClock",
    "MockClock",
    "Transport",
    "BaseTransport",
    "FakeTransport",
    "EventHandler",
    "VadEvent",
    "KeyEvent",
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
    # Errors
    "HarnessError",
    "ConfigError",
    "HarnessNotFoundError",
    "HarnessStateError",
    "ProviderNotFoundError",
    "HookError",
]

__version__ = "0.1.0"
