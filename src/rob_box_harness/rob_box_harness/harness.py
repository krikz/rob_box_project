"""Harness — the abstract contract every dialog/persistent/telegram harness implements.

A :class:`Harness` is a thin orchestrator that:

  * holds five ports (``llm``, ``tools``, ``memory``, ``effects``,
    ``transport``) plus two cross-cutting helpers (``clock``,
    ``hooks``);
  * runs a deterministic init/run/teardown lifecycle (ADR-0001 §2.3);
  * doubles as an async context manager so ``async with harness:``
    guarantees ``teardown`` on exception;
  * exposes a ``snapshot`` / ``restore`` pair for tests and replay.

Design choices:

* The ABC is intentionally narrow. Concrete harnesses (Dialog etc.)
  inherit from :class:`BaseHarness` (below) and override only the
  methods they care about — most importantly :meth:`_run_once`.
* State is supplied by the subclass via ``state`` and indexed under
  ``self.state``. The framework never inspects it, only round-trips
  it through :class:`SessionSnapshot`.
* The lifecycle is idempotent by contract: :meth:`init` and
  :meth:`teardown` can be called multiple times safely. Calling
  :meth:`init` while ``_running`` is True raises :class:`HarnessStateError`.
* The framework wraps user-supplied hooks; a hook that raises goes
  through :class:`HookError` and the harness decides whether to
  swallow it.

The dataclass holding ``state`` is the type parameter ``StateT``;
it defaults to ``dict[str, Any]`` so the framework can be used
without a custom dataclass. Concrete harnesses pass a richer type
when they need it (e.g. ``DialogueState``).
"""

from __future__ import annotations

import abc
import asyncio
import logging
from dataclasses import dataclass, field
from datetime import datetime
from types import TracebackType
from typing import Any, Generic, Mapping, TypeVar

from rob_box_harness.clock import Clock, SystemClock
from rob_box_harness.config import HarnessConfig
from rob_box_harness.errors import HarnessError, HarnessStateError
from rob_box_harness.lifecycle import LifecycleHooks
from rob_box_harness.snapshot import SessionSnapshot
from rob_box_harness.transport import Transport, FakeTransport
from rob_box_harness.tools import ToolProvider, FakeToolProvider
from rob_box_harness.memory import MemoryStore, InMemoryStore
from rob_box_harness.effects import SideEffectBus, NoopBus
from rob_box_llm.provider import LLMProvider

# ``StateT`` is intentionally loose: ``Mapping[str, Any]`` works
# for the framework tests, and concrete harnesses can swap in a
# frozen dataclass. The framework itself never writes to ``state``.
StateT = TypeVar("StateT", bound=Mapping[str, Any])

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Run result
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class HarnessRunResult:
    """The result returned by :meth:`run_harness` and :meth:`Harness.run`.

    ``state`` is the final state mapping of the harness. ``output``
    is whatever the harness chose to communicate back to the caller
    (usually the last assistant turn's text, or a structured payload).
    """

    state: Mapping[str, Any]
    output: Any = None
    metadata: Mapping[str, Any] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# ABC
# ---------------------------------------------------------------------------


class Harness(abc.ABC, Generic[StateT]):
    """Base contract for every harness.

    Subclasses define:

    * the state's type (via the ``StateT`` parameter),
    * the response to a single ``run_once`` step (the only abstract
      method: :meth:`step`),
    * any custom wiring inside :meth:`init` (overriding and
      calling ``super().init()``).

    Subclasses DO NOT override :meth:`init` / :meth:`run` / :meth:`teardown`
    unless they have a strong reason; the framework implementations
    already cover the contract.
    """

    name: str = "abstract"

    def __init__(
        self,
        config: HarnessConfig,
        *,
        clock: Clock | None = None,
        hooks: LifecycleHooks | None = None,
        llm: LLMProvider | None = None,
        tools: ToolProvider | None = None,
        memory: MemoryStore | None = None,
        effects: SideEffectBus | None = None,
        transport: Transport | None = None,
    ) -> None:
        """Store config and default-construct any missing port.

        Honest to ADR-0001 §2.3: ``__init__`` does NOT do I/O — it
        merely stores config and knobs. Real ports are constructed
        in :meth:`init` or supplied by the caller.
        """
        if not isinstance(config, HarnessConfig):
            raise HarnessError(
                f"harness config must be a HarnessConfig, got {type(config).__name__}"
            )
        self.config = config
        self.clock: Clock = clock or SystemClock()
        self.hooks: LifecycleHooks = hooks or LifecycleHooks()
        self.llm: LLMProvider | None = llm
        self.tools: ToolProvider | None = tools
        self.memory: MemoryStore | None = memory
        self.effects: SideEffectBus = effects or NoopBus()
        self.transport: Transport | None = transport
        self.state: StateT = {}  # type: ignore[assignment]
        self._initialized: bool = False
        self._running: bool = False
        self._extensions: dict[str, Any] = {}

    # ----- lifecycle ------------------------------------------------------

    async def init(self) -> None:
        """One-shot resource allocation. Idempotent.

        Builds the missing ports from the config (default
        implementations pick in-memory / fake variants so the
        framework can run without ros2 / network). Subclasses
        that need custom wiring should override and call
        ``await super().init()`` first.
        """
        if self._running:
            raise HarnessStateError(
                "init() called while run() is in progress",
                harness=self.name,
            )
        if self._initialized:
            return

        # Default port construction. Concrete harnesses override
        # init() to inject real providers (DeepSeek, ROS2, ...).
        if self.llm is None:
            self.llm = self._default_llm()
        if self.tools is None:
            self.tools = self._default_tools()
        if self.memory is None:
            self.memory = self._default_memory()
        if self.transport is None:
            self.transport = self._default_transport()

        self.state = self._initial_state()

        # Hook fires *after* state is in place but *before* any
        # turn, so observers can warm caches. Mark the harness as
        # initialized only after the hook succeeds, preventing a
        # partially initialized harness from appearing usable.
        await self.hooks.invoke("on_start", self.name)
        self._initialized = True
        logger.debug("harness %s initialised", self.name)

    async def run(self, input_data: Any = None) -> HarnessRunResult:
        """Process ``input_data`` once and return the result.

        Splits cleanly into:

          1. ``run_one(input_data)`` — the subclass-defined step.
          2. ``_record_step`` — bookkeeping for the snapshot.

        The name is ``run`` (not ``run_loop``) because the framework
        treats each invocation as a single turn. Concrete harnesses
        iterate over a queue by calling ``run`` in their own loop.
        """
        if not self._initialized:
            raise HarnessStateError(
                "run() called before init()",
                harness=self.name,
            )
        if self._running:
            raise HarnessStateError(
                "run() re-entered while a previous run() is still active",
                harness=self.name,
            )
        self._running = True
        try:
            await self.hooks.invoke("on_turn_begin", self.name, input_data)
            output = await self.step(input_data)
            await self._record_step(input_data, output)
            return HarnessRunResult(
                state=dict(self.state),
                output=output,
                metadata={"harness": self.name},
            )
        finally:
            self._running = False

    async def teardown(self) -> None:
        """Release resources. Idempotent."""
        if not self._initialized:
            return
        if self._running:
            raise HarnessStateError(
                "teardown() called while run() is in progress",
                harness=self.name,
            )
        try:
            await self.hooks.invoke("on_stop", self.name)
        finally:
            # Always close ports even if the hook raises, so the
            # shutdown guarantees bit stays intact.
            for port in (self.llm, self.tools, self.memory, self.transport, self.effects):
                if port is not None and hasattr(port, "aclose"):
                    try:
                        await port.aclose()
                    except Exception:  # noqa: BLE001 — close is best-effort
                        logger.exception(
                            "harness %s: aclose on %s failed",
                            self.name,
                            type(port).__name__,
                        )
            self._initialized = False
            logger.debug("harness %s torn down", self.name)

    # ----- async context manager -----------------------------------------

    async def __aenter__(self) -> "Harness[StateT]":
        await self.init()
        return self

    async def __aexit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        if exc_type is not None:
            try:
                await self.hooks.invoke("on_error", self.name, exc)
            except Exception:  # noqa: BLE001 — log, do not re-raise
                logger.exception("harness %s: on_error hook raised", self.name)
        await self.teardown()

    # ----- abstract step --------------------------------------------------

    @abc.abstractmethod
    async def step(self, input_data: Any) -> Any:
        """Process a single input. Subclasses MUST implement.

        The harness must remain a thin orchestrator: it should
        call ``self.llm.complete(...)``, ``self.tools.execute(...)``,
        ``self.hooks.invoke('on_tool_call', self.name, call)``, then
        ``await self.effects.dispatch(...)`` and finally return
        whatever the caller should see. The framework records state
        in :meth:`run`, so :meth:`step` does NOT need to do
        bookkeeping.
        """

    # ----- snapshot / restore --------------------------------------------

    def snapshot(self) -> SessionSnapshot:
        """Return a :class:`SessionSnapshot` of the current state."""
        captured_at: datetime = self.clock.now()
        return SessionSnapshot(
            harness_name=self.name,
            harness_kind=self.config.harness,
            captured_at=captured_at,
            state=dict(self.state),
            extensions=_safe_extensions(self._extensions),
        )

    def restore(self, snapshot: SessionSnapshot) -> None:
        """Restore the harness's state from ``snapshot``.

        Only the ``state`` mapping is restored; the ``extensions``
        dict is left alone. This keeps the contract narrow: tests
        that want to assert "if harness started in state X, it ends
        in state Y" do so through snapshots, not by re-running
        every step.
        """
        if snapshot.harness_kind != self.config.harness:
            raise HarnessError(
                f"snapshot is for harness_kind={snapshot.harness_kind!r}, "
                f"this harness is {self.config.harness!r}"
            )
        self.state = dict(snapshot.state)  # type: ignore[assignment]

    # ----- internal helpers ----------------------------------------------

    @property
    def is_initialized(self) -> bool:
        """Has :meth:`init` successfully completed?"""
        return self._initialized

    @property
    def is_running(self) -> bool:
        """Is :meth:`run` currently in progress?"""
        return self._running

    def _record(self, key: str, value: Any) -> None:
        """Add an entry to the ``extensions`` dict on the snapshot.

        Useful for harnesses that want to track per-step metrics
        (LLM call counts, tool-call counts, …) without polluting
        the ``state`` mapping.
        """
        self._extensions[key] = value

    async def _record_step(self, input_data: Any, output: Any) -> None:
        """Default no-op; concrete harnesses can override.

        The default still touches ``self._extensions`` so a snapshot
        taken after :meth:`run` shows the latest input/output pair.
        """
        self._record("last_input", input_data)
        self._record("last_output", output)
        self._record("last_step_at", self.clock.now())

    # ----- default port factories -----------------------------------------

    def _default_llm(self) -> LLMProvider:
        """Default LLM port. Subclasses override for real providers."""
        from rob_box_harness.providers.dummy import DummyLLMProvider

        return DummyLLMProvider()

    def _default_tools(self) -> ToolProvider:
        """Default tool port — an empty fake."""
        return FakeToolProvider()

    def _default_memory(self) -> MemoryStore:
        """Default memory port — in-memory."""
        return InMemoryStore()

    def _default_transport(self) -> Transport:
        """Default transport — in-memory."""
        return FakeTransport()

    def _initial_state(self) -> StateT:
        """Build the harness's starting state from the config.

        Default: shallow copy of ``config.state``. Subclasses can
        override to project onto a richer dataclass.
        """
        return dict(self.config.state)  # type: ignore[return-value]


def _safe_extensions(extensions: Mapping[str, Any]) -> Mapping[str, Any]:
    """Return a JSON-serialisable copy of ``extensions``.

    Dataclasses / datetimes / objects that aren't JSON-serialisable
    by default are coerced via ``repr`` so a snapshot never crashes
    due to log noise.
    """
    result: dict[str, Any] = {}
    for key, value in extensions.items():
        try:
            import json

            json.dumps(value)
            result[key] = value
        except (TypeError, ValueError):
            result[key] = repr(value)
    return result


__all__ = ["Harness", "HarnessRunResult", "StateT"]
