"""ToolProvider port — execution of tool calls issued by the LLM.

This module owns the abstract contract for tool execution; the
richer ``ToolCall`` / ``ToolResult`` value objects are re-exported
from ``rob_box_llm`` so the harness can pass them through to the
LLM without round-tripping through dicts.

ADR-0001 §2.4.2 lists three required methods:

* :meth:`ToolProvider.discover` — list tool specs the provider can
  execute (used by the LLM's tool-choice step).
* :meth:`ToolProvider.execute`  — run a single call and return a
  ``ToolResult`` (or raise on transport-level failure).
* :meth:`ToolProvider.aclose`   — release resources.

The :class:`FakeToolProvider` is the brain-dead in-memory
implementation: a registered dict of ``name -> callable`` plus
a discovery manifest. It is the workhorse of the smoke test.
"""

from __future__ import annotations

import abc
from dataclasses import dataclass, field
from typing import Any, Awaitable, Callable, Mapping

from rob_box_llm.provider import ToolCall, ToolResult


@dataclass(frozen=True)
class ToolSpec:
    """Description of a single tool exposed by a provider.

    ``parameters`` is the JSON Schema dict that the LLM uses to
    decide what to call. ``name`` is the canonical identifier; two
    providers MUST NOT register the same name under the same
    harness — the harness registry raises on a collision.
    """

    name: str
    description: str
    parameters: Mapping[str, Any] = field(default_factory=dict)


ToolHandler = Callable[[Mapping[str, Any]], Awaitable[Any] | Any]
"""A tool callable. Either sync or async; the framework awaits the
result if it's awaitable."""


class ToolProvider(abc.ABC):
    """Abstract tool executor."""

    name: str = "abstract"

    @abc.abstractmethod
    async def discover(self) -> tuple[ToolSpec, ...]:
        """Return the tool specs that this provider currently exposes."""

    @abc.abstractmethod
    async def execute(self, call: ToolCall) -> ToolResult:
        """Execute ``call`` and return a :class:`ToolResult`.

        Raises :class:`ToolExecutionError` for transport-level
        failures (timeout, connection, schema mismatch). Function-
        level errors should be returned as ``ToolResult`` with
        ``is_error=True`` so the LLM can adjust.
        """

    async def aclose(self) -> None:
        """Release resources. Default no-op."""
        return None


class ToolExecutionError(Exception):
    """Raised when a :class:`ToolProvider` cannot deliver a call.

    Distinct from a tool-level failure (which becomes a
    ``ToolResult(is_error=True)``). This is a transport / wiring
    problem that the harness should surface to ``on_error`` and
    consider retrying.
    """

    def __init__(
        self,
        message: str,
        *,
        provider: str | None = None,
        call: ToolCall | None = None,
    ) -> None:
        super().__init__(message)
        self.provider = provider
        self.call = call


class FakeToolProvider(ToolProvider):
    """In-memory tool provider driven by a name → (spec, handler) map.

    Two ways to use it:

    1. **Register tools up-front** — pass a dict to the constructor.
    2. **Register tools later** — call :meth:`register` from the
       harness's ``init`` lifecycle so the harness stays in control
       of the manifest.
    """

    name = "fake"

    def __init__(
        self,
        tools: Mapping[str, tuple[ToolSpec, ToolHandler]] | None = None,
    ) -> None:
        self._tools: dict[str, tuple[ToolSpec, ToolHandler]] = dict(tools or {})
        # Register the built-in "echo" tool so the smoke harness's
        # DummyLLMProvider tool-call path works out of the box.
        if "echo" not in self._tools:
            self._register_builtin_echo()

    def _register_builtin_echo(self) -> None:
        """Register a built-in ``echo`` tool unless one already exists.

        Tests may register their own ``echo`` tool with custom behaviour;
        we only install the default when no ``echo`` tool has been
        registered yet (either via constructor or ``register()``).
        """
        if "echo" in self._tools:
            return
        echo_spec = ToolSpec(
            name="echo",
            description="Echo back the provided arguments.",
            parameters={
                "type": "object",
                "properties": {
                    "text": {"type": "string", "description": "Text to echo back."},
                },
            },
        )

        async def _echo_handler(args: Mapping[str, Any]) -> str:
            return f"echo: {args.get('text', '')}"

        self._tools["echo"] = (echo_spec, _echo_handler)

    def register(self, spec: ToolSpec, handler: ToolHandler) -> None:
        """Register a tool. Silently overrides an existing registration
        with the same name — tests use this to replace the built-in
        ``echo`` tool with a custom capturing handler."""
        self._tools[spec.name] = (spec, handler)

    async def discover(self) -> tuple[ToolSpec, ...]:
        """Return every registered tool's spec."""
        return tuple(spec for spec, _ in self._tools.values())

    async def execute(self, call: ToolCall) -> ToolResult:
        """Run ``call`` against the matching handler and stringify the result."""
        entry = self._tools.get(call.name)
        if entry is None:
            return ToolResult(
                tool_call_id=call.id,
                content=f"unknown tool: {call.name}",
                is_error=True,
            )
        _spec, handler = entry
        try:
            result = handler(dict(call.arguments))
            if hasattr(result, "__await__"):
                result = await result
        except Exception as exc:  # noqa: BLE001 — wrap into a ToolResult
            return ToolResult(
                tool_call_id=call.id,
                content=f"{type(exc).__name__}: {exc}",
                is_error=True,
            )
        return ToolResult(
            tool_call_id=call.id,
            content=_stringify(result),
            is_error=False,
        )


def _stringify(value: Any) -> str:
    """Coerce a tool return value into a string for the LLM.

    Plain strings pass through; everything else is ``repr()``'d to
    keep the contract narrow. Concrete harnesses can override by
    registering a richer handler.
    """
    if isinstance(value, str):
        return value
    return repr(value)


__all__ = [
    "ToolSpec",
    "ToolHandler",
    "ToolProvider",
    "FakeToolProvider",
    "ToolExecutionError",
]
