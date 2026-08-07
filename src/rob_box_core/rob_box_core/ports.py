"""Executable contract for the P1.2 tool-provider port.

The task body is the API source of truth for this module:

* ``list_tools() -> list[ToolDescriptor]``
* ``invoke(name, args, ctx) -> ToolResult``
* ``validate_args(name, args) -> ValidationResult``

The value objects intentionally stay ROS- and vendor-free. Concrete MCP and
local implementations live in :mod:`rob_box_harness.executors`.
"""

from __future__ import annotations

import abc
from dataclasses import dataclass, field
from typing import Any, Mapping


@dataclass(frozen=True)
class ToolDescriptor:
    """One tool exposed to an LLM, including its JSON Schema arguments."""

    name: str
    description: str = ""
    parameters: Mapping[str, Any] = field(default_factory=dict)
    idempotent: bool = False
    read_only: bool = False


@dataclass(frozen=True)
class ToolResult:
    """Result returned by :meth:`ToolProvider.invoke`."""

    value: Any = None
    error: str | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)

    @property
    def is_error(self) -> bool:
        """Return whether this result represents a tool-level failure."""

        return self.error is not None


@dataclass(frozen=True)
class ValidationResult:
    """Pure argument-validation outcome."""

    valid: bool
    errors: tuple[str, ...] = ()

    @property
    def ok(self) -> bool:
        """Backward-compatible alias used by early P1 callers."""

        return self.valid


@dataclass(frozen=True)
class ToolContext:
    """Per-call execution context supplied by the caller."""

    caller_id: str | None = None
    timeout: float | None = None
    idempotency_key: str | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)


class ToolProviderError(Exception):
    """Base class for provider-level failures."""

    def __init__(
        self,
        message: str,
        *,
        provider: str | None = None,
        tool_name: str | None = None,
    ) -> None:
        super().__init__(message)
        self.provider = provider
        self.tool_name = tool_name


class ToolNotFound(ToolProviderError):
    """The requested tool is not present in the provider catalogue."""


class ToolTimeout(ToolProviderError):
    """Tool execution exceeded the configured timeout."""


class ToolValidationError(ToolProviderError):
    """Arguments did not satisfy the tool schema."""

    def __init__(
        self,
        message: str,
        *,
        errors: tuple[str, ...] = (),
        provider: str | None = None,
        tool_name: str | None = None,
    ) -> None:
        super().__init__(message, provider=provider, tool_name=tool_name)
        self.errors = errors


# Explicit aliases preserve imports written against the error names used by
# the architecture prose before the P1.2 task body shortened them.
ToolNotFoundError = ToolNotFound
ToolTimeoutError = ToolTimeout


class ToolProvider(abc.ABC):
    """Canonical execution abstraction shared by all harness adapters."""

    name: str = "abstract"

    @abc.abstractmethod
    def list_tools(self) -> list[ToolDescriptor]:
        """Return a fresh list containing this provider's stable catalogue."""

    @abc.abstractmethod
    async def invoke(
        self,
        name: str,
        args: Mapping[str, Any],
        ctx: ToolContext | None = None,
    ) -> ToolResult:
        """Validate and execute one named tool."""

    @abc.abstractmethod
    def validate_args(
        self,
        name: str,
        args: Mapping[str, Any],
    ) -> ValidationResult:
        """Validate arguments without executing the tool."""

    async def aclose(self) -> None:
        """Release resources; implementations must keep this idempotent."""

        return None


__all__ = [
    "ToolContext",
    "ToolDescriptor",
    "ToolNotFound",
    "ToolNotFoundError",
    "ToolProvider",
    "ToolProviderError",
    "ToolResult",
    "ToolTimeout",
    "ToolTimeoutError",
    "ToolValidationError",
    "ValidationResult",
]
