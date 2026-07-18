"""LLM provider contract — `LLMProvider` ABC + value objects.

The contract is intentionally minimal:

    complete(messages, *, tools=(), settings=...) -> LLMResponse
    stream(messages,   *, tools=(), settings=...) -> AsyncIterator[LLMChunk]

Everything else (provider names, fallbacks, retries) lives in higher-level
classes that compose a `LLMProvider` (e.g. a fallback wrapper in P1, or
`ProviderManager` in `rob_box_voice.llm`).

Value objects are plain dataclasses — easy to serialise, mock and assert on.
"""

from __future__ import annotations

import abc
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, AsyncIterator, Iterable, Mapping


# ---------------------------------------------------------------------------
# Value objects
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ToolCall:
    """A tool invocation requested by the model."""

    id: str
    name: str
    arguments: Mapping[str, Any]

    def __post_init__(self) -> None:
        # Freeze dict arguments into a read-only MappingProxy so the dataclass
        # stays truly immutable. We deliberately hash on (id, name) only —
        # arguments may be large / nested dicts we don't want to hash every time.
        if isinstance(self.arguments, dict):
            object.__setattr__(self, "arguments", MappingProxyType(self.arguments))

    def __hash__(self) -> int:  # noqa: D401
        return hash((self.id, self.name))

    def __eq__(self, other: object) -> bool:
        # Equality uses the full payload so {tc1, tc2} still works in tests.
        if not isinstance(other, ToolCall):
            return NotImplemented
        return (
            self.id == other.id
            and self.name == other.name
            and dict(self.arguments) == dict(other.arguments)
        )


@dataclass(frozen=True)
class ToolResult:
    """Result of executing a tool; fed back to the model on the next turn."""

    tool_call_id: str
    content: str
    is_error: bool = False


@dataclass(frozen=True)
class LLMMessage:
    """A single message in the conversation.

    `role` ∈ {"system", "user", "assistant", "tool"}. For "tool" messages the
    caller SHOULD use the dedicated `tool_result` field rather than packing the
    payload into `content`.
    """

    role: str
    content: str
    name: str | None = None
    tool_call_id: str | None = None
    tool_calls: tuple[ToolCall, ...] = ()
    tool_result: ToolResult | None = None


@dataclass(frozen=True)
class LLMSettings:
    """Per-call knobs. All optional; the provider chooses sensible defaults."""

    model: str | None = None
    temperature: float | None = None
    max_tokens: int | None = None
    stop: tuple[str, ...] = ()
    tool_choice: str | None = None  # "auto" | "none" | "required" | provider-specific
    extra: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class LLMResponse:
    """Non-streaming response. `content` is fully assembled.

    `tool_calls` is empty when the model answered in plain text; `finish_reason`
    is whatever the underlying SDK returned ("stop", "tool_calls", "length", …).
    """

    content: str = ""
    tool_calls: tuple[ToolCall, ...] = ()
    finish_reason: str | None = None
    usage: Mapping[str, int] = field(default_factory=dict)
    raw: Any | None = None  # the original SDK response, kept for diagnostics


@dataclass(frozen=True)
class LLMChunk:
    """A single streaming chunk.

    Providers MUST emit at least one final chunk with `finish_reason` set so
    callers can detect end-of-stream deterministically (instead of guessing
    based on empty content).
    """

    content_delta: str = ""
    tool_call_delta: ToolCall | None = None
    finish_reason: str | None = None
    usage: Mapping[str, int] | None = None


# ---------------------------------------------------------------------------
# ABC
# ---------------------------------------------------------------------------


class LLMProvider(abc.ABC):
    """Async-only contract for LLM providers.

    Implementations:
        - DeepSeekProvider (P0.1)
        - MiMoProvider     (P0.1)
        - FakeLLMProvider  (P0.1, tests)

    P1 will add:
        - FallbackProvider (wraps two providers, retries on RateLimitError)
    """

    name: str = "abstract"

    @abc.abstractmethod
    async def complete(
        self,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[Mapping[str, Any]] = (),
        settings: LLMSettings | None = None,
    ) -> LLMResponse:
        """Run a non-streaming completion. Raises ProviderError on failure."""

    @abc.abstractmethod
    async def stream(
        self,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[Mapping[str, Any]] = (),
        settings: LLMSettings | None = None,
    ) -> AsyncIterator[LLMChunk]:
        """Run a streaming completion.

        Implementations MUST raise ProviderError BEFORE yielding anything if the
        initial request fails — once the first chunk is yielded, mid-stream
        failures become `LLMChunk(finish_reason="error")` instead, because the
        caller has already started emitting tokens.
        """
        # AsyncIterator declared via return type; concrete impl uses `yield`.
        raise NotImplementedError
        yield  # pragma: no cover — keeps type checkers happy on ABC stub

    async def aclose(self) -> None:  # noqa: D401 — async context-manager hook
        """Release resources. Default impl is no-op for stateless providers."""
        return None


__all__ = [
    "LLMProvider",
    "LLMMessage",
    "LLMResponse",
    "LLMChunk",
    "LLMSettings",
    "ToolCall",
    "ToolResult",
]
