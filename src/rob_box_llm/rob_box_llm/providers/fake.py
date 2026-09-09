"""`FakeLLMProvider` — deterministic, in-memory LLM stand-in for tests.

Two ways to drive it:

1. **Scripted responses** — pre-load a list of `LLMResponse` objects; each
   call pops the next one. Once exhausted, raises ``IndexError`` (the caller
   should assert this in tests so an unexpected extra call is loud).
2. **Echo / rule callback** — pass ``on_complete`` / ``on_stream`` callables
   that produce a response based on the input messages. Useful when a test
   wants the provider to react to what was asked.

It also records every call in ``self.calls`` so assertions like
"did the harness send the system prompt + user msg + tool result?" are trivial.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, AsyncIterator, Awaitable, Callable, Iterable, Mapping, Optional

from rob_box_llm.provider import (
    LLMChunk,
    LLMMessage,
    LLMProvider,
    LLMResponse,
    LLMSettings,
    ToolCall,
)


@dataclass
class FakeCall:
    """Snapshot of one call to the provider — kept on `FakeLLMProvider.calls`."""

    messages: tuple[LLMMessage, ...]
    tools: tuple[Mapping[str, Any], ...]
    settings: LLMSettings | None
    kind: str  # "complete" or "stream"


@dataclass
class _ScriptedStream:
    """Stream source backed by a list of pre-canned content deltas."""

    deltas: list[str]
    tool_calls: list[ToolCall] = field(default_factory=list)
    finish_reason: str = "stop"

    async def __call__(self) -> AsyncIterator[LLMChunk]:
        for d in self.deltas:
            yield LLMChunk(content_delta=d)
        # Emit any tool calls as a final chunk so the harness can detect them.
        for tc in self.tool_calls:
            yield LLMChunk(tool_call_delta=tc)
        yield LLMChunk(finish_reason=self.finish_reason)


class FakeLLMProvider(LLMProvider):
    """Deterministic in-memory LLM for tests.

    Parameters
    ----------
    responses:
        Pre-canned `LLMResponse` objects popped one per `complete()` call.
    stream_scripts:
        Pre-canned async iterables (lists of `LLMChunk`) popped one per
        `stream()` call. Convenience: pass a list of strings and we'll wrap
        each into a chunk sequence.
    on_complete, on_stream:
        Async callables that take `(messages, tools, settings)` and return a
        `LLMResponse` (or async iterable of `LLMChunk` for `on_stream`).
        Override / augment scripted responses.
    name:
        Provider name reported in errors / logs. Defaults to ``"fake"``.
    """

    def __init__(
        self,
        *,
        responses: Optional[list[LLMResponse]] = None,
        stream_scripts: Optional[list[Any]] = None,
        on_complete: Optional[Callable[[tuple[LLMMessage, ...], tuple[Mapping[str, Any], ...], Optional[LLMSettings]], Awaitable[LLMResponse]]] = None,
        on_stream: Optional[Callable[[tuple[LLMMessage, ...], tuple[Mapping[str, Any], ...], Optional[LLMSettings]], AsyncIterator[LLMChunk]]] = None,
        name: str = "fake",
    ) -> None:
        self.name = name
        self._responses: list[LLMResponse] = list(responses or [])
        self._stream_scripts: list[Any] = list(stream_scripts or [])
        self._on_complete = on_complete
        self._on_stream = on_stream
        self.calls: list[FakeCall] = []

    # -- helpers -----------------------------------------------------------

    def queue_response(self, resp: LLMResponse) -> None:
        self._responses.append(resp)

    def queue_stream(self, chunks: Iterable[LLMChunk]) -> None:
        """Queue a stream script — pass any iterable of `LLMChunk`."""
        self._stream_scripts.append(list(chunks))

    def queue_stream_from_text(self, *deltas: str, finish_reason: str = "stop") -> None:
        """Convenience: queue a stream that emits the given deltas in order."""
        script = [LLMChunk(content_delta=d) for d in deltas]
        script.append(LLMChunk(finish_reason=finish_reason))
        self._stream_scripts.append(script)

    def queue_tool_call_stream(self, *tool_calls: ToolCall, finish_reason: str = "tool_calls") -> None:
        script: list[LLMChunk] = []
        for tc in tool_calls:
            script.append(LLMChunk(tool_call_delta=tc))
        script.append(LLMChunk(finish_reason=finish_reason))
        self._stream_scripts.append(script)

    @staticmethod
    def _normalise(
        messages: Iterable[LLMMessage],
        tools: Iterable[Mapping[str, Any]],
        settings: LLMSettings | None,
    ) -> tuple[tuple[LLMMessage, ...], tuple[Mapping[str, Any], ...], LLMSettings | None]:
        return tuple(messages), tuple(tools), settings

    # -- complete ----------------------------------------------------------

    async def complete(
        self,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[Mapping[str, Any]] = (),
        settings: LLMSettings | None = None,
    ) -> LLMResponse:
        msgs, tools_t, settings = self._normalise(messages, tools, settings)
        self.calls.append(FakeCall(msgs, tools_t, settings, kind="complete"))

        if self._on_complete is not None:
            return await self._on_complete(msgs, tools_t, settings)
        if not self._responses:
            raise IndexError(
                "FakeLLMProvider.complete called but no scripted responses left"
            )
        return self._responses.pop(0)

    # -- stream ------------------------------------------------------------

    async def stream(
        self,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[Mapping[str, Any]] = (),
        settings: LLMSettings | None = None,
    ) -> AsyncIterator[LLMChunk]:
        msgs, tools_t, settings = self._normalise(messages, tools, settings)
        self.calls.append(FakeCall(msgs, tools_t, settings, kind="stream"))

        if self._on_stream is not None:
            async for chunk in self._on_stream(msgs, tools_t, settings):
                yield chunk
            return
        if not self._stream_scripts:
            raise IndexError(
                "FakeLLMProvider.stream called but no stream scripts left"
            )
        script = self._stream_scripts.pop(0)
        for chunk in script:
            yield chunk


__all__ = ["FakeLLMProvider", "FakeCall"]
