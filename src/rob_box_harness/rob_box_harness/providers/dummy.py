"""``DummyLLMProvider`` — the simplest possible LLMProvider.

Returns deterministic responses based on the user message:

* The text ``"ping"`` → ``LLMResponse(content="pong")`` plus a
  tool call to ``"echo"`` echoing back the arguments.
* Anything else → ``LLMResponse(content="echo: <text>")``.

This is the workhorse for the smoke test:

* It IS-A :class:`rob_box_llm.provider.LLMProvider`, so the harness
  can pass it to ``self.llm`` and the rest of the code path is
  identical to a real provider.
* It is fully deterministic — no network, no clock, no randomness.
* It exposes a small ``call_count`` so tests can assert "did the
  harness call the LLM exactly N times?".

It deliberately does NOT depend on :class:`FakeLLMProvider` from
``rob_box_llm.providers.fake``: keeping the dependency surface
narrow makes it easier to drop into a project that hasn't installed
``rob_box_llm`` yet (just the shape contract).
"""

from __future__ import annotations

from typing import Any, AsyncIterator, Iterable, Mapping

# rob_box_llm ships without a ``py.typed`` marker; the harness-side
# strict check ignores that package. Local imports below are typed
# ``Any`` at runtime, but the LLMProvider / ProviderCapabilities
# classes have full annotations inside the upstream module, so
# subclassing and the ``capabilities`` property still type-check
# correctly thanks to TYPE_CHECKING re-imports.
from rob_box_llm.provider import (
    LLMChunk,
    LLMMessage,
    LLMProvider,
    LLMResponse,
    LLMSettings,
    ProviderCapabilities,
    ToolCall,
)


class DummyLLMProvider(LLMProvider):  # type: ignore[misc]
    """A deterministic LLM provider for smoke tests."""

    name = "dummy"

    def __init__(self) -> None:
        self.call_count: int = 0
        self.last_messages: tuple[LLMMessage, ...] = ()

    @property
    def capabilities(self) -> ProviderCapabilities:
        """Plain text only — no tools, no vision, no streaming."""
        # Plain text only — no tools, no vision. This matches the
        # smoke harness's expectable contract; if a test needs more,
        # it should use a richer fake.
        return ProviderCapabilities(text=True, streaming_text=False)

    async def complete(
        self,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[Mapping[str, Any]] = (),
        settings: LLMSettings | None = None,
    ) -> LLMResponse:
        """Return a deterministic :class:`LLMResponse`.

        The last user message is the pivot. ``"ping"`` → ``"pong"``;
        anything else → ``"echo: <text>"``. The response carries a
        synthetic tool call when ``"ping"`` is observed, so the
        smoke harness can verify the tool-execution path.
        """
        self.call_count += 1
        captured = list(messages)
        self.last_messages = tuple(captured)
        last = _last_user_text(captured)
        if last is None:
            return LLMResponse(content="", finish_reason="stop")
        if last.strip().lower() == "ping":
            return LLMResponse(
                content="pong",
                tool_calls=(
                    ToolCall(
                        id="call_dummy_1",
                        name="echo",
                        arguments={"text": last},
                    ),
                ),
                finish_reason="tool_calls",
            )
        return LLMResponse(content=f"echo: {last}", finish_reason="stop")

    async def stream(
        self,
        messages: Iterable[LLMMessage],
        *,
        tools: Iterable[Mapping[str, Any]] = (),
        settings: LLMSettings | None = None,
    ) -> AsyncIterator[LLMChunk]:
        """Streaming variant — yields one chunk per character.

        Uses :meth:`complete` so the behaviour is identical to the
        non-streaming path; only the chunk shape differs.
        """
        response = await self.complete(
            messages, tools=tools, settings=settings
        )
        if response.content:
            for index, character in enumerate(response.content):
                if index == 0:
                    yield LLMChunk(content_delta=character)
                else:
                    yield LLMChunk(content_delta=character)
        yield LLMChunk(finish_reason=response.finish_reason or "stop")

    async def aclose(self) -> None:
        """No resources to release."""
        return None


def _last_user_text(messages: list[LLMMessage]) -> str | None:
    """Return the most recent user message's text content, or ``None``.

    Multipart content (text + image) is reduced to the concatenated
    text parts — images are silently ignored for the dummy provider.
    """
    for message in reversed(messages):
        if message.role != "user":
            continue
        content = message.content
        if isinstance(content, str):
            return content
        if isinstance(content, tuple):
            parts = [
                part.text
                for part in content
                if hasattr(part, "text")
            ]
            return "".join(parts) if parts else None
    return None


__all__ = ["DummyLLMProvider"]
