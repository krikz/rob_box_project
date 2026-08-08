"""Offline integration tests across harness orchestration and provider wiring.

These tests intentionally use real framework objects and only replace the
external MiniMax HTTP boundary.  They are marked ``integration`` so CI can
select them independently; no test in this module opens a socket.
"""

from __future__ import annotations

from typing import Any
from unittest.mock import MagicMock

import pytest

from rob_box_harness.config import HarnessConfig, LLMConfig
from rob_box_harness.effects import RecordingBus
from rob_box_harness.harnesses.echo import EchoHarness
from rob_box_harness.providers.minimax import RetryPolicy, build_minimax_provider
from rob_box_harness.runner import run_harness

pytestmark = pytest.mark.integration


class _RecordedCompletions:
    """Replay one sanitized MiniMax Chat Completions fixture."""

    def __init__(self) -> None:
        self.calls: list[dict[str, Any]] = []

    async def create(self, **kwargs: Any) -> Any:
        self.calls.append(kwargs)
        assert kwargs["model"] == "MiniMax-M3"
        assert kwargs["messages"] == [{"role": "user", "content": "hello MiniMax"}]
        # 🔴 FIX (live 06.08): кастомные поля провайдера (thinking policy) идут
        # через extra_body, НЕ в kwargs — OpenAI SDK строго типизирован
        # (create(thinking=...) → TypeError, см. b5879b79 / 6901a14e).
        assert kwargs.get("extra_body", {}).get("thinking") == {"type": "disabled"}
        return MagicMock(
            choices=[
                MagicMock(
                    message=MagicMock(
                        content="fixture says hello",
                        tool_calls=None,
                    ),
                    finish_reason="stop",
                )
            ],
            usage=MagicMock(
                prompt_tokens=2,
                completion_tokens=3,
                total_tokens=5,
            ),
            base_resp={"status_code": 0, "status_msg": "success"},
        )


class _RecordedMiniMaxClient:
    """Minimal AsyncOpenAI-compatible client backed by a recorded fixture."""

    def __init__(self) -> None:
        self.chat = MagicMock()
        self.chat.completions = _RecordedCompletions()
        self.close_count = 0

    async def close(self) -> None:
        self.close_count += 1


@pytest.mark.asyncio
async def test_dummy_harness_runs_end_to_end_through_public_entry_point() -> None:
    """The public runner drives init, one turn, side effects, and teardown."""
    config = HarnessConfig.from_dict(
        {"harness": {"kind": "echo", "name": "offline-integration"}}
    )

    result = await run_harness("echo", "hello", config)

    assert result.output == "echo: hello"
    assert result.metadata == {"harness": "echo"}
    assert result.state == {}


@pytest.mark.asyncio
async def test_minimax_fixture_drives_real_harness_without_network() -> None:
    """A recorded MiniMax-shaped response traverses provider and harness layers."""
    client = _RecordedMiniMaxClient()
    provider = build_minimax_provider(
        LLMConfig(provider="minimax", model="MiniMax-M3"),
        env={"MINIMAX_API_KEY": "fixture-key-not-a-secret"},
        retry=RetryPolicy(max_attempts=1),
        client=client,  # type: ignore[arg-type]
    )
    config = HarnessConfig.from_dict(
        {"harness": {"kind": "minimax-fixture", "name": "recorded-session"}}
    )
    effects = RecordingBus()
    harness = EchoHarness(config, llm=provider, effects=effects)

    async with harness:
        result = await harness.run("hello MiniMax")
        turns = await harness.memory.load_recent("recorded-session", limit=10)

    assert result.output == "fixture says hello"
    assert [turn.content for turn in turns] == [
        "fixture says hello",
        "hello MiniMax",
    ]
    assert [effect.text for effect in effects.effects] == ["fixture says hello"]
    assert len(client.chat.completions.calls) == 1
    assert client.close_count == 1
