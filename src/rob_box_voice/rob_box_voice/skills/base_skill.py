"""
base_skill.py — Abstract base class for all РОББОКС agent skills.

Each concrete skill:
  1. Implements _make_tools() → list of @function_tool callbacks
  2. Calls self.as_tool() which wraps an Agent in a FunctionTool

The parent Compositor agent calls the skill tool with a natural-language task
string; the skill runs its own LLM loop with a focused prompt+tool set and
returns a plain string result back to the Compositor.
"""

import asyncio
import json
from abc import ABC, abstractmethod

from agents import Agent
from agents.model_settings import ModelSettings
from agents.models.openai_chatcompletions import OpenAIChatCompletionsModel


class BaseSkill(ABC):
    """Abstract base for all РОББОКС agent skills.

    Args:
        adapter:        LLMToolCallAdapter that dispatches tool calls via ROS /mcp/execute.
        model:          Shared OpenAIChatCompletionsModel instance (DeepSeek/Qwen).
        prompt:         System prompt for this skill's sub-agent (full text, ready to use).
        name:           Display name for the Agent (used in SDK tracing).
        temperature:    LLM temperature (default 0.7).
        max_tokens:     Max tokens for each LLM response (default 500).
        agent_max_turns: Max tool-call iterations per sub-agent run (default 12).
    """

    def __init__(
        self,
        adapter,
        model: OpenAIChatCompletionsModel,
        prompt: str,
        name: str,
        temperature: float = 0.7,
        max_tokens: int = 500,
        agent_max_turns: int = 12,
    ) -> None:
        self._adapter = adapter
        self._model = model
        self._prompt = prompt
        self._name = name
        self._temperature = temperature
        self._max_tokens = max_tokens
        self._agent_max_turns = agent_max_turns

    # ── Internal helper ──────────────────────────────────────────────────────

    async def _call(self, tool_name: str, params: dict, timeout: float = 10.0) -> str:
        """Dispatch a single MCP tool call via the shared LLMToolCallAdapter."""
        result = await asyncio.get_running_loop().run_in_executor(
            None,
            lambda: self._adapter.execute_tool_call_sync(tool_name, params, timeout=timeout),
        )
        if isinstance(result, dict):
            return result.get("result", json.dumps(result, ensure_ascii=False))
        return str(result)

    # ── Subclass contract ────────────────────────────────────────────────────

    @abstractmethod
    def _make_tools(self) -> list:
        """Return a list of @function_tool callables for this skill's sub-agent."""

    # ── Public API ───────────────────────────────────────────────────────────

    def build_agent(self) -> Agent:
        """Build (or rebuild) the sub-Agent for this skill.  Call once at init."""
        return Agent(
            name=self._name,
            instructions=self._prompt,
            tools=self._make_tools(),
            model=self._model,
            model_settings=ModelSettings(
                temperature=self._temperature,
                max_tokens=self._max_tokens,
                parallel_tool_calls=False,
            ),
        )

    def as_tool(self, tool_name: str, tool_description: str):
        """Return a FunctionTool wrapping this skill's Agent.

        The returned tool can be passed directly to the Compositor Agent's
        ``tools`` list.  When called, it runs the skill's full LLM loop with a
        *fresh* text input (not the Compositor's conversation history).

        Args:
            tool_name:        Identifier used by the Compositor to call this skill.
            tool_description: One-line description shown to the Compositor LLM.
        """
        agent = self.build_agent()
        return agent.as_tool(
            tool_name=tool_name,
            tool_description=tool_description,
        )
