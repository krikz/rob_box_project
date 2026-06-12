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
import logging
from abc import ABC, abstractmethod
from typing import Optional

from agents import Agent, Runner, function_tool
from agents.model_settings import ModelSettings
from agents.models.openai_chatcompletions import OpenAIChatCompletionsModel
from openai import APIConnectionError

logger = logging.getLogger(__name__)


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
        tool_choice:    If set, forces the sub-agent to use a tool on every turn
                        (e.g. ``"required"`` prevents the LLM from hallucinating
                        tool calls as plain text). Passed directly to ModelSettings.
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
        tool_choice: Optional[str] = None,
    ) -> None:
        self._adapter = adapter
        self._model = model
        self._prompt = prompt
        self._name = name
        self._temperature = temperature
        self._max_tokens = max_tokens
        self._agent_max_turns = agent_max_turns
        self._tool_choice = tool_choice

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
        model_settings_kwargs = dict(
            temperature=self._temperature,
            max_tokens=self._max_tokens,
            parallel_tool_calls=False,
        )
        if self._tool_choice is not None:
            model_settings_kwargs["tool_choice"] = self._tool_choice
        return Agent(
            name=self._name,
            instructions=self._prompt,
            tools=self._make_tools(),
            model=self._model,
            model_settings=ModelSettings(**model_settings_kwargs),
        )

    def as_tool(self, tool_name: str, tool_description: str):
        """Return a FunctionTool wrapping this skill's Agent.

        Unlike Agent.as_tool(), this version enforces max_turns so the sub-agent
        cannot loop indefinitely if the LLM over-calls tools.

        Args:
            tool_name:        Identifier used by the Compositor to call this skill.
            tool_description: One-line description shown to the Compositor LLM.
        """
        agent = self.build_agent()
        max_turns = self._agent_max_turns
        skill_name = self._name

        async def _run_skill(task: str) -> str:
            last_exc = None
            for attempt in range(3):  # 1 + 2 retries
                try:
                    logger.info(f"🎵 {skill_name} run started (attempt {attempt + 1}/3, max_turns={max_turns})")
                    result = await Runner.run(agent, input=task, max_turns=max_turns)
                    out = result.final_output or ""
                    if not out:
                        # Log what the sub-agent actually did so we can debug empty results
                        raw = []
                        for item in getattr(result, "new_items", []):
                            raw.append(f"  {type(item).__name__}: {getattr(item, 'raw_item', item)}")
                        logger.warning(
                            f"⚠️ {skill_name} returned EMPTY output! "
                            f"new_items={len(getattr(result, 'new_items', []))}:\n" + "\n".join(raw[:10])
                        )
                    else:
                        logger.info(f"🎵 {skill_name} returned {len(out)} chars")
                    return out
                except APIConnectionError as exc:
                    last_exc = exc
                    if attempt < 2:
                        delay = 2.0 * (2 ** attempt)
                        logger.warning(
                            f"⚠️ {skill_name} APIConnectionError (attempt {attempt + 1}/3), "
                            f"retrying in {delay:.0f}s: {exc}"
                        )
                        await asyncio.sleep(delay)
                    else:
                        logger.error(f"❌ {skill_name} APIConnectionError after 3 attempts: {exc}")
                        raise
                except Exception as exc:
                    logger.error(f"❌ {skill_name} unexpected error: {type(exc).__name__}: {exc}", exc_info=True)
                    raise
            raise last_exc  # unreachable

        _run_skill.__name__ = tool_name
        _run_skill.__doc__ = tool_description

        return function_tool(_run_skill, name_override=tool_name, description_override=tool_description)
