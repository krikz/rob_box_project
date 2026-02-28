#!/usr/bin/env python3
"""
llm_chat.py — Independent LLM chat sessions for Telegram operators.

Each user gets a separate conversation history. Uses the same DeepSeek/Qwen
API as the voice dialogue system but with an "operator" role context.
Supports MCP tool calling through the same tool definitions.
"""

import json
import logging
import os
from typing import Any, Dict, List, Optional

import aiohttp

logger = logging.getLogger(__name__)

# System prompt for operator chat
OPERATOR_SYSTEM_PROMPT = """Ты — Rob Box, автономный ровер. Оператор общается с тобой через Telegram.
Отвечай кратко и по делу. Ты можешь выполнять команды через инструменты (tools).
Если оператор просит что-то сделать (поехать, сказать, анимация и т.д.) — используй инструменты.
Язык общения: русский, если оператор не пишет на другом языке."""


class LLMChat:
    """Per-user LLM chat session manager.

    Attributes:
        sessions: Dict mapping chat_id to conversation history.
        max_history: Maximum messages per session before truncation.
        tools: MCP tool definitions in OpenAI format (for function calling).
    """

    # Provider configs (same as dialogue_node)
    PROVIDERS = {
        "deepseek": {
            "base_url": "https://api.deepseek.com/v1",
            "model": "deepseek-chat",
            "env_vars": ["DEEPSEEK_API_KEY", "LLM_API_KEY"],
        },
        "qwen": {
            "base_url": "https://dashscope-intl.aliyuncs.com/compatible-mode/v1",
            "model": "qwen-max",
            "env_vars": ["QWEN_API_KEY", "LLM_API_KEY"],
        },
    }

    def __init__(
        self,
        provider: str = "deepseek",
        max_history: int = 20,
        temperature: float = 0.7,
        tools: Optional[List[Dict]] = None,
    ):
        self.provider = provider
        self.max_history = max_history
        self.temperature = temperature
        self.tools = tools or []
        self.sessions: Dict[int, List[Dict[str, str]]] = {}

        # Resolve API key
        config = self.PROVIDERS.get(provider, self.PROVIDERS["deepseek"])
        self.base_url = config["base_url"]
        self.model = config["model"]
        self.api_key = ""
        for env_var in config["env_vars"]:
            self.api_key = os.getenv(env_var, "")
            if self.api_key:
                break

        if not self.api_key:
            logger.error("No API key found for provider '%s'", provider)

    def _get_history(self, chat_id: int) -> List[Dict[str, str]]:
        """Get or create conversation history for a user."""
        if chat_id not in self.sessions:
            self.sessions[chat_id] = []
        return self.sessions[chat_id]

    def _truncate_history(self, chat_id: int) -> None:
        """Keep only the last max_history messages."""
        history = self.sessions.get(chat_id, [])
        if len(history) > self.max_history:
            # Keep system-relevant context: trim oldest user/assistant pairs
            self.sessions[chat_id] = history[-self.max_history :]

    def clear_session(self, chat_id: int) -> None:
        """Clear conversation history for a user."""
        self.sessions.pop(chat_id, None)

    def update_tools(self, tools: List[Dict]) -> None:
        """Update available MCP tool definitions."""
        self.tools = tools

    async def chat(self, chat_id: int, user_message: str) -> Dict[str, Any]:
        """Send a message and get LLM response.

        Args:
            chat_id: Telegram chat ID (used as session key).
            user_message: User's text message.

        Returns:
            Dict with keys:
                - "text": LLM response text
                - "tool_calls": List of tool call dicts (if any)
                - "error": Error message (if failed)
        """
        history = self._get_history(chat_id)
        history.append({"role": "user", "content": user_message})

        messages = [{"role": "system", "content": OPERATOR_SYSTEM_PROMPT}] + history

        # Build request
        payload: Dict[str, Any] = {
            "model": self.model,
            "messages": messages,
            "temperature": self.temperature,
            "max_tokens": 1024,
        }

        # Add tools if available
        if self.tools:
            payload["tools"] = self.tools
            payload["tool_choice"] = "auto"

        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
        }

        try:
            async with aiohttp.ClientSession() as session:
                async with session.post(
                    f"{self.base_url}/chat/completions",
                    json=payload,
                    headers=headers,
                    timeout=aiohttp.ClientTimeout(total=30),
                ) as resp:
                    if resp.status != 200:
                        body = await resp.text()
                        logger.error("LLM API error %d: %s", resp.status, body[:500])
                        return {"text": "", "tool_calls": [], "error": f"API error {resp.status}"}

                    data = await resp.json()

            choice = data.get("choices", [{}])[0]
            message = choice.get("message", {})

            assistant_text = message.get("content", "") or ""
            tool_calls = message.get("tool_calls", [])

            # Store assistant response in history
            history.append({"role": "assistant", "content": assistant_text})
            self._truncate_history(chat_id)

            # Parse tool calls
            parsed_tool_calls = []
            for tc in tool_calls:
                func = tc.get("function", {})
                try:
                    args = json.loads(func.get("arguments", "{}"))
                except json.JSONDecodeError:
                    args = {}
                parsed_tool_calls.append({
                    "id": tc.get("id", ""),
                    "name": func.get("name", ""),
                    "arguments": args,
                })

            return {
                "text": assistant_text,
                "tool_calls": parsed_tool_calls,
                "error": None,
            }

        except Exception as e:
            logger.error("LLM chat error: %s", e)
            return {"text": "", "tool_calls": [], "error": str(e)}

    async def chat_with_tools(
        self,
        chat_id: int,
        user_message: str,
        tool_executor,
    ) -> str:
        """Full chat loop: send message, execute tool calls, return final text.

        Args:
            chat_id: Telegram chat ID.
            user_message: User's text message.
            tool_executor: Callable(tool_name, args) -> str that executes MCP tools.

        Returns:
            Final LLM response text (after tool execution, if any).
        """
        result = await self.chat(chat_id, user_message)

        if result.get("error"):
            return f"⚠️ Ошибка LLM: {result['error']}"

        # If no tool calls, return text directly
        if not result["tool_calls"]:
            return result["text"] or "🤔 Пустой ответ"

        # Execute tool calls and feed results back
        tool_results = []
        for tc in result["tool_calls"]:
            try:
                tool_result = await tool_executor(tc["name"], tc["arguments"])
                tool_results.append(f"🔧 {tc['name']}: {tool_result}")
            except Exception as e:
                tool_results.append(f"❌ {tc['name']}: {e}")

        # Add tool results to history and get follow-up response
        history = self._get_history(chat_id)
        tool_summary = "\n".join(tool_results)
        history.append({"role": "user", "content": f"[Результаты инструментов]\n{tool_summary}"})

        followup = await self.chat(chat_id, "")
        # Remove the empty user message we just added
        if history and history[-1]["role"] == "user" and history[-1]["content"] == "":
            history.pop()

        final_text = followup.get("text", "")
        if not final_text:
            return tool_summary

        return final_text
