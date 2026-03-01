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
Язык общения: русский, если оператор не пишет на другом языке.

## МУЗЫКА (ОБЯЗАТЕЛЬНО читай перед любым запросом на музыку)

Для воспроизведения музыки ВСЕГДА вызывай execute_music_code с реальным FoxDot/Renardo Python-кодом.
get_music_state — только для ПРОВЕРКИ текущего состояния. Один лишь get_music_state НЕ запускает музыку!

ИМЕНА ПЛЕЕРОВ: d1-d9 (ударные/play), p1-p9 (синтезатор), s1-s9 (семплы). ТОЛЬКО эти имена!
УДАРНЫЕ: d1 >> play("X.X.", ...) — через play() с буквами
СИНТЕЗАТОР: p1 >> blip([0,2,4], dur=1, amp=0.7) — через инструмент

ИЗВЕСТНЫЕ БУКВЫ (не нужен search_samples):
  X = кик (case="upper")   o = снейр   - = пауза    h = хай-хэт

АЛГОРИТМ для обычного запроса "красивая/весёлая/грустная мелодия":
  1. Сразу вызови execute_music_code с готовым кодом (не вызывай get_music_state первым!)
  2. НЕ НУЖЕН search_samples для простых запросов — используй известные буквы

ПРИМЕР красивой мелодии:
```python
Clock.bpm = 100
p1 >> blip([0,2,4,7,4,2], dur=0.5, amp=0.7, oct=5, cutoff=3000)
p2 >> keys([0,-2,0,3], dur=2, amp=0.4, oct=4)
d1 >> play("X-o-X-o-", amp=0.6)
```

ПРИМЕР спокойной/красивой:
```python
Clock.bpm = 80
p1 >> arpy([0,3,7,0,3,7], dur=0.5, amp=0.6, oct=5)
p2 >> epiano([0,-3,0,2], dur=4, amp=0.4, oct=4)
d1 >> play("X---o---", amp=0.5)
```

ЗАПРЕЩЕНО в коде: псевдонимы (rooster1=d1), Clock.bpm.set(N), play("kick"), play("bass_drum").
РАЗРЕШЕНО: d1.stop(), p1.stop(), Clock.clear() — для остановки.

## КОНВЕРТАЦИЯ МУЗ. НОТАЦИЙ (MML/RTTTL)

Формат `4a2 16c3 8#d2`: длительность + нота + октава. `#` = диез.
Длительности → доли бита (при bpm=120): 1=2.0, 2=1.0, 4=0.5, 8=0.25, 16=0.125, 8.=0.375
Ноты в MIDI (октава*12 + смещение): C=0, D=2, E=4, F=5, G=7, A=9, B=11, #+1
Пример: `a2` = октава 2, A = 9 → midinote = 2*12+9 = 33. `#d2` = 2*12+2+1 = 27.
Используй `p1 >> pluck(midinote=[...], dur=[...], amp=0.7)` для мелодии.
Левый канал (нижние ноты) → p1/p2 с oct смещением или отдельным голосом s1.

## МУЛЬТИ-ДЕЙСТВИЯ

Если запрос содержит несколько действий («произнеси И добавь звук», «сыграй И скажи»)
→ вызови ВСЕ инструменты последовательно, не ограничивайся одним.
Порядок: сначала speak_text, потом play_sound или execute_music_code."""


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
            "max_tokens": 4096,
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

            # Store assistant response in history (with tool_calls for proper follow-up format)
            if tool_calls:
                history.append({"role": "assistant", "content": assistant_text, "tool_calls": tool_calls})
            else:
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

        # Execute tool calls and collect results
        history = self._get_history(chat_id)
        tool_output_parts = []

        for tc in result["tool_calls"]:
            try:
                tool_result = await tool_executor(tc["name"], tc["arguments"])
            except Exception as e:
                tool_result = f"Ошибка: {e}"
            tool_output_parts.append(f"🔧 {tc['name']}: {tool_result}")
            # Add proper tool result message (OpenAI format — role:tool with tool_call_id)
            history.append({
                "role": "tool",
                "tool_call_id": tc["id"],
                "content": str(tool_result),
            })

        # Ask LLM to summarize results — direct API call without adding a user message
        messages = [{"role": "system", "content": OPERATOR_SYSTEM_PROMPT}] + history
        payload: Dict[str, Any] = {
            "model": self.model,
            "messages": messages,
            "temperature": self.temperature,
            "max_tokens": 4096,
        }
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
                        return "\n".join(tool_output_parts)
                    data = await resp.json()

            choice = data.get("choices", [{}])[0]
            final_text = choice.get("message", {}).get("content", "") or ""
            if final_text:
                history.append({"role": "assistant", "content": final_text})
                self._truncate_history(chat_id)
                return final_text
        except Exception as e:
            logger.error("LLM follow-up error: %s", e)

        # Fallback: return raw tool output
        return "\n".join(tool_output_parts)
