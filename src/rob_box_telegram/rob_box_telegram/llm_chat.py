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
import re
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
  3. КОД ДОЛЖЕН БЫТЬ КОРОТКИМ: МАКСИМУМ 8 строк (Clock.bpm + 3-4 паттерна inline)!
     НЕ создавай def функции / Clock.future цепочки — только прямые паттерны!

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

    @staticmethod
    def _sanitize_messages(messages: List[Dict]) -> List[Dict]:
        """Remove orphaned role:tool messages and dangling assistant+tool_calls.

        DeepSeek requires that every role:tool message is immediately preceded
        by a role:assistant message that contains tool_calls. After truncation
        or partial history loading this invariant can break, causing HTTP 400.
        """
        result = []
        for i, msg in enumerate(messages):
            if msg.get("role") == "tool":
                # Only include if previous message is assistant with tool_calls
                prev = result[-1] if result else None
                if prev and prev.get("role") == "assistant" and prev.get("tool_calls"):
                    result.append(msg)
                # else: drop orphaned tool message
            elif msg.get("role") == "assistant" and msg.get("tool_calls"):
                # Check if the NEXT message in original list is a tool result
                # If not (e.g. history was truncated mid-sequence), strip tool_calls
                next_msg = messages[i + 1] if i + 1 < len(messages) else None
                if next_msg and next_msg.get("role") == "tool":
                    result.append(msg)
                else:
                    # Strip tool_calls so it's a plain assistant message
                    clean = {k: v for k, v in msg.items() if k != "tool_calls"}
                    result.append(clean)
            else:
                result.append(msg)
        return result

    def clear_session(self, chat_id: int) -> None:
        """Clear conversation history for a user."""
        self.sessions.pop(chat_id, None)

    def update_tools(self, tools: List[Dict]) -> None:
        """Update available MCP tool definitions."""
        self.tools = tools

    @staticmethod
    def _parse_dsml_tool_calls(text: str) -> List[Dict]:
        """Parse DeepSeek legacy DSML tool call format embedded in response text.

        DeepSeek sometimes outputs tool calls as text like:
          <｜DSML｜function_calls><｜DSML｜invoke name="speak_text">
            <｜DSML｜parameter name="text" string="true">...</｜DSML｜parameter>
          </｜DSML｜invoke></｜DSML｜function_calls>
        Returns list of parsed tool call dicts (same format as API tool_calls).
        """
        # ｜ = U+FF5C fullwidth vertical line (DeepSeek uses this)
        if "\uff5cDSML\uff5c" not in text and "|DSML|" not in text:
            return []

        results = []
        import uuid
        # Handle both <｜DSML｜ (fullwidth) and <|DSML| (ascii)
        invoke_pattern = re.compile(
            r'<[|\uff5c]DSML[|\uff5c]invoke\s+name=["\']([^"\']+)["\']>(.*?)<[|\uff5c]/DSML[|\uff5c]invoke>',
            re.DOTALL,
        )
        param_pattern = re.compile(
            r'<[|\uff5c]DSML[|\uff5c]parameter\s+name=["\']([^"\']+)["\'][^>]*>(.*?)<[|\uff5c]/DSML[|\uff5c]parameter>',
            re.DOTALL,
        )
        for invoke_match in invoke_pattern.finditer(text):
            tool_name = invoke_match.group(1).strip()
            invoke_body = invoke_match.group(2)
            args = {}
            for param_match in param_pattern.finditer(invoke_body):
                pname = param_match.group(1).strip()
                pvalue = param_match.group(2).strip()
                args[pname] = pvalue
            results.append({
                "id": f"dsml_{uuid.uuid4().hex[:8]}",
                "name": tool_name,
                "arguments": args,
            })
        return results

    @staticmethod
    def _strip_dsml(text: str) -> str:
        """Remove DSML function_call blocks from response text."""
        if "\uff5cDSML\uff5c" not in text and "|DSML|" not in text:
            return text
        cleaned = re.sub(
            r'<[|\uff5c]DSML[|\uff5c]function_calls>.*?<[|\uff5c]/DSML[|\uff5c]function_calls>',
            '',
            text,
            flags=re.DOTALL,
        )
        return cleaned.strip()

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

        messages = self._sanitize_messages(
            [{"role": "system", "content": OPERATOR_SYSTEM_PROMPT}] + history
        )

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

        Handles multi-round tool calling and DeepSeek legacy DSML text format.
        Loops up to _MAX_TOOL_ROUNDS times while LLM keeps returning tool calls.
        """
        _MAX_TOOL_ROUNDS = 5

        result = await self.chat(chat_id, user_message)
        if result.get("error"):
            return f"⚠️ Ошибка LLM: {result['error']}"

        history = self._get_history(chat_id)
        all_tool_output: List[str] = []

        for _round in range(_MAX_TOOL_ROUNDS):
            tool_calls = result["tool_calls"]
            response_text = result["text"] or ""

            # Also check for DeepSeek DSML tool calls embedded in text
            dsml_calls = self._parse_dsml_tool_calls(response_text)
            if dsml_calls and not tool_calls:
                logger.info("Detected %d DSML tool call(s) in response text", len(dsml_calls))
                tool_calls = dsml_calls

            # No tool calls → we're done
            if not tool_calls:
                final = self._strip_dsml(response_text)
                return final or ("✅ Выполнено" if all_tool_output else "🤔 Пустой ответ")

            # Execute all tool calls in this round
            for tc in tool_calls:
                try:
                    tool_result = await tool_executor(tc["name"], tc["arguments"])
                except Exception as e:
                    tool_result = f"Ошибка: {e}"
                all_tool_output.append(f"🔧 {tc['name']}: {tool_result}")
                history.append({
                    "role": "tool",
                    "tool_call_id": tc["id"],
                    "content": str(tool_result),
                })

            # Ask LLM for next step (may return more tool calls or final text)
            messages = self._sanitize_messages(
                [{"role": "system", "content": OPERATOR_SYSTEM_PROMPT}] + history
            )
            payload: Dict[str, Any] = {
                "model": self.model,
                "messages": messages,
                "temperature": self.temperature,
                "max_tokens": 4096,
            }
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
                            logger.error("LLM follow-up error %d: %s", resp.status, body[:300])
                            return "\n".join(all_tool_output)
                        data = await resp.json()
            except Exception as e:
                logger.error("LLM follow-up error: %s", e)
                return "\n".join(all_tool_output)

            choice = data.get("choices", [{}])[0]
            message = choice.get("message", {})
            followup_text = message.get("content", "") or ""
            followup_tool_calls_raw = message.get("tool_calls", [])

            # Store in history
            if followup_tool_calls_raw:
                history.append({"role": "assistant", "content": followup_text, "tool_calls": followup_tool_calls_raw})
            else:
                history.append({"role": "assistant", "content": followup_text})
            self._truncate_history(chat_id)

            # Parse tool calls for next round
            parsed = []
            for tc in followup_tool_calls_raw:
                func = tc.get("function", {})
                try:
                    args = json.loads(func.get("arguments", "{}"))
                except json.JSONDecodeError:
                    args = {}
                parsed.append({"id": tc.get("id", ""), "name": func.get("name", ""), "arguments": args})

            result = {"text": followup_text, "tool_calls": parsed, "error": None}

        # Exceeded max rounds — return last text or tool output
        final = self._strip_dsml(result.get("text", "") or "")
        return final or "\n".join(all_tool_output)

