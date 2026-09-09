"""
Управление историей диалога для voice assistant.

Этот модуль предоставляет класс ConversationHistory для управления:
- Историей сообщений (user, assistant, system, tool)
- Ограничением размера истории (max_messages)
- Очисткой истории
- Добавлением различных типов сообщений
"""

import json
from typing import List, Dict, Any, Optional
from dataclasses import dataclass, field
from datetime import datetime


@dataclass
class Message:
    """Сообщение в истории диалога."""

    role: str  # "user", "assistant", "system", "tool"
    content: str
    timestamp: datetime = field(default_factory=datetime.now)
    name: Optional[str] = None  # Для tool messages
    tool_call_id: Optional[str] = None  # Для tool messages
    tool_calls: Optional[List[Dict[str, Any]]] = None  # Для assistant с tool calls


class ConversationHistory:
    """
    Управление историей диалога.

    Пример использования:
        history = ConversationHistory(max_messages=20)

        # Добавить системный промпт
        history.add_system_message("Ты - робот-помощник.")

        # Добавить сообщение пользователя
        history.add_user_message("Привет!")

        # Добавить ответ ассистента
        history.add_assistant_message("Здравствуйте!")

        # Получить все сообщения
        messages = history.get_messages()

        # Очистить историю
        history.clear()
    """

    def __init__(self, max_messages: int = 50, *, max_turns: Optional[int] = None):
        """Initialize dialog history.

        Args:
            max_messages: Max number of messages retained (default: 50). When
                exceeded, the oldest non-system message is dropped (FIFO). The
                first system message is always preserved at index 0.
            max_turns: Backward-compatible alias for ``max_messages`` — older
                production code counted dialog turns (user+assistant pairs)
                but the buffer semantic is the same: at most N entries.
        """
        if max_turns is not None:
            max_messages = max_turns
        self.max_messages: int = max_messages
        self.max_turns: int = max_messages  # legacy alias
        self._messages: List[Message] = []

    def add_system_message(self, content: str) -> None:
        """Append a system message."""
        self._add_message(Message(role="system", content=content))

    def add_user_message(self, content: str) -> None:
        """Append a user message."""
        self._add_message(Message(role="user", content=content))

    def add_assistant_message(
        self, content: str, tool_calls: Optional[List[Dict[str, Any]]] = None
    ) -> None:
        """Append an assistant message, optionally with tool calls."""
        self._add_message(
            Message(role="assistant", content=content, tool_calls=tool_calls)
        )

    def add_assistant_message_with_tools(
        self, content: Optional[str], tool_calls: List[Dict[str, Any]]
    ) -> None:
        """Alias for ``add_assistant_message`` accepting explicit tool_calls."""
        self._add_message(
            Message(role="assistant", content=content or "", tool_calls=tool_calls)
        )

    def add_tool_message(
        self, content: str, name: str, tool_call_id: str
    ) -> None:
        """Append a tool result message."""
        self._add_message(
            Message(
                role="tool", content=content, name=name, tool_call_id=tool_call_id
            )
        )

    def _add_message(self, message: Message) -> None:
        """Append a message and trim the buffer to ``max_messages``."""
        self._messages.append(message)
        self._trim_to_max_messages()

    def _trim_to_max_messages(self) -> None:
        """Trim FIFO so total length never exceeds ``max_messages``.

        System messages are protected: the first one (if any) always stays at
        index 0; only non-system entries are trimmed. This matches the
        ``test_conversation_history`` contract that asserts the system prompt
        survives long sessions.
        """
        if self.max_messages <= 0:
            self._messages = []
            return
        if len(self._messages) <= self.max_messages:
            return

        first_system_idx = next(
            (i for i, m in enumerate(self._messages) if m.role == "system"),
            None,
        )
        if first_system_idx is not None:
            system_msg: Optional[Message] = self._messages[first_system_idx]
            rest = (
                self._messages[:first_system_idx]
                + self._messages[first_system_idx + 1 :]
            )
            keep = max(self.max_messages - 1, 0)
            rest = rest[-keep:] if keep > 0 else []
            self._messages = [system_msg] + rest
        else:
            self._messages = self._messages[-self.max_messages :]

    def get_messages(self) -> List[Dict[str, Any]]:
        """Return all messages in LLM-API format."""
        result: List[Dict[str, Any]] = []
        for msg in self._messages:
            entry = {"role": msg.role, "content": msg.content}
            if msg.name is not None:
                entry["name"] = msg.name
            if msg.tool_call_id is not None:
                entry["tool_call_id"] = msg.tool_call_id
            if msg.tool_calls is not None:
                entry["tool_calls"] = msg.tool_calls
            result.append(entry)
        return result

    def get_raw_messages(self) -> List[Message]:
        """Return raw Message objects (defensive copy)."""
        return self._messages.copy()

    def clear(self) -> None:
        """Drop all messages."""
        self._messages.clear()

    def clear_except_system(self) -> None:
        """Drop all but system messages."""
        self._messages = [m for m in self._messages if m.role == "system"]

    def get_message_count(self) -> int:
        """Return the current number of messages."""
        return len(self._messages)

    def get_last_message(self) -> Optional[Message]:
        """Return the last message or ``None`` if empty."""
        return self._messages[-1] if self._messages else None

    def get_last_n_messages(self, n: int) -> List[Message]:
        """Return the last ``n`` messages (or fewer)."""
        return self._messages[-n:] if n > 0 else []

    def has_system_message(self) -> bool:
        """Return True if at least one system message is present."""
        return any(m.role == "system" for m in self._messages)

    def update_system_message(self, content: str) -> None:
        """Replace or insert the system message at the start."""
        for i, msg in enumerate(self._messages):
            if msg.role == "system":
                self._messages[i] = Message(role="system", content=content)
                return
        self._messages.insert(0, Message(role="system", content=content))

    def remove_tool_messages(self) -> None:
        """Strip incomplete tool-call chains left over from barge-in.

        Rules:
          * assistant(tool_calls) + matching tool results → kept as-is.
          * assistant(tool_calls) WITHOUT follow-up tool results → collapsed
            into a plain assistant(content=<speech text>) entry.
          * Orphaned tool messages (without preceding assistant(tool_calls))
            are deleted.
        """
        new_messages: List[Message] = []
        i = 0
        while i < len(self._messages):
            msg = self._messages[i]
            if msg.role == "assistant" and msg.tool_calls:
                j = i + 1
                while j < len(self._messages) and self._messages[j].role == "tool":
                    j += 1
                tool_count = j - (i + 1)
                expected_count = len(msg.tool_calls)

                if tool_count == expected_count:
                    for k in range(i, j):
                        new_messages.append(self._messages[k])
                else:
                    spoken_texts: List[str] = []
                    for tc in msg.tool_calls or []:
                        fn = tc.get("function", {})
                        if fn.get("name") == "speak_text":
                            try:
                                args = json.loads(fn.get("arguments", "{}"))
                                text = args.get("text", "").strip()
                                if text:
                                    spoken_texts.append(text)
                            except (json.JSONDecodeError, AttributeError, TypeError):
                                pass
                    if spoken_texts:
                        new_messages.append(
                            Message(role="assistant", content=" ".join(spoken_texts))
                        )
                i = j
            elif msg.role == "tool":
                i += 1
            else:
                new_messages.append(msg)
                i += 1
        self._messages = new_messages

    def remove_orphaned_user_messages(self) -> int:
        """Remove trailing user-message duplicates that lack an assistant reply.

        Barge-in can append two consecutive user messages without any
        assistant response in between. Keeping just the most recent trailing
        user message gives the LLM a clean question to answer.

        Returns the number of removed messages.
        """
        removed = 0
        while len(self._messages) >= 2:
            if (
                self._messages[-1].role == "user"
                and self._messages[-2].role == "user"
            ):
                self._messages.pop(-2)
                removed += 1
            else:
                break
        return removed
