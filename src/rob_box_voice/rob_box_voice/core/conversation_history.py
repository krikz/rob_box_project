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

    def __init__(self, max_turns: int = 10):
        """
        Инициализация истории диалога.

        Args:
            max_turns: Максимальное количество инференсов (user+assistant пар) в истории.
                       Системный промпт не считается. При переполнении дропается самый
                       старый тёрн целиком (user + все до следующего user).
        """
        self.max_turns = max_turns
        self._messages: List[Message] = []

    def add_system_message(self, content: str) -> None:
        """
        Добавить системное сообщение.

        Args:
            content: Текст системного сообщения
        """
        message = Message(role="system", content=content)
        self._add_message(message)

    def add_user_message(self, content: str) -> None:
        """
        Добавить сообщение пользователя.

        Args:
            content: Текст сообщения пользователя
        """
        message = Message(role="user", content=content)
        self._add_message(message)

    def add_assistant_message(
        self, content: str, tool_calls: Optional[List[Dict[str, Any]]] = None
    ) -> None:
        """
        Добавить сообщение ассистента.

        Args:
            content: Текст ответа ассистента
            tool_calls: Опциональный список вызовов инструментов
        """
        message = Message(role="assistant", content=content, tool_calls=tool_calls)
        self._add_message(message)

    def add_assistant_message_with_tools(
        self, content: Optional[str], tool_calls: List[Dict[str, Any]]
    ) -> None:
        """
        Добавить сообщение ассистента с tool calls (alias для add_assistant_message).
        
        Args:
            content: Текст ответа ассистента (может быть None)
            tool_calls: Список вызовов инструментов
        """
        message = Message(role="assistant", content=content or "", tool_calls=tool_calls)
        self._add_message(message)

    def add_tool_message(
        self, content: str, name: str, tool_call_id: str
    ) -> None:
        """
        Добавить сообщение от инструмента (tool result).

        Args:
            content: Результат выполнения инструмента
            name: Имя инструмента
            tool_call_id: ID вызова инструмента
        """
        message = Message(
            role="tool", content=content, name=name, tool_call_id=tool_call_id
        )
        self._add_message(message)

    def _add_message(self, message: Message) -> None:
        """
        Внутренний метод для добавления сообщения с проверкой лимита.

        Args:
            message: Сообщение для добавления
        """
        self._messages.append(message)
        self._trim_to_max_turns()

    def _trim_to_max_turns(self) -> None:
        """
        Обрезаем историю до max_turns полных инференсов.

        Один тёрн = user message + всё что за ним до следующего user (assistant,
        tool results, tool_calls). Системный промпт всегда сохраняется на позиции 0.
        При переполнении дропаем самый старый тёрн целиком.
        """
        system_msgs = [m for m in self._messages if m.role == "system"]
        other_msgs = [m for m in self._messages if m.role != "system"]

        while True:
            user_indices = [i for i, m in enumerate(other_msgs) if m.role == "user"]
            if len(user_indices) <= self.max_turns:
                break
            # Дропаем с первого user до (не включая) второго user
            end = user_indices[1] if len(user_indices) > 1 else len(other_msgs)
            del other_msgs[user_indices[0]:end]

        self._messages = system_msgs + other_msgs

    def get_messages(self) -> List[Dict[str, Any]]:
        """
        Получить все сообщения в формате для LLM API.

        Returns:
            Список сообщений в формате словарей
        """
        messages = []
        for msg in self._messages:
            message_dict = {"role": msg.role, "content": msg.content}

            if msg.name is not None:
                message_dict["name"] = msg.name

            if msg.tool_call_id is not None:
                message_dict["tool_call_id"] = msg.tool_call_id

            if msg.tool_calls is not None:
                message_dict["tool_calls"] = msg.tool_calls

            messages.append(message_dict)

        return messages

    def get_raw_messages(self) -> List[Message]:
        """
        Получить все сообщения как объекты Message.

        Returns:
            Список объектов Message
        """
        return self._messages.copy()

    def clear(self) -> None:
        """Очистить всю историю диалога."""
        self._messages.clear()

    def clear_except_system(self) -> None:
        """Очистить историю, но оставить системные сообщения."""
        self._messages = [msg for msg in self._messages if msg.role == "system"]

    def get_message_count(self) -> int:
        """
        Получить количество сообщений в истории.

        Returns:
            Количество сообщений
        """
        return len(self._messages)

    def get_last_message(self) -> Optional[Message]:
        """
        Получить последнее сообщение.

        Returns:
            Последнее сообщение или None если история пуста
        """
        return self._messages[-1] if self._messages else None

    def get_last_n_messages(self, n: int) -> List[Message]:
        """
        Получить последние N сообщений.

        Args:
            n: Количество сообщений

        Returns:
            Список последних N сообщений
        """
        return self._messages[-n:] if n > 0 else []

    def has_system_message(self) -> bool:
        """
        Проверить наличие системного сообщения.

        Returns:
            True если есть хотя бы одно системное сообщение
        """
        return any(msg.role == "system" for msg in self._messages)

    def update_system_message(self, content: str) -> None:
        """
        Обновить (или добавить) системное сообщение.
        Если системное сообщение уже есть, обновляет первое найденное.
        Если нет - добавляет в начало.

        Args:
            content: Новый текст системного сообщения
        """
        # Найти первое системное сообщение
        for i, msg in enumerate(self._messages):
            if msg.role == "system":
                self._messages[i] = Message(role="system", content=content)
                return

        # Если не найдено, добавить в начало
        self._messages.insert(0, Message(role="system", content=content))

    def remove_tool_messages(self) -> None:
        """
        Очистить НЕПОЛНЫЕ tool-цепочки из истории (артефакты barge-in).

        Правила:
        - assistant(tool_calls) + ВСЕ соответствующие tool results → ОСТАВЛЯЕМ как есть.
          Это валидный формат OpenAI API и содержит полную историю что робот делал/говорил.
        - assistant(tool_calls) БЕЗ следующих tool messages (barge-in прервал выполнение)
          → конвертируем в assistant(content=<текст из speak_text>) или удаляем.
        - Orphaned tool messages (без предшествующего assistant) → удаляем.
        """
        new_messages: List[Message] = []
        i = 0
        while i < len(self._messages):
            msg = self._messages[i]
            if msg.role == "assistant" and msg.tool_calls:
                # Считаем сколько tool results идёт следом
                j = i + 1
                while j < len(self._messages) and self._messages[j].role == "tool":
                    j += 1
                tool_count = j - (i + 1)
                expected_count = len(msg.tool_calls)

                if tool_count == expected_count:
                    # Полная цепочка — оставляем оригинал как есть (валидно для API)
                    for k in range(i, j):
                        new_messages.append(self._messages[k])
                else:
                    # Неполная цепочка (barge-in) — конвертируем в текстовый ответ
                    spoken_texts: List[str] = []
                    for tc in (msg.tool_calls or []):
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
                        new_messages.append(Message(role="assistant", content=" ".join(spoken_texts)))
                    # Иначе молча дропаем (play_sound без текста — не нужно в истории)
                i = j
            elif msg.role == "tool":
                # Orphaned tool message — удаляем
                i += 1
            else:
                new_messages.append(msg)
                i += 1
        self._messages = new_messages

    def remove_orphaned_user_messages(self) -> int:
        """
        Удалить подряд идущие user-сообщения без assistant-ответа между ними (артефакт barge-in).

        Проблема: при barge-in первый user message добавляется в историю, но LLM не отвечает.
        Потом добавляется второй user message — в истории два подряд user без assistant.
        DeepSeek видит оба вопроса и пытается ответить на оба.

        Оставляет только последнее user-сообщение в хвосте.
        Возвращает количество удалённых сообщений.
        """
        removed = 0
        while len(self._messages) >= 2:
            if self._messages[-1].role == 'user' and self._messages[-2].role == 'user':
                self._messages.pop(-2)
                removed += 1
            else:
                break
        return removed
