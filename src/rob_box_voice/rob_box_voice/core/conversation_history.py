"""
Управление историей диалога для voice assistant.

Этот модуль предоставляет класс ConversationHistory для управления:
- Историей сообщений (user, assistant, system, tool)
- Ограничением размера истории (max_messages)
- Очисткой истории
- Добавлением различных типов сообщений
"""

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

    def __init__(self, max_messages: int = 50):
        """
        Инициализация истории диалога.

        Args:
            max_messages: Максимальное количество сообщений в истории (по умолчанию 50)
        """
        self.max_messages = max_messages
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

        # Удалить старые сообщения если превышен лимит
        # Но всегда оставляем системное сообщение если оно есть
        if len(self._messages) > self.max_messages:
            # Найти индекс первого не-системного сообщения
            first_non_system_idx = 0
            for i, msg in enumerate(self._messages):
                if msg.role != "system":
                    first_non_system_idx = i
                    break

            # Удалить старые не-системные сообщения
            if first_non_system_idx < len(self._messages):
                self._messages.pop(first_non_system_idx)

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
        Удалить все сообщения с ролью 'tool' и assistant messages с tool_calls из истории.
        
        API требует: assistant message с tool_calls должен следовать за tool messages.
        При новом диалоге старые tool results недействительны, поэтому удаляем и tool messages,
        и assistant messages с tool_calls, чтобы не оставлять orphaned tool_calls.
        """
        self._messages = [
            msg for msg in self._messages 
            if msg.role != "tool" and not (msg.role == "assistant" and msg.tool_calls)
        ]
