"""
Unit тесты для ConversationHistory.
"""

import unittest
from datetime import datetime
from rob_box_voice.core.conversation_history import ConversationHistory, Message


class TestMessage(unittest.TestCase):
    """Тесты для dataclass Message."""

    def test_message_creation(self):
        """Тест создания сообщения."""
        msg = Message(role="user", content="Привет")
        self.assertEqual(msg.role, "user")
        self.assertEqual(msg.content, "Привет")
        self.assertIsInstance(msg.timestamp, datetime)
        self.assertIsNone(msg.name)
        self.assertIsNone(msg.tool_call_id)
        self.assertIsNone(msg.tool_calls)

    def test_message_with_tool_fields(self):
        """Тест создания tool сообщения."""
        msg = Message(
            role="tool",
            content="Result",
            name="get_weather",
            tool_call_id="call_123",
        )
        self.assertEqual(msg.role, "tool")
        self.assertEqual(msg.content, "Result")
        self.assertEqual(msg.name, "get_weather")
        self.assertEqual(msg.tool_call_id, "call_123")


class TestConversationHistoryBasic(unittest.TestCase):
    """Базовые тесты для ConversationHistory."""

    def setUp(self):
        """Создать history перед каждым тестом."""
        self.history = ConversationHistory(max_messages=10)

    def test_initialization(self):
        """Тест инициализации."""
        self.assertEqual(self.history.max_messages, 10)
        self.assertEqual(self.history.get_message_count(), 0)

    def test_initialization_default(self):
        """Тест инициализации с параметрами по умолчанию."""
        history = ConversationHistory()
        self.assertEqual(history.max_messages, 50)

    def test_add_system_message(self):
        """Тест добавления системного сообщения."""
        self.history.add_system_message("Ты робот")
        self.assertEqual(self.history.get_message_count(), 1)
        messages = self.history.get_messages()
        self.assertEqual(len(messages), 1)
        self.assertEqual(messages[0]["role"], "system")
        self.assertEqual(messages[0]["content"], "Ты робот")

    def test_add_user_message(self):
        """Тест добавления сообщения пользователя."""
        self.history.add_user_message("Привет")
        self.assertEqual(self.history.get_message_count(), 1)
        messages = self.history.get_messages()
        self.assertEqual(messages[0]["role"], "user")
        self.assertEqual(messages[0]["content"], "Привет")

    def test_add_assistant_message(self):
        """Тест добавления сообщения ассистента."""
        self.history.add_assistant_message("Здравствуйте")
        self.assertEqual(self.history.get_message_count(), 1)
        messages = self.history.get_messages()
        self.assertEqual(messages[0]["role"], "assistant")
        self.assertEqual(messages[0]["content"], "Здравствуйте")

    def test_add_assistant_message_with_tool_calls(self):
        """Тест добавления сообщения ассистента с tool_calls."""
        tool_calls = [
            {
                "id": "call_123",
                "type": "function",
                "function": {"name": "get_weather", "arguments": '{"city": "Moscow"}'},
            }
        ]
        self.history.add_assistant_message("Проверяю погоду", tool_calls=tool_calls)
        messages = self.history.get_messages()
        self.assertEqual(messages[0]["role"], "assistant")
        self.assertIn("tool_calls", messages[0])
        self.assertEqual(len(messages[0]["tool_calls"]), 1)

    def test_add_tool_message(self):
        """Тест добавления tool сообщения."""
        self.history.add_tool_message("Temperature: 20C", "get_weather", "call_123")
        messages = self.history.get_messages()
        self.assertEqual(messages[0]["role"], "tool")
        self.assertEqual(messages[0]["content"], "Temperature: 20C")
        self.assertEqual(messages[0]["name"], "get_weather")
        self.assertEqual(messages[0]["tool_call_id"], "call_123")


class TestConversationHistoryAdvanced(unittest.TestCase):
    """Продвинутые тесты для ConversationHistory."""

    def setUp(self):
        """Создать history перед каждым тестом."""
        self.history = ConversationHistory(max_messages=5)

    def test_max_messages_limit(self):
        """Тест ограничения максимального количества сообщений."""
        # Добавить больше сообщений чем лимит
        for i in range(10):
            self.history.add_user_message(f"Message {i}")

        # Должно быть только max_messages сообщений
        self.assertEqual(self.history.get_message_count(), 5)

        # Должны остаться последние сообщения
        messages = self.history.get_messages()
        self.assertEqual(messages[-1]["content"], "Message 9")

    def test_max_messages_preserves_system(self):
        """Тест что системное сообщение сохраняется при превышении лимита."""
        self.history.add_system_message("System prompt")

        # Добавить много пользовательских сообщений
        for i in range(10):
            self.history.add_user_message(f"Message {i}")

        # Системное сообщение должно остаться
        messages = self.history.get_messages()
        self.assertEqual(messages[0]["role"], "system")
        self.assertEqual(messages[0]["content"], "System prompt")

    def test_clear(self):
        """Тест очистки истории."""
        self.history.add_user_message("Hello")
        self.history.add_assistant_message("Hi")
        self.history.clear()
        self.assertEqual(self.history.get_message_count(), 0)

    def test_clear_except_system(self):
        """Тест очистки истории кроме системных сообщений."""
        self.history.add_system_message("System")
        self.history.add_user_message("User")
        self.history.add_assistant_message("Assistant")

        self.history.clear_except_system()

        self.assertEqual(self.history.get_message_count(), 1)
        messages = self.history.get_messages()
        self.assertEqual(messages[0]["role"], "system")

    def test_get_raw_messages(self):
        """Тест получения сырых Message объектов."""
        self.history.add_user_message("Test")
        raw_messages = self.history.get_raw_messages()
        self.assertEqual(len(raw_messages), 1)
        self.assertIsInstance(raw_messages[0], Message)
        self.assertEqual(raw_messages[0].content, "Test")

    def test_get_last_message(self):
        """Тест получения последнего сообщения."""
        self.assertIsNone(self.history.get_last_message())

        self.history.add_user_message("First")
        self.history.add_user_message("Last")

        last = self.history.get_last_message()
        self.assertIsNotNone(last)
        self.assertEqual(last.content, "Last")

    def test_get_last_n_messages(self):
        """Тест получения последних N сообщений."""
        self.history.add_user_message("1")
        self.history.add_user_message("2")
        self.history.add_user_message("3")

        last_2 = self.history.get_last_n_messages(2)
        self.assertEqual(len(last_2), 2)
        self.assertEqual(last_2[0].content, "2")
        self.assertEqual(last_2[1].content, "3")

        # Проверка с n=0
        empty = self.history.get_last_n_messages(0)
        self.assertEqual(len(empty), 0)

    def test_has_system_message(self):
        """Тест проверки наличия системного сообщения."""
        self.assertFalse(self.history.has_system_message())

        self.history.add_user_message("User")
        self.assertFalse(self.history.has_system_message())

        self.history.add_system_message("System")
        self.assertTrue(self.history.has_system_message())

    def test_update_system_message_existing(self):
        """Тест обновления существующего системного сообщения."""
        self.history.add_system_message("Old system")
        self.history.add_user_message("User")

        self.history.update_system_message("New system")

        self.assertEqual(self.history.get_message_count(), 2)
        messages = self.history.get_messages()
        self.assertEqual(messages[0]["content"], "New system")

    def test_update_system_message_new(self):
        """Тест добавления нового системного сообщения через update."""
        self.history.add_user_message("User")

        self.history.update_system_message("New system")

        self.assertEqual(self.history.get_message_count(), 2)
        messages = self.history.get_messages()
        # Системное сообщение должно быть в начале
        self.assertEqual(messages[0]["role"], "system")
        self.assertEqual(messages[0]["content"], "New system")


class TestConversationHistoryEdgeCases(unittest.TestCase):
    """Тесты для edge cases."""

    def test_empty_history_operations(self):
        """Тест операций с пустой историей."""
        history = ConversationHistory()

        self.assertEqual(history.get_message_count(), 0)
        self.assertEqual(len(history.get_messages()), 0)
        self.assertEqual(len(history.get_raw_messages()), 0)
        self.assertIsNone(history.get_last_message())
        self.assertEqual(len(history.get_last_n_messages(5)), 0)
        self.assertFalse(history.has_system_message())

    def test_max_messages_one(self):
        """Тест с max_messages=1."""
        history = ConversationHistory(max_messages=1)
        history.add_user_message("First")
        history.add_user_message("Second")

        self.assertEqual(history.get_message_count(), 1)
        messages = history.get_messages()
        self.assertEqual(messages[0]["content"], "Second")


if __name__ == "__main__":
    unittest.main()
