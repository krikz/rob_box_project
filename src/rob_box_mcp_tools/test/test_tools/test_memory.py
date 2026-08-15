"""
test_memory.py - Unit тесты для инструментов памяти

Тестирует:
- MemorySaveTool: сохранение факта, пустой fact, отсутствие voice_memory
- MemorySearchTool: поиск, пустой query, отсутствие voice_memory
- MemoryContextTool: контекст, отсутствие voice_memory
- FaqSearchTool: поиск по FAQ, отсутствие faq_store/event_id

Покрытие: success + error + invalid params.
"""

import sys
from unittest.mock import MagicMock, Mock

import pytest

# Mock ROS 2 модулей перед импортом tools
for _mod in [
    "rclpy",
    "rclpy.node",
    "rclpy.action",
    "rclpy.qos",
]:
    sys.modules.setdefault(_mod, MagicMock())

from rob_box_mcp_tools.tools.memory import (  # noqa: E402
    MemorySaveTool,
    MemorySearchTool,
    MemoryContextTool,
    FaqSearchTool,
)


class _FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, msg):
        self.messages.append(("info", msg))

    def warning(self, msg):
        self.messages.append(("warning", msg))

    def error(self, msg):
        self.messages.append(("error", msg))

    def debug(self, msg):
        self.messages.append(("debug", msg))


class _FakeNode:
    def __init__(self, voice_memory=None, faq_store=None, event_profile=None):
        self._logger = _FakeLogger()
        self.voice_memory = voice_memory
        self.faq_store = faq_store
        self.event_profile = event_profile

    def get_logger(self):
        return self._logger


class _FakeVoiceMemory:
    def __init__(self):
        self.saved = []
        self.search_results = []
        self.embedder = Mock()
        self.embedder.is_available.return_value = False

    def save_fact(self, fact, category="general"):
        self.saved.append((fact, category))
        return len(self.saved)

    def search(self, query, limit=5):
        return self.search_results

    def get_context(self, limit=10, query=None):
        return {
            "recent_turns": [
                {"role": "user", "content": "привет", "session_id": "s1"},
            ],
            "facts": [{"fact": "Пользователя зовут Алексей", "category": "name"}],
            "total_turns": 1,
            "sessions": 1,
            "vec_enabled": False,
            "current_session": "s1",
        }

    def format_facts_for_prompt(self):
        return "- Пользователя зовут Алексей"

    def get_stats(self):
        return {"db_size_kb": 12}


class _FakeFaqStore:
    def __init__(self, results=None):
        self.results = results or []

    def search(self, query, event_id, limit=3):
        return self.results


@pytest.mark.unit
class TestMemorySaveTool:
    def test_tool_metadata(self):
        tool = MemorySaveTool(_FakeNode(voice_memory=_FakeVoiceMemory()))
        assert tool.name == "memory_save"
        assert [p.name for p in tool.parameters] == ["fact", "category"]

    def test_execute_no_voice_memory(self):
        tool = MemorySaveTool(_FakeNode())

        result = tool.execute(fact="Пользователя зовут Алексей")

        assert result.success is False
        assert "не инициализирована" in result.message

    def test_execute_empty_fact(self):
        memory = _FakeVoiceMemory()
        tool = MemorySaveTool(_FakeNode(voice_memory=memory))

        result = tool.execute(fact="   ")

        assert result.success is False
        assert "не может быть пустым" in result.message

    def test_execute_success(self):
        memory = _FakeVoiceMemory()
        tool = MemorySaveTool(_FakeNode(voice_memory=memory))

        result = tool.execute(fact="Пользователя зовут Алексей", category="name")

        assert result.success is True
        assert result.data["fact"] == "Пользователя зовут Алексей"
        assert result.data["category"] == "name"
        assert result.data["fact_id"] == 1
        assert "Факт сохранён" in result.message
        assert memory.saved == [("Пользователя зовут Алексей", "name")]

    def test_execute_save_error_caught(self):
        memory = _FakeVoiceMemory()
        memory.save_fact = Mock(side_effect=RuntimeError("db locked"))
        tool = MemorySaveTool(_FakeNode(voice_memory=memory))

        result = tool.execute(fact="что-то важное")

        assert result.success is False
        assert "Ошибка сохранения" in result.message
        assert "db locked" in result.message


@pytest.mark.unit
class TestMemorySearchTool:
    def test_tool_metadata(self):
        tool = MemorySearchTool(_FakeNode(voice_memory=_FakeVoiceMemory()))
        assert tool.name == "memory_search"
        assert [p.name for p in tool.parameters] == ["query", "limit"]

    def test_execute_no_voice_memory(self):
        tool = MemorySearchTool(_FakeNode())

        result = tool.execute(query="что я просил")

        assert result.success is False
        assert "не инициализирована" in result.message

    def test_execute_empty_query(self):
        tool = MemorySearchTool(_FakeNode(voice_memory=_FakeVoiceMemory()))

        result = tool.execute(query="")

        assert result.success is False
        assert "не может быть пустым" in result.message

    def test_execute_success(self):
        memory = _FakeVoiceMemory()
        memory.search_results = [
            {"role": "user", "content": "хочу пиццу", "session_id": "s1", "score": 0.9, "source": "fts"},
            {"role": "assistant", "content": "ок", "session_id": "s1", "score": 0.5},
        ]
        tool = MemorySearchTool(_FakeNode(voice_memory=memory))

        result = tool.execute(query="пицца", limit=5)

        assert result.success is True
        assert result.data["total"] == 2
        assert result.data["has_more"] is False
        assert result.data["results"][0]["content"] == "хочу пиццу"
        assert result.data["results"][0]["score"] == 0.9
        assert "Найдено 2" in result.message

    def test_execute_limit_capped_at_20(self):
        memory = _FakeVoiceMemory()
        memory.search_results = []
        tool = MemorySearchTool(_FakeNode(voice_memory=memory))

        result = tool.execute(query="пицца", limit=100)

        assert result.success is True
        assert result.data["limit"] == 20

    def test_execute_search_error_caught(self):
        memory = _FakeVoiceMemory()
        memory.search = Mock(side_effect=RuntimeError("fts broken"))
        tool = MemorySearchTool(_FakeNode(voice_memory=memory))

        result = tool.execute(query="пицца")

        assert result.success is False
        assert "Ошибка поиска" in result.message


@pytest.mark.unit
class TestMemoryContextTool:
    def test_tool_metadata(self):
        tool = MemoryContextTool(_FakeNode(voice_memory=_FakeVoiceMemory()))
        assert tool.name == "memory_context"
        assert [p.name for p in tool.parameters] == ["limit", "query"]

    def test_execute_no_voice_memory(self):
        tool = MemoryContextTool(_FakeNode())

        result = tool.execute()

        assert result.success is False
        assert "не инициализирована" in result.message

    def test_execute_success(self):
        memory = _FakeVoiceMemory()
        tool = MemoryContextTool(_FakeNode(voice_memory=memory))

        result = tool.execute(limit=5)

        assert result.success is True
        assert len(result.data["recent_turns"]) == 1
        assert result.data["recent_turns"][0]["role"] == "user"
        assert result.data["facts_block"] == "- Пользователя зовут Алексей"
        assert result.data["stats"]["total_sessions"] == 1
        assert "1 фактов" in result.message or "1 факт" in result.message

    def test_execute_limit_capped_at_30(self):
        memory = _FakeVoiceMemory()
        tool = MemoryContextTool(_FakeNode(voice_memory=memory))

        result = tool.execute(limit=100)

        assert result.success is True  # cap молча

    def test_execute_error_caught(self):
        memory = _FakeVoiceMemory()
        memory.get_context = Mock(side_effect=RuntimeError("boom"))
        tool = MemoryContextTool(_FakeNode(voice_memory=memory))

        result = tool.execute()

        assert result.success is False
        assert "Ошибка получения контекста" in result.message


@pytest.mark.unit
class TestFaqSearchTool:
    def test_tool_metadata(self):
        tool = FaqSearchTool(_FakeNode())
        assert tool.name == "faq_search"
        assert tool.execution_type.value == "fast"
        assert tool.read_only is True
        assert tool.destructive is False

    def test_execute_no_faq_store(self):
        tool = FaqSearchTool(_FakeNode())

        result = tool.execute(query="когда начинается")

        assert result.success is False
        assert "не инициализировано" in result.error

    def test_execute_no_event_id(self):
        faq = _FakeFaqStore()
        tool = FaqSearchTool(_FakeNode(faq_store=faq))

        result = tool.execute(query="когда начинается")

        assert result.success is False
        assert "event_id" in result.error

    def test_execute_no_results(self):
        faq = _FakeFaqStore(results=[])
        event = Mock()
        event.event_id = "event-1"
        tool = FaqSearchTool(_FakeNode(faq_store=faq, event_profile=event))

        result = tool.execute(query="парковка")

        assert result.success is True
        assert result.data["found"] == 0
        assert "ничего не найдено" in result.message

    def test_execute_with_results(self):
        faq = _FakeFaqStore(results=[
            {"question": "Где парковка?", "answer": "У входа", "category": "parking"},
        ])
        event = Mock()
        event.event_id = "event-1"
        tool = FaqSearchTool(_FakeNode(faq_store=faq, event_profile=event))

        result = tool.execute(query="парковка")

        assert result.success is True
        assert result.data["found"] == 1
        assert result.data["results"][0]["question"] == "Где парковка?"
        assert "Найдено 1" in result.message

    def test_execute_limit_clamped(self):
        faq = _FakeFaqStore(results=[])
        event = Mock()
        event.event_id = "event-1"
        tool = FaqSearchTool(_FakeNode(faq_store=faq, event_profile=event))

        result = tool.execute(query="парковка", limit=100)

        assert result.success is True  # limit=10 clamp

    def test_execute_search_error_caught(self):
        faq = _FakeFaqStore()
        faq.search = Mock(side_effect=RuntimeError("faq broken"))
        event = Mock()
        event.event_id = "event-1"
        tool = FaqSearchTool(_FakeNode(faq_store=faq, event_profile=event))

        result = tool.execute(query="парковка")

        assert result.success is False
        assert "Ошибка поиска по FAQ" in result.error
