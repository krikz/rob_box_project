"""Unit tests for the VoiceMemory MCP tools (memory_save / memory_search / memory_context).

Covers TASK-036 acceptance:

* ``memory_save`` creates voice_facts rows via ``VoiceMemory.save_fact``
* ``memory_search`` finds conversation turns AND facts saved via
  ``memory_save`` (facts come back with ``source="fact"``)
* ``memory_context`` returns recent turns + facts + stats
* Tools degrade gracefully (informative error) when ``voice_memory``
  is not initialised on the node.

Uses a real :class:`VoiceMemory` on a temp file so the FTS5/voice_facts
round-trip is exercised end-to-end.
"""

from __future__ import annotations

import sys
from unittest.mock import MagicMock

import pytest

# Mock ROS2 modules before importing anything from rob_box_mcp_tools
# (mirrors test_music.py so the tools package imports without rclpy).
for _mod in [
    "rclpy",
    "rclpy.node",
    "rclpy.action",
    "rclpy.qos",
    "std_msgs",
    "std_msgs.msg",
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav2_msgs",
    "nav2_msgs.action",
    "action_msgs",
    "action_msgs.srv",
    "action_msgs.msg",
]:
    sys.modules.setdefault(_mod, MagicMock())

from rob_box_mcp_tools.tools.memory import (  # noqa: E402
    MemoryContextTool,
    MemorySaveTool,
    MemorySearchTool,
)
from rob_box_voice.core.voice_memory import VoiceMemory  # noqa: E402


class _FakeNode:
    """Minimal node stub exposing ``voice_memory`` and a logger."""

    def __init__(self, voice_memory=None):
        self.voice_memory = voice_memory
        self.log = []

    def get_logger(self):
        class _L:
            def __init__(self, log):
                self.log = log

            def info(self, msg):
                self.log.append(("info", msg))

            def error(self, msg):
                self.log.append(("error", msg))

            def warning(self, msg):
                self.log.append(("warning", msg))

            def debug(self, msg):
                self.log.append(("debug", msg))

        return _L(self.log)


@pytest.fixture
def memory(tmp_path) -> VoiceMemory:
    mem = VoiceMemory(db_path=str(tmp_path / "voice_memory.db"), session_id="sess_test")
    yield mem
    mem.close()


@pytest.fixture
def save_tool(memory) -> MemorySaveTool:
    return MemorySaveTool(_FakeNode(memory))


@pytest.fixture
def search_tool(memory) -> MemorySearchTool:
    return MemorySearchTool(_FakeNode(memory))


@pytest.fixture
def context_tool(memory) -> MemoryContextTool:
    return MemoryContextTool(_FakeNode(memory))


# ---------------------------------------------------------------------------
# memory_save
# ---------------------------------------------------------------------------


class TestMemorySaveTool:
    def test_saves_fact(self, save_tool, memory) -> None:
        result = save_tool.execute(fact="Пользователя зовут Алексей", category="name")
        assert result.success is True
        assert result.data["fact_id"] > 0
        facts = memory.get_facts(category="name")
        assert len(facts) == 1
        assert facts[0]["fact"] == "Пользователя зовут Алексей"

    def test_rejects_empty_fact(self, save_tool) -> None:
        result = save_tool.execute(fact="   ")
        assert result.success is False
        assert "пустым" in result.message

    def test_returns_error_when_memory_missing(self) -> None:
        tool = MemorySaveTool(_FakeNode(voice_memory=None))
        result = tool.execute(fact="x")
        assert result.success is False
        assert "не инициализирована" in result.message


# ---------------------------------------------------------------------------
# memory_search
# ---------------------------------------------------------------------------


class TestMemorySearchTool:
    def test_finds_turns(self, search_tool, memory) -> None:
        memory.save_turn("user", "мы говорили про любимую кухню")
        result = search_tool.execute(query="кухню")
        assert result.success is True
        contents = [r["content"] for r in result.data["results"]]
        assert any("кухню" in c for c in contents)

    def test_finds_facts_saved_via_memory_save(self, search_tool, memory) -> None:
        """Acceptance: voice_facts created through memory_save are reachable
        through memory_search."""
        memory.save_fact("Пользователя зовут Алексей", category="name")
        result = search_tool.execute(query="Алексей")
        assert result.success is True
        facts = [r for r in result.data["results"] if r.get("source") == "fact"]
        assert len(facts) >= 1
        assert facts[0]["content"] == "Пользователя зовут Алексей"
        assert facts[0]["role"] == "fact"
        assert facts[0]["category"] == "name"

    def test_returns_error_when_memory_missing(self) -> None:
        tool = MemorySearchTool(_FakeNode(voice_memory=None))
        result = tool.execute(query="кухня")
        assert result.success is False
        assert "не инициализирована" in result.message

    def test_rejects_empty_query(self, search_tool) -> None:
        result = search_tool.execute(query="  ")
        assert result.success is False
        assert "query" in result.message


# ---------------------------------------------------------------------------
# memory_context
# ---------------------------------------------------------------------------


class TestMemoryContextTool:
    def test_returns_turns_and_facts(self, context_tool, memory) -> None:
        memory.save_turn("user", "в прошлой сессии говорили про кухню",
                         session_id="sess_old")
        memory.save_fact("Предпочитает краткие ответы", category="preference")
        result = context_tool.execute()
        assert result.success is True
        data = result.data
        assert len(data["recent_turns"]) == 1
        assert data["recent_turns"][0]["content"] == "в прошлой сессии говорили про кухню"
        assert len(data["facts"]) == 1
        assert "Предпочитает краткие ответы" in data["facts_block"]
        assert data["stats"]["total_turns"] == 1
        assert data["stats"]["total_sessions"] >= 1

    def test_context_with_query_searches(self, context_tool, memory) -> None:
        memory.save_turn("user", "любимый цвет синий")
        result = context_tool.execute(query="синий")
        assert result.success is True
        assert len(result.data["recent_turns"]) == 1

    def test_returns_error_when_memory_missing(self) -> None:
        tool = MemoryContextTool(_FakeNode(voice_memory=None))
        result = tool.execute()
        assert result.success is False
        assert "не инициализирована" in result.message
