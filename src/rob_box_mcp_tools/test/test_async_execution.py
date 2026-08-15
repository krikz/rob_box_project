"""
test_async_execution.py - Unit тесты async-пути выполнения инструментов (PF-3).

Проверяет:
- MCPTool.aexecute() дефолт (to_thread обёртка над sync execute)
- MCPToolRegistry.aexecute() предпочитает async aexecute
- MemorySearchTool/MemoryContextTool/FaqSearchTool используют async store
  методы (asearch/aget_context) и не вызывают блокирующий sync путь
  при наличии async-реализации.

NOTE: tools/__init__.py импортирует navigation → rclpy, поэтому модуль
memory.py грузится напрямую через importlib (как в test_tools/test_mapping.py).
"""

from __future__ import annotations

import asyncio
import importlib.util
import sys
import types
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock

import pytest

from rob_box_mcp_tools.base import MCPTool, MCPToolParameter, MCPToolResult
from rob_box_mcp_tools.registry import MCPToolRegistry


class SyncOnlyTool(MCPTool):
    """Tool that only implements sync execute (no aexecute override)."""

    @property
    def name(self) -> str:
        return "sync_only"

    @property
    def description(self) -> str:
        return "sync only tool"

    @property
    def parameters(self):
        return [MCPToolParameter(name="x", type="string", description="x", required=False)]

    def execute(self, **kwargs) -> MCPToolResult:
        return MCPToolResult(success=True, message="sync executed", data=kwargs)


class AsyncTool(MCPTool):
    """Tool that overrides aexecute with a real async implementation."""

    @property
    def name(self) -> str:
        return "async_tool"

    @property
    def description(self) -> str:
        return "async tool"

    @property
    def parameters(self):
        return [MCPToolParameter(name="x", type="string", description="x", required=False)]

    def execute(self, **kwargs) -> MCPToolResult:
        raise AssertionError("sync execute should not be called when aexecute exists")

    async def aexecute(self, **kwargs) -> MCPToolResult:
        await asyncio.sleep(0)  # simulate async work
        return MCPToolResult(success=True, message="async executed", data=kwargs)


class TestMCPToolAexecuteDefault:
    def test_default_offloads_to_thread(self) -> None:
        tool = SyncOnlyTool()
        result = asyncio.run(tool.aexecute(x="1"))
        assert result.success is True
        assert result.message == "sync executed"


class TestRegistryAexecute:
    def test_prefers_async_aexecute(self) -> None:
        registry = MCPToolRegistry()
        registry.register(AsyncTool())
        result = asyncio.run(registry.aexecute("async_tool", x="1"))
        assert result.message == "async executed"

    def test_falls_back_to_sync_execute(self) -> None:
        registry = MCPToolRegistry()
        registry.register(SyncOnlyTool())
        result = asyncio.run(registry.aexecute("sync_only", x="1"))
        assert result.message == "sync executed"

    def test_unknown_tool_returns_error(self) -> None:
        registry = MCPToolRegistry()
        result = asyncio.run(registry.aexecute("nope"))
        assert result.success is False
        assert "не найден" in (result.error or "")

    def test_validation_runs_before_execution(self) -> None:
        registry = MCPToolRegistry()
        registry.register(AsyncTool())
        result = asyncio.run(registry.aexecute("async_tool"))
        # x is not required in AsyncTool, so this succeeds
        assert result.success is True


# ---------------------------------------------------------------------------
# Load tools/memory.py directly (tools/__init__ pulls in rclpy via navigation)
# ---------------------------------------------------------------------------


def _load_memory_module(monkeypatch):
    tools_pkg = types.ModuleType("rob_box_mcp_tools.tools")
    tools_pkg.__path__ = [
        str(Path(__file__).resolve().parents[1] / "rob_box_mcp_tools" / "tools")
    ]
    monkeypatch.setitem(sys.modules, "rob_box_mcp_tools.tools", tools_pkg)
    sys.modules.pop("rob_box_mcp_tools.tools.memory", None)

    module_path = (
        Path(__file__).resolve().parents[1]
        / "rob_box_mcp_tools" / "tools" / "memory.py"
    )
    spec = importlib.util.spec_from_file_location(
        "rob_box_mcp_tools.tools.memory", module_path
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def memory_tools(monkeypatch):
    return _load_memory_module(monkeypatch)


class _FakeVoiceMemory:
    """Fake with both sync and async search APIs."""

    def __init__(self) -> None:
        self.embedder = MagicMock()
        self.embedder.is_available.return_value = False
        self.asearch = AsyncMock(
            return_value=[
                {"role": "user", "content": "про кухню", "session_id": "s1", "score": 0.5, "source": "fts"}
            ]
        )
        self.aget_context = AsyncMock(
            return_value={
                "recent_turns": [
                    {"role": "user", "content": "про кухню", "session_id": "s1"}
                ],
                "facts": [],
                "total_turns": 1,
                "sessions": 1,
                "vec_enabled": False,
                "current_session": "s2",
            }
        )
        self.format_facts_for_prompt = MagicMock(return_value="")
        self.get_stats = MagicMock(
            return_value={"turn_count": 1, "session_count": 1, "fact_count": 0, "vec_count": 0, "db_size_kb": 1, "vec_enabled": False}
        )


class TestMemorySearchToolAexecute:
    def test_uses_async_search(self, memory_tools) -> None:
        node = MagicMock()
        node.voice_memory = _FakeVoiceMemory()
        tool = memory_tools.MemorySearchTool(node)

        result = asyncio.run(tool.aexecute(query="кухня", limit=5))

        assert result.success is True
        node.voice_memory.asearch.assert_awaited_once_with("кухня", limit=5)
        assert result.data["total"] == 1

    def test_no_memory_returns_error(self, memory_tools) -> None:
        node = MagicMock()
        node.voice_memory = None
        tool = memory_tools.MemorySearchTool(node)

        result = asyncio.run(tool.aexecute(query="кухня"))
        assert result.success is False


class TestMemoryContextToolAexecute:
    def test_uses_async_context(self, memory_tools) -> None:
        node = MagicMock()
        node.voice_memory = _FakeVoiceMemory()
        tool = memory_tools.MemoryContextTool(node)

        result = asyncio.run(tool.aexecute(limit=10, query="кухня"))

        assert result.success is True
        node.voice_memory.aget_context.assert_awaited_once()
        assert result.data["stats"]["total_sessions"] == 1


class TestFaqSearchToolAexecute:
    class _FakeFaqStore:
        def __init__(self) -> None:
            self.asearch = AsyncMock(
                return_value=[
                    {"question": "Где?", "answer": "Там.", "category": "general"}
                ]
            )

        def search(self, *args, **kwargs):  # pragma: no cover
            raise AssertionError("sync search should not be called when asearch exists")

    def test_uses_async_search(self, memory_tools) -> None:
        node = MagicMock()
        node.faq_store = self._FakeFaqStore()
        node.event_profile = MagicMock()
        node.event_profile.event_id = "open-day-2026"
        tool = memory_tools.FaqSearchTool(node)

        result = asyncio.run(tool.aexecute(query="где", limit=3))

        assert result.success is True
        node.faq_store.asearch.assert_awaited_once()
        assert result.data["found"] == 1

    def test_no_faq_store_returns_error(self, memory_tools) -> None:
        node = MagicMock()
        node.faq_store = None
        tool = memory_tools.FaqSearchTool(node)

        result = asyncio.run(tool.aexecute(query="где"))
        assert result.success is False
