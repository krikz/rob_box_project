"""
test_registry.py - Unit тесты для MCPToolRegistry

Тестирует регистрацию, поиск и управление инструментами.
"""

import pytest

from rob_box_mcp_tools.base import MCPTool, MCPToolParameter, MCPToolResult
from rob_box_mcp_tools.registry import MCPToolRegistry


# ============================================================
# Mock Tool для тестирования
# ============================================================


class MockTestTool(MCPTool):
    """Простой мок инструмент для тестирования"""

    @property
    def name(self) -> str:
        return "mock_test_tool"

    @property
    def description(self) -> str:
        return "A mock tool for testing"

    @property
    def parameters(self):
        return [MCPToolParameter(name="param1", type="string", description="Test param", required=True)]

    def execute(self, **kwargs) -> MCPToolResult:
        return MCPToolResult(success=True, message="Mock execution", data=kwargs)


class AnotherMockTool(MCPTool):
    """Еще один мок инструмент"""

    @property
    def name(self) -> str:
        return "another_mock_tool"

    @property
    def description(self) -> str:
        return "Another mock tool"

    @property
    def parameters(self):
        return []

    def execute(self, **kwargs) -> MCPToolResult:
        return MCPToolResult(success=True, message="Another execution")


# ============================================================
# Тесты
# ============================================================


@pytest.mark.unit
class TestMCPToolRegistry:
    """Тесты для MCPToolRegistry"""

    def test_registry_creation(self, mock_node):
        """Тест создания реестра"""
        registry = MCPToolRegistry(mock_node)

        assert registry.node == mock_node
        assert len(registry.get_all_tools()) == 0

    def test_register_tool(self, mock_node):
        """Тест регистрации инструмента"""
        registry = MCPToolRegistry(mock_node)
        tool = MockTestTool(mock_node)

        registry.register(tool)

        assert len(registry.get_all_tools()) == 1
        assert registry.get_tool("mock_test_tool") == tool

    def test_register_multiple_tools(self, mock_node):
        """Тест регистрации нескольких инструментов"""
        registry = MCPToolRegistry(mock_node)
        tool1 = MockTestTool(mock_node)
        tool2 = AnotherMockTool(mock_node)

        registry.register(tool1)
        registry.register(tool2)

        assert len(registry.get_all_tools()) == 2
        assert registry.get_tool("mock_test_tool") == tool1
        assert registry.get_tool("another_mock_tool") == tool2

    def test_register_duplicate_tool(self, mock_node):
        """Тест регистрации дубликата инструмента"""
        registry = MCPToolRegistry(mock_node)
        tool1 = MockTestTool(mock_node)
        tool2 = MockTestTool(mock_node)

        registry.register(tool1)
        registry.register(tool2)  # Должен заменить первый

        assert len(registry.get_all_tools()) == 1
        assert registry.get_tool("mock_test_tool") == tool2

    def test_unregister_tool(self, mock_node):
        """Тест отмены регистрации инструмента"""
        registry = MCPToolRegistry(mock_node)
        tool = MockTestTool(mock_node)

        registry.register(tool)
        assert len(registry.get_all_tools()) == 1

        registry.unregister("mock_test_tool")
        assert len(registry.get_all_tools()) == 0
        assert registry.get_tool("mock_test_tool") is None

    def test_get_tool_nonexistent(self, mock_node):
        """Тест получения несуществующего инструмента"""
        registry = MCPToolRegistry(mock_node)

        result = registry.get_tool("nonexistent_tool")
        assert result is None

    def test_get_all_tools_empty(self, mock_node):
        """Тест получения всех инструментов из пустого реестра"""
        registry = MCPToolRegistry(mock_node)

        tools = registry.get_all_tools()
        assert isinstance(tools, list)
        assert len(tools) == 0

    def test_get_tool_names(self, mock_node):
        """Тест получения списка имен инструментов"""
        registry = MCPToolRegistry(mock_node)
        tool1 = MockTestTool(mock_node)
        tool2 = AnotherMockTool(mock_node)

        registry.register(tool1)
        registry.register(tool2)

        names = registry.get_tool_names()
        assert "mock_test_tool" in names
        assert "another_mock_tool" in names
        assert len(names) == 2

    def test_get_openai_tools(self, mock_node):
        """Тест получения инструментов в OpenAI формате"""
        registry = MCPToolRegistry(mock_node)
        tool = MockTestTool(mock_node)

        registry.register(tool)

        openai_tools = registry.get_openai_tools()

        assert isinstance(openai_tools, list)
        assert len(openai_tools) == 1
        assert openai_tools[0]["type"] == "function"
        assert openai_tools[0]["function"]["name"] == "mock_test_tool"
        assert "parameters" in openai_tools[0]["function"]

    def test_execute_tool(self, mock_node):
        """Тест выполнения инструмента через реестр"""
        registry = MCPToolRegistry(mock_node)
        tool = MockTestTool(mock_node)

        registry.register(tool)

        result = registry.execute_tool("mock_test_tool", param1="test_value")

        assert result.success is True
        assert result.data["param1"] == "test_value"

    def test_execute_nonexistent_tool(self, mock_node):
        """Тест выполнения несуществующего инструмента"""
        registry = MCPToolRegistry(mock_node)

        result = registry.execute_tool("nonexistent_tool")

        assert result.success is False
        assert "not found" in result.error.lower()
