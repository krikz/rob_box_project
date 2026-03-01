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
        registry = MCPToolRegistry()

        assert len(registry) == 0
        assert registry.list_tools() == []

    def test_register_tool(self, mock_node):
        """Тест регистрации инструмента"""
        registry = MCPToolRegistry()
        tool = MockTestTool(mock_node)

        registry.register(tool)

        assert len(registry) == 1
        assert registry.get_tool("mock_test_tool") == tool
        assert "mock_test_tool" in registry

    def test_register_multiple_tools(self, mock_node):
        """Тест регистрации нескольких инструментов"""
        registry = MCPToolRegistry()
        tool1 = MockTestTool(mock_node)
        tool2 = AnotherMockTool(mock_node)

        registry.register(tool1)
        registry.register(tool2)

        assert len(registry) == 2
        assert registry.get_tool("mock_test_tool") == tool1
        assert registry.get_tool("another_mock_tool") == tool2

    def test_register_duplicate_tool(self, mock_node):
        """Тест регистрации дубликата инструмента (должна быть ошибка)"""
        registry = MCPToolRegistry()
        tool1 = MockTestTool(mock_node)
        tool2 = MockTestTool(mock_node)

        registry.register(tool1)
        # Реальная реализация бросает ValueError при дубликате
        with pytest.raises(ValueError, match="уже зарегистрирован"):
            registry.register(tool2)

    def test_unregister_tool(self, mock_node):
        """Тест отмены регистрации инструмента"""
        registry = MCPToolRegistry()
        tool = MockTestTool(mock_node)

        registry.register(tool)
        assert len(registry) == 1

        result = registry.unregister("mock_test_tool")
        assert result is True
        assert len(registry) == 0
        assert registry.get_tool("mock_test_tool") is None

    def test_get_tool_nonexistent(self, mock_node):
        """Тест получения несуществующего инструмента"""
        registry = MCPToolRegistry()

        result = registry.get_tool("nonexistent_tool")
        assert result is None

    def test_get_all_tools_empty(self, mock_node):
        """Тест получения всех инструментов из пустого реестра"""
        registry = MCPToolRegistry()

        tools = registry.list_tools()
        assert isinstance(tools, list)
        assert len(tools) == 0

    def test_get_tool_names(self, mock_node):
        """Тест получения списка имен инструментов"""
        registry = MCPToolRegistry()
        tool1 = MockTestTool(mock_node)
        tool2 = AnotherMockTool(mock_node)

        registry.register(tool1)
        registry.register(tool2)

        names = registry.list_tools()
        assert "mock_test_tool" in names
        assert "another_mock_tool" in names
        assert len(names) == 2

    def test_get_openai_tools(self, mock_node):
        """Тест получения инструментов в OpenAI формате"""
        registry = MCPToolRegistry()
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
        registry = MCPToolRegistry()
        tool = MockTestTool(mock_node)

        registry.register(tool)

        result = registry.execute("mock_test_tool", param1="test_value")

        assert result.success is True
        assert result.data["param1"] == "test_value"

    def test_execute_nonexistent_tool(self, mock_node):
        """Тест выполнения несуществующего инструмента"""
        registry = MCPToolRegistry()

        result = registry.execute("nonexistent_tool")

        assert result.success is False
        assert "не найден" in result.error or "not found" in result.error.lower()
