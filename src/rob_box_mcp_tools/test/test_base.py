"""
test_base.py - Unit тесты для базовых классов MCP Tools

Тестирует:
- MCPToolParameter
- MCPToolResult  
- MCPTool (abstract base class)
"""

import json
from typing import List

import pytest

from rob_box_mcp_tools.base import MCPTool, MCPToolParameter, MCPToolResult


# ============================================================
# Tests for MCPToolParameter
# ============================================================


class TestMCPToolParameter:
    """Тесты для MCPToolParameter"""

    @pytest.mark.unit
    def test_parameter_creation_minimal(self):
        """Тест создания параметра с минимальными полями"""
        param = MCPToolParameter(name="test_param", type="string", description="Test parameter", required=True)

        assert param.name == "test_param"
        assert param.type == "string"
        assert param.description == "Test parameter"
        assert param.required is True

    @pytest.mark.unit
    def test_parameter_with_enum(self):
        """Тест параметра с enum"""
        param = MCPToolParameter(name="choice", type="string", description="Choose option", required=True, enum=["a", "b", "c"])

        schema = param.to_json_schema()
        assert schema["enum"] == ["a", "b", "c"]

    @pytest.mark.unit
    def test_parameter_with_default(self):
        """Тест параметра со значением по умолчанию"""
        param = MCPToolParameter(name="count", type="integer", description="Count", required=False, default=10)

        schema = param.to_json_schema()
        assert schema["default"] == 10

    @pytest.mark.unit
    def test_parameter_to_json_schema_complete(self):
        """Тест полной конвертации параметра в JSON Schema"""
        param = MCPToolParameter(
            name="test_param",
            type="string",
            description="Test parameter",
            required=True,
            enum=["a", "b", "c"],
            default="a"
        )
        
        schema = param.to_json_schema()
        assert schema["type"] == "string"
        assert schema["description"] == "Test parameter"
        assert schema["enum"] == ["a", "b", "c"]
        assert schema["default"] == "a"


@pytest.mark.unit
class TestMCPToolResult:
    """Тесты для класса MCPToolResult"""

    def test_create_success_result(self):
        """Тест создания успешного результата"""
        result = MCPToolResult(success=True, message="Operation completed", data={"key": "value"})

        assert result.success is True
        assert result.message == "Operation completed"  # Проверяем переданное message
        assert result.data == {"key": "value"}
        assert result.error is None

    @pytest.mark.parametrize(
        "success,message,data,error",
        [
            (True, "Success!", {"result": "ok"}, None),
            (False, "Failed", None, "Error occurred"),
            (True, "Done", {"count": 5}, None),
        ],
    )
    def test_tool_result_creation(self, success, message, data, error):
        result = MCPToolResult(success=success, message=message, data=data, error=error)
        assert result.success == success
        assert result.message == message
        if data:
            assert result.data == data
        if error:
            assert result.error == error