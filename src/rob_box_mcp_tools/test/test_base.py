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
    def test_parameter_validation_required(self):
        """Тест валидации обязательных параметров"""
        param = MCPToolParameter(name="required_param", type="string", description="Required parameter", required=True)

        # Required parameter отсутствует
        result = param.validate(None)
        assert result.success is False
        assert "required" in result.error.lower()

        # Required parameter присутствует
        result = param.validate("some_value")
        assert result.success is True

    @pytest.mark.unit
    def test_parameter_type_validation(self):
        """Тест валидации типов параметров"""
        # String parameter
        param = MCPToolParameter(name="test", type="string", description="Test", required=True)
        
        # Валидные значения
        assert param.validate("hello").success is True
        
        # Невалидные типы
        assert param.validate(123).success is False
        assert param.validate({"key": "value"}).success is False
        
    def test_parameter_enum_validation(self):
        """Тест валидации enum параметров"""
        param = MCPToolParameter(
            name="choice",
            type="string",
            description="Choice parameter",
            required=True,
            enum=["option1", "option2", "option3"]
        )
        
        # Валидное значение
        result = param.validate("option1")
        assert result.success is True
        
        # Невалидное значение
        result = param.validate("invalid")
        assert result.success is False
        assert "not in allowed values" in result.error


@pytest.mark.unit
class TestMCPToolResult:
    """Тесты для класса MCPToolResult"""

    def test_create_success_result(self):
        """Тест создания успешного результата"""
        result = MCPToolResult(success=True, message="Operation completed", data={"key": "value"})

        assert result.success is True
        assert result.message == "Выполнено успешно"
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
```

**Запуск:**
```bash
cd src/rob_box_mcp_tools
pytest test/test_base.py -v
```

## Следующие шаги

После этого коммита нужно создать:
1. Unit тесты для всех компонентов
2. Integration тесты
3. E2E тесты с LLM

Все тесты будут использовать эту инфраструктуру.