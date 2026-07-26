# Тестирование rob_box_mcp_tools

Этот пакет содержит комплексные тесты для системы MCP Tools.

## Структура тестов

```
test/
├── conftest.py              # Общие фикстуры и mock объекты
├── README.md                # Эта документация
├── test_base.py             # Тесты базовых классов (MCPTool, MCPToolParameter, MCPToolResult)
├── test_registry.py         # Тесты MCPToolRegistry
├── test_llm_adapter.py      # Тесты LLMToolCallAdapter
├── test_mcp_server.py       # Тесты MCP Server (integration)
├── test_tools/              # Тесты отдельных инструментов
│   ├── test_navigation.py
│   ├── test_system.py
│   ├── test_perception.py
│   ├── test_mapping.py
│   ├── test_animation.py
│   └── test_sound.py
└── test_integration/        # Интеграционные тесты
    ├── test_full_workflow.py
    └── test_llm_integration.py
```

## Типы тестов

### 1. Unit тесты (`@pytest.mark.unit`)

**Характеристики:**
- Быстрые (< 100ms)
- Не требуют внешних зависимостей
- Используют mock объекты
- Тестируют отдельные компоненты изолированно

**Примеры:**
```python
@pytest.mark.unit
def test_tool_parameter_validation():
    # Тест валидации параметров
    pass

@pytest.mark.unit
def test_tool_result_creation():
    # Тест создания результатов
    pass
```

**Запуск:**
```bash
pytest -m unit
```

### 2. Integration тесты (`@pytest.mark.integration`)

**Характеристики:**
- Требуют ROS 2
- Тестируют взаимодействие компонентов
- Используют реальные ROS 2 publishers/subscribers
- Медленнее unit тестов

**Примеры:**
```python
@pytest.mark.integration
def test_mcp_server_registration():
    # Тест регистрации инструментов в MCP сервере
    pass

@pytest.mark.integration
def test_tool_execution_via_ros():
    # Тест выполнения инструмента через ROS 2
    pass
```

**Запуск:**
```bash
# Требуется запущенный ROS 2
pytest -m integration
```

### 3. LLM API тесты (`@pytest.mark.llm_api`)

**Характеристики:**
- Требуют API ключ (DEEPSEEK_API_KEY, MIMO_API_KEY, или LLM_API_KEY)
- Делают реальные запросы к LLM
- Тестируют tool_calls в реальных условиях
- Самые медленные (сетевые запросы)

**Примеры:**
```python
@pytest.mark.llm_api
def test_tool_calls_from_real_llm(skip_if_no_llm_api):
    # Тест получения tool_calls от реального LLM
    pass
```

**Запуск:**
```bash
# Установите API ключ
export DEEPSEEK_API_KEY="your-key"
pytest -m llm_api
```

### 4. Slow тесты (`@pytest.mark.slow`)

**Характеристики:**
- Тесты занимающие > 1 секунды
- Могут включать sleep, долгие вычисления, или множественные запросы

**Запуск (исключить slow тесты):**
```bash
pytest -m "not slow"
```

## Команды запуска

### Все тесты
```bash
pytest
```

### Только быстрые unit тесты
```bash
pytest -m unit
```

### Unit + Integration (без LLM API)
```bash
pytest -m "unit or integration"
```

### С coverage репортом
```bash
pytest --cov=rob_box_mcp_tools --cov-report=html
# Откройте test/htmlcov/index.html в браузере
```

### Один тестовый файл
```bash
pytest test/test_base.py
```

### Один тест
```bash
pytest test/test_base.py::test_mcp_tool_parameter_creation
```

### Verbose режим
```bash
pytest -vv
```

### Показать print statements
```bash
pytest -s
```

### Остановиться на первой ошибке
```bash
pytest -x
```

### Запустить failed тесты из прошлого запуска
```bash
pytest --lf
```

## Best Practices

### 1. Используйте fixtures для setup/teardown

```python
@pytest.fixture
def navigation_tool(mock_node):
    tool = NavigateToWaypointTool(mock_node)
    yield tool
    # Cleanup если нужен
```

### 2. Параметризуйте тесты для множественных случаев

```python
@pytest.mark.parametrize("input,expected", [
    ("valid", True),
    ("invalid", False),
    ("", False),
])
def test_validation(input, expected):
    assert validate(input) == expected
```

### 3. Изолируйте внешние зависимости

```python
# ❌ Плохо - зависит от реального ROS 2
def test_tool():
    node = rclpy.create_node('test')
    tool = Tool(node)
    
# ✅ Хорошо - использует mock
def test_tool(mock_node):
    tool = Tool(mock_node)
```

### 4. Тестируйте edge cases

```python
def test_tool_with_empty_params():
    # Что происходит с пустыми параметрами?
    
def test_tool_with_invalid_type():
    # Что происходит с неправильным типом?
    
def test_tool_with_missing_required():
    # Что происходит без обязательного параметра?
```

### 5. Проверяйте error handling

```python
def test_tool_handles_exception_gracefully():
    result = tool.execute(bad_params)
    assert result.success is False
    assert "error message" in result.error
```

## Coverage цели

- **Минимум:** 80% для всего пакета
- **Цель:** 90%+ для core компонентов (base, registry, adapter)
- **Исключения:** examples/, deprecated код

## Непрерывная интеграция

Тесты автоматически запускаются при:
- Push в branch
- Pull Request
- Pre-commit hook (опционально)

## Примеры

### Простой unit тест

```python
import pytest
from rob_box_mcp_tools.base import MCPToolParameter

@pytest.mark.unit
def test_parameter_validation():
    # Arrange
    param = MCPToolParameter(
        name="test",
        type="string",
        description="Test parameter",
        required=True
    )
    
    # Act
    schema = param.to_json_schema()
    
    # Assert
    assert schema["type"] == "string"
    assert "test" in schema["description"]
```

### Integration тест с mock node

```python
import pytest
from rob_box_mcp_tools.tools.navigation import NavigateToWaypointTool

@pytest.mark.unit
def test_navigation_tool(mock_node):
    # Arrange
    tool = NavigateToWaypointTool(mock_node)
    
    # Act
    result = tool.execute(waypoint="kitchen")
    
    # Assert
    assert result.success is True
    # Проверяем что publisher был вызван
    nav_pub = mock_node.get_publisher('/nav2/waypoint')
    assert len(nav_pub.published_messages) == 1
```

### LLM API тест

```python
import pytest
from openai import OpenAI
from rob_box_mcp_tools.llm_adapter import LLMToolCallAdapter

@pytest.mark.llm_api
@pytest.mark.slow
def test_real_llm_tool_call(skip_if_no_llm_api, llm_api_key, mock_node, sample_openai_tools):
    # Arrange
    client = OpenAI(api_key=llm_api_key, base_url="https://api.deepseek.com")
    adapter = LLMToolCallAdapter(mock_node)
    
    # Act
    response = client.chat.completions.create(
        model="deepseek-chat",
        messages=[{"role": "user", "content": "Navigate to kitchen"}],
        tools=sample_openai_tools
    )
    
    # Assert
    assert response.choices[0].message.tool_calls is not None
```

## Troubleshooting

### Тесты не находятся

```bash
# Убедитесь что вы в правильной директории
cd src/rob_box_mcp_tools

# Убедитесь что pytest.ini существует
ls -la pytest.ini

# Запустите с явным путём
pytest test/
```

### Import errors

```bash
# Установите пакет в dev режиме
pip install -e .

# Или добавьте в PYTHONPATH
export PYTHONPATH=$PYTHONPATH:$(pwd)
```

### ROS 2 не найден в integration тестах

```bash
# Source ROS 2
source /opt/ros/kilted/setup.bash

# Source workspace
source install/setup.bash

# Запустите тесты
pytest -m integration
```

## Полезные ссылки

- [pytest documentation](https://docs.pytest.org/)
- [pytest-cov documentation](https://pytest-cov.readthedocs.io/)
- [ROS 2 Testing](https://docs.ros.org/en/kilted/Tutorials/Testing.html)
