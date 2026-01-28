# rob_box_mcp_tools

MCP-подобная система инструментов для интеграции LLM с Rob Box роботом.

## 📋 Описание

Этот пакет предоставляет архитектуру для создания и управления инструментами (tools), которые может вызывать LLM (Large Language Model) для управления роботом. Система основана на концепции Model Context Protocol (MCP) и интегрируется с OpenAI Tool Calls format API.

## 🔄 Архитектурный рефакторинг (2025)

> **⚠️ В процессе**: Пакет проходит рефакторинг для улучшения поддержки AI-assisted разработки ("vibe coding")

**Проблема**: Сложная асинхронная логика (600+ LOC) затрудняет понимание и тестирование

**Решение**: Разбиение на модули <300 LOC с четким разделением ответственности

**Документация**:
- [Архитектура Vibe Coding](../../docs/development/VIBE_CODING_ARCHITECTURE.md) - Принципы и паттерны
- [План рефакторинга MCP Tools](../../docs/development/REFACTORING_PLAN_MCP_TOOLS.md) - Детальный план
- [Quick Reference для AI](../../docs/development/VIBE_CODING_QUICK_REF.md) - Быстрый справочник

**Целевая структура**:
```
rob_box_mcp_tools/
├── core/          # Execution engine, задачи, реестр
├── streaming/     # Обработка streaming chunks
├── adapters/      # LLM провайдеры (Qwen, DeepSeek)
├── tools/         # Инструменты робота
└── tests/         # Юнит и интеграционные тесты
```

**Статус**: Планирование завершено, начало реализации - Q1 2025

---

## 🏗️ Архитектура

```
┌─────────────┐
│  Dialogue   │ ← Пользователь (голос)
│    Node     │
└──────┬──────┘
       │ tools parameter
       ▼
┌─────────────┐     tool_calls     ┌─────────────┐
│  LLM API    │ ─────────────────→ │  LLM Tool   │
│ (DeepSeek/  │                     │   Adapter   │
│  Qwen/etc)  │                     │             │
└─────────────┘                     └──────┬──────┘
                                           │ /mcp/execute
                                           ▼
                                    ┌─────────────┐
                                    │ MCP Server  │
                                    │  (Registry) │
                                    └──────┬──────┘
                                           │
                     ┌─────────────────────┼─────────────────────┐
                     ▼                     ▼                     ▼
              ┌────────────┐        ┌────────────┐       ┌────────────┐
              │ Navigation │        │   System   │       │ Perception │
              │   Tools    │        │   Tools    │       │   Tools    │
              └────────────┘        └────────────┘       └────────────┘
                     │                     │                     │
                     ▼                     ▼                     ▼
              ┌────────────────────────────────────────────────────┐
              │              ROS 2 Topics / Services               │
              └────────────────────────────────────────────────────┘
```

## 🛠️ Доступные инструменты

### Navigation (Навигация)
- `navigate_to_waypoint` - Навигация к именованной точке
- `move_direction` - Движение в направлении (вперёд/назад/влево/вправо)
- `stop_navigation` - Остановка движения
- `list_waypoints` - Получить список доступных точек

### System (Управление системой)
- `set_volume` - Установить громкость TTS
- `set_pitch` - Установить высоту голоса
- `set_speed` - Установить скорость речи
- `get_robot_status` - Получить статус робота

### Perception (Восприятие)
- `get_perception_context` - Получить контекст восприятия
- `get_battery_level` - Получить уровень заряда батареи

### Mapping (Картографирование)
- `start_mapping` - Начать новое картографирование
- `continue_mapping` - Продолжить картографирование
- `finish_mapping` - Завершить картографирование

### Animation (Анимация)
- `play_animation` - Запустить LED анимацию с указанной длительностью (2-30 сек)

### Sound (Звуковые эффекты)
- `play_sound` - Воспроизвести звуковой эффект

## 🚀 Использование

### Запуск MCP Server

```bash
ros2 run rob_box_mcp_tools mcp_server
```

MCP Server автоматически:
1. Регистрирует все доступные инструменты
2. Публикует список инструментов в `/mcp/tools` (формат DeepSeek)
3. Слушает запросы на выполнение в `/mcp/execute`
4. Публикует результаты в `/mcp/result`

### Интеграция с dialogue_node

Пример интеграции см. в `examples/dialogue_node_mcp_integration.py`

Основные шаги:
1. Создать `LLMToolCallAdapter`
2. Подписаться на `/mcp/tools` для получения списка инструментов
3. Передать `tools` в LLM API (OpenAI-compatible) при запросе
4. Обработать `tool_calls` из streaming ответа
5. Выполнить инструменты через MCP адаптер
6. Отправить результаты обратно в LLM

```python
from rob_box_mcp_tools.llm_adapter import LLMToolCallAdapter

# В __init__ вашей ноды:
self.mcp_adapter = LLMToolCallAdapter(self)
self.tools_sub = self.create_subscription(
    String, "/mcp/tools", self.on_tools_update, 10
)

# При запросе к LLM:
stream = self.client.chat.completions.create(
    model="your-model",
    messages=messages,
    tools=self.available_tools,  # ← Передаём инструменты
    stream=True
)

# Обработка tool_calls:
if delta.tool_calls:
    # Выполнить через MCP адаптер
    result = self.mcp_adapter.execute_tool_call_sync(
        tool_name, parameters
    )
```

## 📦 Структура пакета

```
rob_box_mcp_tools/
├── rob_box_mcp_tools/
│   ├── __init__.py
│   ├── base.py                 # Базовые классы (MCPTool, MCPToolResult)
│   ├── registry.py             # Реестр инструментов
│   ├── mcp_server.py           # Главная ROS 2 нода
│   ├── llm_adapter.py     # Адаптер для LLM API (OpenAI-compatible)
│   └── tools/
│       ├── __init__.py
│       ├── navigation.py       # Инструменты навигации
│       ├── system.py           # Системные инструменты
│       ├── perception.py       # Инструменты восприятия
│       ├── mapping.py          # Картографирование
│       ├── animation.py        # LED анимации
│       └── sound.py            # Звуковые эффекты
├── examples/
│   └── dialogue_node_mcp_integration.py  # Пример интеграции
├── package.xml
├── setup.py
└── README.md
```

## 🔧 Создание своего инструмента

```python
from rob_box_mcp_tools.base import MCPTool, MCPToolParameter, MCPToolResult
from typing import List

class MyCustomTool(MCPTool):
    """Описание вашего инструмента"""
    
    @property
    def name(self) -> str:
        return "my_custom_tool"
    
    @property
    def description(self) -> str:
        return "Описание для LLM (на русском)"
    
    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="param1",
                type="string",
                description="Описание параметра",
                required=True,
                enum=["option1", "option2"]  # Опционально
            )
        ]
    
    def execute(self, param1: str) -> MCPToolResult:
        """Выполнить инструмент"""
        self.log_info(f"Выполнение с param1={param1}")
        
        # Ваша логика здесь
        # ...
        
        return MCPToolResult(
            success=True,
            data={"result": "something"},
            message="Успешно выполнено"
        )
```

Затем зарегистрируйте в `mcp_server.py`:
```python
self.registry.register(MyCustomTool(self))
```

## 📝 ROS 2 Topics

### Публикуемые MCP Server:
- `/mcp/tools` (String) - JSON список доступных инструментов (DeepSeek format)
- `/mcp/result` (String) - JSON результаты выполнения

### Принимаемые MCP Server:
- `/mcp/execute` (String) - JSON запросы на выполнение инструментов

Формат запроса:
```json
{
    "tool_name": "navigate_to_waypoint",
    "parameters": {
        "waypoint": "кухня"
    },
    "request_id": "optional_uuid"
}
```

Формат ответа:
```json
{
    "tool_name": "navigate_to_waypoint",
    "request_id": "same_uuid",
    "result": {
        "success": true,
        "data": {"waypoint": "кухня", "coordinates": {...}},
        "message": "Иду к точке кухня"
    }
}
```

## 🧪 Тестирование

### Проверка доступных инструментов:
```bash
ros2 topic echo /mcp/tools
```

### Ручное выполнение инструмента:
```bash
ros2 topic pub --once /mcp/execute std_msgs/String \
  'data: "{\"tool_name\": \"list_waypoints\", \"parameters\": {}}"'

# Смотрим результат:
ros2 topic echo /mcp/result --once
```

## 🔗 Связанные компоненты

- `rob_box_voice` - Голосовой ассистент (dialogue_node, command_node)
- `rob_box_perception` - Система восприятия (context_aggregator, reflection_node)
- `nav2` - Навигационный стек
- `rtabmap` - SLAM и картографирование

## 📚 Ссылки

- [OpenAI Tool Calls format Documentation](https://api-docs.deepseek.com/guides/tool_calls)
- [Model Context Protocol (MCP)](https://modelcontextprotocol.io/)
- [Rob Box Project Documentation](../../docs/)

## 📄 Лицензия

MIT
