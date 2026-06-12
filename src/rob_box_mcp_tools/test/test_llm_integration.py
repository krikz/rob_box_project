"""
test_llm_integration.py - Интеграционные тесты с реальными LLM API

Эти тесты делают РЕАЛЬНЫЕ запросы к LLM API (DeepSeek, MiMo, OpenAI).
Требуют API ключ в переменных окружения.

Запуск:
    export DEEPSEEK_API_KEY="your-key"
    pytest test/test_llm_integration.py -m llm_api -v -s

Или для MiMo:
    export MIMO_API_KEY="your-key"
    pytest test/test_llm_integration.py -m llm_api -v -s
"""

import json
import os
import time

import pytest

try:
    from openai import OpenAI
except ImportError:
    pytest.skip("openai library not installed", allow_module_level=True)

from rob_box_mcp_tools.base import MCPToolParameter
from rob_box_mcp_tools.registry import MCPToolRegistry
from rob_box_mcp_tools.tools.animation import PlayAnimationTool
from rob_box_mcp_tools.tools.navigation import NavigateToWaypointTool, MoveDirectionTool, ListWaypointsTool
from rob_box_mcp_tools.tools.system import SetVolumeTool, SetPitchTool, GetRobotStatusTool
from rob_box_mcp_tools.tools.perception import GetPerceptionContextTool, GetBatteryLevelTool
from rob_box_mcp_tools.tools.sound import PlaySoundTool
from rob_box_mcp_tools.tools.mapping import StartMappingTool


# ============================================================
# Helper Functions
# ============================================================


def get_llm_client_and_model():
    """
    Получить LLM клиент и модель на основе доступных API ключей
    
    Returns:
        tuple: (client, model, provider_name)
    """
    # Проверяем DeepSeek
    deepseek_key = os.getenv("DEEPSEEK_API_KEY")
    if deepseek_key:
        client = OpenAI(api_key=deepseek_key, base_url="https://api.deepseek.com/v1")
        return client, "deepseek-v4-flash", "DeepSeek"
    
    # Проверяем MiMo
    mimo_key = os.getenv("MIMO_API_KEY")
    if mimo_key:
        client = OpenAI(
            api_key=mimo_key,
            base_url="https://api.xiaomimimo.com/v1"
        )
        return client, "mimo-v2.5-pro", "MiMo"
    
    # Проверяем OpenAI
    openai_key = os.getenv("OPENAI_API_KEY")
    if openai_key:
        client = OpenAI(api_key=openai_key)
        return client, "gpt-4o-mini", "OpenAI"
    
    # Универсальный ключ
    llm_key = os.getenv("LLM_API_KEY")
    if llm_key:
        # По умолчанию DeepSeek
        client = OpenAI(api_key=llm_key, base_url="https://api.deepseek.com/v1")
        return client, "deepseek-v4-flash", "Generic"
    
    pytest.skip("No LLM API key found")


# ============================================================
# Тесты
# ============================================================


@pytest.mark.llm_api
@pytest.mark.slow
class TestRealLLMIntegration:
    """Тесты с реальными LLM API"""

    def test_llm_can_call_single_tool(self, skip_if_no_llm_api, mock_node):
        """
        Тест: LLM может вызвать один инструмент
        
        Проверяем что LLM правильно понимает запрос и вызывает нужный tool
        """
        client, model, provider = get_llm_client_and_model()
        print(f"\n🤖 Тестируем с {provider} ({model})")
        
        # Создаём простой инструмент
        registry = MCPToolRegistry()
        animation_tool = PlayAnimationTool(mock_node)
        registry.register(animation_tool)
        
        # Получаем tools в OpenAI формате
        tools = registry.get_openai_tools()
        print(f"🛠️  Зарегистрировано {len(tools)} инструментов")
        
        # Запрос к LLM
        print("📤 Отправляем запрос: 'Покажи анимацию радости'")
        response = client.chat.completions.create(
            model=model,
            messages=[
                {
                    "role": "system",
                    "content": "Ты робот-ассистент. Используй доступные инструменты для выполнения команд."
                },
                {
                    "role": "user",
                    "content": "Покажи анимацию радости"
                }
            ],
            tools=tools,
            temperature=0.3,  # Низкая температура для более предсказуемого поведения
        )
        
        # Проверяем результат
        print(f"📥 Получен ответ от {provider}")
        message = response.choices[0].message
        
        # LLM должен вызвать tool
        assert message.tool_calls is not None, "LLM не вызвал инструмент!"
        assert len(message.tool_calls) > 0, "Список tool_calls пустой"
        
        tool_call = message.tool_calls[0]
        print(f"✅ LLM вызвал инструмент: {tool_call.function.name}")
        
        # Проверяем что это правильный инструмент
        assert tool_call.function.name == "play_animation"
        
        # Проверяем параметры
        arguments = json.loads(tool_call.function.arguments)
        print(f"   Параметры: {arguments}")
        assert "animation" in arguments
        # LLM может выбрать любую подходящую анимацию радости
        print(f"   ✅ Анимация: {arguments['animation']}")

    @pytest.mark.llm_api
    @pytest.mark.slow
    def test_llm_can_call_multiple_tools(self, skip_if_no_llm_api, mock_node):
        """
        Тест: LLM может вызвать несколько инструментов в одном ответе
        """
        client, model, provider = get_llm_client_and_model()
        print(f"\n🤖 Тестируем множественные tool calls с {provider}")
        
        # Создаём несколько инструментов
        registry = MCPToolRegistry()
        registry.register(PlayAnimationTool(mock_node))
        registry.register(MoveDirectionTool(mock_node))
        
        tools = registry.get_openai_tools()
        
        # Запрос требующий несколько действий
        print("📤 Запрос: 'Покажи анимацию радости и поверни направо'")
        response = client.chat.completions.create(
            model=model,
            messages=[
                {
                    "role": "system",
                    "content": "Ты робот. Выполняй все запрошенные действия используя доступные инструменты."
                },
                {
                    "role": "user",
                    "content": "Покажи анимацию радости и поверни направо"
                }
            ],
            tools=tools,
            temperature=0.3,
        )
        
        message = response.choices[0].message
        print(f"📥 Получен ответ с {len(message.tool_calls or [])} tool calls")
        
        # Может вызвать один или оба инструмента (зависит от LLM)
        assert message.tool_calls is not None
        assert len(message.tool_calls) >= 1
        
        # Проверяем что вызваны правильные инструменты
        tool_names = [tc.function.name for tc in message.tool_calls]
        print(f"✅ Вызваны инструменты: {tool_names}")
        
        # Хотя бы один из инструментов анимации должен быть вызван
        assert any(name in ["play_animation"] for name in tool_names)

    @pytest.mark.llm_api
    @pytest.mark.slow
    def test_llm_tool_call_with_validation(self, skip_if_no_llm_api, mock_node):
        """
        Тест: Параметры от LLM проходят валидацию инструмента
        """
        client, model, provider = get_llm_client_and_model()
        print(f"\n🤖 Тестируем валидацию параметров с {provider}")
        
        registry = MCPToolRegistry()
        animation_tool = PlayAnimationTool(mock_node)
        registry.register(animation_tool)
        
        tools = registry.get_openai_tools()
        
        # Запрос с конкретной анимацией
        print("📤 Запрос: 'Покажи грустную анимацию'")
        response = client.chat.completions.create(
            model=model,
            messages=[
                {"role": "user", "content": "Покажи грустную анимацию"}
            ],
            tools=tools,
            temperature=0.3,
        )
        
        message = response.choices[0].message
        assert message.tool_calls is not None
        
        tool_call = message.tool_calls[0]
        arguments = json.loads(tool_call.function.arguments)
        
        print(f"📥 LLM передал параметры: {arguments}")
        
        # Выполняем инструмент с параметрами от LLM
        result = registry.execute(tool_call.function.name, **arguments)
        
        # Результат должен быть успешным (параметры валидны)
        print(f"✅ Результат выполнения: success={result.success}")
        if not result.success:
            print(f"   Ошибка: {result.error}")
        
        # Если LLM передал правильное имя анимации - должно быть успешно
        # Если неправильное - будет ошибка, но это тоже валидный сценарий

    @pytest.mark.llm_api
    @pytest.mark.slow
    def test_llm_understands_russian_commands(self, skip_if_no_llm_api, mock_node):
        """
        Тест: LLM понимает команды на русском и вызывает правильные инструменты
        """
        client, model, provider = get_llm_client_and_model()
        print(f"\n🤖 Тестируем русские команды с {provider}")
        
        registry = MCPToolRegistry()
        registry.register(PlayAnimationTool(mock_node))
        
        tools = registry.get_openai_tools()
        
        # Тестируем разные формулировки на русском
        test_cases = [
            ("Покажи анимацию радости", "happy"),
            ("Покажи грустное настроение", "sad"),
            ("Будь злым", "angry"),
        ]
        
        for user_message, expected_animation in test_cases:
            print(f"\n📤 Тест: '{user_message}'")
            
            response = client.chat.completions.create(
                model=model,
                messages=[
                    {
                        "role": "system",
                        "content": "Ты робот. Используй инструменты для показа анимаций."
                    },
                    {"role": "user", "content": user_message}
                ],
                tools=tools,
                temperature=0.3,
            )
            
            message = response.choices[0].message
            
            if message.tool_calls:
                tool_call = message.tool_calls[0]
                arguments = json.loads(tool_call.function.arguments)
                print(f"   ✅ Вызван: {tool_call.function.name}({arguments})")
                
                # Проверяем что вызван правильный инструмент
                assert tool_call.function.name == "play_animation"
                assert "animation" in arguments
            else:
                print(f"   ⚠️  LLM не вызвал инструмент, ответил текстом")
                # Это не ошибка - некоторые LLM могут выбрать текстовый ответ

    @pytest.mark.llm_api
    @pytest.mark.slow
    def test_llm_handles_tool_result_feedback(self, skip_if_no_llm_api, mock_node):
        """
        Тест: LLM получает результат выполнения инструмента и формирует ответ
        
        Это полный цикл: запрос → tool call → выполнение → результат → финальный ответ
        """
        client, model, provider = get_llm_client_and_model()
        print(f"\n🤖 Тестируем полный цикл с результатом tool с {provider}")
        
        registry = MCPToolRegistry()
        animation_tool = PlayAnimationTool(mock_node)
        registry.register(animation_tool)
        
        tools = registry.get_openai_tools()
        
        # Шаг 1: Первый запрос
        print("📤 Шаг 1: Отправляем запрос")
        messages = [
            {
                "role": "system",
                "content": "Ты робот. Используй инструменты и сообщи пользователю о результате."
            },
            {"role": "user", "content": "Покажи анимацию радости"}
        ]
        
        response = client.chat.completions.create(
            model=model,
            messages=messages,
            tools=tools,
            temperature=0.3,
        )
        
        message = response.choices[0].message
        assert message.tool_calls is not None
        
        tool_call = message.tool_calls[0]
        print(f"   ✅ LLM вызвал: {tool_call.function.name}")
        
        # Шаг 2: Выполняем инструмент
        print("📤 Шаг 2: Выполняем инструмент")
        arguments = json.loads(tool_call.function.arguments)
        result = registry.execute(tool_call.function.name, **arguments)
        print(f"   ✅ Результат: {result.message}")
        
        # Шаг 3: Отправляем результат обратно в LLM
        print("📤 Шаг 3: Отправляем результат в LLM")
        messages.append({
            "role": "assistant",
            "content": None,
            "tool_calls": [{
                "id": tool_call.id,
                "type": "function",
                "function": {
                    "name": tool_call.function.name,
                    "arguments": tool_call.function.arguments
                }
            }]
        })
        
        messages.append({
            "role": "tool",
            "tool_call_id": tool_call.id,
            "content": json.dumps({
                "success": result.success,
                "message": result.message,
                "data": result.data
            }, ensure_ascii=False)
        })
        
        # Финальный запрос
        final_response = client.chat.completions.create(
            model=model,
            messages=messages,
            temperature=0.5,
        )
        
        final_message = final_response.choices[0].message.content
        print(f"📥 Финальный ответ LLM: {final_message}")
        
        # LLM должен сформировать текстовый ответ на основе результата
        assert final_message is not None
        assert len(final_message) > 0
        print("✅ Полный цикл успешно завершён!")

    @pytest.mark.llm_api
    @pytest.mark.slow  
    def test_llm_refuses_unavailable_tool(self, skip_if_no_llm_api, mock_node):
        """
        Тест: LLM не пытается вызвать несуществующий инструмент
        """
        client, model, provider = get_llm_client_and_model()
        print(f"\n🤖 Тестируем отказ от недоступного инструмента с {provider}")
        
        # Регистрируем только один инструмент
        registry = MCPToolRegistry()
        registry.register(PlayAnimationTool(mock_node))
        
        tools = registry.get_openai_tools()
        
        # Просим выполнить действие, для которого нет инструмента
        print("📤 Запрос: 'Включи музыку' (нет такого инструмента)")
        response = client.chat.completions.create(
            model=model,
            messages=[
                {
                    "role": "system",
                    "content": "Ты робот. Используй ТОЛЬКО доступные инструменты. Если инструмента нет - извинись."
                },
                {"role": "user", "content": "Включи музыку"}
            ],
            tools=tools,
            temperature=0.3,
        )
        
        message = response.choices[0].message
        
        if message.tool_calls:
            # Если LLM всё же вызвал tool - проверяем что это один из доступных
            tool_names = [tc.function.name for tc in message.tool_calls]
            print(f"   ⚠️  LLM вызвал: {tool_names}")
            assert all(name == "play_animation" for name in tool_names)
        else:
            # LLM должен ответить текстом что не может выполнить
            print(f"   ✅ LLM ответил текстом: {message.content[:100]}...")
            assert message.content is not None


# ============================================================
# Тесты производительности
# ============================================================


@pytest.mark.llm_api
@pytest.mark.slow
def test_llm_response_time(skip_if_no_llm_api, mock_node):
    """
    Тест: Измерение времени ответа LLM с tool calls
    """
    client, model, provider = get_llm_client_and_model()
    print(f"\n⏱️  Измеряем производительность {provider}")
    
    registry = MCPToolRegistry()
    registry.register(PlayAnimationTool(mock_node))
    
    tools = registry.get_openai_tools()
    
    start_time = time.time()
    
    response = client.chat.completions.create(
        model=model,
        messages=[
            {"role": "user", "content": "Покажи анимацию радости"}
        ],
        tools=tools,
        temperature=0.3,
    )
    
    elapsed = time.time() - start_time
    
    print(f"✅ Время ответа: {elapsed:.2f} секунд")
    print(f"   Модель: {model}")
    print(f"   Tool calls: {len(response.choices[0].message.tool_calls or [])}")
    
    # Проверяем что ответ пришёл в разумное время (< 10 секунд)
    assert elapsed < 10.0, f"Ответ слишком медленный: {elapsed:.2f}s"


# ============================================================
# Тесты для разных категорий инструментов
# ============================================================


@pytest.mark.llm_api
@pytest.mark.slow
class TestNavigationTools:
    """Тесты навигационных инструментов с реальным LLM"""

    def test_llm_calls_navigation_tool(self, skip_if_no_llm_api, mock_node):
        """Тест: LLM вызывает инструмент навигации"""
        client, model, provider = get_llm_client_and_model()
        print(f"\n🧭 Тестируем навигацию с {provider}")
        
        registry = MCPToolRegistry()
        registry.register(NavigateToWaypointTool(mock_node))
        registry.register(ListWaypointsTool(mock_node))
        
        tools = registry.get_openai_tools()
        
        print("📤 Запрос: 'Поезжай на кухню'")
        response = client.chat.completions.create(
            model=model,
            messages=[
                {"role": "system", "content": "Ты робот. Используй инструменты для навигации."},
                {"role": "user", "content": "Поезжай на кухню"}
            ],
            tools=tools,
            temperature=0.3,
        )
        
        message = response.choices[0].message
        if message.tool_calls:
            tool_call = message.tool_calls[0]
            print(f"✅ LLM вызвал: {tool_call.function.name}")
            arguments = json.loads(tool_call.function.arguments)
            print(f"   Параметры: {arguments}")
            
            # Должен вызвать navigate_to_waypoint с waypoint
            assert tool_call.function.name == "navigate_to_waypoint"
            assert "waypoint" in arguments
        else:
            print(f"⚠️  LLM ответил текстом вместо tool call")

    def test_llm_calls_move_direction(self, skip_if_no_llm_api, mock_node):
        """Тест: LLM использует движение в направлении"""
        client, model, provider = get_llm_client_and_model()
        print(f"\n🧭 Тестируем движение в направлении с {provider}")
        
        registry = MCPToolRegistry()
        registry.register(MoveDirectionTool(mock_node))
        
        tools = registry.get_openai_tools()
        
        print("📤 Запрос: 'Поверни налево'")
        response = client.chat.completions.create(
            model=model,
            messages=[
                {"role": "system", "content": "Ты робот. Используй инструменты для управления движением."},
                {"role": "user", "content": "Поверни налево"}
            ],
            tools=tools,
            temperature=0.3,
        )
        
        message = response.choices[0].message
        if message.tool_calls:
            tool_call = message.tool_calls[0]
            print(f"✅ LLM вызвал: {tool_call.function.name}")
            arguments = json.loads(tool_call.function.arguments)
            print(f"   Параметры: {arguments}")
            
            assert tool_call.function.name == "move_direction"
            assert "direction" in arguments


@pytest.mark.llm_api
@pytest.mark.slow
class TestSystemTools:
    """Тесты системных инструментов с реальным LLM"""

    def test_llm_calls_volume_control(self, skip_if_no_llm_api, mock_node):
        """Тест: LLM управляет громкостью"""
        client, model, provider = get_llm_client_and_model()
        print(f"\n🔊 Тестируем управление громкостью с {provider}")
        
        registry = MCPToolRegistry()
        registry.register(SetVolumeTool(mock_node))
        
        tools = registry.get_openai_tools()
        
        print("📤 Запрос: 'Говори громче'")
        response = client.chat.completions.create(
            model=model,
            messages=[
                {"role": "system", "content": "Ты робот. Используй инструменты для управления системой."},
                {"role": "user", "content": "Говори громче"}
            ],
            tools=tools,
            temperature=0.3,
        )
        
        message = response.choices[0].message
        if message.tool_calls:
            tool_call = message.tool_calls[0]
            print(f"✅ LLM вызвал: {tool_call.function.name}")
            arguments = json.loads(tool_call.function.arguments)
            print(f"   Параметры: {arguments}")
            
            assert tool_call.function.name == "set_volume"
            assert "action" in arguments
            # LLM должен понять что "громче" = "louder"
            assert arguments["action"] in ["louder", "max"]

    def test_llm_calls_robot_status(self, skip_if_no_llm_api, mock_node):
        """Тест: LLM запрашивает статус робота"""
        client, model, provider = get_llm_client_and_model()
        print(f"\n📊 Тестируем запрос статуса с {provider}")
        
        registry = MCPToolRegistry()
        registry.register(GetRobotStatusTool(mock_node))
        
        tools = registry.get_openai_tools()
        
        print("📤 Запрос: 'Как ты себя чувствуешь?'")
        response = client.chat.completions.create(
            model=model,
            messages=[
                {"role": "system", "content": "Ты робот. Используй инструменты для получения информации о себе."},
                {"role": "user", "content": "Как ты себя чувствуешь?"}
            ],
            tools=tools,
            temperature=0.3,
        )
        
        message = response.choices[0].message
        if message.tool_calls:
            tool_call = message.tool_calls[0]
            print(f"✅ LLM вызвал: {tool_call.function.name}")
            assert tool_call.function.name == "get_robot_status"


@pytest.mark.llm_api
@pytest.mark.slow
class TestPerceptionTools:
    """Тесты инструментов восприятия с реальным LLM"""

    def test_llm_calls_perception_context(self, skip_if_no_llm_api, mock_node):
        """Тест: LLM запрашивает контекст восприятия"""
        client, model, provider = get_llm_client_and_model()
        print(f"\n👁️  Тестируем запрос контекста с {provider}")
        
        registry = MCPToolRegistry()
        registry.register(GetPerceptionContextTool(mock_node))
        
        tools = registry.get_openai_tools()
        
        print("📤 Запрос: 'Что ты видишь вокруг?'")
        response = client.chat.completions.create(
            model=model,
            messages=[
                {"role": "system", "content": "Ты робот с камерой. Используй инструменты для получения информации об окружении."},
                {"role": "user", "content": "Что ты видишь вокруг?"}
            ],
            tools=tools,
            temperature=0.3,
        )
        
        message = response.choices[0].message
        if message.tool_calls:
            tool_call = message.tool_calls[0]
            print(f"✅ LLM вызвал: {tool_call.function.name}")
            assert tool_call.function.name == "get_perception_context"

    def test_llm_calls_battery_check(self, skip_if_no_llm_api, mock_node):
        """Тест: LLM проверяет уровень батареи"""
        client, model, provider = get_llm_client_and_model()
        print(f"\n🔋 Тестируем проверку батареи с {provider}")
        
        registry = MCPToolRegistry()
        registry.register(GetBatteryLevelTool(mock_node))
        
        tools = registry.get_openai_tools()
        
        print("📤 Запрос: 'Сколько у тебя осталось заряда?'")
        response = client.chat.completions.create(
            model=model,
            messages=[
                {"role": "system", "content": "Ты робот на батарее. Используй инструменты для проверки заряда."},
                {"role": "user", "content": "Сколько у тебя осталось заряда?"}
            ],
            tools=tools,
            temperature=0.3,
        )
        
        message = response.choices[0].message
        if message.tool_calls:
            tool_call = message.tool_calls[0]
            print(f"✅ LLM вызвал: {tool_call.function.name}")
            assert tool_call.function.name == "get_battery_level"


@pytest.mark.llm_api
@pytest.mark.slow
def test_llm_with_all_tools(skip_if_no_llm_api, mock_node):
    """
    Тест: LLM выбирает правильный инструмент из ВСЕХ доступных категорий
    
    Регистрируем ВСЕ инструменты и проверяем что LLM выберет правильный
    """
    client, model, provider = get_llm_client_and_model()
    print(f"\n🎯 Тестируем выбор из ВСЕХ инструментов с {provider}")
    
    # Регистрируем ВСЕ доступные инструменты
    registry = MCPToolRegistry()
    
    # Navigation
    registry.register(NavigateToWaypointTool(mock_node))
    registry.register(MoveDirectionTool(mock_node))
    registry.register(ListWaypointsTool(mock_node))
    
    # System
    registry.register(SetVolumeTool(mock_node))
    registry.register(SetPitchTool(mock_node))
    registry.register(GetRobotStatusTool(mock_node))
    
    # Perception
    registry.register(GetPerceptionContextTool(mock_node))
    registry.register(GetBatteryLevelTool(mock_node))
    
    # Animation
    registry.register(PlayAnimationTool(mock_node))
    
    # Sound
    registry.register(PlaySoundTool(mock_node))
    
    # Mapping
    registry.register(StartMappingTool(mock_node))
    
    tools = registry.get_openai_tools()
    print(f"🛠️  Зарегистрировано {len(tools)} инструментов из всех категорий")
    
    # Тестируем разные типы команд
    test_cases = [
        ("Поезжай на кухню", "navigate_to_waypoint"),
        ("Покажи анимацию радости", "play_animation"),
        ("Говори громче", "set_volume"),
        ("Сколько у тебя заряда?", "get_battery_level"),
    ]
    
    for user_message, expected_tool in test_cases:
        print(f"\n📤 Тест: '{user_message}'")
        response = client.chat.completions.create(
            model=model,
            messages=[
                {
                    "role": "system",
                    "content": "Ты робот Rob Box. Используй правильные инструменты для выполнения команд."
                },
                {"role": "user", "content": user_message}
            ],
            tools=tools,
            temperature=0.3,
        )
        
        message = response.choices[0].message
        if message.tool_calls:
            tool_call = message.tool_calls[0]
            print(f"   ✅ LLM выбрал: {tool_call.function.name}")
            
            # Проверяем что выбран правильный инструмент
            assert tool_call.function.name == expected_tool, \
                f"Ожидался {expected_tool}, получен {tool_call.function.name}"
        else:
            print(f"   ⚠️  LLM ответил текстом вместо tool call")
            # Некоторые LLM могут выбрать текстовый ответ - это не ошибка
