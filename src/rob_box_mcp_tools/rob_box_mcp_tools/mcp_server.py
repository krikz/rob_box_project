#!/usr/bin/env python3
"""
mcp_server.py - MCP Server для предоставления инструментов LLM

Эта нода:
1. Регистрирует все доступные MCP инструменты
2. Публикует список инструментов в OpenAI Tool Calls формате (совместимо с DeepSeek, Qwen, и др.)
3. Принимает запросы на выполнение инструментов
4. Возвращает результаты выполнения

ROS 2 интерфейс:
- Публикует: /mcp/tools (String) - JSON список доступных инструментов
- Подписывается на: /mcp/execute (String) - JSON запросы на выполнение
- Публикует: /mcp/result (String) - JSON результаты выполнения
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import json
from typing import Dict, Any

from .registry import MCPToolRegistry
from .tools import (
    NavigateToWaypointTool,
    MoveDirectionTool,
    StopNavigationTool,
    ListWaypointsTool,
    SetVolumeTool,
    SetPitchTool,
    SetSpeedTool,
    GetRobotStatusTool,
    GetPerceptionContextTool,
    GetBatteryLevelTool,
    StartMappingTool,
    ContinueMappingTool,
    FinishMappingTool,
    PlayAnimationTool,
    PlaySoundTool,
    SpeakTextTool,
    ListenForResponseTool,
)


class MCPServer(Node):
    """
    MCP Server - центральная нода для управления инструментами

    Предоставляет инструменты для LLM и обрабатывает запросы на их выполнение.
    """

    def __init__(self):
        super().__init__("mcp_server")

        # Реестр инструментов
        self.registry = MCPToolRegistry()

        # Регистрация инструментов
        self._register_tools()

        # QoS RELIABLE для гарантированной доставки результатов через Zenoh
        # BEST_EFFORT терял сообщения в сетевом окружении!
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publisher для списка инструментов
        self.tools_pub = self.create_publisher(String, "/mcp/tools", qos_profile)

        # Publisher для результатов
        self.result_pub = self.create_publisher(String, "/mcp/result", qos_profile)

        # Subscriber для запросов на выполнение
        self.execute_sub = self.create_subscription(String, "/mcp/execute", self.on_execute_request, qos_profile)

        # Подписка на perception context для обновления инструментов
        try:
            from rob_box_perception_msgs.msg import PerceptionEvent

            self.perception_sub = self.create_subscription(PerceptionEvent, "/perception/context_update", self.on_perception_update, 10)
            self.get_logger().info("✅ Подписан на /perception/context_update")
        except ImportError:
            self.get_logger().warning("⚠️ PerceptionEvent не найден, мониторинг контекста отключен")

        # Таймер для периодической публикации списка инструментов
        self.tools_timer = self.create_timer(10.0, self.publish_tools)

        # Публикуем список инструментов сразу при старте
        self.publish_tools()

        self.get_logger().info(f"🛠️ MCP Server запущен с {len(self.registry)} инструментами")
        self.get_logger().info(f"   Инструменты: {', '.join(self.registry.list_tools())}")

    def _register_tools(self):
        """Регистрация всех доступных инструментов"""
        # Navigation tools
        self.registry.register(NavigateToWaypointTool(self))
        self.registry.register(MoveDirectionTool(self))
        self.registry.register(StopNavigationTool(self))
        self.registry.register(ListWaypointsTool(self))

        # System tools
        self.registry.register(SetVolumeTool(self))
        self.registry.register(SetPitchTool(self))
        self.registry.register(SetSpeedTool(self))
        self.registry.register(GetRobotStatusTool(self))

        # Perception tools
        self.perception_context_tool = GetPerceptionContextTool(self)
        self.battery_tool = GetBatteryLevelTool(self)
        self.registry.register(self.perception_context_tool)
        self.registry.register(self.battery_tool)

        # Mapping tools
        self.registry.register(StartMappingTool(self))
        self.registry.register(ContinueMappingTool(self))
        self.registry.register(FinishMappingTool(self))

        # Animation tools
        self.registry.register(PlayAnimationTool(self))

        # Sound tools
        self.registry.register(PlaySoundTool(self))

        # Dialogue tools (критично для агентного диалога!)
        self.registry.register(SpeakTextTool(self))
        self.registry.register(ListenForResponseTool(self))

    def publish_tools(self):
        """Публикация списка доступных инструментов в OpenAI Tool Calls формате"""
        tools = self.registry.get_openai_tools()
        msg = String()
        msg.data = json.dumps(tools, ensure_ascii=False, indent=2)
        self.tools_pub.publish(msg)
        self.get_logger().debug(f"📤 Опубликован список {len(tools)} инструментов")

    def on_execute_request(self, msg: String):
        """
        Обработка запроса на выполнение инструмента

        Формат запроса (JSON):
        {
            "tool_name": "navigate_to_waypoint",
            "parameters": {
                "waypoint": "кухня"
            },
            "request_id": "optional_unique_id"
        }
        """
        try:
            request = json.loads(msg.data)
            tool_name = request.get("tool_name")
            parameters = request.get("parameters", {})
            request_id = request.get("request_id", "")

            self.get_logger().info(f"📥 Запрос выполнения: {tool_name} с параметрами {parameters}")

            if not tool_name:
                self._publish_error("Не указано имя инструмента", request_id)
                return

            # Выполнить инструмент
            result = self.registry.execute(tool_name, **parameters)

            # Опубликовать результат
            response = {"tool_name": tool_name, "request_id": request_id, "result": result.to_dict()}

            msg_out = String()
            msg_out.data = json.dumps(response, ensure_ascii=False)
            
            # Логируем ПЕРЕД публикацией
            self.get_logger().info(f"📤 Публикую результат для {tool_name} (request_id: {request_id[:8]})")
            self.result_pub.publish(msg_out)
            self.get_logger().info(f"✅ Результат опубликован на /mcp/result")

            if result.success:
                self.get_logger().info(f"✅ Инструмент {tool_name} выполнен успешно")
            else:
                self.get_logger().warning(f"❌ Инструмент {tool_name} завершился с ошибкой: {result.error}")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ Ошибка парсинга JSON запроса: {e}")
            self._publish_error(f"Ошибка парсинга JSON: {str(e)}", "")
        except Exception as e:
            self.get_logger().error(f"❌ Ошибка выполнения запроса: {e}")
            self._publish_error(f"Внутренняя ошибка: {str(e)}", "")

    def on_perception_update(self, msg):
        """Обработка обновления контекста восприятия"""
        try:
            # Обновляем battery tool
            if hasattr(msg, "battery_percentage"):
                self.battery_tool.update_battery(msg.battery_percentage)

            # Обновляем perception context tool
            # Конвертируем PerceptionEvent в dict для хранения
            context = {
                "timestamp": msg.timestamp if hasattr(msg, "timestamp") else 0.0,
                "internet_available": msg.internet_available if hasattr(msg, "internet_available") else False,
                "battery_percentage": msg.battery_percentage if hasattr(msg, "battery_percentage") else 0.0,
                # Добавьте другие поля по необходимости
            }
            self.perception_context_tool.update_context(context)

        except Exception as e:
            self.get_logger().error(f"❌ Ошибка обновления контекста: {e}")

    def _publish_error(self, error_message: str, request_id: str = ""):
        """Опубликовать сообщение об ошибке"""
        response = {"tool_name": None, "request_id": request_id, "result": {"success": False, "error": error_message}}

        msg = String()
        msg.data = json.dumps(response, ensure_ascii=False)
        self.result_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MCPServer()

    # Используем MultiThreadedExecutor для параллельной обработки callbacks
    # Это позволяет получать /voice/tts/finished пока execute() блокируется
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
