#!/usr/bin/env python3
"""
navigation.py - Инструменты навигации и движения робота

Инструменты:
- NavigateToWaypointTool: Навигация к именованной точке
- MoveDirectionTool: Движение в направлении (вперёд/назад/влево/вправо)
- StopNavigationTool: Остановка движения
- ListWaypointsTool: Получить список доступных точек
"""

from typing import Dict, Any, List
import math

from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo

from ..base import MCPTool, MCPToolParameter, MCPToolResult


class NavigateToWaypointTool(MCPTool):
    """Инструмент для навигации к именованной точке"""

    # Словарь известных точек (можно загружать из конфига)
    WAYPOINTS = {
        "дом": {"x": 0.0, "y": 0.0, "theta": 0.0},
        "кухня": {"x": 2.0, "y": 1.0, "theta": 0.0},
        "гостиная": {"x": 3.0, "y": 2.0, "theta": 1.57},
        "точка 1": {"x": 1.0, "y": 0.0, "theta": 0.0},
        "точка 2": {"x": 2.0, "y": 0.0, "theta": 0.0},
        "точка 3": {"x": 3.0, "y": 0.0, "theta": 0.0},
    }

    def __init__(self, node):
        super().__init__(node)
        # Action client для Nav2
        self.nav_client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self.current_goal_handle = None

    @property
    def name(self) -> str:
        return "navigate_to_waypoint"

    @property
    def description(self) -> str:
        return "Навигация робота к именованной точке (waypoint). Используй для команд типа 'иди к кухне', 'поезжай домой'."

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="waypoint",
                type="string",
                description="Название точки назначения",
                required=True,
                enum=list(self.WAYPOINTS.keys()),
            )
        ]

    def execute(self, waypoint: str) -> MCPToolResult:
        """Выполнить навигацию к точке"""
        self.log_info(f"Навигация к точке: {waypoint}")

        # Проверка существования точки
        if waypoint not in self.WAYPOINTS:
            available = ", ".join(self.WAYPOINTS.keys())
            return MCPToolResult(
                success=False,
                error=f"Неизвестная точка '{waypoint}'",
                message=f"Доступные точки: {available}",
            )

        coords = self.WAYPOINTS[waypoint]

        # Проверка доступности Nav2
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            return MCPToolResult(success=False, error="Nav2 action server недоступен", message="Навигация недоступна")

        # Создание цели
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()

        goal.pose.pose.position.x = coords["x"]
        goal.pose.pose.position.y = coords["y"]
        goal.pose.pose.position.z = 0.0

        # Ориентация из угла theta
        goal.pose.pose.orientation.z = math.sin(coords["theta"] / 2.0)
        goal.pose.pose.orientation.w = math.cos(coords["theta"] / 2.0)

        # Отправка цели (асинхронно)
        future = self.nav_client.send_goal_async(goal)
        # NOTE: В реальном использовании нужно обработать future через callback
        # Для MCP инструмента возвращаем немедленный результат

        self.log_info(f"Цель отправлена: {waypoint} ({coords['x']}, {coords['y']})")

        return MCPToolResult(
            success=True,
            data={"waypoint": waypoint, "coordinates": coords},
            message=f"Иду к точке {waypoint}",
        )


class MoveDirectionTool(MCPTool):
    """Инструмент для движения в направлении"""

    DIRECTIONS = {
        "вперёд": {"x": 1.0, "y": 0.0, "theta": 0.0},
        "назад": {"x": -1.0, "y": 0.0, "theta": 0.0},
        "налево": {"x": 0.0, "y": 0.0, "theta": math.pi / 2},
        "направо": {"x": 0.0, "y": 0.0, "theta": -math.pi / 2},
    }

    def __init__(self, node):
        super().__init__(node)
        self.nav_client = ActionClient(node, NavigateToPose, "navigate_to_pose")

    @property
    def name(self) -> str:
        return "move_direction"

    @property
    def description(self) -> str:
        return "Движение робота в указанном направлении. Используй для команд 'вперёд', 'назад', 'поверни налево'."

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="direction",
                type="string",
                description="Направление движения",
                required=True,
                enum=list(self.DIRECTIONS.keys()),
            ),
            MCPToolParameter(
                name="distance",
                type="number",
                description="Расстояние в метрах (по умолчанию 1.0). Только для движения вперёд/назад.",
                required=False,
                default=1.0,
            ),
        ]

    def execute(self, direction: str, distance: float = 1.0) -> MCPToolResult:
        """Выполнить движение в направлении"""
        self.log_info(f"Движение: {direction}, дистанция: {distance}м")

        if direction not in self.DIRECTIONS:
            return MCPToolResult(success=False, error=f"Неизвестное направление: {direction}")

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            return MCPToolResult(success=False, error="Nav2 недоступен")

        # Получаем базовые координаты направления
        coords = self.DIRECTIONS[direction].copy()

        # Для движения вперёд/назад применяем distance
        if direction in ["вперёд", "назад"]:
            coords["x"] *= distance

        # Создание относительной цели (в base_link frame)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "base_link"  # Относительные координаты
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()

        goal.pose.pose.position.x = coords["x"]
        goal.pose.pose.position.y = coords["y"]
        goal.pose.pose.position.z = 0.0

        goal.pose.pose.orientation.z = math.sin(coords["theta"] / 2.0)
        goal.pose.pose.orientation.w = math.cos(coords["theta"] / 2.0)

        # Отправка цели
        future = self.nav_client.send_goal_async(goal)

        self.log_info(f"Относительная цель отправлена: {coords}")

        return MCPToolResult(
            success=True,
            data={"direction": direction, "distance": distance, "relative_coords": coords},
            message=f"Движение {direction}",
        )


class StopNavigationTool(MCPTool):
    """Инструмент для остановки навигации"""

    def __init__(self, node):
        super().__init__(node)
        self.cancel_client = node.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")

    @property
    def name(self) -> str:
        return "stop_navigation"

    @property
    def description(self) -> str:
        return "Остановить текущую навигацию робота. Используй для команд 'стоп', 'остановись', 'отмени движение'."

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return []  # Нет параметров

    def execute(self) -> MCPToolResult:
        """Остановить навигацию"""
        self.log_info("Остановка навигации")

        if not self.cancel_client.wait_for_service(timeout_sec=0.5):
            return MCPToolResult(success=False, error="Cancel service недоступен")

        # Отменить все цели (пустой GoalInfo)
        request = CancelGoal.Request()
        request.goal_info = GoalInfo()

        future = self.cancel_client.call_async(request)
        # NOTE: В реальном использовании нужно дождаться результата

        self.log_info("Команда отмены отправлена")

        return MCPToolResult(success=True, message="Останавливаюсь")


class ListWaypointsTool(MCPTool):
    """Инструмент для получения списка доступных точек"""

    @property
    def name(self) -> str:
        return "list_waypoints"

    @property
    def description(self) -> str:
        return "Получить список всех доступных точек для навигации."

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return []

    def execute(self) -> MCPToolResult:
        """Получить список точек"""
        waypoints = NavigateToWaypointTool.WAYPOINTS
        waypoint_list = [
            {"name": name, "x": coords["x"], "y": coords["y"], "theta": coords["theta"]}
            for name, coords in waypoints.items()
        ]

        return MCPToolResult(success=True, data={"waypoints": waypoint_list}, message=f"Доступно {len(waypoints)} точек")
