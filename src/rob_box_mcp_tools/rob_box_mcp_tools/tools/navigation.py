#!/usr/bin/env python3
"""
navigation.py - Инструменты навигации и движения робота

Инструменты:
- NavigateToWaypointTool: Навигация к именованной точке (из БД)
- NavigateToCoordinatesTool: Навигация к произвольным координатам
- MoveDirectionTool: Движение в направлении (вперёд/назад/влево/вправо)
- StopNavigationTool: Остановка движения
- ListWaypointsTool: Получить список доступных точек (из БД)
- SaveWaypointTool: Сохранить текущую позицию как именованную точку
- DeleteWaypointTool: Удалить именованную точку
- ClearWaypointsTool: Удалить все точки текущей карты
- GetCurrentPoseTool: Получить текущую позицию робота
"""

from typing import Any, Dict, List, Optional, TYPE_CHECKING
import math
import threading
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo

if TYPE_CHECKING:
    import tf2_ros
    from ..waypoint_store import WaypointStore

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType


def _wait_future(future, timeout_sec: float) -> bool:
    """Wait for an rclpy Future without touching the executor.

    ``rclpy.spin_until_future_complete()`` is UNSAFE to call from within a
    callback that is already executing under ``MultiThreadedExecutor`` — it
    internally tries to add the node to a *new* executor, which corrupts the
    existing one and silently breaks all subsequent subscription callbacks.

    This helper attaches a ``done_callback`` to the future so that a plain
    ``threading.Event`` is set when the future completes.  The calling thread
    blocks on the event, leaving the ROS 2 executor completely undisturbed.

    Returns True if the future completed within *timeout_sec*, False otherwise.
    """
    event = threading.Event()
    future.add_done_callback(lambda _: event.set())
    return event.wait(timeout=timeout_sec)


def _send_nav_goal(nav_client, node, x: float, y: float, theta: float, frame_id: str = "map", timeout: float = 120.0):
    """Shared helper: send a NavigateToPose goal and block until completion.

    Returns ``MCPToolResult``.
    """
    if not nav_client.wait_for_server(timeout_sec=1.0):
        return MCPToolResult(success=False, error="Nav2 action server недоступен", message="Навигация недоступна")

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = frame_id
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.position.z = 0.0
    goal.pose.pose.orientation.z = math.sin(theta / 2.0)
    goal.pose.pose.orientation.w = math.cos(theta / 2.0)

    send_future = nav_client.send_goal_async(goal)
    if not _wait_future(send_future, timeout_sec=10.0) or send_future.result() is None:
        return MCPToolResult(success=False, error="Nav2 не ответил на цель", message="Навигация недоступна")

    goal_handle = send_future.result()
    if not goal_handle.accepted:
        return MCPToolResult(success=False, error="Nav2 отклонил цель", message="Цель недостижима")

    result_future = goal_handle.get_result_async()
    if not _wait_future(result_future, timeout_sec=timeout) or result_future.result() is None:
        return MCPToolResult(success=False, error="Навигация превысила таймаут", message="Не доехал до точки")

    return MCPToolResult(success=True)


def _lookup_pose(tf_buffer, logger) -> Optional[Dict[str, float]]:
    """Look up ``map → base_link`` transform and return ``{x, y, theta}`` or *None*."""
    try:
        import tf2_ros  # noqa: F811
        from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
    except ImportError:
        logger.error("tf2_ros не установлен")
        return None

    try:
        t = tf_buffer.lookup_transform("map", "base_link", tf2_ros.Time(), timeout=tf2_ros.Duration(seconds=2.0))
        x = t.transform.translation.x
        y = t.transform.translation.y
        # Extract yaw from quaternion
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w
        theta = 2.0 * math.atan2(qz, qw)
        return {"x": x, "y": y, "theta": theta}
    except (LookupException, ConnectivityException, ExtrapolationException) as exc:
        logger.warning(f"TF lookup map→base_link не удался: {exc}")
        return None


# ============================================================
# NavigateToWaypointTool — reads from WaypointStore (DB)
# ============================================================


class NavigateToWaypointTool(MCPTool):
    """Навигация к именованной точке из базы данных вейпоинтов."""

    def __init__(self, node, waypoint_store: "WaypointStore"):
        super().__init__(node)
        self.nav_client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self.waypoint_store = waypoint_store
        self.current_goal_handle = None

    @property
    def name(self) -> str:
        return "navigate_to_waypoint"

    @property
    def description(self) -> str:
        return (
            "Навигация робота к именованной точке (waypoint) из базы. "
            "Используй для команд типа 'иди к кухне', 'поезжай в зал'. "
            "Сначала проверь доступные точки через list_waypoints."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="waypoint",
                type="string",
                description="Название точки назначения (например 'кухня', 'зал')",
                required=True,
            )
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.LONG

    def execute(self, waypoint: str) -> MCPToolResult:
        """Выполнить навигацию к точке из БД"""
        self.log_info(f"Навигация к точке: {waypoint}")

        coords = self.waypoint_store.get_waypoint(waypoint)
        if coords is None:
            available = self.waypoint_store.list_waypoints()
            names = ", ".join(w["name"] for w in available) if available else "нет сохранённых точек"
            return MCPToolResult(
                success=False,
                error=f"Неизвестная точка '{waypoint}'",
                message=f"Доступные точки: {names}",
            )

        self.log_info(f"📍 Координаты: x={coords['x']:.2f}, y={coords['y']:.2f}, θ={coords['theta']:.2f}")

        result = _send_nav_goal(self.nav_client, self.node, coords["x"], coords["y"], coords["theta"], timeout=120.0)
        if result.success:
            result.data = {"waypoint": waypoint, "coordinates": coords}
            result.message = f"Приехал к точке {waypoint}"
            self.log_info(f"✅ Навигация к {waypoint} завершена")
        return result


# ============================================================
# NavigateToCoordinatesTool — navigate to arbitrary (x, y, θ)
# ============================================================


class NavigateToCoordinatesTool(MCPTool):
    """Навигация к произвольным координатам в map frame."""

    def __init__(self, node):
        super().__init__(node)
        self.nav_client = ActionClient(node, NavigateToPose, "navigate_to_pose")

    @property
    def name(self) -> str:
        return "navigate_to_coordinates"

    @property
    def description(self) -> str:
        return (
            "Навигация робота к произвольным координатам (x, y, theta) в системе координат карты. "
            "Используй для возвращения на сохранённую позицию (после get_current_pose)."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(name="x", type="number", description="Координата X в метрах", required=True),
            MCPToolParameter(name="y", type="number", description="Координата Y в метрах", required=True),
            MCPToolParameter(
                name="theta",
                type="number",
                description="Ориентация в радианах (по умолчанию 0.0)",
                required=False,
                default=0.0,
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.LONG

    def execute(self, x: float, y: float, theta: float = 0.0) -> MCPToolResult:
        self.log_info(f"Навигация к координатам: x={x:.2f}, y={y:.2f}, θ={theta:.2f}")

        result = _send_nav_goal(self.nav_client, self.node, x, y, theta, timeout=120.0)
        if result.success:
            result.data = {"x": x, "y": y, "theta": theta}
            result.message = f"Приехал в точку ({x:.1f}, {y:.1f})"
            self.log_info(f"✅ Навигация к ({x:.2f}, {y:.2f}) завершена")
        return result


# ============================================================
# MoveDirectionTool — relative movement
# ============================================================


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

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.LONG

    def execute(self, direction: str, distance: float = 1.0) -> MCPToolResult:
        """Выполнить движение в направлении"""
        self.log_info(f"Движение: {direction}, дистанция: {distance}м")

        if direction not in self.DIRECTIONS:
            return MCPToolResult(success=False, error=f"Неизвестное направление: {direction}")

        coords = self.DIRECTIONS[direction].copy()
        if direction in ["вперёд", "назад"]:
            coords["x"] *= distance

        result = _send_nav_goal(
            self.nav_client, self.node, coords["x"], coords["y"], coords["theta"], frame_id="base_link", timeout=60.0
        )
        if result.success:
            result.data = {"direction": direction, "distance": distance, "relative_coords": coords}
            result.message = f"Приехал: {direction} {distance}м"
            self.log_info(f"✅ Движение {direction} завершено")
        return result


# ============================================================
# StopNavigationTool
# ============================================================


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
        return []

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    def execute(self) -> MCPToolResult:
        self.log_info("Остановка навигации")

        if not self.cancel_client.wait_for_service(timeout_sec=0.5):
            return MCPToolResult(success=False, error="Cancel service недоступен")

        request = CancelGoal.Request()
        request.goal_info = GoalInfo()
        self.cancel_client.call_async(request)

        self.log_info("Команда отмены отправлена")
        return MCPToolResult(success=True, message="Останавливаюсь")


# ============================================================
# ListWaypointsTool — reads from WaypointStore
# ============================================================


class ListWaypointsTool(MCPTool):
    """Список доступных точек из базы данных."""

    def __init__(self, node, waypoint_store: "WaypointStore"):
        super().__init__(node)
        self.waypoint_store = waypoint_store

    @property
    def name(self) -> str:
        return "list_waypoints"

    @property
    def description(self) -> str:
        return "Получить список всех сохранённых точек (waypoints) для навигации на текущей карте."

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return []

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.INSTANT

    def execute(self) -> MCPToolResult:
        waypoints = self.waypoint_store.list_waypoints()
        active_map = self.waypoint_store.get_active_map()
        map_name = (active_map["name"] or "без имени") if active_map else "карта не задана"

        if not waypoints:
            return MCPToolResult(
                success=True,
                data={"waypoints": [], "map_name": map_name},
                message=f"Нет сохранённых точек (карта: {map_name}). Скажи 'запомни это место как [имя]' чтобы добавить.",
            )

        return MCPToolResult(
            success=True,
            data={"waypoints": waypoints, "map_name": map_name},
            message=f"Карта '{map_name}': {', '.join(w['name'] for w in waypoints)} ({len(waypoints)} точек)",
        )


# ============================================================
# SaveWaypointTool — saves current robot pose as a named waypoint
# ============================================================


class SaveWaypointTool(MCPTool):
    """Сохранить текущую позицию робота как именованную точку."""

    def __init__(self, node, waypoint_store: "WaypointStore", tf_buffer: "tf2_ros.Buffer"):
        super().__init__(node)
        self.waypoint_store = waypoint_store
        self.tf_buffer = tf_buffer

    @property
    def name(self) -> str:
        return "save_waypoint"

    @property
    def description(self) -> str:
        return (
            "Сохранить текущую позицию робота как именованную точку. "
            "Используй когда пользователь говорит 'запомни это место как кухня', "
            "'это зал', 'сохрани точку спальня'. "
            "Если точка с таким именем уже есть — координаты обновятся."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="name",
                type="string",
                description="Название точки (например 'кухня', 'зал', 'спальня')",
                required=True,
            )
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    def execute(self, name: str) -> MCPToolResult:
        self.log_info(f"Сохранение точки: {name}")

        pose = _lookup_pose(self.tf_buffer, self.node.get_logger())
        if pose is None:
            return MCPToolResult(
                success=False,
                error="Не могу определить позицию робота (TF map→base_link недоступен)",
                message="Позиция неизвестна. Убедись что локализация работает.",
            )

        self.waypoint_store.save_waypoint(name, pose["x"], pose["y"], pose["theta"])
        self.log_info(f"✅ Точка '{name}' сохранена: ({pose['x']:.2f}, {pose['y']:.2f}, {pose['theta']:.2f})")

        return MCPToolResult(
            success=True,
            data={"name": name, **pose},
            message=f"Запомнил! Точка '{name}' сохранена ({pose['x']:.1f}, {pose['y']:.1f})",
        )


# ============================================================
# DeleteWaypointTool — removes a named waypoint
# ============================================================


class DeleteWaypointTool(MCPTool):
    """Удалить именованную точку."""

    def __init__(self, node, waypoint_store: "WaypointStore"):
        super().__init__(node)
        self.waypoint_store = waypoint_store

    @property
    def name(self) -> str:
        return "delete_waypoint"

    @property
    def description(self) -> str:
        return (
            "Удалить сохранённую точку по имени. "
            "Используй когда пользователь говорит 'удали зал', 'забудь кухню'."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="name",
                type="string",
                description="Название точки для удаления",
                required=True,
            )
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.INSTANT

    def execute(self, name: str) -> MCPToolResult:
        self.log_info(f"Удаление точки: {name}")

        if self.waypoint_store.delete_waypoint(name):
            return MCPToolResult(success=True, message=f"Точка '{name}' удалена")
        else:
            return MCPToolResult(
                success=False,
                error=f"Точка '{name}' не найдена",
                message=f"Нет такой точки. Проверь список через list_waypoints.",
            )


# ============================================================
# ClearWaypointsTool — removes ALL waypoints for the active map
# ============================================================


class ClearWaypointsTool(MCPTool):
    """Удалить все точки текущей карты."""

    def __init__(self, node, waypoint_store: "WaypointStore"):
        super().__init__(node)
        self.waypoint_store = waypoint_store

    @property
    def name(self) -> str:
        return "clear_waypoints"

    @property
    def description(self) -> str:
        return (
            "Удалить ВСЕ сохранённые точки на текущей карте. "
            "Используй когда пользователь говорит 'очисти все точки', 'удали все точки'."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return []

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.INSTANT

    def execute(self) -> MCPToolResult:
        self.log_info("Очистка всех точек")
        count = self.waypoint_store.clear_waypoints()
        return MCPToolResult(
            success=True,
            data={"deleted_count": count},
            message=f"Удалено {count} точек" if count > 0 else "Нет точек для удаления",
        )


# ============================================================
# GetCurrentPoseTool — returns robot's current map position
# ============================================================


class GetCurrentPoseTool(MCPTool):
    """Получить текущую позицию робота на карте."""

    def __init__(self, node, tf_buffer: "tf2_ros.Buffer"):
        super().__init__(node)
        self.tf_buffer = tf_buffer

    @property
    def name(self) -> str:
        return "get_current_pose"

    @property
    def description(self) -> str:
        return (
            "Получить текущую позицию робота (x, y, theta) в системе координат карты. "
            "Используй перед 'миссиями' чтобы запомнить точку возврата, "
            "или когда пользователь спрашивает 'где ты?'."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return []

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    def execute(self) -> MCPToolResult:
        self.log_info("Запрос текущей позиции")

        pose = _lookup_pose(self.tf_buffer, self.node.get_logger())
        if pose is None:
            return MCPToolResult(
                success=False,
                error="Не могу определить позицию (TF map→base_link недоступен)",
                message="Позиция неизвестна",
            )

        self.log_info(f"📍 Позиция: x={pose['x']:.2f}, y={pose['y']:.2f}, θ={pose['theta']:.2f}")
        return MCPToolResult(
            success=True,
            data=pose,
            message=f"Позиция: ({pose['x']:.2f}, {pose['y']:.2f}), угол {math.degrees(pose['theta']):.0f}°",
        )
