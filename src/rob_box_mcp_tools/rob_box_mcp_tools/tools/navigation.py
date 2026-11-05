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

from typing import Any, Callable, List, Optional, TYPE_CHECKING
import math
import time
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo

if TYPE_CHECKING:
    from ..waypoint_adapter import WaypointAdapter
    from ..waypoint_store import WaypointStore

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType, wait_future
from ..core.action_outcome import (
    CANCEL_CONFIRMED,
    CANCEL_NO_ACTIVE_GOAL,
    NAV_NO_RESPONSE,
    NAV_REJECTED,
    NAV_SERVER_UNAVAILABLE,
    NAV_SUCCEEDED,
    NAV_TIMEOUT,
    classify_cancel_response,
    classify_nav_status,
    classify_service_call,
    nav_data,
    nav_outcome,
    start_distance,
)
from rob_box_dialog import knowledge as kn
from ..map_pose import LEGACY_WAYPOINT_FRAME, MapPose

_CANCEL_ALL_LABEL = "Nav2 cancel_goal"
_CANCEL_ALL_TIMEOUT_SEC = 5.0

# #4435: «сирота» FollowPath. Nav2 Humble ``BtActionNode`` при таймауте подтверждения цели
# (``default_server_timeout``) сбрасывает future и возвращает FAILURE без goal handle, а его
# ``halt()`` отменяет только цель с handle. Запрос цели уже ушёл: controller_server принимает
# её позже и ведёт робота, хотя NavigateToPose давно ABORTED. Поэтому при любом исходе ≠ успех
# наш слой сам гасит контроллер: отмена всех целей ``follow_path`` + нулевой ``cmd_vel``.
_FOLLOW_PATH_CANCEL_SRV = "/follow_path/_action/cancel_goal"
_FOLLOW_PATH_CANCEL_LABEL = "controller_server follow_path cancel_goal"
_NAV_CMD_VEL_TOPIC = "/cmd_vel"  # выход controller_server → twist_mux (вход navigation, priority 10)
_HALT_CANCEL_TIMEOUT_SEC = 2.0
# Второй раунд ловит позднее подтверждение: запрос FollowPath мог дойти до сервера уже после
# нашей первой отмены (отмена «всех» гасит только цели, принятые к моменту отмены).
_HALT_ROUNDS_ON_FAILURE = 2
_HALT_ROUND_GAP_SEC = 1.0
_ZERO_TWIST_REPEATS = 3


def _nav_failure(kind: str, status: Optional[int] = None) -> MCPToolResult:
    """Неуспех навигации: ошибку и фразу строит код (ADR-0148), не LLM."""
    outcome = nav_outcome(kind, status)
    return MCPToolResult(success=False, error=outcome.error, message=outcome.message, data=nav_data(outcome.kind))


def _with_target(result: MCPToolResult, target: str, **extra: Any) -> MCPToolResult:
    """Данные исхода с точкой словами (слот ``{target}`` фразы кода, #4344) и полями тула поверх."""
    result.data = {**(result.data or {}), kn.NAV_TARGET_FIELD: target, **extra}
    return result


def _distance_feedback(distances: List[float]):
    """Обратная связь Nav2 → ``distance_remaining`` в ``distances`` (путь до цели в начале поездки, #4344)."""

    def on_feedback(message) -> None:
        value = getattr(getattr(message, "feedback", None), "distance_remaining", None)
        if isinstance(value, (int, float)):
            distances.append(float(value))

    return on_feedback


def _cancel_all_goals(client, label: str, timeout_sec: float):
    """Отменить ВСЕ цели action-сервера (нулевые uuid и время). Возвращает ``(исход вызова, ответ)``."""
    ready = client.wait_for_service(timeout_sec=0.5)
    response, completed, error = None, False, None
    if ready:
        request = CancelGoal.Request()
        request.goal_info = GoalInfo()  # нулевые uuid и время — отменить все цели
        future = client.call_async(request)
        completed = wait_future(future, timeout_sec=timeout_sec)
        response, error = _future_response(future, completed)
    call = classify_service_call(ready=ready, completed=completed, response=response, label=label, error=error)
    return call, response


class ControllerHalt:
    """Гарантированная остановка controller_server после NavigateToPose (#4435).

    ``halt(reason, rounds)`` отменяет все цели ``follow_path`` и публикует нулевой ``cmd_vel``.
    Возвращает вид исхода отмены (``action_outcome``): ``CANCEL_CONFIRMED`` — «сирота» была
    и погашена, ``CANCEL_NO_ACTIVE_GOAL`` — контроллер и так стоял, иначе — сбой вызова.
    """

    def __init__(self, node):
        self._node = node
        self._cancel_client = node.create_client(CancelGoal, _FOLLOW_PATH_CANCEL_SRV)
        self._cmd_vel = node.create_publisher(Twist, _NAV_CMD_VEL_TOPIC, 10)

    def __call__(self, reason: str, rounds: int = _HALT_ROUNDS_ON_FAILURE) -> str:
        kinds = []
        for i in range(max(1, rounds)):
            if i:
                time.sleep(_HALT_ROUND_GAP_SEC)
            kinds.append(self._cancel_round())
            self._publish_zero()
        kind = CANCEL_CONFIRMED if CANCEL_CONFIRMED in kinds else kinds[-1]
        log = self._node.get_logger()
        line = f"#4435 halt контроллера ({reason}): отмена follow_path {kinds} -> {kind}, cmd_vel=0"
        (log.warning if kind == CANCEL_CONFIRMED else log.info)(line)
        return kind

    def _cancel_round(self) -> str:
        call, response = _cancel_all_goals(self._cancel_client, _FOLLOW_PATH_CANCEL_LABEL, _HALT_CANCEL_TIMEOUT_SEC)
        if not call.success:
            return call.kind
        return classify_cancel_response(response).kind

    def _publish_zero(self) -> None:
        for _ in range(_ZERO_TWIST_REPEATS):
            self._cmd_vel.publish(Twist())


def _halt_after(result: MCPToolResult, halt: Optional[Callable[..., str]], reason: str) -> MCPToolResult:
    """Неуспех навигации → погасить контроллер; исход остановки — в ``data`` (для логов и HIL)."""
    if halt is not None:
        result.data = {**(result.data or {}), "controller_halt": halt(reason)}
    return result


def _cancel_goal_best_effort(goal_handle, logger) -> None:
    """Таймаут не должен оставлять робота ехать дальше, пока мы говорим «не доехал»."""
    try:
        goal_handle.cancel_goal_async()
    except Exception as exc:  # noqa: BLE001 - отмена вторична, ответ об исходе важнее
        logger.warning(f"Не удалось отменить цель Nav2 после таймаута: {exc}")


def _send_nav_goal(
    nav_client,
    node,
    x: float,
    y: float,
    theta: float,
    frame_id: str = "map",
    timeout: float = 120.0,
    halt: Optional[Callable[..., str]] = None,
):
    """Shared helper: send a NavigateToPose goal and block until completion.

    Успех — только если Nav2 вернул ``GoalStatus.STATUS_SUCCEEDED`` (#3789):
    ``ABORTED``/``CANCELED``/таймаут/отказ в приёме цели — честный неуспех.
    Данные успеха — как достигнута цель (``action_outcome.nav_data``, #4344): время от
    принятия цели до ответа Nav2 и путь до цели по обратной связи.

    #4435: если цель ушла в Nav2 и исход ≠ успех (ABORTED/CANCELED/неизвестный статус,
    таймаут результата, нет ответа на отправку) — ``halt`` гасит controller_server
    (``ControllerHalt``). Без сервера и при отказе в приёме цели гасить нечего.

    Returns ``MCPToolResult``.
    """
    if not nav_client.wait_for_server(timeout_sec=1.0):
        return _nav_failure(NAV_SERVER_UNAVAILABLE)

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = frame_id
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.position.z = 0.0
    goal.pose.pose.orientation.z = math.sin(theta / 2.0)
    goal.pose.pose.orientation.w = math.cos(theta / 2.0)

    distances: List[float] = []
    send_future = nav_client.send_goal_async(goal, feedback_callback=_distance_feedback(distances))
    if not wait_future(send_future, timeout_sec=10.0) or send_future.result() is None:
        return _halt_after(_nav_failure(NAV_NO_RESPONSE), halt, NAV_NO_RESPONSE)

    goal_handle = send_future.result()
    if not goal_handle.accepted:
        return _nav_failure(NAV_REJECTED)

    started = time.monotonic()
    result_future = goal_handle.get_result_async()
    if not wait_future(result_future, timeout_sec=timeout) or result_future.result() is None:
        _cancel_goal_best_effort(goal_handle, node.get_logger())
        return _halt_after(_nav_failure(NAV_TIMEOUT), halt, NAV_TIMEOUT)

    status = getattr(result_future.result(), "status", None)
    outcome = classify_nav_status(status)
    if not outcome.success:
        return _halt_after(_nav_failure(outcome.kind, status), halt, outcome.kind)
    return MCPToolResult(
        success=True, data=nav_data(NAV_SUCCEEDED, time.monotonic() - started, start_distance(distances))
    )


def _arrived_message(result: MCPToolResult, where: str) -> str:
    """Подсказка тула модели (v1) по тому же исходу, что и фраза кода диалога: уже стоял или доехал."""
    if (result.data or {}).get(kn.NAV_ARRIVAL_FIELD) == kn.NAV_ALREADY_THERE:
        return f"Уже стою на {where}"
    return f"Приехал к {where}"


def _future_response(future, completed: bool):
    """``(ответ, текст ошибки)`` будущего вызова сервиса; ``result()`` rclpy может бросить."""
    if not completed:
        return None, None
    try:
        return future.result(), None
    except Exception as exc:  # noqa: BLE001 - исключение будущего = вызов не удался
        return None, str(exc) or type(exc).__name__


def _pose_failure(pose: MapPose, message: str) -> MCPToolResult:
    """Отказ по позе на карте: причину строит код из статуса (#4390, ADR-0148), не одометрия молча."""
    return MCPToolResult(
        success=False,
        error=f"Не могу определить позицию робота на карте: {pose.reason}",
        message=message,
        data={"pose_status": pose.status, "frame_id": pose.frame_id},
    )


# ============================================================
# NavigateToWaypointTool — reads from WaypointStore (DB)
# ============================================================


class NavigateToWaypointTool(MCPTool):
    """Навигация к именованной точке из базы данных вейпоинтов."""

    event_source = "action:/navigate_to_pose"

    def __init__(self, node, waypoint_store: "WaypointAdapter"):
        super().__init__(node)
        self.nav_client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self.controller_halt = ControllerHalt(node)
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
    def slice(self) -> str:
        # ADR-0052 / issue #1998 §6.2: navigation, personality.
        return "personality"

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.LONG

    def execute(self, waypoint: str) -> MCPToolResult:
        """Выполнить навигацию к точке из БД."""
        self.log_info(f"Навигация к точке: {waypoint}")

        coords = self.waypoint_store.get_waypoint(waypoint)
        if coords is None:
            available = self.waypoint_store.list_waypoints()
            names = ", ".join(w["name"] for w in available) if available else "нет сохранённых точек"
            return MCPToolResult(
                success=False,
                error=f"Неизвестная точка '{waypoint}'",
                message=f"Доступные точки: {names}",
                data={**nav_data(kn.NAV_UNKNOWN_TARGET), kn.NAV_TARGET_FIELD: waypoint},
            )

        # Цель — в кадре точки (#4390). Точка до #4390 без frame_id — как и раньше в ``map``,
        # но помечена: её писали из /odom, совпадение с картой не гарантировано.
        frame_id = coords.get("frame_id") or LEGACY_WAYPOINT_FRAME
        frame_assumed = not coords.get("frame_id")
        if frame_assumed:
            self.log_warning(f"Точка '{waypoint}' без кадра (записана до #4390 из /odom) — цель в '{frame_id}'")
        self.log_info(
            f"📍 Координаты: x={coords['x']:.2f}, y={coords['y']:.2f}, θ={coords['theta']:.2f} [{frame_id}]"
        )

        result = _send_nav_goal(
            self.nav_client,
            self.node,
            coords["x"],
            coords["y"],
            coords["theta"],
            frame_id=frame_id,
            timeout=120.0,
            halt=self.controller_halt,
        )
        _with_target(
            result, waypoint, waypoint=waypoint, coordinates=coords, frame_id=frame_id, frame_assumed=frame_assumed
        )
        if result.success:
            result.message = _arrived_message(result, f"точке {waypoint}")
            self.log_info(f"✅ Навигация к {waypoint} завершена: {result.data.get(kn.NAV_ARRIVAL_FIELD)}")
        return result


# ============================================================
# NavigateToCoordinatesTool — navigate to arbitrary (x, y, θ)
# ============================================================


class NavigateToCoordinatesTool(MCPTool):
    """Навигация к произвольным координатам в map frame."""

    event_source = "action:/navigate_to_pose"

    def __init__(self, node):
        super().__init__(node)
        self.nav_client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self.controller_halt = ControllerHalt(node)

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
    def slice(self) -> str:
        # ADR-0052 / issue #1998 §6.2: navigation, personality.
        return "personality"

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.LONG

    def execute(self, x: float, y: float, theta: float = 0.0) -> MCPToolResult:
        self.log_info(f"Навигация к координатам: x={x:.2f}, y={y:.2f}, θ={theta:.2f}")

        result = _send_nav_goal(self.nav_client, self.node, x, y, theta, timeout=120.0, halt=self.controller_halt)
        _with_target(result, f"({x:.1f}, {y:.1f})", x=x, y=y, theta=theta)
        if result.success:
            result.message = _arrived_message(result, f"точке ({x:.1f}, {y:.1f})")
            self.log_info(f"✅ Навигация к ({x:.2f}, {y:.2f}) завершена")
        return result


# ============================================================
# MoveDirectionTool — relative movement
# ============================================================


class MoveDirectionTool(MCPTool):
    """Инструмент для движения в направлении."""

    event_source = "action:/navigate_to_pose"

    DIRECTIONS = {
        "вперёд": {"x": 1.0, "y": 0.0, "theta": 0.0},
        "назад": {"x": -1.0, "y": 0.0, "theta": 0.0},
        "налево": {"x": 0.0, "y": 0.0, "theta": math.pi / 2},
        "направо": {"x": 0.0, "y": 0.0, "theta": -math.pi / 2},
    }

    def __init__(self, node):
        super().__init__(node)
        self.nav_client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self.controller_halt = ControllerHalt(node)

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
    def slice(self) -> str:
        # ADR-0052 / issue #1998 §6.2: navigation, personality.
        return "personality"

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.LONG

    def execute(self, direction: str, distance: float = 1.0) -> MCPToolResult:
        """Выполнить движение в направлении."""
        self.log_info(f"Движение: {direction}, дистанция: {distance}м")

        if direction not in self.DIRECTIONS:
            return MCPToolResult(success=False, error=f"Неизвестное направление: {direction}")

        coords = self.DIRECTIONS[direction].copy()
        if direction in ["вперёд", "назад"]:
            coords["x"] *= distance

        result = _send_nav_goal(
            self.nav_client,
            self.node,
            coords["x"],
            coords["y"],
            coords["theta"],
            frame_id="base_link",
            timeout=60.0,
            halt=self.controller_halt,
        )
        if result.success:
            result.data = {
                **(result.data or {}), "direction": direction, "distance": distance, "relative_coords": coords
            }
            result.message = f"Приехал: {direction} {distance}м"
            self.log_info(f"✅ Движение {direction} завершено")
        return result


# ============================================================
# StopNavigationTool
# ============================================================


class StopNavigationTool(MCPTool):
    """Инструмент для остановки навигации."""

    event_source = "srv:/navigate_to_pose/_action/cancel_goal"

    def __init__(self, node):
        super().__init__(node)
        self.cancel_client = node.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
        self.controller_halt = ControllerHalt(node)

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
    def slice(self) -> str:
        # ADR-0052 / issue #1998 §6.2: stop, personality.
        return "personality"

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    def execute(self) -> MCPToolResult:
        self.log_info("Остановка навигации")

        call, response = _cancel_all_goals(self.cancel_client, _CANCEL_ALL_LABEL, _CANCEL_ALL_TIMEOUT_SEC)
        # #4435: контроллер гасим всегда — «сирота» FollowPath едет и без активной NavigateToPose.
        halt_kind = self.controller_halt("stop_navigation", rounds=1)

        # Успех — только по ответу сервера отмены (#3789, ADR-0148): фразу строит код.
        if not call.success:
            self.log_warning(f"⚠️ {call.error}")
            return MCPToolResult(
                success=False,
                error=call.error,
                message="Не уверен, что остановился: отмена навигации не подтверждена",
                data={"outcome": call.kind},
            )

        outcome = classify_cancel_response(response)
        if not outcome.success:
            self.log_warning(f"⚠️ {outcome.error}")
            return MCPToolResult(
                success=False, error=outcome.error, message=outcome.message, data={"outcome": outcome.kind}
            )
        self.log_info(f"Отмена подтверждена: {outcome.kind}")
        kind, message = outcome.kind, outcome.message
        if kind == CANCEL_NO_ACTIVE_GOAL and halt_kind == CANCEL_CONFIRMED:
            # Цели навигации не было, а контроллер ещё вёл робота (#4435): это остановка, не «и так стою».
            kind, message = CANCEL_CONFIRMED, "Останавливаюсь"
        return MCPToolResult(success=True, message=message, data={"outcome": kind, "controller_halt": halt_kind})


# ============================================================
# ListWaypointsTool — reads from WaypointStore
# ============================================================


class ListWaypointsTool(MCPTool):
    """Список доступных точек из базы данных."""

    def __init__(self, node, waypoint_store: "WaypointAdapter"):
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

    event_source = "inproc:waypoint_store"

    def __init__(self, node, waypoint_store: "WaypointAdapter", pose_source: Callable[[], MapPose], mapping_state=None):
        super().__init__(node)
        self.waypoint_store = waypoint_store
        self._pose_source = pose_source
        self.mapping_state = mapping_state

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
    def slice(self) -> str:
        # ADR-0052 / issue #1998 §6.2: save waypoint, personality.
        return "personality"

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    def execute(self, name: str) -> MCPToolResult:
        self.log_info(f"Сохранение точки: {name}")

        # Нельзя сохранять точки пока идёт картографирование
        if self.mapping_state is not None and self.mapping_state.is_mapping():
            return MCPToolResult(
                success=False,
                error="Сейчас идёт картографирование. Завершите картографирование командой 'завершить картографирование', потом сохраняйте точки.",
            )

        pose = self._pose_source()
        if not pose.ok:
            self.log_warning(f"Точку '{name}' не сохранил: {pose.status} {pose.detail}")
            return _pose_failure(pose, f"Позиция на карте неизвестна — точку '{name}' не сохранил.")

        self.waypoint_store.save_waypoint(name, pose.x, pose.y, pose.theta, frame_id=pose.frame_id)
        self.log_info(f"✅ Точка '{name}' сохранена: ({pose.x:.2f}, {pose.y:.2f}, {pose.theta:.2f}) [{pose.frame_id}]")

        return MCPToolResult(
            success=True,
            data={"name": name, **pose.as_dict()},
            message=f"Запомнил! Точка '{name}' сохранена ({pose.x:.1f}, {pose.y:.1f})",
        )


# ============================================================
# DeleteWaypointTool — removes a named waypoint
# ============================================================


class DeleteWaypointTool(MCPTool):
    """Удалить именованную точку."""

    event_source = "inproc:waypoint_store"

    def __init__(self, node, waypoint_store: "WaypointAdapter"):
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
    def slice(self) -> str:
        # ADR-0052 / issue #1998 §6.2: delete waypoint, personality.
        return "personality"

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

    event_source = "inproc:waypoint_store"

    def __init__(self, node, waypoint_store: "WaypointAdapter"):
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
    def slice(self) -> str:
        # ADR-0052 / issue #1998 §6.2: clear waypoints, personality.
        return "personality"

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

    def __init__(self, node, pose_source: Callable[[], MapPose]):
        super().__init__(node)
        self._pose_source = pose_source

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

        pose = self._pose_source()
        if not pose.ok:
            self.log_warning(f"Позиция на карте недоступна: {pose.status} {pose.detail}")
            return _pose_failure(pose, "Позиция на карте неизвестна")

        self.log_info(f"📍 Позиция: x={pose.x:.2f}, y={pose.y:.2f}, θ={pose.theta:.2f} [{pose.frame_id}]")
        return MCPToolResult(
            success=True,
            data=pose.as_dict(),
            message=f"Позиция: ({pose.x:.2f}, {pose.y:.2f}), угол {math.degrees(pose.theta):.0f}°",
        )
