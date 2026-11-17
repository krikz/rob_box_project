"""Nav-цель из VR → Nav2 ``NavigateToPose`` (issue #3151, волна 2).

Почему action, а не ``/goal_pose``. bt_navigator слушает ``goal_pose``
(PoseStamped) и сам заворачивает его в NavigateToPose — это проще, но
оператор тогда слепой: ни «Nav2 принял/отверг», ни «сколько осталось», ни
«доехал/сдался», ни отмены (``goal_pose`` отменить нельзя — только новой
целью). Мостик обещает статус-строку на HUD и кнопку отмены, поэтому —
action client ``navigate_to_pose`` (то же имя, что у
``rob_box_mcp_tools.tools.navigation``; bt_navigator запущен без ROS
namespace в ``docker/main/scripts/nav2/start_nav2_direct.sh``, а
zenoh-namespace ``robots/$ROBOT_ID`` у quest и nav2 общий —
``ros_with_namespace.sh`` в обоих compose).

Потоки. ``send_goal`` / ``cancel`` зовутся из aiohttp-потока (хендлеры
JSON_CMD), колбэки future/feedback — из ROS executor. Общее состояние
(«какая цель текущая») — под локом; ``emit`` обязан быть потокобезопасным
(в проде — ``WSSServer.broadcast_json_event``).

Вытеснение. Новая цель поверх активной — штатно для Nav2: bt_navigator
вытесняет старую, её result приходит как ABORTED. Этот «ABORTED» — не
провал, а замена, поэтому колбэки несут токен цели и события устаревших
целей молча отбрасываются.

Тестируется без rclpy: ``Nav2GoalBridge`` принимает action-клиент и
фабрику goal-сообщений извне (см. test/unit/test_nav2_goal.py).
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

from .core.nav_goal import (
    NACK_NAV2_UNAVAILABLE,
    NAV_GOAL_FRAME,
    STATE_ABORTED,
    STATE_ACCEPTED,
    STATE_ACTIVE,
    STATE_REJECTED,
    TERMINAL_STATES,
    NavGoalRequest,
    RateGate,
    nav_status_event,
    state_from_goal_status,
    yaw_to_quaternion_zw,
)

log = logging.getLogger(__name__)

#: Имя action-сервера bt_navigator (см. шапку модуля).
NAV2_ACTION_NAME: str = "navigate_to_pose"
#: nav_status{state:"active"} не чаще раза в столько секунд (≤ 2 Гц):
#: bt_navigator шлёт feedback на каждом тике BT (bt_loop_duration 10 мс).
NAV_FEEDBACK_MIN_PERIOD_S: float = 0.5
#: Честный обрыв «зависшей» цели (#3151): если Nav2 умирает между accept и
#: result, ``get_result_async()`` future никогда не завершится, и клиент
#: висит на «ЕДЕТ»/«ПРИНЯТО» вечно. Если с момента последнего прогресса
#: (send/accept/feedback) не было вестей столько секунд — честно aborted.
#: Покрывает и «accepted без единого feedback» — таймер стартует уже с
#: send_goal.
NAV2_RESULT_TIMEOUT_S: float = 15.0
#: nav_status{state:"aborted", reason:...} при срабатывании таймаута выше.
NAV2_TIMEOUT_REASON: str = "nav2_timeout"


@dataclass
class _ActiveGoal:
    req: NavGoalRequest
    token: int
    handle: Any = None
    cancel_requested: bool = False
    #: monotonic-время последнего прогресса (send/accept/feedback) — основа
    #: для check_timeout().
    last_progress_at: float = 0.0


class Nav2GoalBridge:
    """Одна активная nav-цель за раз; события → ``emit(dict)``."""

    def __init__(
        self,
        action_client: Any,
        make_goal: Callable[[NavGoalRequest], Any],
        emit: Callable[[dict[str, Any]], Any],
        *,
        on_terminal: Optional[Callable[[], None]] = None,
        clock: Callable[[], float] = time.monotonic,
        wall_ms: Callable[[], int] = lambda: int(time.time() * 1000),
        result_timeout_s: float = NAV2_RESULT_TIMEOUT_S,
    ) -> None:
        self._client = action_client
        self._make_goal = make_goal
        self._emit = emit
        self._on_terminal = on_terminal
        self._clock = clock
        self._wall_ms = wall_ms
        self._result_timeout_s = float(result_timeout_s)
        self._lock = threading.Lock()
        self._active: Optional[_ActiveGoal] = None
        self._token = 0
        self._feedback_gate = RateGate(NAV_FEEDBACK_MIN_PERIOD_S)

    # --- вызовы из ws-хендлеров ------------------------------------------

    def send_goal(self, req: NavGoalRequest) -> Optional[str]:
        """Отправить цель. ``None`` — ушла в Nav2, иначе причина nack."""
        if not self._client.server_is_ready():
            return NACK_NAV2_UNAVAILABLE
        goal = self._make_goal(req)
        with self._lock:
            self._token += 1
            token = self._token
            self._active = _ActiveGoal(req=req, token=token, last_progress_at=self._clock())
            self._feedback_gate.reset()
        future = self._client.send_goal_async(
            goal, feedback_callback=lambda fb: self._on_feedback(token, fb)
        )
        future.add_done_callback(lambda f: self._on_goal_response(token, f))
        log.info(
            "quest nav: goal seq=%d → Nav2 (x=%.2f y=%.2f yaw=%.2f)",
            req.seq, req.x, req.y, req.yaw,
        )
        return None

    def cancel(self) -> bool:
        """Отменить текущую цель. ``False`` — отменять нечего."""
        with self._lock:
            active = self._active
            if active is None:
                return False
            active.cancel_requested = True
            handle = active.handle
        # Ответ Nav2 на send_goal ещё не пришёл — отмену отправит
        # _on_goal_response, как только появится handle.
        if handle is not None:
            handle.cancel_goal_async()
        log.info("quest nav: cancel seq=%d", active.req.seq)
        return True

    def has_active_goal(self) -> bool:
        with self._lock:
            return self._active is not None

    def check_timeout(self) -> None:
        """Дёргать периодически (host timer): честно оборвать зависшую цель.

        Если Nav2 умер между accept и result, ``get_result_async()`` future
        не завершится никогда — клиент застрянет на «ЕДЕТ»/«ПРИНЯТО».
        Нет прогресса (send/accept/feedback) дольше ``result_timeout_s`` →
        aborted{reason:nav2_timeout} + best-effort cancel (ADR-0018: честный
        FAIL лучше тишины).
        """
        token, handle = self._stale_goal()
        if token is None:
            return
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001 — cancel не важнее aborted
                log.warning("quest nav: best-effort cancel on timeout failed: %s", exc)
        self._finish(token, STATE_ABORTED, reason=NAV2_TIMEOUT_REASON)

    def _stale_goal(self) -> tuple[Optional[int], Any]:
        with self._lock:
            active = self._active
            if active is None:
                return None, None
            if self._clock() - active.last_progress_at < self._result_timeout_s:
                return None, None
            return active.token, active.handle

    # --- колбэки ROS executor ---------------------------------------------

    def _current(self, token: int) -> Optional[_ActiveGoal]:
        with self._lock:
            active = self._active
            return active if active is not None and active.token == token else None

    def _on_goal_response(self, token: int, future: Any) -> None:
        active = self._current(token)
        if active is None:
            return  # цель уже вытеснена новой
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001 — сбой транспорта action
            self._finish(token, STATE_REJECTED, reason=f"send_failed: {exc}")
            return
        if handle is None or not getattr(handle, "accepted", False):
            self._finish(token, STATE_REJECTED, reason="nav2_rejected")
            return
        with self._lock:
            active.handle = handle
            active.last_progress_at = self._clock()
            cancel_now = active.cancel_requested
        self._emit_state(active.req, STATE_ACCEPTED)
        handle.get_result_async().add_done_callback(lambda f: self._on_result(token, f))
        if cancel_now:
            handle.cancel_goal_async()

    def _on_feedback(self, token: int, feedback_msg: Any) -> None:
        active = self._current(token)
        if active is None:
            return
        now = self._clock()
        with self._lock:
            active.last_progress_at = now
        # Робот жив — таймаут отложен, даже если emit ниже дросселируется.
        if not self._feedback_gate.admit(now):
            return
        feedback = getattr(feedback_msg, "feedback", None)
        distance = getattr(feedback, "distance_remaining", None)
        self._emit_state(
            active.req,
            STATE_ACTIVE,
            distance_remaining=float(distance) if isinstance(distance, (int, float)) else None,
        )

    def _on_result(self, token: int, future: Any) -> None:
        reason: Optional[str] = None
        try:
            state = state_from_goal_status(future.result().status)
        except Exception as exc:  # noqa: BLE001
            state, reason = None, f"result_failed: {exc}"
        if state not in TERMINAL_STATES:
            # Result без терминального статуса — не выдумываем успех.
            state = STATE_ABORTED
        self._finish(token, state, reason=reason)

    def _finish(self, token: int, state: str, *, reason: Optional[str] = None) -> None:
        with self._lock:
            active = self._active
            if active is None or active.token != token:
                return
            self._active = None
        log.info("quest nav: goal seq=%d → %s%s", active.req.seq, state, f" ({reason})" if reason else "")
        self._emit_state(active.req, state, reason=reason)
        if self._on_terminal is not None:
            self._on_terminal()

    def _emit_state(
        self,
        req: NavGoalRequest,
        state: str,
        *,
        distance_remaining: Optional[float] = None,
        reason: Optional[str] = None,
    ) -> None:
        event = nav_status_event(
            req,
            state,
            ts_ms=self._wall_ms(),
            distance_remaining=distance_remaining,
            reason=reason,
        )
        try:
            self._emit(event)
        except Exception as exc:  # noqa: BLE001 — статус не должен ронять ROS-колбэк
            log.warning("quest nav: nav_status emit failed: %s", exc)


def build_navigate_to_pose_goal(goal_cls: Any, req: NavGoalRequest, stamp: Any) -> Any:
    """NavigateToPose.Goal из проверенного запроса (кадр ``map``)."""
    goal = goal_cls()
    goal.pose.header.frame_id = NAV_GOAL_FRAME
    goal.pose.header.stamp = stamp
    goal.pose.pose.position.x = req.x
    goal.pose.pose.position.y = req.y
    goal.pose.pose.position.z = 0.0
    qz, qw = yaw_to_quaternion_zw(req.yaw)
    goal.pose.pose.orientation.z = qz
    goal.pose.pose.orientation.w = qw
    return goal


def create_nav2_goal_bridge(
    node: Any,
    *,
    emit: Callable[[dict[str, Any]], Any],
    on_terminal: Optional[Callable[[], None]] = None,
) -> Optional[Nav2GoalBridge]:
    """Собрать мост поверх настоящего rclpy ActionClient.

    ``None`` — в образе нет ``nav2_msgs`` (или ``rclpy.action``): nav-цели
    тогда честно отвергаются ``nav2_unavailable``, а не падает нода.
    """
    try:
        from nav2_msgs.action import NavigateToPose  # type: ignore[import-not-found]
        from rclpy.action import ActionClient  # type: ignore[import-not-found]
    except ImportError as exc:
        log.warning("quest nav: nav2_msgs/rclpy.action недоступны (%s) — nav_goal будет nack", exc)
        return None
    client = ActionClient(node, NavigateToPose, NAV2_ACTION_NAME)

    def make_goal(req: NavGoalRequest) -> Any:
        return build_navigate_to_pose_goal(
            NavigateToPose.Goal, req, node.get_clock().now().to_msg()
        )

    return Nav2GoalBridge(client, make_goal, emit, on_terminal=on_terminal)
