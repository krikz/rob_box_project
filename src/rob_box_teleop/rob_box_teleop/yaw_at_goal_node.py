"""Разворот на месте, когда xy уже внутри допуска, а yaw нет (#4694).

Живёт в ``followpath_watchdog_node`` (тот же контейнер, тот же процесс): второй
процесс в контейнере ломает выход по SIGTERM. Решает всё ``yaw_at_goal.turn_at_goal``.
Нода подаёт позу ``map ← base_footprint`` и конец живого ``/plan``: это та точка,
с которой ``SimpleGoalChecker`` сравнивает робота (``controller_server`` берёт
``path.poses.back()``). Допуск yaw — ``yaw_goal_tolerance`` из ``nav2_params.yaml``,
не константа ноды.

Twist идёт в ``/cmd_vel_yaw`` (twist_mux priority 15, выше Nav2). FollowPath не
отменяем и в его ``/cmd_vel`` не пишем: mux сам отдаёт развороту пол, пока тик свежий,
и возвращает его контроллеру, когда yaw вошёл в допуск и нода замолчала. Снаружи
xy-окна нода молчит: это ещё FollowPath. Замки руля (51 и 90) глушат разворот так же,
как Nav2.
"""

from __future__ import annotations

from typing import Optional

import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from rob_box_core.follow_path_halt import YAW_AT_GOAL_CMD_VEL_TOPIC
from rob_box_core.tf_lookup import NoTransform, Tf2Lookup, TransformStale
from rob_box_teleop import yaw_at_goal as yag
from rob_box_teleop.followpath_watchdog_node import PARENT_STATUS_TOPICS

# Те же родители, что у сторожа сирот: полоса HIL — NavigateThroughPoses, не второй топик.
PARENT_TOPICS = tuple(topic for _, topic in PARENT_STATUS_TOPICS)
PLAN_TOPIC = "/plan"
MAP_FRAME = "map"
BASE_FRAME = "base_footprint"
# action_msgs/GoalStatus. Числа, не импорт: заглушка тестов делает из класса MagicMock,
# и «живой» статус тогда истинен всегда. Те же коды, что у followpath_orphan.
_STATUS_ACCEPTED = 1
_STATUS_EXECUTING = 2
# Живая родительская цель (NavigateToPose или NavigateThroughPoses). Отмена и успех — уже не наша забота.
_NAV_LIVE = frozenset({_STATUS_ACCEPTED, _STATUS_EXECUTING})
# План старше этого — цель уже не едет (planner молчит без цели). Не крутим по старому хвосту.
PLAN_STALE_S = 2.0


def _yaw_quat(q) -> float:
    return yag.shortest_yaw(
        0.0,
        _atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
    )


def _atan2(y: float, x: float) -> float:
    import math

    return math.atan2(y, x)


def goal_from_plan(path, now_s: float, stale_s: float = PLAN_STALE_S) -> Optional[tuple]:
    """Конец ``/plan`` как цель checker'а. Пустой, чужой кадр или старый план — не цель."""
    poses = getattr(path, "poses", None) or []
    header = getattr(path, "header", None)
    frame = getattr(header, "frame_id", "")
    if not poses or frame != MAP_FRAME:
        return None
    stamp = getattr(header, "stamp", None)
    if stamp is not None and now_s - (stamp.sec + stamp.nanosec * 1e-9) > stale_s:
        return None
    last = poses[-1].pose
    return (float(last.position.x), float(last.position.y), _yaw_quat(last.orientation))


class YawAtGoal:
    """Поза и конец плана → ``turn_at_goal`` → twist в ``/cmd_vel_yaw``.

    Не нода: параметры объявляет хозяин (``followpath_watchdog``), чтобы имя
    ``declare_parameter`` не повторилось. ``node`` — любой объект с
    ``create_subscription`` / ``create_publisher`` / ``create_timer`` / ``get_logger``.
    """

    def __init__(self, node) -> None:
        self._node = node
        self._xy, self._yaw, self._omega = self._load_tolerances(str(node.get_parameter("nav2_params_file").value))
        self._tf_timeout = float(node.get_parameter("tf_timeout_s").value)

        import tf2_ros  # noqa: PLC0415 — TF нужен только живой ноде

        buffer = tf2_ros.Buffer()
        # Конвертеры времени — заглушки: latest() зовёт lookup_transform, секунды ему не нужны.
        # Без них Tf2Lookup лезет в rclpy.duration, которого в юнит-тесте нет.
        self._lookup = Tf2Lookup(
            buffer, exceptions=(LookupError, LookupError, TimeoutError), to_time=lambda s: s, to_duration=lambda s: s
        )
        self._tf_listener = tf2_ros.TransformListener(buffer, node)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        from action_msgs.msg import GoalStatusArray  # noqa: PLC0415 — тип подписки, в тесте заглушка

        for topic in PARENT_TOPICS:
            node.create_subscription(GoalStatusArray, topic, self._on_nav, qos)
        node.create_subscription(Path, PLAN_TOPIC, self._on_plan, 10)
        self._cmd_vel = node.create_publisher(Twist, YAW_AT_GOAL_CMD_VEL_TOPIC, 10)
        self._nav_live = False
        self._plan = None
        self._turning = False
        rate = float(node.get_parameter("check_rate_hz").value)
        node.create_timer(1.0 / max(rate, 1.0), self._tick)
        node.get_logger().info(
            f"#4694 разворот у цели: xy≤{self._xy:.2f} м, yaw≤{self._yaw:.2f} рад, w≤{self._omega:.2f} рад/с"
        )

    def _load_tolerances(self, path: str) -> tuple:
        with open(path, encoding="utf-8") as handle:
            return yag.tolerances_from_nav2(yaml.safe_load(handle))

    def _on_nav(self, msg) -> None:
        self._nav_live = any(s.status in _NAV_LIVE for s in msg.status_list)

    def _on_plan(self, msg) -> None:
        self._plan = msg

    def _now(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9

    def _pose(self) -> Optional[tuple]:
        try:
            tf = self._lookup.latest(MAP_FRAME, BASE_FRAME, self._tf_timeout)
        except (NoTransform, TransformStale, ValueError):
            return None
        return (tf.translation[0], tf.translation[1], tf.yaw)

    def _tick(self) -> None:
        goal = goal_from_plan(self._plan, self._now()) if self._nav_live else None
        pose = self._pose() if goal is not None else None
        turn = yag.turn_at_goal(pose, goal, self._xy, self._yaw, self._omega) if pose is not None else None
        if turn is None:
            self._turning = False
            return
        self._publish(turn.omega)
        if turn.done:
            if self._turning:
                self._node.get_logger().info("#4694 yaw вошёл в допуск — стоп, дальше goal checker")
            self._turning = False
            return
        if not self._turning:
            self._node.get_logger().info(f"#4694 {turn.reason}: w={turn.omega:.2f}")
        self._turning = True

    def _publish(self, omega: float) -> None:
        twist = Twist()
        twist.angular.z = float(omega)
        self._cmd_vel.publish(twist)
