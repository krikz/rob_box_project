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
# План старше этого далеко от хвоста — не цель. У цели planner замирает; порог не поднимаем.
PLAN_STALE_S = 2.0


def _yaw_quat(q) -> float:
    return yag.shortest_yaw(
        0.0,
        _atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
    )


def _atan2(y: float, x: float) -> float:
    import math

    return math.atan2(y, x)


def goal_from_plan(
    path,
    now_s: float,
    stale_s: float = PLAN_STALE_S,
    pose: Optional[tuple] = None,
    xy_tolerance_m: float = 0.0,
) -> Optional[tuple]:
    """Конец ``/plan`` как цель checker'а.

    Пустой или чужой кадр — не цель. План старше ``stale_s`` — не цель, кроме
    позы уже внутри xy-допуска хвоста: у цели planner замирает, разворот ещё нужен.
    Далеко от старого хвоста путь не выдумываем.
    """
    poses = getattr(path, "poses", None) or []
    header = getattr(path, "header", None)
    frame = getattr(header, "frame_id", "")
    if not poses or frame != MAP_FRAME:
        return None
    last = poses[-1].pose
    tail = (float(last.position.x), float(last.position.y), _yaw_quat(last.orientation))
    stamp = getattr(header, "stamp", None)
    if stamp is not None and now_s - (stamp.sec + stamp.nanosec * 1e-9) > stale_s:
        if not yag.xy_inside(pose, tail, xy_tolerance_m):
            return None
    return tail


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
        # Боевой Tf2Lookup (Time/Duration), как map_pose/vision_hailo. Не lambda s: s:
        # tf2_ros.Buffer.lookup_transform ждёт Time, не float — иначе TypeError каждый тик
        # и /cmd_vel_yaw молчит (Depot 17.11 20:27Z, #4788 в образе, разворота нет).
        self._lookup = Tf2Lookup(buffer)
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
        pose = self._pose() if self._nav_live else None
        goal = (
            goal_from_plan(self._plan, self._now(), pose=pose, xy_tolerance_m=self._xy)
            if self._nav_live
            else None
        )
        turn = (
            yag.turn_at_goal(pose, goal, self._xy, self._yaw, self._omega)
            if pose is not None and goal is not None
            else None
        )
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
