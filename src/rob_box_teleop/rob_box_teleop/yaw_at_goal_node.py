"""Финальный подъезд к цели, когда xy уже внутри допуска, а yaw нет (#4694).

Живёт в ``followpath_watchdog_node`` (тот же контейнер, тот же процесс): второй
процесс в контейнере ломает выход по SIGTERM. Решает всё ``yaw_at_goal.approach_at_goal``.
Нода подаёт позу ``map ← base_footprint`` и конец живого ``/plan``: это та точка,
с которой ``SimpleGoalChecker`` сравнивает робота (``controller_server`` берёт
``path.poses.back()``). Допуски — из ``nav2_params.yaml``, не константы ноды.

Зачем трансляция, а не только разворот. DWB ``RotateToGoal`` защёлкнут: ``rotating_``
снимается только новым ``setPlan``, а у цели план замирает. Защёлкнутый контроллер везти
не может, progress checker (только трансляция) рвёт FollowPath, и робот досиживает таймаут.
Gate 2 21.11 M3: «разворот на месте» по истине шёл по окружности радиуса 0.088 м, импульс
курса на границе 0.25 м сам выносил позу из окна, и нода отпускала пол. Поэтому нода
держит цель (``_hold``), пока сама не доедет, даже если план старше ``PLAN_STALE_S``.

Механизм: захват только внутри ``xy_goal_tolerance``; дальше ``APPROACH_TARGET_M`` (0.10 м)
сначала доворот носа на точку, потом ход не быстрее ``min(0.10, FollowPath.max_speed_xy)``;
у точки — доворот на курс цели; затем ровно один нулевой Twist и тишина. 0.10 + 0.088 =
0.188 м, внутри допуска checker'а 0.25 м с запасом: доворот к курсу цели снова по окружности
позу из окна не вынесет. Допуск checker'а и допуск приёмки нода не ослабляет.

Границы удержания. Поза дальше ``ESCAPE_RADIUS_M`` (0.60 м) от удержанной цели — это уже
не финал, цель забываем. Дольше ``ENGAGE_TIME_CAP_S`` (25 с) одну цель не держим: после
этого ``REENGAGE_COOLDOWN_S`` (5 с) тишины, иначе замороженный хвост крутил бы вечно.
Свежий хвост с другой точкой или курсом перенимаем (checker сравнивает с ним); тот же
замороженный хвост часы захвата не сбрасывает.

Twist идёт в ``/cmd_vel_yaw`` (twist_mux priority 15, выше Nav2, timeout 0.2 с), оба поля:
``linear.x`` и ``angular.z``. FollowPath не отменяем и в его ``/cmd_vel`` не пишем: mux сам
отдаёт подъезду пол, пока тик свежий, и возвращает его контроллеру, когда нода замолчала.
Вне подъезда нода молчит: нулевой поток на 10 Гц не даёт timeout 0.2 истечь, и канал
priority 15 вечно перебивает Nav2 (Depot Gate 2, 20.11: M2/M5/R1/R2/R3, 20/20 ``wz=0``).
Снять ранее начатый подъезд — ровно один нулевой Twist, дальше тишина.
Замки руля (51 и 90) глушат подъезд так же, как Nav2.
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
    позы уже внутри xy-допуска хвоста: у цели planner замирает, подъезд ещё нужен.
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
    """Поза и удержанная цель → ``approach_at_goal`` → twist в ``/cmd_vel_yaw``.

    Не нода: параметры объявляет хозяин (``followpath_watchdog``), чтобы имя
    ``declare_parameter`` не повторилось. ``node`` — любой объект с
    ``create_subscription`` / ``create_publisher`` / ``create_timer`` / ``get_logger``.
    """

    def __init__(self, node) -> None:
        self._node = node
        self._xy, self._yaw, self._omega, self._max_v = self._load_tolerances(
            str(node.get_parameter("nav2_params_file").value)
        )
        self._tf_timeout = float(node.get_parameter("tf_timeout_s").value)
        self._wire(node)

    def _wire(self, node) -> None:
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
        self._hold = None
        self._engaged_at = 0.0
        self._block_until = 0.0
        self._floor = False  # пол у mux: был наш Twist, снять можно только одним нулём
        self._reason = ""
        rate = float(node.get_parameter("check_rate_hz").value)
        node.create_timer(1.0 / max(rate, 1.0), self._tick)
        node.get_logger().info(
            f"#4694 финальный подъезд: xy≤{self._xy:.2f} м, yaw≤{self._yaw:.2f} рад, "
            f"w≤{self._omega:.2f} рад/с, v≤{self._max_v:.2f} м/с"
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

    def _forget(self, now: float, block: bool) -> None:
        """Цель больше не держим. ``block`` — потолок: столько тишины до нового захвата."""
        self._hold = None
        if block:
            self._block_until = now + yag.REENGAGE_COOLDOWN_S
        self._release()

    def _take(self, tail: tuple, now: float) -> None:
        self._hold = tail
        self._engaged_at = now

    def _capture(self, tail, pose, now: float) -> None:
        """Нет удержания — взять хвост, только если окно, кулдаун прошёл и хвост есть."""
        if self._hold is not None or tail is None:
            return
        if now < self._block_until or not yag.xy_inside(pose, tail, self._xy):
            return
        self._take(tail, now)

    def _retarget(self, tail, now: float) -> None:
        """Свежий другой хвост — то, с чем сравнивает checker. Тот же хвост часы не сбрасывает."""
        if self._hold is None or tail is None:
            return
        if yag.goal_differs(self._hold, tail, yag.GOAL_CHANGE_EPS_M, yag.GOAL_CHANGE_EPS_RAD):
            self._take(tail, now)

    def _within_bounds(self, pose, now: float) -> bool:
        """Потолок времени и радиус побега. False — тик закрыт, цель забыта."""
        if now - self._engaged_at > yag.ENGAGE_TIME_CAP_S:
            self._node.get_logger().warning(f"#4694 потолок {yag.ENGAGE_TIME_CAP_S:.0f} с — цель забыта, тишина")
            self._forget(now, block=True)
            return False
        dist = yag.pose_distance(pose, self._hold)
        if dist > yag.ESCAPE_RADIUS_M:
            self._node.get_logger().warning(f"#4694 побег {dist:.2f} м > {yag.ESCAPE_RADIUS_M:.2f} — цель забыта")
            self._forget(now, block=False)
            return False
        return True

    def _track(self, pose, now: float):
        """Удержать цель или отпустить. ``None`` — этот тик уже закрыт."""
        tail = goal_from_plan(self._plan, now, pose=pose, xy_tolerance_m=self._xy)
        self._capture(tail, pose, now)
        self._retarget(tail, now)
        if self._hold is None:
            self._release()
            return None
        if not self._within_bounds(pose, now):
            return None
        return self._hold

    def _tick(self) -> None:
        if not self._nav_live:
            self._hold = None
            self._release()
            return
        now = self._now()
        pose = self._pose()
        if pose is None:
            self._release()  # цель не забываем: TF вернётся — доезжаем ту же
            return
        goal = self._track(pose, now)
        if goal is None:
            return
        self._step(pose, goal)

    def _step(self, pose, goal) -> None:
        step = yag.approach_at_goal(pose, goal, self._xy, self._yaw, self._max_v, self._omega)
        if step is None or step.done:
            self._hold = None
            self._release()
            return
        self._publish(step.vx, step.omega)
        self._log_reason(step, pose, goal)
        self._floor = True

    def _log_reason(self, step, pose, goal) -> None:
        if step.reason == self._reason:
            return
        dist = yag.pose_distance(pose, goal)
        dyaw = yag.shortest_yaw(float(pose[2]), float(goal[2]))
        self._node.get_logger().info(
            f"#4694 {step.reason}: dist={dist:.3f} dyaw={dyaw:.3f} vx={step.vx:.2f} w={step.omega:.2f}"
        )
        self._reason = step.reason

    def _release(self) -> None:
        """Снять пол у mux, если мы его держали. Иначе — ни одного сообщения."""
        if not self._floor:
            return
        self._node.get_logger().info("#4694 финальный подъезд снят — один ноль, дальше тишина, пол у Nav2")
        self._publish(0.0, 0.0)
        self._floor = False
        self._reason = ""

    def _publish(self, vx: float, omega: float) -> None:
        twist = Twist()
        twist.linear.x = float(vx)
        twist.angular.z = float(omega)
        self._cmd_vel.publish(twist)
