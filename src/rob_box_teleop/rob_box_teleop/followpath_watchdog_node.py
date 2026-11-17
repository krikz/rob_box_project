"""followpath_watchdog_node — сторож «сирот» FollowPath для любого источника цели NavigateToPose (#4458).

Тонкая нода (контейнер ``followpath-watchdog`` на образе teleop). Решает всё ``followpath_orphan.decide``:
нода подаёт туда статусы ``/navigate_to_pose/_action/status`` и ``/follow_path/_action/status`` и время.

Срабатывание — как у тулов навигации (#4456): отмена цели ``controller_server`` через
``/follow_path/_action/cancel_goal`` (по ``goal_id``) и нулевой ``Twist`` в ``/cmd_vel`` (имена — общие,
``rob_box_core.follow_path_halt``). Каждое срабатывание пишется в журнал с ``goal_id`` и временем.
Выход по SIGTERM (``docker stop``) — чистый: нода запускается без ``ros2 run`` и сама ловит SIGTERM.
"""

from __future__ import annotations

import signal

import rclpy
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from rob_box_core.follow_path_halt import FOLLOW_PATH_CANCEL_SRV, NAV_CMD_VEL_TOPIC, ZERO_TWIST_REPEATS
from rob_box_teleop import followpath_orphan as fo

NAV_STATUS_TOPIC = "/navigate_to_pose/_action/status"
FOLLOW_STATUS_TOPIC = "/follow_path/_action/status"

_DEFAULTS = {
    "orphan_timeout_s": 1.5,
    "check_rate_hz": 10.0,
}


def _uuid_hex(goal_info) -> str:
    return bytes(bytearray(int(b) for b in goal_info.goal_id.uuid)).hex()


def _stamp_s(goal_info) -> float:
    return goal_info.stamp.sec + goal_info.stamp.nanosec * 1e-9


def _goals(msg) -> list:
    return [(_uuid_hex(s.goal_info), _stamp_s(s.goal_info), int(s.status)) for s in msg.status_list]


class FollowPathWatchdogNode(Node):
    def __init__(self) -> None:
        super().__init__("followpath_watchdog")
        for name, default in _DEFAULTS.items():
            self.declare_parameter(name, default)
        self._timeout = float(self.get_parameter("orphan_timeout_s").value)
        rate = float(self.get_parameter("check_rate_hz").value)

        self._state = fo.WatchState()
        self._nav: list = []
        self._follow: list = []
        self._follow_infos: dict = {}  # goal_id (hex) -> GoalInfo для запроса отмены

        # QoS статусов action: transient_local — поздний подписчик сразу получает последние статусы.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(GoalStatusArray, NAV_STATUS_TOPIC, self._on_nav, qos)
        self.create_subscription(GoalStatusArray, FOLLOW_STATUS_TOPIC, self._on_follow, qos)
        self._cancel_client = self.create_client(CancelGoal, FOLLOW_PATH_CANCEL_SRV)
        self._cmd_vel = self.create_publisher(Twist, NAV_CMD_VEL_TOPIC, 10)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self.get_logger().info(f"#4458 сторож сирот FollowPath: orphan_timeout_s={self._timeout}")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_nav(self, msg) -> None:
        self._nav = _goals(msg)

    def _on_follow(self, msg) -> None:
        self._follow = _goals(msg)
        self._follow_infos = {_uuid_hex(s.goal_info): s.goal_info for s in msg.status_list}

    def _tick(self) -> None:
        now = self._now()
        decision = fo.decide(self._state, now, self._nav, self._follow, self._timeout)
        if decision.cancel and not self._cancel_client.service_is_ready():
            self.get_logger().warning("#4458 сирота есть, но cancel_goal controller_server недоступен — повтор")
            return  # состояние не меняем: попробуем на следующем такте
        self._state = decision.state
        for goal_id in decision.cancel:
            self._halt(goal_id, now, decision)

    def _halt(self, goal_id: str, now: float, decision: fo.Decision) -> None:
        nav_status = fo.STATUS_NAMES.get(decision.nav_status, decision.nav_status)
        self.get_logger().warning(
            f"#4458 сирота FollowPath goal_id={goal_id} t={now:.3f}: NavigateToPose {decision.nav_goal} "
            f"в {nav_status} дольше {self._timeout} с — отмена follow_path и cmd_vel=0"
        )
        request = CancelGoal.Request()
        request.goal_info = self._follow_infos[goal_id]
        future = self._cancel_client.call_async(request)
        future.add_done_callback(lambda f, g=goal_id: self._on_cancel_done(g, f))
        for _ in range(ZERO_TWIST_REPEATS):
            self._cmd_vel.publish(Twist())

    def _on_cancel_done(self, goal_id: str, future) -> None:
        try:
            code = future.result().return_code
        except Exception as exc:  # noqa: BLE001 - отмена вторична, журнал важнее
            self.get_logger().error(f"#4458 cancel_goal goal_id={goal_id}: сбой вызова: {exc}")
            return
        self.get_logger().info(f"#4458 cancel_goal goal_id={goal_id}: return_code={code}")
        for _ in range(ZERO_TWIST_REPEATS):
            self._cmd_vel.publish(Twist())


def _raise_keyboard_interrupt(signum, frame) -> None:
    raise KeyboardInterrupt


def exit_on_sigterm() -> None:
    """SIGTERM (``docker stop``) — как Ctrl+C: spin прерывается, нода гасится в ``finally``."""
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)


def main(args=None) -> None:
    rclpy.init(args=args)
    exit_on_sigterm()
    node = FollowPathWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
