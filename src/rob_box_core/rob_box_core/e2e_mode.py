"""e2e_mode - один владелец режима e2e на роботе (#3904, аудит эпика #3683 §3.5).

Зачем. Боевая ``/data`` робота общая с живыми людьми мастерской (``speakers.db``, ``voice_memory.db``, галерея лиц).
За эпик e2e-данные трижды попадали туда, и каждый раз гард появлялся в одной ноде отдельно: свой ``e2e_mode`` у
``speaker_id_node`` и у ``mcp_server``, свои условия включения в харнессе и своё время жизни - в trap каждого акта.
Акт 3 марафона (наследует акт 2) пошёл на боевую БД дикторов: trap акта 2 снял режим, акт 3 его не включал.

Как теперь. Режим один, и знает его одна нода - **владелец** ``speaker_id_node`` (параметр :data:`OWNER_PARAM`;
это та нода, которая и так переключает БД дикторов и ставит метку ``SpeakerResult.e2e_db`` для галереи лиц).
Владелец публикует состояние в latched-топик :data:`TOPIC` и переиздаёт его раз в :data:`REPUBLISH_PERIOD_S`
(поздний подписчик на rmw_zenoh получает transient_local не всегда, см. ``provider_publisher``). Остальные ноды,
пишущие в ``/data``, - **последователи** (:class:`E2EModeFollower`): своего переключателя у них нет, они
применяют то, что сказал владелец, и показывают применённое в параметре :data:`APPLIED_PARAM` (только для чтения
харнессом: запись снаружи отклоняется).

Время жизни режима - марафон, а не акт. Харнесс, уходя из акта, за которым идёт наследующий, не выключает режим, а
ставит владельцу :data:`RELEASE_PARAM` (секунды): если следующий акт режим не подхватит (марафон оборвался, дедлайн),
владелец сам вернёт робота на боевые БД по таймеру. Наследующий акт подхватывает режим (``RELEASE_PARAM`` = 0),
последний акт выключает. Решение «держать ли» принимает раннер марафона, он один знает план актов.

rclpy и сообщения импортируются лениво: модуль живёт в ``rob_box_core`` (есть во всех образах Sound Pi), а тесты
идут без ROS.
"""

from __future__ import annotations

import math
import time
from typing import Any, Callable, Iterable, Optional

#: Latched-топик состояния режима (``std_msgs/Bool``): ``True`` - активные БД e2e, ``False`` - боевые.
TOPIC = "/e2e/mode"
#: Нода-владелец и её параметр-переключатель (единственный, который ставит харнесс).
OWNER_NODE = "/speaker_id_node"
OWNER_PARAM = "e2e_mode"
#: Параметр владельца: через сколько секунд снять режим, если его никто не подхватит; 0 - таймера нет.
RELEASE_PARAM = "e2e_release_after_s"
#: Параметр последователя: какой режим он реально применил (читает харнесс; запись снаружи отклоняется).
APPLIED_PARAM = "e2e_mode_applied"
REPUBLISH_PERIOD_S = 2.0
RELEASE_CHECK_PERIOD_S = 5.0


def _latched_qos() -> Any:
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: PLC0415

    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _bool_msg_type() -> Any:
    from std_msgs.msg import Bool  # noqa: PLC0415

    return Bool


def _result(ok: bool, reason: str = "") -> Any:
    from rcl_interfaces.msg import SetParametersResult  # noqa: PLC0415

    return SetParametersResult(successful=ok, reason=reason)


def release_after_s(raw: Any) -> Optional[float]:
    """Проверенное значение :data:`RELEASE_PARAM` (секунды, конечное, ``>= 0``) или ``None`` - отказ."""
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value) or value < 0:
        return None
    return value


class ReleaseDeadline:
    """Таймер «снять режим, если не подхватят»: чистая логика, часы подставляются."""

    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self._at: Optional[float] = None

    def arm(self, after_s: float) -> None:
        self._at = self._clock() + after_s if after_s > 0 else None

    def disarm(self) -> None:
        self._at = None

    @property
    def armed(self) -> bool:
        return self._at is not None

    def due(self) -> bool:
        return self._at is not None and self._clock() >= self._at


class E2EModeOwner:
    """Владелец режима на ноде ``speaker_id_node``: публикует состояние и снимает режим по таймеру.

    Само переключение БД дикторов делает нода (её ``e2e_mode`` в ``parameters_callback``); после успешного
    переключения она зовёт :meth:`mode_changed`. Снятие по таймеру идёт тем же путём - ``set_parameters`` ноды,
    поэтому у режима одна дверь и одна строка в логе.
    """

    def __init__(self, node: Any, is_active: Callable[[], bool], clock: Callable[[], float] = time.monotonic) -> None:
        self._node = node
        self._is_active = is_active
        self._deadline = ReleaseDeadline(clock)
        node.declare_parameter(RELEASE_PARAM, 0.0)
        self._pub = node.create_publisher(_bool_msg_type(), TOPIC, _latched_qos())
        node.add_on_set_parameters_callback(self.on_params)
        node.create_timer(REPUBLISH_PERIOD_S, self.publish)
        node.create_timer(RELEASE_CHECK_PERIOD_S, self.check_release)
        self.publish()

    @property
    def release_armed(self) -> bool:
        return self._deadline.armed

    def publish(self) -> None:
        msg = _bool_msg_type()()
        msg.data = bool(self._is_active())
        self._pub.publish(msg)

    def mode_changed(self) -> None:
        """Нода переключила БД: новый режим сразу в топик; любой взведённый таймер снятия больше не действует."""
        self._deadline.disarm()
        self.publish()

    def on_params(self, params: Iterable[Any]) -> Any:
        for param in params:
            if param.name != RELEASE_PARAM:
                continue
            after = release_after_s(param.value)
            if after is None:
                return _result(False, f"{RELEASE_PARAM}: ожидается число секунд >= 0, получено {param.value!r}")
            if after > 0 and not self._is_active():
                return _result(False, f"{RELEASE_PARAM}={after}: режим e2e выключен, держать нечего")
            was_armed = self._deadline.armed
            self._deadline.arm(after)
            if after > 0:
                self._node.get_logger().warning(
                    f"🧪 e2e-режим: снимется через {after:g} с, если следующий акт его не подхватит")
            elif was_armed:
                self._node.get_logger().warning("🧪 e2e-режим подхвачен: таймер снятия отменён, режим держит акт")
        return _result(True)

    def check_release(self) -> None:
        if not self._deadline.due():
            return
        self._deadline.disarm()
        if not self._is_active():
            return
        self._node.get_logger().warning("🧪 e2e-режим: никто не подхватил - возвращаю робота на боевые БД (таймер)")
        from rclpy.parameter import Parameter  # noqa: PLC0415

        self._node.set_parameters([Parameter(OWNER_PARAM, value=False), Parameter(RELEASE_PARAM, value=0.0)])


class E2EModeFollower:
    """Последователь: применяет режим владельца своим ``apply(enabled) -> bool`` и показывает применённое.

    Своего переключателя нет - это и есть смысл «одного владельца». Отказ ``apply`` (например, бэкенд памяти без
    e2e-изоляции) оставляет :data:`APPLIED_PARAM` прежним: харнесс, прочитав его, увидит расхождение и не прогонит акт.
    """

    def __init__(self, node: Any, apply: Callable[[bool], bool]) -> None:
        self._node = node
        self._apply = apply
        self._applied = False
        self._writing = False
        node.declare_parameter(APPLIED_PARAM, False)
        node.add_on_set_parameters_callback(self.on_params)
        node.create_subscription(_bool_msg_type(), TOPIC, self.on_mode, _latched_qos())

    @property
    def applied(self) -> bool:
        return self._applied

    def on_params(self, params: Iterable[Any]) -> Any:
        if not self._writing and any(p.name == APPLIED_PARAM for p in params):
            return _result(False, f"{APPLIED_PARAM} только для чтения: режим e2e ставит {OWNER_NODE} {OWNER_PARAM}")
        return _result(True)

    def on_mode(self, msg: Any) -> None:
        wanted = bool(msg.data)
        if wanted == self._applied or not self._apply(wanted):
            return
        self._applied = wanted
        from rclpy.parameter import Parameter  # noqa: PLC0415

        self._writing = True
        try:
            self._node.set_parameters([Parameter(APPLIED_PARAM, value=wanted)])
        finally:
            self._writing = False


__all__ = [
    "APPLIED_PARAM", "E2EModeFollower", "E2EModeOwner", "OWNER_NODE", "OWNER_PARAM", "RELEASE_PARAM",
    "ReleaseDeadline", "TOPIC", "release_after_s",
]
