"""Mock rclpy + std_srvs + std_msgs для rob_box_supervisor (AV-6).

CI-окружение (Debian без ROS) не имеет rclpy. Этот conftest
подсовывает минимальные fake-модули ДО импорта supervisor_node,
чтобы можно было протестировать:

- создание ``AvatarSupervisor`` (Node-инициализация),
- публикацию ``/avatar/state`` (latched, transient_local),
- сервисы ``AcquireFloor`` / ``ReleaseFloor`` / ``SetAvatarMode``
  (должны отвечать ``success=true / applied=false / reason=…``),
- heartbeat трип → ``/avatar/state`` обновляется.

Паттерн скопирован из ``src/rob_box_voice/test/unit/greeting/conftest.py``
(см. rob_box_voice/test_startup_greeting_node.py), но расширен под
rclpy.qos, std_srvs.srv и Zenoh-сессию.
"""

from __future__ import annotations

import sys
import types
from typing import Any
from unittest.mock import MagicMock


def _install_ros_mocks() -> None:
    # ── rclpy package + sub-modules ───────────────────────────────────
    mock_rclpy = MagicMock()
    mock_rclpy.ok = MagicMock(return_value=True)
    mock_rclpy.init = MagicMock(return_value=None)
    mock_rclpy.shutdown = MagicMock(return_value=None)
    mock_rclpy.spin = MagicMock(return_value=None)

    # ── FakeNode ──────────────────────────────────────────────────────
    class FakePublisher:
        def __init__(self, topic: str, msg_type: Any) -> None:
            self.topic = topic
            self.msg_type = msg_type
            self.published: list[Any] = []

        def publish(self, msg: Any) -> None:
            self.published.append(msg)

    class FakeSubscription:
        def __init__(self, topic: str, callback: Any) -> None:
            self.topic = topic
            self.callback = callback

    class FakeTimer:
        def __init__(self, period: float, callback: Any) -> None:
            self.period = period
            self.callback = callback
            self.cancel = MagicMock()

    class FakeService:
        """Подмена create_service: хранит (name, callback)."""

        def __init__(self, name: str, srv_type: Any, callback: Any) -> None:
            self.name = name
            self.srv_type = srv_type
            self.callback = callback
            self.calls: list[Any] = []

    class FakeNode:
        """Заглушка rclpy.node.Node."""

        def __init__(self, name: str, **kwargs: Any) -> None:
            self._name = name
            self._logger = MagicMock()
            self._parameters: dict[str, Any] = {}
            self._publishers: dict[str, FakePublisher] = {}
            self._subscriptions: list[FakeSubscription] = []
            self._timers: list[FakeTimer] = []
            self._services: list[FakeService] = []

        # ── name / namespace ────────────────────────────────────────────
        def get_name(self) -> str:
            return self._name

        def get_namespace(self) -> str:
            return ""

        # ── logger ─────────────────────────────────────────────────────
        def get_logger(self) -> MagicMock:
            return self._logger

        # ── parameters ─────────────────────────────────────────────────
        def declare_parameter(self, name: str, default: Any = None) -> Any:
            if name not in self._parameters:
                self._parameters[name] = default
            return default

        def get_parameter(self, name: str) -> MagicMock:
            p = MagicMock()
            p.value = self._parameters.get(name)
            return p

        def has_parameter(self, name: str) -> bool:
            return name in self._parameters

        # ── pubs / subs / timers / services ─────────────────────────────
        def create_publisher(self, msg_type: Any, topic: str, qos: Any = 10) -> FakePublisher:
            pub = FakePublisher(topic, msg_type)
            self._publishers[topic] = pub
            return pub

        def create_subscription(self, msg_type: Any, topic: str, callback: Any, qos: int = 10) -> FakeSubscription:
            sub = FakeSubscription(topic, callback)
            self._subscriptions.append(sub)
            return sub

        def create_timer(self, period: float, callback: Any) -> FakeTimer:
            t = FakeTimer(period, callback)
            self._timers.append(t)
            return t

        def create_service(self, srv_type: Any, name: str, callback: Any) -> FakeService:
            svc = FakeService(name, srv_type, callback)
            self._services.append(svc)
            return svc

        def create_client(self, srv_type: Any, name: str) -> MagicMock:
            """Подмена create_client (SetParameters → dialogue_node)."""
            client = MagicMock()
            client.srv_type = srv_type
            client.srv_name = name
            return client

        def destroy_node(self) -> None:
            for t in self._timers:
                try:
                    t.cancel()
                except Exception:
                    pass
            self._timers.clear()
            self._publishers.clear()
            self._subscriptions.clear()
            self._services.clear()

    mock_rclpy_node = types.SimpleNamespace(Node=FakeNode)

    # ── rclpy.qos (DurabilityPolicy, ReliabilityPolicy, QoSProfile) ────
    class _DurabilityPolicy:
        TRANSIENT_LOCAL = "transient_local"
        VOLATILE = "volatile"

    class _ReliabilityPolicy:
        RELIABLE = "reliable"
        BEST_EFFORT = "best_effort"

    class _QoSProfile:
        def __init__(self, depth: int = 10, **kwargs: Any) -> None:
            self.depth = depth
            for k, v in kwargs.items():
                setattr(self, k, v)

    mock_rclpy_qos = types.SimpleNamespace(
        DurabilityPolicy=_DurabilityPolicy,
        ReliabilityPolicy=_ReliabilityPolicy,
        QoSProfile=_QoSProfile,
    )

    # ── std_msgs.msg.String ───────────────────────────────────────────
    class FakeStringMsg:
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            self.data = ""

    class FakeBoolMsg:
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            self.data = False

    mock_std_msgs_msg = types.SimpleNamespace(
        String=FakeStringMsg,
        Bool=FakeBoolMsg,
    )
    mock_std_msgs = types.SimpleNamespace(msg=mock_std_msgs_msg)

    # ── std_srvs.srv.Trigger ──────────────────────────────────────────
    class FakeTriggerRequest:
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            self.data = ""  # Trigger.Request пустой, но оставим поле

    class FakeTriggerResponse:
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            self.success = False
            self.message = ""

    class FakeTrigger:
        Request = FakeTriggerRequest
        Response = FakeTriggerResponse

    mock_std_srvs_srv = types.SimpleNamespace(Trigger=FakeTrigger)
    mock_std_srvs = types.SimpleNamespace(srv=mock_std_srvs_srv)

    # ── register all mocks ────────────────────────────────────────────
    mocks = {
        "rclpy": mock_rclpy,
        "rclpy.node": mock_rclpy_node,
        "rclpy.qos": mock_rclpy_qos,
        "std_msgs": mock_std_msgs,
        "std_msgs.msg": mock_std_msgs_msg,
        "std_srvs": mock_std_srvs,
        "std_srvs.srv": mock_std_srvs_srv,
    }
    for name, mock in mocks.items():
        sys.modules.setdefault(name, mock)


_install_ros_mocks()
