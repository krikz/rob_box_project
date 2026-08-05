"""
conftest.py — Mock ROS2/rclpy для test_startup_greeting_node.py.

Pytest автоматически подхватывает этот conftest.py для тестов в той же
директории. Реальный rclpy не установлен в этом окружении (Debian, без
ROS), поэтому startup_greeting_node импортируется через заглушки.

Подмены:
- rclpy.node.Node — FakeNode с параметрами-по-default из declare_parameter
  и подписчиками, которые можно опрашивать через get_subscriptions_info_by_topic.
- std_msgs.msg.String — конструктор с .data=... как в реальном std_msgs.
- rclpy.init / shutdown / ok — no-op MagicMock-и, чтобы setUpClass/tearDown
  не падали.
"""

from __future__ import annotations

import sys
import types
from unittest.mock import MagicMock


def _install_ros_mocks() -> None:
    # rclpy.ok / init / shutdown / spin — MagicMock (без побочек).
    mock_rclpy = MagicMock()
    mock_rclpy.ok = MagicMock(return_value=True)
    mock_rclpy.init = MagicMock(return_value=None)
    mock_rclpy.shutdown = MagicMock(return_value=None)
    mock_rclpy.spin = MagicMock(return_value=None)

    # rclpy.node.Node — минимальный FakeNode.
    class FakeNode:
        """Заглушка rclpy.node.Node: параметры из declare_parameter,
        publishers/subscriptions/timer — реальные объекты, чтобы тесты
        могли подменить publish и опросить, что создано."""

        def __init__(self, name: str, **kwargs) -> None:
            self._name = name
            self._logger = MagicMock()
            self._parameters: dict[str, object] = {}
            self._publishers: list[tuple[str, object]] = []  # (topic, fake_pub)
            self._subscribers: list[tuple[str, object]] = []  # (topic, fake_sub)
            self._timers: list[object] = []

        # ── name ────────────────────────────────────────────────────────────
        def get_name(self) -> str:
            return self._name

        def get_namespace(self) -> str:
            return ""

        # ── logger ──────────────────────────────────────────────────────────
        def get_logger(self):
            return self._logger

        # ── parameters ──────────────────────────────────────────────────────
        def declare_parameter(self, name: str, default=None):
            # В реальном ROS параметр уже объявлен — мы просто сохраняем default.
            if name not in self._parameters:
                self._parameters[name] = default
            return default

        def get_parameter(self, name: str):
            p = MagicMock()
            p.value = self._parameters.get(name)
            return p

        def has_parameter(self, name: str) -> bool:
            return name in self._parameters

        def set_parameters_atomically(self, params):
            return MagicMock(successful=True)

        # ── publishers / subscribers / timers ───────────────────────────────
        def create_publisher(self, msg_type, topic: str, qos: int = 10):
            fake = MagicMock()
            fake.topic = topic
            fake.msg_type = msg_type
            self._publishers.append((topic, fake))
            return fake

        def create_subscription(self, msg_type, topic: str, callback, qos: int = 10):
            fake = MagicMock()
            fake.topic = topic
            fake.callback = callback
            self._subscribers.append((topic, fake))
            return fake

        def create_timer(self, period, callback):
            fake = MagicMock()
            fake.period = period
            fake.callback = callback
            fake.cancel = MagicMock()
            self._timers.append(fake)
            return fake

        def destroy_node(self):
            self._publishers.clear()
            self._subscribers.clear()
            for t in self._timers:
                try:
                    t.cancel()
                except Exception:
                    pass
            self._timers.clear()

        # ── introspection API (используется startup_greeting_node) ──────────
        def get_publisher_names_and_types_by_node(self, name: str, namespace: str):
            return [(topic, ["std_msgs/String"]) for topic, _ in self._publishers]

        def get_subscriber_names_and_types_by_node(self, name: str, namespace: str):
            return [(topic, ["std_msgs/String"]) for topic, _ in self._subscribers]

        def get_subscriptions_info_by_topic(self, topic: str):
            # Возвращает список словарей-инфо про подписчиков на topic.
            # Если есть хоть один — стек готов.
            matches = [s for t, s in self._subscribers if t == topic]
            return [{"name": "fake", "type": "std_msgs/String"} for _ in matches]


    mock_rclpy_node = MagicMock()
    mock_rclpy_node.Node = FakeNode

    # std_msgs.msg.String — класс с конструктором .data=...
    class FakeString:
        def __init__(self, *args, **kwargs) -> None:
            self.data = ""

    mock_std_msgs = MagicMock()
    mock_std_msgs_msg = MagicMock()
    mock_std_msgs_msg.String = FakeString

    mocks = {
        "rclpy": mock_rclpy,
        "rclpy.node": mock_rclpy_node,
        "rclpy.callback_groups": types.SimpleNamespace(
            ReentrantCallbackGroup=type("ReentrantCallbackGroup", (), {}),
        ),
        "rclpy.qos": types.SimpleNamespace(
            HistoryPolicy=types.SimpleNamespace(KEEP_LAST="KEEP_LAST"),
            ReliabilityPolicy=types.SimpleNamespace(RELIABLE="RELIABLE"),
            QoSProfile=lambda *a, **kw: MagicMock(),
        ),
        "std_msgs": mock_std_msgs,
        "std_msgs.msg": mock_std_msgs_msg,
    }
    for name, mock in mocks.items():
        sys.modules.setdefault(name, mock)


_install_ros_mocks()
