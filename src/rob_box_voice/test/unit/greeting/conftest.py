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
import sys as _sys
from pathlib import Path as _Path

# Shared ROS2 stubs — see test/ros_stubs.py. These per-directory stub sets
# are installed with ``sys.modules.setdefault``, so in a full run whichever
# directory pytest reaches first wins. Using the shared ``rclpy.qos`` here
# means the winner no longer matters: every directory publishes the same
# policy names and the same kwargs-recording ``QoSProfile``.
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from ros_stubs import (  # noqa: E402
    FakeNode as _FakeNode,
    qos_stub as _qos_stub,
)

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
    # FakeNode is shared (test/ros_stubs.py). ``rclpy.node`` is installed
    # with ``sys.modules.setdefault``, so only the first conftest to load
    # supplies the base class for every directory — four private copies
    # meant the winner decided which assertions could pass.
    FakeNode = _FakeNode

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
        "rclpy.qos": _qos_stub(),
        "std_msgs": mock_std_msgs,
        "std_msgs.msg": mock_std_msgs_msg,
    }
    for name, mock in mocks.items():
        sys.modules.setdefault(name, mock)


_install_ros_mocks()
