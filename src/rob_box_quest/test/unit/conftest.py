"""Общие фикстуры unit-тестов rob_box_quest.

``quest_node_mod`` — ``rob_box_quest.quest_node``, импортированный поверх
ROS-заглушек, если настоящих ROS-модулей в окружении нет.

Зачем (issue #2135). ``quest_node`` импортирует ``rclpy``,
``audio_common_msgs`` и остальные ROS-пакеты на верхнем уровне, поэтому все
тесты ``QuestBridge`` исторически написаны через
``pytest.importorskip("geometry_msgs")`` и на dev-машине без ROS
пропускаются ЦЕЛИКОМ. Ровно из-за этого однажды по ошибке закрыли карточку
#1992: «тесты зелёные» на деле означало «78 тестов skipped».

Контракт фикстуры:

* заглушка ставится ТОЛЬКО для модуля, которого в ``sys.modules`` нет — в
  Docker-образе с ROS тесты гоняют настоящие сообщения;
* заглушки и импортированный поверх них ``quest_node`` снимаются в
  teardown, чтобы соседние файлы с ``importorskip`` продолжали честно
  скипаться, а не подхватывали фальшивый ROS.

Заглушки покрывают ровно то, что нужно ``QuestBridge``: конструкторы
сообщений и базовый ``Node``. Всё, что использует настоящую ROS-семантику
(QoS, executors, сервисы), в этих тестах не участвует.
"""

import importlib
import sys
import types

import pytest

_ROS_MODULES = (
    "rclpy",
    "rclpy.executors",
    "rclpy.node",
    "rclpy.qos",
    "audio_common_msgs",
    "audio_common_msgs.msg",
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav_msgs",
    "nav_msgs.msg",
    "sensor_msgs",
    "sensor_msgs.msg",
    "std_msgs",
    "std_msgs.msg",
    "rob_box_core",
    "rob_box_core.avatar_command",
)


class _StubMsg:
    def __init__(self, *args, **kwargs) -> None:
        pass


class _StubAudioData:
    def __init__(self) -> None:
        self.data: list = []


class _StubVector3:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _StubTwist:
    def __init__(self) -> None:
        self.linear = _StubVector3()
        self.angular = _StubVector3()


class _StubString:
    def __init__(self) -> None:
        self.data = ""


class _StubNode:
    def __init__(self, *args, **kwargs) -> None:
        pass


_STUB_ATTRS = {
    "rclpy.executors": {
        "ExternalShutdownException": type("ExternalShutdownException", (Exception,), {})
    },
    "rclpy.node": {"Node": _StubNode},
    "rclpy.qos": {
        "DurabilityPolicy": object,
        "HistoryPolicy": object,
        "QoSProfile": object,
        "ReliabilityPolicy": object,
    },
    "audio_common_msgs.msg": {"AudioData": _StubAudioData},
    "geometry_msgs.msg": {"Twist": _StubTwist},
    "nav_msgs.msg": {"OccupancyGrid": _StubMsg, "Odometry": _StubMsg},
    "sensor_msgs.msg": {"CompressedImage": _StubMsg, "LaserScan": _StubMsg},
    "std_msgs.msg": {"String": _StubString},
    "rob_box_core.avatar_command": {
        "AVATAR_COMMAND_RESULT_TOPIC": "/avatar/command/result"
    },
}


@pytest.fixture()
def quest_node_mod():
    """``rob_box_quest.quest_node`` (поверх ROS-заглушек, если ROS нет)."""
    installed: list[str] = []
    for name in _ROS_MODULES:
        if name in sys.modules:
            continue
        module = types.ModuleType(name)
        for attr, value in _STUB_ATTRS.get(name, {}).items():
            setattr(module, attr, value)
        sys.modules[name] = module
        installed.append(name)

    cached = sys.modules.pop("rob_box_quest.quest_node", None) if installed else None
    try:
        yield importlib.import_module("rob_box_quest.quest_node")
    finally:
        if installed:
            sys.modules.pop("rob_box_quest.quest_node", None)
            for name in installed:
                sys.modules.pop(name, None)
            if cached is not None:
                sys.modules["rob_box_quest.quest_node"] = cached
