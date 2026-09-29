"""Заглушки ROS для тестов музыки, которые не утекают в соседние файлы.

Тесты музыки импортируют ``rob_box_mcp_tools.tools.*`` без ROS: вместо
``rclpy``, ``std_msgs``... кладут ``MagicMock`` в ``sys.modules``. Раньше
заглушки оставались там на весь процесс, а ``MagicMock`` — не пакет: любой
тест, собранный позже, падал на ленивом ``from rclpy.callback_groups
import ...`` («'rclpy' is not a package»). CI этого не видит — там каждый
файл идёт отдельным процессом (scripts/testing/run_mcp_tools_unit_tests.sh).

Код инструментов импортирует ROS лениво, в момент вызова (``base.py``,
``llm_adapter.py``, ``tools/dialogue.py``), поэтому заглушки нужны и при
импорте модуля (сборка), и пока идут его тесты. Пользоваться так::

    _ros = RosStubs()
    with _ros:
        from rob_box_mcp_tools.tools.music import ComposeMusicTool
    _ros_stubs = _ros.fixture()

Убираются и возвращаются только сами заглушки. Настоящие модули,
импортированные под ними, остаются в ``sys.modules`` одной копией: иначе
``patch("rob_box_mcp_tools.tools.music.X")`` попадал бы в другой объект
модуля, чем импортированный тестом.
"""

import sys
from typing import Dict, Iterable
from unittest.mock import MagicMock

import pytest

ROS_MODULES = (
    "rclpy",
    "rclpy.node",
    "rclpy.action",
    "rclpy.qos",
    "std_msgs",
    "std_msgs.msg",
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav2_msgs",
    "nav2_msgs.action",
    "action_msgs",
    "action_msgs.srv",
    "action_msgs.msg",
)


class RosStubs:
    """``MagicMock`` на месте недостающих модулей ROS — только на время."""

    def __init__(self, names: Iterable[str] = ROS_MODULES):
        self._names = tuple(names)
        self._stubs: Dict[str, MagicMock] = {}

    def __enter__(self) -> "RosStubs":
        # Уже загруженное (настоящий ROS или чужие заглушки) не трогаем —
        # как раньше делал ``setdefault``.
        for name in self._names:
            if name not in sys.modules:
                self._stubs[name] = sys.modules[name] = MagicMock()
        return self

    def __exit__(self, *exc) -> None:
        for name, stub in self._stubs.items():
            if sys.modules.get(name) is stub:
                del sys.modules[name]

    def fixture(self):
        """Autouse-фикстура модуля: те же заглушки на время его тестов."""

        @pytest.fixture(scope="module", autouse=True)
        def _ros_stubs():
            saved = {n: sys.modules[n] for n in self._stubs if n in sys.modules}
            sys.modules.update(self._stubs)
            try:
                yield
            finally:
                for name in self._stubs:
                    if name in saved:
                        sys.modules[name] = saved[name]
                    else:
                        sys.modules.pop(name, None)

        return _ros_stubs
