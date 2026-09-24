# Copyright 2026 rob_box_project contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Изоляция module-level стабов ``sys.modules`` между тест-файлами.

Несколько тест-файлов (test_context_aggregator.py, test_perception_bridge.py,
unit/test_vision_events_aggregator.py, unit/test_vision_face_log_stats.py)
на уровне модуля кладут в ``sys.modules`` самодельные стабы ``rclpy``,
``std_msgs``, ``rob_box_perception_msgs`` и т.п., чтобы импортировать
тестируемую ноду без ROS2. pytest импортирует ВСЕ тест-файлы на этапе
коллекции, до запуска первого теста, поэтому такой стаб без отката живёт
до конца сессии и ломает файлы, которым нужен настоящий модуль:

* ``test_launch_factory.py`` — ``launch_ros`` делает ``import rclpy.parameter``
  и получает ``'rclpy' is not a package``;
* ``test_vision_hailo_publishes.py`` — берёт стаб ``rob_box_perception_msgs``
  без ``VisionEvent``;
* ``unit/test_vision_events_aggregator.py`` — ``setdefault`` не ставит его
  собственный стаб с ``VisionEvent``, потому что место уже занято чужим.

Хук ниже после коллекции каждого тест-модуля откатывает в ``sys.modules``
всё, что было подменено или добавлено стабом. Стаб отличаем по отсутствию
``__spec__``: у любого модуля, загруженного import-системой (включая
builtin и namespace-пакеты), он есть, у ``types.ModuleType(...)``,
созданного руками, — ``None``. Настоящие модули, импортированные во время
коллекции, остаются в кэше: повторный импорт C-расширений (numpy, cv2)
в одном процессе небезопасен.

Тестируемый модуль, импортированный поверх стабов, продолжает держать
ссылки на них (``from rclpy.node import Node`` уже связан), так что
тесты самого файла работают как раньше.
"""

import sys

import pytest

_MISSING = object()


def _is_stub(module):
    return module is None or getattr(module, '__spec__', None) is None


@pytest.hookimpl(hookwrapper=True)
def pytest_make_collect_report(collector):
    if not isinstance(collector, pytest.Module):
        yield
        return
    before = dict(sys.modules)
    yield
    for name, module in list(sys.modules.items()):
        if before.get(name, _MISSING) is module or not _is_stub(module):
            continue
        if name in before:
            sys.modules[name] = before[name]
        else:
            del sys.modules[name]
