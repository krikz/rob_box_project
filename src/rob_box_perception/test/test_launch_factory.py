"""Smoke-тест для launch_factory.make_hailo_node_launch (issue #2658).

Цель — покрыть acceptance-критерии #2658:

- factory возвращает корректный ``launch.LaunchDescription``,
- набор launch-аргументов соответствует контракту vision_hailo /
  vision_face (разные дефолты ``confidence_threshold``, наличие/отсутствие
  ``nms_iou_threshold``, наличие/отсутствие legacy ``input_topic``),
- DRY-инвариант: общая launch-конструкция (11 ``DeclareLaunchArgument`` +
  ``OpaqueFunction`` + ``Node``) существует ровно в одном месте —
  ``rob_box_perception/launch_factory.py``,
- smoke-проверка ``generate_launch_description`` обоих шим-файлов
  (``launch/vision_hailo.launch.py``, ``launch/vision_face.launch.py``)
  возвращает ``LaunchDescription`` (без ROS-runtime — только AST/import).

Тест НЕ требует ``ros2 launch``/runtime — только ``launch`` (Python-пакет,
ставится через ``apt install ros-humble-launch`` или colcon). Если
``launch`` недоступен, тест скипается с понятной диагностикой.
"""

from __future__ import annotations

import importlib
import importlib.util
import sys
from pathlib import Path
from typing import List

import pytest

# Статические проверки DRY-инварианта (Acceptance #1, #4 из issue #2658)
# вынесены в ``test_launch_factory_static.py``, чтобы проходить без
# установленного ROS2-пакета ``launch``. Этот файл — runtime-тесты:
# проверяют, что ``make_hailo_node_launch`` действительно возвращает
# корректный ``LaunchDescription`` с правильным набором аргументов.


# __file__ = src/rob_box_perception/test/test_launch_factory.py.
# .parents[1] = src/rob_box_perception/  (где лежат launch/ и rob_box_perception/).
PKG_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_DIR = PKG_ROOT / 'launch'
# parents[2] = src/ — корень src tree, нужен для импорта rob_box_perception.
SRC_ROOT = Path(__file__).resolve().parents[2]
REPO_ROOT = PKG_ROOT
FACTORY_MODULE = 'rob_box_perception.launch_factory'


def _import_module_from_path(module_name: str, path: Path):
    """Импортировать модуль по абсолютному пути без установки в sys.path."""
    spec = importlib.util.spec_from_file_location(module_name, path)
    assert spec is not None and spec.loader is not None, (
        f'cannot load spec for {module_name} from {path}'
    )
    module = importlib.util.module_from_spec(spec)
    # Регистрируем в sys.modules, чтобы внутри работали относительные импорты.
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


# Runtime-инвариант: общая launch-конструкция (11 ``DeclareLaunchArgument`` +
# ``OpaqueFunction`` + ``Node``) существует ровно в одном месте —
# ``rob_box_perception/launch_factory.py``. Это статическая проверка AST,
# которая живёт здесь как ``test_factory_module_imports`` (импорт factory
# без вызова).


# ---- Ниже — runtime-тесты; требуют установленный ``launch`` Python-пакет. ----
# ``launch`` — это ROS2 пакет (apt: ros-${ROS_DISTRO}-launch). В CI-образе
# ghcr.io/krikz/rob-box-ci:humble он есть, в локальной dev-среде — нет.
# Используем importorskip + дополнительную проверку ``hasattr`` на случай
# namespace-package-заглушки (PEP 420) с тем же именем.

_launch = pytest.importorskip(
    'launch', reason='launch не установлен (ros-humble-launch или pip install).'
)
if not hasattr(_launch, 'LaunchDescription'):
    pytest.skip(
        'launch установлен, но без LaunchDescription (заглушка/namespase-pkg).',
        allow_module_level=True,
    )
launch = _launch


def _load_vision_face_launch():
    return _import_module_from_path(
        'vision_face_launch_smoke',
        LAUNCH_DIR / 'vision_face.launch.py',
    )


def _load_vision_hailo_launch():
    return _import_module_from_path(
        'vision_hailo_launch_smoke',
        LAUNCH_DIR / 'vision_hailo.launch.py',
    )


def _load_factory():
    """Импортировать rob_box_perception.launch_factory через sys.path хак.

    ``rob_box_perception`` — обычный Python-пакет. Добавляем src/
    в sys.path чтобы import сработал даже без colcon-сборки.
    """
    if str(SRC_ROOT) not in sys.path:
        sys.path.insert(0, str(SRC_ROOT))
    return importlib.import_module(FACTORY_MODULE)


def test_factory_module_imports():
    """factory живёт в rob_box_perception (Python-пакет), не в launch/."""
    factory = _load_factory()
    assert hasattr(factory, 'make_hailo_node_launch'), (
        'launch_factory.make_hailo_node_launch не найден '
        '(issue #2658 Acceptance #2).'
    )


def _declared_arg_names(launch_description) -> List[str]:
    """Извлечь имена DeclareLaunchArgument из LaunchDescription."""
    from launch.actions import DeclareLaunchArgument

    names: List[str] = []
    for entity in launch_description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            names.append(entity.name)
    return names


def test_make_hailo_node_launch_face_includes_nms_no_legacy_input():
    """vision_face: nms_iou_threshold есть, input_topic — нет."""
    factory = _load_factory()
    ld = factory.make_hailo_node_launch(
        executable='vision_face',
        confidence_threshold_default='0.6',
        include_nms_iou=True,
        include_input_topic=False,
        preflight_prefix='vision_face',
    )
    assert isinstance(ld, launch.LaunchDescription)
    names = _declared_arg_names(ld)
    assert 'nms_iou_threshold' in names, (
        'vision_face должен объявлять nms_iou_threshold.'
    )
    assert 'input_topic' not in names, (
        'vision_face НЕ должен объявлять legacy input_topic.'
    )
    assert 'confidence_threshold' in names


def test_make_hailo_node_launch_hailo_includes_legacy_input_no_nms():
    """vision_hailo: input_topic есть (back-compat), nms — нет."""
    factory = _load_factory()
    ld = factory.make_hailo_node_launch(
        executable='vision_hailo',
        confidence_threshold_default='0.5',
        include_nms_iou=False,
        include_input_topic=True,
        preflight_prefix='vision_hailo',
    )
    assert isinstance(ld, launch.LaunchDescription)
    names = _declared_arg_names(ld)
    assert 'input_topic' in names, (
        'vision_hailo должен сохранять legacy input_topic (ADR-0104).'
    )
    assert 'nms_iou_threshold' not in names, (
        'vision_hailo НЕ должен объявлять nms_iou_threshold (yolo, '
        'NMS внутри HEF).'
    )


def test_make_hailo_node_launch_default_confidence_override():
    """confidence_threshold_default пробрасывается в DeclareLaunchArgument."""
    factory = _load_factory()
    from launch.actions import DeclareLaunchArgument

    ld = factory.make_hailo_node_launch(
        executable='vision_face',
        confidence_threshold_default='0.42',
        include_nms_iou=True,
        preflight_prefix='vision_face',
    )
    args_by_name = {
        e.name: e for e in ld.entities if isinstance(e, DeclareLaunchArgument)
    }
    assert args_by_name['confidence_threshold'].default_value == '0.42', (
        'default_value confidence_threshold должен быть '
        'confidence_threshold_default (0.42), не hardcoded.'
    )


def test_make_hailo_node_launch_node_executable_and_name_match():
    """Node(executable=name) и Node(name=name) — один executable-параметр."""
    factory = _load_factory()
    from launch_ros.actions import Node

    ld = factory.make_hailo_node_launch(
        executable='vision_hailo',
        confidence_threshold_default='0.5',
        include_nms_iou=False,
        include_input_topic=True,
        preflight_prefix='vision_hailo',
    )
    nodes = [e for e in ld.entities if isinstance(e, Node)]
    assert len(nodes) == 1
    node = nodes[0]
    assert node.executable == 'vision_hailo'
    assert node.name == 'vision_hailo'
    assert node.package == 'rob_box_perception'
    assert node.output == 'screen'
    assert isinstance(node.parameters, list) and len(node.parameters) == 1


def test_vision_face_launch_smoke_returns_launch_description():
    """vision_face.launch.py → generate_launch_description() OK."""
    mod = _load_vision_face_launch()
    ld = mod.generate_launch_description()
    assert isinstance(ld, launch.LaunchDescription)
    names = _declared_arg_names(ld)
    assert 'confidence_threshold' in names
    assert 'nms_iou_threshold' in names
    assert 'input_topic' not in names


def test_vision_hailo_launch_smoke_returns_launch_description():
    """vision_hailo.launch.py → generate_launch_description() OK."""
    mod = _load_vision_hailo_launch()
    ld = mod.generate_launch_description()
    assert isinstance(ld, launch.LaunchDescription)
    names = _declared_arg_names(ld)
    assert 'confidence_threshold' in names
    assert 'input_topic' in names
    assert 'nms_iou_threshold' not in names
