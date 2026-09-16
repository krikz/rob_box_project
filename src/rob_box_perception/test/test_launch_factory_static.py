"""Статические проверки DRY-инварианта launch_factory (issue #2658, без runtime).

Цель — покрыть acceptance #1 и #4 из issue #2658 **без** зависимости от
ROS2-пакета ``launch`` (apt: ros-${ROS_DISTRO}-launch). Эти тесты должны
проходить на любой dev-машине, в любом CI, и сразу сигналить о регрессии
если кто-то добавит новый ``DeclareLaunchArgument`` в ``launch/``, а не
в общую ``rob_box_perception/launch_factory.py``.

Что проверяется:
- Acceptance #4: ``grep DeclareLaunchArgument src/rob_box_perception/launch/``
  возвращает 0 настоящих вхождений (AST-проверка, не regex по docstring).
- Acceptance #1: каждый ``launch/*.launch.py`` — шим ≤40 значимых строк.

Runtime-проверки (``make_hailo_node_launch`` возвращает ``LaunchDescription``,
Node(executable=name) и т.д.) лежат в отдельном файле
``test_launch_factory.py``, который скипается без установленного ROS2
пакета ``launch``.
"""

from __future__ import annotations

import ast
from pathlib import Path
from typing import List

import pytest


# __file__ = src/rob_box_perception/test/test_launch_factory_static.py.
# .parents[1] = src/rob_box_perception/.
PKG_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_DIR = PKG_ROOT / 'launch'


def _iter_launch_files() -> List[Path]:
    return sorted(LAUNCH_DIR.glob('*.launch.py'))


def _count_real_declare_launch_argument(path: Path) -> int:
    """Подсчитать настоящие вызовы ``DeclareLaunchArgument(...)`` в файле.

    Docstring-упоминания строкой "DeclareLaunchArgument × 9" не считаются —
    парсим AST и ищем только ``ast.Call`` с ``func.id == 'DeclareLaunchArgument'``
    или ``args.append(DeclareLaunchArgument(...))``.
    """
    tree = ast.parse(path.read_text())
    total = 0
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        target = None
        if isinstance(func, ast.Name):
            target = func.id
        elif isinstance(func, ast.Attribute):
            target = func.attr
        if target == 'DeclareLaunchArgument':
            total += 1
    return total


def _count_code_constructs(path: Path) -> int:
    """Подсчитать значимые конструкции верхнего уровня (без docstring)."""
    tree = ast.parse(path.read_text())
    count = 0
    for node in tree.body:
        if isinstance(node, (ast.Import, ast.ImportFrom,
                             ast.FunctionDef, ast.AsyncFunctionDef,
                             ast.ClassDef, ast.Assign, ast.AnnAssign,
                             ast.If, ast.Try, ast.With, ast.Return)):
            count += 1
    return count


# ---- Acceptance #4 ----

@pytest.mark.parametrize('launch_file', _iter_launch_files(),
                         ids=lambda p: p.name)
def test_launch_file_has_no_real_declare_launch_argument(launch_file: Path):
    """В каждом ``launch/*.launch.py`` ровно 0 настоящих DeclareLaunchArgument.

    Все объявления launch-аргументов должны жить в одном месте —
    ``rob_box_perception/launch_factory.py``.
    """
    n = _count_real_declare_launch_argument(launch_file)
    assert n == 0, (
        f'{launch_file.name}: найдено {n} настоящих DeclareLaunchArgument. '
        'Вынесите их в rob_box_perception/launch_factory.py (issue #2658 '
        'Acceptance #2 и #4).'
    )


# ---- Acceptance #1 ----

@pytest.mark.parametrize('launch_file', _iter_launch_files(),
                         ids=lambda p: p.name)
def test_launch_file_under_40_code_constructs(launch_file: Path):
    """Каждый launch-файл ≤40 значимых конструкций верхнего уровня.

    Шим над ``make_hailo_node_launch(...)`` не должен превращаться обратно
    в дублирование launch-логики. Docstring не считается — он поясняет
    ADR-контекст, но не добавляет логики.
    """
    n = _count_code_constructs(launch_file)
    assert n <= 40, (
        f'{launch_file.name}: {n} code constructs, ожидается <=40 '
        '(issue #2658 Acceptance #1).'
    )


# ---- Sanity-инвариант: factory живёт в Python-пакете, не в launch/ ----

def test_launch_factory_module_lives_in_python_package():
    """``launch_factory.py`` находится в ``rob_box_perception/``, не в ``launch/``.

    Это критично: setup.py устанавливает ``launch/*.launch.py`` как
    ``data_files`` через ament_index, не как Python-пакет. Положить factory
    рядом с launch-файлами и подключать через ``importlib.util`` —
    хрупко при сборке/тестах. Поэтому factory живёт в ``rob_box_perception/``
    (``find_packages`` подхватывает) и использует стандартный паттерн
    ``from rob_box_perception.launch_factory import make_hailo_node_launch``.
    """
    factory = PKG_ROOT / 'rob_box_perception' / 'launch_factory.py'
    assert factory.exists(), (
        f'{factory} не найден. Factory должна жить в rob_box_perception/, '
        'не в launch/.'
    )
    in_launch = LAUNCH_DIR / '_launch_factory.py'
    assert not in_launch.exists(), (
        f'{in_launch} найден, но factory не должна жить в launch/ '
        '(см. docstring launch_factory.py).'
    )
