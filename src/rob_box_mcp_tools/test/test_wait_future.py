"""``wait_future`` — одно объявление на пакет, а не три копии.

Контекст (карточка W6-1, скан дублей
``docs/plans/2026-08-30-dedup-wave3-handoff.md`` §3.2): хелпер ожидания
``rclpy``-future лежал тремя копиями с одинаковым телом:

* ``tools/navigation.py:32`` — с развёрнутым докстрингом, единственным
  местом, где записано ПОЧЕМУ нельзя
  ``rclpy.spin_until_future_complete()``;
* ``tools/system.py:25``     — тот же докстринг;
* ``tools/mapping.py:30``    — докстринг ужат до одной строки, причина
  потеряна.

Именно потерянная причина и опасна: следующий, кто заглянет в
``mapping.py``, увидит «обёртка над event.wait» и спокойно заменит её на
``spin_until_future_complete`` — а тот, будучи вызванным из колбэка под
``MultiThreadedExecutor``, ломает executor и молча гасит все дальнейшие
колбэки подписок.

Хелпер чистый (``threading`` из stdlib), ROS-импортов не тянет — потому
и живёт в ``base.py``, который сам ROS-free: ``tools/navigation.py``
тянет ``rclpy.action`` на импорте, и каталог тулов из-за этого
собирается через AST (см. ``tools/gen_tool_catalog.py``).
"""

import ast
import importlib.util
import sys
import threading
from concurrent.futures import Future
from pathlib import Path

import pytest

_PKG = Path(__file__).resolve().parents[1] / "rob_box_mcp_tools"
_TOOLS = _PKG / "tools"
_USERS = ("navigation.py", "system.py", "mapping.py")

# ``base.py`` грузится по пути, а не через ``import rob_box_mcp_tools.base``:
# test_mcp_server.py подменяет подмодули ``rob_box_mcp_tools`` в
# ``sys.modules`` и часть подмен переживает свой тест, из-за чего обычный
# импорт здесь падает с "unknown location" в зависимости от порядка
# сборки. ``base.py`` — чистый stdlib, грузится по пути без последствий.
_spec = importlib.util.spec_from_file_location(
    "rob_box_mcp_tools_base_for_wait_future", str(_PKG / "base.py")
)
assert _spec is not None and _spec.loader is not None
_base = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = _base
_spec.loader.exec_module(_base)

wait_future = _base.wait_future


@pytest.mark.unit
def test_returns_true_when_future_completes_in_time():
    future = Future()
    threading.Timer(0.01, lambda: future.set_result("ok")).start()

    assert wait_future(future, timeout_sec=2.0) is True
    assert future.result() == "ok"


@pytest.mark.unit
def test_returns_false_on_timeout():
    future = Future()  # никто его не завершает

    assert wait_future(future, timeout_sec=0.05) is False


@pytest.mark.unit
def test_returns_true_when_future_is_already_done():
    """``add_done_callback`` на завершённом future срабатывает сразу."""
    future = Future()
    future.set_result(42)

    assert wait_future(future, timeout_sec=0.0) is True


@pytest.mark.unit
def test_does_not_touch_the_executor():
    """Хелпер не имеет права спинить ноду — он её вообще не видит.

    Проверяем по сигнатуре: единственные аргументы — сам future и
    таймаут. Появится ``node``/``executor`` — значит кто-то вернул
    ``spin_until_future_complete`` через заднюю дверь.
    """
    import inspect

    params = list(inspect.signature(wait_future).parameters)
    assert params == ["future", "timeout_sec"], params


def _toplevel_functions(path: Path) -> set:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    return {
        node.name
        for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    }


@pytest.mark.unit
def test_no_tool_module_declares_its_own_copy():
    """Предохранитель: четвёртая копия должна ронять тест.

    Читаем исходники, а не импортируем: ``tools/navigation.py`` тянет
    ``rclpy.action`` на уровне модуля.
    """
    offenders = [
        name
        for name in _USERS
        if {"_wait_future", "wait_future"} & _toplevel_functions(_TOOLS / name)
    ]
    assert not offenders, (
        "wait_future объявлен заново — он живёт в rob_box_mcp_tools.base "
        "и больше нигде: " + ", ".join(offenders)
    )


def _imports_from_base(path: Path) -> set:
    """Имена, которые модуль тянет из ``..base``."""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module == "base" and node.level == 2:
            names |= {alias.name for alias in node.names}
    return names


@pytest.mark.unit
def test_users_import_the_shared_helper():
    for name in _USERS:
        assert "wait_future" in _imports_from_base(_TOOLS / name), (
            f"{name} не импортирует wait_future из ..base"
        )
