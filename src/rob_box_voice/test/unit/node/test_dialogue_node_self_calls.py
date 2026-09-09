"""Каждый ``self._foo(...)`` в dialogue_node должен быть определён.

Почему этот тест существует
---------------------------

Написан после реального инцидента в change'е skill-scoped-dialogue-context:
рефакторинг вырезал из файла диапазон строк по двум якорям и вместе с
целевым методом снёс два соседних (``_load_skill_prompts`` и
``_on_prompt_stats``). Файл остался синтаксически корректным, вся
существующая юнит-сюита осталась зелёной — потому что тесты
``dialogue_node`` конструируют ноду через ``object.__new__`` и НИКОГДА не
исполняют ``__init__``. Обнаружилось бы это только на роботе, падением
контейнера при старте.

Тест дешёвый и статический: разбираем AST, собираем определённые методы и
все обращения ``self._что-то``. Ни rclpy, ни запуска ноды не требуется.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

_NODE = (
    Path(__file__).resolve().parents[3]
    / "rob_box_voice"
    / "dialogue_node.py"
)


def _tree() -> ast.Module:
    return ast.parse(_NODE.read_text(encoding="utf-8"))


def _class_defs(tree: ast.Module) -> list[ast.ClassDef]:
    return [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]


def _defined_names(cls: ast.ClassDef) -> set[str]:
    """Методы и атрибуты, присвоенные через ``self.x = ...``."""
    names: set[str] = set()
    for node in cls.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            names.add(node.name)
        elif isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name):
                    names.add(target.id)
    for node in ast.walk(cls):
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name):
            if node.value.id == "self" and isinstance(node.ctx, ast.Store):
                names.add(node.attr)
        elif isinstance(node, ast.AnnAssign):
            target = node.target
            if (
                isinstance(target, ast.Attribute)
                and isinstance(target.value, ast.Name)
                and target.value.id == "self"
            ):
                names.add(target.attr)
    return names


def _called_methods(cls: ast.ClassDef) -> set[str]:
    """Имена, вызванные как ``self._foo(...)`` (только приватные)."""
    called: set[str] = set()
    for node in ast.walk(cls):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if (
            isinstance(func, ast.Attribute)
            and isinstance(func.value, ast.Name)
            and func.value.id == "self"
            and func.attr.startswith("_")
        ):
            called.add(func.attr)
    return called


@pytest.mark.parametrize(
    "cls_name",
    [cls.name for cls in _class_defs(_tree())],
)
def test_every_self_call_resolves_to_something_defined(cls_name: str) -> None:
    """``self._foo()`` без ``def _foo`` — падение ноды на старте."""
    cls = next(c for c in _class_defs(_tree()) if c.name == cls_name)
    defined = _defined_names(cls)
    # Наследование: базовые классы (rclpy Node и пр.) вне разбора, поэтому
    # проверяем только то, что класс объявляет сам, а неизвестное имя
    # считаем ошибкой лишь когда класс вообще что-то определяет.
    if not defined:
        pytest.skip(f"{cls_name}: нечего проверять")

    missing = sorted(
        name
        for name in _called_methods(cls)
        if name not in defined
        # Дандеры и вызовы, приходящие из базового класса rclpy.Node.
        and not name.startswith("__")
    )
    known_from_base = {
        "_logger",
    }
    missing = [m for m in missing if m not in known_from_base]

    assert not missing, (
        f"{cls_name}: вызываются несуществующие методы {missing}. "
        f"Нода упадёт на старте, а юнит-тесты этого не увидят — они "
        f"конструируют ноду через object.__new__ и не исполняют __init__."
    )


def test_skill_pipeline_methods_are_present() -> None:
    """Точечная страховка на методы, которые уже терялись при рефакторинге."""
    tree = _tree()
    defined = {
        node.name
        for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    }
    for required in (
        "_load_skill_prompts",
        "_validate_skill_fragments",
        "_activate_skill_for",
        "_publish_skill_load_counters",
        "_on_prompt_stats",
    ):
        assert required in defined, f"{required} пропал из dialogue_node"
