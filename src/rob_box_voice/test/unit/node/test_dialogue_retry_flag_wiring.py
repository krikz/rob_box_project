"""
test_dialogue_retry_flag_wiring.py — статический детектор NameError.

🔴 Два живых инцидента подряд, 30.08.2026, оба одного класса: имя читается
там, где оно не связано, `NameError` вылетает внутри asyncio-таски или в
конструкторе, и снаружи это выглядит НЕ как ошибка, а как молчащий робот.

1. `0e7bb478` завёл флаг ``is_code_retry`` в сигнатуре ``_dispatch_turn``,
   а читал его в теле ``_run_turn``, куда флаг не добавили и не пробросили::

       # _run_turn, ~26-я строка от начала функции
       if not is_code_retry:        # NameError: name 'is_code_retry'

   Падало ДО вызова LLM, на КАЖДОМ ходе. Снаружи: STT принимал фразу
   («✅ ПРИНЯТО»), играл звук подтверждения — и робот замолкал навсегда.
   Коммит откатили целиком (`db0fba22`), вместе с полезной частью фикса.

2. Переприменяя тот же фикс, блок вызова guard'а вставили в ``__init__``
   вместо ``_handle_result``::

       File ".../dialogue_node.py", line 645, in __init__
         if spoken and self._check_embedded_renardo_code_and_retry(
       NameError: name 'spoken' is not defined

   Нода не поднималась вообще.

Первая версия этого теста проверяла только флаги ``is_*_retry`` в двух
методах и второй инцидент пропустила. Поэтому теперь проверка общая: разбор
области видимости через :mod:`symtable` — то же, чем пользуется сам
интерпретатор. Имя, которое функция читает и которое разрешается в
глобальную область, обязано существовать на уровне модуля (присваивание,
импорт, def/class) или быть builtin.

Аннотации исключены сознательно: аннотация локальной переменной или
атрибута внутри функции по PEP 526 не вычисляется в рантайме, поэтому
``self._x: Optional[T] = v`` без импорта ``Optional`` — недочёт линтера,
а не NameError. Тест ловит только то, что реально падает.

Не требует ROS2 и не импортирует ноду — чистый статический анализ.

PR-15 (#3711, часть 15p): нода v1 удалена вместе с отдельной проверкой её файла;
контракт держит проверка всего пакета (в том числе ``dialog_node.py`` v2).
"""

import ast
import builtins
import symtable
from pathlib import Path

import pytest


_VOICE_PKG = Path(__file__).resolve().parents[3] / "rob_box_voice"

#: Модульные dunder'ы, которые интерпретатор кладёт в globals сам.
MODULE_DUNDERS = frozenset({
    "__file__", "__name__", "__doc__", "__package__",
    "__spec__", "__loader__", "__builtins__", "__debug__",
})


# ---------------------------------------------------------------------------
# Статический анализ
# ---------------------------------------------------------------------------


def _annotation_only_names(src: str) -> set:
    """Имена, встречающиеся ТОЛЬКО в позиции аннотации.

    По PEP 526 аннотация внутри тела функции не вычисляется, поэтому такие
    имена не могут дать NameError и в отчёт попадать не должны.
    """
    tree = ast.parse(src)
    in_annotation, elsewhere = set(), set()

    def collect(node, sink):
        for sub in ast.walk(node):
            if isinstance(sub, ast.Name) and isinstance(sub.ctx, ast.Load):
                sink.add(sub.id)

    annotation_nodes = []
    for node in ast.walk(tree):
        if isinstance(node, ast.AnnAssign) and node.annotation is not None:
            annotation_nodes.append(node.annotation)
        elif isinstance(node, ast.arg) and node.annotation is not None:
            annotation_nodes.append(node.annotation)
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.returns:
            annotation_nodes.append(node.returns)
    for ann in annotation_nodes:
        collect(ann, in_annotation)

    annotation_ids = {id(n) for ann in annotation_nodes for n in ast.walk(ann)}
    for node in ast.walk(tree):
        if (isinstance(node, ast.Name) and isinstance(node.ctx, ast.Load)
                and id(node) not in annotation_ids):
            elsewhere.add(node.id)

    return in_annotation - elsewhere


def unbound_global_reads(path: Path) -> list:
    """Имена, которые функция читает, но которые нигде не связаны.

    Returns:
        Список ``(scope_path, name)``. Пустой список = NameError невозможен.
    """
    # utf-8-sig: часть файлов пакета лежит с BOM; сам интерпретатор его
    # съедает, а ``symtable`` на строке с ``﻿`` падает синтаксисом.
    src = path.read_text(encoding="utf-8-sig")
    top = symtable.symtable(src, str(path), "exec")
    known = (
        {s.get_name() for s in top.get_symbols()
         if s.is_assigned() or s.is_imported() or s.is_namespace()}
        | set(dir(builtins))
        | MODULE_DUNDERS
        | _annotation_only_names(src)
    )

    found = []

    def walk(table, trail):
        if table.get_type() == "function":
            for sym in table.get_symbols():
                if (sym.is_global() and sym.is_referenced()
                        and sym.get_name() not in known):
                    found.append((" > ".join(trail[1:]), sym.get_name()))
        for child in table.get_children():
            walk(child, trail + [child.get_name()])

    walk(top, [top.get_name()])
    return found


# ---------------------------------------------------------------------------
# Тесты
# ---------------------------------------------------------------------------


def _voice_sources() -> list:
    return sorted(
        p for p in _VOICE_PKG.rglob("*.py")
        if "__pycache__" not in p.parts
    )


@pytest.mark.parametrize(
    "source", _voice_sources(), ids=lambda p: p.name,
)
def test_voice_package_has_no_unbound_names(source: Path) -> None:
    """Тот же контракт на весь пакет: молчащая нода не должна повториться."""
    found = unbound_global_reads(source)
    assert not found, (
        f"{source.name}: имена читаются, но нигде не связаны:\n"
        + "\n".join(f"    {scope} -> {name}" for scope, name in found)
    )
