"""``ToolCallAccumulator`` — один класс, а не две живые копии.

Контекст (карточка W6-1, скан дублей
``docs/plans/2026-08-30-dedup-wave3-handoff.md`` §3.2). Копий было две,
обе в ОДНОМ пакете и обе используемые:

* ``core/tool_call_accumulator.py`` — экспортировалась через
  ``core/__init__.py``, покрыта тестами
  (``test/unit/core/test_tool_call_accumulator.py``);
* ``async_executor.py:52`` — из неё брал накопитель ``llm_adapter.py``,
  то есть ровно тот путь, по которому реально идут стриминговые
  tool_calls от LLM. Эта копия и удалена.

Сходство нормализованного исходника — 0.91, и расхождение было не
косметическим: у версии из ``async_executor`` **не было** ``get_count()``
и ``has_tool_calls()``. Логика ``add_chunk`` / ``get_complete_tool_calls``
/ ``clear`` совпадала символ в символ, поэтому версия из ``core`` —
строгий надмножество-вариант, и канон именно она.

Тесты поведения самого накопителя лежат рядом, в
``test_tool_call_accumulator.py``; здесь — только про то, что копия одна.
"""

import ast
from pathlib import Path

import pytest

from rob_box_mcp_tools.core import ToolCallAccumulator as _CoreAccumulator
from rob_box_mcp_tools.core.tool_call_accumulator import ToolCallAccumulator

_PKG = Path(__file__).resolve().parents[3] / "rob_box_mcp_tools"


@pytest.mark.unit
def test_core_package_exports_the_canonical_class():
    assert _CoreAccumulator is ToolCallAccumulator


@pytest.mark.unit
def test_consumers_import_from_the_canonical_module():
    """Ре-экспорта через ``async_executor`` больше нет.

    Единственный потребитель — ``llm_adapter.py`` — берёт класс прямо из
    ``core``. Ре-экспорт не оставлен намеренно: обе копии лежали в одном
    пакете, потребитель снаружи ни один (``grep`` по репозиторию), а
    промежуточный алиас — ровно та щель, в которой копии и разъехались.

    ``llm_adapter`` тянет ``rclpy`` на импорте, поэтому проверка
    статическая, по AST.
    """
    tree = ast.parse((_PKG / "llm_adapter.py").read_text(encoding="utf-8"))
    sources = {
        node.module: {alias.name for alias in node.names}
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom)
    }
    assert "ToolCallAccumulator" in sources.get("core.tool_call_accumulator", set())
    assert "ToolCallAccumulator" not in sources.get("async_executor", set())

    exported = ast.parse((_PKG / "async_executor.py").read_text(encoding="utf-8"))
    reexports = {
        alias.name
        for node in ast.walk(exported)
        if isinstance(node, ast.ImportFrom)
        for alias in node.names
    }
    assert "ToolCallAccumulator" not in reexports


@pytest.mark.unit
def test_canonical_class_keeps_the_richer_api():
    """``get_count`` / ``has_tool_calls`` — то, чего не было у копии."""
    acc = ToolCallAccumulator()
    assert acc.has_tool_calls() is False
    assert acc.get_count() == 0


def _toplevel_classes(path: Path) -> set:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    return {node.name for node in tree.body if isinstance(node, ast.ClassDef)}


@pytest.mark.unit
def test_no_module_declares_its_own_copy():
    """Предохранитель: третья копия должна ронять тест.

    ``llm_adapter.py`` читается исходником — он тянет ``rclpy`` на
    импорте.
    """
    offenders = [
        path.name
        for path in (_PKG / "async_executor.py", _PKG / "llm_adapter.py")
        if "ToolCallAccumulator" in _toplevel_classes(path)
    ]
    assert not offenders, (
        "ToolCallAccumulator объявлен заново — он живёт в "
        "rob_box_mcp_tools.core.tool_call_accumulator и больше нигде: "
        + ", ".join(offenders)
    )
