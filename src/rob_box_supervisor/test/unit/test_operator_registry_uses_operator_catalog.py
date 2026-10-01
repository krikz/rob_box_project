"""Реестр оператора строится из каталога оператора, а не из llm-видимого.

Issue #3305: ``say`` (``llm_visible=False``, ``operator_visible=True``) не
доходил до ТАРС, потому что ``_build_operator_tools`` брал ``ToolRegistry()``
только из llm-видимых. Имя ``say`` в supervisor_node не хардкодится — признак
живёт в каталоге.
"""

from __future__ import annotations

import ast
import pathlib

SUPERVISOR_NODE = (
    pathlib.Path(__file__).resolve().parents[2]
    / "rob_box_supervisor"
    / "supervisor_node.py"
)


def _build_operator_tools_node() -> ast.FunctionDef:
    tree = ast.parse(SUPERVISOR_NODE.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.FunctionDef)
            and node.name == "_build_operator_tools"
        ):
            return node
    raise AssertionError("_build_operator_tools не найден")


def test_operator_registry_is_built_for_operator() -> None:
    fn = _build_operator_tools_node()
    calls = [
        n
        for n in ast.walk(fn)
        if isinstance(n, ast.Call)
        and getattr(n.func, "id", "") == "ToolRegistry"
    ]
    assert calls, "ToolRegistry() не создаётся в _build_operator_tools"
    for call in calls:
        kwargs = {k.arg: k.value for k in call.keywords}
        flag = kwargs.get("for_operator")
        assert isinstance(flag, ast.Constant) and flag.value is True


def test_say_name_is_not_hardcoded_in_supervisor_node() -> None:
    fn = _build_operator_tools_node()
    literals = {
        n.value
        for n in ast.walk(fn)
        if isinstance(n, ast.Constant) and isinstance(n.value, str)
    }
    assert "say" not in literals


def test_say_reaches_operator_registry_but_not_personality() -> None:
    from rob_box_harness.core.tool_registry import ToolRegistry

    operator = {s.name for s in ToolRegistry(for_operator=True).list_tools()}
    personality = {s.name for s in ToolRegistry().list_tools()}
    assert "say" in operator
    assert "say" not in personality
