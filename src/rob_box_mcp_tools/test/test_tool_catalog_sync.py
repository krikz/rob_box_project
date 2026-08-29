"""The tool catalog, the tool classes and the MCP server must agree.

This is the guard rail for the single-declaration refactor. The dialogue
tool contract used to exist in several hand-maintained copies, and they
drifted into user-visible breakage:

* ``navigate_to_waypoint`` advertised a ``name`` parameter while
  ``execute()`` took ``waypoint`` — every LLM-driven waypoint navigation
  failed parameter validation.
* ``move_direction`` advertised ``duration`` against a ``distance``
  parameter — ``TypeError`` on every call that used it.
* 13 tools registered by ``mcp_server`` were absent from the LLM catalog,
  including ``task_delta`` (whose in-process interception was therefore
  unreachable) and every mapping-FSM exit tool.
* ``handle_music`` and four sibling skill facades were offered to the LLM
  with no executor behind them at all.

Each test below fails on one of those classes of drift. They are pure AST
and data checks — no ``rclpy``, no ROS2 runtime — so they run anywhere.
"""

from __future__ import annotations

import ast
import importlib.util
import pathlib
import sys

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
GENERATOR = REPO_ROOT / "tools" / "gen_tool_catalog.py"
MCP_SERVER = (
    REPO_ROOT / "src" / "rob_box_mcp_tools" / "rob_box_mcp_tools" / "mcp_server.py"
)
TOOLS_DIR = REPO_ROOT / "src" / "rob_box_mcp_tools" / "rob_box_mcp_tools" / "tools"

pytestmark = pytest.mark.skipif(
    not GENERATOR.exists() or not MCP_SERVER.exists(),
    reason="running outside a source checkout (installed package): sources unavailable",
)


@pytest.fixture(scope="module")
def catalog():
    """The committed catalog, as typed entries."""
    from rob_box_core.tool_catalog import TOOL_CATALOG

    return TOOL_CATALOG


@pytest.fixture(scope="module")
def generator():
    """Import ``tools/gen_tool_catalog.py`` as a module."""
    spec = importlib.util.spec_from_file_location("gen_tool_catalog", GENERATOR)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules["gen_tool_catalog"] = module
    spec.loader.exec_module(module)
    return module


# ---------------------------------------------------------------------------
# 1. The generated catalog is current
# ---------------------------------------------------------------------------


def test_tool_catalog_is_current(generator) -> None:
    """``_tool_catalog_data.py`` must match what the tool classes declare.

    This is what makes the tool class the single source of truth: change a
    description, a parameter or an enum and this test fails until the
    catalog is regenerated, so the LLM-facing schema can never quietly fall
    behind the code that executes it.
    """
    rendered = generator.render(generator.extract_tools())
    committed = generator.OUT_FILE.read_text(encoding="utf-8")
    assert committed == rendered, (
        "src/rob_box_core/rob_box_core/_tool_catalog_data.py is stale — "
        "run `python tools/gen_tool_catalog.py` and commit the result"
    )


# ---------------------------------------------------------------------------
# 2. The advertised schema matches what execute() accepts
# ---------------------------------------------------------------------------


def test_advertised_parameters_are_accepted_by_execute(catalog) -> None:
    """Every advertised parameter must be a real ``execute()`` argument.

    The ``move_direction(duration=…)`` failure mode: the LLM supplies a
    parameter the schema promised, the MCP registry's validation lets it
    through (it only checks required-ness and enums), and ``execute()``
    raises ``TypeError: unexpected keyword argument``.
    """
    offenders = []
    for entry in catalog:
        signature = entry.signature
        if not signature or signature.get("accepts_kwargs"):
            continue
        advertised = set(entry.parameters.get("properties", {}))
        accepted = set(signature.get("params", ()))
        unknown = advertised - accepted
        if unknown:
            offenders.append(f"{entry.name}: advertises {sorted(unknown)}, "
                             f"execute() accepts {sorted(accepted)}")
    assert not offenders, "schema promises parameters execute() rejects:\n" + "\n".join(offenders)


def test_required_execute_arguments_are_advertised(catalog) -> None:
    """Every mandatory ``execute()`` argument must be advertised as required.

    The ``navigate_to_waypoint`` failure mode: ``execute(waypoint)`` is
    mandatory but the schema never mentioned ``waypoint``, so validation
    rejected every call with «Отсутствует обязательный параметр».
    """
    offenders = []
    for entry in catalog:
        signature = entry.signature
        if not signature:
            continue
        required_by_code = set(signature.get("required", ()))
        advertised_required = set(entry.parameters.get("required", ()))
        missing = required_by_code - advertised_required
        if missing:
            offenders.append(
                f"{entry.name}: execute() requires {sorted(missing)}, "
                f"schema requires {sorted(advertised_required)}"
            )
    assert not offenders, "mandatory arguments the LLM is never asked for:\n" + "\n".join(offenders)


# ---------------------------------------------------------------------------
# 3. The catalog matches what mcp_server actually registers
# ---------------------------------------------------------------------------


def _registered_tool_names() -> set[str]:
    """Resolve ``self.registry.register(SomeTool(...))`` calls to tool names."""
    class_to_name: dict[str, str] = {}
    for source in TOOLS_DIR.glob("*.py"):
        tree = ast.parse(source.read_text(encoding="utf-8"))
        for cls in [n for n in tree.body if isinstance(n, ast.ClassDef)]:
            if "MCPTool" not in {getattr(b, "id", getattr(b, "attr", "")) for b in cls.bases}:
                continue
            for fn in [n for n in cls.body if isinstance(n, ast.FunctionDef)]:
                if fn.name != "name":
                    continue
                for node in ast.walk(fn):
                    if isinstance(node, ast.Return) and isinstance(node.value, ast.Constant):
                        class_to_name[cls.name] = node.value.value

    server = ast.parse(MCP_SERVER.read_text(encoding="utf-8"))
    # `self.foo = SomeTool(...)` then `registry.register(self.foo)`
    attr_to_class: dict[str, str] = {}
    for node in ast.walk(server):
        if isinstance(node, ast.Assign) and isinstance(node.value, ast.Call):
            callee = getattr(node.value.func, "id", None)
            for target in node.targets:
                if isinstance(target, ast.Attribute) and callee:
                    attr_to_class[target.attr] = callee

    names: set[str] = set()
    for node in ast.walk(server):
        if not (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)):
            continue
        if node.func.attr != "register" or not node.args:
            continue
        arg = node.args[0]
        if isinstance(arg, ast.Call):
            cls_name = getattr(arg.func, "id", None)
        elif isinstance(arg, ast.Attribute):
            cls_name = attr_to_class.get(arg.attr)
        else:
            continue
        if cls_name and cls_name in class_to_name:
            names.add(class_to_name[cls_name])
    return names


def test_every_llm_visible_tool_is_registered_on_the_server(catalog) -> None:
    """A tool the LLM can choose must be one the MCP server can execute.

    This is the ``handle_music`` failure mode inverted: an entry the model
    sees but the server never registered comes back as «Инструмент не
    найден» in the middle of a conversation.
    """
    registered = _registered_tool_names()
    visible = {entry.name for entry in catalog if entry.llm_visible}
    unexecutable = visible - registered
    assert not unexecutable, (
        "tools offered to the LLM that mcp_server never registers: "
        f"{sorted(unexecutable)}"
    )


def test_registered_tools_are_not_hidden_from_the_llm(catalog) -> None:
    """A registered tool should be in the catalog and visible, or explicitly hidden.

    Catches the reverse drift — the 13 tools that ``mcp_server`` wired up,
    published on ``/mcp/tools`` and kept alive, while the LLM had no way to
    call any of them. Hiding is still allowed, but it has to be a decision
    recorded on the tool (``llm_visible = False``), not an accident.
    """
    by_name = {entry.name: entry for entry in catalog}
    registered = _registered_tool_names()
    missing = sorted(name for name in registered if name not in by_name)
    assert not missing, f"registered tools absent from the catalog: {missing}"
    hidden = sorted(name for name in registered if not by_name[name].llm_visible)
    assert not hidden, (
        "registered tools hidden from the LLM without an explicit reason: "
        f"{hidden} — either expose them or document `llm_visible = False`"
    )


# ---------------------------------------------------------------------------
# 4. Schema hygiene
# ---------------------------------------------------------------------------


def test_every_tool_has_a_usable_schema(catalog) -> None:
    """Names, descriptions and JSON-Schema shape must be well formed."""
    for entry in catalog:
        assert entry.name and entry.name.islower(), f"bad tool name: {entry.name!r}"
        assert entry.description.strip(), f"{entry.name}: empty description"
        params = entry.parameters
        assert params.get("type") == "object", f"{entry.name}: parameters must be an object"
        for prop, schema in params.get("properties", {}).items():
            assert schema.get("type"), f"{entry.name}.{prop}: missing type"
            assert schema.get("description"), f"{entry.name}.{prop}: missing description"
        for required in params.get("required", ()):
            assert required in params.get("properties", {}), (
                f"{entry.name}: required parameter {required!r} is not declared"
            )


# ---------------------------------------------------------------------------
# 5. Relative imports inside the package resolve
# ---------------------------------------------------------------------------
#
# The animation vocabulary move landed ``from ..animations import
# KNOWN_ANIMATIONS, normalize_animation, ToolExecutionType`` in
# ``tools/animation.py`` — but ``ToolExecutionType`` lives in ``..base``.
# Nothing in this suite imports the ``tools`` package (``navigation.py``
# pulls ``rclpy.action`` at module level, which is why the catalog is
# built by AST in the first place), so the broken import shipped and every
# ``mcp_server`` start crashed on it — taking every tool call in the
# dialogue down with it. This check needs no ROS 2 either.

PKG_DIR = TOOLS_DIR.parent


def _module_level_names(path: pathlib.Path) -> set[str] | None:
    """Names a module binds at import time, or ``None`` if it star-imports.

    Deliberately over-generous: anything bound anywhere in the module —
    including inside ``if``/``try`` bodies — counts, so the test can only
    fail on a name that is nowhere to be found.
    """
    tree = ast.parse(path.read_text(encoding="utf-8"))
    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            names.add(node.name)
        elif isinstance(node, ast.Assign):
            names.update(t.id for t in node.targets if isinstance(t, ast.Name))
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            names.add(node.target.id)
        elif isinstance(node, ast.Import):
            names.update(a.asname or a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom):
            if any(a.name == "*" for a in node.names):
                return None
            names.update(a.asname or a.name for a in node.names)
    return names


def _resolve_relative(source: pathlib.Path, node: ast.ImportFrom) -> pathlib.Path | None:
    """The file a ``from ..mod import x`` refers to, or ``None`` if absent."""
    target = source.parent
    for _ in range(node.level - 1):
        target = target.parent
    target = target.joinpath(*(node.module or "").split(".")) if node.module else target
    if target.with_suffix(".py").exists():
        return target.with_suffix(".py")
    if (target / "__init__.py").exists():
        return target / "__init__.py"
    return None


def test_relative_imports_resolve() -> None:
    """Every ``from .x import y`` in the package must name something that exists."""
    broken: list[str] = []
    for source in sorted(PKG_DIR.rglob("*.py")):
        tree = ast.parse(source.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if not isinstance(node, ast.ImportFrom) or not node.level:
                continue
            where = f"{source.relative_to(REPO_ROOT).as_posix()}:{node.lineno}"
            target = _resolve_relative(source, node)
            if target is None:
                broken.append(f"{where}: no module {'.' * node.level}{node.module}")
                continue
            provided = _module_level_names(target)
            if provided is None:  # star-import: cannot tell, do not guess
                continue
            for alias in node.names:
                if alias.name == "*" or alias.name in provided:
                    continue
                sibling = target.parent / alias.name
                if sibling.with_suffix(".py").exists() or (sibling / "__init__.py").exists():
                    continue  # importing a submodule, not a name
                broken.append(
                    f"{where}: {alias.name!r} is not defined in "
                    f"{target.relative_to(REPO_ROOT).as_posix()}"
                )
    assert not broken, "unresolvable relative imports:\n  " + "\n  ".join(broken)
