"""
test_dialogue_node_imports.py — регресс-тест для #1413 (lazy-import ceiling).

Согласно ADR-0021 «Lazy-import ceiling»:
  - Top-of-file: stdlib + declared <exec_depend> + same-package
  - Lazy only if: optional (try/except) ИЛИ ament_index (ROS runtime only)
  - Дубликаты запрещены

Проверяем:
  1. Парсим dialogue_node.py через ast, считаем lazy imports.
  2. Белый список justified: skills.*, ament_index_python, rob_box_mcp_tools,
     rob_box_voice.scheduler.tool_executor.
  3. ``yaml``/``traceback``/``uuid``/``concurrent.futures``/``os``/``tts_voice_registry``
     /``rob_box_harness.health``/``rob_box_harness.core.dialogue_state_machine``
     НЕ должны встречаться как lazy imports внутри функций.
  4. Smoke: top-of-file содержит обязательные модули (yaml, traceback, uuid,
     concurrent.futures, tts_voice_registry, rob_box_harness.health).

Не требует ROS2/harness в dev (статический анализ).
"""

import ast
from pathlib import Path

import pytest


DIALOGUE_NODE = (
    Path(__file__).resolve().parents[3]
    / "rob_box_voice"
    / "dialogue_node.py"
)


# Whitelist: эти модули МОЖНО импортировать лениво (justified optional).
# Каждый — ДОЛЖЕН быть обёрнут в try/except (или иметь комментарий-обоснование).
LAZY_WHITELIST = frozenset({
    # ament_index_python — ROS runtime only, не объявлен в package.xml как
    # <exec_depend> нашему пакету; оставлен lazy, чтобы dev-юниты без
    # ROS2 не падали. Используется в try/except с probe-fallback.
    "ament_index_python.packages",
    # rob_box_mcp_tools — optional, явно сказано «no <exec_depend>» в
    # комментариях вокруг импорта; грузится только при
    # tool_provider='ros_mcp'.
    "rob_box_mcp_tools.llm_adapter",
    # rob_box_voice.scheduler.tool_executor — W7b fail-open: если
    # scheduler не стартует, адаптер возвращается unwrapped, voice не
    # ломается. Lazy — чтобы unit-тесты, не поднимающие scheduler, не
    # зависели от опциональной инфраструктуры.
    "rob_box_voice.scheduler",
})

# Эти модули ОБЯЗАНЫ быть top-of-file.
TOP_OF_FILE_MUST_CONTAIN = frozenset({
    "yaml",                                # pyyaml, requirements.txt
    "traceback",                           # stdlib
    "concurrent.futures",                  # stdlib
    "uuid",                                # stdlib
    "rob_box_voice.tts_voice_registry",    # same package
    "rob_box_harness.health",              # declared <exec_depend>
})


def _load_ast():
    """Парсит dialogue_node.py → AST module."""
    src = DIALOGUE_NODE.read_text(encoding="utf-8")
    return ast.parse(src, filename=str(DIALOGUE_NODE)), src.splitlines()


def _module_name(node: ast.Import | ast.ImportFrom) -> str:
    """Извлекает dotted name (top-2 segments) из Import/ImportFrom node.

    ``from rob_box_voice.scheduler.tool_executor import SchedulerToolExecutor``
    → ``rob_box_voice.scheduler`` — этого достаточно для классификации
    (package boundary).
    """
    if isinstance(node, ast.Import):
        return ".".join(node.names[0].name.split(".")[:2])
    if node.module is None:
        return ""
    parts = node.module.split(".")
    if len(parts) >= 2:
        return f"{parts[0]}.{parts[1]}"
    return parts[0]


def _iter_lazy_imports(tree: ast.Module):
    """Возвращает (lineno, module) для каждого import/importfrom ВНУТРИ функции
    или метода (т.е. не на module-level)."""
    for node in ast.walk(tree):
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            # Пропускаем module-level: их parent — Module
            # ast.walk не даёт parent; используем вложенный обход.
            pass
    # Точнее: обходим с проверкой вложенности
    lazy = []
    def visit(node, depth=0):
        if isinstance(node, (ast.Import, ast.ImportFrom)) and depth > 0:
            lazy.append((node.lineno, _module_name(node), type(node).__name__))
        for child in ast.iter_child_nodes(node):
            # depth > 0 если node внутри FunctionDef/AsyncFunctionDef/Lambda/ClassDef
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
                visit(child, depth + 1)
            else:
                visit(child, depth)
    visit(tree)
    return lazy


# ─────────────────────────────────────────────────────────────────────────────
# Smoke: файл существует и парсится
# ─────────────────────────────────────────────────────────────────────────────

def test_dialogue_node_file_exists():
    assert DIALOGUE_NODE.exists(), f"dialogue_node.py not found at {DIALOGUE_NODE}"


def test_dialogue_node_parses():
    tree, _ = _load_ast()
    assert isinstance(tree, ast.Module)


# ─────────────────────────────────────────────────────────────────────────────
# Acceptance A.1: top-of-file содержит обязательные модули
# ─────────────────────────────────────────────────────────────────────────────

def test_top_of_file_has_required_imports():
    """yaml, traceback, uuid, concurrent.futures, tts_voice_registry,
    rob_box_harness.health — обязаны быть top-of-file (depth=0)."""
    tree, src_lines = _load_ast()

    top_level_modules = set()
    for stmt in tree.body:  # tree.body — top-level statements
        if isinstance(stmt, ast.Import):
            for alias in stmt.names:
                top_level_modules.add(alias.name.split(".")[0])
        elif isinstance(stmt, ast.ImportFrom) and stmt.module:
            top_level_modules.add(stmt.module)

    missing = TOP_OF_FILE_MUST_CONTAIN - top_level_modules
    assert not missing, (
        f"Top-of-file MUST contain these modules (ADR-0021 ceiling): "
        f"{sorted(missing)}. "
        f"Found top-level: {sorted(top_level_modules)}"
    )


# ─────────────────────────────────────────────────────────────────────────────
# Acceptance A.2 / D: lazy imports ограничены whitelist
# ─────────────────────────────────────────────────────────────────────────────

def test_no_lazy_stdlib_or_required_imports():
    """stdlib + declared deps + same-package НЕ ДОЛЖНЫ быть lazy."""
    tree, _ = _load_ast()
    lazy = _iter_lazy_imports(tree)

    forbidden = {
        "yaml", "traceback", "uuid", "concurrent.futures", "os",
        "rob_box_voice.tts_voice_registry",
        "rob_box_harness.health",
        "rob_box_harness.core.dialogue_state_machine",  # duplicate!
    }
    bad = []
    for lineno, mod, kind in lazy:
        if mod in forbidden:
            bad.append((lineno, mod, kind))

    assert not bad, (
        "Found lazy imports of required/stdlib modules. "
        "These MUST be top-of-file (ADR-0021 ceiling):\n"
        + "\n".join(f"  line {ln}: {kind} {mod}" for ln, mod, kind in bad)
    )


def test_lazy_imports_within_whitelist():
    """Все lazy imports должны быть в whitelist (justified optional)."""
    tree, _ = _load_ast()
    lazy = _iter_lazy_imports(tree)

    # Уникальные модули, не в whitelist
    bad = []
    for lineno, mod, kind in lazy:
        if mod and mod not in LAZY_WHITELIST:
            bad.append((lineno, mod, kind))

    assert not bad, (
        "Found lazy imports OUTSIDE the justified whitelist (ADR-0021). "
        "Either move to top-of-file or justify (add to LAZY_WHITELIST "
        "with a comment in this test):\n"
        + "\n".join(f"  line {ln}: {kind} {mod}" for ln, mod, kind in bad)
    )


def test_lazy_import_ceiling_count():
    """ADR-0021 ceiling: ≤ 10 lazy imports в файле (было 18, цель ≤ 10).

    Гибкая проверка: ловим регрессию, если кто-то добавит новые."""
    tree, _ = _load_ast()
    lazy = _iter_lazy_imports(tree)
    # Сейчас в whitelist 10 модулей: 6 skills + 2 ament_index_python + 1
    # rob_box_mcp_tools + 1 rob_box_voice.scheduler.tool_executor.
    # Лимит — 12 с запасом на будущее. Главное — чтобы число не
    # подскочило обратно к 18.
    assert len(lazy) <= 12, (
        f"Lazy-import count = {len(lazy)} exceeds ceiling 12. "
        f"Found:\n"
        + "\n".join(f"  line {ln}: {kind} {mod}" for ln, mod, kind in lazy)
    )


# ─────────────────────────────────────────────────────────────────────────────
# Acceptance B: yaml/traceback точно на top, не дублированы в методах
# ─────────────────────────────────────────────────────────────────────────────

def test_no_duplicate_imports_of_top_level_modules():
    """Не должно быть повторных импортов уже top-level модулей внутри функций."""
    tree, _ = _load_ast()
    top_level = set()
    for stmt in tree.body:
        if isinstance(stmt, ast.Import):
            for alias in stmt.names:
                top_level.add(alias.name)
        elif isinstance(stmt, ast.ImportFrom) and stmt.module:
            top_level.add(stmt.module)

    lazy = _iter_lazy_imports(tree)

    duplicates = []
    for lineno, mod, kind in lazy:
        if mod in top_level or mod.split(".")[0] in top_level:
            duplicates.append((lineno, mod, kind))

    assert not duplicates, (
        "Found DUPLICATE imports (already top-level). "
        "Remove these — they are pointless lazy-imports:\n"
        + "\n".join(f"  line {ln}: {kind} {mod}" for ln, mod, kind in duplicates)
    )


# ─────────────────────────────────────────────────────────────────────────────
# Acceptance C: yaml.safe_load не обёрнут в try/except ImportError на runtime
# ─────────────────────────────────────────────────────────────────────────────

def test_yaml_safe_load_uses_top_level_binding():
    """``yaml.safe_load`` в методах должен ссылаться на top-level ``yaml``,
    не на алиас ``_yaml`` (которого теперь нет)."""
    tree, src_lines = _load_ast()
    bad = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Name) and node.id == "_yaml":
            bad.append(node.lineno)
        if isinstance(node, ast.Name) and node.id == "_tb":
            bad.append(node.lineno)
        if isinstance(node, ast.Name) and node.id == "_uuid":
            bad.append(node.lineno)
        if isinstance(node, ast.Name) and node.id == "_os":
            bad.append(node.lineno)
    assert not bad, (
        f"Found references to removed lazy aliases: lines {bad}. "
        f"After top-of-file refactor these should reference top-level names."
    )
