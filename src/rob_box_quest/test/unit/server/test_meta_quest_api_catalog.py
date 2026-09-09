"""Contract check between the JSON command dispatch and its API catalog.

The API document deliberately keeps planned commands visible, so this test
compares only rows marked ``implemented`` with the string branches in
``WSSServer._on_json_cmd``.
"""

from __future__ import annotations

import ast
import re
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[5]
API_DOC = REPO_ROOT / "docs/architecture/meta-quest-api.md"
WS_SERVER = (
    REPO_ROOT
    / "src"
    / "rob_box_quest"
    / "rob_box_quest"
    / "server"
    / "ws_server.py"
)


def _implemented_dispatch_commands() -> set[str]:
    tree = ast.parse(WS_SERVER.read_text(encoding="utf-8"))
    handler = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == "_on_json_cmd"
    )
    commands: set[str] = set()
    for node in ast.walk(handler):
        if not (
            isinstance(node, ast.Compare)
            and isinstance(node.left, ast.Name)
            and node.left.id == "cmd"
        ):
            continue
        for comparator in node.comparators:
            values = (
                comparator.elts
                if isinstance(comparator, (ast.Tuple, ast.List, ast.Set))
                else (comparator,)
            )
            commands.update(
                value.value
                for value in values
                if isinstance(value, ast.Constant) and isinstance(value.value, str)
            )
    return commands


def _documented_implemented_commands() -> set[str]:
    document = API_DOC.read_text(encoding="utf-8")
    section = document.split("## 5. `JSON_CMD`", 1)[1].split("### 5.1.", 1)[0]
    commands: set[str] = set()
    for row in section.splitlines():
        match = re.match(r"\|\s*([^|]+)\|\s*implemented\s*\|", row)
        if not match:
            continue
        commands.update(
            command
            for value in re.findall(r"`([^`]+)`", match.group(1))
            for command in value.split("/")
        )
    return commands


def test_api_catalog_matches_json_command_dispatch() -> None:
    assert _documented_implemented_commands() == _implemented_dispatch_commands()


def test_api_catalog_keeps_unimplemented_commands_explicit() -> None:
    document = API_DOC.read_text(encoding="utf-8")
    section = document.split("## 5. `JSON_CMD`", 1)[1].split("### 5.1.", 1)[0]
    assert re.search(r"\|[^|]*`ui_button`[^|]*\|\s*planned\s*\|", section)
    assert re.search(
        r"\|[^|]*`admin_logs` / `admin_logs_stop`[^|]*\|\s*planned\s*\|",
        section,
    )
    assert re.search(
        r"\|[^|]*`set_panel_topic`[^|]*\|\s*planned\s*\|", section
    )
