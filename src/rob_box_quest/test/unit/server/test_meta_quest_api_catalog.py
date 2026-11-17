"""Contract check between the JSON command dispatch and its API catalog.

The API document deliberately keeps planned commands visible, so this test
compares only rows marked ``implemented`` with the commands the server
actually dispatches.
"""

from __future__ import annotations

import re
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[5]
API_DOC = REPO_ROOT / "docs/architecture/meta-quest-api.md"


def _implemented_dispatch_commands() -> set[str]:
    """Команды, которые сервер реально диспатчит.

    До voice-vr 10 (issue #2195) это был AST-обход if-цепочки в
    ``WSSServer._on_json_cmd`` (``if cmd == "..."``). Тот рефакторинг
    свёл ``_on_json_cmd`` к терминальному dispatcher'у через
    ``JSON_CMD_HANDLERS.get(cmd)`` — веток в самой функции больше нет,
    поэтому источник истины теперь ключи этого dict'а (см. тот же приём
    в ``test_voice_vr_02_bridge_conformance.py`` и
    ``test_voice_vr_07_catalog_conformance.py``).
    """
    from rob_box_quest.server.ws_server import JSON_CMD_HANDLERS

    return set(JSON_CMD_HANDLERS.keys())


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
