"""Guard #4579: после каждого `git reset --hard` в деплое сабмодули обновляются.

`reset --hard` не двигает сабмодули, и на роботе они оставались там, куда их
поставили руками (vesc_nexus на feature-ветке, ros2leds откатан).
"""

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
WORKFLOW = ROOT / ".github" / "workflows" / "L-Deploy and Verify.yml"
UPDATE_SH = ROOT / "docker" / "main" / "update_and_restart.sh"

RESET = re.compile(r"git reset --hard")
UPDATE = "git submodule update --init --recursive"
SYNC = "git submodule sync"


def _check(text: str, window: int) -> int:
    resets = list(RESET.finditer(text))
    for m in resets:
        tail = text[m.end() : m.end() + window]
        assert SYNC in tail, f"нет submodule sync после reset --hard @{m.start()}"
        assert UPDATE in tail, f"нет submodule update после reset --hard @{m.start()}"
    return len(resets)


def test_workflow_updates_submodules_after_every_reset():
    text = WORKFLOW.read_text(encoding="utf-8")
    assert _check(text, 300) == 3  # Vision, Sound, Main


def test_update_and_restart_updates_submodules():
    text = UPDATE_SH.read_text(encoding="utf-8")
    assert _check(text, 300) == 1


def test_workflow_has_submodule_verify_step_warning_only():
    text = WORKFLOW.read_text(encoding="utf-8")
    assert "Verify submodules at recorded commits (#4579)" in text
    assert "::warning::" in text
