"""Алиасы robbox_aliases.sh, указывающие на скрипты репо, ведут на существующие файлы.

rbcheck и rbmonitor показывали на docker/diagnose_data_flow.sh и docker/monitor_system.sh,
которых нет с рефакторинга раскладки scripts/ (39bd55df6): живые лежат в scripts/diagnostics/.
"""

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ALIASES = ROOT / "scripts" / "maintenance" / "robbox_aliases.sh"
REPO_PATH = re.compile(r"~/rob_box_project/([\w./-]+\.sh)\b")


def test_alias_script_targets_exist():
    targets = REPO_PATH.findall(ALIASES.read_text(encoding="utf-8"))
    assert targets, "в robbox_aliases.sh не найдено ни одной ссылки на скрипт репо"
    missing = [t for t in targets if not (ROOT / t).is_file()]
    assert missing == []


def test_rbcheck_and_rbmonitor_point_to_diagnostics():
    text = ALIASES.read_text(encoding="utf-8")
    assert "alias rbcheck='~/rob_box_project/scripts/diagnostics/diagnose_data_flow.sh'" in text
    assert "alias rbmonitor='~/rob_box_project/scripts/diagnostics/monitor_system.sh'" in text


def test_install_aliases_points_to_existing_file():
    text = (ROOT / "scripts" / "setup" / "install_aliases.sh").read_text(encoding="utf-8")
    assert "scripts/maintenance/robbox_aliases.sh" in text
    assert "~/rob_box_project/scripts/robbox_aliases.sh" not in text
