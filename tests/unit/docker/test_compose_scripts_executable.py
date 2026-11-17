"""Скрипт, который compose запускает напрямую (без ``/bin/bash``), обязан иметь бит исполнения в git.

09.11.2026: ``followpath-watchdog`` на Main Pi с самого деплоя крутился в рестартах:
``/scripts/start_followpath_watchdog.sh: Permission denied`` — файл лёг в репо с режимом 100644,
а деплой берёт режим из git. Сторож «сирот» FollowPath (#4458) на роботе не работал ни дня.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[3]
ROLES = ("main", "vision", "sound")
WRAPPERS = {"/ros_scripts/ros_with_namespace.sh"}
INTERPRETERS = {"/bin/bash", "bash", "/bin/sh", "sh", "python3", "/usr/bin/python3"}


def _git_modes(role: str) -> dict[str, str]:
    out = subprocess.run(
        ["git", "ls-files", "-s", f"docker/{role}/scripts"], cwd=REPO, capture_output=True, text=True, check=True
    ).stdout
    modes: dict[str, str] = {}
    for line in out.splitlines():
        meta, path = line.split("\t", 1)
        modes.setdefault(Path(path).name, meta.split()[0])
    return modes


def _direct_scripts(role: str) -> list[tuple[str, str]]:
    compose = yaml.safe_load((REPO / "docker" / role / "docker-compose.yaml").read_text(encoding="utf-8"))
    found = []
    for name, svc in (compose.get("services") or {}).items():
        cmd = svc.get("command")
        if not isinstance(cmd, list) or not cmd:
            continue
        args = cmd[1:] if cmd[0] in WRAPPERS else cmd
        if args and args[0].startswith("/scripts/") and args[0].endswith(".sh"):
            found.append((name, args[0]))
    return found


CASES = [(role, svc, script) for role in ROLES for svc, script in _direct_scripts(role)]


def test_cases_found():
    assert any(script.endswith("start_followpath_watchdog.sh") for _, _, script in CASES)


@pytest.mark.parametrize("role,service,script", CASES)
def test_direct_script_is_executable_in_git(role, service, script):
    mode = _git_modes(role).get(Path(script).name)
    assert mode is not None, f"{role}/{service}: {script} нет в docker/{role}/scripts"
    assert mode == "100755", (
        f"{role}/{service}: {script} запускается без интерпретатора, а в git режим {mode} — "
        f"`git update-index --chmod=+x` (иначе Permission denied и рестарты)"
    )
