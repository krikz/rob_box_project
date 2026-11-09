"""Деплой, оборванный замком HIL, не трогает карточку деплоя.

Раньше шаг «Create Deployment Issue» писал в карточку пустой комментарий
(run 42180010163 в #4484): `deploy_status=issues` ставился и при намеренном отказе
замка `/tmp/hil.lock`, хотя выкладки не было и диагностики нет.

Теперь «[Main Pi] HIL lock» пишет `hil_locked=true` при коде 1 (замок), а
«Deployment Summary» отдаёт `deploy_status=hil_locked`; шаг карточки срабатывает
только на `issues`. Код 2 (Main Pi молчит) — настоящий сбой, замком не считается.
"""

import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
WORKFLOW = ROOT / ".github" / "workflows" / "L-Deploy and Verify.yml"

pytestmark = pytest.mark.skipif(shutil.which("bash") is None, reason="нужен bash")


def _steps():
    data = yaml.safe_load(WORKFLOW.read_text(encoding="utf-8"))
    return [s for job in data["jobs"].values() for s in job.get("steps", [])]


def _step(**match):
    found = [s for s in _steps() if all(s.get(k) == v for k, v in match.items())]
    assert len(found) == 1, match
    return found[0]


def _run(script: str, cwd: Path, subst: dict):
    """Выполнить run-блок, подставив ${{ expr }} из subst (остальное — пустая строка)."""
    out = cwd / "gh_output"
    out.write_bytes(b"")

    def sub(m):
        return subst.get(m.group(1).strip(), "")

    text = re.sub(r"\$\{\{(.*?)\}\}", sub, script)
    # Путь подставляем в текст: переменные GITHUB_* из окружения родителя не доходят до bash.
    text = text.replace("$GITHUB_OUTPUT", "gh_output")
    env = dict(os.environ)
    script_file = cwd / "step.sh"
    script_file.write_bytes(text.encode("utf-8"))
    # Вывод — в файл, не в pipe: фоновый процесс, унаследовавший pipe, подвесил бы communicate().
    with open(cwd / "step.log", "wb") as log:
        proc = subprocess.run(
            ["bash", "-e", "step.sh"], cwd=cwd, env=env, stdin=subprocess.DEVNULL, stdout=log, stderr=log, timeout=60
        )
    return proc.returncode, out.read_text(encoding="utf-8")


@pytest.mark.parametrize("check_rc,expect_output", [(0, ""), (1, "hil_locked=true"), (2, "")])
def test_hil_step_marks_only_real_lock(tmp_path, check_rc, expect_output):
    stub = tmp_path / "scripts" / "deploy" / "check_hil_lock.sh"
    stub.parent.mkdir(parents=True)
    stub.write_bytes(f"#!/usr/bin/env bash\nexit {check_rc}\n".encode())  # LF: CRLF ломает exit
    step = _step(name="[Main Pi] HIL lock (#4371)")
    assert step["id"] == "hil_lock"
    rc, out = _run(step["run"], tmp_path, {"env.main_pi_ip": "10.1.1.10"})
    assert rc == check_rc  # шаг по-прежнему падает при замке и при молчании Pi
    assert out.strip() == expect_output


def _summary_status(tmp_path, hil_locked: str) -> str:
    step = _step(id="deploy_summary")
    subst = {"steps.hil_lock.outputs.hil_locked": hil_locked, "job.status": "failure"}
    rc, out = _run(step["run"], tmp_path, subst)
    assert rc == 0
    return dict(line.split("=", 1) for line in out.split())["deploy_status"]


def test_summary_marks_hil_lock_separately(tmp_path):
    assert _summary_status(tmp_path, "true") == "hil_locked"


def test_summary_without_lock_still_reports_issues(tmp_path):
    assert _summary_status(tmp_path, "") == "issues"


def test_issue_step_fires_only_on_issues():
    cond = _step(name="🚨 Create Deployment Issue (any issues)")["if"]
    assert "deploy_status == 'issues'" in cond
    assert "hil_locked" not in cond
