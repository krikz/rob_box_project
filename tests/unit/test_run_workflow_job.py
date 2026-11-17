"""scripts/ci/run_workflow_job.py — локальный прогон `run:`-шагов workflow.

Главное свойство: раннер НЕ выдаёт пропущенное за пройденное (AGENTS.md —
«честный FAIL лучше красивого PASS»): `uses:`, `${{ }}`, Install-шаги
печатаются как SKIP, упавший шаг без continue-on-error — FAIL и exit 1.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER = REPO_ROOT / "scripts" / "ci" / "run_workflow_job.py"

WORKFLOW = """
name: demo
env:
  WF_VAR: from-workflow
jobs:
  hosted:
    runs-on: ubuntu-latest
    env:
      JOB_VAR: from-job
    steps:
      - uses: actions/checkout@v7
      - name: Install tools
        run: echo SHOULD-NOT-RUN > "$MARK_DIR/install"
      - name: Env layering
        env:
          STEP_VAR: from-step
        run: echo "$WF_VAR $JOB_VAR $STEP_VAR" > "$MARK_DIR/env"
      - name: Working directory
        working-directory: scripts
        run: basename "$PWD" > "$MARK_DIR/cwd"
      - name: Github expression
        run: echo "${{ github.sha }}" > "$MARK_DIR/expr"
      - name: Soft failure
        run: exit 3
        continue-on-error: true
      - name: Pipefail like GitHub
        run: false | true
  robot:
    runs-on: [self-hosted, rob-box]
    steps:
      - run: echo SHOULD-NOT-RUN > "$MARK_DIR/robot"
"""


def _runner():
    spec = importlib.util.spec_from_file_location("run_workflow_job", RUNNER)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def _run(tmp_path, monkeypatch, capsys, *extra):
    wf = tmp_path / "demo.yml"
    wf.write_text(WORKFLOW, encoding="utf-8")
    marks = tmp_path / "marks"
    marks.mkdir()
    monkeypatch.setenv("MARK_DIR", str(marks))
    rc = _runner().main([str(wf), *extra])
    return rc, marks, capsys.readouterr().out


def test_runs_run_steps_with_github_semantics(tmp_path, monkeypatch, capsys):
    rc, marks, out = _run(tmp_path, monkeypatch, capsys)
    # pipefail: `false | true` обязан упасть, как у GitHub (bash -eo pipefail).
    assert rc == 1, out
    assert "FAIL hosted › Pipefail like GitHub" in out
    assert (marks / "env").read_text().split() == [
        "from-workflow",
        "from-job",
        "from-step",
    ]
    assert (marks / "cwd").read_text().strip() == "scripts"
    assert "WARN hosted › Soft failure" in out


def test_never_reports_skipped_steps_as_passed(tmp_path, monkeypatch, capsys):
    rc, marks, out = _run(tmp_path, monkeypatch, capsys, "--skip", "Pipefail")
    assert rc == 0, out
    assert not (marks / "install").exists()
    assert not (marks / "expr").exists()
    assert "SKIP hosted › actions/checkout@v7" in out
    assert "SKIP hosted › Install tools" in out
    assert "SKIP hosted › Github expression" in out


def test_self_hosted_jobs_are_not_run_by_default(tmp_path, monkeypatch, capsys):
    _rc, marks, out = _run(tmp_path, monkeypatch, capsys)
    assert not (marks / "robot").exists()
    assert "robot" not in out.split("job'ы:")[1].splitlines()[0]


def test_with_install_runs_install_steps(tmp_path, monkeypatch, capsys):
    _run(tmp_path, monkeypatch, capsys, "--with-install", "--step", "Install")
    assert (tmp_path / "marks" / "install").exists()


def test_real_lint_workflow_resolves_and_plans(capsys):
    """Смоук на настоящем G-Lint Code: находится по имени, dry-run не падает."""
    assert _runner().main(["G-Lint Code", "--dry-run"]) == 0
    out = capsys.readouterr().out
    assert "python-lint" in out and "PLAN" in out
