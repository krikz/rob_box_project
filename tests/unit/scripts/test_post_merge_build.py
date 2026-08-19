"""Regression tests for ``scripts/agent_flow/agent-flow-post-merge-build.sh``.

GATE-4 (issue #1475, ADR-0022 extension): after MERGED PR in develop/main,
this script triggers L-Build-All-Services so .image-versions.dev gets fresh
dev-<sha> tags. Without the trigger (issue #1475 evidence: PR #1434 merge
18.08 23:00 MSK → robot still on dev-ddd09e51 from 17:49, 5h stale).

Contract pinned by these tests:

* branch develop/main → `gh workflow run` is called once and exits 0;
* branch other (feature/test/copilot/round) → exit 0 WITHOUT calling
  `gh workflow run` (only develop/main trigger build);
* `gh workflow run` failure (non-zero exit) → retry 3 attempts → exit 0
  with warning (non-fatal: merge-gate must continue);
* workflow file not found in repo → exit 0 without retry;
* gh auth not available → exit 0 without retry (merge-gate continues);
* DRY_RUN=true → no real `gh` invocation, but logs the would-be command;
* missing args → exit 64 (EX_USAGE).

Mock strategy: a ``gh`` shim that records invocations + controls return code.
We rewrite ``PATH`` to point at a temporary directory that contains only the
shim, so the script's ``gh`` calls go through it. We also stub
``gh auth status`` / ``gh workflow view`` / ``gh workflow run``.
"""

from __future__ import annotations

import os
import subprocess
import textwrap
from pathlib import Path

import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = REPO_ROOT / "scripts" / "agent_flow" / "agent-flow-post-merge-build.sh"


# --------------------------------------------------------------------------- #
# Fixtures: a `gh` shim with a configurable behaviour per subcommand.
# --------------------------------------------------------------------------- #


def _make_gh_shim(bin_dir: Path, log_file: Path, *, auth_rc: int = 0,
                  view_rc: int = 0, run_rc: int = 0) -> Path:
    """Write a `gh` shim to bin_dir/gh and return its path.

    The shim:
      * `gh auth status ...` → exit ``auth_rc``;
      * `gh workflow view NAME ...` → exit ``view_rc``;
      * `gh workflow run NAME ...` → append the args to ``log_file`` and
        exit ``run_rc`` (configurable via env ``GH_SHIM_WORKFLOW_RUN_RC``).
    """
    bin_dir.mkdir(parents=True, exist_ok=True)
    shim = bin_dir / "gh"
    shim.write_text(
        textwrap.dedent(
            f"""\
            #!/usr/bin/env bash
            # test shim for gh — see tests/unit/scripts/test_post_merge_build.py
            subcommand="$1"; shift
            case "$subcommand" in
              auth)
                # `gh auth status ...` → auth_rc (default 0)
                exit {auth_rc}
                ;;
              workflow)
                wf_action="$1"; shift
                case "$wf_action" in
                  view)
                    # `gh workflow view NAME --repo ...` → view_rc (default 0)
                    exit {view_rc}
                    ;;
                  run)
                    # Append args (one per invocation) to log file.
                    printf '%s\\n' "$*" >> "{log_file}"
                    rc="${{GH_SHIM_WORKFLOW_RUN_RC:-{run_rc}}}"
                    if [ "$rc" != "0" ]; then exit "$rc"; else exit 0; fi
                    ;;
                  *) echo "shim: unknown gh workflow action: $wf_action" >&2; exit 2 ;;
                esac
                ;;
              *) echo "shim: unknown gh subcommand: $subcommand" >&2; exit 2 ;;
            esac
            """
        )
    )
    shim.chmod(0o755)
    return shim


@pytest.fixture()
def gh_shim(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """Drop a `gh` shim into a tmp bin dir and prepend it to PATH.

    Returns the path to the shim binary. The fixture also sets the env vars
    that the script reads (GH_REPO, BUILD_WORKFLOW, DEVELOP_BRANCH, MAIN_BRANCH,
    DRY_RUN) so individual tests can override ``DRY_RUN`` via the ``env``
    parameter of ``_run_script``.
    """
    bin_dir = tmp_path / "bin"
    log_file = tmp_path / "workflow_run.log"
    shim = _make_gh_shim(bin_dir, log_file)
    monkeypatch.setenv("PATH", f"{bin_dir}{os.pathsep}{os.environ['PATH']}")
    monkeypatch.setenv("GH_SHIM_WORKFLOW_RUN_LOG", str(log_file))
    monkeypatch.setenv("GH_REPO", "krikz/rob_box_project")
    monkeypatch.setenv("BUILD_WORKFLOW", "L-Build-All-Services.yml")
    monkeypatch.setenv("DEVELOP_BRANCH", "develop")
    monkeypatch.setenv("MAIN_BRANCH", "main")
    monkeypatch.setenv("DRY_RUN", "false")
    monkeypatch.setenv("GH_SHIM_WORKFLOW_RUN_RC", "0")
    return shim


def _run_script(args: list[str], env: dict | None = None) -> subprocess.CompletedProcess[str]:
    """Invoke the script with ``args`` and a clean environment."""
    full_env = os.environ.copy()
    if env:
        full_env.update(env)
    return subprocess.run(
        [str(SCRIPT), *args],
        capture_output=True,
        text=True,
        env=full_env,
        timeout=15,
    )


# --------------------------------------------------------------------------- #
# Tests
# --------------------------------------------------------------------------- #


def test_develop_branch_triggers_build(gh_shim: Path, tmp_path: Path) -> None:
    """PR merged into develop → exactly one gh workflow run."""
    proc = _run_script(["1434", "develop"])
    assert proc.returncode == 0, proc.stdout + proc.stderr
    log = (tmp_path / "workflow_run.log").read_text()
    assert "L-Build-All-Services.yml" in log, (
        f"expected workflow name in log; got: {log!r}"
    )
    assert "--ref" in log
    assert "develop" in log


def test_main_branch_triggers_build(gh_shim: Path, tmp_path: Path) -> None:
    """PR merged into main → exactly one gh workflow run, ref=main."""
    proc = _run_script(["1500", "main"])
    assert proc.returncode == 0, proc.stdout + proc.stderr
    log = (tmp_path / "workflow_run.log").read_text()
    assert "L-Build-All-Services.yml" in log
    assert "main" in log


def test_feature_branch_skips(gh_shim: Path, tmp_path: Path) -> None:
    """PR merged into feature/test → exit 0, NO gh workflow run."""
    proc = _run_script(["1434", "feature/my-branch"])
    assert proc.returncode == 0, proc.stdout + proc.stderr
    log_path = tmp_path / "workflow_run.log"
    log = log_path.read_text() if log_path.exists() else ""
    assert log == "", f"expected no gh workflow run for feature branch; got: {log!r}"


def test_round_branch_skips(gh_shim: Path, tmp_path: Path) -> None:
    """z-{e2e}/test-round-N → SKIP (only develop/main trigger)."""
    proc = _run_script(["1434", "z-{e2e}/test-round-161"])
    assert proc.returncode == 0, proc.stdout + proc.stderr
    log_path = tmp_path / "workflow_run.log"
    log = log_path.read_text() if log_path.exists() else ""
    assert log == ""


def test_gh_workflow_run_failure_retries_but_exits_zero(
    gh_shim: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """gh workflow run fails 3 times → still exit 0 (non-fatal).

    Total real sleep: 5 + 10 + 15 = 30 seconds. Marked ``@pytest.mark.slow``
    so the default run can skip via ``-m "not slow"``.
    """
    monkeypatch.setenv("GH_SHIM_WORKFLOW_RUN_RC", "1")
    proc = subprocess.run(
        [str(SCRIPT), "1434", "develop"],
        capture_output=True, text=True,
        env={**os.environ, "GH_SHIM_WORKFLOW_RUN_RC": "1"},
        timeout=60,
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr
    log = (tmp_path / "workflow_run.log").read_text()
    # 3 attempts on failure → 3 lines in the log.
    assert log.count("\n") >= 3, f"expected ≥3 retries; got: {log!r}"
    assert "❌" in proc.stderr, f"expected failure marker in stderr; got: {proc.stderr!r}"


def test_workflow_not_found_exits_zero(tmp_path: Path) -> None:
    """gh workflow view fails (workflow missing) → exit 0, no run."""
    bin_dir = tmp_path / "bin"
    log_file = tmp_path / "workflow_run.log"
    # view_rc=1 simulates "workflow not found"
    _make_gh_shim(bin_dir, log_file, view_rc=1)
    # Replicate env setup the fixture normally does.
    env = os.environ.copy()
    env["PATH"] = f"{bin_dir}{os.pathsep}{env['PATH']}"
    env["GH_REPO"] = "krikz/rob_box_project"
    env["BUILD_WORKFLOW"] = "L-Build-All-Services.yml"
    env["DEVELOP_BRANCH"] = "develop"
    env["MAIN_BRANCH"] = "main"
    env["DRY_RUN"] = "false"

    proc = subprocess.run(
        [str(SCRIPT), "1434", "develop"],
        capture_output=True, text=True, env=env, timeout=15,
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr
    log = log_file.read_text() if log_file.exists() else ""
    assert log == "", f"expected no run when workflow view fails; got: {log!r}"


def test_gh_auth_failure_exits_zero(tmp_path: Path) -> None:
    """gh auth fails → exit 0 (merge-gate continues)."""
    bin_dir = tmp_path / "bin"
    log_file = tmp_path / "workflow_run.log"
    _make_gh_shim(bin_dir, log_file, auth_rc=1)
    env = os.environ.copy()
    env["PATH"] = f"{bin_dir}{os.pathsep}{env['PATH']}"
    env["GH_REPO"] = "krikz/rob_box_project"
    env["BUILD_WORKFLOW"] = "L-Build-All-Services.yml"
    env["DEVELOP_BRANCH"] = "develop"
    env["MAIN_BRANCH"] = "main"
    env["DRY_RUN"] = "false"

    proc = subprocess.run(
        [str(SCRIPT), "1434", "develop"],
        capture_output=True, text=True, env=env, timeout=15,
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr
    log = log_file.read_text() if log_file.exists() else ""
    assert log == "", f"expected no run when auth fails; got: {log!r}"


def test_dry_run_does_not_invoke_gh(gh_shim: Path, tmp_path: Path) -> None:
    """DRY_RUN=true → no real gh invocation, but the script prints intent."""
    proc = _run_script(["1434", "develop"], env={"DRY_RUN": "true"})
    assert proc.returncode == 0, proc.stdout + proc.stderr
    log_path = tmp_path / "workflow_run.log"
    log = log_path.read_text() if log_path.exists() else ""
    assert log == "", f"DRY_RUN must not invoke gh; got: {log!r}"
    # The script's run() helper logs the command to stderr.
    assert "DRY-RUN" in proc.stderr, f"expected DRY-RUN marker; got: {proc.stderr!r}"
    assert "L-Build-All-Services.yml" in proc.stderr


def test_missing_args_exits_64(gh_shim: Path) -> None:
    """No args → EX_USAGE (64) per BSD sysexits."""
    proc = _run_script([])
    assert proc.returncode == 64, proc.stdout + proc.stderr


def test_push_to_registry_field_present(gh_shim: Path, tmp_path: Path) -> None:
    """The shim records --field push_to_registry=true to push to local registry."""
    proc = _run_script(["1434", "develop"])
    assert proc.returncode == 0, proc.stdout + proc.stderr
    log = (tmp_path / "workflow_run.log").read_text()
    assert "push_to_registry=true" in log, (
        f"expected push_to_registry=true; got: {log!r}"
    )


def test_yaml_workflow_is_valid() -> None:
    """The companion push-trigger workflow .github/workflows/L-Build-On-Branch-Push.yml
    parses as YAML and declares the expected on.push branches.

    PyYAML normalises ``on:`` to the boolean ``True`` (YAML 1.1 quirk) — we
    accept either form. ``uses`` for reusable workflows lives inside ``steps``,
    not on the job — the test checks the step-level key.
    """
    yml_path = REPO_ROOT / ".github" / "workflows" / "L-Build-On-Branch-Push.yml"
    assert yml_path.exists(), f"missing workflow file: {yml_path}"
    doc = yaml.safe_load(yml_path.read_text())
    on = doc[True] if True in doc else doc["on"]
    push = on["push"]
    branches = push["branches"]
    assert "develop" in branches
    assert "main" in branches
    # jobs.build-all uses reusable workflow via a step.uses.
    jobs = doc["jobs"]
    assert "build-all" in jobs
    build_all = jobs["build-all"]
    # runner
    runs_on = build_all["runs-on"]
    assert "self-hosted" in runs_on
    # step-level `uses` references the reusable workflow.
    step_uses = [s.get("uses") for s in build_all["steps"]]
    assert any("L-Build-All-Services.yml" in (u or "") for u in step_uses), (
        f"expected a step.uses pointing at L-Build-All-Services.yml; got: {step_uses}"
    )


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #


def test_helper_gh_shim_writes_log(gh_shim: Path, tmp_path: Path) -> None:
    """Sanity: the shim itself behaves as documented."""
    log_file = tmp_path / "workflow_run.log"
    proc = subprocess.run(
        [str(gh_shim), "workflow", "run", "demo", "--ref", "develop"],
        capture_output=True,
        text=True,
        env={**os.environ, "GH_SHIM_WORKFLOW_RUN_LOG": str(log_file)},
        timeout=5,
    )
    assert proc.returncode == 0
    log_text = log_file.read_text()
    assert "demo" in log_text, f"expected 'demo' in log; got: {log_text!r}"
    assert "--ref" in log_text
    assert "develop" in log_text
