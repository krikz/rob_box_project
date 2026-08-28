"""Regression tests for ``scripts/agent_flow/agent-flow-post-merge-build.sh``.

ADR-0022 extension (issue #1475): after MERGED PR in develop/main,
this script triggers L-Build-All-Services so .image-versions.dev gets fresh
dev-<sha> tags. Without the trigger (issue #1475 evidence: PR #1434 merge
18.08 23:00 MSK → robot still on dev-ddd09e51 from 17:49, 5h stale).

Contract pinned by these tests:

* branch develop/main → `gh workflow run` is called once and exits 0;
* branch other (feature/test/copilot/round) → exit 0 WITHOUT calling
  `gh workflow run` (only develop/main trigger build);
* `gh workflow run` failure (non-zero exit) AND no recent run visible via
  `gh run list` → retry up to MAX_ATTEMPTS (default 2) → exit 0
  with warning (non-fatal: merge-gate must continue);
* `gh workflow run` failure BUT a fresh run on the same workflow+branch
  appears via `gh run list` → race condition detected → exit 0 WITHOUT
  retry (issue #1535: prevents duplicate workflow runs);
* workflow file not found in repo → exit 0 without retry;
* gh auth not available → exit 0 without retry (merge-gate continues);
* DRY_RUN=true → no real `gh` invocation, but logs the would-be command
  AND skips `gh run list` dedup (would also hit API);
* missing args → exit 64 (EX_USAGE);
* POST_MERGE_BUILD_MAX_ATTEMPTS=N → overrides retry count.

Mock strategy: a ``gh`` shim that records invocations + controls return code.
We rewrite ``PATH`` to point at a temporary directory that contains only the
shim, so the script's ``gh`` calls go through it. We also stub
``gh auth status`` / ``gh workflow view`` / ``gh workflow run`` /
``gh run list`` / ``gh run view``.
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
                  view_rc: int = 0, run_rc: int = 0,
                  run_list_log: Path | None = None) -> Path:
    """Write a `gh` shim to bin_dir/gh and return its path.

    The shim:
      * `gh auth status ...` → exit ``auth_rc``;
      * `gh workflow view NAME ...` → exit ``view_rc``;
      * `gh workflow run NAME ...` → append the args to ``log_file`` and
        exit ``run_rc`` (configurable via env ``GH_SHIM_WORKFLOW_RUN_RC``);
      * `gh run list ...` → echo JSON from ``run_list_log`` if set,
        else echo ``[]``.

    Issue #1535: dedup via `gh run list` requires the shim to handle this
    subcommand. ``run_list_log`` is the path to a JSON file that the shim
    emits verbatim — tests write the desired fake runs there. The script
    parses the JSON via python3 (it does NOT call ``gh run view`` after
    dedup — only one API roundtrip per race-condition check).
    """
    bin_dir.mkdir(parents=True, exist_ok=True)
    shim = bin_dir / "gh"
    run_list_log_str = str(run_list_log) if run_list_log else ""
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
              run)
                run_action="$1"; shift
                case "$run_action" in
                  list)
                    # gh run list ... → emit JSON from file, else []
                    if [ -n "{run_list_log_str}" ] && [ -f "{run_list_log_str}" ]; then
                      cat "{run_list_log_str}"
                    else
                        echo '[]'
                    fi
                    ;;
                  *) echo "shim: unknown gh run action: $run_action" >&2; exit 2 ;;
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
    """gh workflow run fails repeatedly → still exit 0 (non-fatal).

    Issue #1535: retry count reduced from 3 to 2 (MAX_ATTEMPTS=2, default
    POST_MERGE_BUILD_MAX_ATTEMPTS=2). With dedup via gh run list seeing
    empty result, both attempts call gh workflow run, then we exit 0 with
    a failure marker. Sleep between is only 5s (attempt 1) → fast test.
    """
    monkeypatch.setenv("GH_SHIM_WORKFLOW_RUN_RC", "1")
    proc = subprocess.run(
        [str(SCRIPT), "1434", "develop"],
        capture_output=True, text=True,
        env={**os.environ, "GH_SHIM_WORKFLOW_RUN_RC": "1"},
        timeout=30,
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr
    log = (tmp_path / "workflow_run.log").read_text()
    # MAX_ATTEMPTS=2 → 2 lines in the log (was 3 in old behaviour).
    assert log.count("\n") == 2, f"expected exactly 2 attempts (issue #1535); got: {log!r}"
    assert "❌" in proc.stderr, f"expected failure marker in stderr; got: {proc.stderr!r}"
    assert "no recent run" in proc.stderr, (
        "expected dedup 'miss' log line; got: " + proc.stderr
    )


def test_recent_run_skips_dispatch(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Pre-dispatch dedup (issue #1535 follow-up): a recent build on the
    same workflow+branch means the push-trigger (or a previous merge-gate
    tick / a manual run) already covered this merge — skip WITHOUT calling
    ``gh workflow run`` at all.

    merge-gate fires this backup on every tick while a merged issue stays
    open; GitHub does NOT dedupe workflow_dispatch by workflow_name+branch.
    The pre-dispatch guard makes the script idempotent per branch.

    Total calls to ``gh workflow run``: exactly 0.
    """
    bin_dir = tmp_path / "bin"
    log_file = tmp_path / "workflow_run.log"
    run_list_log = tmp_path / "run_list.json"
    # createdAt within the 900s pre-dispatch window, but older than the 60s
    # race window — models a build started by the push-trigger ~5 min ago.
    now_iso = subprocess.run(
        ["python3", "-c",
         "import datetime; "
         "print((datetime.datetime.now(datetime.timezone.utc) - "
         "datetime.timedelta(minutes=5)).strftime('%Y-%m-%dT%H:%M:%SZ'))"],
        capture_output=True, text=True, check=True,
    ).stdout.strip()
    run_list_log.write_text(
        f'[{{"databaseId":32587539885,"createdAt":"{now_iso}"}}]'
    )
    _make_gh_shim(bin_dir, log_file, run_list_log=run_list_log)
    env = os.environ.copy()
    env["PATH"] = f"{bin_dir}{os.pathsep}{env['PATH']}"
    env["GH_REPO"] = "krikz/rob_box_project"
    env["BUILD_WORKFLOW"] = "L-Build-All-Services.yml"
    env["DEVELOP_BRANCH"] = "develop"
    env["MAIN_BRANCH"] = "main"
    env["DRY_RUN"] = "false"
    env["GH_SHIM_WORKFLOW_RUN_RC"] = "1"  # irrelevant: must skip before dispatch

    proc = subprocess.run(
        [str(SCRIPT), "1434", "develop"],
        capture_output=True, text=True, env=env, timeout=15,
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr
    log = log_file.read_text()
    # CRITICAL: exactly 0 invocations → the pre-dispatch guard skipped.
    assert log == "", f"expected 0 calls to gh workflow run; got: {log!r}"
    assert "skip" in proc.stderr, (
        f"expected pre-dispatch skip log; got: {proc.stderr!r}"
    )
    assert "❌" not in proc.stderr, (
        f"pre-dispatch skip should not be an error; got: {proc.stderr!r}"
    )


def test_old_run_outside_dedup_window_triggers_retry(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """If gh run list returns a run OLDER than the dedup window, retry.

    Distinguishes from test_race_condition_dedup_skips_retry: a stale
    (>60s) run must NOT short-circuit the retry. We expect 2 calls (one
    per attempt) and the "no recent run" log line.
    """
    bin_dir = tmp_path / "bin"
    log_file = tmp_path / "workflow_run.log"
    run_list_log = tmp_path / "run_list.json"
    # createdAt 1 hour ago — way outside both the 900s pre-dispatch window
    # and the 60s dedup window.
    stale_iso = subprocess.run(
        ["python3", "-c",
         "import datetime; "
         "print((datetime.datetime.now(datetime.timezone.utc) - "
         "datetime.timedelta(hours=1)).strftime('%Y-%m-%dT%H:%M:%SZ'))"],
        capture_output=True, text=True, check=True,
    ).stdout.strip()
    run_list_log.write_text(
        f'[{{"databaseId":32500000000,"createdAt":"{stale_iso}"}}]'
    )
    _make_gh_shim(bin_dir, log_file, run_list_log=run_list_log)
    env = os.environ.copy()
    env["PATH"] = f"{bin_dir}{os.pathsep}{env['PATH']}"
    env["GH_REPO"] = "krikz/rob_box_project"
    env["BUILD_WORKFLOW"] = "L-Build-All-Services.yml"
    env["DEVELOP_BRANCH"] = "develop"
    env["MAIN_BRANCH"] = "main"
    env["DRY_RUN"] = "false"
    env["GH_SHIM_WORKFLOW_RUN_RC"] = "1"

    proc = subprocess.run(
        [str(SCRIPT), "1434", "develop"],
        capture_output=True, text=True, env=env, timeout=30,
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr
    log = log_file.read_text()
    # Stale run → must fall through to retry → 2 attempts total.
    assert log.count("\n") == 2, (
        f"stale run must not short-circuit retry; got: {log!r}"
    )
    assert "no recent run" in proc.stderr


def test_max_attempts_env_override(tmp_path: Path) -> None:
    """POST_MERGE_BUILD_MAX_ATTEMPTS=1 → exactly 1 call, no retry."""
    bin_dir = tmp_path / "bin"
    log_file = tmp_path / "workflow_run.log"
    _make_gh_shim(bin_dir, log_file)
    env = os.environ.copy()
    env["PATH"] = f"{bin_dir}{os.pathsep}{env['PATH']}"
    env["GH_REPO"] = "krikz/rob_box_project"
    env["BUILD_WORKFLOW"] = "L-Build-All-Services.yml"
    env["DEVELOP_BRANCH"] = "develop"
    env["MAIN_BRANCH"] = "main"
    env["DRY_RUN"] = "false"
    env["POST_MERGE_BUILD_MAX_ATTEMPTS"] = "1"
    env["GH_SHIM_WORKFLOW_RUN_RC"] = "1"  # always fail

    proc = subprocess.run(
        [str(SCRIPT), "1434", "develop"],
        capture_output=True, text=True, env=env, timeout=15,
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr
    log = log_file.read_text()
    assert log.count("\n") == 1, (
        f"MAX_ATTEMPTS=1 → exactly 1 call; got: {log!r}"
    )


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
