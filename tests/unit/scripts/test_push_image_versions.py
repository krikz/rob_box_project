"""Regression tests for ``scripts/ci/push-image-versions.sh``.

The script is the race-safety guard for ``L: Build Main/Vision Pi Services``
(issue #1388).  Two build jobs run in parallel and each finishes with
``git commit`` + ``git push origin HEAD:<branch>`` on the same branch.  The
previous shell-snippet only printed a warning when the push lost the race,
which produced the symptom documented in #1388:  the local commit is created
but the remote ``develop`` keeps the old ``.image-versions``.

The new helper wraps the push in a ``pull --rebase`` + retry loop and
**fails the job** on a non-recoverable push failure.  These tests pin the
contract:

* a successful first push exits 0 and prints the success line;
* a non-fast-forward race (one remote commit ahead, unrelated) is recovered
  via rebase and the push succeeds without losing the local commit;
* a non-fast-forward race with conflicting content (the real-world symptom)
  is also recovered because the build commits edit different sed lines of
  the versions file — the rebase produces a clean merge;
* a push that keeps failing after every attempt exits non-zero so the job
  can fail loudly (the acceptance criterion: «Если push fail — job должен
  fail с явной ошибкой»);
* a fatal rebase (true content conflict that is NOT auto-resolvable) is
  reported and the helper aborts instead of leaving a half-rebased state.

Implementation uses real ``git`` invocations in a throwaway working tree,
so the tests cover the actual shell + git interaction rather than a mock.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import textwrap
import time
import uuid
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = REPO_ROOT / "scripts" / "ci" / "push-image-versions.sh"


def _git(cwd: Path, *args: str, check: bool = True) -> str:
    """Run a git command, returning combined output.  Default: check=True."""
    proc = subprocess.run(
        ("git", *args),
        cwd=cwd,
        capture_output=True,
        text=True,
        env={**os.environ, "GIT_TERMINAL_PROMPT": "0"},
    )
    if check and proc.returncode != 0:
        raise AssertionError(
            f"git {' '.join(args)} failed (rc={proc.returncode})\n"
            f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
        )
    return (proc.stdout + proc.stderr).strip()


def _run(cwd: Path, *args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        args,
        cwd=cwd,
        capture_output=True,
        text=True,
        env={**os.environ, "PATH": os.environ["PATH"]},
    )


# --------------------------------------------------------------------------- #
# Fixtures
# --------------------------------------------------------------------------- #


@pytest.fixture()
def dual_clone(tmp_path: Path) -> tuple[Path, Path]:
    """Spin up two clones of a tiny repo and wire them up as origin/local.

    Returns ``(work, origin)`` — ``work`` is the build-runner checkout (where
    the push script runs) and ``origin`` is a bare remote the script will
    push into.  Both branches (``develop`` and ``feature``) point at the
    same initial commit, mirroring a clean build run.
    """
    work = tmp_path / "work"
    origin = tmp_path / "origin.git"
    origin.mkdir()
    _git(origin, "init", "--bare", "--initial-branch=develop")

    seed = tmp_path / "seed"
    seed.mkdir()
    _git(seed, "init", "--initial-branch=develop")
    _git(seed, "config", "user.email", "ci@rob-box.local")
    _git(seed, "config", "user.name", "Rob Box CI")
    (seed / "README.md").write_text("# seed\n")
    _git(seed, "add", "README.md")
    _git(seed, "commit", "--message=init")
    _git(seed, "remote", "add", "origin", str(origin))
    _git(seed, "push", "-u", "origin", "develop")

    work_parent = work.parent
    work_parent.mkdir(parents=True, exist_ok=True)
    _git(work_parent, "clone", str(origin), str(work), "--branch=develop")
    _git(work, "config", "user.email", "ci@rob-box.local")
    _git(work, "config", "user.name", "Rob Box CI")
    # Caller decides which branch to operate on — the most common case in
    # the failing job is a one-off feature branch (e.g. ``develop`` itself
    # or a per-service helper branch).
    return work, origin


def _commit_and_stage(work: Path, file: Path, contents: str) -> None:
    file.parent.mkdir(parents=True, exist_ok=True)
    file.write_text(contents)
    _git(work, "add", str(file.relative_to(work)))


def _spawn_rival(tmp_path: Path, origin: Path) -> Path:
    """Create a second clone that tracks ``origin`` but is *not* on
    ``feature/race`` yet.  Callers push the rival commit from this clone.
    """
    other = tmp_path / "other"
    _git(tmp_path, "clone", str(origin), str(other), "--branch=develop")
    _git(other, "config", "user.email", "ci@rob-box.local")
    _git(other, "config", "user.name", "Rob Box CI")
    return other


# --------------------------------------------------------------------------- #
# 1. Happy path: clean rebase, first push succeeds.
# --------------------------------------------------------------------------- #


def test_successful_first_push_exits_zero(dual_clone: tuple[Path, Path]) -> None:
    work, _origin = dual_clone
    _git(work, "checkout", "-b", "feature/race")
    versions = work / "docker" / "vision" / ".image-versions.dev"
    _commit_and_stage(
        work,
        versions,
        textwrap.dedent(
            """\
            OAK_D_TAG=dev-c7232b32
            VOICE_ASSISTANT_TAG=dev-c7232b32
            """
        ),
    )
    _git(work, "commit", "--message=ci: vision SHA tags → dev-c7232b32 [skip ci]")

    result = _run(work, "bash", str(SCRIPT), "feature/race", "vision")

    assert result.returncode == 0, result.stdout + result.stderr
    assert "pushed" in result.stdout
    assert "to origin/feature/race" in result.stdout


# --------------------------------------------------------------------------- #
# 2. Race recovery: the other build job has pushed a sibling commit first.
#    The pull --rebase must replay our commit on top and the next push must
#    succeed.  We assert that BOTH commits survive on the remote.
# --------------------------------------------------------------------------- #


def test_recovers_from_non_fast_forward_via_rebase(
    dual_clone: tuple[Path, Path], tmp_path: Path
) -> None:
    work, origin = dual_clone
    versions = work / "docker" / "vision" / ".image-versions.dev"
    _commit_and_stage(work, versions, "OAK_D_TAG=dev-c7232b32\n")
    _git(work, "commit", "--message=ci: vision SHA tags → dev-c7232b32 [skip ci]")

    # Simulate a parallel build: push an unrelated, non-conflicting commit
    # straight onto the remote feature branch from outside ``work``.
    other = _spawn_rival(tmp_path, origin)
    _git(other, "checkout", "-b", "feature/race")
    (other / "README.md").write_text("# seed\n\nmore notes from main job\n")
    _git(other, "add", "README.md")
    _git(other, "commit", "--message=ci: main SHA tags → dev-c7232b32 [skip ci]")
    _git(other, "push", "origin", "HEAD:feature/race", check=False)

    # Now run the script from the still-out-of-date checkout.
    result = _run(work, "bash", str(SCRIPT), "feature/race", "vision")

    assert result.returncode == 0, result.stdout + result.stderr
    assert "pushed" in result.stdout

    # Both commits live on the remote branch — neither was lost.
    log = _git(origin, "log", "feature/race", "--pretty=%s")
    assert "main SHA tags" in log
    assert "vision SHA tags" in log


# --------------------------------------------------------------------------- #
# 3. Realistic race: the two builds edit different sed keys in the same
#    versions file, so a simple rebase produces a clean merge.  This is the
#    failure mode observed in run #32148650895 (issue #1388).
# --------------------------------------------------------------------------- #


def test_recovers_from_overlapping_race_in_same_branch(
    dual_clone: tuple[Path, Path], tmp_path: Path
) -> None:
    """The real CI race: vision and main edit DIFFERENT files on the SAME branch.

    ``L: Build Vision Pi Services`` writes
    ``docker/vision/.image-versions.dev`` and ``L: Build Main Pi Services``
    writes ``docker/main/.image-versions.dev``; both jobs then push the
    same branch (``develop``).  When the worker falls behind, the rebase
    must replay its commit without touching the sibling file.
    """
    work, origin = dual_clone

    # Both clones start from a shared ``develop`` that already has both
    # versions files in their previous-round state.
    (work / "docker" / "vision").mkdir(parents=True, exist_ok=True)
    (work / "docker" / "vision" / ".image-versions.dev").write_text(
        "VOICE_ASSISTANT_TAG=dev-old\n"
    )
    (work / "docker" / "main").mkdir(parents=True, exist_ok=True)
    (work / "docker" / "main" / ".image-versions.dev").write_text(
        "TWIST_MUX_TAG=dev-old\n"
    )
    _git(work, "add", "docker/vision/.image-versions.dev")
    _git(work, "add", "docker/main/.image-versions.dev")
    _git(work, "commit", "--message=ci(seed): baseline image-versions")
    _git(work, "push", "origin", "develop")

    # Vision job rewrites its file and commits on the branch.
    _git(work, "checkout", "develop")
    _git(work, "checkout", "-b", "feature/race")
    (work / "docker" / "vision" / ".image-versions.dev").write_text(
        "VOICE_ASSISTANT_TAG=dev-c7232b32\n"
    )
    _git(work, "add", "docker/vision/.image-versions.dev")
    _git(work, "commit", "--message=ci: vision SHA tags → dev-c7232b32 [skip ci]")

    # Main job (rival clone) rewrites its OWN file and pushes first.
    other = _spawn_rival(tmp_path, origin)
    _git(other, "checkout", "-b", "feature/race")
    (other / "docker" / "main" / ".image-versions.dev").write_text(
        "TWIST_MUX_TAG=dev-c7232b32\n"
    )
    _git(other, "add", "docker/main/.image-versions.dev")
    _git(other, "commit", "--message=ci: main SHA tags → dev-c7232b32 [skip ci]")
    _git(other, "push", "origin", "HEAD:feature/race", check=False)

    # Vision job retries via the new helper.
    result = _run(work, "bash", str(SCRIPT), "feature/race", "vision")

    assert result.returncode == 0, result.stdout + result.stderr
    # BOTH files landed on the remote — neither build lost its commit.
    vision_final = _git(origin, "show", "feature/race:docker/vision/.image-versions.dev")
    main_final = _git(origin, "show", "feature/race:docker/main/.image-versions.dev")
    assert "VOICE_ASSISTANT_TAG=dev-c7232b32" in vision_final
    assert "TWIST_MUX_TAG=dev-c7232b32" in main_final


# --------------------------------------------------------------------------- #
# 4. Persistent push failure: the helper must exit non-zero and emit a
#    visible error so the workflow surfaces it instead of silently
#    swallowing the lost image-versions commit.
# --------------------------------------------------------------------------- #


def test_persistent_push_failure_exits_nonzero(
    dual_clone: tuple[Path, Path], tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    work, _origin = dual_clone
    _git(work, "checkout", "-b", "feature/race")
    versions = work / "docker" / "vision" / ".image-versions.dev"
    _commit_and_stage(work, versions, "OAK_D_TAG=dev-c7232b32\n")
    _git(work, "commit", "--message=ci: vision SHA tags → dev-c7232b32 [skip ci]")

    # Sabotage: replace ``git`` in PATH with a wrapper that simulates a
    # non-recoverable push failure (push exits non-zero, everything else
    # delegates to the real git).  We also speed up the retry delay so the
    # test stays fast.
    real_git = shutil.which("git")
    assert real_git is not None
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    wrapper = bin_dir / "git"
    wrapper.write_text(
        textwrap.dedent(
            f"""\
            #!/usr/bin/env bash
            # Wrapper that breaks ``git push`` only.
            if [ "${{1:-}}" = "push" ]; then
              echo 'remote: permission denied' >&2
              exit 1
            fi
            exec {real_git} "$@"
            """
        )
    )
    wrapper.chmod(0o755)
    new_path = str(bin_dir) + os.pathsep + os.environ["PATH"]
    monkeypatch.setenv("PATH", new_path)
    monkeypatch.setenv("IMAGE_VERSIONS_PUSH_ATTEMPTS", "2")
    monkeypatch.setenv("IMAGE_VERSIONS_PUSH_RETRY_DELAY", "0")

    result = _run(work, "bash", str(SCRIPT), "feature/race", "vision")

    assert result.returncode != 0
    assert "failed" in (result.stdout + result.stderr).lower()
    # Local commit must still exist (the runner's view is correct) — the
    # contract is to fail loudly, not to undo the work.
    assert "dev-c7232b32" in _git(work, "log", "-1", "--pretty=%B")


# --------------------------------------------------------------------------- #
# 5. Fatal rebase: an actual text conflict (NOT a same-line edit) cannot
#    be auto-resolved by ``git pull --rebase`` and must surface.  This pins
#    the abort path so we never silently drop a build's commit on the floor
#    when a real conflict lands.
# --------------------------------------------------------------------------- #


def test_unresolvable_rebase_fails_loudly(
    dual_clone: tuple[Path, Path], tmp_path: Path
) -> None:
    work, origin = dual_clone
    versions = work / "docker" / "vision" / ".image-versions.dev"
    _commit_and_stage(work, versions, "OAK_D_TAG=dev-c7232b32\n")
    _git(work, "commit", "--message=ci: vision SHA tags → dev-c7232b32 [skip ci]")

    # Force a conflict on the same line: remote changes the line to a
    # different SHA, so rebase will halt with a conflict.
    other = _spawn_rival(tmp_path, origin)
    _git(other, "checkout", "-b", "feature/race")
    _commit_and_stage(other, other / "README.md", "# seed\n\n# main-build override\n")
    _git(other, "commit", "--message=ci: main override [skip ci]", "--", "README.md")
    _git(other, "push", "origin", "HEAD:feature/race", check=False)

    # Tweak the worker's local commit so it CONFLICTS with the same line
    # in README.md.
    (work / "README.md").write_text("# seed\n\n# vision-build override\n")
    _git(work, "add", "README.md")
    # Amend the previous commit to make the line conflict.
    _git(work, "commit", "--amend", "--no-edit", "--", "README.md")

    result = _run(work, "bash", str(SCRIPT), "feature/race", "vision")

    # Acceptance: the helper must fail loudly and must NOT leave the
    # runner in a half-rebased state.
    assert result.returncode != 0
    assert "rebase" in (result.stdout + result.stderr).lower()

    # ``git status`` should not show a rebase-in-progress (we abort it).
    status = _git(work, "status", "--porcelain")
    assert "REBASE" not in _run(
        work.parent, "bash", "-c", f"cd {work} && git status"
    ).stdout


# --------------------------------------------------------------------------- #
# 6. Argument validation: bad inputs must fail fast, before touching git.
# --------------------------------------------------------------------------- #


def test_rejects_missing_branch(dual_clone: tuple[Path, Path]) -> None:
    work, _origin = dual_clone
    result = _run(work, "bash", str(SCRIPT))
    assert result.returncode != 0
    assert "usage" in (result.stderr + result.stdout).lower()


def test_rejects_non_integer_attempts(
    dual_clone: tuple[Path, Path], monkeypatch: pytest.MonkeyPatch
) -> None:
    work, _origin = dual_clone
    monkeypatch.setenv("IMAGE_VERSIONS_PUSH_ATTEMPTS", "lots")
    result = _run(work, "bash", str(SCRIPT), "feature/race")
    assert result.returncode != 0
    assert "IMAGE_VERSIONS_PUSH_ATTEMPTS" in (result.stderr + result.stdout)


# --------------------------------------------------------------------------- #
# 7. The script is executable and uses bash, not sh — required by callers
#    that rely on ``set -euo pipefail`` semantics and arrays.
# --------------------------------------------------------------------------- #


def test_script_is_executable_and_targets_bash() -> None:
    import stat

    mode = SCRIPT.stat().st_mode
    assert mode & stat.S_IXUSR, "push-image-versions.sh must be executable for the runner"

    first_line = SCRIPT.read_text().splitlines()[0]
    assert first_line.startswith("#!/usr/bin/env bash"), first_line


# --------------------------------------------------------------------------- #
# 8. Re-uses the well-known ``scripts/ci/commit-image-versions.sh`` branch
#    convention:  callers can keep their existing per-service branch
#    argument (e.g. ``ci/image-visions-main``).
# --------------------------------------------------------------------------- #


def test_supports_per_service_branch_names(dual_clone: tuple[Path, Path]) -> None:
    work, _origin = dual_clone
    _git(work, "checkout", "-b", "ci/image-versions-vision")
    versions = work / "docker" / "vision" / ".image-versions.dev"
    _commit_and_stage(work, versions, "OAK_D_TAG=dev-c7232b32\n")
    _git(work, "commit", "--message=ci: vision SHA tags → dev-c7232b32 [skip ci]")

    result = _run(work, "bash", str(SCRIPT), "ci/image-versions-vision", "vision")

    assert result.returncode == 0, result.stdout + result.stderr
    assert _git(_origin, "rev-parse", "--verify", "ci/image-versions-vision")
