"""Regression tests for issue #1482: ``.image-versions.{test,dev,latest}``
must point at a SHA-tag that is actually present in the local registry.

The previous workflows took ``SHORT_SHA=$(git rev-parse --short HEAD)`` inside
the ``update-image-versions`` job.  In a multi-workflow race, the local HEAD
could move on a round branch between checkout and the bash step — for example,
a parallel ``L: Build Single Service`` (or another batch run) pushed its
``ci: … SHA tags`` commit ahead of us, and the ``pull --rebase`` inside
``scripts/ci/push-image-versions.sh`` then put a *different* SHA at HEAD.
The resulting ``.image-versions.test`` named a tag that did not exist in the
registry, so ``docker compose pull`` failed and the robot fell back to a stale
image (round-163: ``VOICE_ASSISTANT_TAG=test-ad502e3`` instead of the real
``5b46874a`` — see issue #1482 evidence).

The contract pinned by these tests:

1. every workflow that writes ``.image-versions.{tag}`` derives its
   ``SHORT_SHA`` from ``${GITHUB_SHA::7}`` (which is the commit that
   *triggered* the run and is immutable for the lifetime of the run);
2. none of those workflows still rely on ``git rev-parse --short HEAD`` for
   the SHA, because that value can drift;
3. each such workflow verifies the SHA-image exists in the local registry
   *before* writing it to ``.image-versions`` — so a missing tag fails the
   job loudly instead of silently rolling back to a stale image at deploy
   time.

Why YAML-level checks instead of running the workflow: the workflows touch
``localhost:5000`` (private self-hosted registry) and run on
``[self-hosted, rob-box]`` runners.  We pin the *contract* here; the
self-hosted runner + a real round-164 build will exercise the end-to-end
behaviour as part of Acceptance C.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]

# Workflows that own the .image-versions.{test,dev,latest} SHA-tags written
# during a build (Main Pi, Vision Pi, Single Service).  ``L-Build All & Push
# to GHCR`` uses a different code path (push to ghcr.io with `imagetools
# create`) and is out of scope for the round-branch race fixed in #1482.
WORKFLOWS = [
    REPO_ROOT / ".github/workflows/L-Build Vision Pi Services.yml",
    REPO_ROOT / ".github/workflows/L-Build Main Pi Services.yml",
    REPO_ROOT / ".github/workflows/L-Build Single Service.yml",
]

# Steps we expect to use the new contract.  We assert against step names
# rather than job names so the test survives a future refactor of
# `jobs.<tag-with-sha-and-update-image-versions>`.
TAG_STEP_NAMES = ("Tag with SHA and update .image-versions",)


def _load(path: Path) -> dict:
    with path.open() as fh:
        return yaml.safe_load(fh)


def _find_tag_steps(workflow: dict) -> list[tuple[str, dict]]:
    out: list[tuple[str, dict]] = []
    for job_name, job_def in (workflow.get("jobs") or {}).items():
        for step in job_def.get("steps") or []:
            name = step.get("name") or ""
            if name in TAG_STEP_NAMES:
                out.append((job_name, step))
    return out


@pytest.mark.parametrize("workflow_path", WORKFLOWS, ids=lambda p: p.name)
def test_tag_step_exists(workflow_path: Path) -> None:
    """Each workflow must still contain a Tag-with-SHA step (sanity)."""
    workflow = _load(workflow_path)
    assert _find_tag_steps(workflow), (
        f"{workflow_path.name}: no 'Tag with SHA and update .image-versions' "
        "step found — workflow may have been renamed; update test or fix the "
        "contract."
    )


@pytest.mark.parametrize("workflow_path", WORKFLOWS, ids=lambda p: p.name)
def test_short_sha_uses_github_sha(workflow_path: Path) -> None:
    """``SHORT_SHA`` must be derived from ``${GITHUB_SHA::7}``.

    Acceptable patterns:
      * ``SHORT_SHA="${GITHUB_SHA::7}"``  (preferred; verbatim short hash)
    Forbidden patterns (these were the root cause in #1482):
      * ``git rev-parse --short HEAD``   (can drift under race / rebase)
      * ``git rev-parse --short HEAD^{...}`` (same drift risk)
      * any other ``git rev-parse`` invocation that depends on local HEAD
    """
    workflow = _load(workflow_path)
    for job_name, step in _find_tag_steps(workflow):
        run = step.get("run") or ""
        assert "GITHUB_SHA::7" in run, (
            f"{workflow_path.name} / job={job_name}: SHORT_SHA must use "
            "'${GITHUB_SHA::7}' so the value is immutable for the run "
            "(issue #1482)."
        )
        assert "rev-parse --short HEAD" not in run, (
            f"{workflow_path.name} / job={job_name}: 'git rev-parse --short "
            "HEAD' is unsafe — under race conditions the local HEAD moves and "
            "the recorded SHA no longer matches a real image "
            "(issue #1482, round-163 evidence)."
        )


@pytest.mark.parametrize("workflow_path", WORKFLOWS, ids=lambda p: p.name)
def test_registry_verify_present(workflow_path: Path) -> None:
    """Each tag step must verify the SHA-image exists in the local registry
    *before* writing it to ``.image-versions.{tag}``.

    Without this guard, a missing tag silently reaches the deploy job and
    the robot falls back to a stale image (round-163 fallback to
    ``voice-assistant-humble-test-83d4064f``).  We only require that the
    step talks to ``${LOCAL_REGISTRY}/v2/.../manifests/...`` — the precise
    helper name (``verify_in_registry``) is allowed to evolve.
    """
    workflow = _load(workflow_path)
    for job_name, step in _find_tag_steps(workflow):
        run = step.get("run") or ""
        # The exact tag name varies (NEW_TAG vs SHA_TAG); assert against the
        # common shape: a v2 manifests HEAD request against the registry.
        assert "/v2/" in run and "/manifests/" in run, (
            f"{workflow_path.name} / job={job_name}: registry manifest HEAD "
            "check missing — without it, a non-existent SHA-tag is written "
            "to .image-versions and the robot stays on a stale image "
            "(issue #1482)."
        )
        # 200-OK gate (or non-200 fail) must be present.
        assert re.search(r"http_code\s*=|HTTP_CODE=", run), (
            f"{workflow_path.name} / job={job_name}: registry check does "
            "not capture an HTTP status code — verify gate is incomplete."
        )