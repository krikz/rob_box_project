"""Unit tests for .github/actions/l-build-service/action.yml.

The composite action's bash step is responsible for assembling a correct
`docker buildx build` invocation from a set of multi-line inputs (tags,
build-args). We can't easily run the action inside a unit test, but we CAN
parse the action.yml, extract the bash script, and unit-test the input-
parsing / flag-assembly logic in isolation.

These tests guard against the regressions that motivated the refactor
(issue #2280 acceptance: «buildx --push вместо --load + docker push»).
"""

from __future__ import annotations

import re
import subprocess
import textwrap
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
ACTION_YML = REPO_ROOT / ".github" / "actions" / "l-build-service" / "action.yml"


# --- helpers -----------------------------------------------------------


def _load_action_yaml() -> dict:
    """Parse action.yml into a Python dict (uses stdlib yaml)."""
    import yaml

    with ACTION_YML.open() as fh:
        return yaml.safe_load(fh)


def _extract_build_step_bash(action: dict) -> str:
    """Return the `run:` body of the build step (skipping Clean / Checkout).

    Strips the leading indent so we can exec it via `bash -c` directly.
    """
    steps = action["runs"]["steps"]
    build_step = next(s for s in steps if s["name"].startswith("Build "))
    return textwrap.dedent(build_step["run"])


def _fake_docker(monkeypatch, tmp_path: Path) -> Path:
    """Replace `docker` with a fake that records its argv to a file.

    Returns the path of the recording file (the caller inspects argv via
    `cat <path>`). The fake exits 0 always (we're not testing error paths).
    """
    log = tmp_path / "docker_argv.log"
    fake = tmp_path / "docker"
    fake.write_text(
        "#!/usr/bin/env bash\n"
        f"printf '%s\\n' \"$@\" >> \"{log}\"\n"
        "exit 0\n"
    )
    fake.chmod(0o755)
    # Prepend to PATH so the fake docker is found first
    monkeypatch.setenv("PATH", f"{tmp_path}:{__import__('os').environ['PATH']}")
    # git submodule status is consulted when compute-submodule-sha is set —
    # monkeypatching git is out of scope; tests that use it skip if git fails.
    return log


def _fake_git_submodule(monkeypatch, tmp_path: Path, sha: str = "abc1234567890") -> Path:
    """Replace `git` so that `git submodule status <path>` prints `<sha> <path>`.

    We only intercept `git submodule status`; everything else falls through
    to system git (so tests can still use git for repo setup if needed).
    """
    log = tmp_path / "git_submodule.log"
    shim = tmp_path / "git"
    shim.write_text(
        "#!/usr/bin/env bash\n"
        'if [ "$1" = "submodule" ] && [ "$2" = "status" ]; then\n'
        # git submodule status prints "<sha> <path>" (sha FIRST), so
        # downstream `awk '{print $1}'` picks up the sha. Fake must mirror.
        f'printf "{sha} %s\\n" "$3"\n'
        "exit 0\n"
        "fi\n"
        # Pass through everything else (incl. --version, etc.)\n"
        'exec /usr/bin/git "$@"\n'
    )
    shim.chmod(0o755)
    monkeypatch.setenv("PATH", f"{tmp_path}:{__import__('os').environ['PATH']}")
    return log


def _run_build_step(
    bash_body: str,
    *,
    service_name: str = "robot-state-publisher",
    dockerfile_path: str = "docker/main/robot_state_publisher/Dockerfile",
    build_context: str = ".",
    platform: str = "linux/arm64",
    tags: str = "",
    build_args: str = "",
    compute_submodule_sha: str = "",
    load: str = "false",
    add_host: str = "host.docker.internal:host-gateway",
    progress: str = "plain",
) -> subprocess.CompletedProcess:
    """Substitute `${{ inputs.* }}` placeholders in bash_body with values.

    This mirrors what GitHub Actions does at job-step expansion time: it
    replaces `${{ inputs.X }}` with the literal value of the input. We
    emulate that substitution here, then exec the result via `bash -e`.
    """
    substitutions = {
        "${{ inputs.service-name }}": service_name,
        "${{ inputs.dockerfile-path }}": dockerfile_path,
        "${{ inputs.build-context }}": build_context,
        "${{ inputs.platform }}": platform,
        "${{ inputs.tags }}": tags,
        "${{ inputs.build-args }}": build_args,
        "${{ inputs.compute-submodule-sha }}": compute_submodule_sha,
        "${{ inputs.load }}": load,
        "${{ inputs.add-host }}": add_host,
        "${{ inputs.progress }}": progress,
    }
    expanded = bash_body
    for placeholder, value in substitutions.items():
        expanded = expanded.replace(placeholder, value)

    # Run with -e (errexit, like set -euo pipefail). Set HOME to something
    # writable so mktemp works.
    return subprocess.run(
        ["bash", "-e", "-c", expanded],
        capture_output=True,
        text=True,
        check=False,
        env={"HOME": "/tmp", "PATH": __import__("os").environ["PATH"]},
    )


# --- schema tests ------------------------------------------------------


def test_action_yaml_is_loadable():
    """Smoke test: action.yml is valid YAML and has expected shape."""
    action = _load_action_yaml()
    assert action["name"] == "L-Build Service (composite)"
    assert action["runs"]["using"] == "composite"
    step_names = [s["name"] for s in action["runs"]["steps"]]
    assert any("Clean stale submodules" in n for n in step_names), step_names
    assert "Checkout repository" in step_names, step_names
    assert any(n.startswith("Build ") for n in step_names), step_names


def test_action_declares_expected_inputs():
    """All inputs referenced by the bash step are declared in `inputs:`."""
    action = _load_action_yaml()
    declared = set(action["inputs"].keys())
    bash_body = _extract_build_step_bash(action)
    referenced = set(re.findall(r"\$\{\{\s*inputs\.([\w-]+)\s*\}\}", bash_body))
    # Both must be non-empty and referenced must be subset of declared.
    assert referenced, "expected at least one input reference in build step"
    assert referenced <= declared, (
        f"build step references undeclared inputs: {referenced - declared}"
    )


# --- behaviour tests ---------------------------------------------------


def test_build_with_two_tags_and_one_build_arg(monkeypatch, tmp_path):
    """Multi-line inputs parse correctly: 2 --tag + 1 --build-arg."""
    log = _fake_docker(monkeypatch, tmp_path)
    action = _load_action_yaml()
    bash_body = _extract_build_step_bash(action)
    cp = _run_build_step(
        bash_body,
        tags="localhost:5000/krikz/rob_box:robot-state-publisher-humble-test\nghcr.io/krikz/rob_box:robot-state-publisher-humble-test",
        build_args="APT_PROXY=http://host.docker.internal:3142",
    )
    assert cp.returncode == 0, f"build script failed:\nstderr: {cp.stderr}\nstdout: {cp.stdout}"
    argv = log.read_text().splitlines()
    # buildx build followed by --platform linux/arm64 --file <...> ...
    assert argv[0] == "buildx"
    assert argv[1] == "build"
    assert "--platform" in argv and "linux/arm64" in argv
    assert "--file" in argv and "docker/main/robot_state_publisher/Dockerfile" in argv
    # --push mode (default load=false)
    assert "--push" in argv, f"expected --push in argv: {argv}"
    assert "--load" not in argv, f"did not expect --load: {argv}"
    # --tag appears twice (2 tags)
    tag_idx = [i for i, x in enumerate(argv) if x == "--tag"]
    assert len(tag_idx) == 2, f"expected 2 --tag flags, got {len(tag_idx)}"
    assert "localhost:5000/krikz/rob_box:robot-state-publisher-humble-test" in argv
    assert "ghcr.io/krikz/rob_box:robot-state-publisher-humble-test" in argv
    # --build-arg appears once (single-element form: "--build-arg=KEY=VAL")
    ba_idx = [i for i, x in enumerate(argv) if x.startswith("--build-arg=")]
    assert len(ba_idx) == 1, f"expected 1 --build-arg=…, got {len(ba_idx)}: {argv}"
    # The KEY=VAL form lives inside the single element
    assert "--build-arg=APT_PROXY=http://host.docker.internal:3142" in argv
    # build context is last positional arg
    assert argv[-1] == "."


def test_load_true_yields_load_flag_not_push(monkeypatch, tmp_path):
    """When load=true, the script uses --load and NOT --push.

    This is the explicit escape hatch — see action.yml docstring for
    load=true use case («docker run после build»). Default load=false
    uses --push which is the issue-2280 recommendation.
    """
    log = _fake_docker(monkeypatch, tmp_path)
    action = _load_action_yaml()
    bash_body = _extract_build_step_bash(action)
    cp = _run_build_step(
        bash_body,
        tags="localhost:5000/krikz/rob_box:foo-test",
        load="true",
    )
    assert cp.returncode == 0, f"build script failed:\n{cp.stderr}\n{cp.stdout}"
    argv = log.read_text().splitlines()
    assert "--load" in argv, f"expected --load when load=true: {argv}"
    assert "--push" not in argv, f"did not expect --push when load=true: {argv}"


def test_compute_submodule_sha_adds_uppercase_sha_arg(monkeypatch, tmp_path):
    """compute-submodule-sha=src/ros2leds → ROS2LEDS_SHA=<sha> build-arg."""
    log = _fake_docker(monkeypatch, tmp_path)
    _fake_git_submodule(monkeypatch, tmp_path, sha="deadbeef1234")
    action = _load_action_yaml()
    bash_body = _extract_build_step_bash(action)
    cp = _run_build_step(
        bash_body,
        tags="localhost:5000/krikz/rob_box:led-matrix-humble-test",
        compute_submodule_sha="src/ros2leds",
    )
    assert cp.returncode == 0, f"build script failed:\n{cp.stderr}\n{cp.stdout}"
    argv = log.read_text().splitlines()
    # The build-arg for the submodule SHA should be present
    expected = "ROS2LEDS_SHA=deadbeef1234"
    assert any(expected in x for x in argv), (
        f"expected '{expected}' in docker argv, got: {argv}"
    )


def test_empty_inputs_skip_tags_and_build_args(monkeypatch, tmp_path):
    """Empty tags + build-args → no --tag and no --build-arg flags.

    Edge case: a job with no tags should still execute docker buildx,
    even though it's a nonsense invocation. This test pins current
    behaviour so we notice if it changes.
    """
    log = _fake_docker(monkeypatch, tmp_path)
    action = _load_action_yaml()
    bash_body = _extract_build_step_bash(action)
    cp = _run_build_step(
        bash_body,
        tags="",
        build_args="",
    )
    # Note: docker buildx build with no --tag will fail in practice, but
    # our fake docker exits 0 — we only care about the assembled argv.
    assert cp.returncode == 0, f"build script failed:\n{cp.stderr}\n{cp.stdout}"
    argv = log.read_text().splitlines()
    assert "--tag" not in argv, f"did not expect --tag for empty tags input: {argv}"
    assert not any(x.startswith("--build-arg=") for x in argv), (
        f"did not expect --build-arg=… for empty build-args input: {argv}"
    )


def test_multi_line_build_args_each_get_their_own_build_arg_flag(
    monkeypatch, tmp_path
):
    """Each line in build-args becomes its own --build-arg flag.

    Acceptance B from issue #2280: the per-job build-arg lists vary by
    service (BASE_IMAGE, APT_PROXY, ROS2LEDS_SHA, etc.). The action must
    not lose any of them, and not coalesce them into a single arg.
    """
    log = _fake_docker(monkeypatch, tmp_path)
    action = _load_action_yaml()
    bash_body = _extract_build_step_bash(action)
    cp = _run_build_step(
        bash_body,
        tags="localhost:5000/krikz/rob_box:test",
        build_args=(
            "BASE_IMAGE=localhost:5000/krikz/rob_box_base:ros2-zenoh-humble\n"
            "APT_PROXY=http://host.docker.internal:3142\n"
            "URDF_FILES_HASH=deadbeef\n"
        ),
    )
    assert cp.returncode == 0, f"build script failed:\n{cp.stderr}\n{cp.stdout}"
    argv = log.read_text().splitlines()
    build_arg_values = {
        x[len("--build-arg="):]
        for x in argv
        if x.startswith("--build-arg=")
    }
    assert "BASE_IMAGE=localhost:5000/krikz/rob_box_base:ros2-zenoh-humble" in build_arg_values
    assert "APT_PROXY=http://host.docker.internal:3142" in build_arg_values
    assert "URDF_FILES_HASH=deadbeef" in build_arg_values