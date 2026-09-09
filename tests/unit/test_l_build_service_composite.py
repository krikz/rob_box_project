"""Unit tests for .github/actions/l-build-service/action.yml.

The composite action's bash step is responsible for assembling a correct
`docker buildx build` invocation from a set of multi-line inputs (tags,
build-args). We can't easily run the action inside a unit test, but we CAN
parse the action.yml, extract the bash script, and unit-test the input-
parsing / flag-assembly logic in isolation.

These tests guard against the regressions that motivated the refactor
(issue #2280): buildx --load (daemon, needed by update-image-versions) +
`docker push` ТОЛЬКО локального registry — GHCR никогда не пушится на
локальных test/dev сборках (runner не залогинен в ghcr.io, issue #1503;
run #34368750126 — buildx --push по GHCR-тегу падал unauthorized).
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

    # action.yml contains UTF-8 Russian comments — encoding must be explicit
    # on Windows (default cp1252 would raise UnicodeDecodeError).
    with ACTION_YML.open(encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _extract_build_step_bash(action: dict) -> str:
    """Return the `run:` body of the single Build step.

    The composite is buildx-only (clean+checkout live in the calling job), so
    there is exactly one step whose name starts with "Build ". Strips the
    leading indent so we can exec it via `bash -c` directly.
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
        # Marker line so tests can split the log back into per-invocation argv.
        f"printf '%s\\n' '#DOCKER-CALL#' >> \"{log}\"\n"
        "exit 0\n"
    )
    fake.chmod(0o755)
    # Prepend to PATH so the fake docker is found first
    monkeypatch.setenv("PATH", f"{tmp_path}:{__import__('os').environ['PATH']}")
    # git submodule status is consulted when compute-submodule-sha is set —
    # monkeypatching git is out of scope; tests that use it skip if git fails.
    return log


def _invocations(log_text: str) -> list[list[str]]:
    """Split the fake-docker log into per-invocation argv lists.

    The fake docker appends one argv per line and a '#DOCKER-CALL#' separator
    between invocations (buildx build, then one `docker push` per LOCAL tag).
    """
    calls: list[list[str]] = []
    cur: list[str] = []
    for line in log_text.splitlines():
        if line == "#DOCKER-CALL#":
            calls.append(cur)
            cur = []
        else:
            cur.append(line)
    if cur:
        calls.append(cur)
    return calls


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
    # writable so mktemp works. We pass the FULL os.environ (not a stripped
    # env): on Windows Git Bash fails to start without SystemRoot etc., so a
    # minimal {HOME, PATH} env would break these tests on dev machines (they
    # only passed on Linux CI where bash does not need Windows env vars).
    env = dict(__import__("os").environ)
    env["HOME"] = "/tmp"
    return subprocess.run(
        ["bash", "-e", "-c", expanded],
        capture_output=True,
        text=True,
        check=False,
        env=env,
    )


# --- schema tests ------------------------------------------------------


def test_action_yaml_is_loadable():
    """Smoke test: action.yml is valid YAML and has expected shape.

    Composite is buildx-ONLY: checkout must happen in the CALLING job. GitHub
    resolves `uses: ./.github/actions/...` from the job workspace, which exists
    only after actions/checkout ran — so a composite can never self-checkout.
    Putting Clean/Checkout inside the composite (and calling it as the job's
    first step) makes every job fail with "Can't find action.yml ... Did you
    forget to run actions/checkout" (regression: run #34366133083, 18/18 jobs).
    """
    action = _load_action_yaml()
    assert action["name"] == "L-Build Service (composite)"
    assert action["runs"]["using"] == "composite"
    step_names = [s["name"] for s in action["runs"]["steps"]]
    assert len(step_names) == 1, step_names
    assert step_names[0].startswith("Build "), step_names
    # No self-checkout / clean-stale inside the composite (impossible at runtime
    # + would be redundant with the job-level checkout).
    assert not any("checkout" in n.lower() for n in step_names), step_names
    assert not any("clean stale" in n.lower() for n in step_names), step_names


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
    """2 --tag + 1 --build-arg → buildx --load (both tags), then docker push
    ONLY the localhost:5000 tag (GHCR is never pushed on local builds)."""
    log = _fake_docker(monkeypatch, tmp_path)
    action = _load_action_yaml()
    bash_body = _extract_build_step_bash(action)
    cp = _run_build_step(
        bash_body,
        tags="localhost:5000/krikz/rob_box:robot-state-publisher-humble-test\nghcr.io/krikz/rob_box:robot-state-publisher-humble-test",
        build_args="APT_PROXY=http://host.docker.internal:3142",
    )
    assert cp.returncode == 0, f"build script failed:\nstderr: {cp.stderr}\nstdout: {cp.stdout}"
    calls = _invocations(log.read_text())
    build = next(c for c in calls if c[:2] == ["buildx", "build"])
    pushes = [c for c in calls if c and c[0] == "push"]
    # buildx build --load with both tags (GHCR + LOCAL)
    assert "--platform" in build and "linux/arm64" in build
    assert "--file" in build and "docker/main/robot_state_publisher/Dockerfile" in build
    # --load mode (default load=false builds into the daemon — update-image-versions
    # does `docker tag` from the daemon). Never --push (GHCR would be unauthorized).
    assert "--load" in build, f"expected --load in build argv: {build}"
    assert "--push" not in build, f"did not expect --push: {build}"
    # --tag appears twice (2 tags)
    tag_idx = [i for i, x in enumerate(build) if x == "--tag"]
    assert len(tag_idx) == 2, f"expected 2 --tag flags, got {len(tag_idx)}: {build}"
    assert "localhost:5000/krikz/rob_box:robot-state-publisher-humble-test" in build
    assert "ghcr.io/krikz/rob_box:robot-state-publisher-humble-test" in build
    # single --build-arg in KEY=VAL form
    ba_args = [x for x in build if x.startswith("--build-arg=")]
    assert ba_args == ["--build-arg=APT_PROXY=http://host.docker.internal:3142"], ba_args
    # docker push targets ONLY the LOCAL tag — GHCR is never pushed on local/test
    # builds (runner not logged into ghcr.io, issue #1503; buildx --push of the
    # GHCR tag failed with unauthorized in run #34368750126).
    assert len(pushes) == 1, f"expected exactly 1 docker push (LOCAL), got: {pushes}"
    assert pushes[0] == [
        "push",
        "localhost:5000/krikz/rob_box:robot-state-publisher-humble-test",
    ], pushes[0]


def test_ghcr_tag_is_never_pushed(monkeypatch, tmp_path):
    """Regression (run #34368750126): docker push must ONLY target the local
    registry. GHCR tags are loaded into the daemon (--load) but NOT pushed."""
    log = _fake_docker(monkeypatch, tmp_path)
    action = _load_action_yaml()
    bash_body = _extract_build_step_bash(action)
    cp = _run_build_step(
        bash_body,
        tags="ghcr.io/krikz/rob_box:svc-humble-test\nlocalhost:5000/krikz/rob_box:svc-humble-test",
        build_args="APT_PROXY=http://host.docker.internal:3142",
    )
    assert cp.returncode == 0, f"build script failed:\n{cp.stderr}\n{cp.stdout}"
    calls = _invocations(log.read_text())
    pushes = [c for c in calls if c and c[0] == "push"]
    assert pushes == [["push", "localhost:5000/krikz/rob_box:svc-humble-test"]], (
        f"expected ONLY localhost:5000 push, got: {pushes}"
    )


def test_load_true_yields_load_flag_not_push(monkeypatch, tmp_path):
    """load=true → buildx --load only, and NO docker push to the registry.

    Explicit escape hatch (см. action.yml): образ только в локальном docker
    daemon, например для `docker run` в том же job'е. Default load=false →
    buildx --load + docker push тегов ЛОКАЛЬНОГО registry (GHCR не пушится).
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