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

import os
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


def _fake_docker(
    monkeypatch,
    tmp_path: Path,
    *,
    builder_driver: str = "docker-container",
    containerd_store: bool = False,
    builder_exists: bool = True,
) -> Path:
    """Replace `docker` with a fake that records its argv to a file.

    Returns the path of the recording file (the caller inspects argv via
    `cat <path>`). The fake exits 0 always (we're not testing error paths).

    Two read-only probes are answered instead of being recorded, because the
    cache block (план 2026-09-15-builder-runtime-seam.md §6) asks the real
    docker whether the current buildx driver can export cache:

      * `docker buildx inspect`     → "Driver: <builder_driver>"
      * `docker info --format ...`  → DriverStatus, optionally containerd

    They are deliberately NOT appended to the argv log so that `_invocations`
    keeps returning exactly the build + push calls the older tests expect.
    """
    log = tmp_path / "docker_argv.log"
    fake = tmp_path / "docker"
    driver_status = (
        "[[driver-type io.containerd.snapshotter.v1]]"
        if containerd_store
        else "[[Backing Filesystem extfs] [Supports d_type true]]"
    )
    fake.write_text(
        "#!/usr/bin/env bash\n"
        # Probe 1: `docker buildx inspect` (no build subcommand) — answer with
        # a Driver: line in the same shape real buildx prints.
        'if [ "$1" = "buildx" ] && [ "$2" = "inspect" ]; then\n'
        # `buildx inspect <name>` — проверка «а есть ли такой билдер».
        # Отсутствующий билдер обязан отвечать ненулевым кодом, иначе ветку
        # создания билдера (docker-container для registry-кеша) не проверить.
        + (
            ""
            if builder_exists
            else '  if [ -n "$3" ]; then exit 1; fi\n'
        )
        + f"  printf 'Name:          fake\\nDriver:        {builder_driver}\\n'\n"
        "  exit 0\n"
        "fi\n"
        # `buildx use` — тоже проба, не записываем: иначе старые тесты,
        # считающие ровно build+push вызовы, начнут видеть лишний.
        'if [ "$1" = "buildx" ] && [ "$2" = "use" ]; then\n'
        "  exit 0\n"
        "fi\n"
        # Probe 2: `docker info --format {{ .DriverStatus }}`.
        'if [ "$1" = "info" ]; then\n'
        f"  printf '%s\\n' '{driver_status}'\n"
        "  exit 0\n"
        "fi\n"
        f"printf '%s\\n' \"$@\" >> \"{log}\"\n"
        # Marker line so tests can split the log back into per-invocation argv.
        f"printf '%s\\n' '#DOCKER-CALL#' >> \"{log}\"\n"
        "exit 0\n"
    )
    fake.chmod(0o755)
    # Prepend to PATH so the fake docker is found first. Use os.pathsep (':' on
    # Linux, ';' on Windows) so Git Bash on Windows can still resolve the fake.
    monkeypatch.setenv("PATH", f"{tmp_path}{os.pathsep}{__import__('os').environ['PATH']}")
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
    monkeypatch.setenv("PATH", f"{tmp_path}{os.pathsep}{__import__('os').environ['PATH']}")
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
    local_registry: str = "localhost:5000",
    cache: str = "true",
    cache_ref: str = "",
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
        "${{ inputs.local-registry }}": local_registry,
        "${{ inputs.cache }}": cache,
        "${{ inputs.cache-ref }}": cache_ref,
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
        # Script output contains UTF-8 (emoji/✅); on Windows text=True would
        # otherwise decode with cp1252 and raise UnicodeDecodeError.
        encoding="utf-8",
        errors="replace",
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


# --- registry cache tests ----------------------------------------------
#
# docs/plans/2026-09-15-builder-runtime-seam.md §6 / Этап 0: до этого
# изменения action.yml не содержал ни --cache-from, ни --cache-to (§6.1),
# поэтому cold build после `docker builder prune` / переустановки раннера
# платил полную цену пересборки (§6.5).


def _flag_values(argv: list[str], flag: str) -> list[str]:
    """Values of a repeated `--flag value` pair in an argv list."""
    return [argv[i + 1] for i, x in enumerate(argv) if x == flag and i + 1 < len(argv)]


def _build_argv(log: Path) -> list[str]:
    """argv of the single `docker buildx build` invocation in the fake log."""
    return next(c for c in _invocations(log.read_text()) if c[:2] == ["buildx", "build"])


TAGS_TWO = (
    "ghcr.io/krikz/rob_box:led-matrix-humble-test\n"
    "localhost:5000/krikz/rob_box:led-matrix-humble-test"
)


def test_cache_ref_defaults_to_local_registry_buildcache_tag(monkeypatch, tmp_path):
    """Default cache-ref is derived from the LOCAL tag, not hardcoded (§6.4).

    localhost:5000/krikz/rob_box:led-matrix-humble-test
      → localhost:5000/krikz/rob_box:led-matrix-buildcache
    The repo part (krikz/rob_box) comes from the tag the workflow passed
    (env.LOCAL_PREFIX), so the action never duplicates it.
    """
    log = _fake_docker(monkeypatch, tmp_path)
    bash_body = _extract_build_step_bash(_load_action_yaml())
    cp = _run_build_step(bash_body, service_name="led-matrix", tags=TAGS_TWO)
    assert cp.returncode == 0, f"build script failed:\n{cp.stderr}\n{cp.stdout}"
    build = _build_argv(log)
    expected_ref = "localhost:5000/krikz/rob_box:led-matrix-buildcache"
    assert _flag_values(build, "--cache-from") == [
        f"type=registry,ref={expected_ref}"
    ], build
    assert _flag_values(build, "--cache-to") == [
        f"type=registry,ref={expected_ref},mode=max,ignore-error=true"
    ], build


def test_cache_to_uses_mode_max(monkeypatch, tmp_path):
    """mode=max is mandatory (§6.3).

    Without it BuildKit exports only the FINAL image's layers, so the
    intermediate builder stages of a multi-stage build are not cached at
    all — which is exactly what stages 1+ of the plan need.
    """
    log = _fake_docker(monkeypatch, tmp_path)
    bash_body = _extract_build_step_bash(_load_action_yaml())
    cp = _run_build_step(bash_body, service_name="led-matrix", tags=TAGS_TWO)
    assert cp.returncode == 0, f"build script failed:\n{cp.stderr}\n{cp.stdout}"
    cache_to = _flag_values(_build_argv(log), "--cache-to")
    assert cache_to and "mode=max" in cache_to[0], cache_to


def test_cache_to_sets_ignore_error(monkeypatch, tmp_path):
    """ignore-error=true so the FIRST run does not fail.

    Verified against real buildx v0.17.1 with an unreachable registry:
    `--cache-to type=registry,...` without ignore-error exits 1 ("error
    writing layer blob"), with ignore-error=true it exits 0 and the --load
    image still lands in the daemon. `--cache-from` on a missing ref is
    harmless on its own (exit 0).
    """
    log = _fake_docker(monkeypatch, tmp_path)
    bash_body = _extract_build_step_bash(_load_action_yaml())
    cp = _run_build_step(bash_body, service_name="led-matrix", tags=TAGS_TWO)
    assert cp.returncode == 0, f"build script failed:\n{cp.stderr}\n{cp.stdout}"
    cache_to = _flag_values(_build_argv(log), "--cache-to")
    assert cache_to and "ignore-error=true" in cache_to[0], cache_to


def test_cache_ref_is_a_separate_tag_from_the_image_tags(monkeypatch, tmp_path):
    """The cache manifest must NOT overwrite the image in the registry.

    `<service>-buildcache` never collides with `<service>-<distro>-<tag>`.
    """
    log = _fake_docker(monkeypatch, tmp_path)
    bash_body = _extract_build_step_bash(_load_action_yaml())
    cp = _run_build_step(bash_body, service_name="led-matrix", tags=TAGS_TWO)
    assert cp.returncode == 0, f"build script failed:\n{cp.stderr}\n{cp.stdout}"
    build = _build_argv(log)
    image_tags = set(_flag_values(build, "--tag"))
    for spec in _flag_values(build, "--cache-from") + _flag_values(build, "--cache-to"):
        ref = spec.split("ref=", 1)[1].split(",", 1)[0]
        assert ref not in image_tags, f"cache ref {ref} collides with an image tag"


def test_cache_does_not_break_load_and_local_push(monkeypatch, tmp_path):
    """§6.5: caching is orthogonal to --load + `docker push` of LOCAL tags.

    The image must still be exported into the local docker daemon (needed by
    update-image-versions' `docker tag`) and only the localhost:5000 tag may
    be pushed — GHCR stays unauthorized (issue #1503).
    """
    log = _fake_docker(monkeypatch, tmp_path)
    bash_body = _extract_build_step_bash(_load_action_yaml())
    cp = _run_build_step(bash_body, service_name="led-matrix", tags=TAGS_TWO)
    assert cp.returncode == 0, f"build script failed:\n{cp.stderr}\n{cp.stdout}"
    calls = _invocations(log.read_text())
    build = _build_argv(log)
    assert "--load" in build, build
    assert "--push" not in build, build
    pushes = [c for c in calls if c and c[0] == "push"]
    assert pushes == [["push", "localhost:5000/krikz/rob_box:led-matrix-humble-test"]], pushes


def test_cache_false_disables_all_cache_flags(monkeypatch, tmp_path):
    """cache=false → no cache flags at all (forced-rebuild escape hatch)."""
    log = _fake_docker(monkeypatch, tmp_path)
    bash_body = _extract_build_step_bash(_load_action_yaml())
    cp = _run_build_step(
        bash_body, service_name="led-matrix", tags=TAGS_TWO, cache="false"
    )
    assert cp.returncode == 0, f"build script failed:\n{cp.stderr}\n{cp.stdout}"
    build = _build_argv(log)
    assert "--cache-from" not in build, build
    assert "--cache-to" not in build, build


def test_explicit_cache_ref_overrides_the_derived_default(monkeypatch, tmp_path):
    """An explicit cache-ref wins over the tag-derived default.

    Needed by stages 3-4 of the plan: vesc_nexus + ros2_control are meant to
    SHARE one builder cache ref, which cannot be derived per-service.
    """
    log = _fake_docker(monkeypatch, tmp_path)
    bash_body = _extract_build_step_bash(_load_action_yaml())
    cp = _run_build_step(
        bash_body,
        service_name="vesc-nexus",
        tags=TAGS_TWO,
        cache_ref="localhost:5000/krikz/rob_box:shared-builder-buildcache",
    )
    assert cp.returncode == 0, f"build script failed:\n{cp.stderr}\n{cp.stdout}"
    build = _build_argv(log)
    assert _flag_values(build, "--cache-from") == [
        "type=registry,ref=localhost:5000/krikz/rob_box:shared-builder-buildcache"
    ], build
    assert _flag_values(build, "--cache-to") == [
        "type=registry,ref=localhost:5000/krikz/rob_box:shared-builder-buildcache"
        ",mode=max,ignore-error=true"
    ], build


def _buildx_create_argv(log: Path) -> list[str]:
    """argv единственного вызова `docker buildx create`, [] если его не было."""
    for call in _invocations(log.read_text(encoding="utf-8") if log.exists() else ""):
        if call[:2] == ["buildx", "create"]:
            return call
    return []


def test_buildx_builder_is_created_when_missing(monkeypatch, tmp_path):
    """Билдер docker-container заводится самим action'ом, а не руками на хосте.

    Раннеры на katana — контейнеры (myoung34/github-runner с проброшенным
    docker.sock), у каждого свой ~/.docker. Билдер, созданный руками на хосте,
    им не виден; созданный внутри контейнера — умирает вместе с ним. Поэтому
    единственное место, которое переживает и то и другое, — этот шаг.
    """
    log = _fake_docker(monkeypatch, tmp_path, builder_exists=False)
    bash_body = _extract_build_step_bash(_load_action_yaml())
    cp = _run_build_step(bash_body, service_name="led-matrix", tags=TAGS_TWO)
    assert cp.returncode == 0, f"build script failed:\n{cp.stderr}\n{cp.stdout}"

    create = _buildx_create_argv(log)
    assert create, "docker buildx create не был вызван"
    assert "--driver" in create and "docker-container" in create, create
    # network=host: иначе buildkit в своём контейнере не увидит ни
    # localhost:5000 (куда пишется кеш), ни apt-прокси.
    assert "network=host" in create, create
    # --config: локальный registry без TLS, без buildkitd.toml экспорт кеша
    # молча уходил бы в ignore-error-warning.
    assert "--config" in create, create
    assert "--bootstrap" in create, create


def test_buildx_builder_is_reused_when_present(monkeypatch, tmp_path):
    """Существующий билдер не пересоздаётся на каждой сборке."""
    log = _fake_docker(monkeypatch, tmp_path, builder_exists=True)
    bash_body = _extract_build_step_bash(_load_action_yaml())
    cp = _run_build_step(bash_body, service_name="led-matrix", tags=TAGS_TWO)
    assert cp.returncode == 0, f"build script failed:\n{cp.stderr}\n{cp.stdout}"
    assert _buildx_create_argv(log) == [], "билдер пересоздан, хотя уже был"


def test_cache_export_skipped_on_plain_docker_driver(monkeypatch, tmp_path):
    """Default `docker` buildx driver cannot export cache — skip --cache-to.

    Real buildx v0.17.1 refuses BEFORE the build even starts: "Cache export
    is not supported for the docker driver" (exit 1), and ignore-error=true
    does NOT rescue it — that option is handled inside buildkit, while this
    is a buildx driver-feature check. So the action probes the driver and
    keeps only --cache-from, which is always safe.
    """
    log = _fake_docker(monkeypatch, tmp_path, builder_driver="docker")
    bash_body = _extract_build_step_bash(_load_action_yaml())
    cp = _run_build_step(bash_body, service_name="led-matrix", tags=TAGS_TWO)
    assert cp.returncode == 0, f"build script failed:\n{cp.stderr}\n{cp.stdout}"
    build = _build_argv(log)
    assert "--cache-to" not in build, build
    assert _flag_values(build, "--cache-from") == [
        "type=registry,ref=localhost:5000/krikz/rob_box:led-matrix-buildcache"
    ], build
    assert "cache export" in cp.stdout, cp.stdout


def test_cache_export_enabled_on_docker_driver_with_containerd_store(
    monkeypatch, tmp_path
):
    """`docker` driver + containerd image store CAN export cache → --cache-to."""
    log = _fake_docker(
        monkeypatch, tmp_path, builder_driver="docker", containerd_store=True
    )
    bash_body = _extract_build_step_bash(_load_action_yaml())
    cp = _run_build_step(bash_body, service_name="led-matrix", tags=TAGS_TWO)
    assert cp.returncode == 0, f"build script failed:\n{cp.stderr}\n{cp.stdout}"
    assert _flag_values(_build_argv(log), "--cache-to") == [
        "type=registry,ref=localhost:5000/krikz/rob_box:led-matrix-buildcache"
        ",mode=max,ignore-error=true"
    ], _build_argv(log)


def test_cache_disabled_when_no_local_tag_to_derive_from(monkeypatch, tmp_path):
    """No localhost:5000 tag → nothing to derive a ref from → no cache flags.

    Fail-soft: the build still runs, with a warning instead of a bogus ref.
    """
    log = _fake_docker(monkeypatch, tmp_path)
    bash_body = _extract_build_step_bash(_load_action_yaml())
    cp = _run_build_step(
        bash_body,
        service_name="led-matrix",
        tags="ghcr.io/krikz/rob_box:led-matrix-humble-test",
    )
    assert cp.returncode == 0, f"build script failed:\n{cp.stderr}\n{cp.stdout}"
    build = _build_argv(log)
    assert "--cache-from" not in build, build
    assert "--cache-to" not in build, build