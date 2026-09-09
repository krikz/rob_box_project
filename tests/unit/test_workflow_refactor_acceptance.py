"""Regression tests for the refactored L-Build Main/Vision Pi Services workflows.

Issue #2280 acceptance:
- каждый build-job в L-Build Main Pi Services.yml и L-Build Vision Pi
  Services.yml вызывает ./.github/actions/l-build-service для сборки (buildx
  --load + docker push ТОЛЬКО локального registry, единая сборка флагов) +
  опциональный pre-step для per-service cache-invalidation хешей.
- GHCR никогда не пушится на локальных test/dev сборках: runner не залогинен
  в ghcr.io (issue #1503) — buildx --push по GHCR-тегу падал unauthorized
  (run #34368750126). Поэтому образ собирается в daemon (--load), а пушится
  только LOCAL-тег (localhost:5000); update-image-versions делает `docker tag`
  из локального daemon.

ВАЖНО (fix run #34366133083): composite action — buildx-only. GitHub
требует, чтобы репозиторий был зачекаутен ДО вызова локального composite
action, поэтому каждый build-job начинается с clean-stale submodules +
checkout (как и было до рефакторинга), а composite вызывается ПОСЛЕ.

Эти тесты гарантируют, что:
1. все ожидаемые build-job'ы остались в файлах (никто не потерялся при
   копи-паст рефакторинге),
2. каждый build-job вызывает наш composite action (а не пишет inline
   `docker buildx build ... --load ...`),
3. каждый build-job делает checkout ДО вызова composite (иначе локальный
   action не находится — run #34366133083, все 18 build-job'ов упали),
4. composite action получает ожидаемые теги (LOCAL_PREFIX/IMAGE_PREFIX,
   тот же dockerfile-path, тот же build-context что был в прежнем коде),
5. composite action получает ожидаемые build-args (BASE_IMAGE, APT_PROXY,
   SOURCE_HASH/URDF_FILES_HASH/VESC_NEXUS_SHA/ROS2LEDS_SHA — где были).
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
MAIN_WF = REPO_ROOT / ".github" / "workflows" / "L-Build Main Pi Services.yml"
VISION_WF = REPO_ROOT / ".github" / "workflows" / "L-Build Vision Pi Services.yml"


# --- expected inventory (must match issue #2280 raw evidence) ----------


EXPECTED_MAIN_JOBS = {
    "build-robot-state-publisher",
    "build-rtabmap",
    "build-twist-mux",
    "build-teleop",
    "build-ros2-control",
    "build-nav2",
    "build-lslidar",
    "build-perception",
}

EXPECTED_VISION_JOBS = {
    "build-oak-d",
    "build-led-matrix",
    "build-ceiling-camera",
    "build-voice-assistant",
    "build-voice-resources",
    "build-voice-base",
    "build-telegram-bot",
    "build-supercollider",
    "build-supervisor",
    "build-quest",
}


# --- helpers -----------------------------------------------------------


def _load(path: Path) -> dict:
    # Workflow YAML contains UTF-8 Russian comments; on Windows the default
    # codepage is cp1252, so encoding must be explicit (CI/Linux defaults to
    # utf-8, which is why this only bites on Windows dev machines).
    with path.open(encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _uses_composite(steps: list[dict]) -> bool:
    """True iff at least one step uses the l-build-service composite action."""
    return any(
        (s.get("uses") or "").endswith("/actions/l-build-service")
        for s in steps
    )


def _composite_step(steps: list[dict]) -> dict | None:
    """Return the step that invokes the composite action (or None)."""
    for s in steps:
        if (s.get("uses") or "").endswith("/actions/l-build-service"):
            return s
    return None


def _expected_tags(job_name: str, pi_type: str) -> list[str]:
    """Build the LOCAL_TAG / GHCR_TAG for a job, like the old workflow did.

    `IMAGE_TAG = test|dev|local|latest`, depends on inputs. Tests use
    placeholder `DOCKER_TAG_PLACEHOLDER` (replaced below in assertion).
    """
    svc = job_name.removeprefix("build-")  # 'twist-mux', 'voice-assistant', ...
    # Old logic per pi_type:
    #   GHCR_TAG = "${IMAGE_PREFIX}:<service>-${ROS_DISTRO}-${docker_tag}"
    #   LOCAL_TAG = "${LOCAL_PREFIX}:<service>-${ROS_DISTRO}-${docker_tag}"
    # For some services (robot-state-publisher, twist-mux, ros2-control,
    # voice-assistant, telegram-bot, supervisor, quest) the image_name
    # uses hyphens instead of underscores.
    name_map = {
        "robot-state-publisher": ("robot_state_publisher", "robot-state-publisher"),
        "rtabmap": ("rtabmap", "rtabmap"),
        "twist-mux": ("twist_mux", "twist-mux"),
        "teleop": ("teleop", "teleop"),
        "ros2-control": ("ros2_control", "ros2-control"),
        "nav2": ("nav2", "nav2"),
        "lslidar": ("lslidar", "lslidar"),
        "perception": ("perception", "perception"),
        "oak-d": ("oak-d", "oak-d"),
        "led-matrix": ("led_matrix", "led-matrix"),
        "ceiling-camera": ("ceiling-camera", "ceiling-camera"),
        "voice-assistant": ("voice_assistant", "voice-assistant"),
        "voice-resources": ("voice_resources", "voice-resources"),
        "voice-base": ("voice_base", "voice-base"),
        "telegram-bot": ("telegram_bot", "telegram-bot"),
        "supercollider": ("supercollider", "supercollider"),  # no ROS_DISTRO!
        "supervisor": ("supervisor", "supervisor"),
        "quest": ("quest", "quest"),
    }
    svc_name, img_name = name_map[svc]

    # supercollider uses ${SERVICE}-${docker_tag} (NO ROS_DISTRO)
    if svc == "supercollider":
        ghcr = f"${{IMAGE_PREFIX}}:{svc_name}-${{DOCKER_TAG_PLACEHOLDER}}"
        local = f"${{LOCAL_PREFIX}}:{svc_name}-${{DOCKER_TAG_PLACEHOLDER}}"
    else:
        ghcr = f"${{IMAGE_PREFIX}}:{img_name}-${{ROS_DISTRO}}-${{DOCKER_TAG_PLACEHOLDER}}"
        local = f"${{LOCAL_PREFIX}}:{img_name}-${{ROS_DISTRO}}-${{DOCKER_TAG_PLACEHOLDER}}"
    return [ghcr, local]


# --- common assertions --------------------------------------------------


def test_main_workflow_all_build_jobs_present():
    """All 8 expected Main Pi build jobs are still defined."""
    data = _load(MAIN_WF)
    actual = {k for k in data["jobs"] if k.startswith("build-")}
    assert actual == EXPECTED_MAIN_JOBS, (
        f"Main Pi build jobs changed. Expected {EXPECTED_MAIN_JOBS}, got {actual}"
    )


def test_vision_workflow_all_build_jobs_present():
    """All 10 expected Vision Pi build jobs are still defined."""
    data = _load(VISION_WF)
    actual = {k for k in data["jobs"] if k.startswith("build-")}
    assert actual == EXPECTED_VISION_JOBS, (
        f"Vision Pi build jobs changed. Expected {EXPECTED_VISION_JOBS}, got {actual}"
    )


@pytest.mark.parametrize("job_name", sorted(EXPECTED_MAIN_JOBS | EXPECTED_VISION_JOBS))
def test_each_build_job_uses_composite_action(job_name):
    """Every build-* job MUST delegate to ./.github/actions/l-build-service.

    This is the headline acceptance of issue #2280 — если какой-то job
    остался с inline `docker buildx build --load ...`, рефакторинг был
    неполный и кто-то копипастит назад.
    """
    src = MAIN_WF if job_name in EXPECTED_MAIN_JOBS else VISION_WF
    data = _load(src)
    job = data["jobs"][job_name]
    assert _uses_composite(job["steps"]), (
        f"{job_name}: expected at least one step using "
        f"./.github/actions/l-build-service, got steps: "
        f"{[s.get('name') or s.get('uses') for s in job['steps']]}"
    )


@pytest.mark.parametrize("job_name", sorted(EXPECTED_MAIN_JOBS | EXPECTED_VISION_JOBS))
def test_each_build_job_checks_out_before_composite(job_name):
    """Local composite action MUST be invoked only AFTER a checkout step.

    GitHub resolves `uses: ./.github/actions/...` from the job workspace, which
    exists only after actions/checkout ran. Making the composite call the job's
    first step (with checkout only INSIDE the composite) fails every build job
    with "Can't find action.yml ... Did you forget to run actions/checkout" —
    regression run #34366133083 (18/18 build jobs red).

    Fix: clean-stale + checkout preamble lives in the job (before the composite
    call), the composite action is buildx-only.
    """
    src = MAIN_WF if job_name in EXPECTED_MAIN_JOBS else VISION_WF
    data = _load(src)
    steps = data["jobs"][job_name]["steps"]
    composite_idx = next(
        (i for i, s in enumerate(steps) if (s.get("uses") or "").endswith("/actions/l-build-service")),
        None,
    )
    assert composite_idx is not None, f"{job_name}: composite step missing"
    assert composite_idx > 0, (
        f"{job_name}: composite is the FIRST step — repo not checked out yet, "
        f"runner can't find the local action (run #34366133083). Add "
        f"actions/checkout before the composite call."
    )
    checkouts_before = [
        s
        for s in steps[:composite_idx]
        if (s.get("uses") or "").startswith("actions/checkout")
    ]
    assert checkouts_before, (
        f"{job_name}: no actions/checkout step before the composite call — "
        f"local action would not be found."
    )


@pytest.mark.parametrize("job_name", sorted(EXPECTED_MAIN_JOBS | EXPECTED_VISION_JOBS))
def test_each_build_job_has_no_inline_docker_buildx(job_name):
    """No build-job should still inline `docker buildx build`.

    Тот же контракт, но через negative control: даже если используется
    composite action, проверяем что НЕ дёргается legacy inline-buildx
    (что было бы симптомом частичного рефакторинга).
    """
    src = MAIN_WF if job_name in EXPECTED_MAIN_JOBS else VISION_WF
    data = _load(src)
    for step in data["jobs"][job_name]["steps"]:
        if step.get("uses", "").endswith("/actions/l-build-service"):
            continue  # composite — текст внутри его action.yml
        run = step.get("run", "")
        if isinstance(run, str) and "docker buildx build" in run:
            pytest.fail(
                f"{job_name}: step '{step.get('name')}' still inlines "
                f"`docker buildx build` — should use composite action"
            )


@pytest.mark.parametrize("job_name", sorted(EXPECTED_MAIN_JOBS | EXPECTED_VISION_JOBS))
def test_each_build_job_passes_local_and_ghcr_tags(job_name):
    """Each build-job's composite-action call includes both LOCAL_TAG and GHCR_TAG.

    Local tag = the registry we push to (localhost:5000). GHCR tag = the
    backup tag for prod-push workflow (issue #1503 — GHCR не пушится на
    self-hosted runner, но tag в команде buildx присутствует).
    """
    src = MAIN_WF if job_name in EXPECTED_MAIN_JOBS else VISION_WF
    data = _load(src)
    step = _composite_step(data["jobs"][job_name]["steps"])
    assert step is not None, f"{job_name}: composite step missing"

    with_block = step.get("with", {})
    tags = (with_block.get("tags") or "").strip()
    # Substitute placeholder so we can do substring checks; the actual
    # workflow uses ${{ env.* }} and ${{ needs.prepare.outputs.* }}.
    tags_normed = (
        tags.replace("${{ env.IMAGE_PREFIX }}", "${IMAGE_PREFIX}")
        .replace("${{ env.LOCAL_PREFIX }}", "${LOCAL_PREFIX}")
        .replace("${{ env.ROS_DISTRO }}", "${ROS_DISTRO}")
        .replace("${{ needs.prepare.outputs.docker_tag }}", "${DOCKER_TAG_PLACEHOLDER}")
    )

    expected = _expected_tags(job_name, "main" if src is MAIN_WF else "vision")
    for tag in expected:
        assert tag in tags_normed, (
            f"{job_name}: expected tag '{tag}' in composite tags:\n{tags_normed}"
        )


def test_update_image_versions_main_still_references_all_build_jobs():
    """update-image-versions job needs: должен перечислять все build-job'ы."""
    data = _load(MAIN_WF)
    needs = set(data["jobs"]["update-image-versions"]["needs"])
    assert "prepare" in needs
    for j in EXPECTED_MAIN_JOBS:
        assert j in needs, f"update-image-versions.main: missing needs: {j}"


def test_update_image_versions_vision_still_references_all_build_jobs():
    """update-image-versions job needs: должен перечислять все build-job'ы."""
    data = _load(VISION_WF)
    needs = set(data["jobs"]["update-image-versions"]["needs"])
    assert "prepare" in needs
    for j in EXPECTED_VISION_JOBS:
        assert j in needs, f"update-image-versions.vision: missing needs: {j}"


# --- per-service hash / arg regression tests ---------------------------


@pytest.mark.parametrize(
    "job_name,expected_arg",
    [
        ("build-robot-state-publisher", "URDF_FILES_HASH="),
        ("build-ros2-control", "BASE_IMAGE="),  # VESC_NEXUS_SHA — composite считает
        ("build-led-matrix", "BASE_IMAGE="),  # ROS2LEDS_SHA — composite считает
        ("build-voice-assistant", "SOURCE_HASH="),
        ("build-telegram-bot", "SOURCE_HASH="),
        ("build-supervisor", "SOURCE_HASH="),
        ("build-quest", "SOURCE_HASH="),
        # ARCH-quest (#2278): quest наследует rob_box_base:ros2-zenoh
        # (FROM ${BASE_IMAGE}), НЕ voice-assistant → BASE_IMAGE, не IMAGE_TAG.
        ("build-quest", "BASE_IMAGE="),
    ],
)
def test_per_service_build_arg_present(job_name, expected_arg):
    """Каждый service-specific build-arg из прежнего кода должен быть
    в новом composite call'е (в виде KEY=, может быть с hash-значением).

    Это регрессионный тест: если кто-то в рефакторинге случайно
    уберёт/переименует build-arg, cache invalidation сломается тихо.
    """
    src = MAIN_WF if job_name in EXPECTED_MAIN_JOBS else VISION_WF
    data = _load(src)
    step = _composite_step(data["jobs"][job_name]["steps"])
    assert step is not None, f"{job_name}: composite step missing"

    build_args = step.get("with", {}).get("build-args", "")
    assert expected_arg in build_args, (
        f"{job_name}: expected build-arg starting with '{expected_arg}' "
        f"in composite build-args. Got:\n{build_args}"
    )


def test_ros2_control_uses_compute_submodule_sha_for_vesc_nexus():
    """build-ros2-control: VESC_NEXUS_SHA считается через composite action
    (вход compute-submodule-sha=src/vesc_nexus), а не через pre-step.
    Это явная оптимизация — больше не нужен pre-step для submodule SHA.
    """
    data = _load(MAIN_WF)
    step = _composite_step(data["jobs"]["build-ros2-control"]["steps"])
    assert step["with"]["compute-submodule-sha"] == "src/vesc_nexus", (
        f"expected compute-submodule-sha=src/vesc_nexus, got "
        f"{step['with'].get('compute-submodule-sha')!r}"
    )


def test_led_matrix_uses_compute_submodule_sha_for_ros2leds():
    """build-led-matrix: ROS2LEDS_SHA через composite action."""
    data = _load(VISION_WF)
    step = _composite_step(data["jobs"]["build-led-matrix"]["steps"])
    assert step["with"]["compute-submodule-sha"] == "src/ros2leds", (
        f"expected compute-submodule-sha=src/ros2leds, got "
        f"{step['with'].get('compute-submodule-sha')!r}"
    )


def test_apt_proxy_present_in_every_build_job_with_base_image():
    """APT_PROXY=http://host.docker.internal:3142 — обязателен для всех job'ов,
    у которых есть --build-arg BASE_IMAGE (т.е. нужен APT proxy для apt-cacher-ng).

    Исключение: build-voice-resources / build-supercollider (нет BASE_IMAGE).
    """
    skip = {"build-voice-resources", "build-supercollider"}
    for job_name in EXPECTED_MAIN_JOBS | EXPECTED_VISION_JOBS:
        if job_name in skip:
            continue
        src = MAIN_WF if job_name in EXPECTED_MAIN_JOBS else VISION_WF
        data = _load(src)
        step = _composite_step(data["jobs"][job_name]["steps"])
        build_args = step["with"]["build-args"]
        assert "APT_PROXY=http://host.docker.internal:3142" in build_args, (
            f"{job_name}: missing APT_PROXY build-arg. Got:\n{build_args}"
        )


def test_no_docker_push_or_load_left_in_build_jobs():
    """No build-job should have inline `docker push` or `--load` logic.

    Вся docker-логика живёт в composite action (l-build-service). Если в
    каком-то job'е остался inline `docker push` / `--load` в pre/post-step'е —
    регрессия (копипаст вернулся в job'ы).
    """
    for src in (MAIN_WF, VISION_WF):
        data = _load(src)
        for job_name, job in data["jobs"].items():
            if not job_name.startswith("build-"):
                continue
            for step in job["steps"]:
                # composite action encapsulates buildx — its text lives in action.yml.
                if step.get("uses", "").endswith("/actions/l-build-service"):
                    continue
                run = step.get("run", "")
                if isinstance(run, str):
                    assert "docker push" not in run, (
                        f"{job_name}: step '{step.get('name')}' still has "
                        f"`docker push` — should be inside composite action"
                    )
                    assert "--load" not in run, (
                        f"{job_name}: step '{step.get('name')}' still has "
                        f"`--load` flag — should be `--push` inside composite"
                    )