"""Regression tests for the refactored L-Build Main/Vision Pi Services workflows.

Issue #2280 acceptance + docs/plans/2026-09-15-service-manifest.md Phase 2:
- сборка КАЖДОГО сервиса идёт через ./.github/actions/l-build-service
  (buildx --load + docker push ТОЛЬКО локального registry), а не через
  inline `docker buildx build`;
- GHCR никогда не пушится на локальных test/dev сборках: runner не залогинен
  в ghcr.io (issue #1503) — buildx --push по GHCR-тегу падал unauthorized
  (run #34368750126). Поэтому образ собирается в daemon (--load), а пушится
  только LOCAL-тег (localhost:5000); update-image-versions делает `docker tag`
  из локального daemon;
- composite вызывается только ПОСЛЕ checkout (fix run #34366133083: GitHub
  ищет локальный action в рабочем каталоге, которого до checkout нет — тогда
  красными стали все 18 build-job'ов);
- параметры сборки (dockerfile/context/теги/build-args/submodule-sha) у
  каждого сервиса — те же, что были до перехода на матрицу.

ЧТО ИЗМЕНИЛОСЬ В САМИХ ТЕСТАХ ПОСЛЕ Phase 2. Раньше здесь было по одному
параметризованному тесту на КАЖДЫЙ build-* job: тесты читали `with:`
композит-вызова из текста workflow. После Phase 2 в workflow остался ОДИН
шаблон job'а на все сервисы, а параметры приезжают в него из
docker/build-manifest.yaml через scripts/ci/gen_build_matrix.py. Поэтому
проверки разделились на два уровня:

  * уровень workflow (шаблон): checkout до composite, никакого inline
    buildx/push/--load, параметры composite'а берутся из matrix, а не
    вписаны руками;
  * уровень сервиса (данные): tags/build-args/dockerfile/context для всех
    19 сервисов проверяются на выходе генератора — теперь это единственное
    место, где они существуют.

Сверка "старое поведение == новое" при переходе делалась отдельно, прогоном
генератора против job'ов на коммите 8740ea5ba (до Phase 2) — все 19 сервисов
совпали по всем полям, включая needs.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
MAIN_WF = REPO_ROOT / ".github" / "workflows" / "L-Build Main Pi Services.yml"
VISION_WF = REPO_ROOT / ".github" / "workflows" / "L-Build Vision Pi Services.yml"
BUILD_MANIFEST = REPO_ROOT / "docker" / "build-manifest.yaml"
GEN_SCRIPT = REPO_ROOT / "scripts" / "ci" / "gen_build_matrix.py"

_SPEC = importlib.util.spec_from_file_location("gen_build_matrix_acceptance", GEN_SCRIPT)
assert _SPEC is not None and _SPEC.loader is not None, f"{GEN_SCRIPT}: spec failed"
gen_build_matrix = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(gen_build_matrix)  # type: ignore[union-attr]


# --- expected inventory ---------------------------------------------------
#
# docs/plans/2026-09-15-service-manifest.md §5 п.5 / §6 п.1: раньше здесь
# были два захардкоженных множества EXPECTED_MAIN_JOBS/EXPECTED_VISION_JOBS —
# независимая (седьмая по счёту, план §1.1) копия знания "какие сервисы
# есть", которая уже успела разойтись с реальностью (в ней не было
# build-vision-hailo — красный тест, план §1.7). Единственный источник
# истины — docker/build-manifest.yaml.

_MANIFEST = gen_build_matrix.load_manifest(BUILD_MANIFEST)

MAIN_SERVICES = gen_build_matrix.services_for_pi(_MANIFEST, "main")
VISION_SERVICES = gen_build_matrix.services_for_pi(_MANIFEST, "vision")

PI_DATA = {"main": (MAIN_WF, MAIN_SERVICES), "vision": (VISION_WF, VISION_SERVICES)}

ALL_SERVICES = sorted(
    [("main", name) for name in MAIN_SERVICES]
    + [("vision", name) for name in VISION_SERVICES]
)

# Тег, с которым прогоняется генератор в тестах. В CI сюда приезжает
# needs.prepare.outputs.docker_tag (dev|test|latest|local).
DOCKER_TAG = "dev"


# --- helpers -----------------------------------------------------------


def _load(path: Path) -> dict:
    # Workflow YAML contains UTF-8 Russian comments; on Windows the default
    # codepage is cp1252, so encoding must be explicit (CI/Linux defaults to
    # utf-8, which is why this only bites on Windows dev machines).
    with path.open(encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _build_jobs(path: Path) -> dict[str, dict]:
    """Все job'ы, которые что-то собирают: общий matrix-job + именованные."""
    data = _load(path)
    return {
        name: job
        for name, job in data["jobs"].items()
        if name == "build" or name.startswith("build-")
    }


def _composite_step(steps: list[dict]) -> dict | None:
    for s in steps:
        if (s.get("uses") or "").endswith("/actions/l-build-service"):
            return s
    return None


def _entry(pi: str, service: str) -> dict[str, str]:
    """Параметры сборки сервиса — то, что реально уедет в composite action."""
    _, services = PI_DATA[pi]
    ctx = gen_build_matrix.build_context(_MANIFEST, docker_tag=DOCKER_TAG)
    return gen_build_matrix.matrix_entry(service, services, ctx)


# --- уровень workflow: шаблон job'а ---------------------------------------


@pytest.mark.parametrize("pi", ["main", "vision"])
def test_every_build_job_uses_composite_action(pi):
    """Любой собирающий job делегирует ./.github/actions/l-build-service.

    Headline-acceptance issue #2280 — если где-то остался inline
    `docker buildx build --load ...`, рефакторинг неполный и кто-то
    копипастит назад.
    """
    path, _ = PI_DATA[pi]
    jobs = _build_jobs(path)
    assert jobs, f"{pi}: в workflow не осталось ни одного build-job'а"
    for job_name, job in jobs.items():
        assert _composite_step(job["steps"]) is not None, (
            f"{pi}/{job_name}: нет шага с ./.github/actions/l-build-service, "
            f"шаги: {[s.get('name') or s.get('uses') for s in job['steps']]}"
        )


@pytest.mark.parametrize("pi", ["main", "vision"])
def test_every_build_job_checks_out_before_composite(pi):
    """Локальный composite вызывается только ПОСЛЕ actions/checkout.

    GitHub резолвит `uses: ./.github/actions/...` из рабочего каталога job'а,
    который появляется только после checkout. Composite первым шагом =
    "Can't find action.yml ... Did you forget to run actions/checkout"
    (регрессия run #34366133083, 18/18 build-job'ов красные).
    """
    path, _ = PI_DATA[pi]
    for job_name, job in _build_jobs(path).items():
        steps = job["steps"]
        composite_idx = next(
            (
                i
                for i, s in enumerate(steps)
                if (s.get("uses") or "").endswith("/actions/l-build-service")
            ),
            None,
        )
        assert composite_idx is not None, f"{pi}/{job_name}: composite step missing"
        assert composite_idx > 0, (
            f"{pi}/{job_name}: composite — ПЕРВЫЙ шаг, репозиторий ещё не "
            f"зачекаутен (run #34366133083)"
        )
        assert [
            s
            for s in steps[:composite_idx]
            if (s.get("uses") or "").startswith("actions/checkout")
        ], f"{pi}/{job_name}: нет actions/checkout до вызова composite"


@pytest.mark.parametrize("pi", ["main", "vision"])
def test_no_inline_docker_buildx_push_or_load(pi):
    """Вся docker-логика живёт в composite action.

    Негативный контроль: ни один build-job не должен содержать inline
    `docker buildx build` / `docker push` / `--load` в своих собственных
    шагах (это был бы симптом вернувшейся копипасты).
    """
    path, _ = PI_DATA[pi]
    for job_name, job in _build_jobs(path).items():
        for step in job["steps"]:
            if (step.get("uses") or "").endswith("/actions/l-build-service"):
                continue  # текст composite'а живёт в его action.yml
            run = step.get("run", "")
            if not isinstance(run, str):
                continue
            for forbidden in ("docker buildx build", "docker push", "--load"):
                assert forbidden not in run, (
                    f"{pi}/{job_name}: шаг '{step.get('name')}' содержит "
                    f"`{forbidden}` — это должно быть внутри composite action"
                )


@pytest.mark.parametrize("pi", ["main", "vision"])
def test_composite_inputs_come_from_matrix_not_from_literals(pi):
    """`with:` композит-вызова заполняется из matrix, а не вписан руками.

    Это тот самый шов, ради которого затевался манифест: состав сборки —
    данные, а job — тонкий адаптер над ними (план §2.1).
    """
    path, _ = PI_DATA[pi]
    expected = {
        "service-name": "${{ matrix.name }}",
        "dockerfile-path": "${{ matrix.dockerfile }}",
        "build-context": "${{ matrix.build_context }}",
        "compute-submodule-sha": "${{ matrix.submodule_sha }}",
    }
    for job_name, job in _build_jobs(path).items():
        with_block = _composite_step(job["steps"])["with"]
        for key, value in expected.items():
            assert with_block[key] == value, (
                f"{pi}/{job_name}: {key}={with_block.get(key)!r}, ожидалось {value!r}"
            )
        assert with_block["tags"].strip() == "${{ matrix.tags }}"
        # build-args = статическая часть из манифеста + hash, посчитанный
        # pre-step'ом в рантайме (пустая строка, если хеша у сервиса нет).
        assert with_block["build-args"].splitlines() == [
            "${{ matrix.build_args }}",
            "${{ steps.source-hash.outputs.build_arg }}",
        ], f"{pi}/{job_name}: build-args={with_block['build-args']!r}"


@pytest.mark.parametrize("pi", ["main", "vision"])
def test_update_image_versions_waits_for_every_build_job(pi):
    """update-image-versions/summary зависят от ВСЕХ собирающих job'ов.

    Matrix-job завершается только когда закончились все его элементы,
    поэтому `needs: [prepare, build, ...именованные]` даёт ту же гарантию,
    что прежний список из 11/8 job'ов.
    """
    path, _ = PI_DATA[pi]
    data = _load(path)
    build_jobs = set(_build_jobs(path))
    for consumer in ("update-image-versions", "summary"):
        needs = set(data["jobs"][consumer]["needs"])
        assert "prepare" in needs, f"{pi}/{consumer}: needs: без prepare"
        missing = build_jobs - needs
        assert not missing, f"{pi}/{consumer}: не дожидается job'ов {sorted(missing)}"


# --- уровень сервиса: параметры сборки из манифеста -----------------------


@pytest.mark.parametrize("pi,service", ALL_SERVICES)
def test_service_gets_both_local_and_ghcr_tags(pi, service):
    """У каждого сервиса ровно два тега: GHCR (локальное имя в daemon) и
    LOCAL (его и пушим в localhost:5000, issue #1503).
    """
    tags = _entry(pi, service)["tags"].splitlines()
    assert len(tags) == 2, f"{pi}/{service}: ожидалось 2 тега, получено {tags}"
    ghcr, local = tags
    assert ghcr.startswith("ghcr.io/krikz/rob_box:"), tags
    assert local.startswith("localhost:5000/krikz/rob_box:"), tags
    # Суффикс (имя-дистро-тег) у обоих одинаковый — различаются только
    # префиксы. rsplit, а не split: в "localhost:5000/..." двоеточие есть и
    # в имени хоста.
    assert ghcr.rsplit(":", 1)[1] == local.rsplit(":", 1)[1]
    assert ghcr.rsplit(":", 1)[1].startswith(f"{service}-")
    assert ghcr.endswith(f"-{DOCKER_TAG}")


@pytest.mark.parametrize("pi,service", ALL_SERVICES)
def test_apt_proxy_present_for_every_service(pi, service):
    """APT_PROXY=http://host.docker.internal:3142 нужен всем (apt-cacher-ng).

    До Phase 2 это был единственный build-arg, который встречался в каждом
    job'е; теперь он приходит из defaults манифеста.
    """
    assert (
        "APT_PROXY=http://host.docker.internal:3142"
        in _entry(pi, service)["build_args"].splitlines()
    )


@pytest.mark.parametrize("pi,service", ALL_SERVICES)
def test_dockerfile_and_context_exist_on_disk(pi, service):
    """Путь к Dockerfile и build-context из манифеста существуют.

    До Phase 2 опечатка в пути ловилась только реальным прогоном сборки на
    build-стенде (единственном и медленном) — теперь ловится здесь.
    """
    entry = _entry(pi, service)
    assert (REPO_ROOT / entry["dockerfile"]).is_file(), (
        f"{pi}/{service}: нет файла {entry['dockerfile']}"
    )
    assert (REPO_ROOT / entry["build_context"]).is_dir(), (
        f"{pi}/{service}: build-context {entry['build_context']} не каталог"
    )


@pytest.mark.parametrize(
    "pi,service,expected_arg",
    [
        # Те же регрессионные проверки per-service build-arg'ов, что были до
        # Phase 2 — только источником стал манифест, а не текст workflow.
        ("main", "robot-state-publisher", "URDF_FILES_HASH"),
        ("vision", "voice-assistant", "SOURCE_HASH"),
        ("vision", "telegram-bot", "SOURCE_HASH"),
        ("vision", "supervisor", "SOURCE_HASH"),
        ("vision", "quest", "SOURCE_HASH"),
        ("vision", "vision-hailo", "SOURCE_HASH"),
    ],
)
def test_per_service_hash_arg_preserved(pi, service, expected_arg):
    """Имя build-arg'а для hash'а исходников не потерялось и не съехало.

    Если кто-то в рефакторинге уберёт/переименует его, cache invalidation
    сломается ТИХО: образ соберётся из старых слоёв (issue #2314).
    robot-state-publisher исторически использует URDF_FILES_HASH, а не
    SOURCE_HASH — манифест это фиксирует, а не унифицирует (план §2.4).
    """
    assert _entry(pi, service)["source_hash_arg"] == expected_arg


@pytest.mark.parametrize(
    "pi,service,submodule",
    [
        ("main", "ros2-control", "src/vesc_nexus"),
        ("vision", "led-matrix", "src/ros2leds"),
    ],
)
def test_submodule_sha_delegated_to_composite(pi, service, submodule):
    """VESC_NEXUS_SHA/ROS2LEDS_SHA считает composite (compute-submodule-sha),
    а не pre-step в job'е.
    """
    assert _entry(pi, service)["submodule_sha"] == submodule


@pytest.mark.parametrize("pi,service", ALL_SERVICES)
def test_base_image_arg_matches_manifest_base(pi, service):
    """BASE_IMAGE есть ровно у тех сервисов, у которых в манифесте есть base.

    Исключение (base: null) — supercollider: у него в прежнем коде
    тоже не было --build-arg BASE_IMAGE. Вторым таким сервисом был
    voice-resources — его узел удалён вместе с образом.
    """
    _, services = PI_DATA[pi]
    entry = _entry(pi, service)
    has_base_arg = any(
        line.startswith("BASE_IMAGE=") for line in entry["build_args"].splitlines()
    )
    assert has_base_arg == bool(services[service].get("base")), (
        f"{pi}/{service}: BASE_IMAGE={'есть' if has_base_arg else 'нет'}, "
        f"а в манифесте base={services[service].get('base')!r}"
    )
