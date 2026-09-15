"""Guard: docker/build-manifest.yaml ⇔ реальные L-Build workflow (Phase 1).

docs/plans/2026-09-15-service-manifest.md, §6. Phase 1 не меняет поведение
CI — манифест только ДОГОНЯЕТ существующие
".github/workflows/L-Build Vision Pi Services.yml" /
".github/workflows/L-Build Main Pi Services.yml". Этот guard-тест проверяет,
что манифест не разошёлся с реальностью, тем же способом, каким уже
задокументированный, но не подключённый к CI guard
(tests/unit/test_workflow_refactor_acceptance.py) уже был подключён —
см. ADR-0057 §1.2 («скрипт написан, но не в CI» — тот же класс дефекта,
только для другого guard'а).

Стиль — по образцу scripts/ci/validate_test_packages.py (регэксп-парсинг
bash-heredoc'ов там, где YAML-структура неудобна) и
tests/unit/test_workflow_refactor_acceptance.py (yaml.safe_load для
структурной части workflow).

Что проверяется (план §6):

1. Множество сервисов манифеста ⇔ множество build-* job'ов реальных
   workflow — УЖЕ покрыто расширенным тестом
   tests/unit/test_workflow_refactor_acceptance.py::test_{main,vision}_workflow_all_build_jobs_present
   (EXPECTED_{MAIN,VISION}_JOBS там выводятся из этого же манифеста —
   план §5 п.5, §6 п.1: "не дублировать, а расширить существующий
   тестовый файл третьим источником истины"). Здесь НЕ повторяется.
2. Для каждого сервиса с depends_on: needs: соответствующего job'а
   содержит prepare + все элементы depends_on манифеста.
3. Множество сервисов в tag_and_push/verify_in_registry (bash-heredoc,
   регэксп) == множество сервисов с image_versions != false в манифесте.
4. Множество полей в sed "s|^X_TAG=.*" == множество значений
   image_versions в манифесте (ловит phantom-поля вроде RTABMAP_SYNC_TAG
   автоматически).
5. tag.ros_distro: false в манифесте есть ровно у тех сервисов, чей блок
   tags: в composite-вызове НЕ содержит ${{ env.ROS_DISTRO }} (сегодня —
   только supercollider).

Hard gate в G-Lint Code.yml (рядом с validate_test_packages.py) — см.
ADR-0057 §2.1 (паттерн "guard рядом с уже существующим guard'ом того же
класса, hard gate, не warn-only").
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
MANIFEST_PATH = REPO_ROOT / "docker" / "build-manifest.yaml"
MAIN_WF = REPO_ROOT / ".github" / "workflows" / "L-Build Main Pi Services.yml"
VISION_WF = REPO_ROOT / ".github" / "workflows" / "L-Build Vision Pi Services.yml"

PI_TO_WORKFLOW = {"main": MAIN_WF, "vision": VISION_WF}


# --- helpers ---------------------------------------------------------------


def _load_yaml(path: Path) -> dict:
    # Как и в test_workflow_refactor_acceptance.py: workflow-файлы содержат
    # русские комментарии в UTF-8, на Windows дефолтная codepage — cp1252.
    with path.open(encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _manifest() -> dict[str, Any]:
    return _load_yaml(MANIFEST_PATH)


def _services(pi: str) -> dict[str, Any]:
    return dict(_manifest()["pis"][pi]["services"])


def _composite_step(steps: list[dict]) -> dict | None:
    for s in steps:
        if (s.get("uses") or "").endswith("/actions/l-build-service"):
            return s
    return None


# Регэксп для bash-heredoc части update-image-versions: builds ловят
# tag_and_push "${LOCAL_PREFIX}:<service>-${ROS_DISTRO}-${DOCKER_TAG}"
# (у supercollider — без "-${ROS_DISTRO}").
_TAG_AND_PUSH_RE = re.compile(
    r'tag_and_push\s+"\$\{LOCAL_PREFIX\}:([a-z0-9][a-z0-9-]*?)(?:-\$\{ROS_DISTRO\})?-\$\{DOCKER_TAG\}"'
)

# sed -e "s|^X_TAG=.*|X_TAG=...|" — извлекаем X_TAG.
_SED_TAG_RE = re.compile(r"s\|\^([A-Z][A-Z0-9_]*_TAG)=")


def _tag_and_push_services(pi: str) -> set[str]:
    text = PI_TO_WORKFLOW[pi].read_text(encoding="utf-8")
    return set(_TAG_AND_PUSH_RE.findall(text))


def _sed_image_versions_vars(pi: str) -> set[str]:
    text = PI_TO_WORKFLOW[pi].read_text(encoding="utf-8")
    return set(_SED_TAG_RE.findall(text))


# --- 2. depends_on ⇒ needs: -------------------------------------------------


@pytest.mark.parametrize("pi", ["vision", "main"])
def test_chained_services_needs_match_depends_on(pi):
    """Каждый сервис с depends_on: needs: реального job'а содержит prepare
    + все элементы depends_on (инвариант §2.3 плана).
    """
    services = _services(pi)
    data = _load_yaml(PI_TO_WORKFLOW[pi])
    chained = {name: svc for name, svc in services.items() if svc.get("depends_on")}

    for name, svc in chained.items():
        job_name = f"build-{name}"
        job = data["jobs"].get(job_name)
        assert job is not None, f"{pi}: job {job_name} missing in workflow"
        needs = set(job.get("needs") or [])
        assert "prepare" in needs, f"{pi}/{job_name}: needs: must include 'prepare'"
        for dep in svc["depends_on"]:
            dep_job = f"build-{dep}"
            assert dep_job in needs, (
                f"{pi}/{job_name}: manifest depends_on={svc['depends_on']!r}, "
                f"expected needs: to include {dep_job!r}, got {sorted(needs)}"
            )


# --- 3. tag_and_push/verify_in_registry ⇔ image_versions != false ----------


@pytest.mark.parametrize("pi", ["vision", "main"])
def test_tag_and_push_matches_manifest_image_versions(pi):
    """Множество сервисов в tag_and_push (bash-heredoc в update-image-versions)
    равно множеству сервисов с image_versions != false в манифесте.

    Это буквально фиксирует как факт (план требование 1) все четыре
    сегодняшних исключения на Vision Pi (voice-base, supercollider,
    supervisor, quest) и одно на Main Pi (teleop) — guard не про их
    "правильность", а про то, что манифест и код workflow говорят одно и
    то же.
    """
    services = _services(pi)
    manifest_has_iv = {
        name
        for name, svc in services.items()
        if svc.get("image_versions", False) is not False
    }
    workflow_tag_and_push = _tag_and_push_services(pi)

    assert workflow_tag_and_push == manifest_has_iv, (
        f"{pi}: tag_and_push services {sorted(workflow_tag_and_push)} != "
        f"manifest image_versions!=false services {sorted(manifest_has_iv)}"
    )


# --- 4. sed *_TAG-поля ⇔ image_versions значения (ловит phantom) -----------


@pytest.mark.parametrize("pi", ["vision", "main"])
def test_sed_fields_match_manifest_image_versions_values(pi):
    """Множество X_TAG в sed-блоке update-image-versions равно множеству
    значений image_versions в манифесте.

    Ожидаемо ловит phantom-поля (RTABMAP_SYNC_TAG на Vision Pi,
    MICRO_ROS_AGENT_TAG на Main Pi — план §1.3/§1.4/ADR-0094 §1.4): у sed
    они есть, но ни один сервис манифеста не объявляет их в
    image_versions. План §6 п.4 явно требует whitelist-механизм для таких
    полей (по аналогии с IV_KNOWN_PHANTOMS в check_image_versions_usage.sh) —
    в Phase 1 whitelist фиксирует оба известных на сегодня phantom'а.
    """
    # Известные на 2026-09-15 phantom-поля (план §1.3, §1.4) — sed их
    # пишет, но ни один сервис манифеста ими не владеет. Phase 1 их не
    # чинит (задача 1 в тексте задания), только документирует здесь как
    # whitelist, по аналогии с IV_KNOWN_PHANTOMS (ADR-0094 §3.2).
    known_phantoms = {
        "vision": {"RTABMAP_SYNC_TAG"},
        "main": {"MICRO_ROS_AGENT_TAG"},
    }[pi]

    services = _services(pi)
    manifest_values = {
        str(svc["image_versions"])
        for svc in services.values()
        if svc.get("image_versions", False) is not False
    }
    sed_vars = _sed_image_versions_vars(pi)

    unexpected_phantoms = sed_vars - manifest_values - known_phantoms
    assert not unexpected_phantoms, (
        f"{pi}: new phantom *_TAG field(s) in sed, not owned by any manifest "
        f"service and not whitelisted: {sorted(unexpected_phantoms)}"
    )

    missing_in_sed = manifest_values - sed_vars
    assert not missing_in_sed, (
        f"{pi}: manifest declares image_versions for {sorted(missing_in_sed)}, "
        f"but sed in update-image-versions does not write them"
    )

    # Whitelist сам не должен протухнуть: если кто-то уберёт phantom-поле
    # из sed (почини его отдельным PR по ADR-0094 §3.3), список здесь
    # тоже нужно почистить — иначе guard молча перестанет что-либо ловить.
    stale_whitelist = known_phantoms - sed_vars
    assert not stale_whitelist, (
        f"{pi}: known_phantoms {sorted(stale_whitelist)} no longer present in "
        f"sed — remove from the whitelist in this test (guard-тест не должен "
        f"держать мёртвые исключения)"
    )


# --- 5. tag.ros_distro: false ⇔ тег без ${{ env.ROS_DISTRO }} --------------


@pytest.mark.parametrize("pi", ["vision", "main"])
def test_tag_ros_distro_false_matches_composite_tags(pi):
    """tag.ros_distro: false в манифесте есть ровно у тех сервисов, чей
    composite-вызов НЕ включает ${{ env.ROS_DISTRO }} в tags: (сегодня —
    только supercollider, Vision Pi).
    """
    services = _services(pi)
    data = _load_yaml(PI_TO_WORKFLOW[pi])

    manifest_no_ros_distro = {
        name
        for name, svc in services.items()
        if svc.get("tag", {}).get("ros_distro") is False
    }

    workflow_no_ros_distro = set()
    for name in services:
        job = data["jobs"].get(f"build-{name}")
        if job is None:
            continue
        step = _composite_step(job["steps"])
        if step is None:
            continue
        tags = step.get("with", {}).get("tags", "") or ""
        if "${{ env.ROS_DISTRO }}" not in tags:
            workflow_no_ros_distro.add(name)

    assert workflow_no_ros_distro == manifest_no_ros_distro, (
        f"{pi}: services whose composite tags omit ROS_DISTRO "
        f"{sorted(workflow_no_ros_distro)} != manifest tag.ros_distro=false "
        f"services {sorted(manifest_no_ros_distro)}"
    )


# --- smoke: манифест вообще парсится и покрывает оба pi ---------------------


def test_manifest_declares_both_pis():
    manifest = _manifest()
    assert set(manifest["pis"]) >= {"vision", "main"}, (
        f"manifest.pis must declare at least 'vision' and 'main', "
        f"got {sorted(manifest['pis'])}"
    )
