"""Guard: docker/build-manifest.yaml ⇔ реальный L-Build workflow.

docs/plans/2026-09-15-service-manifest.md, §6; ADR-0159 PR-3 (В4).

Режим Phase 1 (манифест только ДОГОНЯЛ руками написанные workflow) закрыт:
после Phase 2 workflow-файлы больше не содержат списка сервисов — они
вычисляют matrix из этого же манифеста в `prepare`-job'е. Поэтому guard
проверяет не «совпадают ли два списка», а «действительно ли workflow
берёт состав сборки из docker/build-manifest.yaml, а не из копии списка».
Конкретные значения параметров сборки (теги, build-args) покрыты тестами
генератора (scripts/ci/tests/test_gen_build_matrix.py) и acceptance-тестами
(tests/unit/test_workflow_refactor_acceptance.py), здесь не дублируются.

ADR-0159 PR-3: per-Pi workflow (Main, Vision) заменены ОДНИМ
".github/workflows/L-Build Pi Services.yml" с входом `pi`; guard сверяет
манифест с одним файлом. Именованных job'ов по сервисам больше нет — граф
depends_on собирается волнами (`build`, `build-wave-1`, …).

Что проверяется:

1. `prepare` зовёт scripts/ci/gen_build_matrix.py ИМЕННО с этим манифестом,
   для Pi из входа `pi`, и отдаёт waves/wave_count/services как outputs.
2. Вход `pi` предлагает ровно Pi манифеста; "L: Build All Services" зовёт
   workflow для каждого Pi манифеста.
3. Wave-job'ы: волна k разворачивается из fromJSON(...waves)[k], ждёт волну
   k-1 и пропускается, когда волн у Pi меньше; job'ов не меньше, чем волн у
   самого глубокого Pi манифеста.
4. НЕГАТИВНЫЙ КОНТРОЛЬ: ни имя сервиса, ни имя *_TAG-поля манифеста не
   встречается в исполняемом (не-комментарии) тексте workflow.
5. env: workflow'а (IMAGE_PREFIX/LOCAL_PREFIX/ROS_DISTRO) == defaults манифеста.
6. Все build-job'ы имеют ПОБАЙТОВО один и тот же список шагов.
7. Каждый pre_build-хук манифеста реализован шагом workflow.
8. update-image-versions идёт по выводу `gen_build_matrix.py --mode tags`
   (Phase 3), пишет файлы общим scripts/ci/write-image-versions.sh, механизм
   push (ADR-0094) на месте, phantom-поля .image-versions.* никем не пишутся.
9. В .image-versions.* соседние строки полей пишет одна сборка (иначе
   параллельные коммиты Build All конфликтуют на rebase).

Hard gate в G-Lint Code.yml (рядом с validate_test_packages.py) — см.
ADR-0057 §2.1 (паттерн "guard рядом с уже существующим guard'ом того же
класса, hard gate, не warn-only").
"""

from __future__ import annotations

import importlib.util
import re
from pathlib import Path
from typing import Any

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
MANIFEST_PATH = REPO_ROOT / "docker" / "build-manifest.yaml"
WORKFLOWS = REPO_ROOT / ".github" / "workflows"
PI_WF = WORKFLOWS / "L-Build Pi Services.yml"
BUILD_ALL_WF = WORKFLOWS / "L-Build-All-Services.yml"
SINGLE_WF = WORKFLOWS / "L-Build Single Service.yml"
GEN_SCRIPT = REPO_ROOT / "scripts" / "ci" / "gen_build_matrix.py"

# Путь к манифесту, который workflow ОБЯЗАН передавать генератору — тот же,
# что читает этот тест. Если кто-то заведёт второй манифест-копию, п.1 упадёт.
MANIFEST_REL = "docker/build-manifest.yaml"


# --- helpers ---------------------------------------------------------------


def _load_yaml(path: Path) -> dict:
    # Workflow-файлы содержат русские комментарии в UTF-8, на Windows
    # дефолтная codepage — cp1252.
    with path.open(encoding="utf-8") as fh:
        return yaml.safe_load(fh)


_SPEC = importlib.util.spec_from_file_location("gen_build_matrix_guard", GEN_SCRIPT)
assert _SPEC is not None and _SPEC.loader is not None, f"{GEN_SCRIPT}: spec failed"
gen_build_matrix = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(gen_build_matrix)  # type: ignore[union-attr]


def _manifest() -> dict[str, Any]:
    return gen_build_matrix.load_manifest(MANIFEST_PATH)


def _pis() -> list[str]:
    return sorted(_manifest()["pis"])


def _services(pi: str) -> dict[str, Any]:
    return gen_build_matrix.services_for_pi(_manifest(), pi)


def _wf() -> dict:
    return _load_yaml(PI_WF)


def _on(data: dict) -> dict:
    # PyYAML (YAML 1.1) читает ключ `on:` как булево True.
    return data.get("on") or data.get(True)


def _build_jobs(data: dict) -> dict[str, dict]:
    """Wave-job'ы: `build` (волна 0) и `build-wave-<k>`."""
    return {
        k: v for k, v in data["jobs"].items() if k == "build" or k.startswith("build-")
    }


def _wave_index(job_name: str) -> int:
    return 0 if job_name == "build" else int(job_name.removeprefix("build-wave-"))


def _prepare_gen_step(data: dict) -> dict:
    for step in data["jobs"]["prepare"]["steps"]:
        if "gen_build_matrix.py" in (step.get("run") or ""):
            return step
    raise AssertionError(
        "prepare: нет шага, вызывающего scripts/ci/gen_build_matrix.py"
    )


def _strip_comments(path: Path) -> str:
    """Текст workflow без строк-комментариев (для негативного контроля)."""
    return "\n".join(
        line
        for line in path.read_text(encoding="utf-8").splitlines()
        if not line.lstrip().startswith("#")
    )


# --- 1. prepare читает ИМЕННО этот манифест для Pi из входа -----------------


def test_prepare_generates_waves_from_the_manifest():
    data = _wf()
    step = _prepare_gen_step(data)
    run = step["run"]

    assert MANIFEST_REL in run, (
        f"prepare обязан звать генератор с --manifest {MANIFEST_REL} "
        f"(иначе матрица считается по какому-то другому файлу)"
    )
    assert '--pi "$PI"' in run, "prepare зовёт генератор не для Pi из входа `pi`"
    assert (
        data["env"]["PI"] == "${{ inputs.pi }}"
    ), "env.PI обязан быть входом `pi` — иначе генератор соберёт не тот Pi"
    assert "--mode waves" in run, "prepare не вызывает генератор с --mode waves"
    assert "$GITHUB_OUTPUT" in run, "результат генератора не попадает в outputs"

    outputs = data["jobs"]["prepare"]["outputs"]
    for key in ("waves", "wave_count", "services"):
        assert key in outputs, f"prepare.outputs.{key} отсутствует"
        assert (
            f"steps.{step['id']}.outputs.{key}" in outputs[key]
        ), f"prepare.outputs.{key} не ссылается на шаг генератора"


# --- 2. вход pi и три вызова из Build All -----------------------------------


def test_pi_input_offers_exactly_the_manifest_pis():
    on = _on(_wf())
    dispatch = on["workflow_dispatch"]["inputs"]["pi"]
    assert dispatch["type"] == "choice"
    assert (
        sorted(dispatch["options"]) == _pis()
    ), f"workflow_dispatch.inputs.pi.options={dispatch['options']} != Pi манифеста {_pis()}"
    call = on["workflow_call"]["inputs"]["pi"]
    assert call["required"] is True and call["type"] == "string"


def test_build_all_calls_the_one_workflow_for_every_pi():
    jobs = _load_yaml(BUILD_ALL_WF)["jobs"]
    called = {
        job["with"]["pi"]: name
        for name, job in jobs.items()
        if job.get("uses") == "./.github/workflows/L-Build Pi Services.yml"
    }
    assert sorted(called) == _pis(), (
        f"L-Build-All-Services зовёт L-Build Pi Services для {sorted(called)}, "
        f"а в манифесте Pi {_pis()}"
    )
    for pi, name in called.items():
        assert (
            name == f"build-{pi}-services"
        ), f"{name}: job id не по шаблону build-<pi>-services"
        # Pi независимы (depends_on — только внутри Pi): ни один вызов не ждёт
        # другой Pi, только базовые образы.
        assert jobs[name]["needs"] == "build-base-images", (pi, jobs[name]["needs"])
    assert set(called.values()) <= set(jobs["summary"]["needs"])


def test_old_per_pi_workflows_are_gone():
    for old in ("L-Build Main Pi Services.yml", "L-Build Vision Pi Services.yml"):
        assert not (
            WORKFLOWS / old
        ).exists(), f"{old} вернулся — сборка Pi живёт в {PI_WF.name}"


# --- 3. волны -------------------------------------------------------------


def test_wave_jobs_cover_the_deepest_pi():
    jobs = _build_jobs(_wf())
    indexes = sorted(_wave_index(name) for name in jobs)
    need = gen_build_matrix.max_wave_count(_manifest())
    assert indexes == list(range(len(indexes))), f"номера волн с дырой: {indexes}"
    assert len(indexes) >= need, (
        f"в манифесте Pi с {need} волнами сборки, а wave-job'ов {len(indexes)} — "
        f"последняя волна молча не соберётся. Добавь build-wave-{len(indexes)}."
    )


def test_wave_job_reads_its_wave_and_waits_for_the_previous_one():
    jobs = _build_jobs(_wf())
    for name, job in jobs.items():
        k = _wave_index(name)
        assert job["strategy"]["fail-fast"] is False, f"{name}: нужен fail-fast: false"
        include = str(job["strategy"]["matrix"]["include"])
        assert f"fromJSON(needs.prepare.outputs.waves)[{k}]" in include, (name, include)
        needs = job["needs"]
        needs = [needs] if isinstance(needs, str) else list(needs)
        prev = [] if k == 0 else ["build" if k == 1 else f"build-wave-{k - 1}"]
        assert needs == ["prepare", *prev], (name, needs)
        if k > 0:
            cond = job.get("if", "")
            assert f"fromJSON(needs.prepare.outputs.wave_count) > {k}" in cond, (
                name,
                cond,
            )
            assert "!failure()" in cond and "!cancelled()" in cond, (name, cond)


def test_consumers_wait_for_every_wave():
    data = _wf()
    waves = set(_build_jobs(data))
    for consumer in ("update-image-versions", "summary"):
        needs = set(data["jobs"][consumer]["needs"])
        assert {"prepare", *waves} <= needs, (consumer, sorted(needs))
    # Пропущенная волна (у Pi меньше волн) — не повод пропускать запись тегов.
    cond = data["jobs"]["update-image-versions"]["if"]
    assert "!failure()" in cond and "success()" not in cond, cond


# --- 4. негативный контроль: списка сервисов в workflow нет ----------------


def test_no_service_or_tag_names_in_workflow_body():
    body = _strip_comments(PI_WF)
    for pi in _pis():
        services = _services(pi)
        for name in [*services, *gen_build_matrix.image_versions_map(services)]:
            hits = [
                f"    {i}: {line.strip()}"
                for i, line in enumerate(body.splitlines(), 1)
                if re.search(rf"(?<![\w-]){re.escape(name)}(?![\w-])", line)
            ]
            assert not hits, (
                f"{pi}: {name!r} в теле {PI_WF.name} — состав сборки обязан "
                f"приходить из {MANIFEST_REL}:\n" + "\n".join(hits)
            )


# --- 5. env workflow == defaults манифеста ---------------------------------


def test_workflow_env_matches_manifest_defaults():
    defaults = _manifest()["defaults"]
    env = _wf()["env"]
    assert env["IMAGE_PREFIX"] == defaults["registry"]["ghcr"]
    assert env["LOCAL_PREFIX"] == defaults["registry"]["local"]
    assert env["ROS_DISTRO"] == defaults["ros_distro"]
    assert env["LOCAL_REGISTRY"] == defaults["registry"]["local_registry_host"]


# --- 6. тело всех build-job'ов — один шаблон -------------------------------


def test_all_build_jobs_share_one_step_template():
    data = _wf()
    template = data["jobs"]["build"]["steps"]
    for job_name, job in _build_jobs(data).items():
        assert job["steps"] == template, (
            f"{job_name}: шаги разошлись с шаблоном `build`. Из такого "
            f"расхождения копий вырастают stale-слои (issue #2314) — правь "
            f"шаблон целиком, а не одну копию."
        )


# --- 7. pre_build-хуки манифеста реализованы в workflow --------------------


def test_pre_build_hooks_are_implemented():
    body = _strip_comments(PI_WF)
    hooks = {
        svc["pre_build"]
        for pi in _pis()
        for svc in _services(pi).values()
        if svc.get("pre_build")
    }
    for hook in hooks:
        assert f"matrix.pre_build == '{hook}'" in body, (
            f"манифест объявляет pre_build-хук {hook!r}, но в workflow нет шага "
            f"с `if: matrix.pre_build == '{hook}'`"
        )


# --- 8. update-image-versions идёт по выводу генератора (Phase 3) ----------


def test_update_image_versions_reads_tags_from_manifest():
    job = _wf()["jobs"]["update-image-versions"]
    run = "\n".join(step.get("run") or "" for step in job["steps"])

    assert "gen_build_matrix.py" in run and "--mode tags" in run
    assert MANIFEST_REL in run
    assert '--pi "$PI"' in run
    # Запись файлов — общий скрипт (тот же у Single Service), а не свой sed.
    assert "scripts/ci/write-image-versions.sh" in run
    assert "sed -i" not in run
    assert (
        "scripts/ci/push-image-versions.sh" in run
    ), "пропал вызов scripts/ci/push-image-versions.sh (ADR-0094)"
    assert 'git commit -m "ci: ${PI} SHA tags' in run


def test_single_service_writes_through_the_same_script():
    body = _strip_comments(SINGLE_WF)
    assert "scripts/ci/write-image-versions.sh" in body
    assert 'sed -i "s|^${VERSION_VAR}=' not in body
    pi_type = _on(_load_yaml(SINGLE_WF))["workflow_dispatch"]["inputs"]["pi_type"]
    assert set(_pis()) <= set(pi_type["options"]), pi_type["options"]


@pytest.mark.parametrize("pi", ["vision", "main"])
def test_known_phantom_tags_are_gone(pi):
    """Phantom-поля .image-versions.* удалены (план §1.3/§1.4, ADR-0094 §1.4, §3.3).

    RTABMAP_SYNC_TAG (vision) и MICRO_ROS_AGENT_TAG (main) не принадлежали ни
    одному сервису манифеста. Guard ловит их возвращение: в .image-versions.*,
    в теле workflow сборки Pi и в L-Build Single Service.yml.
    """
    phantoms = {
        "vision": {"RTABMAP_SYNC_TAG"},
        "main": {"MICRO_ROS_AGENT_TAG"},
    }[pi]
    body = _strip_comments(PI_WF)
    single_service = _strip_comments(SINGLE_WF)

    for var in phantoms:
        for suffix in ("dev", "test", "latest"):
            versions_file = REPO_ROOT / "docker" / pi / f".image-versions.{suffix}"
            if not versions_file.exists():
                continue
            assert f"{var}=" not in versions_file.read_text(
                encoding="utf-8"
            ), f"{pi}: {var} снова в {versions_file.name} — поле никто не читает"
        assert var not in body
        assert var not in single_service

    manifest_vars = set(gen_build_matrix.image_versions_map(_services(pi)))
    assert not (phantoms & manifest_vars)


@pytest.mark.parametrize("pi", ["vision", "sound", "main"])
def test_image_versions_file_has_a_field_for_every_manifest_service(pi):
    """Каждая переменная image_versions манифеста реально есть в
    docker/<pi>/.image-versions.dev — иначе запись прошла бы вхолостую.
    """
    versions_text = (REPO_ROOT / "docker" / pi / ".image-versions.dev").read_text(
        encoding="utf-8"
    )
    for var in gen_build_matrix.image_versions_map(_services(pi)):
        assert f"{var}=" in versions_text, (
            f"{pi}: манифест объявляет {var}, но такого поля нет в "
            f"docker/{pi}/.image-versions.dev"
        )


# --- 9. соседние поля .image-versions.* пишет одна сборка ------------------


def test_adjacent_image_versions_fields_have_one_writer():
    """Build All собирает Pi параллельно, и две сборки могут править один файл
    (PERCEPTION_TAG пишет Main в docker/sound/). Правки соседних строк git не
    сливает: rebase в push-image-versions.sh падает. Между полями разных
    сборок должна стоять хотя бы одна неизменяемая строка (комментарий).
    """
    owner = {
        var: pi
        for pi in _pis()
        for var in gen_build_matrix.image_versions_map(_services(pi))
    }
    for path in sorted((REPO_ROOT / "docker").glob("*/.image-versions.*")):
        lines = path.read_text(encoding="utf-8").splitlines()
        for upper, lower in zip(lines, lines[1:]):
            a, b = upper.split("=", 1)[0], lower.split("=", 1)[0]
            if a in owner and b in owner:
                assert owner[a] == owner[b], (
                    f"{path.relative_to(REPO_ROOT)}: {a} (сборка {owner[a]}) и {b} "
                    f"(сборка {owner[b]}) на соседних строках — параллельные коммиты "
                    f"конфликтуют на rebase; вставь между ними строку-комментарий"
                )


# --- smoke: манифест вообще парсится и покрывает три Pi --------------------


def test_manifest_declares_all_pis():
    assert set(_pis()) >= {"vision", "sound", "main"}, _pis()
