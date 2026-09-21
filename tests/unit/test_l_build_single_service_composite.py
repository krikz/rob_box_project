"""Regression guard: `L-Build Single Service.yml` builds through the shared
composite action, not through its own `docker buildx build`.

Хендофф 2026-09-21 §8 / docs/plans/2026-09-15-service-manifest.md Phase 4:
"L-Build Single Service.yml" не просто дублировал список сервисов — он не
использовал ./.github/actions/l-build-service ВООБЩЕ, у него был собственный
`docker buildx build`. Из-за этого registry-кеш (--cache-from/--cache-to,
mode=max), docker-container билдер и подмена --add-host host-gateway на
реальный IP шлюза bridge-сети, заведённые в композите (issue #2280,
docs/plans/2026-09-15-builder-runtime-seam.md §6), на одиночные сборки не
распространялись (обнаружено, когда проверочная сборка led-matrix через этот
workflow не показала в логе ни одной строки "Cache:").

Этот файл — младший брат tests/unit/test_workflow_refactor_acceptance.py
(тот же приём: `yaml.safe_load` над реальным workflow, никакого regex по
тексту), но для Single Service, а не для матричных Main/Vision workflow.
Single Service НЕ описан в docker/build-manifest.yaml (план §11, вопрос №2 —
base-образы и zenoh-router вне манифеста), поэтому в отличие от
test_workflow_refactor_acceptance.py здесь нет сверки с генератором — только
структурные проверки самого workflow.
"""

from __future__ import annotations

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
WORKFLOW = REPO_ROOT / ".github" / "workflows" / "L-Build Single Service.yml"

BUILD_JOB_NAME = "build-service"


def _load() -> dict:
    # Workflow YAML содержит UTF-8 русские комментарии; на Windows дефолтная
    # кодировка cp1252 бросила бы UnicodeDecodeError.
    with WORKFLOW.open(encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _build_job() -> dict:
    data = _load()
    assert BUILD_JOB_NAME in data["jobs"], (
        f"{WORKFLOW.name}: job '{BUILD_JOB_NAME}' не найден, "
        f"есть: {sorted(data['jobs'])}"
    )
    return data["jobs"][BUILD_JOB_NAME]


def _composite_step(steps: list[dict]) -> dict | None:
    for s in steps:
        if (s.get("uses") or "").endswith("/actions/l-build-service"):
            return s
    return None


def test_build_service_uses_composite_action():
    """Headline-регрессия хендоффа §8: билд обязан идти через композит.

    Раньше в этом job'е не было ни одного `uses:`-шага — только inline bash
    с собственным `docker buildx build`.
    """
    job = _build_job()
    step = _composite_step(job["steps"])
    assert step is not None, (
        f"{BUILD_JOB_NAME}: нет шага с uses: ./.github/actions/l-build-service, "
        f"шаги: {[s.get('name') or s.get('uses') for s in job['steps']]}"
    )


def test_composite_is_called_after_checkout():
    """Локальный composite резолвится только из зачекаутенного workspace.

    Тот же класс регрессии, что уже ловил run #34366133083 для Main/Vision
    (composite первым шагом → "Did you forget to run actions/checkout").
    """
    job = _build_job()
    steps = job["steps"]
    composite_idx = next(
        (i for i, s in enumerate(steps) if (s.get("uses") or "").endswith("/actions/l-build-service")),
        None,
    )
    assert composite_idx is not None, f"{BUILD_JOB_NAME}: composite step missing"
    assert composite_idx > 0, (
        f"{BUILD_JOB_NAME}: composite — первый шаг, репозиторий ещё не зачекаутен"
    )
    assert [
        s for s in steps[:composite_idx] if (s.get("uses") or "").startswith("actions/checkout")
    ], f"{BUILD_JOB_NAME}: нет actions/checkout до вызова composite"


def test_no_inline_buildx_build_outside_composite():
    """Негативный контроль: в текстовых шагах job'а не должно быть

    собственного `docker buildx build` — это и есть симптом, из-за которого
    завели этот guard (§8 хендоффа). `docker push`/`docker tag` в шаге
    "Tag with SHA and update .image-versions" — легитимны (SHA-иммутабельный
    тег для .image-versions, тот же паттерн, что update-image-versions job в
    Main/Vision), поэтому здесь запрещён только сам `docker buildx build`,
    а не любой docker-вызов.
    """
    job = _build_job()
    for step in job["steps"]:
        if (step.get("uses") or "").endswith("/actions/l-build-service"):
            continue  # buildx-логика внутри композита — это и есть цель
        run = step.get("run", "")
        if not isinstance(run, str):
            continue
        assert "docker buildx build" not in run, (
            f"{BUILD_JOB_NAME}: шаг '{step.get('name')}' содержит inline "
            f"`docker buildx build` — должно быть внутри composite action"
        )


def test_composite_receives_dockerfile_and_context_from_prepare_job():
    """`with:` заполняется из needs.prepare.outputs, а не вписан руками.

    Single Service вычисляет dockerfile/build_context/base_image в job'е
    `prepare` (per-service case, не манифест — план §11 вопрос №2); composite
    обязан читать именно эти outputs, а не дублировать case-логику у себя.
    """
    job = _build_job()
    step = _composite_step(job["steps"])
    assert step is not None
    with_block = step.get("with") or {}
    assert with_block.get("dockerfile-path") == "${{ needs.prepare.outputs.dockerfile_path }}"
    assert with_block.get("build-context") == "${{ needs.prepare.outputs.build_context }}"
    assert with_block.get("service-name") == "${{ needs.prepare.outputs.image_name }}"
    tags = with_block.get("tags") or ""
    assert "needs.prepare.outputs.image_tag" in tags
    assert "needs.prepare.outputs.local_tag" in tags


def test_prepare_job_exposes_image_name_output():
    """`image_name` (hyphenated) — обязателен: это service-name композита,

    от него зависит cache-ref (<service-name>-buildcache). Без него
    одиночная сборка либо не находит cache-ref вовсе, либо заводит СВОЙ
    (под service_name, underscored) вместо того, чтобы делить кеш с
    matrix-job'ом Main/Vision Pi, который использует то же hyphenated имя.
    """
    data = _load()
    prepare_outputs = data["jobs"]["prepare"]["outputs"]
    assert "image_name" in prepare_outputs, (
        "prepare job должен экспортировать outputs.image_name (hyphenated "
        "имя сервиса, == matrix.name в Main/Vision) для composite's service-name"
    )
