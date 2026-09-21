"""Guard: docker/build-manifest.yaml ⇔ реальные L-Build workflow.

docs/plans/2026-09-15-service-manifest.md, §6.

Режим Phase 1 (манифест только ДОГОНЯЛ руками написанные workflow) закрыт:
после Phase 2 workflow-файлы больше не содержат списка сервисов — они
вычисляют matrix из этого же манифеста в `prepare`-job'е. Поэтому guard
сменил предмет проверки ровно так, как предписывает план §5 Phase 2 п.4:
он проверяет не «совпадают ли два списка», а «действительно ли workflow
берёт состав сборки из docker/build-manifest.yaml, а не из копии списка».
Конкретные значения параметров сборки (теги, build-args) покрыты тестами
генератора (scripts/ci/tests/test_gen_build_matrix.py) и acceptance-тестами
(tests/unit/test_workflow_refactor_acceptance.py), здесь не дублируются.

Что проверяется:

1. `prepare` зовёт scripts/ci/gen_build_matrix.py ИМЕННО с этим манифестом
   и отдаёт matrix/chained/services как outputs job'а.
2. matrix-job `build` разворачивается из fromJSON(needs.prepare.outputs.matrix),
   а именованные job'ы — из своего элемента chained по ИМЕНИ.
3. Множество именованных build-* job'ов == множеству сервисов графа
   зависимостей манифеста (ни больше, ни меньше), и их needs: совпадает с
   depends_on (инвариант §2.3).
4. НЕГАТИВНЫЙ КОНТРОЛЬ: имя сервиса из манифеста не встречается в
   исполняемом (не-комментарии) тексте workflow нигде, кроме id именованного
   job'а и обращения к своему элементу chained. Именно это отличает
   «workflow читает данные» от «в workflow снова завёлся список».
5. env: workflow'а (IMAGE_PREFIX/LOCAL_PREFIX/ROS_DISTRO) == defaults
   манифеста — иначе генератор разрешит теги не теми префиксами, которыми
   реально собирает CI.
6. Все build-job'ы одного workflow имеют ПОБАЙТОВО один и тот же список
   шагов (план §5 Phase 2 п.3: тело именованного job'а обязано дословно
   совпадать с шаблоном matrix-job'а).
7. Каждый pre_build-хук манифеста реализован шагом в том workflow, где
   живёт его сервис.
8. tag_and_push/verify_in_registry/sed в update-image-versions идут по
   выводу `gen_build_matrix.py --mode tags` (Phase 3), а не по ручному
   списку; механизм push (ADR-0094) при этом на месте, а phantom-поля
   .image-versions.* больше никем не переписываются.

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
MAIN_WF = REPO_ROOT / ".github" / "workflows" / "L-Build Main Pi Services.yml"
VISION_WF = REPO_ROOT / ".github" / "workflows" / "L-Build Vision Pi Services.yml"
GEN_SCRIPT = REPO_ROOT / "scripts" / "ci" / "gen_build_matrix.py"

PI_TO_WORKFLOW = {"main": MAIN_WF, "vision": VISION_WF}

# Путь к манифесту, который workflow ОБЯЗАН передавать генератору — тот же,
# что читает этот тест. Если кто-то заведёт второй манифест-копию, п.1 упадёт.
MANIFEST_REL = "docker/build-manifest.yaml"


# --- helpers ---------------------------------------------------------------


def _load_yaml(path: Path) -> dict:
    # Как и в test_workflow_refactor_acceptance.py: workflow-файлы содержат
    # русские комментарии в UTF-8, на Windows дефолтная codepage — cp1252.
    with path.open(encoding="utf-8") as fh:
        return yaml.safe_load(fh)


_SPEC = importlib.util.spec_from_file_location("gen_build_matrix_guard", GEN_SCRIPT)
assert _SPEC is not None and _SPEC.loader is not None, f"{GEN_SCRIPT}: spec failed"
gen_build_matrix = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(gen_build_matrix)  # type: ignore[union-attr]


def _manifest() -> dict[str, Any]:
    return gen_build_matrix.load_manifest(MANIFEST_PATH)


def _services(pi: str) -> dict[str, Any]:
    return gen_build_matrix.services_for_pi(_manifest(), pi)


def _build_jobs(data: dict) -> dict[str, dict]:
    """Именованные build-* job'ы (общий matrix-job называется `build`)."""
    return {k: v for k, v in data["jobs"].items() if k.startswith("build-")}


def _prepare_gen_step(data: dict) -> dict:
    for step in data["jobs"]["prepare"]["steps"]:
        if "gen_build_matrix.py" in (step.get("run") or ""):
            return step
    raise AssertionError("prepare: нет шага, вызывающего scripts/ci/gen_build_matrix.py")


def _strip_comments(path: Path, skip_jobs: frozenset[str] = frozenset()) -> str:
    """Текст workflow без строк-комментариев (для негативного контроля).

    `skip_jobs` вырезает блоки указанных job'ов целиком — нужно, пока какой-то
    job ещё не переведён на манифест (см. вызов в негативном контроле).
    """
    lines = []
    skipping = False
    for line in path.read_text(encoding="utf-8").splitlines():
        if re.match(r"^  [A-Za-z0-9_-]+:\s*$", line):
            skipping = line.strip().rstrip(":") in skip_jobs
        if skipping or line.lstrip().startswith("#"):
            continue
        lines.append(line)
    return "\n".join(lines)


# --- 1. prepare читает ИМЕННО этот манифест --------------------------------


@pytest.mark.parametrize("pi", ["vision", "main"])
def test_prepare_generates_matrix_from_the_manifest(pi):
    data = _load_yaml(PI_TO_WORKFLOW[pi])
    step = _prepare_gen_step(data)
    run = step["run"]

    assert MANIFEST_REL in run, (
        f"{pi}: prepare обязан звать генератор с --manifest {MANIFEST_REL} "
        f"(иначе матрица считается по какому-то другому файлу)"
    )
    assert f"--pi {pi}" in run, f"{pi}: prepare зовёт генератор не для своего Pi"
    for mode in ("--mode matrix", "--mode chained"):
        assert mode in run, f"{pi}: prepare не вызывает генератор с {mode}"
    assert "$GITHUB_OUTPUT" in run, f"{pi}: результат генератора не попадает в outputs"

    outputs = data["jobs"]["prepare"]["outputs"]
    for key in ("matrix", "chained", "services"):
        assert key in outputs, f"{pi}: prepare.outputs.{key} отсутствует"
        assert f"steps.{step['id']}.outputs.{key}" in outputs[key], (
            f"{pi}: prepare.outputs.{key} не ссылается на шаг генератора"
        )


# --- 2/3. matrix-job и именованные job'ы разворачиваются из манифеста ------


@pytest.mark.parametrize("pi", ["vision", "main"])
def test_matrix_job_expands_prepare_output(pi):
    data = _load_yaml(PI_TO_WORKFLOW[pi])
    matrix = data["jobs"]["build"]["strategy"]["matrix"]
    assert "fromJSON(needs.prepare.outputs.matrix)" in str(matrix["include"]), (
        f"{pi}: job `build` обязан разворачиваться из "
        f"fromJSON(needs.prepare.outputs.matrix), получено {matrix!r}"
    )
    assert data["jobs"]["build"]["strategy"].get("fail-fast") is False, (
        f"{pi}: без fail-fast: false падение одного сервиса отменяло бы сборку "
        f"остальных — это изменение поведения относительно отдельных job'ов"
    )


@pytest.mark.parametrize("pi", ["vision", "main"])
def test_named_jobs_match_manifest_dependency_graph(pi):
    services = _services(pi)
    data = _load_yaml(PI_TO_WORKFLOW[pi])

    expected = {
        f"build-{name}" for name in gen_build_matrix.named_service_names(services)
    }
    actual = set(_build_jobs(data))
    assert actual == expected, (
        f"{pi}: именованными job'ами должны остаться РОВНО сервисы графа "
        f"зависимостей манифеста {sorted(expected)}, в workflow {sorted(actual)}. "
        f"Лишний именованный job = список сервисов снова просочился в workflow; "
        f"недостающий = цель ребра попала в matrix, и needs: на неё сослаться "
        f"не сможет (план §3.1)."
    )

    for job_name, job in _build_jobs(data).items():
        name = job_name.removeprefix("build-")
        include = str(job["strategy"]["matrix"]["include"])
        assert "fromJSON(needs.prepare.outputs.chained)" in include, (
            f"{pi}/{job_name}: тело job'а должно брать свой элемент из "
            f"chained-вывода генератора, получено {include!r}"
        )
        assert f"'{name}'" in include, (
            f"{pi}/{job_name}: обращение к своему элементу chained должно быть "
            f"по имени '{name}' (обращение по индексу молча поехало бы при "
            f"добавлении сервиса в цепочку), получено {include!r}"
        )


@pytest.mark.parametrize("pi", ["vision", "main"])
def test_named_jobs_needs_match_depends_on(pi):
    """needs: именованного job'а == prepare + build-<каждый depends_on>.

    Тот самый инвариант, ради которого voice-assistant/supervisor не
    растворились в общей matrix (план §3.4): supervisor ждёт ИМЕННО
    voice-assistant, а не весь набор сервисов.
    """
    services = _services(pi)
    data = _load_yaml(PI_TO_WORKFLOW[pi])

    for job_name, job in _build_jobs(data).items():
        name = job_name.removeprefix("build-")
        needs = job.get("needs")
        needs = [needs] if isinstance(needs, str) else list(needs or [])
        expected = ["prepare"] + [
            f"build-{dep}" for dep in (services[name].get("depends_on") or [])
        ]
        assert needs == expected, (
            f"{pi}/{job_name}: needs: {needs} != ожидаемого из манифеста "
            f"{expected} (depends_on={services[name].get('depends_on')})"
        )


# --- 4. негативный контроль: списка сервисов в workflow больше нет ---------


@pytest.mark.parametrize("pi", ["vision", "main"])
def test_no_service_names_left_in_workflow_body(pi):
    """Имя сервиса не должно встречаться в исполняемом тексте workflow.

    Разрешены ровно два места: id именованного job'а (`build-voice-base:`)
    и обращение к своему элементу chained (`['voice-base']`). Всё остальное —
    признак того, что список сервисов снова просочился в workflow (ровно та
    болезнь, ради которой заведён манифест, план §1.1).
    """
    services = _services(pi)
    named = set(gen_build_matrix.named_service_names(services))
    body = _strip_comments(PI_TO_WORKFLOW[pi])

    for name in services:
        # У сервиса графа зависимостей легально остаются id его job'а
        # (`build-voice-base:`), ссылки на этот id в needs: и обращение к
        # своему элементу chained. Вырезаем эти формы из строки и смотрим,
        # осталось ли имя сервиса где-то ещё.
        allowed = [f"build-{name}", f"['{name}']"] if name in named else []
        leftovers = []
        for i, line in enumerate(body.splitlines(), 1):
            if name not in line:
                continue
            rest = line
            for token in allowed:
                rest = rest.replace(token, "")
            if name not in rest:
                continue
            leftovers.append(f"    {i}: {line.strip()}")
        assert not leftovers, (
            f"{pi}: имя сервиса {name!r} осталось в теле workflow — состав "
            f"сборки обязан приходить из {MANIFEST_REL}:\n" + "\n".join(leftovers)
        )


# --- 5. env workflow == defaults манифеста ---------------------------------


@pytest.mark.parametrize("pi", ["vision", "main"])
def test_workflow_env_matches_manifest_defaults(pi):
    """Генератор разрешает теги префиксами, которые ему передаёт workflow из
    своих env. Если env и defaults манифеста разойдутся, guard-и и тесты
    генератора будут проверять не то, что реально собирает CI.
    """
    defaults = _manifest()["defaults"]
    env = _load_yaml(PI_TO_WORKFLOW[pi])["env"]

    assert env["IMAGE_PREFIX"] == defaults["registry"]["ghcr"]
    assert env["LOCAL_PREFIX"] == defaults["registry"]["local"]
    assert env["ROS_DISTRO"] == defaults["ros_distro"]
    assert env["LOCAL_REGISTRY"] == defaults["registry"]["local_registry_host"]


# --- 6. тело всех build-job'ов — один шаблон -------------------------------


@pytest.mark.parametrize("pi", ["vision", "main"])
def test_all_build_jobs_share_one_step_template(pi):
    """План §5 Phase 2 п.3: тело именованного job'а дословно совпадает с
    шаблоном matrix-job'а — отличается только источник данных матрицы.
    """
    data = _load_yaml(PI_TO_WORKFLOW[pi])
    template = data["jobs"]["build"]["steps"]
    for job_name, job in _build_jobs(data).items():
        assert job["steps"] == template, (
            f"{pi}/{job_name}: шаги разошлись с шаблоном matrix-job'а `build`. "
            f"Именно из такого расхождения копий вырастают stale-слои "
            f"(issue #2314) — правь шаблон целиком, а не одну копию."
        )


# --- 7. pre_build-хуки манифеста реализованы в workflow --------------------


@pytest.mark.parametrize("pi", ["vision", "main"])
def test_pre_build_hooks_are_implemented(pi):
    services = _services(pi)
    hooks = {
        svc["pre_build"] for svc in services.values() if svc.get("pre_build")
    }
    body = _strip_comments(PI_TO_WORKFLOW[pi])
    for hook in hooks:
        assert f"matrix.pre_build == '{hook}'" in body, (
            f"{pi}: манифест объявляет pre_build-хук {hook!r}, но в workflow нет "
            f"шага с `if: matrix.pre_build == '{hook}'` — сервис собрался бы без "
            f"предсборочной подготовки и упал бы внутри docker build"
        )


# --- 8. update-image-versions идёт по выводу генератора (Phase 3) ----------

# sed собирается циклом по выводу генератора, поэтому шаблон подстановки
# один, с переменной цикла вместо имени поля.
_SED_LOOP_RE = re.compile(r"s\|\^\$\{VERSION_VAR\}=")


@pytest.mark.parametrize("pi", ["vision", "main"])
def test_update_image_versions_reads_tags_from_manifest(pi):
    """tag_and_push/verify_in_registry/sed больше не перечисляют сервисы.

    План §5 Phase 3 п.1-2: список `*_TAG`-переменных и соответствующих им
    тегов приходит из `gen_build_matrix.py --mode tags --format shell`.
    Сам механизм push (SHA-тег, коммит в develop, push-image-versions.sh)
    этим не затрагивается — это территория ADR-0094.
    """
    data = _load_yaml(PI_TO_WORKFLOW[pi])
    job = data["jobs"]["update-image-versions"]
    run = "\n".join(step.get("run") or "" for step in job["steps"])

    assert "gen_build_matrix.py" in run and "--mode tags" in run, (
        f"{pi}: update-image-versions обязан брать список *_TAG из "
        f"`gen_build_matrix.py --mode tags`, а не из ручного перечисления"
    )
    assert MANIFEST_REL in run, f"{pi}: update-image-versions читает не тот манифест"
    assert _SED_LOOP_RE.search(run), (
        f"{pi}: sed по .image-versions.* должен собираться из переменной цикла "
        f"($VERSION_VAR), а не из захардкоженных имён полей"
    )

    # Ни одно имя *_TAG-переменной манифеста не вписано руками.
    for var in gen_build_matrix.image_versions_map(_services(pi)):
        assert var not in run, (
            f"{pi}: переменная {var} перечислена в update-image-versions руками — "
            f"это снова копия списка из {MANIFEST_REL}"
        )

    # push-механизм ADR-0094 остался на месте (Phase 3 его НЕ трогает).
    assert "scripts/ci/push-image-versions.sh" in run, (
        f"{pi}: пропал вызов scripts/ci/push-image-versions.sh — Phase 3 не "
        f"должна была трогать механизм push (ADR-0094)"
    )


@pytest.mark.parametrize("pi", ["vision", "main"])
def test_known_phantom_tags_are_gone(pi):
    """Phantom-поля .image-versions.* удалены (план §1.3/§1.4, ADR-0094 §1.4, §3.3).

    RTABMAP_SYNC_TAG (vision) и MICRO_ROS_AGENT_TAG (main) не принадлежали ни
    одному сервису манифеста: rtabmap собирается только на Main Pi, а
    micro-ros-agent не собирается вовсе. До Phase 3 sed писал их наравне с
    настоящими; Phase 3 перестала их обновлять, а этот шаг
    (docs/plans/2026-09-15-image-versions-seam.md §7.2) удалил сами поля.

    Предыдущая версия теста держала обратный факт — «поля ещё лежат, их никто
    не обновляет» — и прямо просила переписать себя, когда phantom вычистят.
    Переписана: теперь guard стоит с другой стороны и ловит их возвращение.
    Вернуть phantom можно ровно двумя способами, оба закрыты:
    дописать поле руками в .image-versions.* или вернуть ветку
    PI_TYPE=vision у rtabmap в L-Build Single Service.yml.
    """
    phantoms = {
        "vision": {"RTABMAP_SYNC_TAG"},
        "main": {"MICRO_ROS_AGENT_TAG"},
    }[pi]
    body = _strip_comments(PI_TO_WORKFLOW[pi])
    single_service = _strip_comments(
        REPO_ROOT / ".github" / "workflows" / "L-Build Single Service.yml"
    )

    for var in phantoms:
        for suffix in ("dev", "test", "latest"):
            versions_file = REPO_ROOT / "docker" / pi / f".image-versions.{suffix}"
            if not versions_file.exists():
                continue
            assert f"{var}=" not in versions_file.read_text(encoding="utf-8"), (
                f"{pi}: {var} снова в {versions_file.name} — это поле никто не "
                f"читает (scripts/ci/check_image_versions_usage.sh), удалено "
                f"по ADR-0094 §3.3"
            )
        assert var not in body, (
            f"{pi}: {var} снова появился в исполняемом тексте workflow"
        )
        assert var not in single_service, (
            f"{pi}: {var} вернулся в L-Build Single Service.yml — один запуск "
            f"этого workflow допишет phantom обратно в .image-versions.*"
        )

    manifest_vars = set(gen_build_matrix.image_versions_map(_services(pi)))
    assert not (phantoms & manifest_vars), (
        f"{pi}: phantom-поля {sorted(phantoms & manifest_vars)} внезапно обрели "
        f"владельца в манифесте — обнови этот тест"
    )

@pytest.mark.parametrize("pi", ["vision", "main"])
def test_image_versions_file_has_a_field_for_every_manifest_service(pi):
    """Каждая переменная image_versions манифеста реально есть в
    docker/<pi>/.image-versions.dev.

    Без этого sed отработал бы вхолостую (шаблон ^VAR= ничего не нашёл бы),
    а deploy продолжил тянуть старый тег — молча.
    """
    versions_text = (
        REPO_ROOT / "docker" / pi / ".image-versions.dev"
    ).read_text(encoding="utf-8")
    for var in gen_build_matrix.image_versions_map(_services(pi)):
        assert f"{var}=" in versions_text, (
            f"{pi}: манифест объявляет {var}, но такого поля нет в "
            f"docker/{pi}/.image-versions.dev — sed не нашёл бы что заменить"
        )


# --- smoke: манифест вообще парсится и покрывает оба pi ---------------------


def test_manifest_declares_both_pis():
    manifest = _manifest()
    assert set(manifest["pis"]) >= {"vision", "main"}, (
        f"manifest.pis must declare at least 'vision' and 'main', "
        f"got {sorted(manifest['pis'])}"
    )
