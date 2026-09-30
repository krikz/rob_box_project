import re
from pathlib import Path


WORKFLOW_PATH = Path(__file__).resolve().parents[2] / ".github/workflows/L-Deploy and Verify.yml"

# Раньше очистка была двумя шагами «[Vision Pi] / [Main Pi] Cleanup Docker
# Artifacts»; разбор тайминга run 35645507502 слил их в один параллельный
# пост-деплойный шаг.
CLEANUP_STEP = "[Vision Pi + Main Pi] Cleanup Docker Artifacts (parallel)"


def _steps(workflow):
    """Имена шагов и их тексты по порядку (шаг начинается с `      - name:`)."""
    parts = re.split(r"(?m)^      - name: ", workflow)[1:]
    return [(p.splitlines()[0].strip().strip('"'), p) for p in parts]


def test_deploy_workflow_prunes_unused_docker_artifacts_on_both_pis():
    steps = dict(_steps(WORKFLOW_PATH.read_text(encoding="utf-8")))

    assert CLEANUP_STEP in steps, f"шаг очистки не найден: {CLEANUP_STEP}"
    body = steps[CLEANUP_STEP]
    for host in ("env.vision_pi_ip", "env.main_pi_ip"):
        assert host in body, f"очистка не ходит на {host}"
    assert body.count("docker image prune -af") >= 2, "image prune должен идти на обоих Pi"
    assert body.count("docker builder prune -af") >= 2, "builder prune должен идти на обоих Pi"


def test_deploy_workflow_never_prunes_while_stack_is_stopped():
    """`docker image prune -af` удаляет все образы без контейнеров. Если он
    выполнится между Stop Containers (`compose down`) и Start Containers,
    он снесёт образы стека, которые только что скачал/проверил Pull-шаг
    (#2930), и `up --pull never` упадёт. Исходно очистка стояла ДО Stop,
    сейчас — после Start; инвариант один: не внутри окна Stop..Start."""
    names = [name for name, _ in _steps(WORKFLOW_PATH.read_text(encoding="utf-8"))]
    cleanup = names.index(CLEANUP_STEP)

    for pi in ("Vision Pi", "Main Pi"):
        stop = names.index(f"[{pi}] Stop Containers")
        start = names.index(f"[{pi}] Start Containers")
        assert stop < start
        assert not stop < cleanup < start, (
            f"{CLEANUP_STEP} стоит между [{pi}] Stop и Start — prune снесёт образы остановленного стека"
        )


def test_environment_local_is_aliased_to_dev():
    """Ретро 18.08 (issue #1379, t_544e4aa5): environment=local раньше давал
    IMAGE_TAG=local, которого нет ни в local, ни в github registry → deploy fail.
    Теперь должен быть алиас на IMAGE_TAG=dev с явным WARNING-логом."""
    workflow = WORKFLOW_PATH.read_text(encoding="utf-8")

    # Внутри case "local)" должна быть строка IMAGE_TAG=dev (а не IMAGE_TAG=local)
    local_block_idx = workflow.index('local)')
    # следующая IMAGE_TAG после этого индекса
    local_block = workflow[local_block_idx:local_block_idx + 400]
    assert "IMAGE_TAG=dev" in local_block, (
        "Ретро #1379: environment=local должен алиаситься на IMAGE_TAG=dev, "
        "а не оставаться IMAGE_TAG=local (голый тег 'local' в registry не "
        "публикуется — compose pull падает)."
    )
    # Должен быть явный warning с упоминанием ретро
    assert "WARNING: environment=local" in workflow
    assert "issue #1379" in workflow


def test_deploy_pull_ignores_compose_pull_policy():
    """Issue #2609 (16.09): docker/vision/docker-compose.yaml ставит
    pull_policy: if_not_present (#2634). С ним голый `docker compose pull`
    пишет «Skipped Image is already present locally» и не обновляет :dev —
    деплой проходил зелёным, а робот оставался на старых образах."""
    workflow = WORKFLOW_PATH.read_text(encoding="utf-8")
    # Issue #2930: сам pull переехал в preflight-скрипт, который оба
    # Pull-шага (Vision Pi и Main Pi) вызывают перед Stop Containers.
    preflight = (Path(__file__).resolve().parents[2] / "scripts/deploy/preflight_pull_images.sh").read_text(encoding="utf-8")
    assert workflow.count("scripts/deploy/preflight_pull_images.sh") >= 2, "ожидались pull-шаги для Vision Pi и Main Pi"

    pulls = [
        line
        for line in (workflow + "\n" + preflight).splitlines()
        if "docker compose pull" in line and not line.strip().startswith("#") and "echo" not in line
    ]
    assert pulls, "docker compose pull не найден ни в workflow, ни в preflight"
    for line in pulls:
        assert "--policy always" in line, f"pull без --policy always: {line.strip()}"


def test_vision_autostart_pull_ignores_compose_pull_policy():
    setup = (Path(__file__).resolve().parents[2] / "scripts/setup/setup_vision_pi.sh").read_text(encoding="utf-8")
    exec_pre = [line for line in setup.splitlines() if line.startswith("ExecStartPre=") and "compose pull" in line]
    assert exec_pre and all("--policy always" in line for line in exec_pre)
