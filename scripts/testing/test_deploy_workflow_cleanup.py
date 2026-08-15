from pathlib import Path


WORKFLOW_PATH = Path(__file__).resolve().parents[2] / ".github/workflows/L-Deploy and Verify.yml"


def test_deploy_workflow_prunes_unused_docker_artifacts_on_both_pis():
    workflow = WORKFLOW_PATH.read_text(encoding="utf-8")

    assert "[Vision Pi] Cleanup Docker Artifacts" in workflow
    assert "[Main Pi] Cleanup Docker Artifacts" in workflow
    assert "docker image prune -af" in workflow
    assert "docker builder prune -af" in workflow


def test_deploy_workflow_cleans_up_after_starting_containers():
    workflow = WORKFLOW_PATH.read_text(encoding="utf-8")

    # Cleanup happens AFTER containers are started (post-deploy prune), so a
    # failed start never destroys images the running stack may still need.
    assert workflow.index('[Vision Pi] Cleanup Docker Artifacts') > workflow.index('[Vision Pi] Start Containers')
    assert workflow.index('[Main Pi] Cleanup Docker Artifacts') > workflow.index('[Main Pi] Start Containers')


def test_deploy_workflow_verifies_images_before_stopping_containers():
    workflow = WORKFLOW_PATH.read_text(encoding="utf-8")

    # Ретро 15.08 t_d1263bef (#1315): deploy упал на Start Containers —
    # в реестре не было образов, а pull с --ignore-pull-failures молча
    # проглотил отсутствие. Проверка образов обязана идти ДО остановки
    # контейнеров, чтобы при нехватке образов робот продолжал работать
    # на старом стеке.
    assert workflow.index('[Vision Pi] Verify Images Available') < workflow.index('[Vision Pi] Stop Containers')
    assert workflow.index('[Main Pi] Verify Images Available') < workflow.index('[Main Pi] Stop Containers')


def test_deploy_workflow_pull_fails_on_missing_required_images():
    workflow = WORKFLOW_PATH.read_text(encoding="utf-8")

    # --ignore-pull-failures остаётся только для внешних образов; после pull
    # обязательные образы compose проверяются через docker image inspect,
    # и при нехватке шаг падает с actionable-сообщением (не «No such image»).
    assert "docker compose pull --ignore-pull-failures" in workflow
    assert "Required images missing after pull" in workflow
    assert "docker compose config --images" in workflow
    assert "docker image inspect" in workflow
