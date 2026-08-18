from pathlib import Path


WORKFLOW_PATH = Path(__file__).resolve().parents[2] / ".github/workflows/L-Deploy and Verify.yml"


def test_deploy_workflow_prunes_unused_docker_artifacts_on_both_pis():
    workflow = WORKFLOW_PATH.read_text(encoding="utf-8")

    assert "[Vision Pi] Cleanup Docker Artifacts" in workflow
    assert "[Main Pi] Cleanup Docker Artifacts" in workflow
    assert "docker image prune -af" in workflow
    assert "docker builder prune -af" in workflow


def test_deploy_workflow_cleans_up_before_stopping_containers():
    workflow = WORKFLOW_PATH.read_text(encoding="utf-8")

    assert workflow.index('[Vision Pi] Cleanup Docker Artifacts') < workflow.index('[Vision Pi] Stop Containers')
    assert workflow.index('[Main Pi] Cleanup Docker Artifacts') < workflow.index('[Main Pi] Stop Containers')


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
