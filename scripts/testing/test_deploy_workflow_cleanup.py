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