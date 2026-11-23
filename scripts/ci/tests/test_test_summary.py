"""Сводка G: Run Tests не выдаёт skipped интеграцию за пройденную (#5116).

Integration Tests на pull_request skipped (if по ref develop/main) и в needs
сводки её нет. Раньше шаг писал «All tests passed!». Решение Шифу 22.11.2026:
интеграцию воротами PR не делаем, PR-CI не удлиняем — skipped не красит
прогон, но текст сводки говорит прямо, что интеграция не запускалась.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_PATH = REPO_ROOT / "scripts" / "ci" / "test_summary.py"

_SPEC = importlib.util.spec_from_file_location("test_summary_under_test", SCRIPT_PATH)
assert _SPEC is not None and _SPEC.loader is not None, f"{SCRIPT_PATH}: spec failed"
test_summary = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(test_summary)  # type: ignore[union-attr]


REQUIRED = [
    ("Unit Tests", "success"),
    ("Unit Tests (rob_box_mcp_tools)", "success"),
    ("E2E Contract Guards", "success"),
    ("URDF sim-arg", "success"),
]


def test_pr_skipped_is_not_called_passed():
    text, code = test_summary.render_summary("pull_request", REQUIRED, "")
    assert code == 0
    assert "не запускались (skipped на PR, только develop/main)" in text
    assert "это не «интеграция прошла»" in text
    assert "All tests passed" not in text
    assert "интеграция прошла" in text  # только внутри отрицания


def test_pr_does_not_treat_a_lying_success_result_as_ran():
    # Джобы нет в needs: result GitHub не отдаёт. Если кто-то подставит success,
    # строка PR всё равно говорит «не запускались», а не «success».
    text, code = test_summary.render_summary("pull_request", REQUIRED, "success")
    assert code == 0
    assert (
        "| Integration Tests | не запускались (skipped на PR, только develop/main) |"
        in text
    )
    assert "| Integration Tests | success |" not in text


def test_push_success_may_say_integration_ran():
    text, code = test_summary.render_summary("push", REQUIRED, "success")
    assert code == 0
    assert "| Integration Tests | success |" in text
    assert "включая Integration Tests" in text
    assert "не запускались" not in text


def test_push_skipped_is_not_called_passed():
    text, code = test_summary.render_summary("push", REQUIRED, "skipped")
    assert code == 0
    assert "| Integration Tests | skipped |" in text
    assert "это не «интеграция прошла»" in text
    assert "All tests passed" not in text
    assert "включая Integration Tests" not in text


def test_failed_required_suite_still_fails_and_names_integration():
    suites = [("Unit Tests", "failure"), ("URDF sim-arg", "success")]
    text, code = test_summary.render_summary("pull_request", suites, "")
    assert code == 1
    assert "Tests failed" in text
    assert "Unit Tests" in text
    assert "не запускались (skipped на PR" in text


def test_empty_required_result_is_not_success():
    text, code = test_summary.render_summary("push", [("Unit Tests", "")], "success")
    assert code == 1
    assert "| Unit Tests | unknown |" in text


def test_cli_pr_exit_zero_and_honest_line(capsys):
    code = test_summary.main(
        [
            "--event",
            "pull_request",
            "--suite",
            "Unit Tests=success",
            "--integration-result",
            "skipped",
        ]
    )
    assert code == 0
    out = capsys.readouterr().out
    assert "не запускались (skipped на PR, только develop/main)" in out
    assert "All tests passed" not in out


def test_cli_failure_exit_one(capsys):
    code = test_summary.main(["--event", "push", "--suite", "Unit Tests=failure"])
    assert code == 1
    assert "Tests failed" in capsys.readouterr().out


def test_workflow_wires_script_and_does_not_claim_all_passed():
    workflow = (REPO_ROOT / ".github" / "workflows" / "G-Run Tests.yml").read_text(
        encoding="utf-8"
    )
    assert "scripts/ci/test_summary.py" in workflow
    assert "All tests passed" not in workflow
    # Интеграция по-прежнему не ворота PR: условия джобы не трогаем.
    assert (
        "github.ref == 'refs/heads/develop' || github.ref == 'refs/heads/main'"
        in workflow
    )
    # На push сводка видит реальный result, а не пустую строку (#5116 остаток).
    assert '--integration-result "${{ needs.integration-tests.result }}"' in workflow
    assert "integration-tests" in _test_summary_needs(workflow)


def _test_summary_needs(workflow: str) -> str:
    """Список needs джобы test-summary, не соседних джоб."""
    marker = "  test-summary:\n"
    start = workflow.index(marker)
    window = workflow[start : start + 400]
    for line in window.splitlines():
        stripped = line.strip()
        if stripped.startswith("needs:"):
            return stripped
    raise AssertionError("needs: у test-summary не найден")


def test_push_success_is_not_called_not_run():
    text, code = test_summary.render_summary("push", REQUIRED, "success")
    assert code == 0
    assert "| Integration Tests | success |" in text
    assert "не запускались" not in text


def test_push_failure_is_named_and_does_not_fail_the_run():
    # Интеграция не ворота (решение Шифу): failure красит строку, не код выхода.
    text, code = test_summary.render_summary("push", REQUIRED, "failure")
    assert code == 0
    assert "| Integration Tests | failure |" in text
    assert "не запускались" not in text
    assert "это не «интеграция прошла»" in text


def test_push_empty_result_is_not_called_not_run():
    text, code = test_summary.render_summary("push", REQUIRED, "")
    assert code == 0
    assert "результат не передан" in text
    assert "не запускались" not in text
    assert "| Integration Tests | success |" not in text


def test_pr_skipped_stays_not_run_and_exit_zero():
    text, code = test_summary.render_summary("pull_request", REQUIRED, "skipped")
    assert code == 0
    assert "не запускались (skipped на PR, только develop/main)" in text


@pytest.mark.parametrize(
    "raw",
    ["=success", "no-equals"],
)
def test_bad_suite_arg(raw):
    with pytest.raises(SystemExit):
        test_summary.main(["--event", "push", "--suite", raw])
