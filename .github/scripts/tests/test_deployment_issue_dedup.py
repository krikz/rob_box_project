from __future__ import annotations

import importlib.util
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "deployment_issue_dedup.py"
SPEC = importlib.util.spec_from_file_location("deployment_issue_dedup", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_normalize_pattern_strips_run_specific_noise() -> None:
    raw = (
        "2026-03-09T00:18:45Z ERROR Failed to connect to 10.1.1.21:7447 during run 22833178805 "
        "see https://github.com/krikz/rob_box_project/actions/runs/22833178805"
    )

    normalized = MODULE.normalize_pattern(raw)

    assert normalized == "error failed to connect to <ip>:<num> during run <num> see <url>"


def test_build_signature_is_stable_for_same_problem() -> None:
    first = {
        "environment": "production",
        "scope": "vision",
        "container": "oak-d",
        "kind": "critical_log",
        "raw_text": "2026-03-09T00:18:45Z ERROR Failed to connect to 10.1.1.21:7447 during run 22833178805",
    }
    second = {
        "environment": "production",
        "scope": "vision",
        "container": "oak-d",
        "kind": "critical_log",
        "raw_text": "2026-03-10T12:01:07Z ERROR Failed to connect to 10.1.1.22:7447 during run 22899999999",
    }

    assert MODULE.build_signature(first) == MODULE.build_signature(second)


def test_prepare_issue_candidates_deduplicates_same_problem_within_run() -> None:
    findings = [
        {
            "environment": "production",
            "scope": "vision",
            "container": "oak-d",
            "kind": "critical_log",
            "severity": "critical",
            "summary": "Critical log line detected",
            "raw_text": "2026-03-09T00:18:45Z ERROR Failed to connect to 10.1.1.21:7447 during run 22833178805",
        },
        {
            "environment": "production",
            "scope": "vision",
            "container": "oak-d",
            "kind": "critical_log",
            "severity": "critical",
            "summary": "Critical log line detected",
            "raw_text": "2026-03-10T12:01:07Z ERROR Failed to connect to 10.1.1.22:7447 during run 22899999999",
        },
    ]

    candidates = MODULE.prepare_issue_candidates(
        findings=findings,
        branch="feature/docker-build-optimization",
        workflow_run_url="https://github.com/krikz/rob_box_project/actions/runs/22833178805",
        timestamp="2026-03-09 00:20:00 UTC",
        vision_pi_ip="10.1.1.21",
        main_pi_ip="10.1.1.20",
    )

    assert len(candidates) == 1
    assert candidates[0]["duplicate_count"] == 2
    assert "deploy-signature" in candidates[0]["body"]
    assert "Occurrences in this run: 2" in candidates[0]["body"]


def test_filter_existing_issues_skips_matching_signature() -> None:
    findings = [
        {
            "environment": "production",
            "scope": "main",
            "container": "rtabmap",
            "kind": "container_status",
            "severity": "critical",
            "summary": "Container is not running",
            "raw_text": "container rtabmap exited with state exited",
        }
    ]

    candidates = MODULE.prepare_issue_candidates(
        findings=findings,
        branch="main",
        workflow_run_url="https://github.com/krikz/rob_box_project/actions/runs/123",
        timestamp="2026-03-09 00:20:00 UTC",
        vision_pi_ip="10.1.1.21",
        main_pi_ip="10.1.1.20",
    )

    existing_issues = [
        {
            "number": 42,
            "title": candidates[0]["title"],
            "body": "Known failure\n" + candidates[0]["signature_marker"],
        }
    ]

    new_candidates = MODULE.filter_new_candidates(candidates, existing_issues)

    assert new_candidates == []
