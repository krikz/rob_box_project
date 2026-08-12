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


def test_extract_relevant_log_line_ignores_without_error_message() -> None:
    log_text = (
        "[lslidar_driver_node-1] [INFO] [1773019653.918945289] "
        "[lslidar_driver_node]: Initialised lslidar without error"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="main", severity="critical")

    assert line is None


def test_extract_relevant_log_line_ignores_total_errors_summary() -> None:
    log_text = "[health_monitor-1] Total Errors: 1 (последние 0 за минуту)"

    line = MODULE.extract_relevant_log_line(log_text, scope="main", severity="critical")

    assert line is None


def test_extract_relevant_log_line_ignores_optional_serial_warning() -> None:
    log_text = "⚠️  WARNING: Serial port /dev/ttyUSB0 not found!"

    line = MODULE.extract_relevant_log_line(log_text, scope="main", severity="warning")

    assert line is None


def test_extract_relevant_log_line_matches_real_critical_error() -> None:
    log_text = (
        "[stt_node-6] ERROR (VoskAPI:Model():model.cc:122) Folder "
        "'/models/vosk-model-small-ru-0.22' does not contain model files."
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line == log_text


def test_extract_relevant_log_line_ignores_optional_serial_critical_error() -> None:
    log_text = "❌ ERROR: Serial port /dev/ttyUSB0 still not available after 30s"

    line = MODULE.extract_relevant_log_line(log_text, scope="main", severity="critical")

    assert line is None


def test_extract_relevant_log_line_ignores_nav2_startup_tf_timeout() -> None:
    for frame in ("base_link", "base_footprint"):
        log_text = (
            f'[INFO] [1773045972.826191868] [local_costmap.local_costmap]: Timed out waiting '
            f'for transform from {frame} to odom to become available, tf error: '
            f'Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist'
        )

        line = MODULE.extract_relevant_log_line(log_text, scope="main", severity="critical")

        assert line is None


def test_extract_relevant_log_line_ignores_rtabmap_transport_noise() -> None:
    log_text = (
        "[republish-2] [ERROR] [1773045966.706932963] [rtabmap.republish_depth]: "
        "SubscriberPlugin::subscribeImpl with five arguments has not been overridden"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="main", severity="critical")

    assert line is None


def test_extract_relevant_log_line_ignores_known_main_warnings() -> None:
    vision_stub_log = (
        "[vision_stub_node-5] [WARN] [1773045973.085219650] [vision_stub]: ⚠️  "
        "Это ЗАГЛУШКА! Используйте AI HAT + YOLO для реальной обработки"
    )
    deprecated_log = "[WARN] [1773045964.662232589] []: Old-style arguments are deprecated; see --help for new-style arguments"

    vision_stub_line = MODULE.extract_relevant_log_line(vision_stub_log, scope="main", severity="warning")
    deprecated_line = MODULE.extract_relevant_log_line(deprecated_log, scope="main", severity="warning")

    assert vision_stub_line is None
    assert deprecated_line is None


def test_extract_relevant_log_line_prefers_real_supercollider_error() -> None:
    log_text = "\n".join(
        [
            "Zeroconf: failed to create client: Daemon not running",
            "FAILURE IN SERVER /g_new negative node IDs are reserved",
        ]
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line == "FAILURE IN SERVER /g_new negative node IDs are reserved"


def test_extract_relevant_log_line_ignores_sound_sample_named_error() -> None:
    log_text = "[sound_node-7] [INFO] [1773045895.071714241] [sound_node]:   ✓ error: 769ms, 48000Hz"

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line is None


def test_extract_relevant_log_line_ignores_workflow_section_headers() -> None:
    critical_log = "=== supercollider CRITICAL ERRORS ===\nFAILURE IN SERVER /g_new negative node IDs are reserved"
    warning_log = (
        "=== voice-assistant WARNINGS ===\n"
        "[mcp_server-9] [WARN] [1773045897.770643160] [mcp_server]: [speak_text] "
        "⚠️ Speech b6db84fd... не найден в pending_speeches (возможно уже удалён)"
    )

    critical_line = MODULE.extract_relevant_log_line(critical_log, scope="vision", severity="critical")
    warning_line = MODULE.extract_relevant_log_line(warning_log, scope="vision", severity="warning")

    assert critical_line == "FAILURE IN SERVER /g_new negative node IDs are reserved"
    assert warning_line is None


def test_extract_relevant_log_line_ignores_zenoh_clock_skew_timestamp() -> None:
    """zenoh router rejects future timestamps but replaces them — not an outage."""
    log_text = (
        "2026-08-07T14:45:52.034584Z ERROR rx-0 ThreadId(07) "
        "zenoh::net::routing::dispatcher::pubsub: Error treating timestamp for received Data "
        "(incoming timestamp from cb0a8201d7c4925928c54a14272b351e exceeding delta 500ms is rejected: "
        "2026-08-07T16:16:53.876626388Z vs. now: 2026-08-07T14:45:52.034576841Z). "
        "Replace timestamp: Some(7671301010917843184/86da0b2a0fb4bab4f4614d12676e1bdb)"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="main", severity="critical")

    assert line is None
