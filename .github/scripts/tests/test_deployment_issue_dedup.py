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
    base_link_log = (
        '[INFO] [1773045972.826191868] [local_costmap.local_costmap]: Timed out waiting '
        'for transform from base_link to odom to become available, tf error: '
        'Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist'
    )
    # robot_base_frame was changed base_link -> base_footprint (afbb8793);
    # the benign startup TF timeout now names base_footprint (issue #774).
    base_footprint_log = (
        '[INFO] [1776096003.466816576] [local_costmap.local_costmap]: Timed out waiting '
        'for transform from base_footprint to odom to become available, tf error: '
        'Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist'
    )

    line = MODULE.extract_relevant_log_line(base_link_log, scope="main", severity="critical")
    assert line is None

    line = MODULE.extract_relevant_log_line(base_footprint_log, scope="main", severity="critical")
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


def test_extract_relevant_log_line_ignores_supercollider_g_new_startup_noise() -> None:
    """Issue #778/#672/#840: Renardo sends /g_new before scsynth finished
    allocating group IDs. Music stack comes up healthy afterwards, so the
    deploy gate must not file a critical issue for this startup race.
    """
    log_text = "\n".join(
        [
            "Zeroconf: failed to create client: Daemon not running",
            "FAILURE IN SERVER /g_new negative node IDs are reserved",
        ]
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line is None


def test_extract_relevant_log_line_ignores_minimax_2056_quota() -> None:
    """Issue #1193: MiniMax Token Plan exhausted (error 2056) is an external
    billing limit; TTS falls back to Yandex. Not a deployment failure.
    """
    log_text = (
        "[tts_node-5] [ERROR] [1786776215.499241163] [tts_node]: "
        "MiniMax auth error, NO retry: minimax API error 2056: "
        "Token Plan usage limit reached: Upgrade your Token Plan or "
        "purchase Credits for more usage."
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line is None


def test_extract_relevant_log_line_ignores_python_exception_ignored() -> None:
    """nav2 shutdown noise: interpreter closes stdout while a pipe reader
    (`ros2 topic list | head`) already hung up — benign (round-117).
    """
    log_text = (
        "Exception ignored in: <_io.TextIOWrapper name='<stdout>' mode='w' "
        "encoding='utf-8'>"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="main", severity="critical")

    assert line is None


def test_extract_relevant_log_line_ignores_zenoh_responsefinal_warning() -> None:
    """Stale zenoh reply to an already-timed-out query — benign (round-117)."""
    log_text = (
        "2026-08-15T06:42:47.152056Z  WARN rx-0 ThreadId(08) "
        "zenoh::api::session: Received ResponseFinal for unknown Request: 0"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="warning")

    assert line is None


def test_extract_relevant_log_line_ignores_telegram_echo_drop_warning() -> None:
    """telegram_node boot: bot up but no chat bound yet — normal (round-117)."""
    log_text = (
        "[WARN] [1786776214.623560688] [telegram_node]: Dropping dialogue "
        "echo, bot not ready / no active chat (app=True, loop=True, "
        "chat_id=None, text_len=32)"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="warning")

    assert line is None


def test_extract_relevant_log_line_ignores_tts_none_voice_warning() -> None:
    """Issue #1219: voice 'None' → default voice fallback — the chain works."""
    log_text = (
        "[tts_node-5] [WARN] [1786776214.627448550] [tts_node]: "
        "⚠️ [issue 1219] Голос 'None' недоступен у MiniMax — "
        "использую дефолтный 'male-qn-qingse'"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="warning")

    assert line is None


def test_extract_relevant_log_line_ignores_rtabmap_ini_autocreate_warning() -> None:
    """rtabmap ini auto-created on shutdown — benign (round-117)."""
    log_text = (
        '[rtabmap-2] [ WARN] (2026-08-15 06:44:25.435) Parameters.cpp:1325::'
        'readINIImpl() Section "Core" in /config/rtabmap/rtabmap.ini doesn\'t '
        'exist... Ignore this warning if the ini file does not exist yet. The '
        "ini file will be automatically created when rtabmap will close."
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="main", severity="warning")

    assert line is None


def test_extract_relevant_log_line_ignores_perception_uart_warning() -> None:
    """Optional UART IMU not attached — expected in the lab rig (round-117)."""
    log_text = (
        "[perception_bridge-1] [WARN] [1786776266.903651194] "
        "[perception_bridge]: Sensor UART /dev/ttyAMA0 not available; "
        "reads will no-op until hardware is attached."
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="main", severity="warning")

    assert line is None


def test_extract_relevant_log_line_ignores_nav2_scouted_peer_warning() -> None:
    """zenoh startup handshake: peer scouted but not yet connectable — noise."""
    log_text = (
        "2026-08-15T06:44:31.378308Z  WARN net-0 ThreadId(03) "
        "zenoh::net::runtime::orchestrator: Unable to connect to any locator "
        "of scouted peer b4faae00ce67d5ff3e7ae0a317e27441: [tcp/[::1]:40231]"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="main", severity="warning")

    assert line is None


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

    # Both are noise: /g_new startup race (issue #778) and pending_speeches
    # race are benign — neither should surface as a deploy issue.
    assert critical_line is None
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


def test_extract_relevant_log_line_ignores_stt_empty_rejection_vision() -> None:
    """Issue #989: stt_node WARN на пустое — защита от эхо-петли, не деплой-сбой.

    Реальный лог из run 22857794907 (deploy-signature ...:0ef89820b166).
    Для vision scope должен игнорироваться, для main — это всё равно не main-контейнер.
    """
    log_text = (
        "[stt_node-6] [WARN] [1773066077.600410682] [stt_node]: ❌ ОТКЛОНЕНО (пустое)"
    )

    vision_line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="warning")
    main_line = MODULE.extract_relevant_log_line(log_text, scope="main", severity="warning")

    assert vision_line is None
    assert main_line is None


def test_extract_relevant_log_line_ignores_stt_short_rejection_vision() -> None:
    """Issue #989: stt_node WARN на короткое (<min_text_chars) тоже ложный.

    Vosk/Yandex может вернуть «не»/«ага» — это реальная речь, но не команда.
    Деплой-скрипт не должен открывать issue только из-за этого.
    """
    log_text = (
        "[stt_node-6] [WARN] [1773066077.600410682] [stt_node]: "
        "❌ ОТКЛОНЕНО (короткое, <3 chars): \"ага\""
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="warning")

    assert line is None


def test_extract_relevant_log_line_ignores_missing_critical_synthdefs_none() -> None:
    """Retro 15.08 t_a14ac65d: voice-assistant readiness line.

    'Missing critical SynthDefs: none' means ALL critical SynthDefs are
    present — the word 'critical' alone must not file a deployment issue.
    """
    log_text = "Missing critical SynthDefs: none"

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line is None


def test_extract_relevant_log_line_ignores_nav2_broken_pipe_from_topic_list_head() -> None:
    """Retro 15.08 t_a14ac65d: BrokenPipeError from `ros2 topic list | head`.

    start_nav2_direct.sh greps for /odom via a pipe; head closes the pipe
    after the first line and ros2cli prints a traceback. Benign — the topic
    was found right after (✓ /odom topic found).
    """
    log_text = "\n".join(
        [
            "Traceback (most recent call last):",
            '  File "/opt/ros/humble/lib/python3.10/site-packages/ros2topic/verb/list.py", line 79, in main',
            "    print(msg.format_map(locals()))",
            "BrokenPipeError: [Errno 32] Broken pipe",
        ]
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="main", severity="critical")

    assert line is None


def test_round119_all_findings_are_excluded() -> None:
    """Regression: round-119 deploy (run 31883435418) filed 3 criticals + 4
    warnings that are ALL known non-failures (same set as round-117, PR #1295).

    The e2e branch z-{e2e}/test-round-119 was built from develop BEFORE
    PR #1295 merged, so the deploy-verify step used the OLD dedup script and
    re-filed the same false positives. These tests pin the EXACT evidence
    lines from that run so the exclusion rules cannot regress.
    """
    cases = [
        # supercollider (vision, critical): Renardo/FoxDot startup race
        (
            "FAILURE IN SERVER /g_new negative node IDs are reserved",
            "vision",
            "critical",
        ),
        # voice-assistant (vision, critical): external MiniMax billing quota
        (
            "[tts_node-5] [ERROR] [1786795326.912398382] [tts_node]: "
            "MiniMax auth error, NO retry: minimax API error 2056: "
            "Token Plan usage limit reached: Upgrade your Token Plan or "
            "purchase Credits for more usage.",
            "vision",
            "critical",
        ),
        # nav2 (main, critical): Python shutdown noise from `ros2 topic list | head`
        (
            "Exception ignored in: <_io.TextIOWrapper name='<stdout>' "
            "mode='w' encoding='utf-8'>",
            "main",
            "critical",
        ),
        # ceiling-camera (vision, warning): zenoh stale reply
        (
            "2026-08-15T12:01:27.760308Z  WARN rx-1 ThreadId(09) "
            "zenoh::api::session: Received ResponseFinal for unknown Request: 0",
            "vision",
            "warning",
        ),
        # telegram-bot (vision, warning): bot up, no chat bound yet
        (
            "[WARN] [1786795325.936975769] [telegram_node]: "
            "Dropping dialogue echo, bot not ready / no active chat "
            "(app=True, loop=True, chat_id=None, text_len=32)",
            "vision",
            "warning",
        ),
        # voice-assistant (vision, warning): TTS voice fallback (issue #1219)
        (
            "[tts_node-5] [WARN] [1786795325.959573294] [tts_node]: "
            "⚠️ [issue 1219] Голос 'None' недоступен у MiniMax — "
            "использую дефолтный 'male-qn-qingse'",
            "vision",
            "warning",
        ),
        # perception (main, warning): optional UART hardware absent
        (
            "[perception_bridge-1] [WARN] [1786795388.862663900] "
            "[perception_bridge]: Sensor UART /dev/ttyAMA0 not available; "
            "reads will no-op until hardware is attached.",
            "main",
            "warning",
        ),
    ]

    for log_text, scope, severity in cases:
        line = MODULE.extract_relevant_log_line(log_text, scope=scope, severity=severity)
        assert line is None, f"{scope}/{severity} should be excluded: {log_text!r}"
