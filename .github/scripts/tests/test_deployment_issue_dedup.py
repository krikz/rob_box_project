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


def test_extract_relevant_log_line_ignores_dialogue_error_none_success_echo() -> None:
    """Retro 15.08 t_29230e6f / issue #1335 — also hits issue #1364 (round-129).

    dialogue_node's normal turn completion line is
    `process_input returned: ... error=None` — error=None means NO error.
    The bare \\berror\\b matcher would otherwise flag this INFO line as
    deploy-critical on a fully green round (deploy run 31886490619 SUCCESS +
    E2E SUCCESS, yet issue was created; same pattern observed on round-129
    run 32115362102, issue #1364).
    """
    log_text = (
        "[dialogue_node-4] [INFO] [1787041178.922615857] [dialogue_node]: "
        "✅ [turn] process_input returned: spoken=''[:60] "
        "tools=['set_voice', 'speak_text'] error=None"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line is None


def test_extract_relevant_log_line_ignores_error_colon_none_success_echo() -> None:
    """Same retro as above, alternate formatting `error: None`."""
    log_text = (
        "[dialogue_node-1] [INFO] [1787041178.922615857] [dialogue_node]: "
        "result: error: None"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line is None


def test_extract_relevant_log_line_ignores_minimax_api_error_2054() -> None:
    """Retro 18.08 #1364: minimax bad-request 2054 (voice) at tts_node
    startup — tts chain falls back to next provider, robot keeps speaking.
    Deploy gate must not file a critical_log issue for this single transient.

    Run 32115362102 logs contained:
      [health_monitor-3] [ERROR] tts_node (0s ago): MiniMax bad-request,
      NO retry: minimax API error 2054: voice
    E2E run 32115882100 then reported `✅ TTS: основной голос без
    fallback` — primary voice worked throughout the round.
    """
    log_text = (
        "[health_monitor-3]   [ERROR] tts_node (0s ago): MiniMax bad-request, "
        "NO retry: minimax API error 2054: voice"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="main", severity="critical")

    assert line is None


def test_extract_relevant_log_line_ignores_nav2_tf_to_map_startup_race() -> None:
    """Retro 18.08 #1364: nav2 global_costmap 'TF base_footprint→map'
    at startup, before rtabmap publishes /map. Same startup race as the
    already-excluded 'to odom' line; PR #1151 (commit d1abb2ae) only
    covered 'to odom' but the round-129 run 32115362102 had 'to map'
    as the actual costmap TF timeout, and the deploy detector still
    flagged it on a fully green round.
    """
    log_text = (
        "[INFO] [1787041170.966729633] [global_costmap.global_costmap]: "
        "Timed out waiting for transform from base_footprint to map to "
        "become available, tf error: Could not find a connection between "
        "'map' and 'base_footprint' because they are not part of the same "
        "tree.Tf has two or more unconnected trees."
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


def test_extract_relevant_log_line_ignores_missing_critical_synthdefs_startup_race() -> None:
    """Issue #1520, deploy round-178 / run 32573594346.

    start_voice_assistant.sh invoked validate_music_stack.py 5s after
    launching sclang. sclang needs ~20s to register the OSCdef and
    preload ~38 .scd SynthDefs, so /tmp/sclang.log is still empty at
    the 5s mark. validate_music_stack.py then prints a non-empty
    'Missing critical SynthDefs: <list>' line, which the deploy gate
    would otherwise surface as a critical_log false-positive on an
    otherwise-green run. The fix in start_voice_assistant.sh waits
    for "FoxDot OSCdef registered" (capped at 30s) before validating;
    this exclude is the defence-in-depth for slow runners that still
    fall through.
    """
    log_text = "\n".join(
        [
            "=== voice-assistant CRITICAL ERRORS ===",
            "Missing critical SynthDefs: strangerpulsepad, strangerarp, strangerbrass",
            "",
            "=== voice-assistant WARNINGS ===",
            "[tts_node-5] [WARN] [1787402505.606730380] [tts_node]: "
            "🔇 STOP command received - немедленная остановка TTS",
        ]
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line is None


def test_extract_relevant_log_line_ignores_sclang_log_not_found() -> None:
    """Issue #1520 sibling race: validate_music_stack.py prints
    'Log file not found: /tmp/sclang.log' when sclang hasn't yet
    written its startup log. This is the root-cause echo of the
    same 5s-too-early race, not a real failure. The deploy gate
    must skip it.
    """
    log_text = "\n".join(
        [
            "Music stack degraded",
            "OSCdef ready: no",
            "Missing critical SynthDefs: strangerpulsepad, strangerarp, strangerbrass",
            "Log file not found: /tmp/sclang.log",
        ]
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line is None


def test_extract_relevant_log_line_still_catches_sclang_syntax_error() -> None:
    """Negative test for #1520: a real sclang-side error (e.g. a
    broken .scd) MUST still be reported by the deploy gate. The
    new exclude patterns target only the readiness-echo phrases
    from validate_music_stack.py, not generic sclang faults that
    mention 'critical' / 'fatal' / 'error' in their body.
    """
    log_text = "\n".join(
        [
            "ERROR: syntax error",
            "  in file '/ws/custom_synthdefs/strangerpulsepad.scd'",
            "  line 7 char 2:",
        ]
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line == "ERROR: syntax error"


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


def test_extract_relevant_log_line_ignores_jackd_process_graph_async_master() -> None:
    """Issue #1368: jackd logs a single-line `Process error` whenever a DSP
    cycle overruns the ALSA period (scsynth client without realtime
    scheduling). jackd drops the cycle and recovers automatically — scsynth
    stays up and music/TTS keep flowing. The deploy gate must not flag it.

    The text is case-preserved in the log; the regex is case-insensitive.
    """
    log_text = "\n".join(
        [
            "[SuperCollider] Cleaning up stale JACK SHM files...",
            "[SuperCollider] JACK running. Starting scsynth on UDP port 57110...",
            "[jackd] JackAudioDriver::ProcessGraphAsyncMaster: Process error",
        ]
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")
    assert line is None


def test_extract_relevant_log_line_ignores_audio_node_fell_in_main_scope() -> None:
    """Issue #1368: context_aggregator in the Main Pi perception container
    prints '❌ Нода упала: /audio_node' whenever the Vision Pi voice-assistant
    restarts audio_node (cross-container /rosout leak via the Zenoh router).
    Same shape as the telegram_node exclusion (issue #775). Real root cause
    is on Vision Pi, not a deployment failure.
    """
    log_text = (
        "[context_aggregator-2] [ERROR] [1787042891.609388690] "
        "[context_aggregator]: ❌ Нода упала: /audio_node"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="main", severity="critical")
    assert line is None


def test_extract_relevant_log_line_still_catches_audio_node_in_vision_scope() -> None:
    """Negative test: a real audio_node error in the vision scope MUST still
    be reported. audio_node lives in the voice-assistant container on the
    Vision Pi, so vision scope is the real owner.
    """
    log_text = (
        "[audio_node] [ERROR] [1787042891.609388690] [audio_node]: "
        "Failed to open pyaudio stream: Invalid sample rate"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")
    assert line == log_text


def test_extract_relevant_log_line_still_catches_jackd_unrelated_errors() -> None:
    """Negative test: a real jackd error that is NOT the transient
    ProcessGraphAsyncMaster message MUST still be reported.
    """
    log_text = (
        "[jackd] FATAL could not connect to server: server failed to start"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")
    assert line == log_text


def test_extract_relevant_log_line_ignores_supercollider_synthdef_not_found() -> None:
    """Issue #1485, deploy round-165: FoxDot/Renardo sends `/s_new` for
    SynthDefs (notably `rhpiano`) that ship with renardo but are not
    pre-loaded in the headless scsynth image. The deploy gate must not
    file a critical issue for these benign warnings — the music stack
    stays healthy and TTS/voice flow continues.
    """
    log_text = "\n".join(
        [
            "Buffer UGen: no buffer data",
            "*** ERROR: SynthDef rhpiano not found",
            "FAILURE IN SERVER /s_new SynthDef not found",
            "*** ERROR: SynthDef rhpiano not found",
            "FAILURE IN SERVER /s_new SynthDef not found",
            "SuperCollider 3 server ready.",
        ]
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line is None


def test_extract_relevant_log_line_ignores_rtabmap_scan_voxel_size_transfer() -> None:
    """Issue #1485, deploy round-165: rtabmap icp_odometry prints
    `Transferring value 0.05 of "Icp/VoxelSize" to ros parameter
    "scan_voxel_size" for convenience` when the YAML declares
    scan_voxel_size but Icp/VoxelSize is 0. This is a transparent
    parameter copy, not an SLAM fault — the deploy gate must skip it.
    """
    log_text = (
        "[icp_odometry-1] [WARN] [1787149892.096715766] [rtabmap.icp_odometry]: "
        'IcpOdometry: Transferring value 0.05 of "Icp/VoxelSize" to ros parameter '
        '"scan_voxel_size" for convenience. "Icp/VoxelSize" is set to 0.'
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="main", severity="warning")

    assert line is None


def test_extract_relevant_log_line_ignores_rtabmap_scan_normal_k_transfer() -> None:
    """Issue #1680, deploy round-244: rtabmap icp_odometry prints
    `Transferring value 5 of "Icp/PointToPlaneK" to ros parameter
    "scan_normal_k" for convenience` when the YAML declares scan_normal_k
    but Icp/PointToPlaneK is set to 5. Same root cause as the scan_voxel_size
    pattern (#1485): a transparent parameter copy, not an SLAM fault —
    the deploy gate must skip it.
    """
    log_text = (
        "[icp_odometry-1] [WARN] [1787752712.754846620] [rtabmap.icp_odometry]: "
        'IcpOdometry: Transferring value 5 of "Icp/PointToPlaneK" to ros parameter '
        '"scan_normal_k" for convenience.'
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="main", severity="warning")

    assert line is None


def test_extract_relevant_log_line_ignores_audio_node_hpfonoff_fallback() -> None:
    """Issue #1680, deploy round-244 (ADR-0013 §3.5): voice-assistant's
    audio_node attempts to apply HPFONOFF (high-pass filter) to the UAC1.0
    ReSpeaker. When the device rejects the write_parameter call (busy or
    unsupported on non-XVF-3000 hardware), the node logs a warning and
    falls back to firmware default — audio chain stays healthy. The WARN
    keyword alone must not trigger a deployment-fail issue.
    """
    log_text = (
        "[audio_node-1] [WARN] [1787752638.036746796] [audio_node]:   "
        "HPFONOFF: write_parameter вернул False (устройство занято?). "
        "Используется дефолт firmware."
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="warning")

    assert line is None


def test_extract_relevant_log_line_ignores_respeaker_threshold_warnings() -> None:
    """Issue #1485, deploy round-165 (sibling of issue #989): voice-assistant
    falls back to a software gate when the UAC1.0 ReSpeaker rejects the
    audio_node hw-ctl threshold. The warning itself is the fallback
    notification; real audio_node failures use different phrasing and
    keep their severity.
    """
    vision_log = (
        "[audio_node-1] [WARN] [1787149897.690666359] [audio_node]: "
        "⚠️ [issue 989] ReSpeaker не принял threshold 6.0 dB — программный гейт остаётся"
    )
    main_leak_log = (
        "[health_monitor-3]   [WARN] audio_node (2s ago): "
        "⚠️ [issue 989] ReSpeaker не принял threshold 6.0 dB — програ"
    )

    vision_line = MODULE.extract_relevant_log_line(vision_log, scope="vision", severity="warning")
    main_line = MODULE.extract_relevant_log_line(main_leak_log, scope="main", severity="warning")

    assert vision_line is None
    assert main_line is None


def test_extract_relevant_log_line_still_catches_audio_node_real_fallback() -> None:
    """Negative test for #1485 / #989: an audio_node WARN that is NOT
    the documented ReSpeaker threshold fallback MUST still be reported.
    """
    log_text = (
        "[audio_node-2] [WARN] [1787149901.000000001] [audio_node]: "
        "pyaudio input overflow, 240 frames dropped"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="warning")

    assert line == log_text


def test_extract_relevant_log_line_ignores_supercollider_n_free_stale_node() -> None:
    """Issue #1802, deploy run 33395279992 31.08: scsynth on Vision Pi
    logs `FAILURE IN SERVER /n_free Node <id> not found` when FoxDot's
    cleanup path releases SynthDef nodes that sclang has already freed
    during the headless shutdown sequence. The plain `failure` keyword
    trips CRITICAL_MATCH_RE and was filing a deploy-critical on every
    staging run. The music stack stays healthy (Voice Pi container_status=
    true + 189 ROS2 topics), the node-id race is a benign shutdown
    byproduct, and the real scsynth crash wording (`Exception in Server`)
    is NOT covered by this exclusion — so genuine outages still surface.
    """
    log_text = "\n".join(
        [
            "Buffer UGen: no buffer data",
            "FAILURE IN SERVER /n_free Node 9002 not found",
            "late 0.013253629",
            "FAILURE IN SERVER /n_free Node 9017 not found",
            "SuperCollider 3 server ready.",
        ]
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line is None


def test_extract_relevant_log_line_ignores_voice_assistant_music_stack_validation_echo() -> None:
    """Issue #1802, deploy run 33395279992 31.08: voice-assistant's
    startup wrapper prints `⚠ Music stack validation found non-critical
    errors (degraded but usable)` whenever validate_music_stack.py exits
    non-zero without declaring a fatal sclang crash (see
    docker/vision/scripts/voice_assistant/start_voice_assistant.sh:138
    and src/rob_box_voice/scripts/validate_music_stack.py — returns 1 on
    degraded mode). The container is INTENTIONALLY continuing with a
    degraded music stack; voice/TTS/STT stay healthy. The plain word
    `errors` in the wrapper's own status line trips CRITICAL_MATCH_RE
    and was filing a deploy-critical on every staging run. Same exclusion
    tier as `missing critical synthdefs: none` / `log file not found:
    sclang.log` (retro 15.08 t_a14ac65d, the sclang preload race).
    """
    log_text = "\n".join(
        [
            "✓ Music stack validation passed",
            "  └─ Подробности: /tmp/sclang.log",
            "sclang готов",
            "Проверка music stack readiness...",
            "⚠ Music stack validation found non-critical errors (degraded but usable)",
            "  └─ Подробности: /tmp/sclang.log",
        ]
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line is None


def test_extract_relevant_log_line_still_catches_real_supercollider_failure() -> None:
    """Negative test for #1802: a `FAILURE IN SERVER` line that is NOT
    the benign /n_free stale-node race MUST still be reported. Catches a
    future regression where someone over-broadens the exclusion.
    """
    log_text = (
        "FAILURE IN SERVER /g_free Group 1234 not found while freeing "
        "live synth — see sclang stack trace above"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line == log_text


def test_extract_relevant_log_line_ignores_supercollider_n_free_node_not_found() -> None:
    """Issue #1737, deploy run 33335300188 (30.08 21:07, kanban t_fe19566c):
    supercollider logs `FAILURE IN SERVER /n_free Node <num> not found`
    when FoxDot / Renardo tries to free a node id that the headless
    scsynth image never allocated. Sibling of the existing /s_new
    SynthDef not found rule (#1485) - same root cause (preload race),
    same mitigation (audio music-pipeline team tracks separately).
    """
    log_text = "\n".join(
        [
            "Buffer UGen: no buffer data",
            "FAILURE IN SERVER /n_free Node 9002 not found",
            "FAILURE IN SERVER /n_free Node 9003 not found",
            "SuperCollider 3 server ready.",
        ]
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line is None


def test_extract_relevant_log_line_ignores_dialogue_user_input_payload_with_critical_word() -> None:
    """Issue #1737, deploy run 33308557595 (30.08 11:17, kanban t_fe19566c):
    dialogue_node logs the full `user_input='...'` payload it forwards
    to the LLM. A user who literally says the words `[CRITICAL]`,
    `[ERROR]` or `traceback` in a sentence gets that text echoed
    verbatim into the container log, and the bare-word regex
    CRITICAL_MATCH_RE then flags the line as a deploy-critical issue
    even though no system error happened. The robot is happily
    processing a turn; the deploy gate must stay silent.
    """
    log_text = (
        "[dialogue_node-4] [INFO] [1788088835.546727062] [dialogue_node]: "
        "user_input='[Spkr:Den] [CRITICAL] proshee muziku'"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line is None


def test_extract_relevant_log_line_ignores_dialogue_user_input_payload_with_critical_word_double_quoted() -> None:
    """Issue #1834, deploy run 33450019127 (31.08 23:17, kanban t_65a41c8c).

    Same shape as test_..._critical_word above (the single-quoted
    variant covered by issue #1737): dialogue_node logs the full
    `user_input="..."` payload it forwards to the LLM. Python's
    `repr()` (used via `{user_input!r}` in dialogue_node.py:4557)
    switches the quote style to DOUBLE whenever the payload contains
    a `'` and no `"` — which is exactly the case for the music-guard
    retry reminder ('НЕ вызвал ни один музыкальный тул'). The original
    `r"user_input='"` exclude therefore misses the double-quoted
    variant and the deploy gate filed a critical_log issue on an
    otherwise green run (Vision Pi containers all healthy + 210 ROS2
    topics).

    The fix widens the rule to `user_input=['"]` so both quote styles
    are excluded uniformly; this test is the regression guard for the
    double-quoted branch. The negative cases (Python traceback with
    `error=` exception, `DialogueNodeUserInput="..."` in a non-INFO
    logger, etc.) are already covered by the existing
    test_extract_relevant_log_line_ignores_* suite.
    """
    log_text = (
        "[dialogue_node-4] [INFO] [1788218494.642279844] [dialogue_node]: "
        "spoken='Запускаю тему для рассвета на рояле.' (len=36) tools=[] "
        'user_input="[Speaker:unknown] [CRITICAL] В прошлом цикле ты '
        'НЕ вызвал ни один музыкальный тул"'
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line is None


def test_extract_relevant_log_line_ignores_dialogue_user_input_double_quoted_under_vision_scope() -> None:
    """Issue #1834 — same payload as above but routed through the
    vision-scope pipeline. The deploy gate uses scope="vision" for
    Vision Pi, and the rule lives in CRITICAL_EXCLUDE_COMMON (not
    CRITICAL_EXCLUDE_BY_SCOPE), but we still exercise the vision
    path explicitly to lock in the production routing.
    """
    log_text = (
        "[dialogue_node-4] [INFO] [1788218494.642279844] [dialogue_node]: "
        'user_input="[Speaker:unknown] [CRITICAL] retry reminder"'
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line is None


def test_dialogue_user_input_double_quoted_with_real_crash_still_reports_critical() -> None:
    """Issue #1834 / issue #1737 — make sure widening the `user_input=`
    exclude to cover both quote styles does NOT silence a real
    dialogue_node Python crash. The traceback header + exception
    line below the payload still match CRITICAL_MATCH_RE via
    `traceback` / `Error`, and the deploy gate must keep surfacing
    them.

    Pattern is identical to the negative test
    `test_extract_relevant_log_line_still_catches_dialogue_python_
    exception_after_traceback` (issue #1737), but the user_input
    payload uses DOUBLE quotes to mirror the runtime mismatch this
    PR fixes. If the regex widening ever got too generous (e.g.
    swallowed the whole multi-line block), this test would fail.
    """
    log_text = "\n".join(
        [
            "[dialogue_node-4] [INFO] [...] [dialogue_node]: "
            'user_input="[Speaker:unknown] [CRITICAL] retry reminder"',
            "[dialogue_node-4] Traceback (most recent call last):",
            '  File "/opt/ros/humble/lib/python3.10/site-packages/'
            'rob_box_voice/dialogue_node.py", line 42, in process_input',
            "ERROR: connection refused to upstream zenoh router",
        ]
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    # Concrete Error line still must surface; the bare header alone is
    # silenced (issue #1737), but the traceback + ERROR pair is a real
    # crash signal that the gate must keep reporting.
    assert line is not None
    assert "ERROR" in line


def test_extract_relevant_log_line_ignores_music_stack_validator_self_report() -> None:
    """Issue #1737, deploy runs 33330895761 / 33335300188 (30.08 19:31 /
    21:07, kanban t_fe19566c): start_voice_assistant.sh runs
    validate_music_stack.py right after sclang starts and prints
    `Music stack validation found non-critical errors (degraded but
    usable)` when sclang has not finished its SynthDef preload yet
    (same root cause as the `missing critical synthdefs: ...` race,
    see #1520). The validator itself already downgraded the severity,
    but the literal `critical` and `errors` words in the message
    trigger CRITICAL_MATCH_RE. The deploy gate must not file a
    critical issue every time sclang is still preloading.
    """
    log_text = "Music stack validation found non-critical errors (degraded but usable)"

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line is None


def test_extract_relevant_log_line_ignores_bare_traceback_with_logger_prefix() -> None:
    """Issue #1737, deploy run 33315845764 (30.08 14:08, kanban t_fe19566c):
    the bare "Traceback (most recent call last):" header slips through
    when a ROS2 node logger prepends its tag (e.g.
    `[dialogue_node-4] Traceback (most recent call last):`), because
    the pre-existing #1335 rule anchors on `^traceback ...$`. The
    follow-up Python exception line (BrokenPipeError,
    ModuleNotFoundError, ...) still matches CRITICAL_MATCH_RE via
    `error` and is reported unless another rule excludes it. The
    deploy gate must skip the bare header - alone or with a logger
    prefix - uniformly.
    """
    log_text = "[dialogue_node-4] Traceback (most recent call last):"

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line is None


def test_extract_relevant_log_line_still_catches_dialogue_python_exception_after_traceback() -> None:
    """Negative test for the bare-Traceback exclusion above: a real
    dialogue_node Python crash MUST still surface a deploy-critical
    issue when the exception line carries the actual error word. We
    use an `Exception ignored in:` line (which CRITICAL_MATCH_RE
    matches on the literal `Exception` token, distinct from the
    compound `ModuleNotFoundError` / `BrokenPipeError` cases that do
    not have a word-boundary on `Error`) to make sure the bare-header
    exclusion does not swallow the whole traceback silently.
    """
    log_text = "\n".join(
        [
            "[dialogue_node-4] Traceback (most recent call last):",
            '  File "/opt/ros/humble/lib/python3.10/site-packages/.../dialog_node.py", line 42, in process_input',
            "ERROR: connection refused to upstream zenoh router",
        ]
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line is not None
    assert "ERROR" in line


def test_extract_relevant_log_line_ignores_stt_yandex_error_vosk_ok_success_chain() -> None:
    """Issue #1875, deploy run 33605805375 (02.09 07:53 UTC, kanban
    t_198f9374): voice-assistant logs `[stt_attempt]
    yandex:error(52ms)->yandex:error(551ms)->vosk:ok(1557ms '...') ->
    accepted '...'` whenever Yandex STT returns transient errors but
    Vosk successfully recognizes the phrase and the turn is accepted.
    The `yandex:error` token trips CRITICAL_MATCH_RE on the bare word
    `error` and files a false deploy-critical on otherwise green runs
    (Vision Pi voice-assistant healthy + 193 ROS2 topics, Main Pi
    perception healthy, all container_status checks passing). The
    exclusion matches lines that carry BOTH `[stt_attempt]` and
    `vosk:ok` in the same chain — the operator-facing signal here is
    `accepted`, not `error`.
    """
    log_text = (
        "[INFO] [ros_vision-1]: process started with pid [123]\n"
        "[INFO] [1788335862.073783216] [stt_node]: [stt_attempt] "
        "yandex:error(52ms)->yandex:error(551ms)->vosk:ok(1557ms "
        "'а меня бабушка') -> accepted 'а меня бабушка'\n"
        "[INFO] [voice_node]: pipeline healthy"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line is None


def test_extract_relevant_log_line_ignores_stt_yandex_error_vosk_ok_in_main_scope() -> None:
    """Sibling of the previous test: the exclusion is in
    CRITICAL_EXCLUDE_COMMON, so it must silence the FP regardless of
    scope. health_monitor re-echoes stt_node lines over the shared
    /rosout bus, and the deploy gate used to file a false
    deployment-critical against the perception container for the
    Vision Pi voice-assistant's transient STT fallback. Same pattern
    must NOT match here either.
    """
    log_text = (
        "[dialogue_node-4] dialogue turn completed\n"
        "[INFO] [stt_node]: [stt_attempt] "
        "yandex:error(80ms)->vosk:ok(1200ms 'привет') -> accepted 'привет'"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="main", severity="critical")

    assert line is None


def test_extract_relevant_log_line_still_catches_stt_yandex_error_without_fallback() -> None:
    """Negative test for the yandex:error→vosk:ok exclusion above:
    real STT failures where the fallback chain does NOT recover MUST
    still surface as a deploy-critical issue. `yandex:error(1500ms) -
    no stt provider succeeded` carries no `vosk:ok` token in the line,
    so the exclusion does not match and CRITICAL_MATCH_RE flags the
    `error` word as before.
    """
    log_text = (
        "[ERROR] [stt_node]: [stt_attempt] yandex:error(1500ms) - no stt provider succeeded\n"
        "[INFO] [voice_node]: pipeline degraded"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line is not None
    assert "no stt provider succeeded" in line


def test_extract_relevant_log_line_still_catches_stt_attempt_all_providers_error_rejected() -> None:
    """Negative test for the yandex:error→vosk:ok exclusion: when the
    STT attempt chain fails completely (`yandex:error->vosk:error->
    rejected`), there is no successful `vosk:ok` token and the
    exclusion does not match. CRITICAL_MATCH_RE still flags `error`
    via the yandex:error and vosk:error tokens, so the operator sees
    the deploy-critical issue. Without this negative test the
    exclusion could silently regress to swallow ALL stt_attempt
    failures.
    """
    log_text = (
        "[ERROR] [stt_node]: [stt_attempt] "
        "yandex:error(100ms)->vosk:error(200ms) -> rejected\n"
        "[WARN] [voice_node]: user turn aborted"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line is not None
    assert "rejected" in line


def test_extract_relevant_log_line_still_catches_stt_yandex_ok_without_error_token() -> None:
    """Negative test for the yandex:error→vosk:ok exclusion: a
    successful single-provider STT chain (`yandex:ok(100ms) -> accepted
    'привет'`) carries no `yandex:error` token, so the exclusion does
    not match. CRITICAL_MATCH_RE does not fire on the `ok` token
    either, so the line returns None on its own. The important point
    is that the exclusion does NOT cause this benign line to surface
    as a deploy-critical — we assert None explicitly to lock the
    behaviour.
    """
    log_text = (
        "[INFO] [stt_node]: [stt_attempt] yandex:ok(100ms) -> accepted 'привет'"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line is None


def test_extract_relevant_log_line_still_catches_stt_fatal_microphone_error() -> None:
    """Negative test for the yandex:error→vosk:ok exclusion: a hard
    STT failure (`FATAL: microphone device not found`) carries no
    `[stt_attempt]` envelope at all, so the exclusion cannot apply.
    CRITICAL_MATCH_RE matches `fatal` and the operator sees the
    deploy-critical issue. This guards against the exclusion being
    open-ended enough to swallow real `FATAL` lines that happen to
    mention stt_node in their logger prefix.
    """
    log_text = (
        "[ERROR] [stt_node]: FATAL: microphone device not found - aborting\n"
        "[INFO] [voice_node]: pipeline aborted"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line is not None
    assert "FATAL" in line


def test_extract_relevant_log_line_ignores_dialogue_babble_retry_reminder() -> None:
    """Issue #1877 / deploy run 33609815109 (02.09 08:42, kanban
    t_c9c7238c): dialogue_node's babble guard logs `[issue 992 Bug
    D] LLM babble detected — retrying once with CRITICAL reminder
    (head=...)` at WARN level when the metalanguage detector trips
    (dialogue_node.py:3564). The literal `CRITICAL reminder` string
    is the name of the injected retry prompt — NOT a system failure.
    The deploy gate was filing a false `critical_log` finding on an
    otherwise green run (Vision Pi all containers healthy + 189
    topics, Main Pi perception healthy). The exclusion must silence
    both the CRITICAL and WARNING passes (the line carries `[WARN]`
    too) so a single babble recovery does not double-count as a
    `warning_log` finding on the same healthy turn.
    """
    log_text = (
        "[dialogue_node-4] [WARN] [1788338459.426762332] [dialogue_node]: "
        "[issue 992 Bug D] LLM babble detected \u2014 retrying once with "
        "CRITICAL reminder "
        "(head='\u042e\u0437\u0435\u0440 (\u043e\u043f\u0435\u0440\u0430\u0442\u043e\u0440) \u043f\u0440\u043e\u0441\u0438\u0442 \u0434\u0435\u0440\u043d\u0443\u0442\u044c"
        " gensearchlibrary \u0431\u0435\u0437 \u043f\u0430\u0440\u0430\u043c\u0435\u0442\u0440')"
    )

    critical = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")
    warning = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="warning")

    assert critical is None
    assert warning is None


def test_extract_relevant_log_line_ignores_dialogue_tool_retry_reminder() -> None:
    """Issue #1877, sibling of the babble retry exclusion: the
    tool-skipped guard in dialogue_node.py:4023 logs `[issue 1777 /
    1762] LLM skip non-music tool <name> \u2014 retrying once with
    CRITICAL reminder (user=...)` at WARN level. Same shape as the
    babble exclusion (literal `CRITICAL reminder`, intentional
    one-shot retry notification, not a system failure). The exclusion
    must silence both the CRITICAL and WARNING passes uniformly.
    """
    log_text = (
        "[dialogue_node-4] [WARN] [1788338461.123456] [dialogue_node]: "
        "[issue 1777 / 1762] LLM skip non-music tool 'gensearchlibrary' "
        "\u2014 retrying once with CRITICAL reminder "
        "(user='\u042e\u0437\u0435\u0440 \u043f\u0440\u043e\u0441\u0438\u0442')"
    )

    critical = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")
    warning = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="warning")

    assert critical is None
    assert warning is None


def test_extract_relevant_log_line_still_catches_dialogue_value_error_after_babble() -> None:
    """Negative test for the babble/tool-retry exclusion. If a real
    dialogue_node Python exception fires in the same turn as a babble
    recovery, the exception line must still surface — only the
    babble notification itself is silenced. Without this guard the
    exclusion would swallow real crashes that happen to follow a
    babble event in the log dump.
    """
    log_text = "\n".join(
        [
            "[dialogue_node-4] [WARN] [1788338459.426762332] [dialogue_node]: "
            "[issue 992 Bug D] LLM babble detected \u2014 retrying once with "
            "CRITICAL reminder (head='hello')",
            "[dialogue_node-4] ERROR ValueError: real downstream problem after babble",
        ]
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="critical")

    assert line is not None
    assert "ValueError" in line


def test_extract_relevant_log_line_ignores_tts_node_stop_command_warning() -> None:
    """Issue #1877 / deploy run 33609815109: tts_node's
    `_handle_stop_command()` (tts_node.py:1529) logs `[WARN] STOP
    command received - \u043d\u0435\u043c\u0435\u0434\u043b\u0435\u043d\u043d\u0430\u044f
    \u043e\u0441\u0442\u0430\u043d\u043e\u0432\u043a\u0430 TTS` every time the
    user/operator issues a stop. The WARN severity is correct (it's
    a state change) but unrelated to deployment health. The deploy
    detector must not file a warning_log finding when the operator
    happened to interrupt TTS during the deploy window.
    """
    log_text = (
        "[tts_node-5] [WARN] [1788338456.811434096] [tts_node]: "
        "STOP command received - \u043d\u0435\u043c\u0435\u0434\u043b\u0435\u043d\u043d\u0430\u044f "
        "\u043e\u0441\u0442\u0430\u043d\u043e\u0432\u043a\u0430 TTS"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="warning")

    assert line is None


def test_extract_relevant_log_line_still_catches_other_tts_node_warnings() -> None:
    """Negative test for the STOP-command rule above. A genuinely
    concerning tts_node WARN that's NOT the STOP-receipt echo must
    still surface \u2014 e.g. an audio backend fatal or a synthesiser
    crash warning. The exclusion is anchored on the literal
    `STOP command received` phrase, so any other WARN from
    tts_node keeps its severity.
    """
    log_text = (
        "[tts_node-5] [WARN] [1234.567] [tts_node]: "
        "synthesiser backend fatal: cannot open alsa device hw:0,0"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="warning")

    assert line is not None
    assert "synthesiser backend fatal" in line


def test_extract_relevant_log_line_ignores_quest_node_pin_warning() -> None:
    """Issue #1877 / deploy run 33609815109: rob_box_quest's
    quest_node.py:520 logs `[WARN] Quest PIN: <num> (show this to
    operator \u2014 required to start a session)` once at startup when
    `log_pin` is set. The line is informational (telling the operator
    which PIN unlocks the current session) but the node uses
    `warning()` severity so the deploy detector picks it up. The
    exclusion must silence the literal `Quest PIN: <digits> (show
    this to operator` signature; any other quest_node WARN keeps its
    severity.
    """
    log_text = (
        "[WARN] [1788338402.304899095] [quest_node]: "
        "Quest PIN: 103856 (show this to operator \u2014 required to start a session)"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="warning")

    assert line is None


def test_extract_relevant_log_line_still_catches_other_quest_node_warnings() -> None:
    """Negative test for the Quest-PIN rule above. A genuinely
    concerning quest_node WARN that is NOT the PIN-echo startup line
    must still surface \u2014 e.g. an actual game-flow problem. The
    exclusion is anchored on `Quest PIN: <digits> (show this to
    operator`, so any other WARN from quest_node keeps its severity
    and the operator still sees deploy issues originating from the
    quest subsystem.
    """
    log_text = (
        "[quest_node] [WARN] [1234.567]: "
        "player stuck in invalid state, score=42 not advancing"
    )

    line = MODULE.extract_relevant_log_line(log_text, scope="vision", severity="warning")

    assert line is not None
    assert "player stuck" in line
