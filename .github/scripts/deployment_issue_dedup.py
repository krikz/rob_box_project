from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path
from typing import Iterable, Mapping

TIMESTAMP_RE = re.compile(r"\b\d{4}-\d{2}-\d{2}[tT]\d{2}:\d{2}:\d{2}(?:\.\d+)?[zZ]\b")
URL_RE = re.compile(r"https?://\S+")
IP_RE = re.compile(r"\b(?:\d{1,3}\.){3}\d{1,3}\b")
NUMBER_RE = re.compile(r"\b\d+\b")
WHITESPACE_RE = re.compile(r"\s+")
SIGNATURE_MARKER_RE = re.compile(r"<!--\s*deploy-signature:\s*(.*?)\s*-->")

CRITICAL_MATCH_RE = re.compile(
    r"\b(critical|fatal|error|exception|traceback|failure)\b|failed to|segmentation fault|core dumped",
    re.IGNORECASE,
)
WARNING_MATCH_RE = re.compile(r"\b(warn|warning)\b", re.IGNORECASE)

CRITICAL_EXCLUDE_COMMON = [
    r"without error",
    r"✓ error:",
    r"^=== .*critical errors ===$",
    r"scouting delay elapsed",
    r"could not inspect container",
    r"could not fetch logs",
    r"no such container",
    r"error sending batch.*loki.*push",
    r"post.*loki.*push",
    r"context deadline exceeded",
    r"dial tcp.*3100.*no route to host",
    r"undeclare unknown subscriber",
    r"undeclare unknown queryable",
    r"zeroconf: failed to create client: daemon not running",
    # Voice-assistant readiness line: "Missing critical SynthDefs: none"
    # means ALL critical SynthDefs are present — the word "critical" alone
    # must not trigger a deployment-critical issue (retro 15.08 t_a14ac65d).
    r"missing critical synthdefs: none",
    # Voice-assistant startup race (issue #1520, deploy round-178 /
    # run 32573594346): validate_music_stack.py is invoked 5s after
    # sclang starts, but sclang needs ~20s to (a) wait for scsynth
    # alive-thread, (b) register the OSCdef, (c) sequentially preload
    # ~38 .scd SynthDefs (Renardo's startupSynths + customSynths). At
    # the 5s mark `/tmp/sclang.log` is still empty (sclang hasn't
    # written its first "sclang started" line yet), so load_sclang_health
    # reports "Log file not found" and ALL critical SynthDefs as missing
    # — even though the music stack comes up healthy right after. Real
    # health shows up at the next deploy run once sclang's preload loop
    # has flushed. TTS/STT/voice flow is unaffected (sclang is on a
    # separate code path from tts_node / stt_node). The race is closed
    # in start_voice_assistant.sh (wait for OSCdef registration, capped
    # at 30s), this exclude is the defence-in-depth for slow runners.
    # The previous rule `missing critical synthdefs: none` covered the
    # healthy variant; the lookahead `(?!none\b)` keeps the
    # healthy-positive case caught by that earlier rule and excludes
    # every non-empty missing-list echo.
    r"missing critical synthdefs: (?!none\b)",
    # Same race (issue #1520): validate_music_stack.py prints
    # `Log file not found: <path>` when sclang hasn't yet written
    # /tmp/sclang.log. The bash harness's own "Music stack validation
    # found non-critical errors" line is the upstream signal — the
    # validator's "Log file not found" is the root cause echo, not a
    # new failure mode. Excluded so the deploy gate only sees the
    # post-sclang-warning content, which is what the operator
    # actually needs to triage.
    r"log file not found: .*sclang\.log",
    # BrokenPipeError from `ros2 topic list | head` in start_nav2_direct.sh:
    # head closes the pipe after the first line, ros2cli prints a traceback —
    # benign, the topic was found (retro 15.08 t_a14ac65d).
    r"brokenpipeerror: \[errno 32\] broken pipe",
    # The bare traceback header carries no signal by itself; the exception
    # line that follows (e.g. "BrokenPipeError:", "ModuleNotFoundError:")
    # still matches CRITICAL_MATCH_RE via "error" and is reported unless it
    # is explicitly excluded above. This lets us exclude pipe-noise without
    # hiding real Python crash tracebacks.
    r"^traceback \(most recent call last\):$",
    # Clock-skew noise: zenoh router replaces the offending timestamp and
    # forwards the message — data is not lost, so this is not an outage.
    # Root cause is NTP desync between Pis (see scripts/maintenance/sync_time.sh).
    r"error treating timestamp for received data",
    # ALSA/jackd noise: Invalid CTL on dmix_respeaker is a benign control
    # probe failure, not an audio outage (retro 12.08 t_d3e44336).
    r"alsa lib control\.c.*invalid ctl",
    r"dmix_respeaker",
    # JACK RT-thread transient error (issue #1368): jackd logs
    # "JackAudioDriver::ProcessGraphAsyncMaster: Process error" whenever a
    # single DSP cycle overruns the period (typically the scsynth client
    # under ALSA dmix without realtime scheduling, see
    # docker/vision/scripts/supercollider/start_supercollider.sh). jackd
    # drops the cycle and recovers automatically — scsynth stays up and
    # music/TTS keep flowing. Deploy gate must not flag this as critical.
    r"jackaudiodriver::processgraphasyncmaster: process error",
    # SuperCollider startup noise (issue #778, #672, #840): Renardo/FoxDot
    # sends /g_new before scsynth finished allocating group IDs. The music
    # stack comes up healthy afterwards (voice-assistant reports
    # "Music stack healthy" / "sclang готов" / "Missing critical SynthDefs:
    # none"), so this is a benign startup race, not an audio outage.
    #
    # NOTE (issue #1363): since scsynth is now started with `-l 32` (matches
    # sclang's default of 32 max logins), the negative-node-ID mismatch no
    # longer occurs. The exclusion below remains as a defensive measure for
    # legacy deployments / manual scsynth invocations where the maxLogins
    # default of 64 still leaks negative IDs into the log. Once every
    # deployment uses `-l 32` we can drop the exclusion.
    r"failure in server /g_new negative node ids are reserved",
    # External MiniMax quota/auth (issue #1193): error 2056 "Token Plan usage
    # limit reached" is a billing limit, not a code bug. TTS chain falls back
    # to Yandex/Silero, so the robot keeps speaking. e2e-process already
    # treats 2056 as infra-fail — the deploy gate must not file a critical
    # issue for it.
    r"minimax api error 2056",
    r"token plan usage limit",
    # Error 2054 — MiniMax bad-request "voice" (no detailed status_msg, but the
    # pattern matches 2042/20132 family: voice_id invalid). tts_node falls back
    # to the next provider, robot keeps speaking. Deploy gate seen at it as a
    # critical_log false-positive on the fully-green round-129 run 32115362102
    # (issue #1364, retro 18.08): one transient 4xx at startup, e2e passed
    # with the primary voice (`✅ TTS: основной голос без fallback`). Add the
    # same exclusion pattern so the deploy detector does not open noise issues
    # for this transient. Real voice-config bugs (wrong voice_id, missing
    # voice in registry) WILL still be caught by the mcp_server validation
    # chain + e2e voice check.
    r"minimax api error 2054",
    # Python shutdown noise (nav2): "_io.TextIOWrapper ... Exception ignored
    # in:" is printed when the interpreter closes stdout while a pipe reader
    # (e.g. `ros2 topic list | head`) already hung up. Benign — the follow-up
    # BrokenPipeError line is already excluded above (retro 15.08 t_a14ac65d).
    r"exception ignored in:",
    # dialogue_node INFO success echo: "process_input returned: ... error=None"
    # is the normal turn-completion line — error=None means NO error. The bare
    # \berror\b matcher hits "error=None" and would file a false deploy-critical
    # issue on a fully green round (retro 15.08 t_29230e6f, issue #1335: deploy
    # run 31886490619 SUCCESS + E2E SUCCESS, yet issue created; same for
    # round-129 run 32115362102 / issue #1364).
    r"error\s*=\s*none",
    r"error\s*:\s*none",
    # SuperCollider headless scsynth (issue #1485, deploy round-165): music
    # skill / FoxDot send /s_new for SynthDefs that ship with renardo but are
    # not pre-loaded in the headless scsynth image (notably `rhpiano`). The
    # "*** ERROR: SynthDef rhpiano not found" + "FAILURE IN SERVER /s_new
    # SynthDef not found" pair is benign — the music stack stays healthy and
    # TTS/voice flow continues. Real deployment failures still surface
    # because other scsynth-side errors (JackAudioDriver, sclang crashes,
    # rt-failures) keep their CRITICAL severity. The audio music-pipeline
    # team can address the underlying preload separately.
    r"error: synthdef \S+ not found",
    r"failure in server /s_new synthdef not found",
    # vesc_hardware_interface transient startup segfault (issue #1593,
    # deploy run 32771845791 24.08 / kanban t_efc1a364). libvesc_hardware_interface.so
    # starts CanInterface::receiveLoop() in parallel with hardware activate(),
    # so VescHandler::processCanFrame() can be called on a not-yet-initialised
    # VescHandler and dereferences a null pointer. Docker compose restart
    # policy (`unless-stopped` on the ros2-control service) brings the
    # container up on the second attempt and the controller_manager runs
    # fine (verified live: /joint_states 50 Hz, /diff_drive_controller/odom
    # publishing, ros2_control_node process at ~5% CPU). The deploy detector
    # must not file a critical issue for this transient — the actual race
    # is a vesc_nexus submodule bug (krikz/vesc_nexus@release/v1.0.0) and is
    # tracked separately. The exclusion patterns are intentionally narrow
    # (one pattern per stack-frame line; extract_relevant_log_line matches
    # each line independently and `re.search` does not cross newlines by
    # default). Bare "Segmentation fault" lines without these signatures
    # still match CRITICAL_MATCH_RE and are reported.
    r"libvesc_hardware_interface\.so.*receiveloop",
    r"libvesc_hardware_interface\.so.*processcanframe",
    # The bare SIGSEGV line that pairs with the libvesc_hardware_interface.so
    # stack frames above. Glibc on aarch64 prints this exact wording
    # ("Segmentation fault (Address not mapped to object [(nil)])") for a
    # NULL-pointer dereference, which is what the vesc race produces. We
    # silence it here ONLY when a libvesc_hardware_interface.so frame has
    # already been seen in the same log dump; the multi-line scan in
    # `extract_relevant_log_line` (see _vesc_segfault_window_active())
    # gates this rule on that precondition. A non-vesc segfault on the
    # same hardware produces a different stack trace (librtabmap_core.so,
    # libc backtrace, etc.) which does NOT carry libvesc_hardware_
    # interface.so frames, so this rule is skipped for those and the
    # operator still sees the deploy-fail issue. This avoids the false
    # positive filed by deploy run 32771845791 (issue #1593, kanban
    # t_efc1a364) on every staging deploy until the underlying
    # vesc_nexus submodule race is fixed.
    r"segmentation fault \(address not mapped to object \[\(nil\)\]\)",
    # `[ros2run]: Segmentation fault` is the python-ros2run wrapper's own
    # echo after the ros2_control_node child process exits with SIGSEGV —
    # it carries no extra diagnostic and always accompanies the libvesc_
    # stack-trace frames above, so silencing it is consistent with the
    # transient segfault exclusion. The bare "Segmentation fault (...)"
    # line itself is silenced via the main-scope vesc_segfault_window
    # exclusion below (CRITICAL_EXCLUDE_BY_SCOPE['main']), which uses a
    # multi-line scan to require a libvesc_hardware_interface.so frame
    # in the preceding lines of the same log dump.
    r"\[ros2run\]: segmentation fault",
]
CRITICAL_EXCLUDE_BY_SCOPE = {
    "main": [
        r"robot is out of bounds",
        r"serial port /dev/ttyusb0 still not available after",
        r"timed out waiting for transform from (base_link|base_footprint) to odom",
        # Same retro t_4acd6da0 / PR #1151 (base_footprint migration): nav2
        # global_costmap waits for /map at startup before rtabmap publishes
        # it. tf_static / odom→base_footprint is published first by diff_drive,
        # so the costmap log line is identical to the "to odom" race above.
        # Ретро 18.08 t_2bfa4793 / issue #1364 (round-129 run 32115362102):
        # deploy detector created an issue on a GREEN run because this
        # startup race was still flagged. Add the "to map" sibling pattern.
        r"timed out waiting for transform from (base_link|base_footprint) to map",
        r"cannot transform tag pose",
        r"sensor origin.*out of map bounds",
        r"can controller state: error-active",
        r"subscriberplugin::subscribeimpl with five arguments has not been overridden",
        r"total errors:",
        # Scope leak (issue #775): telegram_node lives in the vision
        # container (`telegram-bot` service, ROS_DOMAIN_ID=0 + network_mode:
        # host). Its ERROR lines leak into the perception container's docker
        # logs because health_monitor subscribes to the shared /rosout bus and
        # prints them in its periodic report. Upstream root cause is fixed by
        # PR #1145 (watchdog detects duplicate TELEGRAM_BOT_TOKEN holders);
        # this exclusion prevents false deployment-critical issues from being
        # filed against the perception container in the meantime.
        # telegram_node never runs in the main scope — any mention of it in
        # a main container log is by definition cross-container leak.
        r"telegram_node",
        r"telegram bot crashed",
        # Scope leak (issue #1368): audio_node lives in the voice-assistant
        # container on the Vision Pi. The Main Pi perception's
        # context_aggregator subscribes to /rosout (shared ROS_DOMAIN_ID=0
        # bus via Zenoh router) and prints "❌ Нода упала: /audio_node"
        # whenever the Vision Pi voice-assistant restarts audio_node. This
        # surfaces as a false deployment-critical against the perception
        # container, masking the actual issue location. Same shape as the
        # telegram_node exclusion above; the real root cause (voice-assistant
        # restart on Vision Pi) is not a deployment failure.
        r"нода упала: /audio_node",
        r"/audio_node\b",
    ],
    "vision": [
        # telegram_node start_polling transient (issue #1433 / deploy run
        # 32170854307, test round-147): on the first long-poll to
        # api.telegram.org the Vision Pi WiFi AP (10.1.1.1, see docs/adr/)
        # is still bringing the DNS/TLS path up, so PTB's
        # start_polling(timeout=30) raises asyncio.TimeoutError. The retry
        # loop in telegram_node._run_telegram_loop catches it, sleeps
        # 5s/10s/15s/... up to 60s and re-enters polling; on the 3rd attempt
        # the connection is warm and the bot stays up. The
        # "[ERROR] telegram_node: Bot crashed (N): Timed out. Retry in Ns"
        # line is the retry loop's own progress log, NOT a deployment
        # failure — container_status / topics / metrics are all healthy.
        # A real telegram_node failure (e.g. "Bot crashed (N): Conflict:
        # terminated by other getUpdates request") does not match this
        # pattern and is still reported; the negative test
        # test_extract_relevant_log_line_still_catches_telegram_node_real_conflict_in_vision_scope
        # covers that case.
        r"bot crashed \(\d+\): timed out\. retry in",
    ],
}

WARNING_EXCLUDE_COMMON = [
    r"^=== .*warnings ===$",
    # audio_node ReSpeaker threshold (issue #1485, deploy round-165):
    # voice-assistant prints "[WARN] [issue 989] ReSpeaker не принял
    # threshold 6.0 dB — программный гейт остаётся" whenever the
    # UAC1.0 ReSpeaker rejects the audio_node-issued hw-ctl threshold.
    # The fallback to a software gate is intentional (issue #989
    # round-117 retro): the audio chain stays healthy, barge-in works
    # through the SW gate, and the threshold issue is tracked
    # separately under issue #989. The Main Pi perception's
    # health_monitor also re-echoes the same warning over the shared
    # /rosout topic — placing this rule in COMMON covers both scopes.
    # Real audio_node failures (ASLA fatal, JACK
    # ProcessGraphAsyncMaster timeouts that miss recovery, sclang
    # crashes) keep their warning/critical severity because the
    # phrasing differs.
    r"\[issue 989\] respeaker не принял threshold",
    r"не принял threshold \d+\.\d+ db — программный гейт оста",
    r"scouting delay elapsed",
    r"нода не найдена",
    r"unknown logical group",
    r"error sending batch.*loki",
    r"enable watchconfig",
    r"serial port /dev/ttyusb0 not found",
    r"framerate:",
    r"animation already playing",
    r"pyaudio status: 2",
    r"speech .* not found in pending_speeches",
    r"speech .* не найден.*pending_speeches",
    r"did not receive data since 5 seconds",
    r"unable to connect to a zenoh router",
    r"could not fetch info from synthdefmanagement server\. using defaults",
    # STT empty-rejection noise: robot heard silence and rejected — not a
    # deployment failure (retro 12.08 t_d3e44336, issue #989, #684).
    r"отклонено \(пустое\)",
    r"yandex:empty\(.*\)->.*:empty\(.*\) -> rejected",
    r"отклонено \(короткое",
    r"интернет недоступен",
    # PyAudio overflow: input overrun is handled by the audio pipeline,
    # no data loss reported (retro 12.08 t_d3e44336).
    r"pyaudio painputoverflow",
    # zenoh session noise: "Received ResponseFinal for unknown Request"
    # is a stale reply to an already-timed-out query — benign (deploy
    # run round-117, ceiling-camera).
    r"received responsefinal for unknown request",
    # telegram_node startup echo: "Dropping dialogue echo, bot not ready /
    # no active chat" means the bot is up but no chat is bound yet — normal
    # boot behaviour, not a deployment failure (round-117, telegram-bot).
    r"dropping dialogue echo, bot not ready",
    # tts_node voice fallback (issue #1219): voice 'None' → default voice.
    # The chain keeps working; the warning itself is the fix notification.
    r"голос 'none' недоступен у minimax",
    # rtabmap ini auto-create: "Section \"Core\" ... doesn't exist" with the
    # explicit "Ignore this warning if the ini file does not exist yet" —
    # rtabmap creates the file on shutdown, benign (round-117).
    r"section .* in /config/rtabmap/rtabmap.ini doesn't exist",
]
WARNING_EXCLUDE_BY_SCOPE = {
    "main": [
        r"could not find a connection.*tree",
        r"это заглушка! используйте ai hat \+ yolo",
        r"root link.*inertia",
        r"no real-time kernel",
        r"old-style arguments are deprecated; see --help for new-style arguments",
        r"total warnings:",
        # perception_bridge: "Sensor UART /dev/ttyAMA0 not available; reads
        # will no-op until hardware is attached" — the UART IMU is optional
        # lab hardware, absence is expected in the test rig (round-117).
        r"sensor uart /dev/ttyama0 not available",
        # zenoh peer discovery: "Unable to connect to any locator of scouted
        # peer" during startup handshake is transient network noise, not an
        # outage (round-117, nav2).
        r"unable to connect to any locator of scouted peer",
        # rtabmap icp_odometry (issue #1485, deploy round-165):
        # "IcpOdometry: Transferring value 0.05 of 'Icp/VoxelSize' to ros
        # parameter 'scan_voxel_size' for convenience" is an informational
        # message printed by rtabmap whenever the YAML declares a
        # scan_voxel_size but Icp/VoxelSize is set to 0 — the mapping node
        # copies the YAML value into the RTAB-Map param transparently, the
        # SLAM pipeline is healthy. The word "WARN" is the only reason the
        # deploy gate flagged it (round-165).
        r"transferring value.*scan_voxel_size",
        # rtabmap icp_odometry (issue #1680, deploy round-244): sibling of
        # the scan_voxel_size pattern above. Same root cause (rtabmap
        # copies a YAML-declared Knn/K parameter into its public ROS param
        # namespace transparently), different destination param.
        # "Transferring value 5 of 'Icp/PointToPlaneK' to ros parameter
        # 'scan_normal_k' for convenience" — SLAM pipeline is healthy, the
        # WARN keyword is the only reason the deploy gate flagged it.
        r"transferring value.*scan_normal_k",
        # Scope leak warning (issue #1485, deploy round-165, sibling of
        # the WARNING_EXCLUDE_COMMON "не принял threshold" entry):
        # main/perception's health_monitor rewrites the audio_node line
        # without the "[issue 989]" prefix (`[WARN] audio_node (Ns ago): ⚠️
        # ReSpeaker не принял threshold ...`). The COMMON rule already
        # covers the literal phrase; this MAIN-scope pattern catches the
        # rewritten form so it cannot slip through as a non-voice-msg.
        r"\[warn\] audio_node.*не принял threshold",
    ],
    "vision": [
        # audio_node HPFONOFF write (issue #1680, deploy round-244):
        # voice-assistant prints "[WARN] HPFONOFF: write_parameter вернул
        # False (устройство занято?). Используется дефолт firmware."
        # whenever the UAC1.0 ReSpeaker rejects the audio_node-issued
        # high-pass filter control. The fallback to firmware default is
        # intentional (ADR-0013 §3.5: "USB write timeout / unknown device
        # → log-warning, нода продолжает работать с firmware default").
        # Real audio_node failures (ASLA fatal, JACK
        # ProcessGraphAsyncMaster timeouts that miss recovery, sclang
        # crashes) keep their severity because the phrasing differs.
        r"hpfonoff: write_parameter вернул false",
    ],
}


def normalize_pattern(raw_text: str) -> str:
    text = raw_text.strip()
    text = TIMESTAMP_RE.sub(" ", text)
    text = text.lower()
    text = URL_RE.sub("<url>", text)
    text = IP_RE.sub("<ip>", text)
    text = NUMBER_RE.sub("<num>", text)
    text = WHITESPACE_RE.sub(" ", text)
    return text.strip()


def _matches_any(patterns: Iterable[str], text: str) -> bool:
    return any(re.search(pattern, text, re.IGNORECASE) for pattern in patterns)


# Marker pattern for the vesc_hardware_interface transient segfault window
# (issue #1593). When any line of the log dump carries this marker, the
# bare "Segmentation fault (Address not mapped to object [(nil)])" line
# that pairs with it is silenced — see CRITICAL_EXCLUDE_COMMON.
_VESC_SEGFAULT_WINDOW_MARKER = re.compile(
    r"libvesc_hardware_interface\.so.*"
    r"(receiveloop|processcanframe)",
    re.IGNORECASE,
)


def _vesc_segfault_window_active(log_text: str) -> bool:
    """Return True if the log dump carries a libvesc_hardware_interface.so
    stack frame, which gates the vesc transient-segfault exclusion.

    The deploy detector iterates lines one by one, so a multi-line pattern
    (e.g. "Segmentation fault ... [(nil)]" preceded by a vesc stack frame)
    cannot be expressed as a single regex on a single line. Instead, we
    pre-scan the full log dump for the libvesc frame and only apply the
    bare-segfault exclusion when the frame is present. A non-vesc segfault
    on the same hardware produces a different stack trace (librtabmap_core.so,
    libc backtrace, ...) that does NOT carry libvesc_hardware_interface.so,
    so the window stays inactive and the operator still sees the real
    critical issue.
    """
    return bool(_VESC_SEGFAULT_WINDOW_MARKER.search(log_text))


def extract_relevant_log_line(log_text: str, *, scope: str, severity: str) -> str | None:
    if severity not in {"critical", "warning"}:
        raise ValueError(f"Unsupported severity: {severity}")

    lines = [line.strip() for line in log_text.splitlines() if line.strip()]
    if severity == "critical":
        patterns = list(CRITICAL_EXCLUDE_COMMON)
        # The bare "Segmentation fault (Address not mapped to object
        # [(nil)])" pattern is only meaningful inside the vesc race
        # window. Outside of it (a non-vesc segfault on the same
        # hardware), the operator must still see the deploy-fail issue.
        # See _vesc_segfault_window_active() for the rationale.
        if not _vesc_segfault_window_active(log_text):
            patterns = [
                p for p in patterns
                if p != r"segmentation fault \(address not mapped to object \[\(nil\)\]\)"
            ]
        patterns += CRITICAL_EXCLUDE_BY_SCOPE.get(scope, [])
        for line in lines:
            if not CRITICAL_MATCH_RE.search(line):
                continue
            if _matches_any(patterns, line):
                continue
            return line
        return None

    patterns = WARNING_EXCLUDE_COMMON + WARNING_EXCLUDE_BY_SCOPE.get(scope, [])
    for line in lines:
        if not WARNING_MATCH_RE.search(line):
            continue
        if _matches_any(patterns, line):
            continue
        return line
    return None


def build_signature(problem: Mapping[str, str]) -> str:
    normalized = normalize_pattern(problem["raw_text"])
    payload = ":".join(
        [
            problem["environment"],
            problem["scope"],
            problem["container"],
            problem["kind"],
            normalized,
        ]
    )
    digest = hashlib.sha256(payload.encode("utf-8")).hexdigest()[:12]
    return (
        f"deploy-problem:{problem['environment']}:{problem['scope']}:"
        f"{problem['container']}:{problem['kind']}:{digest}"
    )


def build_signature_marker(signature: str) -> str:
    return f"<!-- deploy-signature: {signature} -->"


def _build_issue_title(candidate: Mapping[str, str]) -> str:
    severity_emoji = "🚨" if candidate["severity"] == "critical" else "⚠️"
    severity_text = "Critical" if candidate["severity"] == "critical" else "Warning"
    scope = candidate["scope"].capitalize()
    return f"{severity_emoji} Deployment {severity_text}: {candidate['environment']} / {scope} / {candidate['container']} / {candidate['kind']}"


def _build_issue_body(
    candidate: Mapping[str, str],
    *,
    branch: str,
    workflow_run_url: str,
    timestamp: str,
    vision_pi_ip: str,
    main_pi_ip: str,
) -> str:
    diagnostic_ip = vision_pi_ip if candidate["scope"] == "vision" else main_pi_ip
    marker = build_signature_marker(candidate["signature"])
    return f"""## Deployment Problem Report

{marker}

**Branch:** `{branch}`
**Environment:** `{candidate['environment']}`
**Timestamp:** {timestamp}
**Workflow Run:** {workflow_run_url}

### Problem

- Scope: `{candidate['scope']}`
- Container: `{candidate['container']}`
- Kind: `{candidate['kind']}`
- Severity: `{candidate['severity']}`
- Occurrences in this run: {candidate['duplicate_count']}

### Summary

{candidate['summary']}

### Evidence

```text
{candidate['raw_text']}
```

### Quick Commands

```bash
sshpass -p 'open' ssh ros2@{diagnostic_ip} 'docker logs {candidate['container']} --tail 50'
```

---
*Auto-generated by deployment workflow*
"""


def prepare_issue_candidates(
    findings: Iterable[Mapping[str, str]],
    *,
    branch: str,
    workflow_run_url: str,
    timestamp: str,
    vision_pi_ip: str,
    main_pi_ip: str,
) -> list[dict[str, str | int | list[str]]]:
    grouped: dict[str, dict[str, str | int | list[str]]] = {}

    for finding in findings:
        signature = build_signature(finding)
        if signature not in grouped:
            candidate: dict[str, str | int | list[str]] = {
                "signature": signature,
                "signature_marker": build_signature_marker(signature),
                "environment": finding["environment"],
                "scope": finding["scope"],
                "container": finding["container"],
                "kind": finding["kind"],
                "severity": finding["severity"],
                "summary": finding["summary"],
                "raw_text": finding["raw_text"],
                "duplicate_count": 1,
                "labels": ["bug", "deployment"],
                "assignee": "krikz",
            }
            if finding["severity"] == "critical":
                candidate["labels"] = ["bug", "critical", "deployment"]
            candidate["title"] = _build_issue_title(candidate)  # type: ignore[arg-type]
            candidate["body"] = _build_issue_body(
                candidate,  # type: ignore[arg-type]
                branch=branch,
                workflow_run_url=workflow_run_url,
                timestamp=timestamp,
                vision_pi_ip=vision_pi_ip,
                main_pi_ip=main_pi_ip,
            )
            grouped[signature] = candidate
            continue

        grouped[signature]["duplicate_count"] = int(grouped[signature]["duplicate_count"]) + 1

    candidates = list(grouped.values())
    for candidate in candidates:
        candidate["title"] = _build_issue_title(candidate)  # type: ignore[arg-type]
        candidate["body"] = _build_issue_body(
            candidate,  # type: ignore[arg-type]
            branch=branch,
            workflow_run_url=workflow_run_url,
            timestamp=timestamp,
            vision_pi_ip=vision_pi_ip,
            main_pi_ip=main_pi_ip,
        )

    return sorted(candidates, key=lambda item: str(item["signature"]))


def extract_signature_markers(body: str) -> set[str]:
    return {match.group(1).strip() for match in SIGNATURE_MARKER_RE.finditer(body or "")}


def filter_new_candidates(
    candidates: Iterable[Mapping[str, str | int | list[str]]],
    existing_issues: Iterable[Mapping[str, object]],
) -> list[dict[str, str | int | list[str]]]:
    existing_signatures: set[str] = set()
    for issue in existing_issues:
        existing_signatures.update(extract_signature_markers(str(issue.get("body", ""))))

    return [dict(candidate) for candidate in candidates if str(candidate["signature"]) not in existing_signatures]


def load_json_file(path: str) -> object:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def load_findings(paths: Iterable[str]) -> list[dict[str, str]]:
    findings: list[dict[str, str]] = []
    for path in paths:
        file_path = Path(path)
        if not file_path.exists() or file_path.stat().st_size == 0:
            continue
        for line in file_path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            findings.append(json.loads(line))
    return findings


def _cmd_prepare(args: argparse.Namespace) -> int:
    findings = load_findings(args.findings_file)
    candidates = prepare_issue_candidates(
        findings=findings,
        branch=args.branch,
        workflow_run_url=args.workflow_run_url,
        timestamp=args.timestamp,
        vision_pi_ip=args.vision_pi_ip,
        main_pi_ip=args.main_pi_ip,
    )
    json.dump(candidates, sys.stdout, ensure_ascii=False, indent=2)
    sys.stdout.write("\n")
    return 0


def _cmd_filter(args: argparse.Namespace) -> int:
    candidates = load_json_file(args.candidates_file)
    existing_issues = load_json_file(args.issues_file)
    filtered = filter_new_candidates(candidates, existing_issues)  # type: ignore[arg-type]
    json.dump(filtered, sys.stdout, ensure_ascii=False, indent=2)
    sys.stdout.write("\n")
    return 0


def _cmd_extract_log(args: argparse.Namespace) -> int:
    log_text = sys.stdin.read()
    line = extract_relevant_log_line(log_text, scope=args.scope, severity=args.severity)
    if line:
        sys.stdout.write(line + "\n")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Prepare and deduplicate deployment issue candidates.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    prepare_parser = subparsers.add_parser("prepare", help="Build issue candidates from JSONL findings files.")
    prepare_parser.add_argument("--findings-file", action="append", required=True)
    prepare_parser.add_argument("--branch", required=True)
    prepare_parser.add_argument("--workflow-run-url", required=True)
    prepare_parser.add_argument("--timestamp", required=True)
    prepare_parser.add_argument("--vision-pi-ip", required=True)
    prepare_parser.add_argument("--main-pi-ip", required=True)
    prepare_parser.set_defaults(func=_cmd_prepare)

    filter_parser = subparsers.add_parser("filter", help="Remove candidates that already exist as open issues.")
    filter_parser.add_argument("--candidates-file", required=True)
    filter_parser.add_argument("--issues-file", required=True)
    filter_parser.set_defaults(func=_cmd_filter)

    extract_parser = subparsers.add_parser("extract-log", help="Extract the first relevant log line for a scope and severity.")
    extract_parser.add_argument("--scope", required=True, choices=["main", "vision"])
    extract_parser.add_argument("--severity", required=True, choices=["critical", "warning"])
    extract_parser.set_defaults(func=_cmd_extract_log)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
