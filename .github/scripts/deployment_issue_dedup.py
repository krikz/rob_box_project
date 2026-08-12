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
    # Clock-skew noise: zenoh router replaces the offending timestamp and
    # forwards the message — data is not lost, so this is not an outage.
    # Root cause is NTP desync between Pis (see scripts/maintenance/sync_time.sh).
    r"error treating timestamp for received data",
    # ALSA/jackd noise: Invalid CTL on dmix_respeaker is a benign control
    # probe failure, not an audio outage (retro 12.08 t_d3e44336).
    r"alsa lib control\.c.*invalid ctl",
    r"dmix_respeaker",
]
CRITICAL_EXCLUDE_BY_SCOPE = {
    "main": [
        r"robot is out of bounds",
        r"serial port /dev/ttyusb0 still not available after",
        r"timed out waiting for transform from (base_link|base_footprint) to odom",
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
        # rtabmap icp_odometry без свежего IMU — известный шум, не аутэйдж:
        # OAK-D публикует IMU в /camera/camera/imu; если топик недоступен/пуст,
        # icp_odometry пишет ERROR "We didn't receive IMU newer than previous
        # image/scan". SLAM продолжает работать (ICP по лидару + wheel odom guess),
        # поэтому это не critical. См. issue #681.
        r"didn't receive imu newer",
        r"dropping image/scan data.*delay",
    ],
    "vision": [],
}

WARNING_EXCLUDE_COMMON = [
    r"^=== .*warnings ===$",
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
]
WARNING_EXCLUDE_BY_SCOPE = {
    "main": [
        r"could not find a connection.*tree",
        r"это заглушка! используйте ai hat \+ yolo",
        r"root link.*inertia",
        r"no real-time kernel",
        r"old-style arguments are deprecated; see --help for new-style arguments",
        r"total warnings:",
    ],
    "vision": [],
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


def extract_relevant_log_line(log_text: str, *, scope: str, severity: str) -> str | None:
    if severity not in {"critical", "warning"}:
        raise ValueError(f"Unsupported severity: {severity}")

    lines = [line.strip() for line in log_text.splitlines() if line.strip()]
    if severity == "critical":
        patterns = CRITICAL_EXCLUDE_COMMON + CRITICAL_EXCLUDE_BY_SCOPE.get(scope, [])
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
