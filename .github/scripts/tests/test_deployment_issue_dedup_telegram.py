"""Regression test for issue #775.

Issue: deployment_issue_dedup.extract_relevant_log_line() must exclude
[telegram_node] errors when scanning the perception container logs.

Reasoning: telegram_node lives in the Vision Pi `telegram-bot` container
(docker/vision/docker-compose.yaml). The `perception` container's
health_monitor subscribes to /rosout (shared ROS_DOMAIN_ID=0 bus) and
prints ERROR lines from any node. When telegram_node crashes and restarts
(e.g. due to a duplicate TELEGRAM_BOT_TOKEN held by multiple gateways),
its ERROR lines leak into the perception container's docker logs.

The deployment workflow then files a critical issue against perception,
even though the actual root cause is in the vision container. Adding
`telegram_node` to the main-scope critical exclusion list stops this
false-positive.

Upstream root cause is fixed by PR #1145 (watchdog detects duplicate
telegram token holders + reconnect loops).
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "deployment_issue_dedup.py"
)
SPEC = importlib.util.spec_from_file_location(
    "deployment_issue_dedup", SCRIPT_PATH
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC is not None and SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_extract_relevant_log_line_ignores_telegram_node_in_main_scope() -> None:
    """telegram_node errors must be ignored when scanning main container logs.

    telegram_node lives in the vision container; its ERROR lines leak into
    main container logs via the shared /rosout bus (health_monitor prints them).
    """

    log_text = (
        "[health_monitor-1]   [ERROR] telegram_node (8s ago): "
        "Telegram bot crashed (attempt 12): Timed out. Restarting in"
    )

    line = MODULE.extract_relevant_log_line(
        log_text, scope="main", severity="critical"
    )

    assert line is None


def test_extract_relevant_log_line_still_catches_telegram_node_in_vision_scope() -> (
    None
):
    """Negative test: same line in vision scope MUST still be reported.

    The vision container IS the real owner of telegram_node. We must
    only silence it for the main scope.
    """

    log_text = (
        "[telegram-bot] [ERROR] telegram_node: "
        "Telegram bot crashed (attempt 12): Timed out. Restarting in"
    )

    line = MODULE.extract_relevant_log_line(
        log_text, scope="vision", severity="critical"
    )

    assert line == log_text


def test_extract_relevant_log_line_ignores_telegram_start_polling_timeout_in_vision_scope() -> (
    None
):
    """Issue #1433 / deploy run 32170854307 (test round-147): on the first
    long-poll to api.telegram.org the Vision Pi WiFi AP is still bringing
    the DNS/TLS path up, so PTB's start_polling(timeout=30) raises
    asyncio.TimeoutError. telegram_node._run_telegram_loop catches it,
    sleeps 5s/10s/... and re-enters polling; on the 3rd attempt the
    connection is warm and the bot stays up. The
    "[ERROR] telegram_node: Bot crashed (N): Timed out. Retry in Ns" line
    is the retry loop's own progress log, NOT a deployment failure —
    container_status / topics / metrics are all healthy. The deploy gate
    must not flag it as a critical issue.

    The format ``Bot crashed (N): Timed out. Retry in Ns`` is exact; the
    regex is case-insensitive on the substring ``bot crashed (N): timed
    out. retry in`` so the captured text (with the ``s`` suffix on
    ``Retry in``) still matches.
    """

    log_text = (
        "[ERROR] [1787077612.581925751] [telegram_node]: "
        "Bot crashed (1): Timed out. Retry in 5s"
    )

    line = MODULE.extract_relevant_log_line(
        log_text, scope="vision", severity="critical"
    )

    assert line is None


def test_extract_relevant_log_line_still_catches_telegram_real_conflict_in_vision_scope() -> (
    None
):
    """Negative test for the issue #1433 exclusion: a REAL telegram_node
    failure (different exception type — ``Conflict: terminated by other
    getUpdates request``, i.e. duplicate-token holder caught by the
    PR #1145 watchdog) MUST still be reported in the vision scope. The
    exclusion is narrow to the start_polling TimeoutError transient only.
    """

    log_text = (
        "[ERROR] [1787077612.581925751] [telegram_node]: "
        "Bot crashed (3): Conflict: terminated by other getUpdates "
        "request; make sure that only one bot instance is running. "
        "Retry in 15s"
    )

    line = MODULE.extract_relevant_log_line(
        log_text, scope="vision", severity="critical"
    )

    assert line == log_text


def test_extract_relevant_log_line_still_catches_telegram_unrelated_error_in_vision_scope() -> (
    None
):
    """Negative test for the issue #1433 exclusion: an unrelated
    telegram_node ERROR that doesn't mention ``Timed out. Retry in``
    MUST still be reported in the vision scope (e.g. a YAML parse
    failure on TELEGRAM_ALLOWED_USERS).
    """

    log_text = (
        "[ERROR] [1787077612.581925751] [telegram_node]: "
        "Invalid TELEGRAM_ALLOWED_USERS value: '495039871,6059238600,"
        "-5269346516' — must be a comma-separated list of integers"
    )

    line = MODULE.extract_relevant_log_line(
        log_text, scope="vision", severity="critical"
    )

    assert line == log_text
