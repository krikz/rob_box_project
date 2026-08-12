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
