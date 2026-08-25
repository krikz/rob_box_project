"""Regression test for issue #1593.

Issue: deployment_issue_dedup.extract_relevant_log_line() must exclude
the vesc_hardware_interface transient startup segfault.

Reasoning: when the Main Pi `ros2-control` container starts, the
`vesc_hardware_interface` plugin (`src/vesc_nexus`, submodule at
release/v1.0.0) starts `CanInterface::receiveLoop()` in parallel with
hardware `activate()`. On the very first CAN frame after activate,
`VescHandler::processCanFrame()` is called on a not-yet-initialised
VescHandler and dereferences a null pointer → SIGSEGV. The Docker
compose service uses `restart: unless-stopped` (docker/main/docker-compose.yaml
around the ros2-control service definition) and the second start succeeds —
the controller_manager runs fine, /joint_states publishes at 50 Hz,
/diff_drive_controller/odom is healthy. So the only effect of the first
segfault is a one-shot critical_log finding in `docker logs ros2-control`,
which used to file a deploy-fail issue (#1593) on every staging deploy.

This test pins down two contracts:

1. POSITIVE: a stack-trace frame that points at libvesc_hardware_interface.so
   is NOT returned by `extract_relevant_log_line(..., severity="critical")`
   in the main scope (the only scope where ros2-control runs).

2. NEGATIVE: other critical patterns (e.g. generic Python traceback,
   unrelated segfault in a different .so, real controller_manager error)
   are STILL returned, so we don't accidentally silence real failures.
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


# ----- 1. POSITIVE: vesc segfault must be excluded -----

VESC_SEGFAULT_STACK_TRACE = "\n".join(
    [
        "2026-08-24T20:27:09.345199386Z [INFO] [resource_manager]: "
        "Successful 'activate' of hardware 'VescSystem'",
        "Stack trace (most recent call last) in thread 43:",
        "#5  Object \"/usr/lib/aarch64-linux-gnu/ld-linux-aarch64.so.1\", "
        "at 0xffffffffffffffff, in ",
        "#4  Object \"/usr/lib/aarch64-linux-gnu/libc.so.6\", "
        "at 0xffffb5a99edb, in ",
        "#3  Object \"/usr/lib/aarch64-linux-gnu/libc.so.6\", "
        "at 0xffffb5a303c7, in ",
        "#2  Object \"/usr/lib/aarch64-linux-gnu/libstdc++.so.6.0.30\", "
        "at 0xffffb5c629cb, in ",
        "#1  Object \"/ws/build/vesc_nexus/libvesc_hardware_interface.so\", "
        "at 0xffffb00ea4fb, in CanInterface::receiveLoop()",
        "#0  Object \"/ws/build/vesc_nexus/libvesc_hardware_interface.so\", "
        "at 0xffffb00e77c4, in VescHandler::processCanFrame(can_frame const&)",
        "Segmentation fault (Address not mapped to object [(nil)])",
        "[ros2run]: Segmentation fault",
    ]
)


def test_extract_relevant_log_line_ignores_vesc_hardware_interface_segfault_in_main_scope() -> None:
    """The full vesc_hardware_interface segfault stack-trace must be excluded.

    This mirrors the exact docker logs content from run 32771845791
    (issue #1593): on the first start of the ros2-control container,
    the receiveLoop/processCanFrame path segfaults on a not-yet-initialised
    VescHandler. Docker compose restarts the container, the second start
    succeeds. The deploy gate must not file a critical issue for this.
    """
    line = MODULE.extract_relevant_log_line(
        VESC_SEGFAULT_STACK_TRACE, scope="main", severity="critical"
    )
    assert line is None, (
        "vesc_hardware_interface transient segfault must be excluded; "
        f"got: {line!r}"
    )


def test_extract_relevant_log_line_ignores_vesc_segfault_in_vision_scope() -> None:
    """The same exclusion applies in vision scope too.

    ros2-control currently runs only on the Main Pi, but the exclusion
    lives in CRITICAL_EXCLUDE_COMMON (not in CRITICAL_EXCLUDE_BY_SCOPE['main']),
    so it must also work if someone moves the service to vision or if the
    stack trace leaks into the vision container via the shared /rosout
    bus. This test pins that contract.
    """
    line = MODULE.extract_relevant_log_line(
        VESC_SEGFAULT_STACK_TRACE, scope="vision", severity="critical"
    )
    assert line is None


# ----- 2. NEGATIVE: real critical errors must still be reported -----


def test_extract_relevant_log_line_still_catches_real_critical_error_in_main_scope() -> None:
    """A real critical-log line unrelated to vesc must still be reported.

    Ensures the new exclusion does not accidentally silence a different
    critical_log. Note: ``extract_relevant_log_line`` returns the FIRST
    line that matches the critical regex and is not excluded — for a
    Python traceback that is the ``Traceback (most recent call last):``
    header (the actual exception type/value is on later lines). The
    deploy detector's job is to surface any critical pattern; the operator
    reads the full log dump from the GitHub Actions artifact for
    triage. We only assert that some critical line is returned, not a
    specific one.
    """
    log_text = (
        "[perception-1] [ERROR] perception_bridge: "
        "Traceback (most recent call last):\n"
        "  File /opt/ros/humble/lib/python3.10/site-packages/perception_bridge/_serial.py line 42 in _callback\n"
        "RuntimeError: serial port /dev/ttyUSB0 returned garbage"
    )

    line = MODULE.extract_relevant_log_line(
        log_text, scope="main", severity="critical"
    )
    assert line is not None
    assert "Traceback" in line or "RuntimeError" in line or "ttyUSB0" in line


def test_extract_relevant_log_line_still_catches_unrelated_segfault_in_main_scope() -> None:
    """A bare Segmentation fault from a different binary must still be reported.

    A segfault in any other .so (not libvesc_hardware_interface.so) is
    a real critical event that the operator must see. The exclusion is
    deliberately scoped to the vesc plugin.
    """
    log_text = (
        "[perception-1] Segmentation fault (Address not mapped to object [(nil)])\n"
        "#0  Object /opt/ros/humble/lib/librtabmap_core.so "
        "at 0xffffa1234 in rtabmap core update"
    )

    line = MODULE.extract_relevant_log_line(
        log_text, scope="main", severity="critical"
    )
    assert line is not None
    assert "Segmentation fault" in line


def test_extract_relevant_log_line_still_catches_vesc_real_error_in_main_scope() -> None:
    """A real vesc-side error that is NOT the startup segfault must be reported.

    The exclusion only silences the specific stack-trace frames that
    identify the receiveLoop/processCanFrame race. Other vesc-related
    critical_log lines (e.g. CAN bus-off, hardware fault) still carry
    real diagnostic value and must surface.
    """
    log_text = (
        "[ros2-control] [ERROR] [VescSystemHardwareInterface]: "
        "CAN bus-off detected on can0 — motors disabled for safety. "
        "Check wiring and termination resistors."
    )

    line = MODULE.extract_relevant_log_line(
        log_text, scope="main", severity="critical"
    )
    assert line is not None
    assert "bus-off" in line
