"""Regression test for ``docker/main/scripts/ros2_control/start_ros2_control.sh``.

Issue #2727 (deploy-fail:develop:staging:2026-09-21, run 35645507502):
PR #2726 introduced a builder/runtime seam — the runtime stage of
``docker/main/ros2_control/Dockerfile`` now only contains ``/ws/install``
(via ``COPY --from=builder /ws/install /ws/install``), not ``/ws/src``.

The ``URDF_PATH`` default in ``start_ros2_control.sh`` still pointed to
``/ws/src/rob_box_description/urdf/rob_box.xacro`` from before the seam.
After PR #2726 was merged into ``develop`` and the resulting image was
deployed to Main Pi, ros2-control kept exit-coding on ``URDF not found``,
the container restarted 11 times in the first 15s, and the deploy failed.

Fix: change the default to the ament install layout that the runtime
stage actually ships:

    /ws/install/rob_box_description/share/rob_box_description/urdf/rob_box.xacro

This test asserts the default path is correct so any future regression
breaks the build instead of breaking the deploy.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = (
    REPO_ROOT
    / "docker"
    / "main"
    / "scripts"
    / "ros2_control"
    / "start_ros2_control.sh"
)


def _read_script() -> str:
    assert SCRIPT.exists(), f"missing start script: {SCRIPT}"
    return SCRIPT.read_text(encoding="utf-8")


def _default_urdf_path() -> str:
    """Return the default URDF_PATH from the script (no shell eval)."""
    text = _read_script()
    m = re.search(r'^URDF_PATH="\$\{URDF_PATH:-(?P<default>[^}]+)\}"', text, re.MULTILINE)
    assert m is not None, (
        "URDF_PATH default not found in start_ros2_control.sh; "
        "the variable was renamed or removed"
    )
    return m.group("default")


class TestStartRos2ControlUrdfPath:
    """Regression coverage for deploy #2727 (URDF not found after #2726)."""

    def test_default_urdf_path_does_not_use_ws_src(self) -> None:
        # /ws/src is the BUILDER layout — not available in runtime after PR #2726.
        default = _default_urdf_path()
        assert "/ws/src/" not in default, (
            "URDF_PATH default still points to /ws/src/... — that's the "
            "builder layout. After PR #2726 the runtime image only has "
            "/ws/install, so ros2-control fails with 'URDF not found' and "
            "restart-loops (deploy #2727 / RestartCount=11)."
        )

    def test_default_urdf_path_uses_ament_install_layout(self) -> None:
        # Standard ament layout: install/<pkg>/share/<pkg>/urdf/<file>.xacro
        default = _default_urdf_path()
        assert default.startswith("/ws/install/"), (
            f"URDF_PATH default must start with /ws/install/ "
            f"(runtime stage seam from PR #2726), got {default!r}"
        )
        assert "/share/rob_box_description/urdf/" in default, (
            f"URDF_PATH default must use ament share/ layout "
            f"(install/<pkg>/share/<pkg>/urdf/), got {default!r}"
        )
        assert default.endswith("/rob_box.xacro"), (
            f"URDF_PATH default must end with rob_box.xacro, got {default!r}"
        )

    def test_full_default_path_under_rob_box_description_share(self) -> None:
        # Specific path: /ws/install/rob_box_description/share/rob_box_description/urdf/rob_box.xacro
        # — matches the `install(DIRECTORY ... DESTINATION share/${PROJECT_NAME})` in
        # src/rob_box_description/CMakeLists.txt combined with colcon's default
        # install prefix /ws/install.
        default = _default_urdf_path()
        expected = (
            "/ws/install/rob_box_description/share/"
            "rob_box_description/urdf/rob_box.xacro"
        )
        assert default == expected, (
            f"URDF_PATH default mismatch.\n"
            f"  expected: {expected}\n"
            f"  actual:   {default}\n"
            "If the path was intentionally changed, update this assertion "
            "AND verify against a real `colcon build` output of "
            "rob_box_description."
        )

    def test_script_executable_bit_not_required(self) -> None:
        # Sanity: file is tracked in repo. (We don't require +x on a worktree copy.)
        text = _read_script()
        assert "#!/bin/bash" in text, (
            "start_ros2_control.sh must keep its bash shebang for the runtime image"
        )

    def test_script_documented_path_change_for_deploy_2727(self) -> None:
        # The script's leading comment block should reference the regression it fixes.
        # If someone reverts the path, this comment becomes stale — and they should
        # also remove the comment or update this test to reflect the new state.
        text = _read_script()
        assert "#2727" in text or "2727" in text, (
            "start_ros2_control.sh must reference issue #2727 in its comment "
            "block so the next reader understands why the path is non-obvious"
        )
