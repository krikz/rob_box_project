"""Запуск контроллера после подтверждённого update cycle Gazebo."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time


def _wait_for_gazebo_state(timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        listed = subprocess.run(
            ["ign", "service", "-l"],
            capture_output=True,
            text=True,
            timeout=max(0.1, min(5.0, remaining)),
        )
        service = next(
            (
                line
                for line in listed.stdout.splitlines()
                if line.startswith("/world/") and line.endswith("/state")
            ),
            None,
        )
        if service is not None:
            remaining = deadline - time.monotonic()
            state = subprocess.run(
                [
                    "ign",
                    "service",
                    "-s",
                    service,
                    "--reqtype",
                    "ignition.msgs.Empty",
                    "--reptype",
                    "ignition.msgs.SerializedStepMap",
                    "--timeout",
                    str(max(1, int(remaining * 1000))),
                    "--req",
                    "",
                ],
                capture_output=True,
                text=True,
                timeout=max(0.1, remaining),
            )
            if state.returncode == 0 and "true" in state.stdout:
                return True
        time.sleep(min(0.2, max(0.0, deadline - time.monotonic())))
    return False


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Spawner контроллера после ответа Gazebo state service",
        allow_abbrev=False,
    )
    ap.add_argument("controller")
    ap.add_argument("--startup-timeout", type=float, default=60.0)
    args, spawner_args = ap.parse_known_args(argv)

    if not _wait_for_gazebo_state(args.startup_timeout):
        print(
            f"Gazebo state service не подтвердил update cycle за {args.startup_timeout:g} с; "
            f"контроллер {args.controller} не запущен",
            file=sys.stderr,
        )
        return 1

    os.execvp(
        "ros2",
        [
            "ros2",
            "run",
            "controller_manager",
            "spawner",
            args.controller,
            *spawner_args,
        ],
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
