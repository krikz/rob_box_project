"""Exercise the production copy steps against stock and rebuilt Nav2 artifacts (#5226)."""

import os
import shutil
import subprocess
from pathlib import Path


def test_image_installs_rebuilt_nav2_servers_and_plugins(tmp_path: Path):
    artifacts = {
        "nav2_controller": ["nav2_controller/controller_server", "libcontroller_server_core.so"],
        "nav2_behaviors": [
            "nav2_behaviors/behavior_server",
            "libbehavior_server_core.so",
            "libnav2_spin_behavior.so",
            "libnav2_wait_behavior.so",
            "libnav2_assisted_teleop_behavior.so",
            "libnav2_drive_on_heading_behavior.so",
            "libnav2_back_up_behavior.so",
        ],
        "nav2_costmap_2d": ["libnav2_costmap_2d_core.so"],
        "nav2_navfn_planner": ["libnav2_navfn_planner.so"],
    }
    for package, paths in artifacts.items():
        for path in paths:
            for root in (tmp_path / "ros", tmp_path / "colcon" / package):
                file = root / "lib" / path
                file.parent.mkdir(parents=True, exist_ok=True)
                file.write_text(f"{root.name}/{path}", encoding="utf-8")

    dockerfile = Path(__file__).resolve().parents[3] / "docker/main/nav2/Dockerfile"
    commands = dockerfile.read_text(encoding="utf-8").replace("\\\n", " ").split("&&")
    copies = [cmd.strip() for cmd in commands if cmd.lstrip().startswith("cp /tmp/nav2-fix/install/")]
    staging = " && ".join(copies).replace("/tmp/nav2-fix/install/", "colcon/")
    staging = staging.replace("/opt/ros/${ROS_DISTRO}/", "ros/")
    shell = shutil.which("sh")
    if not shell and os.name == "nt":
        # Git for Windows ships sh and cp even when only git.exe is on PATH.
        shell = Path(shutil.which("git")).resolve().parents[1] / "bin/sh.exe"
    subprocess.run([shell or "sh", "-ec", staging], cwd=tmp_path, check=True)

    stale = [
        path
        for package, paths in artifacts.items()
        for path in paths
        if (tmp_path / "ros/lib" / path).read_bytes() != (tmp_path / "colcon" / package / "lib" / path).read_bytes()
    ]
    assert not stale, f"stock artifacts left beside patched Nav2 libraries: {stale}"
