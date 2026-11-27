"""Guard against APT fetching an archive in place of a vendored Zenoh .deb."""

import os
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
VENDOR = ROOT / "docker/base/vendor"
BASH = shutil.which("bash")


@pytest.mark.skipif(BASH is None, reason="bash is required to record the package commands")
@pytest.mark.parametrize(
    ("dockerfile", "arch"),
    [
        (f"docker/base/Dockerfile.{base}", arch)
        for base in ("ros2-zenoh", "depthai", "rtabmap")
        for arch in ("arm64", "amd64")
    ]
    + [("docker/sim/Dockerfile", "amd64")],
)
def test_vendored_zenoh_unpacks_before_apt_resolves_dependencies(dockerfile: str, arch: str) -> None:
    content = (ROOT / dockerfile).read_text(encoding="utf-8")
    layer = content.split("/tmp/rmw-zenoh-vendor/\nRUN ", 1)[1].split("\n\n", 1)[0]
    # Run the image's shell logic with package managers and cleanup replaced by recorders.
    shell = "\n".join(
        [
            'record() { printf "%s\\t" "$@"; printf "\\n"; }',
            'dpkg() { if [ "$1" = --print-architecture ]; then echo "$TEST_ARCH"; else record dpkg "$@"; fi; }',
            'apt-get() { record apt-get "$@"; }',
            'rm() { :; }',
            layer.replace("/tmp/rmw-zenoh-vendor/", VENDOR.as_posix() + "/"),
        ]
    )
    env = {**os.environ, "TEST_ARCH": arch, "RMW_ZENOH_VENDOR": "true"}
    result = subprocess.run([BASH, "-c", shell], env=env, capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr

    commands = [line.rstrip("\t").split("\t") for line in result.stdout.splitlines()]
    packages = ("ros-humble-zenoh-cpp-vendor", "ros-humble-rmw-zenoh-cpp")
    local = [next(VENDOR.glob(f"{package}_*_{arch}.deb")) for package in packages]
    unpack = ["dpkg", "--unpack", *(path.as_posix() for path in local)]
    configure = commands[-1]
    assert unpack in commands[:-1], commands
    assert configure[0] == "apt-get" and "install" in configure and "--fix-broken" in configure
    assert not any(arg.endswith(".deb") for cmd in commands if cmd[0] == "apt-get" for arg in cmd)
    for package, path in zip(packages, local):
        assert f"{package}={path.name.split('_')[1]}" in configure, configure
