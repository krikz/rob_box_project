"""Raw Gazebo при повторах подъёма сцены (offline guard, без HIL)."""

import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "hil" / "speed" / "run_scene.sh"


def test_failed_attempt_saves_gazebo_log_before_remove(tmp_path):
    text = SCRIPT.read_text(encoding="utf-8")
    marker = "sim_controllers_ok() {"
    functions_body = text.split(marker, 1)[1]
    functions = marker + functions_body.split("\nsleep 15\n", 1)[0]
    retry_body = text.split("up=0\n", 1)[1]
    retry = "up=0\n" + retry_body.split('\nlog "сим поднят', 1)[0]
    harness = tmp_path / "retry.sh"
    harness.write_text(
        """set -uo pipefail
G=rob-box-hil-gazebo
SIMDC=mock
RES="$PWD"
EVENTS="$PWD/events.txt"
log() { :; }
mn() { printf 'exited\\n'; }
sleep() { :; }
sb() {
    case "$*" in
        "docker logs --timestamps "*)
            printf 'capture\\n' >>"$EVENTS"
            printf 'timestamped spawner raw\\n'
            ;;
        *"grep -cE"*) printf '1\\n' ;;
        *"grep -c 'Configured and activated'"*) printf '0\\n' ;;
        *"up -d --no-build gazebo"*) : ;;
        *"rob_box_sim.hil count"*) printf '1\\n' ;;
        *"docker inspect -f"*) printf 'running\\n' ;;
        *"rm -sf gazebo"*) printf 'remove\\n' >>"$EVENTS" ;;
        *) printf 'unexpected sb: %s\\n' "$*" >&2; return 1 ;;
    esac
}
"""
        + functions
        + "\n"
        + retry
        + "\n",
        encoding="utf-8",
        newline="\n",
    )

    result = subprocess.run(
        ["bash", harness.name],
        cwd=tmp_path,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 6, result.stderr
    assert [
        (tmp_path / f"gazebo_attempt_{n}.log").read_text() for n in range(1, 5)
    ] == ["timestamped spawner raw\n"] * 4
    assert (tmp_path / "events.txt").read_text().splitlines() == [
        "capture",
        "remove",
    ] * 4


@pytest.mark.skipif(shutil.which("bash") is None, reason="нет bash")
def test_shell_syntax():
    subprocess.run(["bash", "-n", SCRIPT.name], check=True, cwd=SCRIPT.parent)
