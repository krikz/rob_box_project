"""Разбор capture гейта twist_mux (scripts/hil/checks/mux_gate.py) без ROS и стенда.

Живая инъекция на стенде не проверялась. Здесь — вердикт по записанным пробам
2026-11-20 (/tmp/m4_depot_20261120, katana) и по синтетике, помеченной честно:
обрыв 15:31 в файл capture не снят, числа взяты из замера координатора.
Плюс opt-in: без HIL_MUX_GATE=1 обёртка runvideo.sh гейт не вызывает.
"""

from __future__ import annotations

import importlib.util
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
GATE_PATH = ROOT / "scripts" / "hil" / "checks" / "mux_gate.py"
RUNVIDEO = ROOT / "scripts" / "hil" / "video" / "runvideo.sh"
RECORDINGS = Path("/tmp/m4_depot_20261120")

_spec = importlib.util.spec_from_file_location("hil_checks_mux_gate", GATE_PATH)
gate = importlib.util.module_from_spec(_spec)
sys.modules["hil_checks_mux_gate"] = gate
_spec.loader.exec_module(gate)


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _section(text: str, which: str) -> str:
    """turn_probe/capture.txt хранит два замера подряд: поворот, затем === ZERO ===."""
    if which == "turn":
        return text.split("=== ZERO ===", 1)[0]
    if which == "zero":
        return text.split("=== ZERO ===", 1)[1]
    return text


@pytest.mark.skipif(
    not RECORDINGS.is_dir(), reason="записей /tmp/m4_depot_20261120 на этой машине нет"
)
def test_recordings_20261120_match_analyst_verdicts():
    """Три живых захвата — PASS, обрыв (синтетика по числам координатора) — FAIL.

    Файлы сняты на katana, не этим прогоном. Хвост нулей 16:01 — отдельная инъекция
    нулей, выход честно их повторил, поэтому PASS, а не FAIL.
    """
    cases = [
        (
            "15:57 alive",
            RECORDINGS / "m3m4_std/pre_restart_capture/capture.txt",
            None,
            gate.PASS,
        ),
        (
            "16:01 turn",
            RECORDINGS / "m3m4_std/turn_probe/capture.txt",
            "turn",
            gate.PASS,
        ),
        (
            "16:01 zero inject",
            RECORDINGS / "m3m4_std/turn_probe/capture.txt",
            "zero",
            gate.PASS,
        ),
        (
            "16:47 alive",
            RECORDINGS / "m3m4_std/probe_1647/cap_prestart2.txt",
            None,
            gate.PASS,
        ),
    ]
    for name, path, section, expect in cases:
        text = _section(_read(path), section) if section else _read(path)
        verdict, reasons = gate.parse_capture(text)
        assert verdict == expect, f"{name}: {verdict} {reasons}"
        assert not reasons, f"{name}: {reasons}"


def test_dead_window_1531_from_coordinator_numbers_is_fail():
    """СИНТЕТИКА. Обрыв 15:31 файлом capture не снят.

    Числа — замер координатора, на котором разборщик получил FAIL: 49 вколотых
    vx=-0.1, на выходе 24 нуля, часы живые, защёлки и замков нет. Совпавших нет.
    """
    text = "\n".join(
        [
            "sent 49 vx -0.1 wz 0.0 seconds 1.5",
            "clock count 1000 first 1.000 last 2.000",
            "mux_out count 24 nonzero 0 sample [(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)]",
            "joystick_lock count 10 nonzero 0 sample [False, False, False]",
            "teleop_lock count 10 nonzero 0 sample [False, False, False]",
            "DIAG",
            "  velocity topics.navigation = masked",
            "  current priority = 0",
        ]
    )
    verdict, reasons = gate.parse_capture(text)
    assert verdict == gate.FAIL
    assert any(r.startswith("NOT_FORWARDED") for r in reasons), reasons
    assert not any(
        r.startswith("LATCH") or r.startswith("LOCK") or r.startswith("CLOCK")
        for r in reasons
    )


def test_latch_unmasked_above_navigation_is_fail():
    """СИНТЕТИКА. Выход совпал, но yaw_at_goal (priority 15) unmasked — это защёлка, FAIL."""
    text = "\n".join(
        [
            "sent 15 vx 0.1 wz 0.0 seconds 1.5",
            "clock count 100 first 10.000 last 12.000",
            "mux_out count 15 nonzero 15 sample [(0.1, 0.0), (0.1, 0.0), (0.1, 0.0)]",
            "joystick_lock count 5 nonzero 0 sample [False]",
            "teleop_lock count 5 nonzero 0 sample [False]",
            "DIAG",
            "  velocity topics.yaw_at_goal = unmasked (listening to cmd_vel_yaw)",
            "  velocity topics.navigation = masked",
        ]
    )
    verdict, reasons = gate.parse_capture(text)
    assert verdict == gate.FAIL
    assert any("yaw_at_goal" in r for r in reasons), reasons


def test_frozen_clock_is_fail_even_when_output_matches():
    """СИНТЕТИКА. Часы не шли: проверка не может отличить защёлку от исправного mux."""
    text = "\n".join(
        [
            "sent 15 vx 0.1 wz 0.0 seconds 1.5",
            "clock count 0",
            "mux_out count 15 nonzero 15 sample [(0.1, 0.0)]",
            "joystick_lock count 0 nonzero 0 sample []",
            "teleop_lock count 0 nonzero 0 sample []",
        ]
    )
    verdict, reasons = gate.parse_capture(text)
    assert verdict == gate.FAIL
    assert any(r.startswith("CLOCK_FROZEN") for r in reasons), reasons


def test_lock_true_is_fail():
    """СИНТЕТИКА. Замок teleop_lock data=true — маршрут не слать, даже если выход совпал."""
    text = "\n".join(
        [
            "sent 15 vx 0.1 wz 0.0 seconds 1.5",
            "clock count 50 first 3.000 last 5.000",
            "mux_out count 15 nonzero 15 sample [(0.1, 0.0)]",
            "joystick_lock count 4 nonzero 0 sample [False]",
            "teleop_lock count 4 nonzero 2 sample [True, True]",
        ]
    )
    verdict, reasons = gate.parse_capture(text)
    assert verdict == gate.FAIL
    assert any(r.startswith("LOCK: teleop_lock") for r in reasons), reasons


def test_mixed_sample_does_not_invent_forwarded_count():
    """СИНТЕТИКА. Sample неоднородный — совпавшими считаются только его пары, порог не добран."""
    text = "\n".join(
        [
            "sent 15 vx 0.1 wz 0.0 seconds 1.5",
            "clock count 20 first 1.000 last 2.000",
            "mux_out count 15 nonzero 15 sample [(0.1, 0.0), (0.0, 0.0), (0.1, 0.0)]",
            "joystick_lock count 0 nonzero 0 sample []",
            "teleop_lock count 0 nonzero 0 sample []",
        ]
    )
    verdict, reasons = gate.parse_capture(text)
    assert verdict == gate.FAIL
    assert any("matching 2" in r for r in reasons), reasons


def test_sentinel_refuses_without_hil_lock_and_outside_hil_container():
    """Скорость публикуется только в контейнере HIL при живом замке hil.sh on."""
    assert gate.hil_sentinel_ok(
        "HIL on since 2026-11-20 by x@y sandbox=10.1.1.30\n", "rob-box-hil-gazebo"
    )
    assert not gate.hil_sentinel_ok("", "rob-box-hil-gazebo")
    assert not gate.hil_sentinel_ok(
        "HIL on since 2026-11-20 sandbox=10.1.1.30\n", "voice-assistant"
    )
    assert not gate.hil_sentinel_ok("something else\n", "rob-box-hil-gazebo")
    assert not gate.hil_sentinel_ok("HIL on since yesterday\n", "rob-box-hil-gazebo")


def test_verdict_of_counts_real_pairs_not_the_capture_shorthand():
    """Живой прогон судит по парам, не по шапке: 9 совпадений из 15 — FAIL."""
    mux = [(0.1, 0.0)] * 9 + [(0.0, 0.0)] * 6
    verdict, reasons = gate.verdict_of(
        15, 0.1, 0.0, [1.0, 2.0], mux, [], {"joystick_lock": [], "teleop_lock": []}
    )
    assert verdict == gate.FAIL
    assert any(r.startswith("NOT_FORWARDED") for r in reasons)


def test_capture_main_writes_verdict(tmp_path: Path):
    path = tmp_path / "capture.txt"
    path.write_text(
        "\n".join(
            [
                "sent 15 vx 0.1 wz 0.0 seconds 1.5",
                "clock count 40 first 8.000 last 10.000",
                "mux_out count 15 nonzero 15 sample [(0.1, 0.0), (0.1, 0.0), (0.1, 0.0)]",
                "joystick_lock count 3 nonzero 0 sample [False, False, False]",
                "teleop_lock count 3 nonzero 0 sample [False, False, False]",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    assert gate.capture_main(str(path)) == 0
    assert gate.capture_main(str(tmp_path / "нет.txt")) == gate.RC_UNRUN


def _bash_works() -> bool:
    if shutil.which("bash") is None:
        return False
    try:
        return (
            subprocess.run(
                ["bash", "-c", "true"], capture_output=True, timeout=20
            ).returncode
            == 0
        )
    except (OSError, subprocess.TimeoutExpired):
        return False


@pytest.mark.skipif(not _bash_works(), reason="нет рабочего bash")
def test_runvideo_does_not_call_gate_without_flag(tmp_path: Path):
    """Без HIL_MUX_GATE=1 обёртка не копирует гейт и не запускает его. Поведение HIL прежнее."""
    video = tmp_path / "video"
    (video / "out").mkdir(parents=True)
    (video / "route.py").write_text("# stub\n", encoding="utf-8")
    bindir = tmp_path / "bin"
    bindir.mkdir()
    docker = bindir / "docker"
    docker.write_text(
        '#!/usr/bin/env bash\necho "docker $*" >> "${FAKE_DOCKER_LOG:?}"\n'
        'case "$*" in */route.py*) echo "GOAL_START M1";; esac\nexit 0\n',
        encoding="utf-8",
    )
    docker.chmod(0o755)
    env = os.environ.copy()
    env.pop("HIL_MUX_GATE", None)
    env.update(
        {
            "PATH": f"{bindir}{os.pathsep}{env.get('PATH', '')}",
            "VIDEO_DIR": str(video),
            "CAMERAS": "0",
            "SETTLE_S": "0",
            "STOP_S": "0",
            "FAKE_DOCKER_LOG": str(tmp_path / "docker.log"),
        }
    )
    proc = subprocess.run(
        ["bash", str(RUNVIDEO), "M1"],
        capture_output=True,
        text=True,
        encoding="utf-8",
        env=env,
        timeout=30,
    )
    log = (tmp_path / "docker.log").read_text(encoding="utf-8")
    assert proc.returncode == 0, proc.stderr
    assert "mux_gate" not in log
    assert "MUX_GATE" not in proc.stdout
    assert "MUX_GATE" not in proc.stderr
