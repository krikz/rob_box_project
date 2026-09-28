"""Контракт ``scripts/music/live_check_3115.sh`` (issue #3115).

Здесь НЕ проверяется, что музыка на роботе звучит, — это живой прогон у
Шифу. Тесты закрепляют, что одна команда покрывает все 5 пунктов #3115,
раскладывает доказательства по ожидаемым путям, в ``--dry-run`` ничего не
трогает (фальшивый ``docker`` в PATH ни разу не вызван), а механические
проверки (``live_check_3115_assert.py``) отличают PASS от FAIL.
"""
from __future__ import annotations

import base64
import json
import math
import os
import re
import shutil
import struct
import subprocess
import wave
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = REPO_ROOT / "scripts" / "music" / "live_check_3115.sh"
ASSERT = REPO_ROOT / "scripts" / "music" / "live_check_3115_assert.py"
HELPER = REPO_ROOT / "scripts" / "music" / "live_check_mcp_call.py"
REFERENCE = (
    REPO_ROOT / "src/rob_box_mcp_tools/rob_box_mcp_tools/core/reference_tracks/by_design_dj_dave.foxdot"
)
GOLDEN = REPO_ROOT / "src/rob_box_mcp_tools/test/fixtures/arranger_golden.json"


def _fake_docker_path(tmp_path: Path) -> tuple[dict, Path]:
    """PATH с фальшивым docker/ssh/scp, пишущими маркер при вызове."""
    bindir = tmp_path / "bin"
    bindir.mkdir()
    marker = tmp_path / "called.txt"
    for name in ("docker", "ssh", "scp", "sshpass"):
        stub = bindir / name
        stub.write_text(f'#!/bin/sh\necho "{name} $*" >> "{marker}"\n', encoding="utf-8")
        stub.chmod(0o755)
    env = dict(os.environ, PATH=f"{bindir}{os.pathsep}{os.environ['PATH']}")
    return env, marker


def _dry_run(tmp_path: Path, *args: str) -> tuple[subprocess.CompletedProcess, Path, Path]:
    env, marker = _fake_docker_path(tmp_path)
    out = tmp_path / "evidence"
    proc = subprocess.run(
        ["bash", str(SCRIPT), "--dry-run", "--out", str(out), *args],
        capture_output=True, text=True, timeout=60, env=env, cwd=tmp_path,
    )
    return proc, out, marker


def _mcp_params(stdout: str, tool: str) -> list:
    """Декодировать все base64-параметры вызовов ``tool`` из dry-run вывода."""
    found = re.findall(rf"python3 - {tool} (\S+) \d+'", stdout)
    return [json.loads(base64.b64decode(b).decode("utf-8")) for b in found]


# --------------------------------------------------------------------------- #
# Синтаксис
# --------------------------------------------------------------------------- #


def test_files_exist_and_executable() -> None:
    for f in (SCRIPT, ASSERT, HELPER):
        assert f.exists(), f
        assert os.access(f, os.X_OK), f"{f} не исполняемый"


def test_bash_syntax_clean() -> None:
    r = subprocess.run(["bash", "-n", str(SCRIPT)], capture_output=True, text=True, timeout=10)
    assert r.returncode == 0, r.stderr


def test_shellcheck_clean() -> None:
    if shutil.which("shellcheck") is None:
        pytest.skip("shellcheck не установлен")
    r = subprocess.run(["shellcheck", str(SCRIPT)], capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stdout + r.stderr


def test_python_helpers_compile() -> None:
    for f in (ASSERT, HELPER):
        r = subprocess.run(["python3", "-m", "py_compile", str(f)], capture_output=True, text=True)
        assert r.returncode == 0, r.stderr


# --------------------------------------------------------------------------- #
# --dry-run: все 5 пунктов, раскладка доказательств, без побочных эффектов
# --------------------------------------------------------------------------- #


def test_dry_run_covers_all_five_checks(tmp_path: Path) -> None:
    proc, out, marker = _dry_run(tmp_path)
    assert proc.returncode == 0, proc.stderr
    so = proc.stdout
    for header in ("## п.1 fuzz", "## п.2-3 compose_music(name=tetris)", "## п.4 compose_music(style=club)", "## п.5 эталон"):
        assert header in so, header
    for item in range(1, 6):
        assert f"[assert п.{item}]" in so, f"нет механической проверки п.{item}"
    # п.1: редирект #3008 ищется в sclang.log
    assert "grep [#3008] /foxdot" in so and "sclang.log" in so
    # п.2/п.3: dur у d1/d2 и amplify-список
    assert " drums " in so and " melody " in so
    # п.4/п.5: лог supercollider на not found / FAILURE
    assert so.count(" logerr ") >= 3


def test_dry_run_mcp_calls_decode_to_expected_params(tmp_path: Path) -> None:
    proc, _, _ = _dry_run(tmp_path)
    so = proc.stdout
    compose = _mcp_params(so, "compose_music")
    assert {"name": "tetris"} in compose
    assert {"style": "club"} in compose
    execute = _mcp_params(so, "execute_music_code")
    assert len(execute) == 2
    fuzz = execute[0]
    assert re.search(r"\bfuzz\(", fuzz["code"]) and "oct=0" in fuzz["code"]
    # эталон уходит без искажений (кавычки, кириллица, переводы строк)
    assert execute[1]["code"] == REFERENCE.read_text(encoding="utf-8")
    # между пунктами музыка гасится
    assert len(_mcp_params(so, "stop_music")) == 8


def test_dry_run_records_every_check_via_jack(tmp_path: Path) -> None:
    proc, _, _ = _dry_run(tmp_path)
    recs = re.findall(r"DRY: docker exec supercollider jack_rec -f /tmp/(\S+) -d (\d+) -b 16 jack:out_1 jack:out_2 &", proc.stdout)
    # 10/15/30/30 с + запас REC_PAD=5
    assert recs == [
        ("fuzz_oct0.wav", "15"), ("classic_tetris.wav", "20"), ("club.wav", "35"), ("reference_by_design.wav", "35"),
    ]


def test_dry_run_evidence_layout(tmp_path: Path) -> None:
    proc, out, _ = _dry_run(tmp_path)
    so = proc.stdout
    for sub, wav in (
        ("check1_fuzz", "fuzz_oct0.wav"),
        ("check2_3_classic_tetris", "classic_tetris.wav"),
        ("check4_club", "club.wav"),
        ("check5_reference", "reference_by_design.wav"),
    ):
        d = f"{out}/{sub}"
        assert f"docker cp supercollider:/tmp/{wav} '{d}/{wav}'" in so
        for c in ("voice-assistant", "supercollider"):
            assert f"docker logs --tail 50 {c} > '{d}/{c}.tail50.log'" in so
        assert f"docker cp voice-assistant:/tmp/sclang.log '{d}/sclang.log'" in so
        assert f"{d}/mcp_result.json" in so
        assert f"{d}/code.foxdot" in so
    assert f"{out}/summary.md" in so


def test_dry_run_has_no_side_effects(tmp_path: Path) -> None:
    proc, out, marker = _dry_run(tmp_path)
    assert proc.returncode == 0
    assert not marker.exists(), marker.read_text() if marker.exists() else ""
    assert not out.exists(), "dry-run не должен создавать каталог доказательств"


def test_dry_run_only_subset(tmp_path: Path) -> None:
    proc, _, _ = _dry_run(tmp_path, "--only", "4")
    assert "## п.4" in proc.stdout
    assert "## п.1" not in proc.stdout and "## п.5" not in proc.stdout


def test_dry_run_ssh_mode_bundles_and_fetches(tmp_path: Path) -> None:
    proc, _, marker = _dry_run(tmp_path, "--host", "10.1.1.21")
    so = proc.stdout
    assert proc.returncode == 0, proc.stderr
    assert "ros2@10.1.1.21 mkdir -p /tmp/live_check_3115_bundle" in so
    assert "live_check_mcp_call.py" in so and "live_check_3115_assert.py" in so and "by_design_dj_dave.foxdot" in so
    assert "--local --out /tmp/live_check_3115_bundle/evidence" in so
    assert "scp -r" in so and "ros2@10.1.1.21:/tmp/live_check_3115_bundle/evidence" in so
    assert "## п.5" in so  # удалённый план тоже напечатан
    assert not marker.exists()


FAKE_DOCKER = r'''#!/usr/bin/env python3
"""Фальшивый docker для прогона НЕ-dry пути без робота (только механика)."""
import base64, json, math, os, re, struct, sys, wave
a = sys.argv[1:]
with open(os.environ["FAKE_DOCKER_LOG"], "a", encoding="utf-8") as fh:
    fh.write(" ".join(a) + "\n")
golden = os.environ["FAKE_GOLDEN_CODE"]
silent = os.environ.get("FAKE_SILENT") == "1"


def wav(path):
    amp = 0.0 if silent else 0.3
    with wave.open(path, "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(16000)
        w.writeframes(b"".join(struct.pack("<hh", v, v) for v in
                               (int(amp * 32767 * math.sin(i / 10.0)) for i in range(16000))))


if a[0] == "exec" and a[1] == "-i":
    m = re.search(r"python3 - (\S+) (\S+) \d+", a[-1])
    tool, params = m.group(1), json.loads(base64.b64decode(m.group(2)))
    sys.stdin.read()
    data = None
    if tool == "execute_music_code":
        data = {"code": params["code"]}
    elif tool == "compose_music":
        data = {"code": golden if params.get("name") else "d1 >> play('X', dur=1/4)"}
    print("login-shell noise")
    print(json.dumps({"tool_name": tool, "request_id": "r", "result": {"success": True, "data": data}}))
elif a[0] == "exec" and "command -v" in a[-1]:
    print("/usr/bin/jack_rec")
elif a[0] == "exec" and a[2] == "jack_lsp":
    print("jack:out_1")
    print("jack:out_2")
elif a[0] == "exec" and a[2] == "grep":
    print("1")
elif a[0] == "cp" and a[1].endswith(".wav"):
    wav(a[2])
elif a[0] == "cp" and a[1].endswith("sclang.log"):
    with open(a[2], "w", encoding="utf-8") as fh:
        fh.write("SynthDef preload finished: 60\n"
                 "[#3008] /foxdot /r/tmp_code/scsynth/fuzz.scd -> /r/scsynth/fuzz.scd\n")
elif a[0] == "logs":
    print("[SuperCollider] JACK running")
elif a[0] == "inspect":
    print("/voice-assistant image=fake")
'''


def _full_run(tmp_path: Path, silent: bool = False) -> tuple[subprocess.CompletedProcess, Path, Path]:
    bindir = tmp_path / "bin"
    bindir.mkdir()
    docker = bindir / "docker"
    docker.write_text(FAKE_DOCKER, encoding="utf-8")
    docker.chmod(0o755)
    golden = json.loads(GOLDEN.read_text(encoding="utf-8"))["cases"][0]["code"]
    log = tmp_path / "docker.log"
    env = dict(
        os.environ,
        PATH=f"{bindir}{os.pathsep}{os.environ['PATH']}",
        FAKE_DOCKER_LOG=str(log),
        FAKE_GOLDEN_CODE=golden,
        FAKE_SILENT="1" if silent else "0",
        REC_FUZZ="0", REC_CLASSIC="0", REC_CLUB="0", REC_REFERENCE="0", REC_PAD="0",
    )
    out = tmp_path / "evidence"
    proc = subprocess.run(
        ["bash", str(SCRIPT), "--out", str(out)],
        capture_output=True, text=True, timeout=120, env=env, cwd=tmp_path,
    )
    return proc, out, log


def test_full_run_with_fake_docker_writes_evidence_and_summary(tmp_path: Path) -> None:
    """НЕ-dry путь на фальшивом docker: раскладка и summary.md собираются.

    Это НЕ живая проверка: звук синтетический, ответы MCP подставлены.
    """
    proc, out, _ = _full_run(tmp_path)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    summary = (out / "summary.md").read_text(encoding="utf-8")
    assert "0 FAIL" in summary, summary
    assert summary.count("НА СЛУХ: требуется вердикт Шифу") == 5
    for sub, wav_name in (
        ("check1_fuzz", "fuzz_oct0.wav"),
        ("check2_3_classic_tetris", "classic_tetris.wav"),
        ("check4_club", "club.wav"),
        ("check5_reference", "reference_by_design.wav"),
    ):
        d = out / sub
        for f in (wav_name, "code.foxdot", "mcp_result.json", "mcp_result.request.json", "sclang.log",
                  "voice-assistant.tail50.log", "supercollider.tail50.log"):
            assert (d / f).exists(), d / f
    for f in ("commands.log", "env.txt", "jack_lsp.txt"):
        assert (out / f).exists(), f
    ref = (out / "check5_reference" / "code.foxdot").read_text(encoding="utf-8")
    assert ref == REFERENCE.read_text(encoding="utf-8")


def test_full_run_silence_is_fail_not_pass(tmp_path: Path) -> None:
    proc, out, _ = _full_run(tmp_path, silent=True)
    assert proc.returncode != 0
    summary = (out / "summary.md").read_text(encoding="utf-8")
    assert "4 FAIL" in summary and "тишина" in summary, summary


# --------------------------------------------------------------------------- #
# Механические проверки
# --------------------------------------------------------------------------- #


def _assert(*args: str) -> tuple[int, str]:
    r = subprocess.run(["python3", str(ASSERT), *args], capture_output=True, text=True, timeout=30)
    return r.returncode, r.stdout.strip()


@pytest.fixture()
def golden_code(tmp_path: Path) -> Path:
    case = json.loads(GOLDEN.read_text(encoding="utf-8"))["cases"][0]
    p = tmp_path / "golden.foxdot"
    p.write_text(case["code"], encoding="utf-8")
    return p


def test_drums_and_melody_pass_on_arranger_output(golden_code: Path) -> None:
    rc, out = _assert("drums", str(golden_code))
    assert rc == 0 and out.startswith("PASS"), out
    rc, out = _assert("melody", str(golden_code))
    assert rc == 0 and "p2" in out, out


def test_drums_fail_without_sixteenths(tmp_path: Path) -> None:
    p = tmp_path / "c.foxdot"
    p.write_text("d1 >> play('X.o.', amp=0.5)\nd2 >> play('-.-.', dur=0.25)\n", encoding="utf-8")
    rc, out = _assert("drums", str(p))
    assert rc == 1 and "d1" in out and "d2" not in out.split("FAIL:")[1], out


def test_drums_accepts_quarter_fraction_and_multiline(tmp_path: Path) -> None:
    p = tmp_path / "c.foxdot"
    p.write_text("d1 >> play('X..X',\n    dur=1/4)\nd2 >> play('--', dur=0.25)\n", encoding="utf-8")
    assert _assert("drums", str(p))[0] == 0


def test_melody_fail_when_amplify_is_var(tmp_path: Path) -> None:
    p = tmp_path / "c.foxdot"
    p.write_text("p1 >> bass([0], amplify=var([1, 0.3], [1, 1]))\n", encoding="utf-8")
    assert _assert("melody", str(p))[0] == 1


def test_code_extracted_from_mcp_result(tmp_path: Path) -> None:
    res = tmp_path / "r.json"
    res.write_text(
        "noise from login shell\n"
        + json.dumps({"tool_name": "compose_music", "request_id": "x", "result": {"success": True, "data": {"code": "d1 >> play('X')"}}}),
        encoding="utf-8",
    )
    out_code = tmp_path / "code.foxdot"
    rc, out = _assert("code", str(res), str(out_code))
    assert rc == 0, out
    assert out_code.read_text(encoding="utf-8") == "d1 >> play('X')"


def test_code_fails_on_tool_error_or_timeout(tmp_path: Path) -> None:
    res = tmp_path / "r.json"
    res.write_text(json.dumps({"result": {"success": False, "error": "SynthDef not found"}}), encoding="utf-8")
    rc, out = _assert("code", str(res), str(tmp_path / "c"))
    assert rc == 1 and "SynthDef not found" in out
    res.write_text(json.dumps({"error": "таймаут 90s"}), encoding="utf-8")
    assert _assert("code", str(res), str(tmp_path / "c"))[0] == 1


def _write_wav(path: Path, amp: float, seconds: float = 1.0, rate: int = 16000) -> None:
    with wave.open(str(path), "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(rate)
        frames = b"".join(
            struct.pack("<hh", v, v)
            for v in (int(amp * 32767 * math.sin(2 * math.pi * 220 * i / rate)) for i in range(int(rate * seconds)))
        )
        w.writeframes(frames)


def test_wav_sound_vs_silence(tmp_path: Path) -> None:
    loud, quiet = tmp_path / "loud.wav", tmp_path / "quiet.wav"
    _write_wav(loud, 0.3)
    _write_wav(quiet, 0.0)
    rc, out = _assert("wav", str(loud))
    assert rc == 0 and "16000Hz 2ch" in out, out
    rc, out = _assert("wav", str(quiet))
    assert rc == 1 and "тишина" in out, out
    assert _assert("wav", str(tmp_path / "missing.wav"))[0] == 1


def test_logerr_and_grep(tmp_path: Path) -> None:
    clean, dirty = tmp_path / "clean.log", tmp_path / "dirty.log"
    clean.write_text("[SuperCollider] JACK running\n", encoding="utf-8")
    dirty.write_text("ok\nFAILURE IN SERVER /s_new SynthDef not found\n", encoding="utf-8")
    assert _assert("logerr", str(clean))[0] == 0
    rc, out = _assert("logerr", str(dirty))
    assert rc == 1 and "FAILURE IN SERVER" in out
    sclang = tmp_path / "sclang.log"
    sclang.write_text("[#3008] /foxdot /x/tmp_code/scsynth/fuzz.scd -> /y/scsynth/fuzz.scd\n", encoding="utf-8")
    assert _assert("grep", "[#3008] /foxdot", str(sclang))[0] == 0
    assert _assert("absent", "ERROR: syntax error", str(sclang))[0] == 0
    assert _assert("grep", "[#3008] /foxdot", str(clean))[0] == 1
