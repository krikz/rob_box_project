"""Тесты scripts/music/scsynth_block_bench.sh и scsynth_bench_stimulus.py (issue #3114).

Реального Pi/docker здесь нет: ``docker`` и ``sleep`` подменяются fake'ами в
PATH, scsynth — UDP-сервером на Python. Проверяется ЛОГИКА бенча (какие
compose-команды он зовёт, что восстанавливает исходный block size и
voice-assistant, что валит строку при неприменённом -z), а не цифры CPU.
"""
from __future__ import annotations

import importlib.util
import os
import shutil
import socket
import struct
import subprocess
import sys
import threading
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
BENCH = REPO / "scripts/music/scsynth_block_bench.sh"
STIMULUS = REPO / "scripts/music/scsynth_bench_stimulus.py"


def _load_stimulus():
    spec = importlib.util.spec_from_file_location("scsynth_bench_stimulus", STIMULUS)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules["scsynth_bench_stimulus"] = mod
    spec.loader.exec_module(mod)
    return mod


# ── структурные ──────────────────────────────────────────────────────────────

def test_bench_bash_syntax_and_executable() -> None:
    assert os.access(BENCH, os.X_OK)
    r = subprocess.run(["bash", "-n", str(BENCH)], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr


@pytest.mark.skipif(shutil.which("shellcheck") is None, reason="shellcheck не установлен")
def test_bench_shellcheck_clean() -> None:
    r = subprocess.run(["shellcheck", str(BENCH)], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr


def test_bench_help() -> None:
    r = subprocess.run(["bash", str(BENCH), "--help"], capture_output=True, text=True)
    assert r.returncode == 0 and "--sizes" in r.stdout


def test_bench_rejects_bad_size() -> None:
    r = subprocess.run(["bash", str(BENCH), "--sizes", "1024 100"], capture_output=True, text=True)
    assert r.returncode != 0 and "100" in r.stderr


def test_bench_does_not_write_into_repo() -> None:
    text = BENCH.read_text(encoding="utf-8")
    # никаких sed -i / > в файлы репо: размер уходит только env'ом compose
    assert "sed -i" not in text
    assert not any(tok in text for tok in ("> \"$COMPOSE_DIR", ">\"$COMPOSE_DIR", "tee \"$COMPOSE_DIR"))
    assert "trap restore EXIT INT TERM" in text


# ── stimulus ─────────────────────────────────────────────────────────────────

def test_osc_encoding_roundtrip() -> None:
    m = _load_stimulus()
    msg = m.osc_message("/s_new", "tb303", -1, 1, 31140, "bus", 200, "freq", 110.0)
    addr, vals = m.parse_osc(msg)
    assert addr == "/s_new"
    assert vals[:4] == ["tb303", -1, 1, 31140]
    assert vals[4:6] == ["bus", 200] and vals[6] == "freq" and abs(vals[7] - 110.0) < 1e-6
    assert len(m.osc_string("/abc")) == 8  # длина кратна 4 → +4 нуля


def test_schedule_is_deterministic_and_six_players() -> None:
    m = _load_stimulus()
    players = m.DEFAULT_SYNTHS.split(",")
    assert len(players) == 6
    a = m.schedule(players, 120.0, 10.0)
    assert a == m.schedule(players, 120.0, 10.0)
    assert {e[1] for e in a} == set(range(6))
    assert all(0 <= e[0] < 10.0 for e in a)


class _FakeScsynth(threading.Thread):
    """Отвечает на /status и копит остальные адреса."""

    def __init__(self) -> None:
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.2)
        self.port = self.sock.getsockname()[1]
        self.seen: list[tuple[str, list]] = []
        self.stop = False

    def run(self) -> None:
        m = sys.modules["scsynth_bench_stimulus"]
        while not self.stop:
            try:
                data, addr = self.sock.recvfrom(65536)
            except socket.timeout:
                continue
            a, v = m.parse_osc(data)
            self.seen.append((a, v))
            if a == "/status":
                reply = (m.osc_string("/status.reply") + m.osc_string(",iiiiiffdd")
                         + struct.pack(">iiiiiffdd", 1, 50, 7, 3, 6, 12.5, 30.0, 16000.0, 16000.0))
                self.sock.sendto(reply, addr)


def test_stimulus_against_fake_scsynth(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    m = _load_stimulus()
    for name in ("tb303", "organ"):
        (tmp_path / f"{name}.scsyndef").write_bytes(b"SCgf")
    srv = _FakeScsynth()
    srv.start()
    try:
        rc = m.main(["--duration", "2.2", "--synths", "tb303,organ,nope",
                     "--synthdef-dir", str(tmp_path), "--port", str(srv.port)])
    finally:
        srv.stop = True
        srv.join(2)
    out = capsys.readouterr().out
    assert rc == 0, out
    line = [ln for ln in out.splitlines() if ln.startswith("BENCH_RESULT ")][-1]
    import json
    res = json.loads(line.split(" ", 1)[1])
    assert res["players"] == ["tb303", "organ"] and res["missing"] == ["nope"]
    assert res["avg_cpu_mean"] == 12.5 and res["peak_cpu_max"] == 30.0 and res["max_synths"] == 7
    addrs = [a for a, _ in srv.seen]
    assert addrs.count("/d_load") == 2
    assert "/g_new" in addrs and addrs[-1] == "/n_free"
    s_new = [v for a, v in srv.seen if a == "/s_new"]
    assert s_new and all(v[3] == m.BENCH_GROUP for v in s_new)
    assert res["notes_sent"] == len(s_new)


# ── полный прогон бенча на fake docker ────────────────────────────────────────

FAKE_DOCKER = r"""#!/usr/bin/env bash
# fake docker для test_scsynth_block_bench.py
S="$FAKE_STATE"
echo "docker $* | SCSYNTH_BLOCK_SIZE=${SCSYNTH_BLOCK_SIZE-<unset>} IMAGE_TAG=${IMAGE_TAG-} PREFIX=${SERVICE_IMAGE_PREFIX-}" >> "$S/calls.log"
case "$1" in
  compose)
    shift
    case "$1" in
      version) echo "Docker Compose version v2.fake"; exit 0 ;;
      up) echo "${SCSYNTH_BLOCK_SIZE:-1024}" > "$S/block"
          [ -n "${FAKE_BROKEN_Z:-}" ] && echo 1024 > "$S/block"
          exit 0 ;;
      stop) echo false > "$S/va_running"; exit 0 ;;
      start) echo true > "$S/va_running"; exit 0 ;;
    esac ;;
  inspect)
    fmt="$3"; name="$4"
    case "$fmt" in
      *Config.Image*) echo "ghcr.io/krikz/rob_box:supercollider-test-abc1234" ;;
      *Config.Env*) printf 'PATH=/usr/bin\nSCSYNTH_BLOCK_SIZE=1024\n' ;;
      *State.Running*) if [ "$name" = voice-assistant ]; then cat "$S/va_running"; else echo true; fi ;;
      *) : ;;
    esac
    exit 0 ;;
  logs) echo "[jackd] Jack: JackEngine::XRun: client = jack was not finished"; exit 0 ;;
  exec)
    shift
    [ "$1" = "-i" ] && { shift; cat > /dev/null
      echo 'BENCH_RESULT {"players":["tb303"],"missing":[],"notes_sent":42,"status_samples":5,"avg_cpu_mean":9.5,"peak_cpu_max":20.1,"max_synths":4}'
      exit 0; }
    shift  # container
    case "$*" in
      "pgrep -x scsynth") echo 42 ;;
      "pgrep -x jackd") echo 7 ;;
      "cat /proc/42/cmdline") printf '%s\0' scsynth -u 57110 -z "$(cat "$S/block")" -H jack ;;
      cat\ /proc/*/stat) n=$(( $(cat "$S/tick" 2>/dev/null || echo 0) + 50 )); echo $n > "$S/tick"
                         echo "42 (scsynth) S 1 1 1 0 -1 0 0 0 0 0 $n $n 0 0" ;;
      "getconf CLK_TCK") echo 100 ;;
      sh\ -c*) exit 0 ;;
    esac
    exit 0 ;;
esac
exit 0
"""


def _sandbox(tmp_path: Path) -> tuple[dict, Path]:
    bindir = tmp_path / "bin"
    state = tmp_path / "state"
    bindir.mkdir()
    state.mkdir()
    (state / "va_running").write_text("true\n")
    (bindir / "docker").write_text(FAKE_DOCKER)
    (bindir / "docker").chmod(0o755)
    (bindir / "sleep").write_text("#!/bin/sh\nexit 0\n")
    (bindir / "sleep").chmod(0o755)
    env = {
        "PATH": f"{bindir}:/usr/bin:/bin",
        "FAKE_STATE": str(state),
        "HOME": str(tmp_path),
    }
    return env, state


def test_bench_full_flow_restores_state(tmp_path: Path) -> None:
    env, state = _sandbox(tmp_path)
    out_tsv = tmp_path / "res.tsv"
    r = subprocess.run(
        ["bash", str(BENCH), "--sizes", "1024 256 64", "--duration", "10", "--idle", "1",
         "--settle", "0", "--out", str(out_tsv)],
        capture_output=True, text=True, env=env, timeout=120,
    )
    assert r.returncode == 0, r.stdout + r.stderr
    calls = (state / "calls.log").read_text()
    ups = [ln for ln in calls.splitlines() if "compose up" in ln]
    assert [ln.split("SCSYNTH_BLOCK_SIZE=")[1].split()[0] for ln in ups] == ["1024", "256", "64", "1024"], ups
    assert all("--pull never" in ln and "--no-deps" in ln and "IMAGE_TAG=test-abc1234" in ln
               and "PREFIX=ghcr.io/krikz/rob_box" in ln for ln in ups), ups
    assert "compose stop voice-assistant" in calls and "compose start voice-assistant" in calls
    assert (state / "va_running").read_text().strip() == "true"
    rows = out_tsv.read_text().splitlines()
    assert rows[0].startswith("block\tctl_Hz")
    body = [row.split("\t") for row in rows[1:]]
    assert [b[0] for b in body] == ["1024", "256", "64"]
    assert [b[1] for b in body] == ["15.6", "62.5", "250.0"]
    assert all(b[2] == b[0] and b[-1] == "ok" and b[11] == "1" for b in body), body


def test_bench_flags_z_not_applied(tmp_path: Path) -> None:
    env, state = _sandbox(tmp_path)
    env["FAKE_BROKEN_Z"] = "1"
    out_tsv = tmp_path / "res.tsv"
    r = subprocess.run(
        ["bash", str(BENCH), "--sizes", "256", "--duration", "10", "--idle", "1",
         "--settle", "0", "--out", str(out_tsv)],
        capture_output=True, text=True, env=env, timeout=60,
    )
    assert r.returncode == 0, r.stderr
    row = out_tsv.read_text().splitlines()[1].split("\t")
    assert row[2] == "1024" and row[-1].startswith("FAIL:-z не применился"), row


def test_bench_restores_on_failure(tmp_path: Path) -> None:
    """Если скрипт упал посреди прогона — trap всё равно возвращает 1024 и voice."""
    env, state = _sandbox(tmp_path)
    # Ломаем python3 (json_field) → set -e роняет скрипт после первого прогона.
    (Path(env["PATH"].split(":")[0]) / "python3").write_text("#!/bin/sh\nexit 3\n")
    (Path(env["PATH"].split(":")[0]) / "python3").chmod(0o755)
    r = subprocess.run(
        ["bash", str(BENCH), "--sizes", "256", "--duration", "10", "--idle", "1", "--settle", "0"],
        capture_output=True, text=True, env=env, timeout=60,
    )
    assert r.returncode != 0
    calls = (state / "calls.log").read_text()
    ups = [ln for ln in calls.splitlines() if "compose up" in ln]
    assert ups[-1].split("SCSYNTH_BLOCK_SIZE=")[1].split()[0] == "1024", ups
    assert (state / "va_running").read_text().strip() == "true"
