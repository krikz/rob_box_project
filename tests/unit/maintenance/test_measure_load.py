"""test_measure_load.py — контракт ``scripts/maintenance/measure_load.sh`` (ADR-0137 §2.4).

Проверяется синтаксис и формат CSV на поддельном ``docker`` в PATH (без
реального Docker и без Hailo). Реальная загрузка Pi здесь НЕ измеряется.

Run:
  python -m pytest tests/unit/maintenance/test_measure_load.py -v -p no:cacheprovider --no-cov -o addopts=""
"""

from __future__ import annotations

import csv
import os
import shutil
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = REPO_ROOT / "scripts" / "maintenance" / "measure_load.sh"
HEADER = ["ts", "host", "kind", "name", "cpu_pct", "mem_usage", "mem_pct", "value"]


def test_bash_syntax_clean():
    res = subprocess.run(["bash", "-n", str(SCRIPT)], capture_output=True, text=True, timeout=10)
    assert res.returncode == 0, res.stderr


def test_shellcheck_clean():
    shellcheck = shutil.which("shellcheck")
    if shellcheck is None:
        pytest.skip("shellcheck not installed locally")
    res = subprocess.run([shellcheck, str(SCRIPT)], capture_output=True, text=True, timeout=30)
    assert res.returncode == 0, res.stdout + res.stderr


def _fake_bin(tmp_path: Path, docker_has_hailortcli: bool) -> Path:
    bindir = tmp_path / "bin"
    bindir.mkdir()
    exec_rc = 0 if docker_has_hailortcli else 1
    (bindir / "docker").write_text(
        "#!/bin/bash\n"
        'case "$1" in\n'
        "  info) exit 0 ;;\n"
        "  stats) echo 'vision-hailo|37.5%|512MiB / 7.8GiB|6.4%';"
        " echo 'voice-assistant|120.25%|1.2GiB / 7.8GiB|15.1%' ;;\n"
        '  exec) if [[ "$*" == *monitor* ]]; then echo "fake monitor frame"; exit 0; fi;'
        f" exit {exec_rc} ;;\n"
        "esac\n",
        encoding="utf-8",
    )
    (bindir / "docker").chmod(0o755)
    return bindir


def _run(tmp_path: Path, bindir: Path):
    env = dict(os.environ)
    env["PATH"] = f"{bindir}:{env['PATH']}"
    out = tmp_path / "out"
    res = subprocess.run(
        ["bash", str(SCRIPT), "--duration", "1", "--interval", "1", "--out-dir", str(out)],
        capture_output=True, text=True, timeout=60, env=env,
    )
    csvs = sorted(out.glob("load_*.csv"))
    return res, csvs


def _rows(path: Path):
    with path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    assert rows[0] == HEADER
    return rows[1:]


def test_csv_with_containers_and_no_npu(tmp_path):
    if shutil.which("hailortcli"):
        pytest.skip("на этой машине есть hailortcli — ветка 'NPU unavailable' недостижима")
    res, csvs = _run(tmp_path, _fake_bin(tmp_path, docker_has_hailortcli=False))
    assert res.returncode == 0, res.stdout + res.stderr
    assert len(csvs) == 1
    rows = _rows(csvs[0])
    assert all(len(r) == len(HEADER) for r in rows)
    kinds = {r[2] for r in rows}
    assert {"host_cpu_pct", "host_load1", "container", "npu"} <= kinds
    containers = {r[3]: r for r in rows if r[2] == "container"}
    assert containers["voice-assistant"][4] == "120.25"  # % снят, 100 = одно ядро
    assert containers["vision-hailo"][6] == "6.4"
    npu = [r for r in rows if r[2] == "npu"]
    assert npu[0][7].startswith("NPU metric unavailable")
    assert "СВОДКА" in res.stdout


def test_npu_raw_snapshot_when_hailortcli_in_container(tmp_path):
    if shutil.which("hailortcli"):
        pytest.skip("на этой машине есть hailortcli на хосте")
    res, csvs = _run(tmp_path, _fake_bin(tmp_path, docker_has_hailortcli=True))
    assert res.returncode == 0, res.stdout + res.stderr
    npu = [r for r in _rows(csvs[0]) if r[2] == "npu"]
    assert npu[0][3] == "hailortcli"
    raw = Path(npu[0][7].removeprefix("raw:"))
    assert raw.read_text(encoding="utf-8").strip() == "fake monitor frame"


def test_rejects_non_integer_duration(tmp_path):
    res = subprocess.run(
        ["bash", str(SCRIPT), "--duration", "1.5", "--out-dir", str(tmp_path)],
        capture_output=True, text=True, timeout=10,
    )
    assert res.returncode == 1
