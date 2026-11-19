#!/usr/bin/env python3
"""Один сэмпл ресурсов HIL-прогона, 1 Гц. Не ros2, не docker stats, не cadvisor.

Песочница и Main Pi зовут один и тот же разбор. Нет файла — поле ``missing``,
процесс не падает. nvidia-smi не используем: Gazebo на Intel iGPU.

Строка stdout — одна, с UTC, чтобы Тигр свёл её с /clock и Failed to make progress.
"""

from __future__ import annotations

import argparse
import datetime
import sys
import time
from pathlib import Path
from typing import List, Optional, Sequence


def utc_now() -> str:
    return datetime.datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")


def read_text(path: Path) -> Optional[str]:
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None


def parse_proc_stat(text: str) -> str:
    """Сумма cpu и каждая cpuN: user nice system idle iowait. Нет строки — missing."""
    if not text:
        return "cpu=missing"
    parts: List[str] = []
    for line in text.splitlines():
        if not line.startswith("cpu"):
            continue
        fields = line.split()
        name, nums = fields[0], fields[1:]
        if len(nums) < 4:
            continue
        parts.append(name + ":" + ",".join(nums[:7]))
    return "cpu=" + (";".join(parts) if parts else "missing")


def parse_meminfo(text: str) -> str:
    """MemTotal и MemAvailable, кБ. Нет ключа — missing, ноль не подставляем."""
    if not text:
        return "mem_total_kb=missing mem_available_kb=missing"
    found = {}
    for line in text.splitlines():
        if ":" not in line:
            continue
        key, _, rest = line.partition(":")
        num = rest.strip().split()
        if key in ("MemTotal", "MemAvailable") and num and num[0].isdigit():
            found[key] = num[0]
    total = found.get("MemTotal", "missing")
    avail = found.get("MemAvailable", "missing")
    return f"mem_total_kb={total} mem_available_kb={avail}"


def parse_pressure_cpu(text: str) -> str:
    """Строка some из /proc/pressure/cpu. Нет файла — missing."""
    if not text:
        return "psi_cpu=missing"
    for line in text.splitlines():
        if line.startswith("some "):
            return "psi_cpu=" + line[len("some "):].replace(" ", ",")
    return "psi_cpu=missing"


def parse_diskstats(text: str, dev: str) -> str:
    """Одна строка diskstats устройства: reads, writes, io_ticks. Нет — missing."""
    if not text or not dev:
        return "disk=missing"
    for line in text.splitlines():
        fields = line.split()
        if len(fields) < 14 or fields[2] != dev:
            continue
        # reads, read_ms, writes, write_ms, io_in_progress, io_ms
        return "disk={0}:r={1},rms={2},w={3},wms={4},io={5},iot={6}".format(
            dev, fields[3], fields[6], fields[7], fields[10], fields[11], fields[12]
        )
    return "disk=missing"


def parse_gpu(sysfs_text: Optional[str]) -> str:
    """Intel iGPU: сырьё gt_cur_freq_mhz или gt_busy_percent. nvidia-smi не зовём."""
    if sysfs_text is None:
        return "gpu=missing"
    text = sysfs_text.strip()
    if not text:
        return "gpu=missing"
    return "gpu_mhz=" + text.split()[0]


def format_sample(
    host: str,
    stat: str,
    mem: str,
    pressure: str,
    disk: str,
    gpu: str,
    docker: str = "",
    when: str = "",
) -> str:
    """Одна строка. Пустой docker — поле не пишем (песочница)."""
    stamp = when or utc_now()
    fields = [
        f"utc={stamp}",
        f"host={host}",
        parse_proc_stat(stat),
        parse_meminfo(mem),
        parse_pressure_cpu(pressure),
        parse_diskstats(disk, "sda") if "\n" in (disk or "") or (disk or "").startswith(" ") else disk,
        gpu if gpu.startswith("gpu") else parse_gpu(gpu),
    ]
    if docker:
        fields.append(docker if docker.startswith("docker=") else f"docker={docker}")
    return " ".join(fields)


def sample_host(root: Path, host: str, disk_dev: str, gpu_path: Optional[Path]) -> str:
    """Считать /proc под ``root`` (в тесте — временный каталог, на хосте — /)."""
    stat = read_text(root / "proc/stat") or ""
    mem = read_text(root / "proc/meminfo") or ""
    pressure = read_text(root / "proc/pressure/cpu") or ""
    disk = read_text(root / "proc/diskstats") or ""
    gpu_text = read_text(gpu_path) if gpu_path is not None else None
    disk_field = parse_diskstats(disk, disk_dev)
    # format_sample сам разберёт stat/mem/pressure; disk и gpu уже готовы.
    return " ".join(
        [
            f"utc={utc_now()}",
            f"host={host}",
            parse_proc_stat(stat),
            parse_meminfo(mem),
            parse_pressure_cpu(pressure),
            disk_field,
            parse_gpu(gpu_text),
        ]
    )


def _loop(argv: Sequence[str]) -> int:
    """Писать одну строку в --out каждую секунду, пока процесс жив. 1 Гц, без docker."""
    p = argparse.ArgumentParser()
    p.add_argument("--host", required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--root", type=Path, default=Path("/"))
    p.add_argument("--disk", default="sda")
    p.add_argument("--gpu", type=Path, default=None)
    ns = p.parse_args(list(argv))
    while True:
        line = sample_host(ns.root, ns.host, ns.disk, ns.gpu)
        with ns.out.open("a", encoding="utf-8") as fh:
            fh.write(line + "\n")
        time.sleep(1)


def main(argv: Sequence[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    # Скрипт зовёт ``resource_sample.py --loop --host … --out …``.
    # Раньше --loop принимался только первым аргументом — argparse отвечал
    # unrecognized arguments, и лог ресурсов был пуст (пара run1).
    if "--loop" in args:
        rest = [a for a in args if a != "--loop"]
        return _loop(rest)
    p = argparse.ArgumentParser(description=__doc__.split("\n\n", 1)[0])
    p.add_argument("--host", required=True)
    p.add_argument("--root", type=Path, default=Path("/"))
    p.add_argument("--disk", default="sda")
    p.add_argument("--gpu", type=Path, default=None)
    ns = p.parse_args(args)
    print(sample_host(ns.root, ns.host, ns.disk, ns.gpu))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
