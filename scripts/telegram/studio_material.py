"""Материал для роликов студии: что есть на хостах и как его забрать через гейт только на чтение.

Решение Шифу 06.11 (#4513): «видео оно должно собирать само и любое, которое мы обсудим». Студия берёт
записи прогонов HIL с песочницы ключом ``studio_ro`` (forced command ``studio_ro_gate.sh``, runbook
``docs/runbooks/studio-ro-access.md``): ``ls`` и ``scp -O -f`` одного файла. Ничего не пишет на хосты.

Что такое «прогон» здесь — каталог в ``/home/ros2/rob_box_hilvideo/out`` с ``maprec/poses.csv`` (правда Gazebo
и оценка rtabmap по времени). Подписи известных прогонов задаёт код (``KNOWN_RUNS``): «до» и «после» —
это прогоны дрейфа локализации на складе Depot (#4495). Остальные прогоны попадают в каталог под своим именем.

Скачанное кладётся в кэш ``run/media/material/<прогон>/``: записи прогонов не меняются, второй ролик не качает
их заново.
"""

from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path
from typing import Callable, Optional

SANDBOX = os.environ.get("STUDIO_HIL_HOST", "ros2@10.1.1.250")
HIL_OUT = os.environ.get("STUDIO_HIL_OUT", "/home/ros2/rob_box_hilvideo/out")
KEY = Path(os.environ.get("STUDIO_RO_KEY", str(Path.home() / ".ssh" / "studio_ro")))
SSH_OPTS = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "-o", "StrictHostKeyChecking=accept-new"]
MAP_SAMPLES = 12  # снимков карты на прогон: подложка меняется по ходу ролика, а качать все 400 — долго
TOP_RUN = "depot"  # корень out/ — прогон с кадрами камер (вид сверху, compose.py)
NAME_RE = re.compile(r"^[A-Za-z0-9_.-]{1,64}$")

# Подписи прогонов, которые уже обсуждали (#4495, дрейф локализации). Код, а не модель, знает, что есть «до».
KNOWN_RUNS = {
    "shisyun_s1": {"label": "ДО: прогон s1", "topic": "drift", "role": "before"},
    "shisyun_s2": {"label": "ПОСЛЕ: прогон s2", "topic": "drift", "role": "after"},
}
TOPICS = {
    "drift": {
        "title": "Дрейф локализации на складе Depot (HIL, #4495)",
        "markers": ("#4495", "rtabmap", "локализац", "дрейф", "depot", "icp", "поза уплыва", "уплыв"),
    }
}


class Gate:
    """ssh/scp к хосту ключом студии. runner подменяется в тестах (сети и ssh там нет)."""

    def __init__(self, host: str = SANDBOX, key: Path = KEY, runner: Callable = subprocess.run, timeout: int = 60):
        self.host, self.key, self.runner, self.timeout = host, Path(key), runner, timeout
        self.calls = 0

    def _run(self, cmd: list) -> tuple:
        self.calls += 1
        try:
            done = self.runner(cmd, capture_output=True, text=True, timeout=self.timeout, check=False)
        except (OSError, subprocess.TimeoutExpired) as exc:
            return False, "", f"{type(exc).__name__}"
        if done.returncode != 0:
            return False, done.stdout or "", (done.stderr or "").strip()[:200] or f"код {done.returncode}"
        return True, done.stdout or "", ""

    def ls(self, path: str) -> tuple:
        """Имена в каталоге: (список, причина при неудаче)."""
        ok, out, err = self._run(["ssh", "-i", str(self.key), *SSH_OPTS, self.host, f"ls -1 {path}"])
        if not ok:
            return [], f"ls {path}: {err}"
        return [line.strip() for line in out.splitlines() if line.strip()], ""

    def fetch(self, remote: str, local: Path) -> str:
        """Один файл через legacy scp (-O обязателен: sftp гейт не пропускает). Пустая строка — успех."""
        local = Path(local)
        if local.is_file() and local.stat().st_size > 0:
            return ""
        local.parent.mkdir(parents=True, exist_ok=True)
        part = local.with_suffix(local.suffix + ".part")
        ok, _out, err = self._run(
            ["scp", "-O", "-q", "-i", str(self.key), *SSH_OPTS, f"{self.host}:{remote}", str(part)]
        )
        if not ok or not part.is_file():
            return f"scp {remote}: {err or 'файла нет'}"
        part.replace(local)
        return ""


def runs_catalog(gate: Gate) -> tuple:
    """Прогоны на песочнице: [{"id", "label", "kind", "topic"}], причина при неудаче.

    Прогон с позой — каталог с maprec/poses.csv. Корень out/ с кадрами камер — прогон TOP_RUN (вид сверху).
    """
    names, reason = gate.ls(HIL_OUT)
    if reason:
        return [], reason
    found = []
    if "frames" in names and "route.log" in names:
        found.append({"id": TOP_RUN, "label": "склад Depot: вид сверху и камера", "kind": "frames", "topic": ""})
    for name in sorted(names):
        if not NAME_RE.match(name) or "." in name or name in ("frames", "maprec"):
            continue
        inner, _why = gate.ls(f"{HIL_OUT}/{name}/maprec")
        if "poses.csv" not in inner:
            continue
        known = KNOWN_RUNS.get(name, {})
        found.append(
            {
                "id": name,
                "label": known.get("label", f"прогон {name}"),
                "kind": "poses",
                "topic": known.get("topic", ""),
                "role": known.get("role", ""),
            }
        )
    return found, ""


def topic_of(text: str) -> str:
    """Тема разговора по словам поста и просьбы: код сопоставляет её с известным материалом."""
    low = (text or "").lower()
    for topic, spec in TOPICS.items():
        if any(marker in low for marker in spec["markers"]):
            return topic
    return ""


def pair_for(topic: str, catalog: list) -> list:
    """Пара «до/после» по теме: [до, после] из каталога или []."""
    by_role = {item.get("role"): item["id"] for item in catalog if item.get("topic") == topic and item.get("role")}
    if "before" in by_role and "after" in by_role:
        return [by_role["before"], by_role["after"]]
    return []


def map_time(name: str) -> Optional[float]:
    """map_000055982.npz → 55.982 с (как в compose.py)."""
    match = re.match(r"^map_(\d+)\.npz$", name)
    return int(match.group(1)) / 1000 if match else None


def sample(items: list, count: int) -> list:
    """Равномерно count элементов, последний — всегда (финальная карта)."""
    if len(items) <= count:
        return list(items)
    step = (len(items) - 1) / (count - 1)
    return [items[round(i * step)] for i in range(count)]


def fetch_pose_run(gate: Gate, run_id: str, cache: Path, maps: int = MAP_SAMPLES) -> tuple:
    """poses.csv и выборка снимков карты прогона. ({"poses": путь, "maps": [пути]}, причина)."""
    if not NAME_RE.match(run_id):
        return None, f"имя прогона {run_id!r} не годится"
    base = f"{HIL_OUT}/{run_id}/maprec"
    local = Path(cache) / run_id
    reason = gate.fetch(f"{base}/poses.csv", local / "poses.csv")
    if reason:
        return None, reason
    names, _why = gate.ls(f"{base}/maps")
    snapshots = sorted((n for n in names if map_time(n) is not None), key=map_time)
    paths = []
    for name in sample(snapshots, maps):
        if not gate.fetch(f"{base}/maps/{name}", local / "maps" / name):
            paths.append(local / "maps" / name)
    return {"poses": local / "poses.csv", "maps": paths}, ""


def load_maps(paths: list) -> tuple:
    """Снимки карты для подложки: [{"t", "resolution", "origin_x", "origin_y", "data"}], заметка.

    Без numpy подложки не будет — ролик соберётся без неё, а индикатор скажет об этом.
    """
    try:
        import numpy as np
    except ImportError:
        return [], "подложки карты не будет: нет numpy"
    loaded = []
    for path in paths:
        try:
            snap = np.load(path)
            loaded.append(
                {
                    "t": map_time(Path(path).name),
                    "resolution": float(snap["resolution"]),
                    "origin_x": float(snap["origin_x"]),
                    "origin_y": float(snap["origin_y"]),
                    "data": snap["data"],
                }
            )
        except (OSError, KeyError, ValueError):
            continue
    return loaded, ""
