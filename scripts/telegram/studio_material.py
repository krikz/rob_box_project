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
import time
from pathlib import Path
from typing import Callable, Optional

SANDBOX = os.environ.get("STUDIO_HIL_HOST", "ros2@10.1.1.250")
HIL_OUT = os.environ.get("STUDIO_HIL_OUT", "/home/ros2/rob_box_hilvideo/out")
KEY = Path(os.environ.get("STUDIO_RO_KEY", str(Path.home() / ".ssh" / "studio_ro")))
SSH_OPTS = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "-o", "StrictHostKeyChecking=accept-new"]
MAP_SAMPLES = 12  # снимков карты на прогон: подложка меняется по ходу ролика, а качать все 400 — долго
TOP_RUN = "depot"  # корень out/ — прогон с кадрами камер (вид сверху, compose.py)
NAME_RE = re.compile(r"^[A-Za-z0-9_.-]{1,64}$")

# Сколько живёт файл материала в кэше, прежде чем его перекачать с песочницы.
# Разбор 14.11 (issue #4711): `Gate.fetch()` возвращал успех на любой непустой локальный
# файл, поэтому route.log лежал с 07.11 — студия монтировала материал недельной давности,
# пока песочница давно перезаписала прогон. Шесть часов — материал свежий в пределах сессии.
CACHE_MAX_AGE_S = 6 * 3600

# Порог «прогон — материал», а не отладочный заход: у отладочных одна цель и
# почти нулевое покрытие (ADR-0171 §2a — незавершённая работа и отладочные прогоны
# не материал).
MIN_GOALS = 2
MIN_COVER = 1.0  # м² размаха траектории

# Подписи прогонов, которые уже обсуждали (#4495, дрейф локализации). Код, а не модель, знает, что есть «до».
KNOWN_RUNS = {
    "shisyun_s1": {"label": "ДО: прогон s1", "topic": "drift", "role": "before"},
    "shisyun_s2": {"label": "ПОСЛЕ: прогон s2", "topic": "drift", "role": "after"},
}
# Маркеры — только про сам замер дрейфа. Номер epic (#4495) и общее
# «rtabmap» убраны: по ним подходил любой PR той же epic, и ролик про
# дрейф цеплялся к посту про конфиг (Шифу, 07.11: «текст не соответствует
# видео»).
TOPICS = {
    "drift": {
        "title": "Дрейф локализации на складе Depot (HIL, #4495)",
        "markers": ("дрейф", "depot", "icp", "локализац", "поза уплыва", "уплыв", "потеря опоры"),
    }
}


def cache_stale(local: Path, max_age_s: int = CACHE_MAX_AGE_S) -> bool:
    """Файл в кэше протух: пора перекачать с песочницы.

    Разбор 14.11 (issue #4711): кэш не обновлялся вообще, и студия монтировала
    материал недельной давности — route.log от 07.11, пока песочница давно
    перезаписала прогон.
    """
    try:
        age = time.time() - Path(local).stat().st_mtime
    except OSError:
        return True
    return age > max_age_s


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
        """Один файл через legacy scp (-O обязателен: sftp гейт не пропускает). Пустая строка — успех.

        Файл в кэше берётся, только если он свежий: иначе материал протухает (issue #4711).
        """
        local = Path(local)
        if local.is_file() and local.stat().st_size > 0 and not cache_stale(local):
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


def route_goals(route_text: str) -> int:
    """Сколько целей маршрута прошёл робот: все строки с меткой GOAL_START.

    Считаются ВСЕ цели (M* и R*): факт о прогоне и окно монтажа должны говорить
    одно и то же число (issue #4711). Раньше факт считал все, а окно — только M*:
    читатель видел «10 целей», в ролике было пять.
    """
    goals = 0
    for line in (route_text or "").splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[1] == "GOAL_START":
            goals += 1
    return goals


def measure_run(rows: list) -> Optional[dict]:
    """Измерение прогона по строкам позы: {"goals", "cover", "length", "points"}.

    cover — размах траектории по осям (span_x, span_y): чем больше, тем шире
    прошёл робот. length — длина пути в метрах. None — измерять нечего.
    """
    xs, ys = [], []
    for row in rows or []:
        try:
            xs.append(float(row["tx"]))
            ys.append(float(row["ty"]))
        except (KeyError, TypeError, ValueError):
            continue
    if len(xs) < 2:
        return None
    length = 0.0
    for i in range(len(xs) - 1):
        length += ((xs[i + 1] - xs[i]) ** 2 + (ys[i + 1] - ys[i]) ** 2) ** 0.5
    return {
        "goals": 0,  # цели считаются по route.log, позы о них не знают
        "cover": (max(xs) - min(xs), max(ys) - min(ys)),
        "length": length,
        "points": len(xs),
    }


def best_run(runs: list) -> str:
    """Прогон с самым длинным заездом и максимальным покрытием. "" — если ни один не годится.

    Правило кода, а не выбор модели: «найди самый длинный заезд» (Шифу, 14.11)
    должно находиться по данным. Отладочные прогоны (одна цель, ничтожное
    покрытие) в выдачу не попадают — см. MIN_GOALS и MIN_COVER.
    """
    best_id, best_key = "", None
    for item in runs or []:
        cover = item.get("cover") or (0.0, 0.0)
        goals = int(item.get("goals") or 0)
        area = float(cover[0]) * float(cover[1])
        if goals < MIN_GOALS or area < MIN_COVER:
            continue
        # Сначала покрытие (максимальное), потом длина пути, потом число целей.
        key = (area, float(item.get("length") or 0.0), goals)
        if best_key is None or key > best_key:
            best_key, best_id = key, str(item.get("id") or "")
    return best_id


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
