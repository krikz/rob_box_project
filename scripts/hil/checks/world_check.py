#!/usr/bin/env python3
"""Проверка мира Gazebo до первой цели Depot (эпик #4694).

Имя файла polygon.sdf само по себе ни PASS, ни FAIL: mkdepot тоже зовёт мир
polygon, но файл ``/sim_video/world.sdf`` и в нём ``Depot_vid``. Решение — код
по трём raw-фактам (args, mounts, текст SDF), без ROS и стенда.

    python world_check.py depot --args '…' --mount /sim_video --sdf-file world.sdf
Код выхода: 0 PASS, 1 FAIL. Строка stdout — ``PASS: …`` или ``FAIL: WORLD_MISMATCH: …``.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import List, Sequence, Tuple

PASS, FAIL = "PASS", "FAIL"
WORLD_MISMATCH = "WORLD_MISMATCH"

DEPOT_WORLD_ARG = "world:=/sim_video/world.sdf"
DEPOT_WORLD_PATH = "/sim_video/world.sdf"
DEPOT_MOUNT = "/sim_video"
DEPOT_MARKER = "Depot_vid"

# XML-комментарии целиком; uri — только реальный include, не текст в <!-- … -->.
_XML_COMMENT_RE = re.compile(r"<!--.*?-->", re.DOTALL)
_URI_RE = re.compile(r"<uri\b[^>]*>\s*([^<]*?)\s*</uri>", re.IGNORECASE)


def _has_exact_world_arg(args: str) -> bool:
    """Точный токен ``world:=/sim_video/world.sdf``, не подстрока (world.sdf.old — нет)."""
    return DEPOT_WORLD_ARG in (args or "").split()


def _sdf_has_depot_uri(sdf_text: str) -> bool:
    """``Depot_vid`` в значении ``<uri>…</uri>`` вне XML-комментариев."""
    bare = _XML_COMMENT_RE.sub("", sdf_text or "")
    for m in _URI_RE.finditer(bare):
        if DEPOT_MARKER in m.group(1):
            return True
    return False


def check_world(expect: str, args: str, mounts: Sequence[str], sdf_text: str) -> Tuple[str, str]:
    """Вердикт по ожидаемому миру.

    Для ``expect == "depot"`` — PASS только если одновременно:
      1. среди whitespace-токенов args есть ровно ``world:=/sim_video/world.sdf``;
      2. среди mount destination есть ``/sim_video``;
      3. в SDF есть ``<uri>…Depot_vid…</uri>`` вне XML-комментариев.
    Иначе FAIL с ``WORLD_MISMATCH`` и перечнем того, что не сошлось.
    Имя ``polygon.sdf`` само по себе ни PASS, ни FAIL.
    """
    if expect != "depot":
        return FAIL, f"{WORLD_MISMATCH}: unknown expect={expect!r} (only depot)"

    missing: List[str] = []
    if not _has_exact_world_arg(args):
        missing.append(f"args missing {DEPOT_WORLD_ARG}")

    dests = {(m or "").rstrip("/") for m in mounts}
    if DEPOT_MOUNT not in dests:
        missing.append(f"mounts missing {DEPOT_MOUNT}")

    if not _sdf_has_depot_uri(sdf_text):
        missing.append(f"sdf missing <uri>…{DEPOT_MARKER}…</uri>")

    if missing:
        return FAIL, f"{WORLD_MISMATCH}: {'; '.join(missing)}"
    return PASS, "depot: args world.sdf, mount /sim_video, sdf Depot_vid"


def main(argv: Sequence[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n", 1)[0])
    p.add_argument("expect", help="ожидаемый мир: depot")
    p.add_argument("--args", default="", help="строка Args контейнера gazebo")
    p.add_argument("--mount", action="append", default=[], dest="mounts", help="destination тома (повтор)")
    p.add_argument("--mounts-file", type=Path, help="файл: destination по строке")
    p.add_argument("--sdf", default="", help="текст SDF")
    p.add_argument("--sdf-file", type=Path, help="файл SDF")
    ns = p.parse_args(list(argv) if argv is not None else None)

    mounts: List[str] = list(ns.mounts)
    if ns.mounts_file is not None:
        mounts.extend(ns.mounts_file.read_text(encoding="utf-8").splitlines())
    mounts = [m.strip() for m in mounts if m.strip()]

    sdf = ns.sdf
    if ns.sdf_file is not None:
        sdf = ns.sdf_file.read_text(encoding="utf-8")

    status, detail = check_world(ns.expect, ns.args, mounts, sdf)
    print(f"{status}: {detail}")
    return 0 if status == PASS else 1


if __name__ == "__main__":
    sys.exit(main())
