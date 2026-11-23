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
# Штатный сценарий Depot: копия после patch_compose.py (не имя мира polygon).
DEPOT_SCENARIO = "depot"
POLYGON_SCENARIO = "polygon"

# XML-комментарии целиком; uri — только include, не текст в <!-- … -->.
_XML_COMMENT_RE = re.compile(r"<!--.*?-->", re.DOTALL)
_URI_RE = re.compile(r"<uri\b[^>]*>\s*([^<]*?)\s*</uri>", re.IGNORECASE)


def _has_exact_world_arg(args: str) -> bool:
    """Точный токен ``world:=/sim_video/world.sdf``, не подстрока (world.sdf.old — нет)."""
    return DEPOT_WORLD_ARG in (args or "").split()


def world_arg_path(args: str) -> str:
    """Путь из первого токена ``world:=…`` (пусто, если токена нет)."""
    for tok in (args or "").split():
        if tok.startswith("world:="):
            return tok[len("world:=") :]
    return ""


def scenario_expects_depot(scenario: str) -> bool:
    """Штатный сценарий Depot обязан проверять мир. Polygon/building — нет."""
    return (scenario or "").strip() == DEPOT_SCENARIO


def expect_world_for_scenario(scenario: str) -> str:
    """``HIL_EXPECT_WORLD`` для сценария: depot — всегда ``depot``, иначе пусто.

    Пустое значение гейт само по себе не включает.
    """
    if scenario_expects_depot(scenario):
        return DEPOT_SCENARIO
    return ""


def gate_required(expect_world: str, args: str) -> bool:
    """Гейт обязателен, если сценарий выставил depot или args уже Depot-мир.

    ``world.sdf.old`` и обычный polygon без переменной гейт не включают.
    """
    if (expect_world or "").strip() == DEPOT_SCENARIO:
        return True
    return world_arg_path(args) == DEPOT_WORLD_PATH


def compose_is_depot(compose_text: str) -> bool:
    """Копия Compose после ``patch_compose.py``: точный токен мира и том ``/sim_video``.

    Обычный ``compose.hil.yaml`` (``world:=/sim_worlds/${SIM_WORLD:-polygon}.sdf``, без тома)
    — не Depot, даже если в комментарии есть ``Depot_vid``.
    """
    if not _has_exact_world_arg(compose_text or ""):
        return False
    return "/sim_video" in (compose_text or "")


def on_must_check_depot(expect_world: str, compose_text: str) -> bool:
    """``hil.sh on`` обязан проверить мир до первой цели.

    Да, если сценарий уже выставил depot или копия Compose — штатный Depot
    (полный маршрут runvideo, одиночные M1/R7, pathcalc). Обычный Polygon
    и building без переменной — нет. ``world.sdf.old`` и ``Depot_vid`` в
    комментарии копию Depot не делают.
    """
    if (expect_world or "").strip() == DEPOT_SCENARIO:
        return True
    return compose_is_depot(compose_text)


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
    p.add_argument("expect", nargs="?", default="", help="ожидаемый мир: depot")
    p.add_argument("--args", default="", help="строка Args контейнера gazebo")
    p.add_argument("--mount", action="append", default=[], dest="mounts", help="destination тома (повтор)")
    p.add_argument("--mounts-file", type=Path, help="файл: destination по строке")
    p.add_argument("--sdf", default="", help="текст SDF")
    p.add_argument("--sdf-file", type=Path, help="файл SDF")
    p.add_argument(
        "--scenario",
        default="",
        help="штатный сценарий: depot печатает HIL_EXPECT_WORLD=depot, иначе пусто",
    )
    p.add_argument(
        "--gate-required",
        action="store_true",
        help="с --args: печатает required|skip (гейт обязателен?)",
    )
    p.add_argument(
        "--compose-file",
        type=Path,
        help="копия compose.hil.yaml: печатает depot|polygon (штатный сценарий Depot?)",
    )
    p.add_argument(
        "--on-must-check",
        action="store_true",
        help="с --compose-file: печатает required|skip (hil.sh on обязан проверить мир?)",
    )
    ns = p.parse_args(list(argv) if argv is not None else None)

    if ns.scenario:
        print(expect_world_for_scenario(ns.scenario))
        return 0
    if ns.compose_file is not None:
        text = ns.compose_file.read_text(encoding="utf-8")
        if ns.on_must_check:
            print("required" if on_must_check_depot(ns.expect, text) else "skip")
            return 0
        kind = DEPOT_SCENARIO if compose_is_depot(text) else POLYGON_SCENARIO
        print(kind)
        return 0
    if ns.gate_required:
        print("required" if gate_required(ns.expect, ns.args) else "skip")
        return 0
    if not ns.expect:
        p.error("expect обязателен (либо --scenario / --gate-required)")

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
