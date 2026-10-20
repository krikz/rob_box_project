#!/usr/bin/env python3
"""Батч PDMX на katana: отбор → импорт → манифест → индекс (ADR-0154 §3.2, PR-6, М1/М7).

Надстройка над ``score_import.process_file`` (разбор партитуры там, здесь его копии нет): то, что ``score_import.py``
не умеет на 100–200 тысячах файлов, — отбор по рангу, продолжение после остановки, пул с перезапуском процессов
(music21 копит память), таймаут на файл и манифест сделанного.

``select``
    ``PDMX.csv`` → ``selection.jsonl`` в порядке обработки. Отбор (ADR-0154 В2, ADR-0155 §2.4): ``subset:no_license_conflict``,
    лицензия пригодна для индекса (``material.license_usable``), нет имён из ``knowledge.LICENSE_STOP_LIST``.
    Порядок: сначала ``subset:deduplicated`` (ступень 1), затем остальные (ступень 2, ``--tier2-dedup`` — ещё и
    дедуп по нормализованным «название|композитор»); внутри ступени — байесовский рейтинг, затем просмотры.

``run``
    Строки ``selection.jsonl`` (минус уже сделанные по ``manifest.jsonl``) → ``score_import.process_file`` в пуле;
    JSON материала — в ``--lib``, строка результата (с причиной отказа и строкой индекса) — в манифест сразу, так что
    остановка в любой момент не теряет сделанное, повтор команды продолжает.

``index``
    Манифест → ``score_index.db`` в каталоге библиотеки (ровно принятые материалы; лицензионный гейт индекса).

``report``
    Манифест → отчёт M1/M7: принято/отказы по причинам, файлов/с по ``seconds`` и по стене.

    python3 scripts/music/pdmx_batch.py select --csv ~/pdmx/PDMX.csv --out ~/pr6/selection.jsonl
    python3 scripts/music/pdmx_batch.py run --sel ~/pr6/selection.jsonl --root ~/pr6/mxl --lib ~/pr6/lib \\
        --manifest ~/pr6/manifest.jsonl --jobs 5 [--limit 200]
    python3 scripts/music/pdmx_batch.py index --manifest ~/pr6/manifest.jsonl --lib ~/pr6/lib
    python3 scripts/music/pdmx_batch.py report --manifest ~/pr6/manifest.jsonl
"""

from __future__ import annotations

import argparse
import collections
import csv
import dataclasses
import json
import multiprocessing
import os
import pathlib
import signal
import sys
import time
from typing import Any, Dict, Iterable, Iterator, List, Mapping, Optional, Sequence, Set, Tuple

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parent.parent
INDEX_FILE = "score_index.db"
FILE_TIMEOUT_S = 180
NA = {"", "NA", "nan", "None"}
#: Байесовский рейтинг: ``(rating·n + PRIOR·W) / (n + W)`` — партитура с одной пятёркой не обгоняет проверенную 4.8×50.
PRIOR_RATING, PRIOR_WEIGHT = 3.5, 3.0


def _paths() -> None:
    for p in (str(REPO / "src" / "rob_box_music"), str(HERE)):
        if p not in sys.path:
            sys.path.insert(0, p)


def _num(value: Any, kind: type) -> Optional[Any]:
    try:
        x = float(value)
    except (TypeError, ValueError):
        return None
    return None if x != x else kind(x)


def _clean(value: Any) -> str:
    return "" if value in NA or value is None else str(value)


def bayes_rating(rating: Optional[float], n_ratings: Optional[int]) -> float:
    n = n_ratings or 0
    return ((rating or 0.0) * n + PRIOR_RATING * PRIOR_WEIGHT) / (n + PRIOR_WEIGHT)


def rank_key(row: Mapping[str, Any]) -> Tuple[float, int]:
    return (-bayes_rating(row.get("rating"), row.get("n_ratings")), -(row.get("n_views") or 0))


# ── select ─────────────────────────────────────────────────────────────────────────────────────────────────────

def select_rows(csv_path: str, tier2_dedup: bool = True) -> Tuple[List[Dict[str, Any]], Dict[str, int]]:
    """Отбор и порядок обработки; возвращает ``(строки, счётчики для отчёта)``."""
    _paths()
    from rob_box_music import material as mt, works

    csv.field_size_limit(min(sys.maxsize, 2 ** 31 - 1))
    stats: collections.Counter = collections.Counter()
    tier1: List[Dict[str, Any]] = []
    tier2: List[Dict[str, Any]] = []
    with open(csv_path, encoding="utf-8", newline="") as fh:
        for r in csv.DictReader(fh):
            stats["csv_rows"] += 1
            if r.get("subset:no_license_conflict") != "True" or str(r.get("license_conflict")).strip().lower() == "true":
                stats["drop_license_conflict"] += 1
                continue
            lic = _clean(r.get("license"))
            if not mt.license_usable(lic):
                stats["drop_license_unusable"] += 1
                continue
            title = _clean(r.get("title")) or _clean(r.get("song_name"))
            composer = _clean(r.get("composer_name")) or _clean(r.get("artist_name"))
            if works.is_stop_listed(title, _clean(r.get("song_name")), composer, _clean(r.get("artist_name"))):
                stats["drop_stop_list"] += 1
                continue
            mxl = _clean(r.get("mxl"))
            if not mxl:
                stats["drop_no_mxl"] += 1
                continue
            row = {"stem": pathlib.PurePosixPath(mxl).stem, "mxl": mxl.lstrip("./"), "license": lic,
                   "license_conflict": False, "rating": _num(r.get("rating"), float),
                   "n_ratings": _num(r.get("n_ratings"), int), "n_views": _num(r.get("n_views"), int),
                   "complexity": _num(r.get("complexity"), int), "genres": _clean(r.get("genres")),
                   "title": title, "composer": composer}
            (tier1 if r.get("subset:deduplicated") == "True" else tier2).append(row)
    stats["nlc_selected"] = len(tier1) + len(tier2)
    stats["tier1_dedup"] = len(tier1)
    tier1.sort(key=rank_key)
    tier2.sort(key=rank_key)
    if tier2_dedup:
        seen = {_work_key(r) for r in tier1}
        kept = []
        for r in tier2:  # лучший по рангу из версий одного произведения; произведение уже в ступени 1 — не берём
            key = _work_key(r)
            if key not in seen:
                seen.add(key)
                kept.append(r)
        stats["tier2_before_dedup"] = len(tier2)
        tier2 = kept
    stats["tier2"] = len(tier2)
    rows = tier1 + tier2
    for i, r in enumerate(rows):
        r["rank"] = i
        r["tier"] = 1 if i < len(tier1) else 2
    return rows, dict(stats)


def _work_key(row: Mapping[str, Any]) -> str:
    _paths()
    from rob_box_music import works

    return works.norm(str(row["title"])) + "|" + works.norm(str(row["composer"]))


def cmd_select(args: argparse.Namespace) -> int:
    rows, stats = select_rows(args.csv, not args.no_tier2_dedup)
    if args.limit:
        rows = rows[:args.limit]
    with open(args.out, "w", encoding="utf-8") as fh:
        for r in rows:
            fh.write(json.dumps(r, ensure_ascii=False) + "\n")
    for k, v in stats.items():
        print(f"{k}: {v}")
    print(f"→ {args.out}: {len(rows)} строк")
    return 0


# ── run ────────────────────────────────────────────────────────────────────────────────────────────────────────

def read_jsonl(path: pathlib.Path) -> Iterator[Dict[str, Any]]:
    if not path.is_file():
        return
    with path.open(encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                try:
                    yield json.loads(line)
                except ValueError:  # оборванная при остановке последняя строка — файл будет обработан заново
                    continue


class FileTimeout(Exception):
    pass


def _on_alarm(_signum, _frame) -> None:
    raise FileTimeout()


def _work(task: Tuple[str, Dict[str, Any]]) -> Dict[str, Any]:
    """Один файл в процессе пула: разбор через ``score_import.process_file`` с таймаутом; без исключений наружу."""
    path, row = task
    _paths()
    import score_import as si

    stem = row["stem"]
    res: Dict[str, Any]
    try:
        if hasattr(signal, "SIGALRM"):
            signal.signal(signal.SIGALRM, _on_alarm)
            signal.alarm(FILE_TIMEOUT_S)
        res = si.process_file((path, {stem: row}, "", ""))
    except FileTimeout:
        res = {"file": os.path.basename(path), "status": "refused", "code": "timeout",
               "reason": f"разбор дольше {FILE_TIMEOUT_S} с", "seconds": float(FILE_TIMEOUT_S)}
    except Exception as exc:  # noqa: BLE001 - любой сбой файла — в манифест, не в падение батча
        res = {"file": os.path.basename(path), "status": "refused", "code": "worker_error",
               "reason": f"{type(exc).__name__}: {exc}"[:160], "seconds": 0.0}
    finally:
        if hasattr(signal, "SIGALRM"):
            signal.alarm(0)
    res["stem"] = stem
    return res


def manifest_line(res: Mapping[str, Any], json_bytes: int) -> Dict[str, Any]:
    """Строка манифеста: итог файла + строка индекса (чтобы индекс собирался из манифеста, без чтения JSON)."""
    line: Dict[str, Any] = {"stem": res["stem"], "status": res["status"], "seconds": res.get("seconds", 0.0)}
    if res["status"] == "ok":
        line.update(material_id=res["material_id"], bytes=json_bytes, bars_used=res.get("bars_used"),
                    bars_total=res.get("bars_total"), index=dataclasses.asdict(res["index"]))
    else:
        line.update(code=res.get("code", "?"), reason=res.get("reason", ""))
    return line


def cmd_run(args: argparse.Namespace) -> int:
    _paths()
    import score_import as si

    sel = list(read_jsonl(pathlib.Path(args.sel)))
    manifest = pathlib.Path(args.manifest)
    done: Set[str] = {m["stem"] for m in read_jsonl(manifest)}
    root, lib = pathlib.Path(args.root).expanduser(), pathlib.Path(args.lib).expanduser()
    lib.mkdir(parents=True, exist_ok=True)
    todo = [r for r in sel if r["stem"] not in done]
    if args.limit:
        todo = todo[:args.limit]
    missing = [r for r in todo if not (root / r["mxl"]).is_file()]
    if missing:
        print(f"нет файла на диске: {len(missing)} (первые: {[r['mxl'] for r in missing[:3]]}) — "
              f"они пойдут в манифест как missing_file", flush=True)
    present = [r for r in todo if (root / r["mxl"]).is_file()]
    t0 = time.time()
    n_ok = n_ref = 0
    with manifest.open("a", encoding="utf-8") as mf:
        for r in missing:
            mf.write(json.dumps({"stem": r["stem"], "status": "refused", "code": "missing_file",
                                 "reason": r["mxl"], "seconds": 0.0}, ensure_ascii=False) + "\n")
        tasks = [(str(root / r["mxl"]), r) for r in present]
        ctx = multiprocessing.get_context("fork" if hasattr(os, "fork") else "spawn")
        with ctx.Pool(max(1, args.jobs), maxtasksperchild=args.recycle) as pool:
            for i, res in enumerate(pool.imap_unordered(_work, tasks, chunksize=2), 1):
                size = 0
                if res["status"] == "ok":
                    text = res.pop("json")
                    (lib / si.file_name_of(res["material_id"])).write_text(text, encoding="utf-8")
                    size = len(text.encode("utf-8"))
                    n_ok += 1
                else:
                    n_ref += 1
                mf.write(json.dumps(manifest_line(res, size), ensure_ascii=False) + "\n")
                if i % 500 == 0:
                    mf.flush()
                    dt = time.time() - t0
                    print(f"[{time.strftime('%H:%M:%S')}] {i}/{len(tasks)}  принято {n_ok}  отказов {n_ref}  "
                          f"{i / dt:.2f} файл/с", flush=True)
    dt = time.time() - t0
    print(f"готово: обработано {n_ok + n_ref + len(missing)} за {dt:.0f} с "
          f"({(n_ok + n_ref) / max(dt, 1e-9):.2f} файл/с), принято {n_ok}, отказов {n_ref + len(missing)}")
    return 0


# ── index / report ─────────────────────────────────────────────────────────────────────────────────────────────

def cmd_index(args: argparse.Namespace) -> int:
    """Манифест → ``score_index.db`` каталога; JSON без записи индекса и записи без JSON — ошибка, не пропуск."""
    _paths()
    import sqlite3
    from rob_box_music import works
    import score_import as si

    lib = pathlib.Path(args.lib).expanduser()
    rows = []
    for m in read_jsonl(pathlib.Path(args.manifest)):
        if m["status"] == "ok":
            if not (lib / si.file_name_of(m["material_id"])).is_file():
                raise SystemExit(f"в манифесте {m['material_id']}, но JSON нет в {lib}")
            rows.append(works.ScoreIndexRow(**m["index"]))
    db = lib / INDEX_FILE
    if db.exists():
        db.unlink()
    conn = sqlite3.connect(str(db))
    try:
        written, rejected = works.write_score_index(conn, rows)
    finally:
        conn.close()
    print(f"score_index: записано {written}, отклонено гейтом {len(rejected)}: {rejected[:5]}")
    return 0 if not rejected else 1


def _fmt_size(n: float) -> str:
    return f"{n / 1e6:.1f} МБ" if n >= 1e6 else f"{n / 1e3:.1f} КБ"


def report_lines(manifest: Iterable[Mapping[str, Any]]) -> List[str]:
    ms = list(manifest)
    ok = [m for m in ms if m["status"] == "ok"]
    ref = [m for m in ms if m["status"] != "ok"]
    n = len(ms)
    codes = collections.Counter(m["code"] for m in ref)
    cpu = sum(m.get("seconds", 0.0) for m in ms)
    times = sorted(m.get("seconds", 0.0) for m in ms)
    total_bytes = sum(m.get("bytes", 0) for m in ok)
    lines = [f"в манифесте {n}; принято {len(ok)} ({len(ok) / max(1, n):.1%} — M1, порог ≥ 95 %); отказов {len(ref)}",
             "отказы по причинам: " + (", ".join(f"{c} ×{k}" for c, k in codes.most_common()) or "нет"),
             f"CPU-время разбора {cpu:.0f} с, медиана {times[len(times) // 2] if times else 0:.2f} с, "
             f"максимум {times[-1] if times else 0:.1f} с; JSON материалов {_fmt_size(total_bytes)}"
             f" (в среднем {_fmt_size(total_bytes / max(1, len(ok)))})"]
    lines += [f"  пример отказа {m['stem']}: {m['code']}: {m['reason'][:120]}" for m in ref[:15]]
    return lines


def cmd_report(args: argparse.Namespace) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    print("\n".join(report_lines(read_jsonl(pathlib.Path(args.manifest)))))
    return 0


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("select", help="PDMX.csv → selection.jsonl в порядке обработки")
    s.add_argument("--csv", required=True)
    s.add_argument("--out", required=True)
    s.add_argument("--limit", type=int, default=0)
    s.add_argument("--no-tier2-dedup", action="store_true", help="ступень 2 целиком, без дедупа по название|композитор")
    r = sub.add_parser("run", help="selection.jsonl → JSON материалов + манифест (продолжаемо)")
    r.add_argument("--sel", required=True)
    r.add_argument("--root", required=True, help="корень распакованных .mxl (путь строки selection — относительно него)")
    r.add_argument("--lib", required=True, help="каталог JSON-библиотеки (вне git)")
    r.add_argument("--manifest", required=True)
    r.add_argument("--jobs", type=int, default=4)
    r.add_argument("--limit", type=int, default=0)
    r.add_argument("--recycle", type=int, default=200, help="файлов на процесс до перезапуска (music21 копит память)")
    i = sub.add_parser("index", help="манифест → score_index.db в каталоге библиотеки")
    i.add_argument("--manifest", required=True)
    i.add_argument("--lib", required=True)
    p = sub.add_parser("report", help="манифест → отчёт M1/M7")
    p.add_argument("--manifest", required=True)
    return ap


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    return {"select": cmd_select, "run": cmd_run, "index": cmd_index, "report": cmd_report}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
