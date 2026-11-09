#!/usr/bin/env python3
"""Сводка e2e-БД ходов ``dialog_events`` без текстов (issue #4512).

Репо публичное, артефакты прогона качает любой залогиненный на GitHub, а в ``text`` лежат реплики людей мастерской.
Поэтому сама БД остаётся на билд-машине (``dialog_events_e2e_db.tar`` в OUT_DIR акта, в архив workflow не уходит),
а в артефакт идёт только ``dialog_events_e2e_db.json``: число событий по видам, число ходов и итоги ходов
(``stage``/``reason`` из агрегата ``kind=turn``, он «без текста», ``rob_box_dialog.turn_log``).

Использование: dialog_db_summary.py <tar> <имя БД внутри tar> <out.json>. Печатает одну строку для лога.
"""

from __future__ import annotations

import json
import os
import sqlite3
import sys
import tarfile
import tempfile
from collections import Counter


def summarize(con: sqlite3.Connection) -> dict:
    kinds = {kind: n for kind, n in con.execute("SELECT kind, COUNT(*) FROM dialog_events GROUP BY kind")}
    outcomes: Counter = Counter()
    for (raw,) in con.execute("SELECT data_json FROM dialog_events WHERE kind='turn'"):
        try:
            data = json.loads(raw or "{}")
        except ValueError:
            data = {}
        outcomes["%s/%s" % (data.get("stage", "-"), data.get("reason", "-"))] += 1
    turns = con.execute("SELECT COUNT(DISTINCT turn_id) FROM dialog_events").fetchone()[0]
    return {"events": kinds, "turns": turns, "turn_outcomes": dict(outcomes)}


def main(argv) -> int:
    tar_path, base, out_path = argv[1:4]
    report: dict = {"db": base}
    try:
        with tempfile.TemporaryDirectory() as tmp:
            with tarfile.open(tar_path) as tf:
                members = [m for m in tf.getmembers() if m.isfile() and m.name in (base, base + "-wal", base + "-shm")]
                report["files"] = [{"name": m.name, "bytes": m.size} for m in members]
                tf.extractall(tmp, members=members)
            con = sqlite3.connect(os.path.join(tmp, base))
            report.update(summarize(con))
            con.close()
    except Exception as exc:  # копия есть и без сводки: разбор руками по tar
        report["error"] = "%s: %s" % (type(exc).__name__, exc)
    with open(out_path, "w", encoding="utf-8") as fh:
        json.dump(report, fh, ensure_ascii=False, indent=1)
    print(report.get("error") or "events %s, turns %s" % (sum(report["events"].values()), report["turns"]))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
