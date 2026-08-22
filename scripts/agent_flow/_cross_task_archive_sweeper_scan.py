#!/usr/bin/env python3
"""Scan helper for cross-task-archive-sweeper.sh.

Выбирает из tasks.db кандидатов на archive-sweep:
  - status='blocked'
  - assignee=ASSIGNEE
  - last update (COALESCE(started_at, created_at)) < now - STALE_HOURS*3600

Выводит JSON в stdout с полями:
  cutoff, now, candidates: [{id, title, branch_name, started_at,
                              stale_seconds, refs, claim_lock,
                              has_worker_pid, root_task_id}]

refs — список номеров PR/issue, найденных в body по regex (?<![A-Za-z0-9_])#\d{3,5}
Это read-only SELECT. Никаких мутаций.

Usage:
  _cross_task_archive_sweeper_scan.py KANBAN_DB ASSIGNEE STALE_HOURS LIMIT

Exit codes:
  0 — ok (даже если candidates==0)
  1 — fatal (DB missing/unreadable/sys.argv invalid)
"""
import sqlite3
import json
import re
import sys
import time

ISSUE_RE = re.compile(r'(?<![A-Za-z0-9_])#(\d{3,5})\b')


def main() -> int:
    if len(sys.argv) != 5:
        print("usage: _cross_task_archive_sweeper_scan.py KANBAN_DB ASSIGNEE STALE_HOURS LIMIT",
              file=sys.stderr)
        return 2
    db_path, assignee, stale_h_s, limit_s = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
    try:
        stale_h = int(stale_h_s)
        limit = int(limit_s)
    except ValueError as e:
        print(f"invalid numeric arg: {e}", file=sys.stderr)
        return 2
    if stale_h < 0 or limit <= 0:
        print("STALE_HOURS must be >= 0, LIMIT must be > 0", file=sys.stderr)
        return 2

    now = int(time.time())
    cutoff = now - stale_h * 3600

    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    try:
        cur = conn.cursor()
        sql = ("SELECT id, title, body, branch_name, status, started_at, completed_at, "
               "created_at, claim_lock, worker_pid, repo_url, root_task_id "
               "FROM tasks "
               "WHERE status = 'blocked' AND assignee = ? "
               "AND COALESCE(started_at, created_at) < ? "
               "ORDER BY COALESCE(started_at, created_at) ASC "
               "LIMIT ?")
        rows = list(cur.execute(sql, (assignee, cutoff, limit)))
    except sqlite3.OperationalError as e:
        print(f"DB query failed: {e}", file=sys.stderr)
        return 1
    finally:
        conn.close()

    results = []
    for r in rows:
        body = r['body'] or ''
        refs = []
        seen = set()
        for m in ISSUE_RE.finditer(body):
            n = int(m.group(1))
            if n in seen:
                continue
            seen.add(n)
            refs.append(n)
        results.append({
            "id": r['id'],
            "title": r['title'],
            "branch_name": r['branch_name'] or '',
            "started_at": r['started_at'] or r['created_at'],
            "stale_seconds": now - (r['started_at'] or r['created_at']),
            "refs": refs,
            "claim_lock": r['claim_lock'] or '',
            "has_worker_pid": r['worker_pid'] is not None,
            "root_task_id": r['root_task_id'] or '',
        })

    print(json.dumps({"cutoff": cutoff, "now": now, "candidates": results},
                     ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
