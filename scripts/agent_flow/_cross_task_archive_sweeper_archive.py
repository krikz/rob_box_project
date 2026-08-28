#!/usr/bin/env python3
"""Archive helper for cross-task-archive-sweeper.sh.

Выполняет один прямой SQL UPDATE, идентичный тому, что делает
hermes_cli.kanban_db.archive_task:

    UPDATE tasks
    SET status='archived',
        claim_lock=NULL, claim_expires=NULL, worker_pid=NULL
    WHERE id = ? AND status != 'archived'

Это намеренный BYPASS worker-scope-limit (см. ADR-0024 / ретро 22.08
t_d9b4c600): вызывающий скрипт запускается от операторского cron-процесса,
НЕ от worker с claim_lock, поэтому kernel-scope не применяется.

Вызывающий скрипт cross-task-archive-sweeper.sh ПЕРЕД этим helper'ом
уже проверил:
  - assignee=devops (allowlist профиля)
  - board=robbox (allowlist board'а)
  - в body есть хотя бы один #NNNN (presence of refs)
  - в gh PR/issue закрыт (state in {MERGED, CLOSED})
  - remote-ветка удалена ИЛИ у карточки нет branch_name (orphan-state снят)
  - DRY_RUN=0 (явное разрешение на мутацию)

Так что этот UPDATE — последний шаг enforcement.

Usage:
  _cross_task_archive_sweeper_archive.py KANBAN_DB TASK_ID

Output (stdout):
  "ARCHIVED <id>"           — row updated (rowcount==1)
  "SKIP  <id> (rowcount=N)" — row not updated (status уже archived или id нет)

Exit codes:
  0 — ok (включая SKIP)
  1 — fatal (DB missing/unreadable)
"""
import sqlite3
import sys


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: _cross_task_archive_sweeper_archive.py KANBAN_DB TASK_ID",
              file=sys.stderr)
        return 2
    db_path, task_id = sys.argv[1], sys.argv[2]

    conn = sqlite3.connect(db_path)
    try:
        cur = conn.cursor()
        cur.execute(
            "UPDATE tasks SET status='archived', "
            "claim_lock=NULL, claim_expires=NULL, worker_pid=NULL "
            "WHERE id=? AND status != 'archived'",
            (task_id,),
        )
        if cur.rowcount == 1:
            conn.commit()
            print(f"ARCHIVED {task_id}")
            return 0
        # rowcount == 0 — статус уже archived или id отсутствует.
        # Не считаем ошибкой; id-empotent retry-friendly.
        print(f"SKIP  {task_id} (rowcount={cur.rowcount}, status may already be archived)")
        return 0
    except sqlite3.OperationalError as e:
        print(f"DB update failed: {e}", file=sys.stderr)
        return 1
    finally:
        conn.close()


if __name__ == "__main__":
    sys.exit(main())
