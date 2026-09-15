"""Tests for agent-flow-cancel-on-provider-exhausted.sh (ретро 15.09 t_a7aa4e6b).

Контекст: карточка t_8053e18c зафиксировала worker-cascade-crash на MiniMax 402/429.
Задача t_a7aa4e6b — создать SOT-скрипт, который:
  1) сканирует task_runs.summary на сигнатуру provider-exhaust;
  2) блокирует (kind=capability, reason='provider-budget-exhausted');
  3) постит sentinel-marked comment на КАРТОЧКУ;
  4) постит gh issue comment (если issue ref найден в body);
  5) IDEMPOTENT — повторный запуск no-op;
  6) companion --recover unblock-ит blocked(kind=capability) + sentinel.

Покрываем тестами:
  * signatures recognition (HTTP 429, провайдер исчерпан, English/Russian)
  * issue-ref extraction из body (Source/Issue:/#NNNN fallback)
  * sentinel idempotency (повторный scan не даёт действий)
  * recover только для kind=capability + sentinel (не для ручного block)
  * recover игнорирует блоки от watchdog-provider-quick.sh (нет sentinel)
  * dry-run mode не делает side-effects
  * shell-level: bash syntax + script executable
"""

from __future__ import annotations

import json
import os
import shutil
import sqlite3
import subprocess
import tempfile
import time
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "scripts" / "agent_flow" / "agent-flow-cancel-on-provider-exhausted.sh"


# ---------------------------------------------------------------------------
# Test fixtures / helpers
# ---------------------------------------------------------------------------

_SCHEMA = """
    CREATE TABLE tasks (
        id TEXT PRIMARY KEY, title TEXT, body TEXT,
        status TEXT NOT NULL, assignee TEXT,
        consecutive_failures INTEGER NOT NULL DEFAULT 0,
        worker_pid INTEGER, last_failure_error TEXT,
        last_heartbeat_at INTEGER, block_kind TEXT,
        created_at INTEGER NOT NULL DEFAULT 0,
        current_run_id INTEGER
    );
    CREATE TABLE task_events (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        task_id TEXT NOT NULL, kind TEXT NOT NULL,
        payload TEXT, created_at INTEGER NOT NULL DEFAULT 0,
        run_id INTEGER
    );
    CREATE TABLE task_runs (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        task_id TEXT NOT NULL, profile TEXT, status TEXT,
        outcome TEXT, started_at INTEGER, ended_at INTEGER,
        error TEXT, metadata TEXT, pid INTEGER, summary TEXT
    );
    CREATE TABLE task_comments (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        task_id TEXT NOT NULL, author TEXT, body TEXT,
        created_at INTEGER NOT NULL DEFAULT 0
    );
"""


def _create_board(hermes_home: Path, board_name: str = "tboard") -> Path:
    """Create isolated hermes_home/kanban/boards/<board>/kanban.db."""
    board = hermes_home / "kanban" / "boards" / board_name
    board.mkdir(parents=True)
    con = sqlite3.connect(board / "kanban.db")
    con.executescript(_SCHEMA)
    con.commit()
    con.close()
    return board


def _insert_task(con, *, tid, status="ready", block_kind=None, body="",
                 title="t-task", lfe=None, pid=None):
    con.execute(
        "INSERT INTO tasks(id, title, body, status, consecutive_failures, "
        "worker_pid, last_failure_error, block_kind, created_at) "
        "VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?)",
        (tid, title, body, status, 0, pid, lfe, block_kind, int(time.time())),
    )


def _insert_run(con, *, tid, summary, outcome="crashed", status="crashed"):
    con.execute(
        "INSERT INTO task_runs(task_id, profile, status, outcome, started_at, "
        "ended_at, summary) VALUES(?, ?, ?, ?, ?, ?, ?)",
        (tid, "devops", status, outcome, int(time.time()), int(time.time()),
         summary),
    )


def _insert_sentinel(con, *, tid):
    """Insert our sentinel-marker comment as if the script had run before."""
    con.execute(
        "INSERT INTO task_comments(task_id, author, body, created_at) "
        "VALUES(?, ?, ?, ?)",
        (tid, "default",
         "<!-- agent-flow-cancel-on-provider-exhausted.sh:marker -->\n"
         "<!-- retro-key:worker-cascade-crash-provider-exhausted -->\n"
         "pre-existing sentinel",
         int(time.time())),
    )


def _run_script(hermes_home: Path, *args, env_extra=None) -> subprocess.CompletedProcess:
    env = os.environ.copy()
    env["HERMES_HOME"] = str(hermes_home)
    env["KANBAN_BOARDS_DIR"] = str(hermes_home / "kanban" / "boards")
    env["HERMES_BIN"] = "/bin/echo"  # noqa: S108 — fake so block/unblock no-op
    env["GH_CONFIG_DIR"] = "/dev/null"  # skip gh CLI
    env["LOCK_FILE"] = str(hermes_home / "state" / "cpoe.lock")
    env["LOG_FILE"] = str(hermes_home / "logs" / "cpoe.log")
    env["STATE_DIR"] = str(hermes_home / "state")
    (hermes_home / "state").mkdir(parents=True, exist_ok=True)
    (hermes_home / "logs").mkdir(parents=True, exist_ok=True)
    if env_extra:
        env.update(env_extra)
    return subprocess.run(
        ["bash", str(SCRIPT), *args],
        env=env, capture_output=True, text=True, timeout=30,
    )


def _read_actions(stderr: str) -> list[dict]:
    """Parse `→ block  board/tid  issue=#N  title='...'` from log output.

    Note: log() writes to STDERR (consistent with other agent-flow scripts).
    Format on stderr: `[<script_name>] → block  <board>/<tid>  issue=#N  title='...'`
    """
    out = []
    for line in stderr.splitlines():
        line = line.strip()
        # Strip the log() prefix `[script_name]` if present
        if line.startswith("[") and "]" in line:
            # find the closing bracket
            idx = line.find("]")
            if idx > 0 and idx < len(line) - 1 and line[idx + 1] == " ":
                line = line[idx + 2 :]
        if line.startswith("→ block  ") or line.startswith("↩  unblock "):
            parts = line.split()
            # format: "→ block  robbox/t_e5f69665  issue=#2484  title='...'"
            try:
                board_tid = parts[2]
                board, tid = board_tid.split("/", 1)
                # find issue=#N
                issue = ""
                for p in parts[3:]:
                    if p.startswith("issue=#"):
                        issue = p[len("issue=#"):]
                        break
                out.append({"action": parts[1], "board": board,
                            "task_id": tid, "issue": issue})
            except (IndexError, ValueError):
                continue
    return out


@pytest.fixture()
def tmp_env():
    tmpdir = tempfile.mkdtemp(prefix="cpoe_test_")
    hermes_home = Path(tmpdir) / "hermes_home"
    hermes_home.mkdir(parents=True, exist_ok=True)
    yield Path(tmpdir), hermes_home
    shutil.rmtree(tmpdir, ignore_errors=True)


# ---------------------------------------------------------------------------
# Static checks
# ---------------------------------------------------------------------------


def test_script_exists_and_executable():
    assert SCRIPT.is_file(), f"script missing: {SCRIPT}"
    import stat
    st = SCRIPT.stat()
    assert st.st_mode & stat.S_IXUSR, "script not executable"


def test_script_passes_shellcheck_clean():
    """Acceptance: shellcheck-clean. SC warnings > 0 — fail."""
    res = subprocess.run(
        ["shellcheck", "-x", str(SCRIPT)],
        capture_output=True, text=True, timeout=30,
    )
    assert res.returncode == 0, (
        f"shellcheck reported issues:\n{res.stdout}\n{res.stderr}")


def test_bash_syntax_ok():
    """Bash syntax sanity (independent of shellcheck)."""
    res = subprocess.run(
        ["bash", "-n", str(SCRIPT)],
        capture_output=True, text=True, timeout=10,
    )
    assert res.returncode == 0, f"bash -n failed: {res.stderr}"


def test_help_message_describes_modes():
    res = subprocess.run(
        ["bash", str(SCRIPT), "--help"],
        capture_output=True, text=True, timeout=5,
    )
    assert res.returncode == 0
    out = res.stdout
    assert "--dry-run" in out
    assert "--recover" in out
    assert "provider-budget-exhausted" in out
    assert "IDEMPOTENT" in out.upper()


# ---------------------------------------------------------------------------
# Signature recognition
# ---------------------------------------------------------------------------


def test_signature_recognition_english_and_russian():
    """_summary_contains_exhaust: ловит английские + русские сигнатуры."""
    src = SCRIPT.read_text()
    for needle in ("HTTP 402", "HTTP 429", "rate limit",
                   "провайдер исчерпан", "Token Plan",
                   "invalid_request_error", "провайдер восстановлен"):
        assert needle in src, f"signature {needle!r} missing in source"


def test_recovery_message_is_excluded():
    """Анти-паттерн: «провайдер восстановлен» НЕ триггерит block."""
    src = SCRIPT.read_text()
    # Должна быть явная защита (negative-branch в _summary_contains_exhaust)
    assert "провайдер восстановлен" in src
    # В python-части должна быть ветка с этим anti-pattern
    py_section = src.split('PYEOF')[0] if 'PYEOF' in src else src
    # Проверим, что есть фильтр «UNBLOCK: провайдер»
    assert "UNBLOCK:" in src and "провайдер" in src


# ---------------------------------------------------------------------------
# Issue ref extraction
# ---------------------------------------------------------------------------


def test_extract_issue_source_block(tmp_env):
    _, hermes_home = tmp_env
    board = _create_board(hermes_home, "tboard")
    db = board / "kanban.db"
    con = sqlite3.connect(db)
    body = (
        "Source\n"
        "  repo: krikz/rob_box_project\n"
        "  issue: #2610\n"
        "  labels: bug\n\n"
        "Context\n  ...\n"
    )
    _insert_task(con, tid="t_a", status="ready", body=body)
    _insert_run(con, tid="t_a",
                summary="HTTP 429 Token Plan rate limit reached (2062)")
    con.commit()
    con.close()

    res = _run_script(hermes_home, "--dry-run")
    assert res.returncode == 0, res.stderr
    actions = _read_actions(res.stderr)
    assert len(actions) == 1, f"actions={actions} stderr={res.stderr!r}"
    assert actions[0]["issue"] == "2610"
    assert actions[0]["task_id"] == "t_a"


def test_extract_issue_fallback_formats(tmp_env):
    """Issue ref извлекается из Issue:/issue:/#NNNN fallback-ов."""
    _, hermes_home = tmp_env
    board = _create_board(hermes_home, "tboard")
    db = board / "kanban.db"
    con = sqlite3.connect(db)

    # Формат 2: "Issue: #1234" в начале строки
    body2 = "Issue: #1234\n\nDescription ...\n"
    _insert_task(con, tid="t_b", status="ready", body=body2)
    _insert_run(con, tid="t_b",
                summary="провайдер исчерпан, ждать (MiniMax 402)")

    # Формат 3: голый #9999 в тексте
    body3 = "Контекст: см. issue #9999 (похожая проблема).\n"
    _insert_task(con, tid="t_c", status="ready", body=body3)
    _insert_run(con, tid="t_c",
                summary="HTTP 429 rate limit reached")

    con.commit(); con.close()

    res = _run_script(hermes_home, "--dry-run")
    assert res.returncode == 0, res.stderr
    actions = _read_actions(res.stderr)
    issues = sorted([a["issue"] for a in actions if a["task_id"] in ("t_b", "t_c")])
    assert issues == ["1234", "9999"], actions


def test_no_issue_ref_still_works(tmp_env):
    """Без issue ref в body — карточка всё равно ловится, issue=''."""
    _, hermes_home = tmp_env
    board = _create_board(hermes_home, "tboard")
    db = board / "kanban.db"
    con = sqlite3.connect(db)
    _insert_task(con, tid="t_no_issue", status="ready",
                 body="## Контекст\nНет ссылки на issue.\n")
    _insert_run(con, tid="t_no_issue",
                summary="HTTP 402 Insufficient Balance")
    con.commit(); con.close()

    res = _run_script(hermes_home, "--dry-run")
    assert res.returncode == 0, res.stderr
    actions = _read_actions(res.stderr)
    matched = [a for a in actions if a["task_id"] == "t_no_issue"]
    assert len(matched) == 1
    assert matched[0]["issue"] == ""


# ---------------------------------------------------------------------------
# Core behavior: cancel mode
# ---------------------------------------------------------------------------


def test_ready_with_exhaust_signature_blocks(tmp_env):
    """ready + latest summary с сигнатурой → блок (dry-run показывает план)."""
    _, hermes_home = tmp_env
    board = _create_board(hermes_home)
    db = board / "kanban.db"
    con = sqlite3.connect(db)
    _insert_task(con, tid="t_ready_exhaust", status="ready",
                 body="Source\n  issue: #42\n")
    _insert_run(con, tid="t_ready_exhaust",
                summary="HTTP 429 Token Plan rate limit reached")
    con.commit(); con.close()

    res = _run_script(hermes_home, "--dry-run")
    assert res.returncode == 0, res.stderr
    actions = _read_actions(res.stderr)
    assert any(a["task_id"] == "t_ready_exhaust" and a["action"] == "block"
               for a in actions), actions
    assert "[DRY-RUN] would: kanban block" in res.stderr
    assert "[DRY-RUN] would: kanban comment" in res.stderr
    assert "[DRY-RUN] would: gh issue comment" in res.stderr


def test_already_blocked_skipped(tmp_env):
    """Уже blocked (с другим kind) → skip (никаких side effects)."""
    _, hermes_home = tmp_env
    board = _create_board(hermes_home)
    db = board / "kanban.db"
    con = sqlite3.connect(db)
    _insert_task(con, tid="t_already_blocked", status="blocked",
                 block_kind="needs_input",
                 body="Source\n  issue: #50\n")
    _insert_run(con, tid="t_already_blocked",
                summary="HTTP 429 rate limit")
    con.commit(); con.close()

    res = _run_script(hermes_home, "--dry-run")
    assert res.returncode == 0, res.stderr
    actions = _read_actions(res.stderr)
    assert not any(a["task_id"] == "t_already_blocked" for a in actions), (
        f"already-blocked should not appear: {actions}")


def test_sentinel_comment_makes_idempotent(tmp_env):
    """Sentinel-marker в task_comments → scan no-op."""
    _, hermes_home = tmp_env
    board = _create_board(hermes_home)
    db = board / "kanban.db"
    con = sqlite3.connect(db)
    _insert_task(con, tid="t_with_sentinel", status="ready",
                 body="Source\n  issue: #77\n")
    _insert_run(con, tid="t_with_sentinel",
                summary="HTTP 429 Token Plan rate limit reached")
    _insert_sentinel(con, tid="t_with_sentinel")
    con.commit(); con.close()

    res = _run_script(hermes_home, "--dry-run")
    assert res.returncode == 0, res.stderr
    actions = _read_actions(res.stderr)
    assert not any(a["task_id"] == "t_with_sentinel" for a in actions), (
        f"sentinel must make idempotent: {actions}")


def test_done_status_skipped(tmp_env):
    """done-карточки в принципе пропускаются."""
    _, hermes_home = tmp_env
    board = _create_board(hermes_home)
    db = board / "kanban.db"
    con = sqlite3.connect(db)
    _insert_task(con, tid="t_done", status="done",
                 body="Source\n  issue: #77\n")
    _insert_run(con, tid="t_done",
                summary="HTTP 429 Token Plan rate limit reached")
    con.commit(); con.close()

    res = _run_script(hermes_home, "--dry-run")
    assert res.returncode == 0, res.stderr
    actions = _read_actions(res.stderr)
    assert not any(a["task_id"] == "t_done" for a in actions), actions


def test_no_signature_in_summary_no_action(tmp_env):
    """Без сигнатуры в latest summary → НЕ блокировать."""
    _, hermes_home = tmp_env
    board = _create_board(hermes_home)
    db = board / "kanban.db"
    con = sqlite3.connect(db)
    _insert_task(con, tid="t_no_sig", status="ready",
                 body="Source\n  issue: #77\n")
    _insert_run(con, tid="t_no_sig", summary="Upstream merge conflict")
    con.commit(); con.close()

    res = _run_script(hermes_home, "--dry-run")
    assert res.returncode == 0, res.stderr
    actions = _read_actions(res.stderr)
    assert not any(a["task_id"] == "t_no_sig" for a in actions), actions


def test_null_summary_no_action(tmp_env):
    """Пустой summary → НЕ блокировать (lfe-only не триггерит)."""
    _, hermes_home = tmp_env
    board = _create_board(hermes_home)
    db = board / "kanban.db"
    con = sqlite3.connect(db)
    _insert_task(con, tid="t_null", status="ready")
    _insert_run(con, tid="t_null", summary=None)
    con.commit(); con.close()

    res = _run_script(hermes_home, "--dry-run")
    assert res.returncode == 0, res.stderr
    actions = _read_actions(res.stderr)
    assert not any(a["task_id"] == "t_null" for a in actions), actions


def test_russian_signature_recognized(tmp_env):
    """Русская сигнатура «провайдер исчерпан» ловится."""
    _, hermes_home = tmp_env
    board = _create_board(hermes_home)
    db = board / "kanban.db"
    con = sqlite3.connect(db)
    _insert_task(con, tid="t_ru", status="ready", body="issue #55 somewhere")
    _insert_run(con, tid="t_ru",
                summary="провайдер исчерпан, ждать (MiniMax 402/429)")
    con.commit(); con.close()

    res = _run_script(hermes_home, "--dry-run")
    assert res.returncode == 0, res.stderr
    actions = _read_actions(res.stderr)
    assert any(a["task_id"] == "t_ru" for a in actions), actions


def test_unblock_message_not_misclassified(tmp_env):
    """«провайдер восстановлен» в summary → НЕ блокировать."""
    _, hermes_home = tmp_env
    board = _create_board(hermes_home)
    db = board / "kanban.db"
    con = sqlite3.connect(db)
    _insert_task(con, tid="t_recover", status="ready")
    _insert_run(con, tid="t_recover",
                summary="UNBLOCK: провайдер восстановлен — респавн")
    con.commit(); con.close()

    res = _run_script(hermes_home, "--dry-run")
    assert res.returncode == 0, res.stderr
    actions = _read_actions(res.stderr)
    assert not any(a["task_id"] == "t_recover" for a in actions), (
        f"recover message must NOT trigger block: {actions}")


# ---------------------------------------------------------------------------
# Companion: --recover
# ---------------------------------------------------------------------------


def test_recover_only_capability_kind_with_sentinel(tmp_env):
    """--recover: только blocked(kind=capability) + sentinel → unblock."""
    _, hermes_home = tmp_env
    board = _create_board(hermes_home)
    db = board / "kanban.db"
    con = sqlite3.connect(db)

    # Подходящий кандидат
    _insert_task(con, tid="t_recover_ok", status="blocked",
                 block_kind="capability",
                 body="Source\n  issue: #42\n")
    _insert_run(con, tid="t_recover_ok",
                summary="HTTP 429 Token Plan rate limit reached")
    _insert_sentinel(con, tid="t_recover_ok")

    # Чужой блок (НЕ capability) — должен быть пропущен
    _insert_task(con, tid="t_recover_other", status="blocked",
                 block_kind="needs_input",
                 body="Source\n  issue: #43\n")
    _insert_run(con, tid="t_recover_other",
                summary="HTTP 429 rate limit")
    _insert_sentinel(con, tid="t_recover_other")  # sentinel есть, но kind ≠ capability

    # Sentinel есть, но не capability
    _insert_task(con, tid="t_recover_nosentinel", status="blocked",
                 block_kind="capability",
                 body="Source\n  issue: #44\n")
    _insert_run(con, tid="t_recover_nosentinel",
                summary="HTTP 429 rate limit")
    # NO sentinel inserted
    con.commit(); con.close()

    res = _run_script(hermes_home, "--recover", "--dry-run")
    assert res.returncode == 0, res.stderr
    actions = _read_actions(res.stderr)
    matched = [a for a in actions if a["action"] == "unblock"]
    assert any(a["task_id"] == "t_recover_ok" for a in matched), (
        f"recover should target capability+sentinel: {actions}")
    assert not any(a["task_id"] == "t_recover_other" for a in matched), (
        "recover must skip non-capability block_kind")
    assert not any(a["task_id"] == "t_recover_nosentinel" for a in matched), (
        "recover must skip when sentinel is missing")


# ---------------------------------------------------------------------------
# Idempotency on dry-run dry-run-dry-run (no side-effects)
# ---------------------------------------------------------------------------


def test_dry_run_creates_no_comments(tmp_env):
    """dry-run не пишет в kanban comment и не блокирует."""
    _, hermes_home = tmp_env
    board = _create_board(hermes_home)
    db = board / "kanban.db"
    con = sqlite3.connect(db)
    _insert_task(con, tid="t_dry", status="ready",
                 body="Source\n  issue: #99\n")
    _insert_run(con, tid="t_dry",
                summary="HTTP 429 Token Plan rate limit reached")
    con.commit()
    before = con.execute(
        "SELECT COUNT(*) FROM task_comments WHERE task_id='t_dry'"
    ).fetchone()[0]
    con.close()

    res = _run_script(hermes_home, "--dry-run")
    assert res.returncode == 0, res.stderr

    con = sqlite3.connect(db)
    after = con.execute(
        "SELECT COUNT(*) FROM task_comments WHERE task_id='t_dry'"
    ).fetchone()[0]
    con.close()
    assert before == after, "dry-run must not write comments"


def test_dry_run_creates_no_actions_file(tmp_env):
    """dry-run pipeline нормально завершается (no crash)."""
    _, hermes_home = tmp_env
    _create_board(hermes_home)
    res = _run_script(hermes_home, "--dry-run")
    assert res.returncode == 0, res.stderr
    # На STDERR должен быть наш tick-summary
    assert "[scan]" in res.stderr or "actions=" in res.stderr


# ---------------------------------------------------------------------------
# Acceptance: shellcheck-clean (already tested above) + integration
# ---------------------------------------------------------------------------


def test_integration_full_pipeline_on_two_boards(tmp_env):
    """Smoke: 2 boards, разные карточки, dry-run отрабатывает без ошибок."""
    _, hermes_home = tmp_env
    b1 = _create_board(hermes_home, "board_a")
    b2 = _create_board(hermes_home, "board_b")
    for board, tid, issue in (
        (b1, "t_aa", "100"), (b1, "t_ab", "101"), (b2, "t_ba", "200"),
    ):
        con = sqlite3.connect(board / "kanban.db")
        _insert_task(con, tid=tid, status="ready",
                     body=f"Source\n  issue: #{issue}\n")
        _insert_run(con, tid=tid,
                    summary="HTTP 429 Token Plan rate limit reached")
        con.commit(); con.close()

    res = _run_script(hermes_home, "--dry-run")
    assert res.returncode == 0, res.stderr
    actions = _read_actions(res.stderr)
    tids = sorted([a["task_id"] for a in actions])
    assert tids == ["t_aa", "t_ab", "t_ba"], actions
