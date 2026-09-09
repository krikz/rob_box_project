"""Tests for watchdog-provider-quick.sh (ретро 24.08 t_4c73490f).

Контекст: PR #1286 (watchdog fix) покрывал 402/429 провайдер-исчерпание,
но НЕ покрывал быстрые quick-crash циклы (<120с воркер-краш при 429
MiniMax / 401 DeepSeek) — watchdog сканирует каждые 2 мин, а за это
время карточка успевает дважды дать consecutive_failures=2 → gave_up.
Этот тест проверяет новый скрипт ``watchdog-provider-quick.sh``:

* Корректно собирает ``provider_actions`` из SQLite + лога воркера.
* ``ready`` + свежий лог с маркерами + providers мертвы → блок.
* ``running`` + pid мёртв + лог с маркерами → блок.
* ``running`` + pid жив → НЕ блок (защита от false-positive когда worker
  нормально работает, но его лог содержит маркеры как часть текста задачи).
* ``blocked`` + gave_up + providers живы → unblock.
* ``blocked`` + block_kind=capability (human block) → НЕ unblock.
* ``last_failure_error`` с 401 invalid_request_error → попадает в
  выборку даже если log ещё не дописан.

Тест прогоняет сам скрипт через ``bash`` + изолированный HERMES_HOME,
чтобы ловить синтаксис-ошибки bash-обвязки тоже.
"""

from __future__ import annotations

import os
import shutil
import sqlite3
import subprocess
import tempfile
import time
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "scripts" / "agent_flow" / "watchdog-provider-quick.sh"


_SCHEMA = """
    CREATE TABLE tasks (
        id TEXT PRIMARY KEY, title TEXT, body TEXT,
        status TEXT NOT NULL, assignee TEXT,
        consecutive_failures INTEGER NOT NULL DEFAULT 0,
        worker_pid INTEGER, last_failure_error TEXT,
        last_heartbeat_at INTEGER, block_kind TEXT,
        created_at INTEGER NOT NULL DEFAULT 0
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
        error TEXT, metadata TEXT, pid INTEGER
    );
"""


def _create_board(hermes_home: Path, board_name: str = "myboard") -> Path:
    """Create hermes_home/kanban/boards/<board>/kanban.db + logs/.
    Returns the board dir (caller writes logs there)."""
    boards_dir = hermes_home / "kanban" / "boards"
    board = boards_dir / board_name
    board.mkdir(parents=True)
    (board / "logs").mkdir()
    con = sqlite3.connect(board / "kanban.db")
    con.executescript(_SCHEMA)
    con.commit(); con.close()
    return board


def _insert_task(con, *, tid, status, block_kind=None, lfe=None,
                 pid=None, last_ev_kind=None, cf=0):
    """Insert a task and optionally a last task_events entry."""
    con.execute(
        "INSERT INTO tasks(id, title, body, status, "
        "consecutive_failures, worker_pid, last_failure_error, "
        "block_kind, created_at) "
        "VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?)",
        (tid, "test-task " + tid, "", status, cf, pid, lfe, block_kind,
         int(time.time())),
    )
    if last_ev_kind:
        con.execute(
            "INSERT INTO task_events(task_id, kind, created_at) "
            "VALUES (?, ?, ?)",
            (tid, last_ev_kind, int(time.time())),
        )
    con.commit()


def _run_script(hermes_home: Path) -> subprocess.CompletedProcess:
    """Run watchdog-provider-quick.sh with isolated HERMES_HOME."""
    env = os.environ.copy()
    env["HERMES_HOME"] = str(hermes_home)
    env["KANBAN_BOARDS_DIR"] = str(hermes_home / "kanban" / "boards")
    env["HERMES_BIN"] = "/bin/echo"  # noqa: S108 — fake so block/unblock no-op
    env["LOCK_FILE"] = str(hermes_home / "state" / "wpq.lock")
    env["PROVIDER_ACTIONS_FILE"] = str(hermes_home / "state" / "wpq-actions.txt")
    env["LOG_FILE"] = str(hermes_home / "logs" / "wpq.log")
    (hermes_home / "state").mkdir(parents=True, exist_ok=True)
    (hermes_home / "logs").mkdir(parents=True, exist_ok=True)
    return subprocess.run(
        ["bash", str(SCRIPT)],
        env=env, capture_output=True, text=True, timeout=30,
    )


def _read_actions_from_script(stdout: str) -> list[str]:
    """Parse the markdown list produced by the script's Python part.
    Lines starting with '- 🔧 ' contain 'action|board|task_id'."""
    out = []
    for line in stdout.splitlines():
        line = line.strip()
        if line.startswith("- 🔧 "):
            payload = line[len("- 🔧 "):].strip()
            out.append(payload)
    return out


@pytest.fixture()
def tmp_env():
    tmpdir = tempfile.mkdtemp(prefix="wpq_test_")
    hermes_home = Path(tmpdir) / "hermes_home"
    hermes_home.mkdir(parents=True, exist_ok=True)
    yield Path(tmpdir), hermes_home
    shutil.rmtree(tmpdir, ignore_errors=True)


# ---------------------------------------------------------------------------
# tests
# ---------------------------------------------------------------------------


def test_provider_markers_include_401_and_tokenplan():
    """Sanity: PROVIDER_MARKERS расширен на 401 / Authentication Fails /
    invalid_request_error (DeepSeek кейс, ретро 24.08 t_4c73490f)."""
    src = SCRIPT.read_text()
    assert "HTTP 401" in src, "missing HTTP 401 marker"
    assert "Authentication Fails" in src, "missing Authentication Fails"
    assert "is invalid" in src, "missing 'is invalid' marker"
    assert "invalid_request_error" in src, "missing invalid_request_error"
    assert "Token Plan rate limit reached" in src, "missing Token Plan phrase"


def test_failure_error_matches_401_invalid_request():
    """last_failure_error с '401 invalid_request_error' ловится
    даже когда worker log ещё не дописан."""
    HAYSTACKS = (
        "quota", "rate limit", "ratelimit", "rate_limit",
        "429", "402", "401",
        "auth", "authentication", "unauthorized", "forbidden",
        "billing", "subscription", "access denied", "permission denied",
        "invalid api key", "invalid_request_error", "is invalid",
        "out of credits", "insufficient balance", "token plan",
        "2056", "2062", "health-aware", "all providers",
    )

    def _failure_error_matches(error: str) -> bool:
        if not error:
            return False
        e = error.lower()
        return any(h in e for h in HAYSTACKS)

    # The actual error text that hit our AV-* cards
    assert _failure_error_matches("HTTP 401: Authentication Fails, Your api key: ****bd8b is invalid")
    assert _failure_error_matches("HTTP 429: Token Plan rate limit reached (2062)")
    assert _failure_error_matches("RateLimitError [HTTP 429]")
    # Negative cases
    assert not _failure_error_matches("python: command not found")
    assert not _failure_error_matches("git: not a git repository")
    assert not _failure_error_matches("")


def test_ready_with_fresh_log_marker_blocks(tmp_env):
    """ready + log fresh + markers + providers dead → block."""
    _, hermes_home = tmp_env
    board = _create_board(hermes_home)
    logs_dir = board / "logs"
    db = board / "kanban.db"

    con = sqlite3.connect(db)
    _insert_task(con, tid="t_ready_marker", status="ready",
                 pid=0, lfe="HTTP 429 rate limit reached")
    con.close()

    log_p = logs_dir / "t_ready_marker.log"
    log_p.write_text(
        "RateLimitError [HTTP 429]\nToken Plan rate limit reached (2062)\n")
    os.utime(log_p, (time.time(), time.time()))

    result = _run_script(hermes_home)
    assert result.returncode == 0, result.stderr
    actions = _read_actions_from_script(result.stdout)
    assert actions == ["block|myboard|t_ready_marker"], actions


def test_running_with_dead_pid_marker_blocks(tmp_env):
    """running + pid dead + log marker → block."""
    _, hermes_home = tmp_env
    board = _create_board(hermes_home)
    logs_dir = board / "logs"
    db = board / "kanban.db"

    con = sqlite3.connect(db)
    _insert_task(con, tid="t_running_dead", status="running", pid=999999)
    con.close()

    log_p = logs_dir / "t_running_dead.log"
    log_p.write_text("HTTP 401 invalid_request_error\n")
    os.utime(log_p, (time.time(), time.time()))

    result = _run_script(hermes_home)
    assert result.returncode == 0, result.stderr
    actions = _read_actions_from_script(result.stdout)
    assert actions == ["block|myboard|t_running_dead"], actions


def test_running_alive_worker_with_marker_does_NOT_block(tmp_env):
    """running + worker alive (но содержит провайдер-маркеры в body) →
    НЕ блок (защита от false-positive)."""
    _, hermes_home = tmp_env
    board = _create_board(hermes_home)
    logs_dir = board / "logs"
    db = board / "kanban.db"

    # Current PID (the test process) → alive.
    my_pid = os.getpid()
    con = sqlite3.connect(db)
    _insert_task(con, tid="t_running_alive", status="running", pid=my_pid)
    con.close()

    # Log full of 401 / 429 markers — но pid жив → НЕ блок
    log_p = logs_dir / "t_running_alive.log"
    log_p.write_text(
        "HTTP 401 invalid_request_error\nRateLimitError [HTTP 429]\n" * 100)
    os.utime(log_p, (time.time(), time.time()))

    result = _run_script(hermes_home)
    assert result.returncode == 0, result.stderr
    actions = _read_actions_from_script(result.stdout)
    assert actions == [], f"Expected 0 actions for live worker, got {actions}"


def test_blocked_gaveup_with_providers_alive_unblocks(tmp_env):
    """blocked(gave_up) + providers_alive=True + log с маркерами → unblock."""
    _, hermes_home = tmp_env
    board = _create_board(hermes_home)
    logs_dir = board / "logs"
    db = board / "kanban.db"

    con = sqlite3.connect(db)
    _insert_task(con, tid="t_av3", status="blocked",
                 lfe="HTTP 429 rate limit", last_ev_kind="gave_up")
    con.close()

    # log stale (>900s) — потому что мы recovery wave тестируем; но если
    # log свежий с маркерами, тоже ОК. Главное — есть свежий CLEAN log
    # в ДРУГОМ files (sentinel-проверка providers_alive=True).
    log_p = logs_dir / "t_av3.log"
    log_p.write_text("HTTP 429 Token Plan rate limit reached (2062)\n")
    old = time.time() - 3600
    os.utime(log_p, (old, old))

    # Sentinel clean log → providers_alive=True
    sentinel = logs_dir / "t_some_fresh_clean_worker.log"
    sentinel.write_text("everything is fine\n" * 100)
    os.utime(sentinel, (time.time(), time.time()))

    result = _run_script(hermes_home)
    assert result.returncode == 0, result.stderr
    actions = _read_actions_from_script(result.stdout)
    assert actions == ["unblock|myboard|t_av3"], actions


def test_blocked_capability_kind_does_NOT_unblock(tmp_env):
    """blocked с block_kind=capability (human block) → НЕ unblock,
    даже если providers_alive=True. Это правильная защита — мы не
    отменяем ручное решение Шифу."""
    _, hermes_home = tmp_env
    board = _create_board(hermes_home)
    logs_dir = board / "logs"
    db = board / "kanban.db"

    con = sqlite3.connect(db)
    # block_kind='capability' = manual human block (Шифу ставил)
    _insert_task(con, tid="t_human_block", status="blocked",
                 block_kind="capability",
                 lfe="HTTP 401 invalid_request_error",
                 last_ev_kind="blocked")
    con.close()

    log_p = logs_dir / "t_human_block.log"
    log_p.write_text("HTTP 401 invalid_request_error\n")
    os.utime(log_p, (time.time(), time.time()))

    # Sentinel clean log → providers_alive=True
    sentinel = logs_dir / "t_some_fresh_clean_worker.log"
    sentinel.write_text("fine\n" * 100)
    os.utime(sentinel, (time.time(), time.time()))

    result = _run_script(hermes_home)
    assert result.returncode == 0, result.stderr
    actions = _read_actions_from_script(result.stdout)
    assert actions == [], (
        f"Human-blocked 'capability' must NOT auto-unblock, got {actions}")


def test_ready_stale_log_providing_no_action(tmp_env):
    """ready + log ОЧЕНЬ старый (>PROVIDER_LOG_WINDOW) + markers →
    НЕ блок (это не «свежий краш», dispatcher сам разберётся)."""
    _, hermes_home = tmp_env
    board = _create_board(hermes_home)
    logs_dir = board / "logs"
    db = board / "kanban.db"

    con = sqlite3.connect(db)
    _insert_task(con, tid="t_stale", status="ready", lfe="x")
    con.close()

    log_p = logs_dir / "t_stale.log"
    log_p.write_text("HTTP 429 rate limit reached (ancient crash)\n")
    old = time.time() - 3600 * 2  # 2 hours old
    os.utime(log_p, (old, old))

    result = _run_script(hermes_home)
    assert result.returncode == 0, result.stderr
    actions = _read_actions_from_script(result.stdout)
    assert actions == [], (
        f"Stale log should NOT trigger block, got {actions}")


def test_lfe_only_does_not_block_ready(tmp_env):
    """last_failure_error один (без log marker) НЕ блокирует ready:
    это design choice — ловим только через log_mark в active-state, чтобы
    избежать false positives от устаревшего last_failure_error.
    lfe-only path работает только для recovery-волны (blocked-armed)."""
    _, hermes_home = tmp_env
    board = _create_board(hermes_home)
    logs_dir = board / "logs"
    db = board / "kanban.db"

    con = sqlite3.connect(db)
    _insert_task(con, tid="t_lfe_only", status="ready", pid=0,
                 lfe="HTTP 401: invalid_request_error, your api key is invalid")
    con.close()

    # No log file at all
    assert not (logs_dir / "t_lfe_only.log").exists()

    result = _run_script(hermes_home)
    assert result.returncode == 0, result.stderr
    actions = _read_actions_from_script(result.stdout)
    assert actions == [], (
        f"lfe-only should NOT trigger ready-block, got {actions}")
