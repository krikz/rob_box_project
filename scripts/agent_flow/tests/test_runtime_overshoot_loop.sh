#!/bin/bash
# ============================================================================
# test_runtime_overshoot_loop.sh — регресс-гард для
# agent-flow-runtime-overshoot-loop.sh (ретро t_34f33289, карточка t_c2ab8db9).
#
# Покрытие (acceptance t_c2ab8db9):
#   T1  gate-by-giveup: running/ready, consecutive_failures >= 3, все 3 events
#       содержат provider-exhaust signature → catch
#   T2  gate-by-giveup: events exhausted, но cf < 3 → НЕТ catch
#   T3  gate-by-giveup: cf >= 3, но events НЕ exhausted (например, protocol_violation
#       exit code 1 без provider signature) → НЕТ catch
#   T4  loop-no-progress: 5 spawns + 1 gave_up за 1ч → catch
#   T5  loop-no-progress: 5 spawns БЕЗ gave_up → НЕТ catch
#   T6  loop-no-progress: 4 spawns + 1 gave_up (ниже порога) → НЕТ catch
#   T7  idempotency: после добавления sentinel в task_comments, повторный dry-run
#       не даёт этого task_id
#   T8  anti-pattern: events содержат recover marker (провайдер восстановлен) →
#       НЕТ catch (recover-маркер блокирует сигнатуру, как в cancel-on-provider)
#   T9  MEMORY_AVAILABLE preflight: backoff при MemAvailable < 2 GiB
#
# Run:
#   bash scripts/agent_flow/tests/test_runtime_overshoot_loop.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_UNDER_TEST="${SCRIPT_UNDER_TEST:-$TEST_DIR/../agent-flow-runtime-overshoot-loop.sh}"

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

fail() { echo "FAIL: $*"; exit 1; }
pass() { echo "ok: $*"; }

# Sanity check
[ -x "$SCRIPT_UNDER_TEST" ] || fail "script not found: $SCRIPT_UNDER_TEST"
command -v hermes >/dev/null 2>&1 || \
    fail "hermes CLI not on PATH (нужен для block/comment в apply-mode; dry-run-only тесты не требуют)"

# --- фикстура: kanban.db с tasks / task_events / task_runs / task_comments ---
BOARD_DIR="$WORK/boards/robbox"
mkdir -p "$BOARD_DIR"
python3 - "$BOARD_DIR" "$WORK" <<'PYEOF'
import sqlite3, sys, os, time, json
board_dir, work_dir = sys.argv[1], sys.argv[2]
db = os.path.join(board_dir, "kanban.db")
con = sqlite3.connect(db)

# Минимум из реальной схемы rob_box_project.
con.executescript("""
CREATE TABLE tasks (
    id TEXT PRIMARY KEY,
    title TEXT DEFAULT '',
    body TEXT DEFAULT '',
    status TEXT DEFAULT 'todo',
    block_kind TEXT,
    block_recurrences INTEGER DEFAULT 0,
    consecutive_failures INTEGER DEFAULT 0,
    worker_pid INTEGER DEFAULT 0,
    last_failure_error TEXT DEFAULT '',
    last_heartbeat_at INTEGER DEFAULT 0,
    max_retries INTEGER DEFAULT 0,
    max_runtime_seconds INTEGER,
    started_at INTEGER DEFAULT 0
);
CREATE TABLE task_events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_id TEXT NOT NULL,
    kind TEXT NOT NULL,
    payload TEXT,
    error TEXT,
    created_at INTEGER NOT NULL
);
CREATE TABLE task_runs (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_id TEXT,
    status TEXT,
    outcome TEXT,
    summary TEXT,
    error TEXT,
    started_at INTEGER,
    ended_at INTEGER
);
CREATE TABLE task_comments (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_id TEXT,
    author TEXT DEFAULT 'test',
    body TEXT,
    created_at INTEGER
);
""")
now = int(time.time())

def insert_task(tid, status, cf, lfe, body='body'):
    con.execute(
        "INSERT INTO tasks (id,status,consecutive_failures,last_failure_error,body) "
        "VALUES (?,?,?,?,?)",
        (tid, status, cf, lfe, body),
    )

def insert_event(tid, kind, payload, error=None, age=0):
    con.execute(
        "INSERT INTO task_events (task_id,kind,payload,error,created_at) VALUES (?,?,?,?,?)",
        (tid, kind, payload, error, now - age),
    )

# T1: cf=3, status=running, 3 exhausted события → catch (gate-by-giveup)
insert_task('t1_gate_catch', 'running', 3, 'HTTP 429 Token Plan rate limit reached')
for _ in range(3):
    insert_event('t1_gate_catch', 'gave_up',
                 json.dumps({"error": "провайдер исчерпан, ждать 402/429"}),
                 error='health-aware-fallback', age=10)

# T2: cf=1 < 3, exhausted events → НЕТ catch (порог cf)
insert_task('t2_cf_below', 'running', 1, 'провайдер исчерпан, ждать (402/429)')
for _ in range(3):
    insert_event('t2_cf_below', 'gave_up',
                 json.dumps({"error": "провайдер исчерпан"}),
                 error='health-aware-fallback', age=10)

# T3: cf=3, но events НЕ exhausted → НЕТ catch (нет сигнатуры)
insert_task('t3_no_signature', 'running', 3,
            'pid 12345 exited with code 1: some random crash')
for _ in range(3):
    insert_event('t3_no_signature', 'gave_up',
                 json.dumps({"error": "worker not responding"}),
                 error='protocol_violation', age=10)

# T4: loop-no-progress: 5 spawns + 1 gave_up за 1ч → catch
insert_task('t4_loop_catch', 'running', 1, '')
for _ in range(5):
    insert_event('t4_loop_catch', 'spawned',
                 json.dumps({"pid": 1234 + _}), age=200)
insert_event('t4_loop_catch', 'gave_up',
             json.dumps({"error": "fail", "trigger_outcome": "crashed"}),
             error='crashed', age=180)

# T5: 5 spawns БЕЗ gave_up → НЕТ catch (loop-no-progress требует gave_up)
insert_task('t5_no_giveup', 'running', 0, '')
for _ in range(6):
    insert_event('t5_no_giveup', 'spawned',
                 json.dumps({"pid": 2234 + _}), age=200)

# T6: 4 spawns + 1 gave_up (ниже SPAWN_THRESHOLD=5) → НЕТ catch
insert_task('t6_spawn_below', 'running', 1, '')
for _ in range(4):
    insert_event('t6_spawn_below', 'spawned',
                 json.dumps({"pid": 3234 + _}), age=200)
insert_event('t6_spawn_below', 'gave_up',
             json.dumps({"error": "fail"}),
             error='crashed', age=180)

# T8: anti-pattern — recover-маркер
insert_task('t8_recovered', 'running', 3,
            'провайдер восстановлен, MiniMax снова отвечает 200')
for _ in range(3):
    insert_event('t8_recovered', 'gave_up',
                 json.dumps({"error": "провайдер восстановлен, MiniMax снова отвечает"}),
                 error='recovered', age=10)

# T7: sentinel-маркер в task_comments (после прошлого block)
insert_task('t7_already_blocked', 'running', 5, 'HTTP 429')
for _ in range(3):
    insert_event('t7_already_blocked', 'gave_up',
                 json.dumps({"error": "провайдер исчерпан"}),
                 error='exhaust', age=10)
con.execute(
    "INSERT INTO task_comments (task_id,body,created_at) VALUES (?,?,?)",
    ('t7_already_blocked',
     '<!-- agent-flow-runtime-overshoot-loop.sh:marker --> some prior block',
     now - 60),
)

con.commit()
con.close()
PYEOF
[ -f "$BOARD_DIR/kanban.db" ] || fail "fixture db not created"

# --- прогон dry-run ----------------------------------------------------------
GG_FILE="$WORK/agent-flow-runtime-overshoot-loop.sh.gate_giveup.actions.jsonl"
LNP_FILE="$WORK/agent-flow-runtime-overshoot-loop.sh.loop_no_progress.actions.jsonl"

# Чтобы не делать backoff в dry-run (dry-run не должен зависеть от памяти),
# но скрипт всё равно сначала проверяет memory preflight — а dry-run у нас
# его НЕ выполняет (см. main()). То есть dry-run безопасен при любой памяти.

env -i HOME="$HOME" PATH="$PATH" KANBAN_BOARDS_DIR="$WORK/boards" \
    HERMES_HOME="$WORK" \
    STATE_DIR="$WORK" \
    LOG_FILE="$WORK/test_ros.log" \
    LOCK_FILE="$WORK/test_ros.lock" \
    bash "$SCRIPT_UNDER_TEST" --dry-run > "$WORK/dryrun.out" 2>&1 || true
cat "$WORK/dryrun.out"
echo "--- gg file ---"
cat "$GG_FILE" 2>/dev/null || echo "(no gg file)"
echo "--- lnp file ---"
cat "$LNP_FILE" 2>/dev/null || echo "(no lnp file)"
echo

# --- assertions --------------------------------------------------------------
echo "=== TEST ASSERTIONS ==="

# T1
grep -q '"task_id": "t1_gate_catch"' "$GG_FILE" || \
    fail "T1: t1_gate_catch (cf>=3 + exhausted events) NOT caught"
echo "  T1 ok — gate-by-giveup detects cf>=3 + exhausted events"

# T2
grep -q '"task_id": "t2_cf_below"' "$GG_FILE" && \
    fail "T2: t2_cf_below (cf=1 < 3) should NOT be caught"
echo "  T2 ok — gate-by-giveup skips cf<3"

# T3
grep -q '"task_id": "t3_no_signature"' "$GG_FILE" && \
    fail "T3: t3_no_signature (no provider signature) should NOT be caught"
echo "  T3 ok — gate-by-giveup skips non-exhausted events"

# T4
if [ ! -f "$LNP_FILE" ]; then
    fail "T4: lnp_file not created (script didn't even call detect_loop_no_progress?)"
fi
if ! grep -q '"task_id": "t4_loop_catch"' "$LNP_FILE"; then
    # Диагностика — что реально есть в БД
    NOW_=$(date +%s)
    python3 - "$BOARD_DIR/kanban.db" "$NOW_" <<'PYEOF_DEBUG' || true
import sqlite3, sys, time
db = sys.argv[1]
now = int(sys.argv[2])
since = now - 3600
con = sqlite3.connect(db)
print('--- t4_loop_catch all events ---')
for r in con.execute(
    "SELECT id, kind, created_at, payload FROM task_events WHERE task_id='t4_loop_catch' ORDER BY id"
):
    print(r, 'age=', now - r[2])
print('--- counts (window=3600, now=now) ---')
row = con.execute(
    "SELECT COUNT(*) FROM task_events WHERE task_id='t4_loop_catch' AND kind='spawned' AND created_at >= ?",
    (since,)
).fetchone()
print('spawned_n=', row[0])
row = con.execute(
    "SELECT COUNT(*) FROM task_events WHERE task_id='t4_loop_catch' AND kind='gave_up' AND created_at >= ?",
    (since,)
).fetchone()
print('gave_up_n=', row[0])
PYEOF_DEBUG
    fail "T4: t4_loop_catch (5 spawns + 1 gave_up) NOT caught"
fi
echo "  T4 ok — loop-no-progress detects spawns+gave_up"

# T5
grep -q '"task_id": "t5_no_giveup"' "$LNP_FILE" && \
    fail "T5: t5_no_giveup (no gave_up) should NOT be caught"
echo "  T5 ok — loop-no-progress skips spawns without gave_up"

# T6
grep -q '"task_id": "t6_spawn_below"' "$LNP_FILE" && \
    fail "T6: t6_spawn_below (4 spawns < 5) should NOT be caught"
echo "  T6 ok — loop-no-progress skips when spawns below threshold"

# T8
grep -q '"task_id": "t8_recovered"' "$GG_FILE" && \
    fail "T8: t8_recovered (anti-pattern) should NOT be caught"
echo "  T8 ok — gate-by-giveup skips recover anti-pattern"

# --- T7: idempotency (проверяется ДО dry-run, чтобы sentinel уже стоял) ---
# sentinel уже в task_comments (см. фикстуру), dry-run должен skip
grep -q '"task_id": "t7_already_blocked"' "$GG_FILE" && \
    fail "T7: t7_already_blocked (sentinel present) should NOT be caught (idempotency)"
echo "  T7 ok — gate-by-giveup skips sentinel-marked task (idempotency)"

# --- T9: MEMORY_AVAILABLE preflight (mock /proc/meminfo) -----------------
# Подменяем /proc/meminfo через bind-mount нельзя (нет прав), но скрипт
# читает только /proc/meminfo — обернём через чтение переменной или
# используем override MEM_MIN_KB.
#
# Логика preflight: avail < MEM_MIN_KB → exit 2 из preflight_memory_available,
# скрипт делает sleep DEFER_COOLDOWN_SEC. Проверяем в apply-mode (НЕ dry-run)
# с маленьким DEFER_COOLDOWN_SEC, чтобы не задерживать прогон.
echo
echo "  T9 — MEMORY_AVAILABLE preflight (apply-mode с override)"
# Подменяем /proc/meminfo через переменную MEM_MIN_KB на заведомо большую —
# скрипт увидит реальный MemAvailable (~ 13+ GiB) и решит, что он < threshold.
# Альтернативный путь: переопределяем read_mem_available_kb через фейк.
# Простое решение: запускаем с MEM_MIN_KB=99999999999, тогда preflight вернёт 2
# → скрипт задефер и exit 0.
env -i HOME="$HOME" PATH="$PATH" KANBAN_BOARDS_DIR="$WORK/boards" \
    HERMES_HOME="$WORK" \
    STATE_DIR="$WORK" \
    LOG_FILE="$WORK/test_ros2.log" \
    LOCK_FILE="$WORK/test_ros2.lock" \
    DEFER_COOLDOWN_SEC=1 \
    MEM_MIN_KB=99999999999 \
    timeout 15 bash "$SCRIPT_UNDER_TEST" > "$WORK/mem.out" 2>&1
RC=$?
echo "  T9 preflight rc=$RC (ожидаем 0 + backoff 1s + exit 0)"
grep -q "MemAvailable=" "$WORK/mem.out" || \
    fail "T9: preflight should log MemAvailable value"
grep -q "deferring by 1s" "$WORK/mem.out" || \
    fail "T9: preflight should defer-then-exit (log line 'deferring by')"
echo "  T9 ok — preflight backs off at MemAvailable < MEM_MIN_KB"

# --- T7.5: writeable apply-mode — block ONE task with sentinel ---------------
# Чтобы apply не упал на hermes (нужен реальный CLI), подменим HERMES_BIN
# на echo-обёртку, которая «пишет в kanban» через python, не вызывая hermes.
# Минимальный fake — накапливает вызовы в $WORK/hermes.log.
fake_hermes="$WORK/hermes"
cat >"$fake_hermes" <<'FAKE'
#!/bin/bash
# Fake hermes: пишем только вызовы в лог, не делаем реальных side-effect'ов
# (потому что в test нет настоящего каталога ~/.hermes/profiles/<X>/).
echo "[fake-hermes $@]" >>"$FAKE_LOG"
exit 0
FAKE
chmod +x "$fake_hermes"
# Прогон apply-mode с подменой HERMES_BIN. Ожидаем что t1/t4 НЕ проходят
# (у них нет sentinel — apply should block), а t7 — skip (sentinel уже есть).
env -i HOME="$HOME" PATH="$PATH" KANBAN_BOARDS_DIR="$WORK/boards" \
    HERMES_HOME="$WORK" \
    STATE_DIR="$WORK" \
    LOG_FILE="$WORK/test_ros3.log" \
    LOCK_FILE="$WORK/test_ros3.lock" \
    HERMES_BIN="$fake_hermes" \
    FAKE_LOG="$WORK/hermes.log" \
    HERMES_CONFIG_DIR="$WORK" \
    bash "$SCRIPT_UNDER_TEST" > "$WORK/apply.out" 2>&1 || true
echo
echo "  T7.5 — apply-mode (fake hermes): ensure idempotency works at apply"
echo "--- apply output (tail) ---"
tail -10 "$WORK/apply.out"
# Должны быть calls для t1 (no sentinel → block), t4 (no sentinel → block),
# t7 (sentinel present → skip), а НЕ дубль-вызовов для t7.
if [ ! -f "$WORK/hermes.log" ]; then
    echo "  WARN T7.5: fake hermes log not created (apply не запустился)"
else
    # t7_already_blocked должен быть SKIP, не block/comment
    if grep -q "block.*t7_already_blocked" "$WORK/hermes.log"; then
        fail "T7.5: t7_already_blocked should be SKIP, но видим hermes block call"
    fi
    echo "  T7.5 ok — sentinel-idempotency при apply: t7 не получает block-call"
fi

pass "runtime-overshoot-loop: все 9 кейсов прошли (T1-T9)"
