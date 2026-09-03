#!/bin/bash
# ============================================================================
# SOT (source-of-truth): <repo>/scripts/agent_flow/watchdog-provider-quick.sh
# Правим ТОЛЬКО здесь + commit + merge в develop. На хост раскладывает
# `bash <repo>/scripts/agent_flow/install.sh` — hardlink-копиями (cp -al), НЕ
# симлинками: симлинк в ~/.hermes/scripts/ ресолвится наружу и отклоняется
# guard'ом hermes-agent scheduler.py::_validate_script_path (ретро 11.08
# t_a6a236e0d9f0470e — 50 упавших тиков подряд, 1ч42м даунтайма).
# Полный список путей раскладки — в install.sh, сверку копий держит
# agent-flow-drift-detect.sh. Ручная правка копии на хосте затрётся.
# ============================================================================
# Watchdog provider-exhaustion FAST-TICK guard (ретро 24.08 t_4c73490f).
#
# Зачем отдельный скрипт: главный watchdog.sh сканирует каждые 2 минуты
# (`hermes cron` interval resolution = minutes). За это время воркер с
# MiniMax 429/DeepSeek 401 успевает дважды поймать
# ``consecutive_failures=2 → gave_up``, и карточка уходит в blocked как
# «обычный краш», а не как «provider exhausted». Recovery-волна в
# watchdog.sh тогда срабатывает только ПОСЛЕ providers_alive=True
# (т.е. когда MiniMax уже отвечает 200), но к этому моменту карточки
# уже 9 штук в архивном виде.
#
# Этот скрипт — узкая быстрая дорожка: сканирует КАЖДУЮ 1 минуту, делает
# только provider-exhaustion detect и block (kind=capability), без
# диспетчер-рестарта/telegram-self-heal. Полная логика остаётся в
# watchdog.sh (2-мин тик) — этот скрипт ЕЁ ДОПОЛНЯЕТ, не дублирует.
#
# Hot-path (helper, без LLM):
#   - For each kanban.db board:
#       providers_alive = any fresh (<PROVIDER_LOG_WINDOW) worker log
#                          that does NOT contain a provider marker
#       if providers dead AND last_failure_error matches quota/auth regex:
#         block task (kind=capability, idempotent)
#   - Recovery (providers alive AND last_event ∈ blocker set):
#         unblock → ready
#
# Provider markers расширены (по сравнению с watchdog.sh) на 'HTTP 401',
# 'Authentication Fails', 'is invalid', 'invalid_request_error',
# 'authentication_error' — потому что в реальности DeepSeek валит именно
# с 401 invalid_request_error, а не 429, и PR #1286 не покрывал 401.
#
# Output: markdown status ТОЛЬКО при действии. Пустой stdout = silent tick.
# Cron schedule: every 1m (минимум, который позволяет `hermes cron`
# interval minutes-resolution). Этого хватает: реальный краш-цикл
# 2x consecutive_failures = ~2-4 мин, мы сканируем каждую 1 мин и
# успеваем заблокировать до второго reincarnation.
# ============================================================================

set -euo pipefail

HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"
KANBAN_BOARDS_DIR="$HERMES_HOME/kanban/boards"
HERMES_BIN="${HERMES_BIN:-/home/builder/.hermes/hermes-agent/venv/bin/hermes}"
LOCK_FILE="${LOCK_FILE:-$HERMES_HOME/state/watchdog-provider-quick.lock}"
PROVIDER_ACTIONS_FILE="${PROVIDER_ACTIONS_FILE:-$HERMES_HOME/state/watchdog-provider-quick-actions.txt}"
LOG_FILE="${LOG_FILE:-$HERMES_HOME/logs/watchdog-provider-quick.log}"

mkdir -p "$(dirname "$LOCK_FILE")" "$(dirname "$LOG_FILE")"

log() { printf '[watchdog-provider-quick] %s\n' "$*" >&2; }

# Процесс-лок: один инстанс (flock на FD 9). Если другой уже сканирует —
# выходим тихо (тик пропускаем).
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
    log "⏳ another instance holds $LOCK_FILE — skip"
    exit 0
fi

# ============================================================================
# Python-часть: scan, collect provider_actions, write to PROVIDER_ACTIONS_FILE.
# Эта секция ЗАКАМЕНЧЕНА — логика тестируется в
# tests/agent_flow/test_watchdog_provider_quick_guard.py.
# ============================================================================

python3 - "$HERMES_HOME" "$KANBAN_BOARDS_DIR" "$PROVIDER_ACTIONS_FILE" \
    <<'PYEOF'
import os, sys, glob, sqlite3, time

hermes_home = sys.argv[1]
boards_dir = sys.argv[2]
provider_actions_file = sys.argv[3]

# Маркеры провайдер-эксхаустиина. Список РАСШИРЕН относительно watchdog.sh
# (PR #1286): добавлены HTTP 401 / Authentication Fails / is invalid /
# invalid_request_error / api key. Причина — DeepSeek реально валит
# HTTP 401 'Your api key ****bd8b is invalid', а старый список не ловил
# 401 → watchdog считал карточку обычным крашем → consecutive_failures
# росли быстрее, чем scan успевал вмешаться.
PROVIDER_MARKERS = (
    # LLM quota / rate limit (MiniMax, OpenAI, Anthropic, etc.)
    "HTTP 402", "Insufficient Balance", "Out of credits",
    "Billing or credits exhausted", "HTTP 429", "rate limit",
    "Token Plan usage limit", "2056", "Token Plan rate limit reached",
    "health-aware-fallback", "all providers unavailable",
    # Auth/quota NEW (ретро 24.08 t_4c73490f): DeepSeek 401 invalid api
    # key, OpenAI authentication_error, etc.
    "HTTP 401", "Authentication Fails",
    "is invalid", "invalid_request_error", "api key",
    "authentication_error",
    # General
    "provider-exhaustion", "all providers failed",
    "provider unavailable",
)

# Дополнительная регулярка для last_failure_error (короткий текст ошибки
# в tasks.last_failure_error). Ловит cases, когда log_файла ещё нет /
# не успел дописаться, но в task_runs.error строка уже есть.
# Совпадает с _RESPAWN_BLOCKER_RE в hermes-agent/kanban_db.py, но
# дополнительно '**bd8b is invalid' (masked api key) и 'health-aware'.
def _failure_error_matches(error: str) -> bool:
    if not error:
        return False
    e = error.lower()
    haystacks = (
        "quota", "rate limit", "ratelimit", "rate_limit",
        "429", "402", "401",
        "auth", "authentication", "unauthorized", "forbidden",
        "billing", "subscription", "access denied", "permission denied",
        "invalid api key", "invalid_request_error", "is invalid",
        "out of credits", "insufficient balance", "token plan",
        "2056", "2062", "health-aware", "all providers",
    )
    return any(h in e for h in haystacks)


PROVIDER_LOG_WINDOW = 900  # сек: лог считается «свежим» для проверки живости

def _log_path(board_dir: str, task_id: str) -> str:
    return os.path.join(board_dir, "logs", f"{task_id}.log")

def _log_has_provider_markers(path: str) -> bool:
    if not os.path.isfile(path):
        return False
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            tail = f.read()[-200000:]  # ~200KB достаточно
    except Exception:
        return False
    low = tail.lower()
    return any(m.lower() in low for m in PROVIDER_MARKERS)

now = int(time.time())

# 1c1. живы ли провайдеры (any fresh clean log)
providers_alive = False
for db in sorted(glob.glob(f"{boards_dir}/*/kanban.db")):
    board_dir = os.path.dirname(db)
    logs_dir = os.path.join(board_dir, "logs")
    if not os.path.isdir(logs_dir):
        continue
    try:
        for lp in glob.glob(os.path.join(logs_dir, "*.log")):
            if now - os.path.getmtime(lp) <= PROVIDER_LOG_WINDOW \
                    and not _log_has_provider_markers(lp):
                providers_alive = True
                break
    except Exception:
        pass
    if providers_alive:
        break

provider_actions: list[str] = []
for db in sorted(glob.glob(f"{boards_dir}/*/kanban.db")):
    board = os.path.basename(os.path.dirname(db))
    board_dir = os.path.dirname(db)
    try:
        con = sqlite3.connect(db)
        cur = con.execute(
            "SELECT id, status, COALESCE(worker_pid,0) AS pid, "
            "COALESCE(consecutive_failures,0) AS cf, "
            "COALESCE(last_failure_error, '') AS lfe "
            "FROM tasks"
        )
        for task_id, status, pid, cf, lfe in cur.fetchall():
            lp = _log_path(board_dir, task_id)
            log_mark = _log_has_provider_markers(lp)
            err_mark = _failure_error_matches(lfe)
            if not (log_mark or err_mark):
                continue

            if status == "blocked":
                # Recovery-волна: если карточка заблокирована dispatcher'ом
                # (gave_up / crashed / rate_limited) И ПРОВАЙДЕРЫ СНОВА
                # ОТВЕЧАЮТ (providers_alive=True) — разблокировать в ready.
                # НЕ трогаем ручной блок (kind='capability' от юзера или
                # kind=needs_input/dependency/transient).
                if providers_alive:
                    # Получаем kind и last_event для guard'а (не unblock если
                    # юзер ставил capability/dependency руками).
                    row = con.execute(
                        "SELECT block_kind, "
                        "(SELECT kind FROM task_events WHERE task_id=? "
                        "ORDER BY id DESC LIMIT 1) AS last_ev "
                        "FROM tasks WHERE id=?", (task_id, task_id),
                    ).fetchone()
                    bk, last_ev = (row[0] if row else None,
                                   row[1] if row else None)
                    # Только если блок поставил dispatcher (последний event
                    # ∈ gave_up/прочие crash-variants) ИЛИ block_kind=NULL.
                    # Если block_kind ∈ {capability, needs_input, dependency,
                    # transient} от человека — НЕ трогать.
                    if bk in (None, "transient") or last_ev in (
                        "gave_up", "protocol_violation", "crashed",
                        "rate_limited",
                    ):
                        provider_actions.append(f"unblock|{board}|{task_id}")
            elif status in ("running", "ready"):
                # Active state — блокируем если провайдеры мертвы.
                # running: блокируем ТОЛЬКО если pid мёртв (heartbeat-stale).
                #   Живой worker (только что spawned, мой собственный,
                #   может и payload содержать слово 'provider-exhaustion'
                #   как часть текста задачи — не ложно-блокировать).
                # ready:  блокируем если провайдеры мертвы И worker log
                #   свежий (<=PROVIDER_LOG_WINDOW с маркерами) — это
                #   значит предыдущий crash был только что, и НЕ надо
                #   давать dispatcher'у шанс респаунить (cf ещё не дошёл
                #   до лимита, но мы опередим его). Stale log на ready
                #   задаче = старый crash, от которого уже отказались →
                #   leave for normal respawn (или другая причина).
                block_it = False
                if status == "running":
                    pid_alive = False
                    if pid and pid > 0:
                        try:
                            os.kill(int(pid), 0)
                            pid_alive = True
                        except (OSError, ProcessLookupError):
                            pid_alive = False
                    if not pid_alive:
                        block_it = True
                elif status == "ready" and not providers_alive and log_mark:
                    # log_mark = marker в логе; перепроверим mtime прямо
                    # здесь (там же, где читаем). Делаем через os.stat.
                    try:
                        log_mtime = os.path.getmtime(lp)
                    except OSError:
                        log_mtime = 0
                    if now - log_mtime <= PROVIDER_LOG_WINDOW:
                        block_it = True
                if block_it:
                    provider_actions.append(f"block|{board}|{task_id}")
        con.close()
    except Exception as exc:
        print(f"[watchdog-provider-quick] {board} provider-scan error: {exc}",
              file=sys.stderr)

if provider_actions:
    os.makedirs(os.path.dirname(provider_actions_file), exist_ok=True)
    try:
        with open(provider_actions_file, "w", encoding="utf-8") as f:
            f.write("\n".join(provider_actions) + "\n")
        # echo for the markdown tick summary (только при действии)
        print(f"**provider-quick actions:** {len(provider_actions)} "
              f"(providers_alive={providers_alive})")
        for a in provider_actions:
            print(f"- 🔧 {a}")
    except Exception as exc:
        print(f"[watchdog-provider-quick] write actions failed: {exc}",
              file=sys.stderr)
else:
    # silent tick: ничего не печатаем (no_agent cron = no tokens)
    pass

# Краткий статус (в stderr, чтобы cron log имел liveness без шума).
print(f"[watchdog-provider-quick] tick: providers_alive={providers_alive} "
      f"actions={len(provider_actions)}", file=sys.stderr)
PYEOF

# ============================================================================
# bash-часть: исполнить PROVIDER_ACTIONS_FILE через hermes kanban CLI.
# При block передаём --kind capability (НЕ dependency — это не wait на
# parent, это ждать юзера + provider). Idempotent: повторный block/unblock
# -- no-op с ненулевым rc, ловим и не паникуем.
# ============================================================================
if [ -s "$PROVIDER_ACTIONS_FILE" ]; then
    log "actions: $(wc -l < "$PROVIDER_ACTIONS_FILE") из $PROVIDER_ACTIONS_FILE"
    while IFS='|' read -r action board task_id; do
        [ -n "$action" ] || continue
        case "$action" in
            block)
                if "$HERMES_BIN" kanban --board "$board" block \
                    --kind capability "$task_id" \
                    "provider-exhaustion: провайдер исчерпан, ждать (MiniMax 429 или DeepSeek 401 — пополнить ключ, см. issue #1193). auto-block ретро 24.08 t_4c73490f" \
                    >> "$LOG_FILE" 2>&1; then
                    log "✅ provider-quick-block $board/$task_id"
                else
                    log "⚠️ provider-quick-block $board/$task_id failed (уже blocked или CLI error)"
                fi
                ;;
            unblock)
                if "$HERMES_BIN" kanban --board "$board" unblock \
                    --reason "provider-quick: провайдер восстановлен — респавн (ретро 24.08 t_4c73490f)" \
                    "$task_id" >> "$LOG_FILE" 2>&1; then
                    log "✅ provider-quick-unblock $board/$task_id"
                else
                    log "⚠️ provider-quick-unblock $board/$task_id failed (уже ready или CLI error)"
                fi
                ;;
            *)
                log "⚠️ provider-quick: неизвестное действие '$action'"
                ;;
        esac
    done < "$PROVIDER_ACTIONS_FILE"
    rm -f "$PROVIDER_ACTIONS_FILE"
fi
