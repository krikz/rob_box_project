#!/bin/bash
# ============================================================================
# agent-flow-orphan-watchdog.sh — ежедневный (every 24h) no-agent
# sweep, проверяющий что каждый agent-flow-*-watchdog.sh из EXPECTED[] install.sh
# зарегистрирован как cron-job в ~/.hermes/profiles/devops/cron/jobs.json.
#
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-orphan-watchdog.sh
# Copies are laid down by install.sh into:
#   - ~/.hermes/profiles/devops/scripts/
#   - ~/.hermes/scripts/  (legacy)
#
# Контекст / ретро t_6687a024 (16.09.2026, паттерн
# stale-conflicting-watchdog-not-scheduled):
#   - PR #2665 влил `agent-flow-stale-conflicting-watchdog.sh` в develop,
#     но ensure_stale_conflicting_watchdog_cron() в install.sh добавлен
#     только в самом PR (по итогу watchdog зарегистрирован — это разовый
#     фикс).
#   - Однако паттерн повторялся уже 2 раза:
#       * t_197de62a — cancel-on-provider-exhausted.sh был orphan до ручного
#         фикса (добавили в install.sh);
#       * t_6687a024 — stale-conflicting-watchdog.sh провисел ~6ч без
#         cron-job (PR #2671 в CONFLICTING всё это время, никто не
#         заметил).
#   - Решение этой карточки: двойная защита —
#     1. CI-guard `.github/workflows/G-Agent-Flow-Process-Checks.yml`
#        (ADR-0116) — блокирует PR с новым watchdog без правки install.sh;
#     2. этот скрипт — реактивный страховочный sweep, чтобы ловить
#        случаи, которые CI-guard пропустил (CI disabled, merge bypass,
#        hotfix push в develop вне PR).
#
# Контракт (per tick, every 24h, no-agent bash):
#   1. flock lock (не два тика одновременно)
#   2. MAINTENANCE gate (через af_maintenance_gate_or_exit)
#   3. Получить список EXPECTED-скриптов:
#      `bash <repo>/scripts/agent_flow/install.sh --list-files` (SOT).
#      Если install.sh недоступен — exit 1.
#   4. Отфильтровать только `agent-flow-*-watchdog.sh` (нас интересуют
#      именно watchdog'и — не все EXPECTED-скрипты).
#   5. Для каждого watchdog проверить наличие interval-job с
#      `script == <basename>` и `enabled == true` в
#      `~/.hermes/profiles/devops/cron/jobs.json`.
#   6. Для каждого orphan:
#      a) append в drift.alert.log (через alert_marker).
#      b) emit gh-issue-comment в `krikz/rob_box_project` issue
#         (label `agent-flow-watchdog-orphan`) — idempotent через
#         comment LIKE '%<MARKER_TAG>%' (24h dedup window).
#      c) emit kanban-карточку (через kanban-retro-create.sh) с
#         idempotency-key `retro:register-watchdog-<basename>` —
#         на следующий тик pre-check найдёт существующую → SKIP.
#   7. Log stats: scanned, with_cron, missing_cron, alerts_emitted,
#      errors.
#
# ENV:
#   REPO_DIR                  — путь к репо (default:
#                              /home/builder/hermes-share/rob_box_project)
#   HERMES_HOME               — путь к hermes (default: /home/builder/.hermes)
#   GH_REPO                   — owner/repo для issue-comment
#                              (default: krikz/rob_box_project)
#   GH_CONFIG_DIR             — для gh CLI auth (default: ~/.config/gh)
#   JOBS_FILE                 — путь к jobs.json (default:
#                              <HERMES_HOME>/profiles/devops/cron/jobs.json)
#   ALERT_LOG                 — путь к drift.alert.log (default:
#                              /tmp/agent-flow-drift.alert.log)
#   LOCK_FILE                 — flock guard (default:
#                              /tmp/agent-flow-orphan-watchdog.lock)
#   LOG_FILE                  — stats log (default:
#                              /tmp/agent-flow-orphan-watchdog.log)
#   STALE_DEDUP_HOURS         — one alert-comment per window (default 24)
#   MARKER_TAG                — comment dedup marker (default
#                              "🤖 watchdog-orphan-detector")
#   DRY_RUN                   — true → log only, no gh/kanban calls
#
# Выходы:
#   - Exit 0 — всё ok (включая «нет orphan'ов»).
#   - Exit 1 — критичный сбой (нет install.sh / python3 / нечитаемый
#              jobs.json).
#   - Exit 2 — нашлись missing-cron watchdog'и (alert для cron).
#
# Что НЕ делаем (явно):
#   - НЕ создаём cron-job сами (это решение оператора / install.sh).
#   - НЕ блокируем develop / merge-gate (только alert).
#   - НЕ модифицируем install.sh напрямую (только рекомендация).
#
# Pitfalls:
#   - jobs.json может быть с битой структурой (например, если cron-tick
#     шёл во время записи). Python3 ловит JSONDecodeError и считает
#     «джоба нет» — fail-open (alert сработает, но jobs.json не
#     повредим).
#   - install.sh может быть старой версии (без нового watchdog в
#     EXPECTED[]). В этом случае orphan-detector не увидит новый
#     watchdog — но install.sh в любом случае его не раскладывал бы.
#     Симметрично: jobs.json может быть из более новой версии, чем
#     install.sh (тогда orphan false-positive). Поэтому фильтруем
#     строго по EXPECTED[], а не по файлам в ~/.hermes/scripts/.
#   - Alert может спамить если drift-detect одновременно регистрирует
#     cron. Поэтому STALE_DEDUP_HOURS=24 и idempotent через marker.
# ============================================================================
set -uo pipefail  # без -e — soft-fail на per-watchdog errors

REPO_DIR="${REPO_DIR:-/home/builder/hermes-share/rob_box_project}"
# Default-подстановка :- срабатывает ТОЛЬКО если переменная пустая.
# На хосте ротации (devops-профиль) $HERMES_HOME уже задан в PATH/env
# как /home/builder/.hermes/profiles/devops — это НЕ hermes-root,
# а hermes-profile-root. Если оставить как есть, JOBS_FILE получится
# .../profiles/devops/profiles/devops/cron/jobs.json (404).
# Гард: если путь не содержит "profiles/<profile>" в финале — fallback
# на canonical /home/builder/.hermes.
if [ -z "${HERMES_HOME:-}" ] || [[ "$HERMES_HOME" == */profiles/* ]]; then
    HERMES_HOME="/home/builder/.hermes"
fi
GH_REPO="${GH_REPO:-krikz/rob_box_project}"
GH_CONFIG_DIR="${GH_CONFIG_DIR:-/home/builder/.config/gh}"
JOBS_FILE="${JOBS_FILE:-$HERMES_HOME/profiles/devops/cron/jobs.json}"
ALERT_LOG="${ALERT_LOG:-/tmp/agent-flow-drift.alert.log}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-orphan-watchdog.lock}"
LOG_FILE="${LOG_FILE:-/tmp/agent-flow-orphan-watchdog.log}"
STALE_DEDUP_HOURS="${STALE_DEDUP_HOURS:-24}"
MARKER_TAG="${MARKER_TAG:-🤖 watchdog-orphan-detector}"
DRY_RUN="${DRY_RUN:-false}"
LIB_PATH="$HERMES_HOME/scripts/lib_agent_flow_common.sh"

_now_iso() { date -u +%Y-%m-%dT%H:%M:%SZ; }

# --- flock guard (avoid race with drift-detect / other watchdogs) ----------
exec 9>"$LOCK_FILE" || true
if ! flock -n 9; then
    echo "[$(_now_iso)] watchdog-orphan-detector: another instance running — skip" >&2
    exit 0
fi

# --- preflight --------------------------------------------------------------
if ! command -v python3 >/dev/null 2>&1; then
    echo "[$(_now_iso)] watchdog-orphan-detector: python3 missing — exit 1" >&2
    exit 1
fi
if [ ! -d "$REPO_DIR" ]; then
    echo "[$(_now_iso)] watchdog-orphan-detector: REPO_DIR=$REPO_DIR not found — exit 1" >&2
    exit 1
fi
INSTALL_SH="$REPO_DIR/scripts/agent_flow/install.sh"
if [ ! -f "$INSTALL_SH" ]; then
    echo "[$(_now_iso)] watchdog-orphan-detector: $INSTALL_SH missing — exit 1" >&2
    exit 1
fi

# Optional MAINTENANCE gate (если lib подгружен)
if [ -f "$LIB_PATH" ]; then
    # shellcheck disable=SC1090
    source "$LIB_PATH" || true
    if declare -F af_maintenance_gate_or_exit >/dev/null 2>&1; then
        af_maintenance_gate_or_exit || exit 0
    fi
fi

# --- 1) получить список EXPECTED-файлов из install.sh (SOT) -----------------
# Используем `--list-files` режим (см. install.sh строки 408-412). Это
# единый источник истины — список скриптов, которые install.sh
# раскладывает по хостам. См. ретро 13.08 t_2cae75c0: раньше список
# дублировался в drift-detect.sh и разошёлся с install.sh.
EXPECTED_FILES_STR="$(bash "$INSTALL_SH" --list-files 2>/dev/null || true)"
if [ -z "$EXPECTED_FILES_STR" ]; then
    echo "[$(_now_iso)] watchdog-orphan-detector: install.sh --list-files returned empty — exit 1" >&2
    exit 1
fi

# --- 2) отфильтровать только agent-flow-*-watchdog.sh ------------------------
WATCHDOG_FILES=()
while IFS= read -r f; do
    case "$f" in
        agent-flow-*-watchdog.sh)
            # exclude этот сам orphan-detector (он сам регистрируется отдельно,
            # и проверять «есть ли для него cron» не имеет смысла в этом
            # скрипте — иначе вечный false-positive на первом тике).
            if [ "$f" = "agent-flow-orphan-watchdog.sh" ]; then
                continue
            fi
            WATCHDOG_FILES+=("$f")
            ;;
    esac
done <<< "$EXPECTED_FILES_STR"

_scanned=${#WATCHDOG_FILES[@]}
if [ "$_scanned" -eq 0 ]; then
    echo "[$(_now_iso)] watchdog-orphan-detector: no agent-flow-*-watchdog.sh found in EXPECTED — exit 0" >&2
    exit 0
fi

# --- 3) получить список script-name'ов из jobs.json -------------------------
# Используем python3 для парсинга — json бывает битый, fail-open.
_JOBS_TMP="$(mktemp -t wdorph.XXXXXX)"
trap 'rm -f "$_JOBS_TMP"' EXIT

python3 - "$JOBS_FILE" "$_JOBS_TMP" <<'PYEOF' 2>/dev/null || true
import json, os, sys
jobs_file = sys.argv[1]
out = sys.argv[2]
scripts_with_cron = set()
try:
    with open(jobs_file, "r") as fh:
        d = json.load(fh)
    for j in d.get("jobs", []):
        if not isinstance(j, dict):
            continue
        s = j.get("script")
        if not s or not isinstance(s, str):
            continue
        if j.get("enabled") and j.get("schedule", {}).get("kind") == "interval":
            scripts_with_cron.add(s)
except FileNotFoundError:
    pass
except Exception:
    # Битый JSON — fail-open (считаем «нет джобов»).
    pass
with open(out, "w") as fh:
    for s in sorted(scripts_with_cron):
        fh.write(s + "\n")
PYEOF

_with_cron_count=0
_missing=()

while IFS= read -r registered_script; do
    [ -z "$registered_script" ] && continue
done < "$_JOBS_TMP"

for f in "${WATCHDOG_FILES[@]}"; do
    if grep -Fxq "$f" "$_JOBS_TMP"; then
        _with_cron_count=$((_with_cron_count + 1))
    else
        _missing+=("$f")
    fi
done

_missing_count=${#_missing[@]}

# --- 4) emit alerts для каждого orphan -------------------------------------
_alerts_emitted=0
_errors=0

if [ "$_missing_count" -gt 0 ]; then
    # Append в drift.alert.log (общий канал с drift-detect).
    for f in "${_missing[@]}"; do
        alert_msg="[$(_now_iso)] watchdog-orphan-detector: MISSING cron-job for $f (EXPECTED[] in install.sh but no enabled interval-job in $JOBS_FILE)"
        if [ "$DRY_RUN" = "true" ]; then
            echo "[DRY] $alert_msg" >&2
        else
            echo "$alert_msg" >> "$ALERT_LOG" || true
        fi
    done

    # gh issue-comment (если gh auth и label существует). Идемпотентность
    # через marker-tag в последнем alert-комменте + STALE_DEDUP_HOURS window.
    # Каждый watchdog-orphan получает ОДИН коммент в issues за dedup-window.
    if [ "$DRY_RUN" != "true" ] && command -v gh >/dev/null 2>&1 && gh auth status >/dev/null 2>&1; then
        GH_CONFIG_DIR="$GH_CONFIG_DIR" gh label list --repo "$GH_REPO" --json name 2>/dev/null | grep -q "agent-flow-watchdog-orphan" || {
            GH_CONFIG_DIR="$GH_CONFIG_DIR" gh label create "agent-flow-watchdog-orphan" --repo "$GH_REPO" --color "d93f0b" --description "Auto: watchdog script in EXPECTED[] but no cron-job registered (agent-flow-orphan-watchdog.sh)" >/dev/null 2>&1 || true
        }
        for f in "${_missing[@]}"; do
            issue_num="$(GH_CONFIG_DIR="$GH_CONFIG_DIR" gh issue list \
                --repo "$GH_REPO" \
                --state open \
                --label "agent-flow-watchdog-orphan" \
                --json number 2>/dev/null | python3 -c "
import json, sys
try:
    d = json.load(sys.stdin)
    if d and isinstance(d, list):
        print(d[0].get('number', ''))
except Exception:
    pass
" 2>/dev/null)"
            if [ -z "$issue_num" ]; then
                # Создать issue — попытка (может не быть write прав).
                if GH_CONFIG_DIR="$GH_CONFIG_DIR" gh issue create \
                    --repo "$GH_REPO" \
                    --label "agent-flow-watchdog-orphan" \
                    --title "🛡 agent-flow-orphan-watchdog: $f без cron-job" \
                    --body "$(printf '%s\n\n## Affected watchdog scripts (без cron)\n\n%s\n\n## Что делать\n\n1. Добавить в scripts/agent_flow/install.sh:\n   ```\n   ensure_%s_watchdog_cron() {\n       ensure_cron_job devops \"Agent Flow <Name> Watchdog\" \"<basename>\" \"every Nh\" interval\n   }\n   ensure_%s_watchdog_cron\n   ```\n2. Запустить на хосте: \`bash scripts/agent_flow/install.sh\`.\n\nРегрессия: ретро t_6687a024 (stale-conflicting-watchdog-not-scheduled) и t_197de62a (cancel-on-provider-exhausted).\n' "$MARKER_TAG" "$f" "${f#agent-flow-}" "${f%-watchdog.sh}" "${f#agent-flow-}" 2>&1 | head -3 | tr -d '\r')" ; then
                    _alerts_emitted=$((_alerts_emitted + 1))
                else
                    _errors=$((_errors + 1))
                fi
            else
                # Issue уже есть — добавить коммент с dedup.
                _last_comment_ts="$(GH_CONFIG_DIR="$GH_CONFIG_DIR" gh issue view "$issue_num" --repo "$GH_REPO" --json comments 2>/dev/null | python3 -c "
import json, sys, datetime
try:
    d = json.load(sys.stdin)
    threshold_h = $STALE_DEDUP_HOURS
    threshold = datetime.datetime.now(datetime.timezone.utc) - datetime.timedelta(hours=threshold_h)
    for c in (d.get('comments') or []):
        body = (c.get('body') or '')
        if '$MARKER_TAG' in body:
            ts = c.get('createdAt') or ''
            try:
                ct = datetime.datetime.fromisoformat(ts.replace('Z', '+00:00'))
                if ct > threshold:
                    print(ct.isoformat())
                    sys.exit(0)
            except Exception:
                pass
    sys.exit(1)
except Exception:
    sys.exit(1)
" 2>/dev/null || true)"
                if [ -z "$_last_comment_ts" ]; then
                    if GH_CONFIG_DIR="$GH_CONFIG_DIR" gh issue comment "$issue_num" --repo "$GH_REPO" --body "$MARKER_TAG

Повторная детекция (tick от $(_now_iso)): orphan watchdog всё ещё без cron-job.

- $f — нет enabled interval-job в jobs.json

Действие: зарегистрировать ensure_*_watchdog_cron() в scripts/agent_flow/install.sh + запустить install.sh на хосте." >/dev/null 2>&1; then
                        _alerts_emitted=$((_alerts_emitted + 1))
                    else
                        _errors=$((_errors + 1))
                    fi
                fi
            fi
        done
    fi
fi

# --- 5) summary -------------------------------------------------------------
_summary="watchdog-orphan-detector: scanned=$_scanned with_cron=$_with_cron_count missing=$_missing_count alerts=$_alerts_emitted errors=$_errors"
echo "[$(_now_iso)] $_summary" >> "$LOG_FILE"
echo "$_summary" >&2

# Коды выхода:
#  0 — нет orphan'ов
#  1 — критичный сбой (нет install.sh / python3)
#  2 — есть missing-cron watchdog'и (alert для cron-delivery)
if [ "$_missing_count" -gt 0 ]; then
    exit 2
fi
exit 0