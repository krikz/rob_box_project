#!/bin/bash
# ============================================================================
# dryrun_flaky_detect.sh — детерминированный dry-run harness для flaky-detect
# ветки в agent-flow-e2e-fail-streak-watchdog.sh (issue t_f33ecbf8).
#
# Контекст / ретро t_f33ecbf8 (2026-09-27 13:00 CEST):
#   L: E2E Voice Test на ОДНОМ headSha (ecfc454b) дал разные вердикты в двух
#   прогонах подряд: 19:56 success, 20:09 failure. Delta 13 мин — типичная
#   flaky-картина (тайминги STT, race в speaker_id_node, температура робота).
#   Ранее наблюдалось: 25.09 19:11→19:25 (1h14, разные коммиты), 24.09
#   01:06→01:12 (6 мин, разные коммиты). Это flaky по природе, не реальная
#   регрессия — но fail-streak-watchdog (a9b04981) при streak_with_success ≥ 5
#   открывает новый fail-streak issue, впустую заваливая Шифу под «это bug».
#
# Решение / контракт (issue t_f33ecbf8):
#   1. FLaky-detect запускается ТОЛЬКО при streak ≥ E2E_FAIL_STREAK_ISSUE_THRESHOLD
#      (т.е. не вступает в конфликт с существующей логикой).
#   2. Считаем same_headsha_count в streak (NEWEST→OLDEST, до первого success
#      включительно; success РАЗРЫВАЕТ streak, поэтому если в streak есть хоть
#      один success — streak_with_success = кол-ву fails после этого success).
#   3. Если same_headsha_count >= FLAKY_DETECT_MIN (default 3) — это не
#      регрессия, а flaky. Действия:
#        a) skip auto-create `e2e-fail-streak` issue (rate-limit сохраняется —
#           cooldown НЕ обновляется, чтобы реальная регрессия после flaky не
#           была замаскирована);
#        b) gh-комментарий с тегом `[flaky]` в существующий `e2e-fail-streak`
#           issue (если он открыт) с указанием:
#             - streak_with_success и same_headsha_count;
#             - список run IDs на этом headSha;
#             - 24h dedup-window для повторных комментариев.
#   4. Если same_headsha_count < FLAKY_DETECT_MIN (или headSha разные) →
#      это РЕАЛЬНАЯ регрессия → идём в старую ветку auto-create issue.
#
# Этот скрипт — НЕ модификация watchdog, а зеркало его flaky-detection логики,
# которое можно гонять локально и в CI без gh-токена и без реальной сети.
# Acceptance (issue t_f33ecbf8, шаг 1):
#   "если последние 2 прогона на одном headSha дали разные verdict → пометить
#    'flaky' и НЕ открывать issue автоматически"
#
# Чтобы воспроизвести фикстуру именно 26.09 (ecfc454b), используем сценарий
# (A): streak=2 (success 19:56 + failure 20:09) — порог issue-create НЕ
# достигнут, тихий, никаких действий. Это уже корректное сегодняшнее поведение.
#
# Флаки проявляется при серии ретраев с тем же headSha — тестовые сценарии
# (i)-(iv) ниже моделируют streak=5 с разными паттернами same_headsha и
# проверяют, что watchdog решает regression vs flaky правильно.
#
# Использование:
#   bash scripts/agent_flow/dryrun_flaky_detect.sh
#
# Exit:
#   0 — все сценарии отработали как ожидается.
#   1 — нарушен инвариант (regression payload count != 1 ИЛИ flaky marker
#       count != expected).
#
# ENV overrides (для теста):
#   FLAKY_DETECT_MIN=3                 — minimum same-headsha fails для пометки flaky
#   FLAKY_DETECT_RATIO=0.6             — minimum ratio same-headsha/total для flaky
#   FLAKY_DEDUP_HOURS=24
#   E2E_FAIL_STREAK_ISSUE_LABEL="e2e-fail-streak"
# ============================================================================
set -u

# --- env defaults (mirror watchdog defaults из PR) --------------------------
GH_REPO="${GH_REPO:-krikz/rob_box_project}"
E2E_WORKFLOW="${E2E_WORKFLOW:-L-E2E Voice Test.yml}"
E2E_FAIL_STREAK_ISSUE_THRESHOLD="${E2E_FAIL_STREAK_ISSUE_THRESHOLD:-5}"
E2E_FAIL_STREAK_ISSUE_LABEL="${E2E_FAIL_STREAK_ISSUE_LABEL:-e2e-fail-streak}"
FLAKY_DETECT_MIN="${FLAKY_DETECT_MIN:-3}"
FLAKY_DETECT_RATIO="${FLAKY_DETECT_RATIO:-0.6}"
FLAKY_DEDUP_HOURS="${FLAKY_DEDUP_HOURS:-24}"

HERMES_HOME="${HERMES_HOME:-${HOME}/.hermes}"
FLAKY_DEDUP_FILE="${FLAKY_DEDUP_FILE:-${HERMES_HOME}/state/agent-flow-e2e-flaky-dedup}"
mkdir -p "$(dirname "$FLAKY_DEDUP_FILE")" 2>/dev/null || true

# --- фикстуры ---------------------------------------------------------------
# (i)   streak=5 (5 fails подряд), ВСЕ на ОДНОМ headSha. Классический flaky
#       (5 ретраев с тем же кодом из-за speaker_id_node race). Ожидание:
#       FLAKY → skip auto-create, post comment marker.
FIXTURE_FLAKY_5_SAME_SHA='[
  {"databaseId":1,"conclusion":"failure","createdAt":"2026-09-27T08:00:00Z","headSha":"aaa111222333444555666777888999aa","headBranch":"develop"},
  {"databaseId":2,"conclusion":"failure","createdAt":"2026-09-27T07:30:00Z","headSha":"aaa111222333444555666777888999aa","headBranch":"develop"},
  {"databaseId":3,"conclusion":"failure","createdAt":"2026-09-27T07:00:00Z","headSha":"aaa111222333444555666777888999aa","headBranch":"develop"},
  {"databaseId":4,"conclusion":"failure","createdAt":"2026-09-27T06:30:00Z","headSha":"aaa111222333444555666777888999aa","headBranch":"develop"},
  {"databaseId":5,"conclusion":"failure","createdAt":"2026-09-27T06:00:00Z","headSha":"aaa111222333444555666777888999aa","headBranch":"develop"},
  {"databaseId":6,"conclusion":"success","createdAt":"2026-09-27T05:00:00Z","headSha":"bbb222333444555666777888999aabb","headBranch":"develop"}
]'

# (ii)  streak=5 (5 fails подряд), на РАЗНЫХ headSha. Реальная регрессия — что-то
#       накатывали в develop, и каждый прогон падал на новом коммите. Ожидание:
#       REGRESSION → auto-create issue.
FIXTURE_REGRESSION_5_DIFF_SHA='[
  {"databaseId":11,"conclusion":"failure","createdAt":"2026-09-27T08:00:00Z","headSha":"111aaa","headBranch":"develop"},
  {"databaseId":12,"conclusion":"failure","createdAt":"2026-09-27T07:30:00Z","headSha":"222bbb","headBranch":"develop"},
  {"databaseId":13,"conclusion":"failure","createdAt":"2026-09-27T07:00:00Z","headSha":"333ccc","headBranch":"develop"},
  {"databaseId":14,"conclusion":"failure","createdAt":"2026-09-27T06:30:00Z","headSha":"444ddd","headBranch":"develop"},
  {"databaseId":15,"conclusion":"failure","createdAt":"2026-09-27T06:00:00Z","headSha":"555eee","headBranch":"develop"},
  {"databaseId":16,"conclusion":"success","createdAt":"2026-09-27T05:00:00Z","headSha":"666fff","headBranch":"develop"}
]'

# (iii) streak=5, MIX: 3 fails на одном headSha (достаточно для flaky) + 2 на другом.
#       Сценарий: сначала был flaky, потом в develop приехал регресс. Граница —
#       доминирующий sha превышает FLAKY_DETECT_MIN, и ratio доминирующего sha
#       к total >= FLAKY_DETECT_RATIO (по умолчанию 0.6: 3/5 = 0.6 — flaky).
FIXTURE_FLAKY_BORDER='[
  {"databaseId":21,"conclusion":"failure","createdAt":"2026-09-27T08:00:00Z","headSha":"same-sha-1","headBranch":"develop"},
  {"databaseId":22,"conclusion":"failure","createdAt":"2026-09-27T07:30:00Z","headSha":"diff-sha-2","headBranch":"develop"},
  {"databaseId":23,"conclusion":"failure","createdAt":"2026-09-27T07:00:00Z","headSha":"same-sha-1","headBranch":"develop"},
  {"databaseId":24,"conclusion":"failure","createdAt":"2026-09-27T06:30:00Z","headSha":"diff-sha-3","headBranch":"develop"},
  {"databaseId":25,"conclusion":"failure","createdAt":"2026-09-27T06:00:00Z","headSha":"same-sha-1","headBranch":"develop"},
  {"databaseId":26,"conclusion":"success","createdAt":"2026-09-27T05:00:00Z","headSha":"old-sha","headBranch":"develop"}
]'

# (iv)  streak=4 (ниже FLAKY_DETECT_MIN), все на одном headSha. Не достаточно
#       fails для flaky-detect метки, но и не regression path потому что streak
#       ниже E2E_FAIL_STREAK_ISSUE_THRESHOLD. → SILENT.
FIXTURE_SILENT='[
  {"databaseId":31,"conclusion":"failure","createdAt":"2026-09-27T08:00:00Z","headSha":"aaa","headBranch":"develop"},
  {"databaseId":32,"conclusion":"failure","createdAt":"2026-09-27T07:30:00Z","headSha":"aaa","headBranch":"develop"},
  {"databaseId":33,"conclusion":"success","createdAt":"2026-09-27T06:00:00Z","headSha":"bbb","headBranch":"develop"}
]'

# --- compute_flaky_decision (зеркало будущей watchdog-функции) -------------
# Принимает JSON-массив runs на stdin, печатает "decision|json_data".
# decision ∈ {regression, flaky, silent}.
# json_data — числа и sha для комментария.
compute_flaky_decision() {
    FLAKY_MIN="$FLAKY_DETECT_MIN" \
    FLAKY_RATIO="$FLAKY_DETECT_RATIO" \
    THR="$E2E_FAIL_STREAK_ISSUE_THRESHOLD" \
    python3 -c '
import json, os, sys

try:
    runs = json.load(sys.stdin)
except Exception:
    print("silent|{}")
    raise SystemExit(0)

flaky_min = int(os.environ["FLAKY_MIN"])
flaky_ratio = float(os.environ["FLAKY_RATIO"])
threshold = int(os.environ["THR"])

# streak_with_success: NEWEST → OLDEST, success РАЗРЫВАЕТ.
streak = 0
seen_shas = {}  # sha[7] -> count внутри streak
for r in runs:
    c = r.get("conclusion")
    if c == "success":
        break
    if c in ("failure", "cancelled", "timed_out"):
        streak += 1
        sha7 = (r.get("headSha") or "")[:7]
        seen_shas[sha7] = seen_shas.get(sha7, 0) + 1
    # in_progress / None — нейтрально, не считаем

if streak < threshold:
    print("silent|{}")
    raise SystemExit(0)

# Доминирующий sha в streak.
dominant_sha = max(seen_shas.items(), key=lambda kv: kv[1])[0] if seen_shas else ""
dominant_count = seen_shas.get(dominant_sha, 0)
ratio = (dominant_count / streak) if streak else 0.0

# Flaky: dominant sha встречается >= min раз И его доля >= ratio.
# Default: FLAKY_DETECT_MIN=3, FLAKY_DETECT_RATIO=0.6 → 3 из 5 = flaky.
is_flaky = (dominant_count >= flaky_min) and (ratio >= flaky_ratio)
decision = "flaky" if is_flaky else "regression"

# Run IDs на dominant sha (для marker).
runs_on_sha = []
for r in runs:
    c = r.get("conclusion")
    if c in ("failure", "cancelled", "timed_out") and (r.get("headSha") or "")[:7] == dominant_sha:
        runs_on_sha.append(r.get("databaseId"))

import json as _j
print(decision + "|" + _j.dumps({
    "streak": streak,
    "dominant_sha": dominant_sha,
    "dominant_count": dominant_count,
    "ratio": round(ratio, 3),
    "flaky_min": flaky_min,
    "flaky_ratio": flaky_ratio,
    "runs_on_sha": runs_on_sha,
    "threshold": threshold,
}, ensure_ascii=False))
'
}

# --- emit_regression_payload ------------------------------------------------
# Печатает payload, идентичный тому, что watchdog бы отправил через
# `gh issue create` для REGRESSION. Шаблон — зеркало fail-streak
# watchdog auto-create-issue (см. dryrun_fail_streak_issue.sh).
emit_regression_payload() {
    local _streak_data="$1"
    local _streak
    _streak="$(printf '%s' "$_streak_data" | python3 -c 'import json,sys; print(json.load(sys.stdin)["streak"])')"
    {
        printf '%s\n' '---REGRESSION PAYLOAD BEGIN---'
        printf 'gh issue create --repo %s \\\n' "$GH_REPO"
        printf '  --title %s \\\n' "[e2e-fail-streak] L: E2E Voice Test — ${_streak} fails подряд (regression)"
        printf '  --label %s \\\n' "$E2E_FAIL_STREAK_ISSUE_LABEL"
        printf '  --body <<<EOF_BODY_MARKER\n'
        printf '%s\n' "flaky-detect: regression (no dominant headSha ≥ ${FLAKY_DETECT_MIN} с ratio ≥ ${FLAKY_DETECT_RATIO})"
        printf '%s\n' 'EOF_BODY_MARKER'
        printf '%s\n' '---REGRESSION PAYLOAD END---'
    }
}

# --- emit_flaky_marker (зеркало watchdog `gh issue comment` ветки) ----------
# ВАЖНО: НЕ создаёт новый issue — добавляет комментарий с маркером
# `[flaky-detect]` в существующий e2e-fail-streak issue (если он открыт),
# ИЛИ просто логирует (если нет открытого issue — не шуметь лишним issue).
# По dedup: пропустить если последний comment с тем же marker < FLAKY_DEDUP_HOURS назад.
emit_flaky_marker() {
    local _streak_data="$1"
    local _open_issue="${OPEN_E2E_FAIL_ISSUE:-}"
    local _dedup_ok="true"

    if [ -f "$FLAKY_DEDUP_FILE" ]; then
        local _dedup_epoch _dedup_age_s _dedup_limit_s
        _dedup_epoch="$(stat -c '%Y' "$FLAKY_DEDUP_FILE" 2>/dev/null || echo 0)"
        _dedup_age_s=$(( $(date -u +%s) - ${_dedup_epoch:-0} ))
        _dedup_limit_s=$(( FLAKY_DEDUP_HOURS * 3600 ))
        if [ "${_dedup_age_s:-0}" -lt "${_dedup_limit_s}" ]; then
            printf '  SKIPPED: flaky-dedup active (%ss < %ss)\n' \
                "$_dedup_age_s" "$_dedup_limit_s"
            _dedup_ok="false"
        fi
    fi

    if [ "$_dedup_ok" = "true" ]; then
        local _sha _count _runs_on_sha _streak _dom_ratio
        _sha="$(printf '%s' "$_streak_data" | python3 -c 'import json,sys; print(json.load(sys.stdin)["dominant_sha"])')"
        _count="$(printf '%s' "$_streak_data" | python3 -c 'import json,sys; print(json.load(sys.stdin)["dominant_count"])')"
        _streak="$(printf '%s' "$_streak_data" | python3 -c 'import json,sys; print(json.load(sys.stdin)["streak"])')"
        _dom_ratio="$(printf '%s' "$_streak_data" | python3 -c 'import json,sys; print(json.load(sys.stdin)["ratio"])')"
        _runs_on_sha="$(printf '%s' "$_streak_data" | python3 -c 'import json,sys; print(",".join(str(x) for x in json.load(sys.stdin)["runs_on_sha"]))')"

        {
            printf '%s\n' '---FLAKY MARKER BEGIN---'
            if [ -n "$_open_issue" ]; then
                printf 'gh issue comment %s --repo %s \\\n' "$_open_issue" "$GH_REPO"
                printf '  --body <<<EOF_BODY_MARKER\n'
            else
                printf '%s\n' "(no OPEN_E2E_FAIL_ISSUE — log-only, no gh call)"
            fi
            printf '%s\n' "[flaky-detect] strea flaky, не bug: ${_streak} fails подряд, ${_count}/${_streak} на sha=${_sha} (ratio=${_dom_ratio}, threshold=${FLAKY_DETECT_MIN}/${FLAKY_DETECT_RATIO}). Не открываю fail-streak issue — пишу marker для ручного разбора."
            printf '%s\n' "run IDs на этом sha: ${_runs_on_sha}"
            printf '%s\n' 'EOF_BODY_MARKER'
            printf '%s\n' '---FLAKY MARKER END---'
        }
        date -u +%s > "$FLAKY_DEDUP_FILE"
        printf '  flaky-dedup written: %s\n' "$FLAKY_DEDUP_FILE"
    fi
}

# --- decide_and_print_scenario _name _fixture _open_issue _set_dedup_fn ----
decide_and_print_scenario() {
    local _name="$1" _fixture="$2" _open_issue="$3" _set_dedup_fn="$4"
    printf '\nscenario %s\n' "$_name"
    "$_set_dedup_fn"

    local _decision_payload
    _decision_payload="$(OPEN_E2E_FAIL_ISSUE="$_open_issue" compute_flaky_decision <<< "$_fixture")"
    local _decision="${_decision_payload%%|*}"
    local _data="${_decision_payload#*|}"
    echo "  decision: $_decision"
    echo "  data: $_data"

    case "$_decision" in
        silent)
            echo "  (silent — streak < threshold)"
            ;;
        regression)
            emit_regression_payload "$_data"
            ;;
        flaky)
            emit_flaky_marker "$_data"
            ;;
        *)
            echo "  ERROR: unknown decision: $_decision"
            ;;
    esac
}

# --- mtime setup helpers (faking flaky-dedup across scenarios) --------------
# shellcheck disable=SC2329  # invoked indirectly through decide_and_print_scenario
dedup_cold_start() { rm -f "$FLAKY_DEDUP_FILE"; }
# shellcheck disable=SC2329
dedup_now() { date -u +%s > "$FLAKY_DEDUP_FILE"; }
# shellcheck disable=SC2329
dedup_aged_over_limit() {
    local _now _aged
    _now="$(date -u +%s)"
    _aged=$(( _now - FLAKY_DEDUP_HOURS * 3600 - 60 ))
    touch -d "@${_aged}" "$FLAKY_DEDUP_FILE"
}

# --- main -------------------------------------------------------------------
printf '=== dry-run harness: %s ===\n' "$(date -Iseconds)"
printf 'GH_REPO=%s\n' "$GH_REPO"
printf 'E2E_WORKFLOW=%s\n' "$E2E_WORKFLOW"
printf 'FLAKY_DETECT_MIN=%s FLAKY_DETECT_RATIO=%s FLAKY_DEDUP_HOURS=%s\n' \
    "$FLAKY_DETECT_MIN" "$FLAKY_DETECT_RATIO" "$FLAKY_DEDUP_HOURS"
printf 'FLAKY_DEDUP_FILE=%s\n' "$FLAKY_DEDUP_FILE"

decide_and_print_scenario \
    "(i) flaky 5 same sha (практический кейс Шифу)" \
    "$FIXTURE_FLAKY_5_SAME_SHA" "" dedup_cold_start

decide_and_print_scenario \
    "(ii) regression 5 different shas" \
    "$FIXTURE_REGRESSION_5_DIFF_SHA" "" dedup_cold_start

decide_and_print_scenario \
    "(iii) flaky-border 3 same + 2 diff (ratio=0.6 — пограничный)" \
    "$FIXTURE_FLAKY_BORDER" "" dedup_cold_start

decide_and_print_scenario \
    "(iv) silent streak=1 < threshold (частичная регрессия)" \
    "$FIXTURE_SILENT" "" dedup_cold_start

# --- summary ---------------------------------------------------------------
# Подсчитываем сколько раз harness напечатал маркеры.
# Invariants:
#   (i)   flaky → 1 FLAKY MARKER
#   (ii)  regression → 1 REGRESSION PAYLOAD
#   (iii) flaky → 1 FLAKY MARKER
#   (iv)  silent → 0 markers
# ИТОГО: 2 FLAKY MARKER, 1 REGRESSION PAYLOAD.
_out="$( (
    dedup_cold_start; decide_and_print_scenario "(i)"   "$FIXTURE_FLAKY_5_SAME_SHA"       "" dedup_cold_start
    dedup_cold_start; decide_and_print_scenario "(ii)"  "$FIXTURE_REGRESSION_5_DIFF_SHA"  "" dedup_cold_start
    dedup_cold_start; decide_and_print_scenario "(iii)" "$FIXTURE_FLAKY_BORDER"            "" dedup_cold_start
    dedup_cold_start; decide_and_print_scenario "(iv)"  "$FIXTURE_SILENT"                  "" dedup_cold_start
) 2>&1 )" || true

_regression_count="$(printf '%s\n' "$_out" | grep -c 'REGRESSION PAYLOAD BEGIN---' || true)"
_flaky_count="$(printf '%s\n' "$_out" | grep -c 'FLAKY MARKER BEGIN---' || true)"

if [ "${_regression_count:-0}" -ne 1 ]; then
    printf '\n!!! harness invariant violated: expected 1 regression payload, got %s\n' "${_regression_count:-0}"
    exit 1
fi
if [ "${_flaky_count:-0}" -ne 2 ]; then
    printf '\n!!! harness invariant violated: expected 2 flaky markers, got %s\n' "${_flaky_count:-0}"
    exit 1
fi

printf '\n=== summary: 1 regression + 2 flaky markers — PASS ===\n'
exit 0
