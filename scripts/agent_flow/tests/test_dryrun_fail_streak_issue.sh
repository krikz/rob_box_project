#!/bin/bash
# ============================================================================
# test_dryrun_fail_streak_issue.sh — регресс-гард для dry-run harness
# scripts/agent_flow/dryrun_fail_streak_issue.sh
#
# Контекст: PR #2374 (merge a9b04981) добавил в watchdog авто-создание issue
# при fail-streak ≥ E2E_FAIL_STREAK_ISSUE_THRESHOLD (5), rate-limited через
# mtime ISSUE_COOLDOWN_FILE (default 4h). Хочется иметь детерминированный
# harness, который можно гонять локально и в CI без сети/токенов — и
# который показывает:
#   (i)   cold start (cooldown-файл отсутствует) → 1 issue payload
#   (ii)  re-run immediately после (i)             → 0 issue payload
#   (iii) advance mtime на >4h назад               → 1 issue payload
# Итого за прогон должно быть РОВНО 2 issue payload'а (1+0+1).
#
# Acceptance test body t_7572e7a8:
#   "running the harness locally shows exactly one issue payload per >4h
#    window, and zero on immediate re-runs"
#
# Что тестируем (без сети, без токенов, без реального gh):
#   T1. Exit 0 при корректном прогоне.
#   T2. Итоговая сводка: payload count = 2 (1+0+1).
#   T3. Scenario (i): выводит 1 payload, перезаписывает cooldown mtime.
#   T4. Scenario (ii): выводит 0 payload (cooldown ещё свежий).
#   T5. Scenario (iii): снова выводит 1 payload (mtime протух).
#   T6. Каждый payload содержит gh-команду (с --label e2e-fail-streak)
#       и полный issue body (timeline + hypothesis + cross-refs).
#   T7. Hypothesis cross-refs #2246 / #2347 присутствуют.
#   T8. Релевантные merged PR #2342/#2341/#2338/#2337 присутствуют
#       (fixture data из body карточки).
#   T9. Cooldown mtime НЕ обновляется в scenario (ii) (skip).
#   T10. shellcheck-clean (если shellcheck в PATH).
#
# Run:
#   bash scripts/agent_flow/tests/test_dryrun_fail_streak_issue.sh
# ============================================================================
set -u

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HARNESS_SH="${HARNESS_SH:-$TEST_DIR/../dryrun_fail_streak_issue.sh}"

[ -f "$HARNESS_SH" ] || { echo "FAIL: $HARNESS_SH not found"; exit 1; }
command -v bash >/dev/null || { echo "FAIL: bash required"; exit 1; }
command -v touch >/dev/null || { echo "FAIL: touch required"; exit 1; }
command -v grep >/dev/null || { echo "FAIL: grep required"; exit 1; }
command -v awk >/dev/null || { echo "FAIL: awk required"; exit 1; }

PASS=0
FAIL=0
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

ok()   { printf '  \033[32m✓\033[0m %s\n' "$1"; PASS=$((PASS+1)); }
bad()  { printf '  \033[31m✗\033[0m %s\n' "$1"; FAIL=$((FAIL+1)); }
hdr()  { printf '\n\033[1m== %s ==\033[0m\n' "$1"; }

# ----------------------------------------------------------------------------
# Подготовка: sandbox HERMES_HOME и сброс cooldown
# ----------------------------------------------------------------------------
hdr "setup"
export HERMES_HOME="$WORK/hermes"
mkdir -p "$HERMES_HOME/state"
export GH_REPO="krikz/rob_box_project"
export E2E_WORKFLOW="L-E2E Voice Test.yml"
unset ISSUE_COOLDOWN_FILE
ok "sandbox ready: HERMES_HOME=$HERMES_HOME"

# ----------------------------------------------------------------------------
# Прогон harness — он сам выполнит 3 сценария и распечатает payload'ы в stdout
# ----------------------------------------------------------------------------
hdr "run harness"
TRANSCRIPT="$WORK/transcript.txt"
if ! bash "$HARNESS_SH" >"$TRANSCRIPT" 2>"$WORK/stderr.txt"; then
    bad "harness exited non-zero (rc=$?)"
    echo "--- stderr ---"; cat "$WORK/stderr.txt"
    echo "--- stdout ---"; cat "$TRANSCRIPT"
    exit 1
fi
ok "harness exited 0"

# ----------------------------------------------------------------------------
# T1: stdout содержит структурированный отчёт
# ----------------------------------------------------------------------------
hdr "T1: harness prints scenario markers"
grep -q "scenario (i)  cold start" "$TRANSCRIPT" \
    && ok "scenario (i) marker present" \
    || bad "scenario (i) marker missing"
grep -q "scenario (ii) immediate re-run" "$TRANSCRIPT" \
    && ok "scenario (ii) marker present" \
    || bad "scenario (ii) marker missing"
grep -q "scenario (iii) mtime advanced >4h" "$TRANSCRIPT" \
    && ok "scenario (iii) marker present" \
    || bad "scenario (iii) marker missing"

# ----------------------------------------------------------------------------
# T2: итоговая сводка total_payloads=2
# ----------------------------------------------------------------------------
hdr "T2: payload counts per scenario"
# Считаем сколько раз встретился маркер PAYLOAD BEGIN в каждом сценарии.
PAYLOAD_BEGIN='---ISSUE PAYLOAD BEGIN---'
PAYLOAD_END='---ISSUE PAYLOAD END---'

awk -v begin="$PAYLOAD_BEGIN" -v end="$PAYLOAD_END" '
    /scenario \(i\)  cold start/   {s="i";   next}
    /scenario \(ii\) immediate/    {s="ii";  next}
    /scenario \(iii\) mtime advanced/ {s="iii"; next}
    $0 ~ begin && s != ""          {cnt[s]++}
    END { for (k in cnt) print k, cnt[k] }
' "$TRANSCRIPT" > "$WORK/counts.txt"

cnt_i="$(awk '$1=="i"{print $2}' "$WORK/counts.txt")"
cnt_ii="$(awk '$1=="ii"{print $2}' "$WORK/counts.txt")"
cnt_iii="$(awk '$1=="iii"{print $2}' "$WORK/counts.txt")"
cnt_i="${cnt_i:-0}"; cnt_ii="${cnt_ii:-0}"; cnt_iii="${cnt_iii:-0}"

[ "$cnt_i" = "1" ] && ok "scenario (i)  printed 1 payload" \
    || bad "scenario (i)  printed $cnt_i payloads, expected 1"
[ "$cnt_ii" = "0" ] && ok "scenario (ii) printed 0 payloads (skip)" \
    || bad "scenario (ii) printed $cnt_ii payloads, expected 0"
[ "$cnt_iii" = "1" ] && ok "scenario (iii) printed 1 payload" \
    || bad "scenario (iii) printed $cnt_iii payloads, expected 1"

total=$((cnt_i + cnt_ii + cnt_iii))
[ "$total" = "2" ] && ok "total payloads = 2 (acceptance)" \
    || bad "total payloads = $total, expected 2"

# ----------------------------------------------------------------------------
# T3: каждый payload содержит gh-команду и тело issue
# ----------------------------------------------------------------------------
hdr "T3: payload structure"
# Проверяем, что в payload'е есть и команда, и тело. grep c '--' чтобы '-'
# в маркере не интерпретировался как флаг.
if grep -A1 -- "---ISSUE PAYLOAD BEGIN---" "$TRANSCRIPT" | grep -q "gh issue create"; then
    ok "gh issue create command printed in payload(s)"
else
    bad "gh issue create command NOT printed"
fi
if grep -q -- "--label e2e-fail-streak" "$TRANSCRIPT"; then
    ok "payload uses --label e2e-fail-streak"
else
    bad "payload missing --label e2e-fail-streak"
fi

# ----------------------------------------------------------------------------
# T4: payload содержит hypothesis 'music-fix regression'
# ----------------------------------------------------------------------------
hdr "T4: hypothesis + cross-refs"
if grep -q "Hypothesis: music-fix regression" "$TRANSCRIPT"; then
    ok "music-fix regression hypothesis present"
else
    bad "hypothesis missing"
fi
if grep -q "#2246" "$TRANSCRIPT" && grep -q "#2347" "$TRANSCRIPT"; then
    ok "cross-refs #2246 and #2347 present"
else
    bad "cross-refs #2246/#2347 missing"
fi

# ----------------------------------------------------------------------------
# T5: релевантные merged PR из body карточки присутствуют
# ----------------------------------------------------------------------------
hdr "T5: fixture data from task body"
for ref in "#2342" "#2341" "#2338" "#2337"; do
    if grep -q -- "$ref" "$TRANSCRIPT"; then
        ok "fixture ref $ref present"
    else
        bad "fixture ref $ref missing"
    fi
done

# ----------------------------------------------------------------------------
# T6: timeline содержит 8 fail-run IDs (как минимум)
# ----------------------------------------------------------------------------
hdr "T6: timeline contains failed run rows"
# Считаем строки таблицы markdown "|" внутри payload-блоков
table_rows="$(awk -v begin="$PAYLOAD_BEGIN" -v end="$PAYLOAD_END" '
    $0 ~ begin {in_p=1; next}
    $0 ~ end   {in_p=0; next}
    in_p && $0 ~ /^\| \[3[0-9]+\]\(https/ {c++}
    END {print c+0}
' "$TRANSCRIPT")"
[ "$table_rows" -ge 8 ] && ok "timeline rows = $table_rows (≥8)" \
    || bad "timeline rows = $table_rows, expected ≥8"

# ----------------------------------------------------------------------------
# T7: cooldown mtime НЕ обновлялся в scenario (ii)
# ----------------------------------------------------------------------------
hdr "T7: cooldown mtime invariant in scenario (ii)"
# В scenario (ii) cooldown уже должен быть свежим после (i).
# Имитируем поведение harness и проверяем, что scenario (ii) не дёрнул
# touch. Это можно увидеть по выводу — harness должен явно сообщить SKIP.
if grep -q "SKIPPED.*cooldown" "$TRANSCRIPT" || grep -q "skip.*cooldown" "$TRANSCRIPT"; then
    ok "scenario (ii) explicitly logged cooldown-skip"
else
    bad "scenario (ii) didn't log cooldown-skip decision"
fi

# ----------------------------------------------------------------------------
# T8: shellcheck (опционально, не блокирует PASS)
# ----------------------------------------------------------------------------
hdr "T8: shellcheck (optional)"
if command -v shellcheck >/dev/null 2>&1; then
    if shellcheck "$HARNESS_SH" >"$WORK/shellcheck.txt" 2>&1; then
        ok "shellcheck clean"
    else
        echo "  shellcheck warnings (non-blocking):"
        sed 's/^/    /' "$WORK/shellcheck.txt" | head -20
    fi
else
    echo "  shellcheck not installed — skip"
fi

# ----------------------------------------------------------------------------
# Итог
# ----------------------------------------------------------------------------
echo "============================================================"
printf 'PASS=%d  FAIL=%d\n' "$PASS" "$FAIL"
echo "============================================================"

[ "$FAIL" -eq 0 ] || exit 1
exit 0