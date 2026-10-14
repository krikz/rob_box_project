#!/bin/bash
# ============================================================================
# test_triage_dedup_iso_date_collision.sh — regression test для G9a intra-tick
#   dedup, когда разные deploy-issue одной серии ложно схлопываются как дубли
#   из-за того, что title_prefix() отбрасывает YYYY-MM-DD дату при
#   tokenization (issue #3374, retro t_cdd9524a, ADR-AF-0080).
#
# Background:
#   #3346 «🚨 Deploy issues on develop (staging) — 2026-10-02» vs
#   #3354 «🚨 Deploy issues on develop (staging) — 2026-10-03» —
#   разные deploy-инциденты, но title_prefix(first-6) для обоих возвращал
#     «deploy issues on develop staging 2026»
#   (регекс [\w]+ режет дефис, поэтому «2026-10-02» токенизировался как
#   «2026», «10», «02» — и в первые 6 попадало только «2026»).
#   → G9a оставлял #3346 (старейшую) и skip-ал #3354 → 19+ часов orphan.
#
# Acceptance (ADR-AF-0080):
#   T1: разные даты (10-02 vs 10-03) → 2 kept, 0 markers (НЕ дубль).
#   T2: одна дата (10-02 vs 10-02) → 1 kept, 1 marker (настоящий дубль).
#   T3: разные даты в ОБРАТНОМ порядке (10-03 vs 10-02) → 2 kept, 0 markers.
#   T4: 3 deploy-issue (10-01, 10-02, 10-03) → 3 kept, 0 markers.
#   T5: 2 deploy-issue с одной датой + 1 с другой (10-02, 10-02, 10-03) →
#       2 kept (лидер=10-02 старейшая + лидер=10-03), 1 marker.
#   T6: regression — title_prefix() теперь содержит «YYYY-MM-DD» атомарно.
#   T7: presence check — код содержит комментарий с упоминанием
#       «t_cdd9524a» / «#3374» / «ADR-AF-0080».
#
# Использование: bash test_triage_dedup_iso_date_collision.sh
# Env: VERBOSE=1 — подробный лог
# ============================================================================
set -uo pipefail

TESTS_DIR="$(cd "$(dirname "$0")" && pwd)"
SCRIPT_UNDER_TEST="$TESTS_DIR/../agent-flow-triage.sh"

PASS=0
FAIL=0
FAILED_CASES=()

log() { if [ "${VERBOSE:-0}" = "1" ]; then printf '  %s\n' "$*"; fi; }
pass() { PASS=$((PASS+1)); printf '  \033[32m✓\033[0m %s\n' "$1"; }
fail() {
    FAIL=$((FAIL+1)); FAILED_CASES+=("$1")
    printf '  \033[31m✗\033[0m %s\n' "$1"
    if [ -n "${2:-}" ]; then printf '      %s\n' "$2"; fi
}

# --- Load helpers from triage.sh ---------------------------------------------
TMP_HELPERS="$(mktemp -t test-iso-helpers.XXXXXX)" || { echo "mktemp failed"; exit 1; }
trap 'rm -f "$TMP_HELPERS"' EXIT

cat > "$TMP_HELPERS" <<'SRC'
log() { printf '%s %s %s\n' "[test]" "$(date -Iseconds)" "$*" >&2; }
SRC

awk '/^dedup_intra_filter\(\) \{/,/^}/' "$SCRIPT_UNDER_TEST" >> "$TMP_HELPERS"

# shellcheck disable=SC1090
. "$TMP_HELPERS"

if ! command -v dedup_intra_filter >/dev/null 2>&1; then
    echo "ERROR: failed to load dedup_intra_filter from $SCRIPT_UNDER_TEST"
    exit 1
fi

export DRY_RUN=true
export GH_REPO=""
export AGENT_FLOW_DEDUP_TITLE_PREFIX_WORDS=6

# Common labels (как у настоящих deploy-issue: deployment/hermes/agent:devops).
DEPLOY_LABELS='[{"name":"deployment"},{"name":"hermes"},{"name":"agent:devops"}]'

# --- T1: разные даты (10-02 vs 10-03) → 2 kept, 0 markers ---------------------
echo ""
echo "=== T1: deploy-issue с разными датами → НЕ дубль (regression для #3374) ==="
FIXTURE_T1='[
  {"number": 3346, "title": "🚨 Deploy issues on develop (staging) — 2026-10-02", "labels": '"$DEPLOY_LABELS"', "body": ""},
  {"number": 3354, "title": "🚨 Deploy issues on develop (staging) — 2026-10-03", "labels": '"$DEPLOY_LABELS"', "body": ""}
]'
ERR_T1="$(mktemp -t g9a-iso-t1.XXXXXX)"
OUT_T1="$(dedup_intra_filter "phase1" "$FIXTURE_T1" 2>"$ERR_T1" || true)"
COUNT_T1_KEEP="$(printf '%s' "$OUT_T1" | python3 -c 'import json,sys; print(len(json.load(sys.stdin)))' 2>/dev/null || echo 0)"
COUNT_T1_MARKERS="$(grep -c '^DEDUP_INTRA' "$ERR_T1" || true)"
log "T1: kept=$COUNT_T1_KEEP markers=$COUNT_T1_MARKERS"
if [ "$COUNT_T1_KEEP" = "2" ] && [ "$COUNT_T1_MARKERS" = "0" ]; then
    pass "T1: deploy-issue с разными датами → 2 kept, 0 markers (regression fix)"
else
    fail "T1: expected kept=2, markers=0; got kept=$COUNT_T1_KEEP, markers=$COUNT_T1_MARKERS" \
        "stdout=$OUT_T1 stderr=$(cat "$ERR_T1")"
fi
rm -f "$ERR_T1"

# --- T2: одна дата (10-02 vs 10-02) → 1 kept, 1 marker (настоящий дубль) ----
echo ""
echo "=== T2: deploy-issue с одинаковой датой → дубль (1 kept, 1 marker) ==="
FIXTURE_T2='[
  {"number": 3346, "title": "🚨 Deploy issues on develop (staging) — 2026-10-02 (retry 1)", "labels": '"$DEPLOY_LABELS"', "body": ""},
  {"number": 3347, "title": "🚨 Deploy issues on develop (staging) — 2026-10-02 (retry 2)", "labels": '"$DEPLOY_LABELS"', "body": ""}
]'
ERR_T2="$(mktemp -t g9a-iso-t2.XXXXXX)"
OUT_T2="$(dedup_intra_filter "phase1" "$FIXTURE_T2" 2>"$ERR_T2" || true)"
COUNT_T2_KEEP="$(printf '%s' "$OUT_T2" | python3 -c 'import json,sys; print(len(json.load(sys.stdin)))' 2>/dev/null || echo 0)"
COUNT_T2_MARKERS="$(grep -c '^DEDUP_INTRA' "$ERR_T2" || true)"
log "T2: kept=$COUNT_T2_KEEP markers=$COUNT_T2_MARKERS"
if [ "$COUNT_T2_KEEP" = "1" ] && [ "$COUNT_T2_MARKERS" = "1" ]; then
    pass "T2: deploy-issue с одинаковой датой → 1 kept (старейшая), 1 marker"
else
    fail "T2: expected kept=1, markers=1; got kept=$COUNT_T2_KEEP, markers=$COUNT_T2_MARKERS" \
        "stdout=$OUT_T2"
fi
rm -f "$ERR_T2"

# --- T3: разные даты в обратном порядке (10-03 vs 10-02) → 2 kept, 0 markers -
echo ""
echo "=== T3: deploy-issue с разными датами в обратном input-order → 2 kept ==="
FIXTURE_T3='[
  {"number": 3354, "title": "🚨 Deploy issues on develop (staging) — 2026-10-03", "labels": '"$DEPLOY_LABELS"', "body": ""},
  {"number": 3346, "title": "🚨 Deploy issues on develop (staging) — 2026-10-02", "labels": '"$DEPLOY_LABELS"', "body": ""}
]'
ERR_T3="$(mktemp -t g9a-iso-t3.XXXXXX)"
OUT_T3="$(dedup_intra_filter "phase1" "$FIXTURE_T3" 2>"$ERR_T3" || true)"
COUNT_T3_KEEP="$(printf '%s' "$OUT_T3" | python3 -c 'import json,sys; print(len(json.load(sys.stdin)))' 2>/dev/null || echo 0)"
COUNT_T3_MARKERS="$(grep -c '^DEDUP_INTRA' "$ERR_T3" || true)"
log "T3: kept=$COUNT_T3_KEEP markers=$COUNT_T3_MARKERS"
if [ "$COUNT_T3_KEEP" = "2" ] && [ "$COUNT_T3_MARKERS" = "0" ]; then
    pass "T3: обратный input-order тоже даёт 2 kept, 0 markers"
else
    fail "T3: expected kept=2, markers=0; got kept=$COUNT_T3_KEEP, markers=$COUNT_T3_MARKERS"
fi
rm -f "$ERR_T3"

# --- T4: 3 deploy-issue (10-01, 10-02, 10-03) → 3 kept, 0 markers -------------
echo ""
echo "=== T4: 3 deploy-issue с тремя разными датами → 3 kept ==="
FIXTURE_T4='[
  {"number": 3300, "title": "🚨 Deploy issues on develop (staging) — 2026-10-01", "labels": '"$DEPLOY_LABELS"', "body": ""},
  {"number": 3346, "title": "🚨 Deploy issues on develop (staging) — 2026-10-02", "labels": '"$DEPLOY_LABELS"', "body": ""},
  {"number": 3354, "title": "🚨 Deploy issues on develop (staging) — 2026-10-03", "labels": '"$DEPLOY_LABELS"', "body": ""}
]'
ERR_T4="$(mktemp -t g9a-iso-t4.XXXXXX)"
OUT_T4="$(dedup_intra_filter "phase1" "$FIXTURE_T4" 2>"$ERR_T4" || true)"
COUNT_T4_KEEP="$(printf '%s' "$OUT_T4" | python3 -c 'import json,sys; print(len(json.load(sys.stdin)))' 2>/dev/null || echo 0)"
COUNT_T4_MARKERS="$(grep -c '^DEDUP_INTRA' "$ERR_T4" || true)"
log "T4: kept=$COUNT_T4_KEEP markers=$COUNT_T4_MARKERS"
if [ "$COUNT_T4_KEEP" = "3" ] && [ "$COUNT_T4_MARKERS" = "0" ]; then
    pass "T4: 3 deploy-issue с разными датами → 3 kept, 0 markers"
else
    fail "T4: expected kept=3, markers=0; got kept=$COUNT_T4_KEEP, markers=$COUNT_T4_MARKERS"
fi
rm -f "$ERR_T4"

# --- T5: смешанный (10-02, 10-02-dup, 10-03) → 2 kept, 1 marker --------------
echo ""
echo "=== T5: 10-02 + 10-02 (true dups) + 10-03 → 2 kept (лидеры), 1 marker ==="
FIXTURE_T5='[
  {"number": 3346, "title": "🚨 Deploy issues on develop (staging) — 2026-10-02 attempt 1", "labels": '"$DEPLOY_LABELS"', "body": ""},
  {"number": 3347, "title": "🚨 Deploy issues on develop (staging) — 2026-10-02 attempt 2", "labels": '"$DEPLOY_LABELS"', "body": ""},
  {"number": 3354, "title": "🚨 Deploy issues on develop (staging) — 2026-10-03", "labels": '"$DEPLOY_LABELS"', "body": ""}
]'
ERR_T5="$(mktemp -t g9a-iso-t5.XXXXXX)"
OUT_T5="$(dedup_intra_filter "phase1" "$FIXTURE_T5" 2>"$ERR_T5" || true)"
COUNT_T5_KEEP="$(printf '%s' "$OUT_T5" | python3 -c 'import json,sys; print(len(json.load(sys.stdin)))' 2>/dev/null || echo 0)"
COUNT_T5_MARKERS="$(grep -c '^DEDUP_INTRA' "$ERR_T5" || true)"
log "T5: kept=$COUNT_T5_KEEP markers=$COUNT_T5_MARKERS"
if [ "$COUNT_T5_KEEP" = "2" ] && [ "$COUNT_T5_MARKERS" = "1" ]; then
    pass "T5: смешанный кейс (true dups + unique) → 2 kept, 1 marker"
else
    fail "T5: expected kept=2, markers=1; got kept=$COUNT_T5_KEEP, markers=$COUNT_T5_MARKERS" \
        "stdout=$OUT_T5"
fi
rm -f "$ERR_T5"

# --- T6: regression — title_prefix() теперь содержит «YYYY-MM-DD» атомарно --
echo ""
echo "=== T6: title_prefix сохраняет YYYY-MM-DD как атомарный токен ==="
# Inline-проверка через Python: вытаскиваем python-исходник функции и
# прогоняем через stdin в обход полной функции.
_TP_SOURCE="$(awk '/^def title_prefix\(/,/^    return " ".join\(tokens\[:n\]\)/' "$TMP_HELPERS")"
log "title_prefix source:"; log "$_TP_SOURCE"
# Inline exec: дописываем нашу фикстуру + import re (т.к. функция использует)
RESULT_T6="$(python3 -c "
import re
$_TP_SOURCE

# Test: deploy-issue #3346 vs #3354
p1 = title_prefix('🚨 Deploy issues on develop (staging) — 2026-10-02', 6)
p2 = title_prefix('🚨 Deploy issues on develop (staging) — 2026-10-03', 6)
print(repr(p1))
print(repr(p2))
print('DIFFERENT' if p1 != p2 else 'COLLIDED')
" 2>&1)"
log "T6 result: $RESULT_T6"
if echo "$RESULT_T6" | grep -q "DIFFERENT"; then
    pass "T6: title_prefix() сохраняет YYYY-MM-DD как атомарный токен (разные даты → разные prefix)"
else
    fail "T6: title_prefix() НЕ сохраняет YYYY-MM-DD (collision persists!)" "result=$RESULT_T6"
fi

# --- T7: presence check -----------------------------------------------------
echo ""
echo "=== T7: presence check — bug-context в комментариях triage.sh ==="
if grep -qE 't_cdd9524a|issue #3374|ADR-AF-0080' "$SCRIPT_UNDER_TEST"; then
    pass "T7: regression-маркер (t_cdd9524a/#3374/ADR-AF-0080) присутствует в коде"
else
    fail "T7: regression-маркер не найден — будущие воркеры не поймут, почему token-парсер такой"
fi

# --- bash syntax ---
echo ""
echo "=== bash syntax check ==="
if bash -n "$SCRIPT_UNDER_TEST" 2>/tmp/bk_err; then
    pass "bash -n syntax check passed"
else
    fail "bash -n syntax check failed" "$(cat /tmp/bk_err)"
fi

# --- summary -----------------------------------------------------------------
echo ""
echo "============================================================"
echo "PASS: $PASS    FAIL: $FAIL"
echo "============================================================"
if [ "$FAIL" -gt 0 ]; then
    printf '\nFAILED CASES:\n'
    for c in "${FAILED_CASES[@]}"; do printf '  - %s\n' "$c"; done
    exit 1
fi
exit 0