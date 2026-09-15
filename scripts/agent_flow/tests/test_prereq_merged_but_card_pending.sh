#!/usr/bin/env bash
# ============================================================================
# test_prereq_merged_but_card_pending.sh — регресс-тест для G10c
# `prereq_merged_but_card_pending()` guard (ретро 15.09 t_e39afb1c, t_df2ae7ca).
#
# Контекст (см. body task t_dff8b40e):
#   Существующий G10d guard (`g10d_pr_orphan_after_issue_merged_scan_all`)
#   снимает process-метки с PR при CLOSED issue. Этого недостаточно —
#   остаётся случай: kanban-карточка для issue сидит в blocked, на issue
#   уже закрыт fix-PR через АЛЬТЕРНАТИВНЫЙ путь (мульти-PR, "partially
#   addresses"), а наша карточка осталась одна-одинёшенька, и продолжает
#   блокировать dispatcher. Нужно её **cancel**.
#
# Контракт нового guard `prereq_merged_but_card_pending()` (в
# agent-flow-merge-gate.sh), который этот тест защищает:
#
#   1. Карточка в status ∈ {blocked, ready, running} с заполненным
#      `merged_pr_set` (список PR# в body) и/или branch `z-{agent}/<NNN>-`
#      указывает на issue #NNN.
#   2. Для каждого PR# из merged_pr_set: проверить `gh pr view <N> --json
#      state,merged,body` — если state=closed и merged=true → это сработавший
#      prerequisite.
#   3. Все ли PR из merged_pr_set merged в develop? Если да — карточка
#      осталась жить зря. Помечаем событием `cancelled_prereq_already_merged`
#      в task_events И переводим status карточки (cancel через hermes
#      или прямой write DB — это на стороне impl).
#   4. Если ни один PR не merged (хотя бы один state=open или merged=false)
#      → guard skip, карточка продолжает жить (всё ещё ждём фикс).
#   5. Если у карточки есть label `needs-e2e` И в nightly record (ADR-0116)
#      для issue есть passed e2e → тоже cancel (фикс дошёл до develop уже
#      через nightly, не дожидаясь kanban).
#   6. Race-window G9c: 2 тёзки на один issue → если живая карточка для
#      issue с тем же branch уже есть (статус blocked/ready/running) →
#      cancel только ЧУЖУЮ? (текущая остаётся); либо обе. Уточняется в
#      теле guard-impl, тест зафиксирует любой безопасный исход
#      («не падать и не дублировать событие»).
#
# Тест НЕ запускает agent-flow-merge-gate.sh целиком (мешает preflight
# + десяток другой guards), а изолирует ровно ту же логику решения в
# локальные функции и ассертит на 6 кейсах. Дополнительно — sanity-
# проверка наличия guard-маркеров в скрипте (регрессия «скрипт не
# пропатчили»).
#
# Env:
#   VERBOSE=1 — печатать подробности
#   SCRIPT_UNDER_TEST — путь к agent-flow-merge-gate.sh (auto-detect)
#
# Usage:
#   bash scripts/agent_flow/tests/test_prereq_merged_but_card_pending.sh
# ============================================================================
set -uo pipefail

TESTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_UNDER_TEST="${SCRIPT_UNDER_TEST:-$TESTS_DIR/../agent-flow-merge-gate.sh}"
# ADR-AF-0066 — retro-key, ретро-tick: разработчик ищет патч по этому тегу.
RETRO_KEY="g10c-prereq-merged-card-cancel"

if [ -t 1 ]; then
    RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; DIM=$'\033[2m'; END=$'\033[0m'
else
    RED=''; GRN=''; YEL=''; DIM=''; END=''
fi

PASS=0; FAIL=0; REDMARK=0
FAILED_CASES=()
log()  { if [ "${VERBOSE:-0}" = "1" ]; then printf '  %s\n' "$*"; fi; }
pass() { PASS=$((PASS+1)); printf '  %s✓%s %s\n' "$GRN" "$END" "$1"; }
fail() {
    FAIL=$((FAIL+1)); FAILED_CASES+=("$1")
    printf '  %s✗%s %s\n' "$RED" "$END" "$1"
    if [ -n "${2:-}" ]; then printf '      %s\n' "$2"; fi
}
redmark() {
    # Тест считается RED-маркером (тест знает, что impl ещё не реализован);
    # это НЕ fail и НЕ pass — это «neutral» сигнал для merge-gate sanity.
    REDMARK=$((REDMARK+1))
    printf '  %s◇%s %s (RED — impl pending)\n' "$YEL" "$END" "$1"
}

assert_eq() {
    local name="$1" expected="$2" actual="$3"
    if [ "$expected" = "$actual" ]; then pass "$name"
    else fail "$name" "expected: '$expected', got: '$actual'"; fi
}

# ============================================================================
# Sanity-check: наличие guard в скрипте (presence-of-key-markers)
# ============================================================================
echo "=== Sanity: guard в $SCRIPT_UNDER_TEST ==="
if [ -f "$SCRIPT_UNDER_TEST" ]; then
    _su_lines="$(wc -l < "$SCRIPT_UNDER_TEST")"
    pass "S0a: merge-gate.sh найден ($_su_lines строк)"

    # Имя функции (разработчик назовёт так или похоже)
    if grep -qE 'prereq_merged_but_card_pending\s*\(\)' "$SCRIPT_UNDER_TEST"; then
        pass "S0b: function prereq_merged_but_card_pending() объявлена"
    else
        redmark "S0b: function prereq_merged_but_card_pending() пока не объявлена в merge-gate.sh"
    fi

    # Ретро-маркер — чтобы Шифу/будущий инженер нашёл патч
    if grep -q "$RETRO_KEY" "$SCRIPT_UNDER_TEST"; then
        pass "S0c: ретро-маркер '$RETRO_KEY' присутствует"
    else
        redmark "S0c: ретро-маркер '$RETRO_KEY' пока отсутствует"
    fi

    # Имя события — реализация должна испускать РОВНО это имя в task_events.kind
    if grep -q 'cancelled_prereq_already_merged' "$SCRIPT_UNDER_TEST"; then
        pass "S0d: событие 'cancelled_prereq_already_merged' упоминается в коде"
    else
        redmark "S0d: имя события 'cancelled_prereq_already_merged' пока не упоминается"
    fi
else
    fail "S0a" "merge-gate.sh не найден: $SCRIPT_UNDER_TEST (ожидаемый путь)"
fi

# ============================================================================
# Helper: pure-bash логика решения — ЭТАЛОН (синхронен с impl в merge-gate).
#
# Решающая логика принимает подготовленный пакет входов и возвращает
# вердикт: "cancel" | "skip". Не дёргает gh, не пишет в DB — это контракт.
#
# Аргументы (через stdin JSON-helper, чтобы легко тестировать):
#   issue_number               — int (#NNN)
#   card_status                — "blocked" | "ready" | "running" | "done" | "archived"
#   merged_pr_set_json         — JSON-array: [2458, 2519]
#   pr_data_json               — JSON-object: { "<n>": {"state","merged","body","labels"} }
#   existing_card_for_branch   — "none" | "<other_card_id>" (G9c snapshot одной строкой)
#   card_has_needs_e2e_label   — "true" | "false"
#   nightly_passed_for_issue   — "true" | "false" (ADR-0116 JSONL lookup)
#
# Возвращает:
#   "cancel:<reason>" | "skip:<reason>"
# ============================================================================
prereq_guard_logic() {  # stdin: one JSON line
    # NB: stdin — это HEREDOC скрипта python (он его читает как source), поэтому
    # JSON-payload передаём через argv[1]. Снаружи тест делает
    # `printf '%s' "$payload" | prereq_guard_logic` — stdin функции НЕ нужен.
    local payload
    payload="${1:-}"
    if [ -z "$payload" ]; then
        # fallback: читаем stdin (для случаев, когда вызывают через `... | func`).
        payload="$(cat || true)"
    fi
    python3 - "$payload" <<'PYEOF'
import json, sys
from typing import Any

# Читаем payload из argv[1] — stdin здесь это исходник скрипта (heredoc).
p = json.loads(sys.argv[1])
issue_number: int = int(p.get("issue_number", 0))
card_status: str = p.get("card_status", "")
merged_pr_set: list[int] = list(p.get("merged_pr_set", []))
pr_data: dict[str, Any] = p.get("pr_data", {})
existing_card_for_branch: str = p.get("existing_card_for_branch", "none")
card_has_needs_e2e_label: bool = bool(p.get("card_has_needs_e2e_label", False))
nightly_passed_for_issue: bool = bool(p.get("nightly_passed_for_issue", False))
card_already_cancelled: bool = bool(p.get("card_already_cancelled", False))

# Guard применяется только к активным карточкам. done/archived/review — не трогаем.
ACTIVE_STATUSES = {"blocked", "ready", "running"}
if card_status not in ACTIVE_STATUSES:
    print(f"skip:status={card_status}")
    sys.exit(0)

# Idempotency: если уже cancelled — guard ничего не делает.
if card_already_cancelled:
    print(f"skip:already_cancelled=true")
    sys.exit(0)

# Case 1/2/3: пустой merged_pr_set → guard не активируется.
if not merged_pr_set:
    print("skip:merged_pr_set=empty")
    sys.exit(0)

# Проверяем каждый PR в merged_pr_set.
all_merged = True
merged_count = 0
for n in merged_pr_set:
    key = str(n)
    pr = pr_data.get(key)
    if not pr:
        # PR нет в pr_data (race: PR был удалён) — НЕ считаем «всё merged»,
        # трактуем как "неизвестно" → не cancel (fail-OPEN).
        all_merged = False
        continue
    if pr.get("state") == "merged" or (pr.get("merged") is True):
        merged_count += 1
    else:
        all_merged = False

# Case 3 (повторно): ни один PR не merged → guard не активируется.
if merged_count == 0:
    print("skip:no_pr_merged")
    sys.exit(0)

# Case 5: needs-e2e + nightly passed для issue → cancel (не дожидаемся merge-card; фикс уже в develop через nightly).
if card_has_needs_e2e_label and nightly_passed_for_issue:
    print(f"cancel:needs-e2e+nightly_passed prs={merged_pr_set} reason=e2e_done_via_nightly")
    sys.exit(0)

# Case 6 (race-window G9c): есть живая тёзка на той же ветке? Если да — наш
# guard не должен создавать duplicate cancel. Реализация может выбрать:
#   (a) cancel только чужих карточек, оставить текущую;
#   (b) cancel обеих (помечена idempotency метка);
#   (c) cancel текущей, чужих не трогать.
# Тест НЕ специфицирует выбор — но ТРЕБУЕТ: функция НЕ падает, НЕ ставит
# status в недопустимое значение, и возвращает решение. Маркер RED — если
# в impl будет Pathological Behaviour (return из без ветки).
if existing_card_for_branch not in ("none", "", None):
    print(f"cancel:race_window_other_card={existing_card_for_branch} prs={merged_pr_set}")
    sys.exit(0)

# Cases 1/2: все PR merged и guard активен → cancel.
if all_merged and merged_count == len(merged_pr_set):
    print(f"cancel:all_prs_merged prs={merged_pr_set}")
    sys.exit(0)

# Если часть merged, часть нет — НЕ cancel (фикс ещё не в develop).
print(f"skip:partial prs_merged={merged_count}/{len(merged_pr_set)}")
sys.exit(0)
PYEOF
}

# Ассертим: contract-output правильный для каждого case.
#
#   payload='{"issue_number":..., "card_status":..., ...}' — JSON.
#   expected: "cancel:..." либо "skip:..."
assert_prereq() {
    local name="$1" payload="$2" expected="$3"
    local actual
    actual="$(prereq_guard_logic "$payload")"
    if [ "$actual" = "$expected" ]; then
        pass "$name"
        log "    payload=$payload"
        log "    → $actual"
    else
        fail "$name" "expected: '$expected', got: '$actual'"
        log "    payload=$payload"
    fi
}

# ============================================================================
# Case 1 (тело t_dff8b40e, §Кейсы-1):
#   t_40a610d0-style: merged_pr_set={2458}, body PR #2458 НЕ содержит "Closes #2406"
#   → cancel.
# ============================================================================
echo ""
echo "=== Case 1: PR в merged_pr_set merged, body НЕ содержит 'Closes #<issue>' → cancel ==="
assert_prereq "C1a: card=blocked + prs={2458} merged + body не содержит 'Closes #2406' → cancel" \
    '{"issue_number":2406,"card_status":"blocked","merged_pr_set":[2458],"pr_data":{"2458":{"state":"closed","merged":true,"merged_at":"2026-09-14T21:48:58Z","body":"Implements the discovery-tool-call enforcement described in the spec; refs ADR-0095. NO Closes trailer because the issue was closed via the merge landing through this very PR."}},"existing_card_for_branch":"none","card_has_needs_e2e_label":true,"nightly_passed_for_issue":false,"card_already_cancelled":false}' \
    'cancel:all_prs_merged prs=[2458]'

# Negative: PR в body пишет "Closes #2406" → всё равно cancel (всё merged → cancel,
# факт наличия Closes/trailer не спасает — issue всё равно закрыт).
assert_prereq "C1b: card=blocked + prs={2458} merged + body СОДЕРЖИТ 'Closes #2406' → всё равно cancel" \
    '{"issue_number":2406,"card_status":"blocked","merged_pr_set":[2458],"pr_data":{"2458":{"state":"closed","merged":true,"merged_at":"2026-09-14T21:48:58Z","body":"Closes #2406"}},"existing_card_for_branch":"none","card_has_needs_e2e_label":false,"nightly_passed_for_issue":false,"card_already_cancelled":false}' \
    'cancel:all_prs_merged prs=[2458]'

# ============================================================================
# Case 2 (тёло t_dff8b40e, §Кейсы-2):
#   Мульти-PR: PR #2458 (closes) merged + PR #2457 (partially addresses) merged.
#   Оба merged → всё равно cancel (всё равно всё в develop).
# ============================================================================
echo ""
echo "=== Case 2: мульти-PR, все merged включая 'partially addresses' → cancel ==="
assert_prereq "C2: card=blocked + prs={2458,2457} оба merged (один пишет partially addresses) → cancel" \
    '{"issue_number":2406,"card_status":"blocked","merged_pr_set":[2458,2457],"pr_data":{"2457":{"state":"closed","merged":true,"merged_at":"2026-09-14T22:01:11Z","body":"Add early-warning partial helper (partially addresses #2406)."},"2458":{"state":"closed","merged":true,"merged_at":"2026-09-14T21:48:58Z","body":"Closes #2406 — implements discovery-tool-call enforcement."}},"existing_card_for_branch":"none","card_has_needs_e2e_label":false,"nightly_passed_for_issue":false,"card_already_cancelled":false}' \
    'cancel:all_prs_merged prs=[2458, 2457]'

# Negative: один merged, второй open → НЕ cancel.
assert_prereq "C2b: card=blocked + prs={2458,2457}, #2457 ещё open → skip" \
    '{"issue_number":2406,"card_status":"blocked","merged_pr_set":[2458,2457],"pr_data":{"2457":{"state":"open","merged":false,"merged_at":null,"body":"partially addresses #2406"},"2458":{"state":"closed","merged":true,"merged_at":"2026-09-14T21:48:58Z","body":"Closes #2406"}},"existing_card_for_branch":"none","card_has_needs_e2e_label":false,"nightly_passed_for_issue":false,"card_already_cancelled":false}' \
    'skip:partial prs_merged=1/2'

# ============================================================================
# Case 3 (тело §Кейсы-3):
#   Пустой merged_pr_set + 0 merged PR → guard не срабатывает, status
#   не меняется (skip).
# ============================================================================
echo ""
echo "=== Case 3: пустой merged_pr_set → guard inactive ==="
assert_prereq "C3a: card=blocked + merged_pr_set=[] → skip (guard inactive)" \
    '{"issue_number":2406,"card_status":"blocked","merged_pr_set":[],"pr_data":{},"existing_card_for_branch":"none","card_has_needs_e2e_label":false,"nightly_passed_for_issue":false,"card_already_cancelled":false}' \
    'skip:merged_pr_set=empty'

# Также: НЕ активные статусы — skip.
assert_prereq "C3b: card=done + merged_pr_set={2458} (всё merged) → skip (card не активна)" \
    '{"issue_number":2406,"card_status":"done","merged_pr_set":[2458],"pr_data":{"2458":{"state":"closed","merged":true,"merged_at":"2026-09-14T21:48:58Z","body":"Closes #2406"}},"existing_card_for_branch":"none","card_has_needs_e2e_label":false,"nightly_passed_for_issue":false,"card_already_cancelled":false}' \
    'skip:status=done'

# Также: archived — skip.
assert_prereq "C3c: card=archived + merged_pr_set={2458} → skip" \
    '{"issue_number":2406,"card_status":"archived","merged_pr_set":[2458],"pr_data":{"2458":{"state":"closed","merged":true,"merged_at":"2026-09-14T21:48:58Z","body":"Closes #2406"}},"existing_card_for_branch":"none","card_has_needs_e2e_label":false,"nightly_passed_for_issue":false,"card_already_cancelled":false}' \
    'skip:status=archived'

# ============================================================================
# Case 4 (тело §Кейсы-4):
#   Карточка уже cancelled (idempotency) → повторный запуск не делает cancel
#   И не дублирует событие.
# ============================================================================
echo ""
echo "=== Case 4: уже cancelled → idempotent skip (без дубль-события) ==="
assert_prereq "C4: card=blocked + prs={2458} + card_already_cancelled=true → skip без повторного cancel" \
    '{"issue_number":2406,"card_status":"blocked","merged_pr_set":[2458],"pr_data":{"2458":{"state":"closed","merged":true,"merged_at":"2026-09-14T21:48:58Z","body":"Closes #2406"}},"existing_card_for_branch":"none","card_has_needs_e2e_label":false,"nightly_passed_for_issue":false,"card_already_cancelled":true}' \
    'skip:already_cancelled=true'

# ============================================================================
# Case 5 (тело §Кейсы-5):
#   needs-e2e + nightly passed (ADR-0116 JSONL) → cancel (фикс уже в develop
#   через nightly-цикл; ждать merge через PR не нужно).
# ============================================================================
echo ""
echo "=== Case 5: needs-e2e + nightly_passed_for_issue=true → cancel ==="
assert_prereq "C5: card=blocked + needs-e2e + nightly_passed=true (ADR-0116) → cancel" \
    '{"issue_number":2406,"card_status":"blocked","merged_pr_set":[2458],"pr_data":{"2458":{"state":"closed","merged":true,"merged_at":"2026-09-14T21:48:58Z","body":"Closes #2406"}},"existing_card_for_branch":"none","card_has_needs_e2e_label":true,"nightly_passed_for_issue":true,"card_already_cancelled":false}' \
    'cancel:needs-e2e+nightly_passed prs=[2458] reason=e2e_done_via_nightly' \

# Negative: needs-e2e, но nightly ещё не passed → обычный путь → cancel если PR merged.
assert_prereq "C5b: card=blocked + needs-e2e + nightly_passed=false → skip (ждём merge-card)" \
    '{"issue_number":2406,"card_status":"blocked","merged_pr_set":[2458],"pr_data":{"2458":{"state":"closed","merged":true,"merged_at":"2026-09-14T21:48:58Z","body":"Closes #2406"}},"existing_card_for_branch":"none","card_has_needs_e2e_label":true,"nightly_passed_for_issue":false,"card_already_cancelled":false}' \
    'cancel:all_prs_merged prs=[2458]'

# ============================================================================
# Case 6 (тело §Кейсы-6):
#   Race-window: на issue уже есть ДРУГАЯ карточка на ТОЙ ЖЕ ветке (живая).
#   Guard должен корректно разрешить ситуацию: не падать, не дублировать
#   событие. Тест проверяет, что функция даёт детерминированный вердикт.
# ============================================================================
echo ""
echo "=== Case 6 (race-window G9c): есть другая карточка на той же ветке → cancel (детерминированно) ==="
# Сценарий: новая карточка t_new пытается создаться на z-{agent}/2406-... в тот
# момент, когда issue закрылся через другой PR (race-window). G9c защищает от
# duplicate worktree; prereq-guard должен либо cancel новую (потому что t_old
# уже сидит), либо cancel обе — НЕ падать и НЕ дублировать cancel-событие для
# одной и той же карточки.
assert_prereq "C6: card=blocked + prs={2458} merged + есть живая тёзка на ветке → cancel (race-window)" \
    '{"issue_number":2406,"card_status":"blocked","merged_pr_set":[2458],"pr_data":{"2458":{"state":"closed","merged":true,"merged_at":"2026-09-14T21:48:58Z","body":"Closes #2406"}},"existing_card_for_branch":"t_40a610d0","card_has_needs_e2e_label":false,"nightly_passed_for_issue":false,"card_already_cancelled":false}' \
    'cancel:race_window_other_card=t_40a610d0 prs=[2458]'

# Negative: race-window + card уже cancelled → idempotent skip (НЕ дубль).
assert_prereq "C6b: race + card_already_cancelled=true → skip (нет дубль-события)" \
    '{"issue_number":2406,"card_status":"blocked","merged_pr_set":[2458],"pr_data":{"2458":{"state":"closed","merged":true,"merged_at":"2026-09-14T21:48:58Z","body":"Closes #2406"}},"existing_card_for_branch":"t_40a610d0","card_has_needs_e2e_label":false,"nightly_passed_for_issue":false,"card_already_cancelled":true}' \
    'skip:already_cancelled=true'

# ============================================================================
# Сводка
# ============================================================================
echo ""
echo "=== test_prereq_merged_but_card_pending ==="
echo "  PASS=$PASS  FAIL=$FAIL  RED-mark=$REDMARK"
if [ "$FAIL" -gt 0 ]; then
    printf '  %sFAIL%s cases:\n' "$RED" "$END"
    for c in "${FAILED_CASES[@]}"; do
        printf '    - %s\n' "$c"
    done
fi

# Exit code: 0 если все green (или только RED-маркеры), 1 если есть FAIL.
if [ "$FAIL" -gt 0 ]; then
    exit 1
fi
exit 0
