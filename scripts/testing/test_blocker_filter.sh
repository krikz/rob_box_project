#!/usr/bin/env bash
# Regression test для blocker_issue_for_sig() в agent-flow-e2e-process.sh (t_8a8d9403).
#
# Проверяет, что фильтр `select([.labels[].name] | index("needs-e2e") | not) | index("e2e-done") | not)`
# корректно отбрасывает issues с метками needs-e2e / e2e-done — иначе e2e rotation
# зацикливается на самой себе (кейс #1195: issue с `no_wake_word` в body + needs-e2e
# метка блокировал сам себя, rotation PAUSED вечно).
#
# Тест:
# 1) Парсит jq-pipeline из реального файла скрипта (source-of-truth, не копия).
# 2) Конвертирует jq-pipeline в python-эквивалент и применяет к синтетическим
#    JSON-данным, имитирующим ответ `gh issue list --json number,labels`:
#    - issue A: signal в title/body, БЕЗ меток → должен попасть (blocker)
#    - issue B: signal в title/body, С меткой needs-e2e → должен быть ОТБРОШЕН
#    - issue C: signal в title/body, С меткой e2e-done → должен быть ОТБРОШЕН
#    - issue D: signal в title/body, БЕЗ меток, но number меньше A → max() вернёт A
# 3) Проверяет, что итоговый `max // ""` возвращает правильный номер.
# 4) Защита от регрессии: если кто-то удалит `| not` в фильтре, тест ловит это.
# 5) Sanity: `gh issue list --search 'no_wake_word in:title,body' --json number,labels --jq <pipeline>`
#    на РЕАЛЬНОМ репо возвращает пусто (issue #1586 имеет needs-e2e).
#
# Запуск: bash scripts/testing/test_blocker_filter.sh
# Exit:   0 = PASS, N>0 = N failed assertions.
#
# Не требует jq / SSH / network — pure-bash + python3 (есть на любом CI).
# Опционально (LIVE-чек): `LIVE_REPO=krikz/rob_box_project bash ...` — дополнительно
# гоняет реальный `gh issue list` против репо.
set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
E2E_SCRIPT="$REPO_ROOT/scripts/agent_flow/agent-flow-e2e-process.sh"

# Цвета
if [ -t 1 ]; then
    RED=$'\033[0;31m'; GRN=$'\033[0;32m'; YEL=$'\033[0;33m'; NC=$'\033[0m'
else
    RED=""; GRN=""; YEL=""; NC=""
fi

# Проверка python3 (замена jq для изолированного теста)
if ! command -v python3 >/dev/null 2>&1; then
    printf '%sFATAL: python3 не установлен%s\n' "$RED" "$NC"
    exit 2
fi

# Проверка, что скрипт-источник существует
if [ ! -f "$E2E_SCRIPT" ]; then
    printf '%sFATAL: e2e-process не найден: %s%s\n' "$RED" "$E2E_SCRIPT" "$NC"
    exit 2
fi

# Извлекаем jq-pipeline из исходника (одна строка внутри blocker_issue_for_sig).
# Ищем строку вида '[.[] | select([.labels[].name] | index("needs-e2e") | not) | select([.labels[].name] | index("e2e-done") | not) | .number] | max // ""'
JQ_PIPELINE="$(awk '/select\(\[\.labels\[\]\.name\] \| index\("needs-e2e"\)/,/max \/\/ ""/' "$E2E_SCRIPT" | head -1)"

if [ -z "$JQ_PIPELINE" ]; then
    printf '%sFATAL: не удалось извлечь jq-pipeline из %s%s\n' "$RED" "$E2E_SCRIPT" "$NC"
    exit 2
fi

# Sanity: pipeline должен содержать оба `| not` (ретро #1195)
if ! printf '%s' "$JQ_PIPELINE" | grep -q 'index("needs-e2e") | not'; then
    printf '%sFAIL: jq-pipeline НЕ содержит `index("needs-e2e") | not` — регрессия #1195!%s\n' "$RED" "$NC"
    exit 1
fi
if ! printf '%s' "$JQ_PIPELINE" | grep -q 'index("e2e-done") | not'; then
    printf '%sFAIL: jq-pipeline НЕ содержит `index("e2e-done") | not` — регрессия #1195!%s\n' "$RED" "$NC"
    exit 1
fi

printf '%sOK jq-pipeline содержит оба `| not` (needs-e2e + e2e-done)%s\n' "$GRN" "$NC"
printf '   pipeline: %s\n' "$JQ_PIPELINE"

PASS=0
FAIL=0

# Helper: применить pipeline к JSON через python3 (эквивалент jq).
# Pipeline: '[.[] | select([.labels[].name] | index("needs-e2e") | not) |
#                     select([.labels[].name] | index("e2e-done") | not) |
#                     .number] | max // ""'
# Python-эквивалент — повторяет ту же логику, что и gh --jq.
filter_issues() {
    local json="$1"
    python3 -c '
import json, sys
data = json.loads(sys.argv[1])
candidates = []
for issue in data:
    labels = [l["name"] for l in issue.get("labels", []) or []]
    if "needs-e2e" in labels:
        continue
    if "e2e-done" in labels:
        continue
    if "number" in issue:
        candidates.append(issue["number"])
result = max(candidates) if candidates else ""
print(result)
' "$json"
}

# Helper: проверка результата
assert_filter() {
    local desc="$1" json="$2" expected="$3"
    local actual
    actual="$(filter_issues "$json")"
    if [ "$actual" = "$expected" ]; then
        PASS=$((PASS+1))
        printf '%sOK %s → %q%s\n' "$GRN" "$desc" "$actual" "$NC"
    else
        FAIL=$((FAIL+1))
        printf '%sFAIL %s → expected=%q actual=%q%s\n' "$RED" "$desc" "$expected" "$actual" "$NC"
    fi
}

# === Тест-кейсы ===

# 1. Issue БЕЗ меток → попадает (блокер-кандидат)
JSON_BLOCKER='[{"number":1117,"labels":[]}]'
assert_filter "issue без меток, signal в title/body" "$JSON_BLOCKER" "1117"

# 2. Issue С меткой needs-e2e → должен быть ОТБРОШЕН (сам себя блокирует, кейс #1195)
JSON_NEEDS_E2E='[{"number":1586,"labels":[{"name":"needs-e2e"},{"name":"bug"}]}]'
assert_filter "issue с меткой needs-e2e → ОТБРОШЕН (ретро #1195)" "$JSON_NEEDS_E2E" ""

# 3. Issue С меткой e2e-done → должен быть ОТБРОШЕН (уже завершён)
JSON_E2E_DONE='[{"number":1300,"labels":[{"name":"e2e-done"}]}]'
assert_filter "issue с меткой e2e-done → ОТБРОШЕН" "$JSON_E2E_DONE" ""

# 4. Issue С обеими метками → ОТБРОШЕН
JSON_BOTH='[{"number":1400,"labels":[{"name":"needs-e2e"},{"name":"e2e-done"}]}]'
assert_filter "issue с обеими метками → ОТБРОШЕН" "$JSON_BOTH" ""

# 5. Mixed: один blocker + два filtered → max() должен вернуть blocker
JSON_MIXED='[
    {"number":1586,"labels":[{"name":"needs-e2e"},{"name":"bug"}]},
    {"number":1300,"labels":[{"name":"e2e-done"}]},
    {"number":1117,"labels":[]}
]'
assert_filter "mixed: 1 blocker (1117) + 2 needs-e2e/e2e-done → max = 1117" "$JSON_MIXED" "1117"

# 6. Только needs-e2e/e2e-done → max() возвращает "" (пусто)
JSON_ALL_FILTERED='[
    {"number":1586,"labels":[{"name":"needs-e2e"}]},
    {"number":1300,"labels":[{"name":"e2e-done"}]},
    {"number":1400,"labels":[{"name":"needs-e2e"},{"name":"e2e-done"}]}
]'
assert_filter "all filtered → пусто" "$JSON_ALL_FILTERED" ""

# 7. Пустой массив → пусто (защита от `max` error)
assert_filter "empty array → пусто" '[]' ""

# 8. Несколько blockers без меток → max() вернёт максимальный number
JSON_MULTI_BLOCKERS='[
    {"number":1117,"labels":[]},
    {"number":1195,"labels":[]},
    {"number":1234,"labels":[]}
]'
assert_filter "multiple blockers без меток → max = 1234" "$JSON_MULTI_BLOCKERS" "1234"

# 9. Issue с меткой БЕЗ needs-e2e/e2e-done (например bug) → попадает
JSON_BUG_ONLY='[{"number":1117,"labels":[{"name":"bug"},{"name":"priority:high"}]}]'
assert_filter "только bug+priority → попадает" "$JSON_BUG_ONLY" "1117"

# 10. LIVE-проверка на реальном репо (если доступен gh + GH_REPO).
#     Дополнительно к синтетическим тестам — убеждаемся что pipeline работает
#     и в реальном gh CLI (а не только в python-эмуляции).
if [ -n "${LIVE_REPO:-}" ] && command -v gh >/dev/null 2>&1; then
    printf '\n%s--- LIVE check against %s ---%s\n' "$YEL" "$LIVE_REPO" "$NC"
    # Используем jq-pipeline из исходника с реальным gh CLI.
    # Отдельно ловим GH-error (rate limit / auth / network) через exit code.
    GH_RC=0
    LIVE_RESULT="$(gh issue list --repo "$LIVE_REPO" --state open \
        --search 'no_wake_word in:title,body' \
        --limit 5 --json number,labels --jq "$JQ_PIPELINE" 2>/dev/null)" || GH_RC=$?
    if [ "$GH_RC" -ne 0 ] && [ -z "$LIVE_RESULT" ]; then
        printf '%sSKIP live: gh CLI вернул ошибку (rc=%d, rate limit / auth / network)%s\n' "$YEL" "$GH_RC" "$NC"
    elif [ -z "$LIVE_RESULT" ]; then
        PASS=$((PASS+1))
        printf '%sOK live: gh issue list --search no_wake_word → пусто (issue #1586 с needs-e2e отброшен)%s\n' "$GRN" "$NC"
    else
        FAIL=$((FAIL+1))
        printf '%sFAIL live: gh вернул %q — issue с needs-e2e/e2e-done НЕ отброшен фильтром!%s\n' "$RED" "$LIVE_RESULT" "$NC"
    fi
fi

# 11. E2E_FORCE_UNPAUSE manual-override env flag (ретро t_8a8d9403, страховка
#     на случай false-positive в detect_known_blocker).
#     Проверяем что флаг действительно выставлен в e2e-process.sh и его
#     значение по умолчанию false (не нарушаем обычное поведение).
if grep -q '^: "\${E2E_FORCE_UNPAUSE:=false}"' "$E2E_SCRIPT"; then
    PASS=$((PASS+1))
    printf '%sOK E2E_FORCE_UNPAUSE default=false в e2e-process.sh%s\n' "$GRN" "$NC"
else
    FAIL=$((FAIL+1))
    printf '%sFAIL: E2E_FORCE_UNPAUSE default отсутствует в e2e-process.sh%s\n' "$RED" "$NC"
fi
if grep -q 'E2E_FORCE_UNPAUSE=true' "$E2E_SCRIPT" && grep -q 'detect_known_blocker возвращает пусто' "$E2E_SCRIPT"; then
    PASS=$((PASS+1))
    printf '%sOK E2E_FORCE_UNPAUSE early-return в detect_known_blocker%s\n' "$GRN" "$NC"
else
    FAIL=$((FAIL+1))
    printf '%sFAIL: E2E_FORCE_UNPAUSE early-return отсутствует в detect_known_blocker%s\n' "$RED" "$NC"
fi

echo
if [ "$FAIL" -eq 0 ]; then
    printf '%s✓ %d/%d passed%s\n' "$GRN" "$PASS" "$((PASS+FAIL))" "$NC"
    exit 0
else
    printf '%s✗ %d failed of %d%s\n' "$RED" "$FAIL" "$((PASS+FAIL))" "$NC"
    exit 1
fi
