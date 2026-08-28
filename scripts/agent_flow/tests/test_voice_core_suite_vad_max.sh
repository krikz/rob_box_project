#!/bin/bash
# ============================================================================
# test_voice_core_suite_vad_max.sh — issue #1506 e2e-revalidation
#
# Pre-flight sanity-check для .github/e2e/scenarios/voice_core_suite_v1.json
# (и парного voice_core_acceptance_v1.json). Запускается ЛОКАЛЬНО и в CI
# (как часть `make test-agents-flow`) перед любым e2e-раундом на роботе —
# чтобы поймать структурные ошибки и VAD-reject до того, как мы сожжём
# раунд на 249/21.
#
# Контекст (ADR-0022 voice e2e, см. .github/e2e/VOICE_COMMANDS_RESEARCH.md):
#   - audio_node на роботе имеет VAD max=12с. Команда ДЛИННЕЕ 12с отклоняется
#     ДО STT: «Речь отклонена: 12.76с (min=0.3, max=12.0)» → e2e падает с
#     no_accept (ложный FAIL).
#   - Yandex TTS (voices: anton/ermil/zahar/alena) для русского синтезирует
#     ~2.5–3 слова/сек. Pre-roll 1.5с (adelay). Итого при 10 словах:
#     ~10/2.7 + 1.5 = ~5.2с. При 19 словах: ~7.0 + 1.5 = ~8.5с — укладывается.
#   - Харнесс e2e_voice_test.sh синтезирует КАЖДЫЙ step отдельно (НЕ объединяет),
#     поэтому лимит применяется к каждому шагу, а не ко всему сценарию.
#
# Проверки:
#   1) voice_core_suite_v1.json — валидный JSON, name, steps[]
#   2) Каждый step имеет обязательные поля: label, text, voice
#   3) Каждый step имеет уникальный label (snake_case, ≤64 символов)
#   4) Длина text в словах ≤ WORDS_WARN (10 слов = ~5с синтез + 1.5с pre-roll,
#      запас до 12с VAD)
#   5) voice в списке разрешённых (anton/ermil/zahar/alena/jane)
#   6) expect ∈ {cycle, backlog} (по умолчанию cycle)
#   7) Backlog-шаги (expect=backlog) НЕ начинаются с wake-word (иначе
#      wake-gate не отбросит фразу → не накопится в backlog)
#   8) acceptance_file существует, валидный JSON, expected_tool_calls —
#      покрыты хотя бы одним step'ом scenario
#
# Этот тест НЕ запускает Yandex TTS (для скорости CI), а валидирует
# структуру + word-count (консервативная оценка длительности).
# Точный замер с Yandex делает e2e_voice_test.sh на 249 уже после merge.
#
# Run:
#   bash scripts/agent_flow/tests/test_voice_core_suite_vad_max.sh
# ============================================================================
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# tests/ → scripts/agent_flow/tests/ → scripts/agent_flow/ → scripts/ → REPO_ROOT
REPO_ROOT="$(cd "$TEST_DIR/../../.." && pwd)"
SUITE_FILE="$REPO_ROOT/.github/e2e/scenarios/voice_core_suite_v1.json"
ACCEPT_FILE="$REPO_ROOT/.github/e2e/scenarios/voice_core_acceptance_v1.json"

# --- Test config ----------------------------------------------------------
# Контекст (ADR-0022, см. .github/e2e/VOICE_COMMANDS_RESEARCH.md):
#   - audio_node VAD max=12.0с.
#   - Yandex TTS (anton/ermil/zahar/alena) для русского: ~2.7 слов/сек
#     (замерено на e2e 22.08 — ds01 19 слов ≈ 7с синтеза + 1.5с pre-roll = 8.5с).
#   - Харнесс e2e_voice_test.sh синтезирует КАЖДЫЙ step отдельно, лимит
#     применяется к одному шагу, не ко всему сценарию.
#   - ds01/ds02 backlog-шаги в текущем suite имеют 19 слов каждый —
#     укладываются в 12с, но на грани. WORDS_HARD_FAIL=20 оставляет запас.
WORDS_WARN=12                  # >12 слов — warning (но не блок)
WORDS_HARD_FAIL=20             # >20 слов — fail (Yandex + pre-roll >12с на медленных голосах)
ALLOWED_VOICES="anton ermil zahar alena jane"
LABEL_REGEX='^[a-zA-Z][a-zA-Z0-9_]{2,63}$'  # snake_case (с заглавными в camelCase-частях), 3-64 chars
WAKE_REGEX='^(Робот|Робокс)[ ,]'

# --- Helpers --------------------------------------------------------------
TESTS_TOTAL=0
TESTS_PASSED=0
TESTS_FAILED=0
FAILED_NAMES=()

if [ -t 1 ]; then
    RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; END=$'\033[0m'
else
    RED=''; GRN=''; YEL=''; END=''
fi

pass() { local n="$1"; TESTS_TOTAL=$((TESTS_TOTAL+1)); TESTS_PASSED=$((TESTS_PASSED+1)); printf '  %s✓%s %s\n' "$GRN" "$END" "$n"; }
fail() { local n="$1" r="$2"; TESTS_TOTAL=$((TESTS_TOTAL+1)); TESTS_FAILED=$((TESTS_FAILED+1)); FAILED_NAMES+=("$n"); printf '  %s✗%s %s — %s\n' "$RED" "$END" "$n" "$r"; }

count_words() {
    # Python reliable word counter для русского текста (awk не дружит с UTF-8).
    # Используем single-quoted heredoc-style через stdin, чтобы не мучаться с bash-escape.
    python3 - "$1" <<'PY'
import sys, re
text = sys.argv[1]
print(len(re.findall(r'\w+', text, flags=re.UNICODE)))
PY
}

# --- Проверка 1: suite file существует и валидный JSON -------------------
echo "1) voice_core_suite_v1.json — структурные проверки"
if [ ! -f "$SUITE_FILE" ]; then
    fail "suite_file_exists" "file not found: $SUITE_FILE"
    echo "ABORT: suite file missing"; exit 1
fi
pass "suite_file_exists ($SUITE_FILE)"

if ! python3 -c "import json,sys; json.load(open('$SUITE_FILE'))" 2>/dev/null; then
    fail "suite_valid_json" "invalid JSON in $SUITE_FILE"
    echo "ABORT: invalid JSON"; exit 1
fi
pass "suite_valid_json"

# Парсим в bash через python (надёжнее чем jq в edge-cases)
SUITE_DATA=$(python3 <<PY
import json, sys
with open("$SUITE_FILE") as f:
    s = json.load(f)
print(s.get("name", ""))
print(len(s.get("steps", [])))
for st in s.get("steps", []):
    label = st.get("label", "")
    text = st.get("text", "")
    voice = st.get("voice", "")
    expect = st.get("expect", "cycle")
    print(f"STEP\t{label}\t{voice}\t{expect}\t{text}")
PY
)
SUITE_NAME=$(printf '%s' "$SUITE_DATA" | sed -n '1p')
SUITE_STEP_COUNT=$(printf '%s' "$SUITE_DATA" | sed -n '2p')

if [ -z "$SUITE_NAME" ]; then
    fail "suite_name_present" "suite JSON has empty 'name' field"
else
    pass "suite_name_present ($SUITE_NAME)"
fi

if [ "$SUITE_STEP_COUNT" -lt 1 ] 2>/dev/null; then
    fail "suite_has_steps" "suite has $SUITE_STEP_COUNT steps (<1)"
    echo "ABORT: no steps"; exit 1
fi
pass "suite_has_steps ($SUITE_STEP_COUNT steps)"

# --- Проверка 2-7: каждый step ------------------------------------------
echo
echo "2) Каждый step — поля, label, voice, words, expect"
declare -A SEEN_LABELS
STEP_NUM=0

while IFS=$'\t' read -r tag label voice expect text; do
    [ "$tag" != "STEP" ] && continue
    STEP_NUM=$((STEP_NUM+1))

    # 2) обязательные поля (label/text/voice)
    if [ -z "$label" ] || [ -z "$text" ] || [ -z "$voice" ]; then
        fail "step_${STEP_NUM}_fields (label='$label' voice='$voice' text_len=${#text})" "missing required field"
        continue
    fi
    pass "step_${STEP_NUM}_fields ($label)"

    # 3) уникальный label + формат snake_case
    if [ -n "${SEEN_LABELS[$label]:-}" ]; then
        fail "step_${STEP_NUM}_unique_label ($label)" "duplicate label (first seen at ${SEEN_LABELS[$label]})"
    else
        SEEN_LABELS[$label]=$STEP_NUM
    fi
    if ! printf '%s' "$label" | grep -qE "$LABEL_REGEX"; then
        fail "step_${STEP_NUM}_label_format ($label)" "must match snake_case 3-64 chars"
    else
        pass "step_${STEP_NUM}_label_format ($label)"
    fi

    # 5) voice в whitelist
    voice_ok=0
    for v in $ALLOWED_VOICES; do
        if [ "$voice" = "$v" ]; then voice_ok=1; break; fi
    done
    if [ "$voice_ok" -ne 1 ]; then
        fail "step_${STEP_NUM}_voice_whitelist ($label: voice='$voice')" "voice not in [$ALLOWED_VOICES]"
    else
        pass "step_${STEP_NUM}_voice_whitelist ($label: $voice)"
    fi

    # 6) expect ∈ {cycle, backlog}
    if [ "$expect" != "cycle" ] && [ "$expect" != "backlog" ]; then
        fail "step_${STEP_NUM}_expect_valid ($label: expect='$expect')" "expect must be 'cycle' or 'backlog'"
    else
        pass "step_${STEP_NUM}_expect_valid ($label: $expect)"
    fi

    # 4) длина text в словах (VAD max=12с → max ~10 слов с запасом)
    word_count=$(count_words "$text")
    if [ "$word_count" -gt "$WORDS_HARD_FAIL" ]; then
        fail "step_${STEP_NUM}_vad_max ($label: $word_count words)" "exceeds $WORDS_HARD_FAIL words → risk >12s VAD reject"
    elif [ "$word_count" -gt "$WORDS_WARN" ]; then
        # Warn: среднее (warn level не блокирует, но печатаем)
        printf '  %s!%s step_%d_vad_warn (%s: %d words, target ≤%d)\n' \
            "$YEL" "$END" "$STEP_NUM" "$label" "$word_count" "$WORDS_WARN"
        TESTS_TOTAL=$((TESTS_TOTAL+1)); TESTS_PASSED=$((TESTS_PASSED+1))
    else
        pass "step_${STEP_NUM}_vad_max ($label: $word_count words)"
    fi

    # 7) backlog-шаги НЕ должны начинаться с wake-word
    if [ "$expect" = "backlog" ]; then
        if printf '%s' "$text" | grep -qE "$WAKE_REGEX"; then
            fail "step_${STEP_NUM}_backlog_no_wake ($label)" "backlog step starts with wake-word → won't accumulate"
        else
            pass "step_${STEP_NUM}_backlog_no_wake ($label)"
        fi
    fi

    # 8) cycle-шаги ДОЛЖНЫ начинаться с wake-word (иначе wake-gate отбросит)
    if [ "$expect" = "cycle" ]; then
        if ! printf '%s' "$text" | grep -qE "$WAKE_REGEX"; then
            fail "step_${STEP_NUM}_cycle_has_wake ($label: text='${text:0:30}...')" "cycle step lacks wake-word → no_accept"
        else
            pass "step_${STEP_NUM}_cycle_has_wake ($label)"
        fi
    fi

done <<< "$SUITE_DATA"

if [ "$STEP_NUM" -ne "$SUITE_STEP_COUNT" ]; then
    fail "step_count_consistency" "expected $SUITE_STEP_COUNT steps, parsed $STEP_NUM"
fi

# --- Проверка 8: acceptance file -----------------------------------------
echo
echo "3) voice_core_acceptance_v1.json — acceptance cross-check"
if [ ! -f "$ACCEPT_FILE" ]; then
    fail "acceptance_file_exists" "file not found: $ACCEPT_FILE"
else
    pass "acceptance_file_exists ($ACCEPT_FILE)"
    if ! python3 -c "import json,sys; json.load(open('$ACCEPT_FILE'))" 2>/dev/null; then
        fail "acceptance_valid_json" "invalid JSON in $ACCEPT_FILE"
    else
        pass "acceptance_valid_json"
    fi

    # Cross-check: каждый expected_tool_call из acceptance покрыт хотя бы одним step'ом
    EXPECTED_TOOLS=$(python3 <<PY
import json
with open("$ACCEPT_FILE") as f:
    a = json.load(f)
print("\n".join(a.get("expected_tool_calls", [])))
PY
)
    if [ -z "$EXPECTED_TOOLS" ]; then
        printf '  %s!%s acceptance_has_no_expected_tool_calls — это OK для backlog-only suite\n' "$YEL" "$END"
    else
        while IFS= read -r tool; do
            [ -z "$tool" ] && continue
            found=$(python3 <<PY
import json
with open("$SUITE_FILE") as f: s = json.load(f)
labels = []
for st in s.get("steps", []):
    if "acceptance" in st and "$tool" in st["acceptance"].get("expected_tool_calls", []):
        labels.append(st["label"])
print(",".join(labels) if labels else "NONE")
PY
)
            if [ "$found" = "NONE" ] || [ -z "$found" ]; then
                fail "tool_$tool" "no step covers expected_tool_call '$tool'"
            else
                pass "tool_$tool (covered by: $found)"
            fi
        done <<< "$EXPECTED_TOOLS"
    fi
fi

# --- Summary -------------------------------------------------------------
echo
echo "────────────────────────────────────────"
TOTAL=$((TESTS_PASSED + TESTS_FAILED))
if [ "$TESTS_FAILED" -eq 0 ]; then
    printf '%s✅%s Voice core suite pre-flight: %d/%d checks PASSED\n' "$GRN" "$END" "$TESTS_PASSED" "$TOTAL"
    exit 0
else
    printf '%s❌%s Voice core suite pre-flight: %d/%d checks FAILED (%d passed)\n' \
        "$RED" "$END" "$TESTS_FAILED" "$TOTAL" "$TESTS_PASSED"
    printf '  Failed:\n'
    for n in "${FAILED_NAMES[@]}"; do printf '    - %s\n' "$n"; done
    exit 1
fi
