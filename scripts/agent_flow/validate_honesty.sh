#!/usr/bin/env bash
# ============================================================================
# validate_honesty.sh — pre-PR / pre-merge check на «голословный PASS».
#
# Принцип (ADR-0018, 18.08.2026, наказ товарища Шифу):
#   «Честный FAIL лучше красивого PASS» + raw-вывод обязателен.
#
# Что делает:
#   1. Читает PR body (через `gh pr view N --json body` / файл / stdin).
#   2. Ищет «голословные маркеры» по КЛАССАМ серьёзности:
#      - **critical** — `closes #N`/`resolves #N`/`[x]` checkbox без evidence
#        (= «закрываю issue» или «отмечаю чек-бокс» без доказательств).
#        → soft-fail: exit !=0 (merge-gate логирует WARN, не блокирует).
#      - **major**    — общие claim-маркеры без evidence (`проверил`,
#        `работает`, `PASS`, `✅`, `done`, `fixed`, …).
#        → warning в stderr, exit 0.
#      - **minor**    — мягкие hedge-формулировки (`вроде работает`).
#        → silent (только информативный debug в --verbose).
#   3. Если в тексте ЕСТЬ хотя бы один evidence-маркер (run_id / pytest /
#      docker logs / sqlite / git log / вывод / лог / блок кода) — ВСЕ
#      классы оправданы (clean).
#   4. По умолчанию exit 0; critical без evidence → exit !=0 (soft-fail).
#      --strict → exit !=0 при ЛЮБОМ warn (major и critical).
#
# Использование:
#   bash validate_honesty.sh --file <path-to-pr-body>
#   bash validate_honesty.sh --pr <N>            # через gh pr view
#   cat pr.md | bash validate_honesty.sh         # stdin
#   bash validate_honesty.sh --strict --file ... # exit !=0 на любой warn
#   bash validate_honesty.sh --verbose           # печатать и minor hits
#
# Регистрация:
#   - EXPECTED в scripts/agent_flow/install.sh (drift-detect контролирует).
#   - hint-вызов из agent-flow-merge-gate.sh (pre-merge, non-blocking;
#     exit !=0 на critical → лог-WARN, не блокер merge).
#
# Тест:
#   bash scripts/agent_flow/tests/test_validate_honesty.sh
# ============================================================================

set -uo pipefail

# ---- CLI args ----
INPUT_FILE=""
PR_NUMBER=""
STRICT=0
VERBOSE=0
while [ $# -gt 0 ]; do
    case "$1" in
        --file)    INPUT_FILE="${2:-}"; shift 2 ;;
        --pr)      PR_NUMBER="${2:-}"; shift 2 ;;
        --strict)  STRICT=1; shift ;;
        --verbose) VERBOSE=1; shift ;;
        -h|--help)
            sed -n '2,33p' "$0" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        *) echo "validate_honesty: unknown arg: $1" >&2; exit 2 ;;
    esac
done

# ---- Получить текст ----
TEXT=""
if [ -n "$INPUT_FILE" ]; then
    if [ ! -r "$INPUT_FILE" ]; then
        echo "validate_honesty: cannot read file: $INPUT_FILE" >&2
        exit 2
    fi
TEXT="$(cat "$INPUT_FILE")"
elif [ -n "$PR_NUMBER" ]; then
    if ! command -v gh >/dev/null 2>&1; then
        echo "validate_honesty: gh not on PATH (cannot fetch PR #$PR_NUMBER)" >&2
        exit 2
    fi
    if ! TEXT="$(gh pr view "$PR_NUMBER" --json body --jq '.body' 2>/dev/null)"; then
        echo "validate_honesty: gh pr view failed for #$PR_NUMBER" >&2
        exit 2
    fi
elif [ ! -t 0 ]; then
    TEXT="$(cat)"
else
    echo "validate_honesty: provide --file, --pr, or pipe text via stdin" >&2
    exit 2
fi

# ---- Маркеры по классам (ADR-0018 + issue #1465 §6.2) ----
#
# CRITICAL (Class 1 — soft-fail, exit !=0):
#   «закрытие» или «чек-бокс» без raw-evidence — это самая опасная форма
#   голословного PASS: воркер утверждает, что issue закрыт / задача выполнена,
#   но не привёл pytest / docker logs / gh run view / git log.
#   По ADD research §6.2 — это именно тот класс, который вернётся, если
#   downgrade'нуть культуру честности. Поэтому → soft-fail (не блокер,
#   но merge-gate УВИДИТ не-нулевой exit и залогирует WARN).
CRITICAL_PATTERNS=(
    'closes #'        # "closes #N" без raw-evidence — закрывает issue без доказательства
    'resolves #'      # то же самое (GitHub autolink)
    'fixes #'         # третий вариант autolink-маркера
    '\[x\]'           # checked checkbox без описания рядом (loosely)
)

# MAJOR (Class 2 — warning, exit 0):
#   Голословные маркеры готовности / проверки / зелёного статуса — без
#   подтверждающего raw-evidence. Сами по себе не «закрывают» задачу, но
#   сигнализируют ревьюеру: «попроси pytest -v / gh run view / docker logs».
MAJOR_PATTERNS=(
    'проверил[аио]?'
    'провер[её]н[оа]?'
    'работает'
    'готово'
    'вс[ёе] ок'
    '✅'
    'PASS'
    'passed'
    'done'
    'fixed'
    'all good'
    'it works'
    'should work'
    'looks good'
    'green'
    'зел[ёе]н[аыо]?й'
    'зел[ёе]н[аыо]?ые'
)

# MINOR (Class 3 — silent):
#   Мягкие hedge-формулировки, в которых воркер САМ признаёт неуверенность.
#   По дизайну — silent: дополнительный шум не помогает ревьюеру, а только
#   размывает сигнал. Печатаются ТОЛЬКО при --verbose (debug).
MINOR_PATTERNS=(
    'вроде работает'
    'вроде ок'
    'кажется работает'
    'наверное работает'
    'похоже работает'
)

# Evidence-маркеры: если хоть один есть в тексте, голословные оправданы.
EVIDENCE_PATTERNS=(
    'gh run view'
    'gh pr view'
    'pytest'
    'docker logs'
    'sqlite3'
    'git log'
    'run_id:'
    'run-id:'
    'вывод'
    'лог[аи]?'
    'дамп'
    '`.*`'           # backtick — инлайн-код
    '```'            # code-fence
    'console.log'
    'ros2 topic'
    'tail -f'
)

# ---- Поиск ----
critical_count=0
major_count=0
minor_count=0

# Нижний регистр — для case-insensitive поиска.
TEXT_LC="$(printf '%s' "$TEXT" | tr '[:upper:]' '[:lower:]')"

# Есть ли хоть одно evidence-слово?
has_evidence=0
for ev in "${EVIDENCE_PATTERNS[@]}"; do
    if printf '%s' "$TEXT_LC" | grep -qiE -- "$ev"; then
        has_evidence=1
        break
    fi
done

# Если evidence есть — чисто по всем классам.
if [ "$has_evidence" -eq 1 ]; then
    echo "validate_honesty: clean (evidence present, claims justified)."
    exit 0
fi

# Без evidence — каждый паттерн в своём классе.
check_class() {  # $1=class_label  $2=pattern  → echo "<label> hit" / пусто
    local label="$1"
    local pat="$2"
    local hits
    hits="$(printf '%s' "$TEXT_LC" | grep -ciE -- "$pat" || true)"
    if [ "${hits:-0}" -gt 0 ]; then
        printf '%s: pattern %q found %dx without raw-evidence\n' \
            "$label" "$pat" "$hits" >&2
        return 0
    fi
    return 1
}

for pat in "${CRITICAL_PATTERNS[@]}"; do
    if check_class "CRITICAL" "$pat"; then
        critical_count=$((critical_count + 1))
    fi
done

for pat in "${MAJOR_PATTERNS[@]}"; do
    if check_class "WARN" "$pat"; then
        major_count=$((major_count + 1))
    fi
done

for pat in "${MINOR_PATTERNS[@]}"; do
    if [ "$VERBOSE" -eq 1 ] && check_class "info" "$pat"; then
        minor_count=$((minor_count + 1))
    fi
done

# ---- Итог ----
if [ "$critical_count" -gt 0 ]; then
    echo "validate_honesty: $critical_count CRITICAL (closes/[x] без evidence) + $major_count WARN. Soft-fail: review must demand pytest -v / gh run view / docker logs." >&2
    # Critical → soft-fail (exit !=0). merge-gate видит не-нулевой код и логирует WARN,
    # но НЕ блокирует merge (по ADR-0018 — ревьюер решает).
    exit 1
fi

if [ "$major_count" -gt 0 ]; then
    echo "validate_honesty: $major_count warning(s). PR has claims without raw-evidence. Reviewer: ask worker for pytest -v / gh run view / docker logs." >&2
    if [ "$STRICT" -eq 1 ]; then
        echo "validate_honesty: --strict → exit 1" >&2
        exit 1
    fi
else
    echo "validate_honesty: clean (no critical/major claims without evidence)."
fi

exit 0
