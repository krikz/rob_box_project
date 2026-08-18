#!/usr/bin/env bash
# ============================================================================
# validate_honesty.sh — pre-PR / pre-merge check на «голословный PASS».
#
# Принцип (ADR-0018, 18.08.2026, наказ товарища Шифу):
#   «Честный FAIL лучше красивого PASS» + raw-вывод обязателен.
#
# Что делает:
#   1. Читает PR body (через `gh pr view N --json body` / файл / stdin).
#   2. Ищет «голословные маркеры»: «проверил», «работает», «PASS», «✅»,
#      «done», «fixed», «готов», «all good», «it works» и т.п.
#   3. Если маркер найден И в тексте НЕТ хотя бы одного «evidence-маркера»
#      (run_id / pytest / docker logs / sqlite / git log / вывод / лог / блок
#      кода) — печатает WARN.
#   4. ВСЕГДА exit 0 (по дизайну — не блокер; ревьюер сам решит).
#      --strict → exit 1 при наличии warning (для CI в будущем).
#
# Использование:
#   bash validate_honesty.sh --file <path-to-pr-body>
#   bash validate_honesty.sh --pr <N>            # через gh pr view
#   cat pr.md | bash validate_honesty.sh         # stdin
#   bash validate_honesty.sh --strict --file ... # exit 1 на WARN
#
# Регистрация:
#   - EXPECTED в scripts/agent_flow/install.sh (drift-detect контролирует).
#   - hint-вызов из agent-flow-merge-gate.sh (pre-merge, non-blocking).
#
# Тест:
#   bash scripts/agent_flow/tests/test_validate_honesty.sh
# ============================================================================

set -uo pipefail

# ---- CLI args ----
INPUT_FILE=""
PR_NUMBER=""
STRICT=0
while [ $# -gt 0 ]; do
    case "$1" in
        --file)    INPUT_FILE="${2:-}"; shift 2 ;;
        --pr)      PR_NUMBER="${2:-}"; shift 2 ;;
        --strict)  STRICT=1; shift ;;
        -h|--help)
            sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'
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

# ---- Маркеры ----
# Голословные: то, что воркер пишет, когда говорит «готово» без доказательств.
# Регистр игнорируется (grep -i).
# NB: \b границы слов где возможно; русские слова — без \b (PCRE нет).
CLAIM_PATTERNS=(
    'проверил[аио]?'
    'провер[её]н[оа]?'
    'работает'
    'готово'
    'вс[ёе] ок'
    'вроде работает'
    '✅'
    '\[x\]'           # checked checkbox без описания (loosely)
    'PASS'
    'passed'
    'done'
    'fixed'
    'closes #'        # "closes #N" без raw-evidence — подозрительно
    'resolves #'
    'all good'
    'it works'
    'should work'
    'looks good'
    'green'
    'зел[ёе]н[аыо]?й'
    'зел[ёе]н[аыо]?ые'
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
warn_count=0

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

# Если evidence нет — каждый голословный маркер = WARN.
if [ "$has_evidence" -eq 0 ]; then
    for pat in "${CLAIM_PATTERNS[@]}"; do
        # grep -iE; -- чтобы паттерн не начинался с '-'
        hits="$(printf '%s' "$TEXT_LC" | grep -ciE -- "$pat" || true)"
        if [ "${hits:-0}" -gt 0 ]; then
            # Показываем оригинальный паттерн (для лога).
            echo "WARN: claim pattern '$pat' found ${hits}x with NO raw-evidence (pytest/docker logs/gh run view/git log/sqlite/код-блок)." >&2
            warn_count=$((warn_count + 1))
        fi
    done
fi

# Итог (на stdout — чтобы CI мог grep'ать; warn'ы выше идут в stderr).
if [ "$warn_count" -gt 0 ]; then
    echo "validate_honesty: $warn_count warning(s). PR has claims without raw-evidence. Reviewer: ask worker for pytest -v / gh run view / docker logs." >&2
    if [ "$STRICT" -eq 1 ]; then
        echo "validate_honesty: --strict → exit 1" >&2
        exit 1
    fi
else
    echo "validate_honesty: clean (no claims without evidence)."
fi

exit 0
