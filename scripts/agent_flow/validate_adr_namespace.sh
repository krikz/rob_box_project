#!/usr/bin/env bash
# validate_adr_namespace.sh — pre-PR check на ADR namespace collision.
#
# Принцип (ADR-AF-0030, issue #2076):
#   ADR-номера делятся на ДВА НЕЗАВИСИМЫХ домена:
#     - ADR-NNNN (RT, рантайм робота): голос, Quest, perception, supervisor, harness runtime.
#     - ADR-AF-NNNN (AF, agent-flow): процесс, воркеры, AI-харнес агентов.
#   Внутри каждого домена номер уникален (origin/develop).
#   Коллизия считается ТОЛЬКО внутри одного домена: AF-0052 vs AF-0052
#   падает, AF-0052 vs 0052 (RT) — не падает (разные домены).
#
#   Схема принята владельцем 07.09.2026 в issue #2076 (после 13 коллизий
#   в develop). Реализация Phase 1 — гард в merge-gate (PR #2073). Phase 2 —
#   раскладка 13 коллизий + переименование доменов — issue #2076 (этот PR).
#
# Использование:
#   bash scripts/agent_flow/validate_adr_namespace.sh                # default: origin/develop..HEAD
#   bash scripts/agent_flow/validate_adr_namespace.sh --ref main    # другой baseline
#   bash scripts/agent_flow/validate_adr_namespace.sh --ref <sha>    # абсолютный коммит
#   bash scripts/agent_flow/validate_adr_namespace.sh --strict       # exit 1 на любой warn
#
# Регистрация:
#   - EXPECTED в scripts/agent_flow/install.sh (drift-detect контролирует).
#   - упоминание в AGENTS.md / scripts/agent_flow/README.md как pre-PR gate.
#   - НЕ вызывается из merge-gate (там своя полная реализация с override).
#
# Тест:
#   bash scripts/agent_flow/tests/test_validate_adr_namespace.sh
#
# Exit codes:
#   0 — нет коллизии (clean) или нет новых ADR-файлов в diff.
#   1 — ADR namespace collision: новый файл в AF или в RT с занятым в своём домене номером.
#   2 — usage error (неизвестный флаг, отсутствует git, baseline не достижим).
# ============================================================================

set -euo pipefail

# ---- CLI args ----
REF="origin/develop"
STRICT=0
while [ $# -gt 0 ]; do
    case "$1" in
        --ref)    REF="${2:-}"; [ -n "$REF" ] || { echo "validate_adr_namespace: --ref требует аргумент" >&2; exit 2; }; shift 2 ;;
        --strict) STRICT=1; shift ;;
        -h|--help)
            sed -n '2,38p' "$0" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        *) echo "validate_adr_namespace: unknown arg: $1" >&2; exit 2 ;;
    esac
done

# ---- Предусловия ----
command -v git >/dev/null 2>&1 || { echo "validate_adr_namespace: git не найден в PATH" >&2; exit 2; }

# Baseline должен быть достижим. Для origin/develop этого достаточно
# `git rev-parse --verify refs/remotes/origin/develop`; для произвольного
# SHA/ветки/тега — то же. Если нет — fail-soft с подсказкой (fetch).
if ! git rev-parse --verify "$REF" >/dev/null 2>&1; then
    echo "validate_adr_namespace: baseline '$REF' не достижим." >&2
    echo "  Подсказка: git fetch origin $REF" >&2
    exit 2
fi

# ---- Извлечь номера ADR из новых файлов (diff baseline..HEAD, --diff-filter=A) ----
#
# Логика:
#   1. `git diff <REF>...HEAD --name-only --diff-filter=A` → ТОЛЬКО added файлы
#      (rename --diff-filter=R даёт OLD name; правка существующего --diff-filter=M
#      не считается «новым» — collision guard'у не интересно, что воркер
#      поправил существующий ADR-файл).
#   2. grep '^docs/adr/(AF-)?[0-9]+-' фильтрует ADR-файлы в ОБОИХ доменах.
#   3. Префикс домена: AF- → домен AF; иначе → домен RT.
#   4. sed извлекает 4-значный zero-padded NNNN (нормализуем без ведущих нулей
#      для совместимости с merge-gate).
# NB: `|| true` в конце pipeline ОБЯЗАТЕЛЬНО: при пустом вводе grep закрывает
# pipe до того, как sort успевает прочитать, → sort получает SIGPIPE (141) →
# pipefail видит ненулевой код → set -e падает.
#
# Формат ключа: "<DOMAIN>:<num>" (например "RT:16" или "AF:0058"). Это даёт
# нам раздельные ключи для каждого домена — comm -12 найдёт коллизию только
# если оба ключа находятся в ОБОИХ множествах.
extract_keys() {  # stdin: list of paths (docs/adr/NNNN-*.md or docs/adr/AF-NNNN-*.md)
    # $1 = input list. Output format: "<DOMAIN>:<NNN>"
    #
    # File layout:
    #   docs/adr/AF-NNNN-<slug>.md  → domain AF, NNNN at chars 13..16 (1-indexed)
    #   docs/adr/NNNN-<slug>.md     → domain RT, NNNN at chars 10..13
    #   (verified: 'docs/adr/' = 9 chars, then either 'AF-' or digit)
    awk '
        /^docs\/adr\/AF-[0-9]+-[a-zA-Z0-9_-]+\.md$/ { print "AF:" substr($0, 13, 4)+0; next }
        /^docs\/adr\/[0-9]+-[a-zA-Z0-9_-]+\.md$/   { print "RT:" substr($0, 10, 4)+0; next }
    '
}

NEW_KEYS=""
NEW_KEYS="$(git diff "$REF"...HEAD --name-only --diff-filter=A 2>/dev/null | extract_keys | sort -u || true)"

if [ -z "$NEW_KEYS" ]; then
    echo "validate_adr_namespace: clean (нет новых ADR-файлов в $REF...HEAD)."
    exit 0
fi

# ---- Извлечь номера ADR из baseline ----
EXISTING_KEYS=""
EXISTING_KEYS="$(git ls-tree -r "$REF" --name-only 2>/dev/null | extract_keys | sort -u || true)"

if [ -z "$EXISTING_KEYS" ]; then
    echo "validate_adr_namespace: clean (baseline $REF не содержит ADR-файлов; занятых номеров нет)."
    exit 0
fi

# ---- Номера, занятые ОТКРЫТЫМИ PR ----
#
# Гард сверялся только с origin/develop и не видел PR в полёте: один номер
# спокойно берут два одновременных PR, оба проходят проверку, коллизия
# появляется после второго мержа. Так на develop оказались ДВА ADR-0080
# (PR #2185 08.09 и PR #2221 09.09) и едва не оказались два ADR-0083
# (PR #2247 и #2248, созданы с разницей в минуту).
#
# Требует gh с токеном: в Actions есть GITHUB_TOKEN, локально без
# авторизации — печатаем явное сообщение, а не пропускаем молча.
if command -v gh >/dev/null 2>&1 && gh auth status >/dev/null 2>&1; then
    INFLIGHT_RAW="$(gh pr list --state open --limit 100 --json number,files \
        --jq '.[] | .number as $n | .files[].path | select(startswith("docs/adr/")) | "\($n)\t\(.)"' \
        2>/dev/null || true)"
    SELF_PATHS="$(git diff "$REF"...HEAD --name-only --diff-filter=A 2>/dev/null || true)"
    INFLIGHT_PATHS=""
    while IFS="$(printf '\t')" read -r _pr _path; do
        [ -n "$_path" ] || continue
        # свои файлы не считаем — PR не конфликтует сам с собой
        printf '%s\n' "$SELF_PATHS" | grep -qxF "$_path" && continue
        # только ДОБАВЛЯЕМЫЕ: если путь уже в baseline, PR его правит
        if git cat-file -e "$REF:$_path" 2>/dev/null; then continue; fi
        INFLIGHT_PATHS="$INFLIGHT_PATHS$_pr	$_path
"
    done <<EOF
$INFLIGHT_RAW
EOF
    INFLIGHT_KEYS="$(printf '%s\n' "$INFLIGHT_PATHS" | cut -f2 | extract_keys | sort -u || true)"
    if [ -n "$INFLIGHT_KEYS" ]; then
        INFLIGHT_COLLISION="$(comm -12 <(printf '%s\n' "$NEW_KEYS") <(printf '%s\n' "$INFLIGHT_KEYS") || true)"
        if [ -n "$INFLIGHT_COLLISION" ]; then
            echo "validate_adr_namespace: FAIL - номер занят ОТКРЫТЫМ PR (не только develop):"
            printf '%s\n' "$INFLIGHT_COLLISION" | sed 's/^/  /'
            echo ""
            echo "  Новые ADR-файлы в открытых PR:"
            printf '%s' "$INFLIGHT_PATHS" | sed 's/^/    PR #/'
            echo ""
            echo "  Возьмите следующий свободный номер С УЧЁТОМ PR в полёте."
            exit 1
        fi
    fi
else
    echo "validate_adr_namespace: проверка по открытым PR ПРОПУЩЕНА (нет gh или авторизации)."
fi

# ---- Пересечение множеств (внутри каждого домена отдельно) ----
COLLISION="$(comm -12 <(printf '%s\n' "$NEW_KEYS") <(printf '%s\n' "$EXISTING_KEYS") || true)"

if [ -z "$COLLISION" ]; then
    NEW_HUMAN="$(printf '%s\n' "$NEW_KEYS" | sed 's/^/  /' | tr '\n' ' ')"
    echo "validate_adr_namespace: clean (новые ADR: $NEW_HUMAN; все номера свободны в своём домене $REF)."
    exit 0
fi

# ---- Коллизия: красивый отчёт ----
#
# Печатаем:
#   - список ЗАНЯТЫХ ключей (с разбивкой по домену и slug из baseline).
#   - next free slot = max(existing) + 1 в каждом затронутом домене.
#   - явный actionable hint.

declare -a COLLISION_LINES=()
declare -a AF_NEXT_FREE=()
declare -a RT_NEXT_FREE=()
# Считаем max в каждом домене отдельно
AF_MAX_NUM="$(printf '%s\n' "$EXISTING_KEYS" | awk -F: '/^AF:/ {print $2}' | sort -n | tail -n1)"
RT_MAX_NUM="$(printf '%s\n' "$EXISTING_KEYS" | awk -F: '/^RT:/ {print $2}' | sort -n | tail -n1)"
[ -z "$AF_MAX_NUM" ] && AF_MAX_NUM=0
[ -z "$RT_MAX_NUM" ] && RT_MAX_NUM=0
AF_NEXT=$((AF_MAX_NUM + 1))
RT_NEXT=$((RT_MAX_NUM + 1))
AF_NEXT_PADDED="$(printf '%04d' "$AF_NEXT" 2>/dev/null || echo "$AF_NEXT")"
RT_NEXT_PADDED="$(printf '%04d' "$RT_NEXT" 2>/dev/null || echo "$RT_NEXT")"

while IFS= read -r key; do
    [ -z "$key" ] && continue
    domain="${key%%:*}"
    num="${key#*:}"
    # Файлы в baseline с этим ключом.
    if [ "$domain" = "AF" ]; then
        base_slugs="$(git ls-tree -r "$REF" --name-only 2>/dev/null \
            | grep -E "^docs/adr/AF-0*${num}-.*\.md$" \
            | sed -E 's|^docs/adr/AF-0*[0-9]+-||; s|\.md$||' \
            | paste -sd ', ' -)"
        new_slugs="$(git diff "$REF"...HEAD --name-only --diff-filter=A 2>/dev/null \
            | grep -E "^docs/adr/AF-0*${num}-.*\.md$" \
            | sed -E 's|^docs/adr/AF-0*[0-9]+-||; s|\.md$||' \
            | paste -sd ', ' -)"
    else
        base_slugs="$(git ls-tree -r "$REF" --name-only 2>/dev/null \
            | grep -E "^docs/adr/0*${num}-.*\.md$" \
            | sed -E 's|^docs/adr/0*[0-9]+-||; s|\.md$||' \
            | paste -sd ', ' -)"
        new_slugs="$(git diff "$REF"...HEAD --name-only --diff-filter=A 2>/dev/null \
            | grep -E "^docs/adr/0*${num}-.*\.md$" \
            | sed -E 's|^docs/adr/0*[0-9]+-||; s|\.md$||' \
            | paste -sd ', ' -)"
    fi
    {
        printf '  %s (занято в %s: %s; новый в PR: %s)\n' "$key" "$REF" "${base_slugs:-<нет>}" "${new_slugs:-<нет>}"
    } >&2
    COLLISION_LINES+=("$key")
done <<< "$COLLISION"

{
    echo ""
    echo "ERROR: ADR namespace collision detected (внутри одного домена)."
    echo "  Новый ADR-файл(ы) в PR используют домен+номер, уже занятые в $REF:"
    printf '  %s\n' "${COLLISION_LINES[@]}"
    echo ""
    # next free для каждого затронутого домена
    touched_domains="$(printf '%s\n' "${COLLISION_LINES[@]}" | awk -F: '{print $1}' | sort -u)"
    for d in $touched_domains; do
        if [ "$d" = "AF" ]; then
            echo "  Next free в AF-домене: $AF_NEXT_PADDED"
        else
            echo "  Next free в RT-домене: $RT_NEXT_PADDED"
        fi
    done
    echo ""
    echo "  Что делать:"
    echo "    1. Переименуй файл(ы) в collisions → <домен>-<свободный-номер>-<slug>.md"
    echo "       AF-домен: docs/adr/AF-${AF_NEXT_PADDED}-<slug>.md"
    echo "       RT-домен: docs/adr/${RT_NEXT_PADDED}-<slug>.md"
    echo "    2. Поправь внутренний H1 «# ADR-<домен>-<NNN>: ...» — он обязан совпадать с именем файла."
    echo "    3. Перепрогон: bash scripts/agent_flow/validate_adr_namespace.sh"
    echo ""
    echo "  Напоминание про схему:"
    echo "    ADR-AF-NNNN — agent-flow / процесс / AI-харнес (ADR-AF-0030 §2.1)."
    echo "    ADR-NNNN    — рантайм робота: голос, Quest, perception, supervisor."
    echo "    ADR-AF-NNNN vs ADR-NNNN — РАЗНЫЕ домены, коллизия не считается."
} >&2

exit 1