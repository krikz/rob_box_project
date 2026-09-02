#!/usr/bin/env bash
# ============================================================================
# validate_adr_namespace.sh — pre-PR check на ADR namespace collision.
#
# Принцип (ADR-0030, ретро 01.09 t_debcb647):
#   ADR-номера (NNNN в `docs/adr/NNNN-*.md`) — global monotonic counter,
#   unique в origin/develop. Любой новый файл с занятым номером ломает
#   cross-references и ADR-First поиск. Merge-gate уже ловит коллизию
#   post-factum (scripts/agent_flow/agent-flow-merge-gate.sh →
#   check_adr_number_collision); этот скрипт — pre-PR версия для локального
#   запуска агентом ДО `gh pr create`, чтобы:
#     - не открывать PR, который merge-gate сразу же отвергнет,
#     - показать воркеру свободный слот и список занятых номеров
#       (чтобы переименовать 0040-collide.md → 0043-fresh.md без раунда
#       с архитектором).
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
#   1 — ADR namespace collision: новый файл с NNNN, который уже занят в baseline.
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
            sed -n '2,32p' "$0" | sed 's/^# \{0,1\}//'
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

HEAD_SHA="$(git rev-parse --verify HEAD)"

# ---- Извлечь номера ADR из новых файлов (diff baseline..HEAD, --diff-filter=A) ----
#
# Логика:
#   1. `git diff <REF>...HEAD --name-only --diff-filter=A` → ТОЛЬКО added файлы
#      (rename --diff-filter=R даёт OLD name; правка существующего --diff-filter=M
#      не считается «новым» — collision guard'у не интересно, что воркер
#      поправил существующий ADR-файл).
#   2. grep '^docs/adr/[0-9]+-' фильтрует ADR-файлы.
#   3. sed -E извлекает 4-значный zero-padded NNNN (без ведущих нулей
#      в выводе — сохраняем как есть, чтобы совпадать с тем, что
#      показывает `git ls-tree`).
# NB: `|| true` в конце pipeline ОБЯЗАТЕЛЬНО: при пустом вводе grep закрывает
# pipe до того, как sort успевает прочитать, → sort получает SIGPIPE (141) →
# pipefail видит ненулевой код → set -e падает. Без || true скрипт просто
# молча выходит с exit 1 даже когда коллизии нет.
NEW_ADRS=""
NEW_ADRS="$(git diff "$REF"...HEAD --name-only --diff-filter=A 2>/dev/null \
    | grep -E '^docs/adr/[0-9]+-[a-zA-Z0-9_-]+\.md$' \
    | sed -E 's|^docs/adr/0*([0-9]+)-.*\.md$|\1|' \
    | sort -u || true)"

if [ -z "$NEW_ADRS" ]; then
    echo "validate_adr_namespace: clean (нет новых ADR-файлов в $REF...HEAD)."
    exit 0
fi

# ---- Извлечь номера ADR из baseline ----
#
# `git ls-tree -r <REF> --name-only` рекурсивно перечисляет все файлы
# baseline. grep фильтрует ADR. sed нормализует номер к той же форме
# (без ведущих нулей), что и для NEW_ADRS — иначе сравнение 0040 vs 40
# может дать ложный PASS.
EXISTING_ADRS=""
EXISTING_ADRS="$(git ls-tree -r "$REF" --name-only 2>/dev/null \
    | grep -E '^docs/adr/[0-9]+-[a-zA-Z0-9_-]+\.md$' \
    | sed -E 's|^docs/adr/0*([0-9]+)-.*\.md$|\1|' \
    | sort -u || true)"

if [ -z "$EXISTING_ADRS" ]; then
    # Baseline существует, но в нём нет ADR-файлов — collision невозможна.
    echo "validate_adr_namespace: clean (baseline $REF не содержит ADR-файлов; занятых номеров нет)."
    exit 0
fi

# ---- Пересечение множеств ----
COLLISION="$(comm -12 <(printf '%s\n' "$NEW_ADRS") <(printf '%s\n' "$EXISTING_ADRS") || true)"

if [ -z "$COLLISION" ]; then
    echo "validate_adr_namespace: clean (новые ADR: $(printf '%s ' $NEW_ADRS | sed 's/ $//'); все номера свободны в $REF)."
    exit 0
fi

# ---- Коллизия: красивый отчёт ----
#
# Печатаем:
#   - список ЗАНЯТЫХ номеров (с slug из baseline, чтобы воркер видел, кто
#     сидит под этим номером — обычно достаточно переименовать свой файл).
#   - next free slot = max(existing) + 1.
#   - явный actionable hint.

# Slug для каждого collision-номера (для human-readable вывода).
declare -a COLLISION_LINES=()
while IFS= read -r num; do
    [ -z "$num" ] && continue
    # Файлы в baseline с этим номером.
    base_slugs="$(git ls-tree -r "$REF" --name-only 2>/dev/null \
        | grep -E "^docs/adr/0*${num}-.*\.md$" \
        | sed -E 's|^docs/adr/0*[0-9]+-||; s|\.md$||' \
        | paste -sd ', ' -)"
    # Файлы в PR с этим номером (на случай, если collision-номер был добавлен в
    # PR раньше и воркер видит «свой собственный» collision — полезно для
    # понимания, какой именно новый файл виноват).
    new_slugs="$(git diff "$REF"...HEAD --name-only --diff-filter=A 2>/dev/null \
        | grep -E "^docs/adr/0*${num}-.*\.md$" \
        | sed -E 's|^docs/adr/0*[0-9]+-||; s|\.md$||' \
        | paste -sd ', ' -)"
    {
        printf '  %s (занято в %s: %s; новый в PR: %s)\n' "$num" "$REF" "${base_slugs:-<нет>}" "${new_slugs:-<нет>}"
    } >&2
    COLLISION_LINES+=("$num")
done <<< "$COLLISION"

# Next free slot.
MAX_NUM="$(printf '%s\n' "$EXISTING_ADRS" | sort -n | tail -n1)"
NEXT_FREE=$((MAX_NUM + 1))
NEXT_FREE_PADDED="$(printf '%04d' "$NEXT_FREE" 2>/dev/null || echo "$NEXT_FREE")"

{
    echo ""
    echo "ERROR: ADR namespace collision detected."
    echo "  Новый ADR-файл(ы) в PR используют номер(а), уже занятые в $REF:"
    printf '  %s\n' "${COLLISION_LINES[@]}"
    echo ""
    echo "  Next free slot: $NEXT_FREE_PADDED"
    echo ""
    echo "  Что делать:"
    echo "    1. Переименуй файл(ы) в collisions → $NEXT_FREE_PADDED-<новый-slug>.md"
    echo "    2. Поправь внутренний H1 «# ADR-NNNN: ...» — он обязан совпадать с именем файла (ADR-0030 §2.1)."
    echo "    3. Перепрогон: bash scripts/agent_flow/validate_adr_namespace.sh"
    echo ""
    echo "  Если $NEXT_FREE_PADDED занят И в $REF (race condition: параллельный PR с тем же номером) —"
    echo "  перепрогон fetch + ls-tree, чтобы получить свежий next-free."
} >&2

exit 1
