#!/bin/bash
# ============================================================================
# retro_dryrun_last14d.sh — DRY_RUN прогон retro-path за последние 14 дней
# (ретро 07.09 #2069: «в DRY_RUN по последним 14 дням — в логе нет
# закрытий, которых не должно быть»).
#
# Что делает:
#   1. DRY_RUN=true + RETRO_MERGED_DAYS=14 → merge-gate проходит по merged
#      PR в окне, но НЕ делает реальных gh issue close/comment.
#   2. Парсит вывод stderr (логи merge-gate) на наличие строк
#      «retro-path: issue #N … closing» — это были бы реальные close'ы.
#   3. Если таких строк нет — PASS. Иначе — FAIL с перечнем issue'ов, у
#      которых retro-path СОБИРАЛСЯ закрыть (что было бы багом фикса).
#
# Использование:
#   bash scripts/agent_flow/tests/retro_dryrun_last14d.sh
#
# CI hook: запуск после merge-gate тестов, перед закрытием PR. Это страховка
# от регрессии: «опять закрыли что-то не то».
#
# Зависимости: gh auth (krikz credentials), GH_REPO env.
# ============================================================================
set -euo pipefail

# Загружаем пути из текущего репо
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
MERGE_GATE="$REPO_ROOT/scripts/agent_flow/agent-flow-merge-gate.sh"

if [ ! -x "$MERGE_GATE" ]; then
    echo "FAIL: $MERGE_GATE не найден или не исполняемый" >&2
    exit 2
fi

# gh auth (берёт из env как остальные тесты)
export GH_CONFIG_DIR="${GH_CONFIG_DIR:-/home/builder/.config/gh}"
if ! gh auth status >/dev/null 2>&1; then
    echo "FAIL: gh auth status не прошёл (проверьте GH_CONFIG_DIR/GH_TOKEN)" >&2
    exit 2
fi

export GH_REPO="${GH_REPO:-krikz/rob_box_project}"
export RETRO_MERGED_DAYS="${RETRO_MERGED_DAYS:-14}"
export DRY_RUN=true
export KANBAN_DB="/tmp/retro_dryrun_kanban_$$"
touch "$KANBAN_DB"

# Запуск merge-gate. stderr содержит «retro-path: issue #N ... closing»
# строки. Эти строки пишутся ПЕРЕД реальным close — но т.к. DRY_RUN=true,
# реального close не будет.
STAGE_LOG="$(mktemp /tmp/retro-dryrun-XXXXXX.log)"
trap 'rm -f "$STAGE_LOG" "$KANBAN_DB"' EXIT

# Аналогично production: HOME=/home/builder форсит merge-gate, LOCK_FILE
# изолируем в tmp.
export HOME="${HOME:-/home/builder}"
LOCK_FILE="$(mktemp /tmp/retro-dryrun-lock-XXXXXX.lock)"
export LOCK_FILE
trap 'rm -f "$STAGE_LOG" "$KANBAN_DB" "$LOCK_FILE"' EXIT

echo "=== Retro-path DRY_RUN за $RETRO_MERGED_DAYS дней для $GH_REPO ==="
echo "(merge-gate работает в DRY_RUN — никаких реальных close'ов)"
echo ""

if ! bash "$MERGE_GATE" >/dev/null 2>"$STAGE_LOG"; then
    echo "FAIL: merge-gate вернул ошибку (см. $STAGE_LOG)" >&2
    tail -20 "$STAGE_LOG" >&2
    exit 2
fi

# Парсим stderr на «closing» — это были бы потенциальные баги фикса,
# которые DRY_RUN не выполнил. В норме после фикса #2069 их быть не должно
# для type:functional/perf/testing issues (т.к. docs-only guard их skip'ает).
# Но closing для process-issues или e2e-PASS — это норма.
UNEXPECTED_CLOSING="$(grep -E 'retro-path: issue #[0-9]+ (PASS-доказательство|CLOSED|state=|already-closed)' "$STAGE_LOG" || true)"

echo "=== Статистика за $RETRO_MERGED_DAYS дней ==="
CLOSED_COUNT="$(grep -cE 'retro-path: issue #[0-9]+ CLOSED' "$STAGE_LOG" || true)"
LABELED_COUNT="$(grep -cE 'ретро-путь: PR #[0-9]+ смержен, но PASS-доказательства не найдено' "$STAGE_LOG" || true)"
SKIPPED_COUNT="$(grep -cE 'retro-path: issue #[0-9]+ ' "$STAGE_LOG" || true)"

echo "  Потенциальные close (DRY_RUN подавлены): $CLOSED_COUNT"
echo "  Поставлено needs-e2e (e2e-process возьмёт): $LABELED_COUNT"
echo "  Всего retro-path-строк в логе: $SKIPPED_COUNT"

if [ -z "$UNEXPECTED_CLOSING" ]; then
    echo ""
    echo "PASS: retro-path не собирался закрыть issue, которые не должен был."
    exit 0
fi

echo ""
echo "FAIL: retro-path собрался закрыть следующие issues (DRY_RUN — реальных close не было):"
echo "$UNEXPECTED_CLOSING"
echo ""
echo "Полный лог: $STAGE_LOG"
echo ""
echo "ВАЖНО: проверьте вручную, что ни одно из этих issues не должно быть закрыто."
echo "Если ВСЕ — нормальные process-fix / e2e-PASS, добавьте exclude-pattern."
exit 1