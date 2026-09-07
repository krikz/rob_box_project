#!/bin/bash
# ============================================================================
# validate_pr_scope.sh — post-PR gate: блокирует push/PR если в diff vs
# BASE_REF (default origin/develop) есть файлы вне allowed prefixes.
#
# Issue #2038 (повтор #1978/#1979/#2036, ADR-0054 wake stream): воркеры
# притаскивают в PR застрявшие коммиты прошлых эпиков (msgpack encoder
# AV-17, supervisor_state, status_hud, tests) — 12 «чужих» файлов. Юзер
# просит post-flight gate: «после того как они завершили работу тоже
# [проверить]».
#
# Это **второй** рубеж после freshness-check (ADR-0045, validate_branch_freshness.sh):
#   - freshness: ветка ≤ MAX_BEHIND коммитов behind origin/develop;
#   - scope:     diff vs origin/develop содержит только разрешённые prefix'ы.
# Freshness-check на #2036 бы прогорел, потому что base sha 7bf025cb уже
# содержит AV-17 squash-merge (#1944) — коммиты wip(av-17) пришли из старой
# ветки `z-{agent}/1909-av-17-webxr-client-v2-...` через rebase.
#
# Использование (воркер вызывает перед `gh pr create`):
#   PR_ALLOWED_PREFIXES="docs/adr/,src/rob_box_voice/" \
#       bash scripts/agent_flow/validate_pr_scope.sh [BASE_REF]
#
# Без PR_ALLOWED_PREFIXES — режим INFO (exit 0), только печатает список файлов.
# Это сделано потому, что правило «явно указывать scope» должно внедряться
# через карточку-инструкцию, а не через жёсткий дефолт (который бы блокировал
# 100% существующих PR — антипаттерн «kill switch без ramp»).
#
# Env:
#   PR_ALLOWED_PREFIXES — comma-separated list of allowed path prefixes
#                          (например: "docs/adr/,src/rob_box_voice/"). Файл
#                          подпадает под allowed если его путь начинается с
#                          одного из префиксов (case-sensitive, после trim).
#   PR_ALLOWED_GLOBS     — comma-separated list of fnmatch-style globs
#                          (например: "*.md,docs/**/*.png"). Дополняет, а
#                          не заменяет prefix'ы; обрабатывается последним.
#   MAX_OUT_OF_SCOPE=10  — максимум out-of-scope файлов для INFO-режима;
#                          > MAX → exit 1 даже в INFO (defensive).
#   SKIP_PR_SCOPE=true   — opt-out (legitimate fix для смежного файла, который
#                          формально вне scope карточки).
#   BASE_REF             — что считать эталоном (default origin/develop).
#   GITHUB_EVENT_NAME=pull_request + MERGE_COMMIT_INFERRED=true → skip.
#
# Exit codes:
#   0 — OK (нет out-of-scope файлов ИЛИ skipped ИЛИ info-режим)
#   1 — есть out-of-scope файлы (блокирующий fail); в stderr — список.
#   2 — usage error (нет git, base недоступен).
#
# Регресси-тест: scripts/agent_flow/tests/test_validate_pr_scope.sh
# (6 сценариев: OK, only-prefixes, prefix+glob, out-of-scope, INFO-режим,
# merge-commit skip).
# ============================================================================
set -euo pipefail

BASE_REF="${1:-origin/develop}"
DRIFT_LOG="${HOME:-/home/builder}/.hermes/state/pr_scope_drift_max"

# Skip на merge-commit (CI) — он не «ветка воркера».
if [ "${GITHUB_EVENT_NAME:-}" = "pull_request" ] && \
   [ "${MERGE_COMMIT_INFERRED:-}" = "true" ]; then
    echo "[validate_pr_scope] skip: merge-commit (CI)"
    exit 0
fi

if [ "${SKIP_PR_SCOPE:-}" = "true" ]; then
    echo "[validate_pr_scope] SKIP via SKIP_PR_SCOPE=true"
    exit 0
fi

if ! command -v git >/dev/null 2>&1; then
    echo "ERROR: git not in PATH" >&2
    exit 2
fi

# Безопасная проверка base (set -euo pipefail + непонятный base не должны
# тихо проходить). resolve-base возвращает 1 если ref недоступен локально.
if ! git rev-parse --verify --quiet "$BASE_REF" >/dev/null 2>&1; then
    echo "ERROR: base ref '$BASE_REF' not resolvable. Fetch and retry:" >&2
    echo "  git fetch origin $(echo "$BASE_REF" | sed 's|^origin/||')" >&2
    exit 2
fi

# Список файлов в diff: уникальный, отсортированный. Используем «names-only»
# через «--name-only» — самый дешёвый способ получить ровно то, что пойдёт в PR.
DIFF_FILES="$(git diff --name-only "$BASE_REF...HEAD" 2>/dev/null | sort -u || true)"

if [ -z "$DIFF_FILES" ]; then
    echo "[validate_pr_scope] OK: no file changes between $BASE_REF and HEAD"
    exit 0
fi

# Парсим allowed prefixes и globs.
PREFIXES=""
if [ -n "${PR_ALLOWED_PREFIXES:-}" ]; then
    # IFS=',' нормально читает csv без учёта кавычек — на текущем масштабе
    # (карточки пишут руками) этого достаточно; пробелы trim'аем.
    PREFIXES="$(printf '%s' "$PR_ALLOWED_PREFIXES" | tr ',' '\n' | sed 's/^[[:space:]]*//; s/[[:space:]]*$//' | sed '/^$/d')"
fi
GLOBS=""
if [ -n "${PR_ALLOWED_GLOBS:-}" ]; then
    GLOBS="$(printf '%s' "$PR_ALLOWED_GLOBS" | tr ',' '\n' | sed 's/^[[:space:]]*//; s/[[:space:]]*$//' | sed '/^$/d')"
fi

# INFO-режим: ни prefixes, ни globs. Выводим файлы и выходим без fail.
if [ -z "$PREFIXES" ] && [ -z "$GLOBS" ]; then
    echo "[validate_pr_scope] INFO: PR_ALLOWED_PREFIXES not set; showing diff only:"
    echo "$DIFF_FILES" | sed 's/^/  /'
    echo "Set PR_ALLOWED_PREFIXES='docs/adr/,src/rob_box_voice/' to enforce (see ADR-0055)."
    # Defensive: даже в INFO-режиме exit 1 если файлов слишком много — почти
    # наверняка drift (нормальная задача редко меняет > 10 файлов).
    MAX_OUT_OF_SCOPE="${MAX_OUT_OF_SCOPE:-10}"
    COUNT="$(printf '%s\n' "$DIFF_FILES" | wc -l | tr -d ' ')"
    if [ "$COUNT" -gt "$MAX_OUT_OF_SCOPE" ] 2>/dev/null; then
        echo "WARN: $COUNT files in diff > MAX_OUT_OF_SCOPE=$MAX_OUT_OF_SCOPE — possible scope drift" >&2
        exit 1
    fi
    exit 0
fi

OUT_OF_SCOPE=""
while IFS= read -r f; do
    [ -n "$f" ] || continue
    ALLOWED=0
    # 1) Prefix check.
    if [ -n "$PREFIXES" ]; then
        while IFS= read -r p; do
            [ -n "$p" ] || continue
            case "$f" in
                "$p"*) ALLOWED=1; break ;;
            esac
        done <<< "$PREFIXES"
    fi
    # 2) Glob check (fnmatch через case — без зависимостей).
    if [ "$ALLOWED" = "0" ] && [ -n "$GLOBS" ]; then
        while IFS= read -r g; do
            [ -n "$g" ] || continue
            # bash case поддерживает *, ? — для ** нужен extglob. У нас fnmatch-
            # не нужен (paths типа "*.md", "docs/**/*.png" → раскладываем
            # вручную: для ** это "docs/*/*.png" и т.п. — TODO если спросят).
            # Пока — простой wildcard через case.
            case "$f" in
                $g) ALLOWED=1; break ;;
            esac
        done <<< "$GLOBS"
    fi
    if [ "$ALLOWED" = "0" ]; then
        OUT_OF_SCOPE="$OUT_OF_SCOPE
$f"
    fi
done <<< "$DIFF_FILES"

if [ -z "$OUT_OF_SCOPE" ]; then
    echo "[validate_pr_scope] OK: all $(printf '%s\n' "$DIFF_FILES" | wc -l | tr -d ' ') files in allowed scope (base=$BASE_REF)"
    exit 0
fi

OUT_COUNT="$(printf '%s\n' "$OUT_OF_SCOPE" | sed '/^$/d' | wc -l | tr -d ' ')"
TOTAL_COUNT="$(printf '%s\n' "$DIFF_FILES" | wc -l | tr -d ' ')"

echo "FAIL: $OUT_COUNT of $TOTAL_COUNT files in diff vs $BASE_REF are out-of-scope" >&2
echo "  Allowed prefixes: $PREFIXES" >&2
echo "  Allowed globs: $GLOBS" >&2
echo "  Out-of-scope files (no allowed prefix matched):" >&2
printf '%s\n' "$OUT_OF_SCOPE" | sed '/^$/d' | sed 's/^/    /' >&2
echo "" >&2
echo "  Why this matters (issue #2038, ADR-0055):" >&2
echo "    PR #2036 притащил 12 файлов webxr_client/* + style.css от эпика AV-17," >&2
echo "    которые не относились к ADR-0054 wake stream — drift." >&2
echo "" >&2
echo "  Fix path:" >&2
echo "    1) Если чужие файлы реально нужны — cherry-pick нужные коммиты" >&2
echo "       и расширь PR_ALLOWED_PREFIXES явно (и напиши в карточке)." >&2
echo "    2) Если нет — пересоздай ветку:" >&2
echo "         git fetch origin develop" >&2
echo "         git checkout origin/develop -b z-\$agent/<id>-\$slug" >&2
echo "         git cherry-pick <only-relevant-commits>" >&2
echo "    3) Opt-out: SKIP_PR_SCOPE=true (только для legitimate fix'а)" >&2

# Persist max-drift stat для cron-мониторинга (аналог freshness_drift_max).
mkdir -p "$(dirname "$DRIFT_LOG")" 2>/dev/null || true
HEAD_REF="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo HEAD)"
echo "$(date +%s) ${HEAD_REF} ${OUT_COUNT}/${TOTAL_COUNT}" >> "$DRIFT_LOG" 2>/dev/null || true

exit 1
