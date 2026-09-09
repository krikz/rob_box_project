#!/bin/bash
# ============================================================================
# agent-flow-openspec-sync.sh
#
# Единая точка sync между OpenSpec changes/ и agent-flow процессом.
# Вызывается из:
#   * agent-flow-triage.sh      — после `hermes kanban create` → create-changes
#   * agent-flow-merge-gate.sh  — после merge PR → archive-change (apply specs)
#
# Подкоманды:
#   create-change <issue-number> <task-id> <slug> <title>
#       Создаёт openspec/changes/<task-id>-<slug>/{proposal.md, tasks.md, README.md,
#       .openspec.yaml} с минимальным шаблоном. Воркер расширяет (specs/, design.md).
#
#   archive-change <issue-number> <task-id> <slug>
#       Архивирует openspec/changes/<task-id>-<slug>/ в archive/ и применяет
#       изменения к openspec/specs/. Идемпотентно (если уже в archive/ — noop).
#
#   status
#       Печатает JSON со списком active changes + specs (для /health и crons).
#
# Дизайн (ADR-0039):
#   * Гибрид: triage создаёт МИНИМУМ (proposal + tasks), воркер расширяет
#     specs/ и design.md, merge-gate автоматически архивирует.
#   * Скрипт не падает при ошибке OpenSpec (warn + return 0) — agent-flow не
#     должен блокироваться из-за OpenSpec. Возвращаемый код: 0=ok, 1=hard fail.
#   * `--root <path>` позволяет тестировать на sandbox (tests/ использует tmp).
#
# ENV:
#   OPENSPEC_ROOT      путь к корню OpenSpec (по умолчанию ищет ./openspec,
#                      ../openspec, $REPO_DIR/openspec, docs/research/openspec-pilot/openspec)
#   OPENSPEC_BIN       путь к openspec CLI (default: openspec)
#   DRY_RUN            если "true", только печатает команды
#   AGENT_FLOW_OPENSPEC_DISABLED  если "1", скрипт — noop (для emergency off)
#
# SOT: <repo>/scripts/agent_flow/agent-flow-openspec-sync.sh
# ============================================================================
set -euo pipefail

# --- paths / env ------------------------------------------------------------
SCRIPT_NAME="$(basename "${BASH_SOURCE[0]}")"
LOG_PREFIX="${LOG_PREFIX:-[agent-flow-openspec-sync]}"
LOG() { printf '%s %s\n' "$LOG_PREFIX" "$*" >&2; }
DIE() { printf '%s FATAL: %s\n' "$LOG_PREFIX" "$*" >&2; exit 1; }

OPENSPEC_BIN="${OPENSPEC_BIN:-$(command -v openspec 2>/dev/null || echo openspec)}"
DRY_RUN="${DRY_RUN:-false}"

if [ "${AGENT_FLOW_OPENSPEC_DISABLED:-0}" = "1" ]; then
    LOG "AGENT_FLOW_OPENSPEC_DISABLED=1 — skipping"
    exit 0
fi

# --- resolve OpenSpec root -------------------------------------------------
# Поиск в порядке приоритета:
#   1. $OPENSPEC_ROOT (если задан)
#   2. ./openspec  (cwd)
#   3. $REPO_DIR/openspec (если задан)
#   4. docs/research/openspec-pilot/openspec  (pilot location)
#   5. ../openspec  (parent of cwd)
resolve_openspec_root() {
    local candidate
    if [ -n "${OPENSPEC_ROOT:-}" ] && [ -d "${OPENSPEC_ROOT}" ]; then
        echo "${OPENSPEC_ROOT}"
        return 0
    fi
    for candidate in \
        "./openspec" \
        "${REPO_DIR:-}/openspec" \
        "docs/research/openspec-pilot/openspec" \
        "../openspec"; do
        if [ -d "$candidate" ] && [ -f "$candidate/config.yaml" ]; then
            (cd "$candidate" && pwd)
            return 0
        fi
    done
    return 1
}

OPENSPEC_ROOT="$(resolve_openspec_root || true)"
if [ -z "$OPENSPEC_ROOT" ]; then
    LOG "OpenSpec root not found — skipping (set OPENSPEC_ROOT or REPO_DIR)"
    exit 0
fi
LOG "OpenSpec root: $OPENSPEC_ROOT"

# --- slugify ----------------------------------------------------------------
# Превращает произвольную строку в kebab-case slug (макс 50 символов).
slugify() {
    local s="$1"
    echo "$s" | tr '[:upper:]' '[:lower:]' \
        | sed -E 's/[^a-z0-9]+/-/g; s/^-+//; s/-+$//' \
        | cut -c1-50
}

# --- сейчас (ISO 8601) -----------------------------------------------------
iso_now() { date -u +"%Y-%m-%dT%H:%M:%SZ"; }

# --- create-change ----------------------------------------------------------
# $1=issue_number $2=task_id $3=slug $4=title $5=issue_url (опц.) $6=body (опц.)
cmd_create_change() {
    local issue_number="$1"
    local task_id="$2"
    local slug="$3"
    local title="$4"
    local issue_url="${5:-}"
    local body="${6:-}"

    [ -z "$task_id" ] && DIE "create-change: task_id required"
    [ -z "$slug" ] && DIE "create-change: slug required"

    local change_name="${task_id}-${slug}"
    local change_dir="${OPENSPEC_ROOT}/changes/${change_name}"

    # Идемпотентность: если уже есть (активный или в archive/), skip
    if [ -d "$change_dir" ]; then
        LOG "create-change: ${change_name} already exists — skip"
        return 0
    fi
    if [ -d "${OPENSPEC_ROOT}/changes/archive/${change_name}" ]; then
        LOG "create-change: ${change_name} already archived — skip"
        return 0
    fi

    if [ "$DRY_RUN" = "true" ]; then
        LOG "DRY-RUN: would create ${change_dir}"
        return 0
    fi

    LOG "create-change: ${change_name}"
    mkdir -p "${change_dir}/specs"

    # .openspec.yaml — минимальный manifest
    cat > "${change_dir}/.openspec.yaml" <<EOF
schema: spec-driven
created: $(date -u +"%Y-%m-%d")
issue: "#${issue_number}"
task: "${task_id}"
agent: "agent-flow"
goal: "${title}"
EOF

    # README.md — ссылка на issue
    cat > "${change_dir}/README.md" <<EOF
# ${change_name}

Авто-сгенерировано agent-flow-triage для issue #${issue_number} (kanban ${task_id}).

Воркер должен расширить: \`proposal.md\`, \`specs/\`, \`design.md\`, \`tasks.md\`.

${issue_url:+Issue: ${issue_url}}
EOF

    # proposal.md — skeleton, воркер дописывает Why/What Changes/Impact
    cat > "${change_dir}/proposal.md" <<EOF
## Why

<!-- Воркер: кратко (2-3 предложения) — какую проблему решает change.
     Источник: issue body, ADR (если есть), контекст из kanban-карточки. -->

## What Changes

<!-- Воркер: список изменений (bullet list). Сверху — high-level, снизу — детали. -->

## Capabilities

### New Capabilities
<!-- Воркер: какие НОВЫЕ capabilities появляются (id, описание). -->

### Modified Capabilities
<!-- Воркер: какие существующие capabilities меняются. -->

## Impact

<!-- Воркер: какие компоненты / файлы / топики / сервисы затрагиваются. -->

## Source

* Issue: #${issue_number} (kanban ${task_id})
* Title: ${title}
* Created: $(iso_now) by agent-flow-triage
EOF

    # tasks.md — skeleton, воркер дописывает чеклист
    cat > "${change_dir}/tasks.md" <<EOF
## 1. Implementation

- [ ] 1.1 <!-- Воркер: первый concrete step (не более 2 часов работы) -->
- [ ] 1.2 <!-- Воркер: следующий step -->

## 2. Tests

- [ ] 2.1 <!-- Воркер: unit / integration test -->

## 3. Documentation

- [ ] 3.1 <!-- Воркер: ADR / docs/process / README обновления -->

## 4. Deployment

- [ ] 4.1 <!-- Воркер: deploy step (rebuild / rollout / smoke) -->
- [ ] 4.2 PR влит в \`develop\` → agent-flow-merge-gate заархивирует change
EOF

    # specs/ — пустая (воркер создаст capability specs)
    : > "${change_dir}/specs/.gitkeep" 2>/dev/null || true

    LOG "create-change: ${change_name} ok (skeleton)"
    return 0
}

# --- archive-change ---------------------------------------------------------
# $1=issue_number $2=task_id $3=slug $4=pr_number (опц.)
cmd_archive_change() {
    local issue_number="$1"
    local task_id="$2"
    local slug="$3"
    local pr_number="${4:-}"

    [ -z "$task_id" ] && DIE "archive-change: task_id required"
    [ -z "$slug" ] && DIE "archive-change: slug required"

    local change_name="${task_id}-${slug}"
    local change_dir="${OPENSPEC_ROOT}/changes/${change_name}"

    # Идемпотентность: если уже в archive/ — skip
    if [ ! -d "$change_dir" ]; then
        if [ -d "${OPENSPEC_ROOT}/changes/archive/${change_name}" ]; then
            LOG "archive-change: ${change_name} already archived — skip"
            return 0
        fi
        LOG "archive-change: ${change_name} not found in active changes — skip"
        return 0
    fi

    # Sanity: должен содержать proposal.md (воркер его создал/расширил)
    if [ ! -f "${change_dir}/proposal.md" ]; then
        LOG "archive-change: WARN ${change_name} has no proposal.md — skipping apply-specs (will just move)"
    fi

    if [ "$DRY_RUN" = "true" ]; then
        LOG "DRY-RUN: would archive ${change_name} (pr=#${pr_number})"
        return 0
    fi

    LOG "archive-change: ${change_name} (pr=#${pr_number})"

    # Шаг 1: openspec archive применяет изменения к specs/ и переносит в archive/.
    # --skip-specs если change — чисто process/tooling (ADR-0039 §3).
    # --no-validate если proposal пустой/скелетный (бывает для fix-issues).
    local skip_specs_flag=""
    local no_validate_flag=""
    if [ "${OPENSPEC_SKIP_SPECS:-0}" = "1" ]; then
        skip_specs_flag="--skip-specs"
    fi
    if [ "${OPENSPEC_NO_VALIDATE:-0}" = "1" ]; then
        no_validate_flag="--no-validate"
    fi

    local archive_out=()
    if "$OPENSPEC_BIN" --root "$OPENSPEC_ROOT" archive "$change_name" \
            --yes $skip_specs_flag $no_validate_flag >/tmp/openspec-archive.$$.log 2>&1; then
        LOG "archive-change: openspec archive ${change_name} ok"
    else
        local rc=$?
        # Fallback: ручной перенос в archive/ (если openspec CLI не справился)
        LOG "archive-change: WARN openspec archive failed (rc=${rc}), falling back to manual mv"
        cat /tmp/openspec-archive.$$.log >&2 || true
        mkdir -p "${OPENSPEC_ROOT}/changes/archive"
        mv "$change_dir" "${OPENSPEC_ROOT}/changes/archive/${change_name}"
    fi
    rm -f /tmp/openspec-archive.$$.log

    # Шаг 2: комментарий в issue (если gh доступен + issue_number задан)
    if [ -n "$issue_number" ] && [ -n "${GH_REPO:-}" ] && command -v gh >/dev/null 2>&1; then
        local comment="OpenSpec: change \`${change_name}\` archived"
        if [ -n "$pr_number" ]; then
            comment="${comment} (PR #${pr_number} MERGED)"
        fi
        gh issue comment "$issue_number" --repo "$GH_REPO" --body "$comment" >/dev/null 2>&1 || \
            LOG "archive-change: WARN gh issue comment failed (non-fatal)"
    fi

    return 0
}

# --- status -----------------------------------------------------------------
# Печатает JSON со списком active changes + specs.
cmd_status() {
    if [ ! -x "$(command -v "$OPENSPEC_BIN" 2>/dev/null)" ] && ! command -v "$OPENSPEC_BIN" >/dev/null 2>&1; then
        echo '{"error":"openspec CLI not found"}'
        return 0
    fi
    local changes_json=""
    changes_json="$("$OPENSPEC_BIN" --root "$OPENSPEC_ROOT" list --changes --json 2>/dev/null || echo '[]')"
    local specs_json=""
    specs_json="$("$OPENSPEC_BIN" --root "$OPENSPEC_ROOT" list --specs --json 2>/dev/null || echo '[]')"
    python3 -c "
import json, sys
changes = json.loads('''$changes_json''') if '''$changes_json'''.strip() else []
specs = json.loads('''$specs_json''') if '''$specs_json'''.strip() else []
print(json.dumps({
    'root': '''$OPENSPEC_ROOT''',
    'active_changes': [c.get('name') for c in changes] if isinstance(changes, list) else [],
    'specs_count': len(specs) if isinstance(specs, list) else 0,
}, indent=2))
" 2>/dev/null || echo '{"error":"parse failed"}'
}

# --- main dispatch ----------------------------------------------------------
usage() {
    cat <<EOF
Usage: $SCRIPT_NAME <command> [args]

Commands:
  create-change <issue-number> <task-id> <slug> <title> [issue-url] [body]
      Создать openspec/changes/<task-id>-<slug>/ с минимальным шаблоном
      (proposal.md + tasks.md + README.md + .openspec.yaml).
      Идемпотентно: если уже есть — skip.

  archive-change <issue-number> <task-id> <slug> [pr-number]
      Заархивировать openspec/changes/<task-id>-<slug>/ → changes/archive/
      и применить изменения к openspec/specs/.
      Идемпотентно: если уже в archive/ — skip.

  status
      Печатает JSON со списком active changes + specs.

Env:
  OPENSPEC_ROOT=/path/to/openspec       (auto-detect if unset)
  REPO_DIR=/path/to/repo                (auto-detect if unset)
  DRY_RUN=true                          (печатать, не делать)
  AGENT_FLOW_OPENSPEC_DISABLED=1        (полный noop)
  OPENSPEC_SKIP_SPECS=1                 (archive без apply-specs)
  OPENSPEC_NO_VALIDATE=1                (archive без валидации)

Exit codes:
  0 — OK (или noop, или skip)
  1 — hard fail (битый env, битые аргументы)
EOF
}

main() {
    [ $# -ge 1 ] || { usage; exit 1; }
    local cmd="$1"; shift

    case "$cmd" in
        create-change)
            [ $# -ge 4 ] || DIE "create-change: need issue-number task-id slug title"
            cmd_create_change "$@"
            ;;
        archive-change)
            [ $# -ge 3 ] || DIE "archive-change: need issue-number task-id slug"
            cmd_archive_change "$@"
            ;;
        status)
            cmd_status
            ;;
        -h|--help|help|"")
            usage
            ;;
        *)
            DIE "unknown command: $cmd (use --help)"
            ;;
    esac
}

main "$@"
