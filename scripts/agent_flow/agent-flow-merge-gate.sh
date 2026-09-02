#!/bin/bash
# ============================================================================
# SOT (source-of-truth): <repo>/scripts/agent_flow/agent-flow-merge-gate.sh
# Правим ТОЛЬКО здесь + commit + merge в develop. На хост раскладывает
# `bash <repo>/scripts/agent_flow/install.sh` — hardlink-копиями (cp -al), НЕ
# симлинками: симлинк в ~/.hermes/scripts/ ресолвится наружу и отклоняется
# guard'ом hermes-agent scheduler.py::_validate_script_path (ретро 11.08
# t_a6a236e0d9f0470e — 50 упавших тиков подряд, 1ч42м даунтайма).
# Полный список путей раскладки — в install.sh, сверку копий держит
# agent-flow-drift-detect.sh. Ручная правка копии на хосте затрётся.
# ============================================================================
# agent-flow-merge-gate.sh — Phase 3: PR is green -> add label `needs-e2e` to issue.
#
# Pure bash. No LLM. Idempotent. Driven entirely by env (see
# ~/.hermes/profiles/agent-flow/.env).
#
# Pipeline per tick (per AGENT_FLOW_PROPOSAL §3.3, instantiated per §3.2):
#   1. MAINTENANCE gate (remote + local) -> exit 0 if paused
#   2. gh auth check                     -> exit 1 if not authed
#   3. List open issues with label `hermes` (carry state of triage)
#   4. For each issue:
#        a. Skip if it already has `needs-e2e`, `e2e-done`, or `e2e:rejected`
#        b. Look up the kanban-card id from the `kanban: t_<id>` comment marker
#        c. Derive PR branch name: agent/<issue>-<slug>
#        d. Find the PR by head branch (idempotent — gh pr list is the lookup)
#        e. If statusCheckRollup is all SUCCESS and merge_state=clean and
#           the PR is OPEN against `develop` -> add label `needs-e2e`
#        f. Idempotency: skip if label already present (3a short-circuits)
#   5. flock lock prevents parallel ticks.
#
# Gates G2..G7 follow the table in agent-flow SKILL.md (G2 gh auth, G3 GH
# rate-limit, G6 flock sentinel). We never block on the dispatcher (G7).

set -euo pipefail

# --- defaults (overridden by env / .env) -------------------------------------
# NOTE: hardcode /home/builder/.hermes — cron from per-profile gateway sets
# HERMES_HOME to the profile dir; PROFILE_ENV would then point at a
# non-existent path and GH_REPO would never load (see agent-flow-triage.sh).
HERMES_HOME=/home/builder/.hermes
HERMES_BIN="${HERMES_BIN:-/home/builder/.hermes/hermes-agent/venv/bin/hermes}"

# Force HOME=/home/builder — see comments in agent-flow-triage.sh.
export HOME=/home/builder

ISSUE_LABEL="${ISSUE_LABEL:-hermes}"
NEEDS_E2E_LABEL="${NEEDS_E2E_LABEL:-needs-e2e}"
NEEDS_REVIEW_LABEL="${NEEDS_REVIEW_LABEL:-needs-review}"
DONE_LABEL="${DONE_LABEL:-e2e-done}"
REJECTED_LABEL="${REJECTED_LABEL:-e2e:rejected}"
NO_E2E_LABEL="${NO_E2E_LABEL:-no-e2e-required}"
BIG_BANG_OVERRIDE_LABEL="${BIG_BANG_OVERRIDE_LABEL:-big-bang-override}"
# Ретро 31.08 t_04371252 (PR #1753): stale-branch scan ставит эту метку на
# OPEN PR с уже влитой веткой + функциональным диффом, чтобы downstream
# (e2e-process, clean-pr-sweep) могли skip-нуть без дополнительных проверок.
STALE_BRANCH_REUSE_LABEL="${STALE_BRANCH_REUSE_LABEL:-stale-branch-reuse}"
# Ретро 25.08 t_00ba0224: на origin/develop обнаружена нумерационная коллизия
# ADR — 5 файлов под 3 номерами (0027×3, 0028×2). merge-gate должен проверять,
# что NNNN в новом docs/adr/NNNN-*.md не занят существующим файлом в develop
# (иначе rename/перенос ломают историю ссылок на ADR). Шифу/воркер может
# пометить issue меткой ADR_COLLISION_OVERRIDE_LABEL, чтобы продолжить merge
# при полной уверенности (например, согласованный re-numbering). Без override
# PR блокируется: comment (24h dedup) + метка на issue.
ADR_COLLISION_OVERRIDE_LABEL="${ADR_COLLISION_OVERRIDE_LABEL:-adr-collision-override}"
ADR_COLLISION_BLOCKED_LABEL="${ADR_COLLISION_BLOCKED_LABEL:-agent-flow:adr-collision}"
ADR_COLLISION_COMMENT_DEDUP_HOURS="${ADR_COLLISION_COMMENT_DEDUP_HOURS:-24}"
# ADR-0013 (docs/adr/0013-incremental-delivery-over-big-bang.md): PR > 50
# commits OR > 3000 lines is forbidden without an explicit `big-bang-override`
# label on the issue. Шифу (товарищ) is the only one allowed to set it. We
# enforce the rule BOTH at merge-gate (before adding needs-e2e) AND at triage
# (before creating the card) so the worker can't even start a 100-commit job
# without the override. See also agent-flow-triage.sh `big_bang_check()`.
BIG_BANG_MAX_COMMITS="${BIG_BANG_MAX_COMMITS:-50}"
BIG_BANG_MAX_LINES="${BIG_BANG_MAX_LINES:-3000}"
DEVELOP_BRANCH="${DEVELOP_BRANCH:-develop}"
# Ретро 22.08 t_562a8682: open PR с needs-review может «отстать» от develop
# на десятки/сотни коммитов (CI на собственной ветке зелёный → CLEAN),
# но фактическая merge-base = ДРЕВНИЙ develop. Юзер жмёт merge → github
# делает merge-commit с 180 коммитами впереди develop (или огромный merge
# commit с потенциальными конфликтами) → CONFLICTING. Решение: для PR
# с needs-review проверяем ahead-of-develop через REST compare API; если
# выше порога — alert в карточку воркера (rate-limited 2ч) + comment
# на issue (24h dedup). Сам PR НЕ блокируем — это watchdog, не gate.
STALE_REBASE_AHEAD_THRESHOLD="${STALE_REBASE_AHEAD_THRESHOLD:-30}"
STALE_REBASE_COMMENT_DEDUP_HOURS="${STALE_REBASE_COMMENT_DEDUP_HOURS:-24}"
STALE_REBASE_REMINDER_COOLDOWN_SECONDS="${STALE_REBASE_REMINDER_COOLDOWN_SECONDS:-7200}"
# Ретро 31.08 t_9d375e3e / ADR-0035: stale-after-upstream-fix detector для
# diagnostic-карточек (PR #1743, retro t_e00f448d). Маркеры `<!-- diag-* -->`
# в body диагностической карточки (PR + head SHA + sig + tests + class +
# created-ts) → scan-all-prs обнаруживает upstream-фикс (стратегии A/B/C)
# и auto-block с kind=transient.
STALE_AFTER_UPSTREAM_FIX_SCAN="${STALE_AFTER_UPSTREAM_FIX_SCAN:-true}"
STALE_AFTER_UPSTREAM_FIX_COOLDOWN_SECONDS="${STALE_AFTER_UPSTREAM_FIX_COOLDOWN_SECONDS:-7200}"
# Ретро 15.08 t_238ff3f7: deploy-issue label-less orphan backstop. L-Deploy and
# Verify создаёт deploy-issues с версией workflow-файла С ВЕТКИ e2e-раунда
# (z-{e2e}/test-round-N). Если round-ветка ответвилась ДО фикса #1263
# (hermes+agent:devops при создании), issue получает только метку deployment →
# агентский триаж (фильтр по hermes) карточку не создаёт → issue висит сиротой
# (#1276). Минимальный возраст для реконсилейшна (минуты): свежие issue
# пропускаем — workflow раунда может ещё дописывать/метить их сам.
DEPLOY_RECONCILE_MINUTES="${DEPLOY_RECONCILE_MINUTES:-30}"
MAINTENANCE_BRANCH="${MAINTENANCE_BRANCH:-develop}"
MAINTENANCE_FILE="${MAINTENANCE_FILE:-MAINTENANCE}"
# Ретро 18.08 t_873ebef2 (#1391, дополнение к PR #1399 от e3f227e2):
# PR #1399 покрыл только MERGED+e2e-done путь — тут мы добавляем:
#   1) Защиту Q22-orphan-close пути (там нет e2e-done в принципе,
#      нужен широкий recent-reopen детект).
#   2) Whitelist label `user-reopened-this` — ручной override Шифу,
#      который гарантированно блокирует close независимо от timeline.
USER_REOPEN_AUDIT_LABEL="${USER_REOPEN_AUDIT_LABEL:-user-reopened-this}"
USER_REOPEN_RECENT_DAYS="${USER_REOPEN_RECENT_DAYS:-7}"
AGENT_FLOW_DEFAULT_ROLE="${AGENT_FLOW_DEFAULT_ROLE:-architect}"
DRY_RUN="${DRY_RUN:-false}"
ISSUE_LIMIT="${ISSUE_LIMIT:-50}"
LOCK_FILE="${LOCK_FILE:-/tmp/agent-flow-merge-gate.lock}"
LOG_PREFIX="${LOG_PREFIX:-[agent-flow-merge-gate]}"

# --- shared library bootstrap ------------------------------------------------
# Отсюда merge-gate берёт: af_load_profile_env, af_flock_guard_or_exit,
# af_maintenance_gate_or_exit, gh_list_issues_by_label, has_label, slugify,
# detect_pr_kind, free_stale_worktrees_for (дедуп 30.08). Source ДО загрузки
# .env — сам загрузчик живёт в библиотеке.
_LIB_DIR_HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=lib_agent_flow_common.sh
. "$_LIB_DIR_HERE/lib_agent_flow_common.sh"

# --- source profile .env if present -------------------------------------------
PROFILE_ENV="${HERMES_HOME}/profiles/agent-flow/.env"
af_load_profile_env "$PROFILE_ENV"

# Defensive defaults (in case .env is partial).
: "${KANBAN_BOARD:=robbox}"
: "${MAINTENANCE_BRANCH:=develop}"
: "${MAINTENANCE_FILE:=MAINTENANCE}"
: "${REPO_DIR:=}"
: "${AGENT_FLOW_DEFAULT_ROLE:=architect}"
: "${DRY_RUN:=false}"
: "${ISSUE_LABEL:=hermes}"
: "${ISSUE_LIMIT:=50}"
: "${NEEDS_E2E_LABEL:=needs-e2e}"
: "${NEEDS_REVIEW_LABEL:=needs-review}"
: "${DONE_LABEL:=e2e-done}"
: "${REJECTED_LABEL:=e2e:rejected}"
: "${NO_E2E_LABEL:=no-e2e-required}"
: "${BIG_BANG_OVERRIDE_LABEL:=big-bang-override}"
: "${BIG_BANG_MAX_COMMITS:=50}"
: "${BIG_BANG_MAX_LINES:=3000}"
: "${DEVELOP_BRANCH:=develop}"
: "${STALE_REBASE_AHEAD_THRESHOLD:=30}"
: "${STALE_REBASE_COMMENT_DEDUP_HOURS:=24}"
: "${STALE_REBASE_REMINDER_COOLDOWN_SECONDS:=7200}"

# --- shared helpers ---------------------------------------------------------
# user-unlabel guard (ретро 18.08 t_de6bea69, PR #1398) — если Шифу руками
# снял метку (e2e-done / needs-review) после auto-установки, merge-gate
# НЕ должен её возвращать в reconcile / lint-путях. Источник — рядом со
# скриптом (для тестов и для install-раскладки в ~/.hermes/...).
# shellcheck source=lib_user_unlabel_check.sh
. "$_LIB_DIR_HERE/lib_user_unlabel_check.sh"
# self-id / whoami helper (issue #1534): before any destructive
# side-effect on PR/issue (close / reopen / label / assignee) этот скрипт
# пишет «🤖 [agent:devops] script=… action=… reason=…» чтобы в истории
# GitHub было видно КТО это сделал, а не только krikz (actor = holder of
# GH token). Идемпотентность: helper сам скипает дубль в окне
# HERMES_WHOAMI_WINDOW_SECONDS (default 2h).
# shellcheck source=hermes_github.sh
. "$_LIB_DIR_HERE/hermes_github.sh"

log() { printf '%s %s %s\n' "$LOG_PREFIX" "$(date -Iseconds)" "$*" >&2; }
run() { if [ "$DRY_RUN" = "true" ]; then printf '%s DRY-RUN %s\n' "$LOG_PREFIX" "$*" >&2; else eval "$@"; fi; }

# --- функциональные файлы PR (ретро 14.08 t_28afb585, t_04371252) -----------
# Возвращает 1, если среди файлов PR есть ФУНКЦИОНАЛЬНЫЙ код (docker/, src/,
# скрипты процесса scripts/agent_flow/* и тесты процесса tests/agent_flow/*);
# 0 если все файлы в ci-only-зонах (.github/, docs/).
#
# Контекст: ретро 31.08 t_04371252 (PR #1753) показал, что guard считал
# scripts/agent_flow/* — ci-only. На PR #1753 (23 файла в scripts/agent_flow/ +
# 1 тест в tests/agent_flow/) guard формально сработал (через tests/agent_flow/*),
# но для PR, состоящего ТОЛЬКО из scripts/agent_flow/* (что типично для
# аддитивного фикса поверх влитой процессной ветки), guard бы пропустил —
# а это явный stale-branch reuse (повтор паттерна #1238/#1218). Теперь:
#   - .github/, docs/ → ci-only (не функциональные)
#   - scripts/agent_flow/, tests/agent_flow/ → ФУНКЦИОНАЛЬНЫЕ (процессные)
#   - всё остальное (src/, docker/, etc.) → функциональное
#
# Используется чтобы отличить «аддитивное продолжение docs/ci-ветки»
# (разрешено, #1197 docs W7) от «нового функционального фикса на уже
# влитой ветке» (блок, #1238, повтор — #1753).
pr_has_functional_files() {  # $1=pr_number → 1/0
    local pr_num="$1" files_json
    files_json="$(gh pr view "$pr_num" --repo "$GH_REPO" --json files \
        --jq '[.files[].path]' 2>/dev/null || echo '[]')"
    printf '%s' "$files_json" | python3 -c '
import json, sys
try:
    files = json.load(sys.stdin)
except Exception:
    files = []
# Pure ci-only: только .github/ (workflows/actions) и docs/ (markdown).
# scripts/agent_flow/* и tests/agent_flow/* — это ПРОЦЕССНЫЕ скрипты, не ci-only.
# Любой такой файл в PR = функциональное изменение → блокируем stale-reuse.
def is_ci_only(f):
    return f.startswith(".github/") or f.startswith("docs/")
ok = bool(files) and not all(is_ci_only(f) for f in files)
print("1" if ok else "0")
' 2>/dev/null || echo 0
}

# Специализированная проверка: меняет ли PR процессные скрипты/тесты?
# Возвращает 1 если среди файлов есть scripts/agent_flow/* или tests/agent_flow/*.
# Используется для дополнительного alerting в stale_branch_scan_all (даже если
# PR уже зарегистрирован как «не ci-only» по pr_has_functional_files).
pr_has_process_changes() {  # $1=pr_number → 1/0
    local pr_num="$1" files_json
    files_json="$(gh pr view "$pr_num" --repo "$GH_REPO" --json files \
        --jq '[.files[].path]' 2>/dev/null || echo '[]')"
    printf '%s' "$files_json" | python3 -c '
import json, sys
try:
    files = json.load(sys.stdin)
except Exception:
    files = []
ok = any(f.startswith("scripts/agent_flow/") or f.startswith("tests/agent_flow/") for f in files)
print("1" if ok else "0")
' 2>/dev/null || echo 0
}

# --- dead-content detector (ретро 22.08 t_e8d52cb7 / t_944df2c5, PR #1507) ---
# Ситуация: после rebase develop ушёл вперёд, в PR остались только binary
# артефакты (.ogg / .png / .bin / .wav / .mp3 / .jpg / ...), а вся
# meaningful-часть (код/тесты/конфиги) уже в develop. PR висит OPEN +
# MERGEABLE + CLEAN, но merge-ui предложит «влить» пустоту поверх develop.
# Acceptance из задачи: если PR не содержит НИ ОДНОГО meaningful-файла
# (.py|.json|.yaml|.yml|.toml|.md|.sh|.ts|.cpp|.h|.hpp|.launch.xml|.txt)
# И при этом в diff есть хотя бы один файл (т.е. не пустой PR — это
# отдельный кейс, регрессия-guard для случая «0 файлов») → помечаем PR
# меткой `dead-content` и комментим issue с предложением закрыть PR без
# merge.
#
# Возвращает 1 если PR является dead-content (только binary/asset), иначе 0.
# L3 acceptance: пустой PR (0 файлов) → 0 (не dead-content).
pr_is_dead_content() {  # $1=pr_number → 1/0
    local pr_num="$1" files_json
    files_json="$(gh pr view "$pr_num" --repo "$GH_REPO" --json files \
        --jq '[.files[].path]' 2>/dev/null || echo '[]')"
    printf '%s' "$files_json" | python3 -c '
import json, sys, re
try:
    files = json.load(sys.stdin)
except Exception:
    files = []
# Регрессия-guard: пустой PR (0 файлов) — не dead-content (L3 acceptance).
# Это может быть merge commit без изменений или PR с метаданными — отдельный
# путь, не помечаем автоматически.
if not files:
    print("0"); sys.exit(0)
# Meaningful-расширения по acceptance (.py|.json|.yaml|.yml|.toml|.md|.sh|
# .ts|.cpp|.h|.hpp|.launch.xml|.txt). Регистр не важен.
meaningful = re.compile(r"\.(py|json|ya?ml|toml|md|sh|ts|cpp|hpp?|launch\.xml|txt)$", re.IGNORECASE)
has_meaningful = any(meaningful.search(f) for f in files)
# Dead-content: есть файлы в diff, но НЕТ meaningful. Метим и идём дальше.
print("0" if has_meaningful else "1")
' 2>/dev/null || echo 0
}

# --- honesty-hint (ADR-0018, 18.08.2026) ------------------------------------
# Pre-merge проверка PR body на «голословный PASS»: если воркер не приложил
# raw-evidence (pytest / docker logs / gh run view / sqlite / git log /
# code-block) к claim-маркерам («проверил», «работает», «✅», «done» и т.п.) —
# печатает WARN в лог merge-gate. НЕ блокирует, НЕ комментирует PR (по
# дизайну — ревьюер сам решает; воркеры прогоняют локально до kanban complete).
#
# Использует scripts/agent_flow/validate_honesty.sh (рядовой, в EXPECTED).
honesty_hint_for_pr() {  # $1=pr_number → печатает WARN в stderr (если есть)
    local pr_num="$1"
    local vh
    vh="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/validate_honesty.sh"
    [ -x "$vh" ] || { log "honesty-hint: validate_honesty.sh не найден, skip"; return 0; }
    local body out rc
    body="$(gh pr view "$pr_num" --repo "$GH_REPO" --json body --jq '.body' 2>/dev/null || true)"
    [ -z "$body" ] && { log "honesty-hint: PR #${pr_num} body пустой, skip"; return 0; }
    out="$(printf '%s' "$body" | bash "$vh" 2>&1)"
    rc=$?
    if [ "$rc" -ne 0 ] || printf '%s' "$out" | grep -q '^WARN:'; then
        log "honesty-hint: PR #${pr_num} → голословные claim'ы без raw-evidence (ADR-0018, не блокер, ревьюер решит):"
        printf '%s\n' "$out" | sed 's/^/    /' >&2
    fi
}

# --- proposal-ветки архитектора (ретро 15.08 t_6024f414) --------------------
# Долгоживущие ветки вида z-architect/<slug> БЕЗ номера issue и БЕЗ t_<card>
# (например z-architect/voice-selection-proposal). По процессу proposal-ветка
# живёт после merge: в неё добавляются новые коммиты и открывается следующий
# PR (кейс #1247 → #1254 → #1255). Это НЕ ретро-ветка (z-architect/t_<id>-...)
# и НЕ issue-ветка (z-architect/NNNN-...). Stale-branch guard (t_28afb585)
# НЕ должен блокировать такие PR и снимать needs-review — иначе proposal-цикл
# рвётся и PR невидим для очереди ревью товарища Шифу.
is_proposal_branch() {  # $1=head → 1/0
    local head="$1"
    case "$head" in
        z-architect/*)
            case "$head" in
                z-architect/t_*|z-architect/[0-9]*-*) return 1 ;;
            esac
            return 0
            ;;
        *) return 1 ;;
    esac
}

# --- stale-branch re-commit guard для ВСЕХ open PR (ретро 12.08 t_d3aeaa9b) --
# Сценарий: ветка УЖЕ влита в develop (есть MERGED PR с тем же head), но
# воркер продолжал коммитить в неё ПОСЛЕ merge (база устарела; re-коммиты =
# дубли merged-содержимого) и открыл НОВЫЙ PR с той же ветки. Diff такого PR
# vs develop — РЕГРЕССИЯ (удаляет влитые voice-фиксы: dialogue_node.py,
# health.py, .image-versions). Детект: у head-ветки уже есть MERGED PR,
# отличный от текущего. Блокируем: коммент в issue/PR + НЕ ставим needs-e2e.
# Вызывается ДО раннего exit при отсутствии hermes-issues (ретро-ветки
# devops/architect issues не имеют — иначе guard бы не сработал вообще).
#
# Ретро 14.08 t_28afb585: аддитивный PR (deletions≤20) на влитой ветке — НЕ
# регрессия, НО только если это продолжение ТОЙ ЖЕ темы (docs/ci, #1197 docs
# W7: +308/-1). Если аддитивный PR несёт НОВЫЕ функциональные файлы на уже
# влитой ветке (#1238: e2e+teleop фиксы на ветке docs-PR #1218) — это
# переиспользование ветки влитого PR с новым фиксом → блокируем и снимаем
# needs-review (поставленный без e2e). Новый фикс обязан идти в НОВОЙ ветке
# z-{agent}/t_<card>-<slug>.
stale_branch_scan_all() {
    _stale_all_prs="$(gh pr list --repo "$GH_REPO" --state open \
        --json number,headRefName,title,additions,deletions 2>/dev/null || echo '[]')"
    printf '%s' "$_stale_all_prs" | python3 -c '
import json, sys
data = json.load(sys.stdin)
for pr in data:
    print("{}\t{}\t{}\t{}\t{}".format(pr["number"], pr.get("headRefName",""), pr.get("title",""), pr.get("additions",0), pr.get("deletions",0)))
' 2>/dev/null | while IFS=$'\t' read -r _spr_num _spr_head _spr_title _spr_additions _spr_deletions; do
        [ -z "$_spr_num" ] && continue
        _spr_prev_merged="$(gh pr list --repo "$GH_REPO" --state merged --head "$_spr_head" \
            --json number --jq '.[0].number // ""' 2>/dev/null || true)"
        if [ -n "$_spr_prev_merged" ] && [ "$_spr_prev_merged" != "$_spr_num" ]; then
            # Ретро 15.08 t_6024f414: proposal-ветки архитектора (z-architect/*
            # без issue-номера и без t_<card>) ЖИВУТ после merge — в них легально
            # добавляются новые коммиты и открывается следующий PR (#1247 →
            # #1254 → #1255). Для них stale-branch guard НЕ применяется: PR
            # остаётся в нормальном процессе (clean-pr-sweep поставит
            # needs-review). Иначе proposal-цикл рвётся и PR невидим для ревью.
            if is_proposal_branch "$_spr_head"; then
                log "stale-branch-scan: ветка ${_spr_head} — proposal-ветка архитектора (ретро 15.08 t_6024f414) — НЕ блокирую, needs-review не снимаю"
            else
            # Ретро 13.08 t_a3f170fe: guard бьёт по ИМЕНИ ветки, но аддитивный
            # PR (docs/ci, deletions≈0) — НЕ регрессия: ветка влита, воркер
            # до-пушил НОВЫЙ контент (#1197 docs W7: +308/-1). Регрессия =
            # удаление влитых фиксов → deletions значимы. Блокируем только её.
            if [ "${_spr_deletions:-0}" -le 20 ] 2>/dev/null; then
                # Ретро 14.08 t_28afb585: аддитивный PR на влитой ветке — НЕ
                # регрессия, НО только если это продолжение ТОЙ ЖЕ темы
                # (docs/ci, #1197 docs W7). Если аддитивный PR несёт НОВЫЕ
                # функциональные файлы (docker/, src/ и т.п.) на уже влитой
                # ветке (#1238: e2e+teleop фиксы на ветке docs-PR #1218) —
                # это переиспользование ветки влитого PR с новым фиксом →
                # блокируем и снимаем needs-review (поставленный без e2e).
                _spr_func="$(pr_has_functional_files "$_spr_num")"
                if [ "$_spr_func" = "1" ]; then
                    log "stale-branch-scan: 🛑 ветка ${_spr_head} влита через PR #${_spr_prev_merged}, PR #${_spr_num} аддитивный, но несёт ФУНКЦИОНАЛЬНЫЕ файлы — block (ретро 14.08 t_28afb585)"
                    if [ "$DRY_RUN" = "true" ]; then
                        log "DRY-RUN would: add ${STALE_BRANCH_REUSE_LABEL} + comment stale-branch block + remove needs-review on PR #${_spr_num}"
                        continue
                    fi
                    _spr_dedup_since="$(date -u -d '24 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
                    _spr_dup="$(gh api "repos/${GH_REPO}/issues/${_spr_num}/comments?since=${_spr_dedup_since}&per_page=100" \
                        --jq '[.[] | select(.body | startswith("🛑 **stale-branch reuse"))] | length' 2>/dev/null || echo 0)"
                    if [ "${_spr_dup:-0}" -eq 0 ]; then
                        gh pr comment "$_spr_num" --repo "$GH_REPO" --body \
                            "🛑 **stale-branch reuse with new functional fix** (merge-gate, ретро 14.08 t_28afb585, метка ретро 31.08 t_04371252)

Ветка \`${_spr_head}\` уже была влита в develop через PR #${_spr_prev_merged}. PR #${_spr_num} аддитивный, НО несёт НОВЫЕ функциональные фиксы (docker/, src/, scripts/agent_flow/ и т.п.) поверх уже влитой ветки — это переиспользование ветки влитого PR (повтор паттерна #1238/#1218, #1753).

**Что делать:**
1. Создай **новую** ветку от свежего origin/develop: \`git fetch origin develop && git checkout -b z-{agent}/t_<card>-<slug> origin/develop\`.
2. Перенеси ТОЛЬКО этот фикс (cherry-pick/rebase), один PR = одна тема.
3. Закрой/удали этот PR и открой новый с новой ветки.
4. needs-review ставится только после e2e-прогона PR.

Снято: \`needs-review\` (поставлен без e2e). Merge-gate **не поставит needs-e2e** на PR с уже влитой ветки и поставил метку \`${STALE_BRANCH_REUSE_LABEL}\` (сигнал downstream'у: e2e-rotation, clean-pr-sweep)." >/dev/null 2>&1 || true
                    fi
                    # Снимаем needs-review, поставленный без e2e (ретро 14.08 t_28afb585).
                    gh pr edit "$_spr_num" --repo "$GH_REPO" --remove-label "$NEEDS_REVIEW_LABEL" >/dev/null 2>&1 || true
                    # Ретро 31.08 t_04371252 (PR #1753): ставим явную метку для
                    # downstream-фильтров (e2e-process, clean-pr-sweep). whoami
                    # helper обеспечивает защиту от race с пользователем.
                    _spr_proc="$(pr_has_process_changes "$_spr_num")"
                    _spr_proc_msg="аддитивный функциональный PR на влитой ветке (ретро 14.08 t_28afb585)"
                    if [ "$_spr_proc" = "1" ]; then
                        _spr_proc_msg="${_spr_proc_msg} + меняет процессные скрипты scripts/agent_flow/ или tests/agent_flow/ (ретро 31.08 t_04371252)"
                    fi
                    whoami_add_label "$_spr_num" "$STALE_BRANCH_REUSE_LABEL" \
                        "${_spr_proc_msg}" \
                        "branch=${_spr_head}" "merged_via_pr=#${_spr_prev_merged}" || log "stale-branch-scan: WARNING add ${STALE_BRANCH_REUSE_LABEL} on PR #${_spr_num} failed (non-fatal)"
                else
                    log "stale-branch-scan: ветка ${_spr_head} влита через PR #${_spr_prev_merged}, но PR #${_spr_num} аддитивный docs/ci (del=${_spr_deletions:-0}) — НЕ регрессия, не блокируем (ретро 13.08 t_a3f170fe)"
                fi
            else
            log "stale-branch-scan: 🛑 ветка ${_spr_head} уже влита через PR #${_spr_prev_merged}, PR #${_spr_num} снова OPEN — block"
            if [ "$DRY_RUN" = "true" ]; then
                log "DRY-RUN would: comment stale-branch block on PR #${_spr_num}"
                continue
            fi
            _spr_dedup_since="$(date -u -d '24 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
            _spr_dup="$(gh api "repos/${GH_REPO}/issues/${_spr_num}/comments?since=${_spr_dedup_since}&per_page=100" \
                --jq '[.[] | select(.body | startswith("🛑 **stale-branch re-commit"))] | length' 2>/dev/null || echo 0)"
            if [ "${_spr_dup:-0}" -eq 0 ]; then
                gh pr comment "$_spr_num" --repo "$GH_REPO" --body \
                    "🛑 **stale-branch re-commit detected** (merge-gate, ретро 12.08 t_d3aeaa9b)

Ветка \`${_spr_head}\` уже была влита в develop через PR #${_spr_prev_merged}. Новые коммиты в неё ПОСЛЕ merge — re-коммиты поверх устаревшей базы: diff origin/develop...HEAD **удаляет** уже влитые фиксы.

**Что делать:**
1. НЕ пушить в уже влитую ветку.
2. Создай **новую** ветку от свежего origin/develop и открой новый PR.
3. Закрой/удали этот PR (ветка влита, PR-дифф регрессионный).

Merge-gate **не поставит needs-e2e** на PR с уже влитой ветки." >/dev/null 2>&1 || true
            fi
            fi
            fi
        fi
    done
    return 0
}

# --- stale-CONFLICTING watcher (ретро 24.08 t_cd32788f) ---------------------
# Сценарий: PR с needs-e2e висит в CONFLICTING >24ч (PR #1567, #1565 на
# 24.08). Merge-gate уже пишет rebase reminder с rate-limit 2ч (стр. ~1947),
# но Шифу не получает ЭСКАЛАЦИИ когда PR залипает дольше суток — карточка
# воркера блокируется PR-ом который никто не перебазил, а e2e-rotation
# каждый тик пытается merge и падает на CONFLICTING (silent noise).
#
# Решение: каждый тик merge-gate сканирует needs-e2e PR, для CONFLICTING:
#   - updatedAt > 24h  → label `stale-conflicting` + comment с rebase-инструкцией
#     (rate-limit 24ч на коммент, чтобы не спамить);
#   - updatedAt > 48h  → дополнительно упомянуть @krikz в карточке воркера
#     (если карточка живая) или в новом recovery-комменте issue.
#
# Эскалация к @krikz — это сигнал «карточка stale, нужен ручной rebase или
# close». Не пытаемся делать auto-rebase в чужую ветку намеренно (Шифу
# прямо: «не плодить коммиты в чужие ветки»). Auto-rebase для hotfix был
# PR #1248 — escape-hatch, но для stale-CONFLICTING mass-scenario это
# overkill и опасно.
#
# Метка `stale-conflicting` сигнализирует e2e-process'у: skip round с
# log-причиной «skip CONFLICTING PR #NNNN» (см. agent-flow-e2e-process.sh,
# pre-round guard). Это разрывает цикл «merge → CONFLICTING → recovery
# card → никто не ребейзит → repeat через 5 мин».
#
# Идемпотентность: если label уже стоит и 24ч не прошло — skip. Если
# PR перестал быть CONFLICTING (rebase прошёл, push дошёл) — label
# снимается здесь же (cleanup-блок ниже), и e2e-process берёт его в
# ротацию.
#
# Вызывается рядом со stale_branch_scan_all (основной путь + no-issues путь).
STALE_CONFLICTING_LABEL="${STALE_CONFLICTING_LABEL:-stale-conflicting}"
STALE_CONFLICTING_HOURS="${STALE_CONFLICTING_HOURS:-24}"
STALE_CONFLICTING_ESCALATE_HOURS="${STALE_CONFLICTING_ESCALATE_HOURS:-48}"
stale_conflicting_scan_all() {
    local now_epoch
    now_epoch="$(date +%s)"
    local cutoff_epoch_24h
    cutoff_epoch_24h="$((now_epoch - STALE_CONFLICTING_HOURS * 3600))"
    local cutoff_epoch_48h
    cutoff_epoch_48h="$((now_epoch - STALE_CONFLICTING_ESCALATE_HOURS * 3600))"
    local cutoff_iso_24h
    cutoff_iso_24h="$(date -u -d "@${cutoff_epoch_24h}" +%Y-%m-%dT%H:%M:%SZ 2>/dev/null \
        || date -u +%Y-%m-%dT%H:%M:%SZ)"
    local cutoff_iso_48h
    cutoff_iso_48h="$(date -u -d "@${cutoff_epoch_48h}" +%Y-%m-%dT%H:%M:%SZ 2>/dev/null \
        || date -u +%Y-%m-%dT%H:%M:%SZ)"

    # Берём open PR с needs-e2e + mergeable + updatedAt + headRefName + issue body
    # для поиска связанной kanban-карточки (для @krikz-escalation).
    local _sc_prs
    _sc_prs="$(gh pr list --repo "$GH_REPO" --state open --label "$NEEDS_E2E_LABEL" \
        --json number,headRefName,mergeable,updatedAt,title,labels 2>/dev/null || echo '[]')"
    if [ -z "$_sc_prs" ] || [ "$_sc_prs" = "[]" ]; then
        log "stale-conflicting-scan: no needs-e2e PRs — nothing to do"
        return 0
    fi

    # Идемпотентный комментарий (24ч window, как в stale-branch-scan).
    local _sc_dedup_since
    _sc_dedup_since="$(date -u -d '24 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null \
        || date -u +%Y-%m-%dT%H:%M:%SZ)"

    # Разбор JSON в tsv (PR_num, headRef, mergeable, updatedAtISO, has_label_already).
    printf '%s' "$_sc_prs" | python3 -c '
import json, sys, subprocess
GH = sys.argv[1]
LABEL = sys.argv[2]
data = json.load(sys.stdin)
for pr in data:
    pr_num = pr["number"]
    head = pr.get("headRefName","")
    mergeable = pr.get("mergeable","") or ""
    upd = pr.get("updatedAt","") or ""
    labels = [l["name"] for l in pr.get("labels", [])]
    has_sc = "yes" if LABEL in labels else "no"
    print("{}\t{}\t{}\t{}\t{}".format(pr_num, head, mergeable, upd, has_sc))
' "$GH_REPO" "$STALE_CONFLICTING_LABEL" 2>/dev/null \
    | while IFS=$'\t' read -r _sc_pr_num _sc_head _sc_mergeable _sc_updated_at _sc_has_label; do
        [ -z "$_sc_pr_num" ] && continue

        # CLEAN-блок: PR стал MERGEABLE, но label stale-conflicting ещё висит
        # → cleanup (e2e-process берёт PR в ротацию). Cleanup важен, чтобы
        # метка не залипала после успешного rebase.
        if [ "$_sc_mergeable" != "CONFLICTING" ]; then
            if [ "$_sc_has_label" = "yes" ]; then
                if [ "$DRY_RUN" = "true" ]; then
                    log "stale-conflicting-scan: PR #${_sc_pr_num} mergeable=${_sc_mergeable}, has ${STALE_CONFLICTING_LABEL} — DRY-RUN would remove label"
                else
                    gh pr edit "$_sc_pr_num" --repo "$GH_REPO" \
                        --remove-label "$STALE_CONFLICTING_LABEL" >/dev/null 2>&1 \
                        && log "stale-conflicting-scan: PR #${_sc_pr_num} mergeable=${_sc_mergeable} — removed ${STALE_CONFLICTING_LABEL} (cleanup, rebase прошёл)" \
                        || log "stale-conflicting-scan: WARNING remove ${STALE_CONFLICTING_LABEL} on PR #${_sc_pr_num} failed (non-fatal)"
                fi
            else
                log "stale-conflicting-scan: PR #${_sc_pr_num} mergeable=${_sc_mergeable} — ok, не CONFLICTING"
            fi
            continue
        fi

        # CONFLICTING. Проверяем updatedAt: если < 24ч — рано эскалировать,
        # rebase reminder ещё не отработал. Skip (merge-gate основной цикл
        # и так пишет ему reminder каждые 2ч).
        if [ -z "$_sc_updated_at" ]; then
            log "stale-conflicting-scan: PR #${_sc_pr_num} updatedAt пуст — skip (fresh)"
            continue
        fi
        local _sc_updated_epoch
        _sc_updated_epoch="$(date -u -d "$_sc_updated_at" +%s 2>/dev/null || echo 0)"
        if [ "$_sc_updated_epoch" -le 0 ] || [ "$_sc_updated_epoch" -gt "$now_epoch" ]; then
            log "stale-conflicting-scan: PR #${_sc_pr_num} updatedAt='${_sc_updated_at}' — не парсится или из будущего, skip"
            continue
        fi
        local _sc_age_hours
        _sc_age_hours="$(( (now_epoch - _sc_updated_epoch) / 3600 ))"

        # < 24ч — рано. Merge-gate rebase reminder (стр. ~1947) ещё активен.
        if [ "$_sc_updated_epoch" -gt "$cutoff_epoch_24h" ]; then
            log "stale-conflicting-scan: PR #${_sc_pr_num} CONFLICTING ${_sc_age_hours}ч (< ${STALE_CONFLICTING_HOURS}ч) — рано, rebase reminder активен"
            continue
        fi

        # >= 24ч. Если label ещё нет — ставим + пишем comment (24ч dedup).
        if [ "$_sc_has_label" = "no" ]; then
            if [ "$DRY_RUN" = "true" ]; then
                log "DRY-RUN would: add ${STALE_CONFLICTING_LABEL} on PR #${_sc_pr_num} + comment stale > ${STALE_CONFLICTING_HOURS}ч"
                continue
            fi
            local _sc_dup
            _sc_dup="$(gh api "repos/${GH_REPO}/issues/${_sc_pr_num}/comments?since=${_sc_dedup_since}&per_page=100" \
                --jq '[.[] | select(.body | startswith("🟠 **stale-CONFLICTING"))] | length' 2>/dev/null \
                || echo 0)"
            if [ "${_sc_dup:-0}" -eq 0 ]; then
                gh pr comment "$_sc_pr_num" --repo "$GH_REPO" --body \
"🟠 **stale-CONFLICTING: rebase на develop > ${STALE_CONFLICTING_HOURS}ч (merge-gate, ретро 24.08 t_cd32788f)**

PR #${_sc_pr_num} (\\\`${_sc_head}\\\`) → develop = **CONFLICTING** уже ${_sc_age_hours}ч. Develop убежал вперёд (last updatedAt = ${_sc_updated_at}), стандартный rebase-reminder (стр. ~1947) не помог.

**Что делать** (по процессу Шифу 10.08):
1. **В той же ветке** \\\`${_sc_head}\\\` — НЕ создавай новую ветку и НЕ новый PR.
2. **rebase** на origin/develop:
   \\\`\\\`\\\`bash
   git fetch origin develop
   git checkout ${_sc_head}
   git rebase origin/develop
   # ... resolve conflicts ...
   git add -A && git rebase --continue
   git push --force-with-lease origin ${_sc_head}
   \\\`\\\`\\\`
3. Метку \\\`${STALE_CONFLICTING_LABEL}\\\` снимать НЕ надо — merge-gate снимет её автоматически когда PR станет MERGEABLE.

**Если rebase не помогает** (конфликт в коде, который ты не можешь разрешить) → оставь PR как есть, явно закрой и открой новый PR с новой ветки \\\`z-{agent}/<new-id>-<slug>\\\` от свежего develop. Иначе PR висит вечно.

Если карточка старше ${STALE_CONFLICTING_ESCALATE_HOURS}ч — merge-gate пингом упомянет \\\`@krikz\\\`.
" >/dev/null 2>&1 || true
                log "stale-conflicting-scan: PR #${_sc_pr_num} CONFLICTING ${_sc_age_hours}ч — comment «stale-CONFLICTING» добавлен"
            else
                log "stale-conflicting-scan: PR #${_sc_pr_num} CONFLICTING ${_sc_age_hours}ч — comment dedup (есть < 24ч), skip comment"
            fi
            whoami_add_label "$_sc_pr_num" "$STALE_CONFLICTING_LABEL" \
                "stale-CONFLICTING (merge-gate, ретро 24.08 t_cd32788f): CONFLICTING > ${STALE_CONFLICTING_HOURS}ч (age=${_sc_age_hours}ч)" \
                "pr=${_sc_pr_num}" || log "stale-conflicting-scan: WARNING add ${STALE_CONFLICTING_LABEL} on PR #${_sc_pr_num} failed"
        fi

        # >= 48ч — эскалация к @krikz (Шифу). Ищем задачу в карточках через
        # wt/<branch> → t_<id>. Не блокируем CI/процесс, только пинг.
        if [ "$_sc_updated_epoch" -gt "$cutoff_epoch_48h" ]; then
            log "stale-conflicting-scan: PR #${_sc_pr_num} CONFLICTING ${_sc_age_hours}ч (< ${STALE_CONFLICTING_ESCALATE_HOURS}ч) — без escalation"
            continue
        fi
        # >= 48ч: escalation.
        local _sc_task_id
        _sc_task_id=""
        if [[ "$_sc_head" =~ ^wt/(t_[a-f0-9]+)- ]]; then
            _sc_task_id="${BASH_REMATCH[1]}"
        fi
        # Ищем issue-number в PR body (Closes #N / Refs #N) → достаём kanban task.
        local _sc_issue_num
        _sc_issue_num="$(gh pr view "$_sc_pr_num" --repo "$GH_REPO" --json body \
            --jq '[scan("(?im)^(?:closes|fixes|resolves|refs|part of)\\s+#?(\\d+)\\b"; .body)] | .[0] // empty' 2>/dev/null \
            || true)"
        if [ -z "$_sc_task_id" ] && [ -n "$_sc_issue_num" ]; then
            # Ретро 14.08 t_de6bea69: task_id из kanban-marker на issue body.
            _sc_task_id="$(gh issue view "$_sc_issue_num" --repo "$GH_REPO" --comments --json comments \
                --jq '.comments[].body' 2>/dev/null \
                | grep -Eo '^kanban: t_[a-f0-9]+' | tail -n1 | sed 's/^kanban: //' || true)"
        fi
        # Rate-limit escalation-comment (24ч).
        local _sc_escalate_dedup
        _sc_escalate_dedup="$(gh api "repos/${GH_REPO}/issues/${_sc_pr_num}/comments?since=${_sc_dedup_since}&per_page=100" \
            --jq '[.[] | select(.body | startswith("🟠 **stale-CONFLICTING ESCALATION"))] | length' 2>/dev/null \
            || echo 0)"
        if [ "${_sc_escalate_dedup:-0}" -gt 0 ]; then
            log "stale-conflicting-scan: PR #${_sc_pr_num} CONFLICTING ${_sc_age_hours}ч — escalation dedup (<24ч), skip"
            continue
        fi
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would: comment «ESCALATION @krikz» on PR #${_sc_pr_num} (task=${_sc_task_id:-none})"
            continue
        fi
        # Эскалация: пишем в PR (комментарий dedup-ится через prefix).
        gh pr comment "$_sc_pr_num" --repo "$GH_REPO" --body \
"🟠 **stale-CONFLICTING ESCALATION: @krikz пинг, PR CONFLICTING > ${STALE_CONFLICTING_ESCALATE_HOURS}ч (merge-gate, ретро 24.08 t_cd32788f)**

PR #${_sc_pr_num} (\\\`${_sc_head}\\\`) → develop = **CONFLICTING** уже ${_sc_age_hours}ч (last updatedAt = ${_sc_updated_at}). Label \\\`${STALE_CONFLICTING_LABEL}\\\` стоит, rebase reminder не помог, e2e-rotation skip-ает round каждый тик.

Это последний авто-сигнал перед ручным вмешательством. Дальше либо:
- **Шифу сам** делает rebase (force-with-lease) в \\\`${_sc_head}\\\`;
- или **закрывает** этот PR и просит воркера пересоздать PR с новой ветки \\\`z-{agent}/<new-id>-<slug>\\\` от свежего develop (НО не делает новую ветку внутри существующей, Шифу прямо: «не плодить»);
- или явно ставит \\\`needs-handoff\\\` если хочет передать задачу другому профилю.

Карточка воркера: \\\`${_sc_task_id:-none-detected}\\\`. Связанный issue: #${_sc_issue_num:-?}.
" >/dev/null 2>&1 || log "stale-conflicting-scan: WARNING escalation comment on PR #${_sc_pr_num} failed (non-fatal)"
        # Если нашли живую карточку воркера — допишем reminder с @krikz
        # (та же карточка должна знать, Шифу прямо).
        if [ -n "$_sc_task_id" ]; then
            local _sc_card_status
            _sc_card_status="$(kanban_card_status "$_sc_task_id" 2>/dev/null || echo "")"
            case "$_sc_card_status" in
                running|ready|todo)
                    local _sc_escalate_marker_ts
                    _sc_escalate_marker_ts="$(kanban_last_reminder_ts "$_sc_task_id" "STALE CONFLICTING > ${STALE_CONFLICTING_ESCALATE_HOURS}h" 2>/dev/null || echo "")"
                    local _sc_now_ts="${now_epoch:-$(date +%s)}"
                    if [ -n "$_sc_escalate_marker_ts" ] && [ $(( _sc_now_ts - _sc_escalate_marker_ts )) -lt 86400 ]; then
                        log "stale-conflicting-scan: PR #${_sc_pr_num} — escalation reminder dedup (< 24ч), skip card comment"
                    else
                        "$HERMES_BIN" kanban --board "$KANBAN_BOARD" comment "$_sc_task_id" \
"## 🟠 stale-CONFLICTING escalation @krikz (merge-gate, ретро 24.08 t_cd32788f, $(date -u +%H:%M:%SZ))

PR #${_sc_pr_num} (\\\`${_sc_head}\\\`) → develop = **CONFLICTING** уже ${_sc_age_hours}ч (last updatedAt = ${_sc_updated_at}). Label \\\`${STALE_CONFLICTING_LABEL}\\\` стоит, rebase reminder не помог.

**Рекомендация** (Шифу прямо: «та же карточка должна знать»):
- Если ребейзишь в той же ветке: \\\`git fetch origin develop && git rebase origin/develop && git push --force-with-lease origin ${_sc_head}\\\`.
- Если конфликт неразрешим: попроси закрыть PR и пересоздать с новой ветки (retро 13.08 t_a3f170fe — только не на той же влитой ветке).

e2e-rotation каждый тик skip-ает round с reason «stale-conflicting PR #${_sc_pr_num}». Метка \\\`${STALE_CONFLICTING_LABEL}\\\` снимется автоматически когда PR станет MERGEABLE.
" >/dev/null 2>&1 || log "stale-conflicting-scan: WARNING append escalation to card ${_sc_task_id} failed"
                        log "stale-conflicting-scan: PR #${_sc_pr_num} — escalation reminder written to card ${_sc_task_id} (status=${_sc_card_status})"
                    fi
                    ;;
                *)
                    log "stale-conflicting-scan: PR #${_sc_pr_num} — task ${_sc_task_id} status=${_sc_card_status:-?}, escalation только в PR-коммент"
                    ;;
            esac
        fi
    done
    return 0
}

# --- duplicate-file scan для ВСЕХ open PR (ретро 15.08 t_20383d32) ----------
# Сценарий: две ПАРАЛЛЕЛЬНЫЕ карточки пришли к одному корневому фиксу и каждая
# добавила ОДИН И ТОТ ЖЕ файл с ИДЕНТИЧНЫМ содержимым (одинаковый blob sha):
# PR #1262 (t_392d6000 setup.cfg) и PR #1267 (issue #1266 setup.cfg) — оба
# добавили src/rob_box_teleop/setup.cfg (blob 66dad822). Триаж/e2e не dedup-ит
# PR по изменяемым файлам → фикс задвоен, при merge первого второй получит
# add/add конфликт или пустой diff.
# Guard: тянем pulls/N/files (filename+sha) для open PR с needs-review/needs-e2e
# (кандидаты на ревью/мерж), ищем пару (filename, sha) в РАЗНЫХ PR →
# инфо-коммент на оба PR (dedup 24h). НЕ блокируем CI и НЕ снимаем needs-e2e:
# решение «какой влить, какой закрыть» — за Шифу при ревью.
# Вызывается рядом со stale_branch_scan_all (основной путь + no-issues путь).
duplicate_file_scan_all() {
    _dup_prs="$(gh pr list --repo "$GH_REPO" --state open \
        --json number,headRefName,labels 2>/dev/null || echo '[]')"
    # Собираем (filename, sha) -> [pr...] только для needs-review/needs-e2e PR.
    printf '%s' "$_dup_prs" | python3 -c '
import json, sys, subprocess
GH_REPO = sys.argv[1]
data = json.load(sys.stdin)
seen = {}
for pr in data:
    labels = {l.get("name","") for l in pr.get("labels", [])}
    if not ({"needs-review", "needs-e2e"} & labels):
        continue
    num = pr["number"]
    r = subprocess.run(
        ["gh", "api", f"repos/{GH_REPO}/pulls/{num}/files?per_page=100"],
        capture_output=True, text=True)
    try:
        files = json.loads(r.stdout or "[]")
    except Exception:
        files = []
    for f in files:
        fname = f.get("filename", "")
        sha = f.get("sha", "")
        if fname and sha:
            seen.setdefault((fname, sha), []).append(num)
# Печатаем дубли: (filename, sha) встречается в >=2 РАЗНЫХ PR.
for (fname, sha), prs in sorted(seen.items()):
    uniq = sorted(set(prs))
    if len(uniq) < 2:
        continue
    for i in range(len(uniq)):
        for j in range(i+1, len(uniq)):
            print(f"{fname}\t{sha}\t{uniq[i]}\t{uniq[j]}")
' "$GH_REPO" 2>/dev/null | while IFS=$'\t' read -r _df _ds _dp1 _dp2; do
        [ -z "$_df" ] && continue
        log "duplicate-file-scan: файл ${_df} (blob ${_ds}) в PR #${_dp1} и PR #${_dp2} — ИДЕНТИЧНЫЙ контент (ретро 15.08 t_20383d32)"
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would: duplicate-file comment on PR #${_dp1} и PR #${_dp2}"
            continue
        fi
        _dd_since="$(date -u -d '24 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
        for _pr in "$_dp1" "$_dp2"; do
            _other="$([ "$_pr" = "$_dp1" ] && echo "$_dp2" || echo "$_dp1")"
            _dup_cnt="$(gh api "repos/${GH_REPO}/issues/${_pr}/comments?since=${_dd_since}&per_page=100" \
                --jq '[.[] | select(.body | contains("duplicate file detected"))] | length' 2>/dev/null || echo 0)"
            if [ "${_dup_cnt:-0}" -eq 0 ]; then
                gh pr comment "$_pr" --repo "$GH_REPO" --body \
                    "⚠️ **duplicate file detected** (merge-gate, ретро 15.08 t_20383d32)

Файл \`${_df}\` (blob \`${_ds}\`) уже изменён в открытом PR #${_other} с ИДЕНТИЧНЫМ содержимым — фикс задвоен двумя независимыми карточками.

**Что делать (при ревью Шифу):** влейте ОДИН из PR (обычно более широкий — с доп. фиксами), второй закройте как дубль или rebase на develop после merge первого. Merge-gate **НЕ блокирует** CI/e2e — это информационное предупреждение." >/dev/null 2>&1 || true
            fi
        done
    done
    return 0
}

# --- PR-without-kanban-marker scan (ретро 25.08 t_1a4f3275 / issue #1624) ----
# Сценарий: воркер открыл PR с process-метками (agent-flow-error / needs-e2e /
# needs-review) в обход процесса — через `gh api` напрямую, без kanban-marker
# в issue body и без kanban-карточки. Основной цикл merge-gate (выше) ходит
# по issue с меткой `hermes` → ищет kanban-marker → строит каноническую
# ветку `z-{agent}/<issue>-<slug>` → не находит PR (ветка отличается) → skip.
# Итог: PR висит в CONFLICTING 5-8ч без внимания процесса, Шифу видит
# «зомби-PR» без kanban-card.
#
# Кейсы:
#   - PR #1623 / #1611 (25.08.2026): architect-worker открыл напрямую,
#     base=develop (пример), нет kanban-marker в issue #1600/#1597. CONFLICTING
#     висит 5-8ч.
#   - PR с label `agent-flow` / `agent-flow-error` / `needs-e2e` /
#     `needs-review` но без marker'а — должен попасть под этот guard.
#
# Guard: для ВСЕХ open PR с хотя бы одной process-меткой
# (`agent-flow-error`, `needs-e2e`, `needs-review`) определяем связанный issue
# (по #NNNN в title или z-{agent}/NNNN-* в head), проверяем kanban-marker в
# issue comments (тот же regex, что основной цикл). Если маркера нет —
# комментарий-предупреждение (dedup 24h) на PR + на issue, чтобы процесс
# увидел orphan'а. НЕ блокируем CI/e2e (alert, не gate) — решение за
# шисюном (architect-worker) и Шифу.
#
# Идемпотентность: проверка 24h dedup на substring тела комментария, как у
# duplicate-file-scan.
#
# Этот guard НЕ ловит PR без process-меток (они вне процесса — вне scope).
# Для них возможен future-work: alert если PR из z-{agent}/* ветки без
# issue-marker'а вообще.
pr_without_marker_scan_all() {
    _wm_prs="$(gh pr list --repo "$GH_REPO" --state open \
        --json number,title,headRefName,labels 2>/dev/null || echo '[]')"
    printf '%s' "$_wm_prs" | python3 -c '
import json, sys, subprocess, re
GH_REPO = sys.argv[1]
try:
    data = json.load(sys.stdin)
except Exception:
    sys.exit(1)
for pr in data:
    labels = {l.get("name","") for l in pr.get("labels", [])}
    # Только PR с process-метками — иначе шумим на каждый black-label PR.
    if not ({"agent-flow","agent-flow-error","needs-e2e","needs-review"} & labels):
        continue
    num = pr["number"]
    title = pr.get("title","") or ""
    head = pr.get("headRefName","") or ""
    # Определяем issue_number: сначала из title (#NNNN), иначе из branch
    # z-{agent}/NNNN-... (НЕ t_<id>-..., это task_id).
    m = re.search(r"#(\d+)", title)
    issue_num = m.group(1) if m else ""
    if not issue_num:
        m2 = re.search(r"z-\{agent\}/(\d+)-", head)
        issue_num = m2.group(1) if m2 else ""
    if not issue_num:
        # Не можем сопоставить PR ↔ issue → пропускаем (нет точки приложения).
        continue
    # Достаём комментарии issue и ищем kanban-marker (любой t_xxxxxxxxxxxxx).
    r = subprocess.run(
        ["gh","api",f"repos/{GH_REPO}/issues/{issue_num}/comments?per_page=100"],
        capture_output=True, text=True)
    try:
        comments = json.loads(r.stdout or "[]")
    except Exception:
        comments = []
    has_marker = any(
        re.search(r"^kanban: t_[a-f0-9]+", c.get("body","") or "", re.M)
        for c in comments
    )
    if has_marker:
        continue
    # Без marker (кандидат на alert). Печатаем: pr_num<TAB>issue_num для bash.
    print(f"{num}\t{issue_num}")
' "$GH_REPO" 2>/dev/null | while IFS=$'\t' read -r _wm_pr _wm_issue; do
        [ -z "$_wm_pr" ] && continue
        log "pr-without-marker-scan: PR #${_wm_pr} имеет process-метку, но kanban-marker в issue #${_wm_issue} ОТСУТСТВУЕТ (ретро 25.08 t_1a4f3275 / issue #1624)"
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would: process-marker-missing comment on PR #${_wm_pr} и issue #${_wm_issue}"
            continue
        fi
        _wm_since="$(date -u -d '24 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
        _wm_head="$(gh pr view "$_wm_pr" --repo "$GH_REPO" --json headRefName --jq '.headRefName' 2>/dev/null || echo "?")"
        _wm_base="$(gh pr view "$_wm_pr" --repo "$GH_REPO" --json baseRefName --jq '.baseRefName' 2>/dev/null || echo "?")"
        _wm_state="$(gh pr view "$_wm_pr" --repo "$GH_REPO" --json mergeStateStatus --jq '.mergeStateStatus' 2>/dev/null || echo "?")"
        # 24h dedup на substring тела (как у duplicate-file-scan).
        _wm_dup_pr="$(gh api "repos/${GH_REPO}/issues/${_wm_pr}/comments?since=${_wm_since}&per_page=100" \
            --jq '[.[] | select(.body | contains("process marker missing"))] | length' 2>/dev/null || echo 0)"
        if [ "${_wm_dup_pr:-0}" -eq 0 ]; then
            gh pr comment "$_wm_pr" --repo "$GH_REPO" --body \
                "⚠️ **process marker missing** (merge-gate, ретро 25.08 t_1a4f3275 / issue #1624)

PR имеет process-метку (agent-flow* / needs-e2e / needs-review), но в issue #${_wm_issue} НЕТ kanban-marker (\`kanban: t_xxxxxxxxxxxxx\`). Без маркера merge-gate основного цикла не может сопоставить PR ↔ kanban-карточку и пропускает обработку — PR висит вне процесса.

**Текущее состояние:**
- PR: #${_wm_pr}
- head: \`${_wm_head}\`
- base: \`${_wm_base}\`
- mergeStateStatus: \`${_wm_state}\`
- issue: #${_wm_issue}

**Что делать (приоритет для шисюна/Шифу):**
1. Связать PR ↔ kanban-карточку: добавить kanban-marker в issue #${_wm_issue} (комментарий \`kanban: t_xxxxxxxxxxxxx branch: ${_wm_head} role: <role>\`), затем \`hermes kanban --board robbox complete t_xxxxxxxxxxxxx\` с raw-evidence и тестами.
2. Либо закрыть этот PR (если архитектура не предполагает его merge) и переоткрыть из новой kanban-карточки.
3. Если PR нужен (например AV-6 / AV-3, base=develop) — добавить kanban-card через \`hermes kanban create --assignee <agent>\` и привязать.

Merge-gate **НЕ блокирует** CI/e2e (alert, не gate). Решение за человеком (Шифу / шисюн)." >/dev/null 2>&1 || true
        fi
        # Параллельно комментим issue (если issue существует и не duplicate).
        if [ -n "$_wm_issue" ] && [ "$_wm_issue" != "?" ]; then
            _wm_dup_iss="$(gh api "repos/${GH_REPO}/issues/${_wm_issue}/comments?since=${_wm_since}&per_page=100" \
                --jq '[.[] | select(.body | contains("process marker missing on PR"))] | length' 2>/dev/null || echo 0)"
            if [ "${_wm_dup_iss:-0}" -eq 0 ]; then
                gh issue comment "$_wm_issue" --repo "$GH_REPO" --body \
                    "⚠️ **process marker missing on PR** (merge-gate, ретро 25.08 t_1a4f3275 / issue #1624)

PR #${_wm_pr} (\`${_wm_head}\` → \`${_wm_base}\`, state=${_wm_state}) имеет process-метку, но в этом issue НЕТ kanban-marker. PR вне процесса — merge-gate основного цикла пропустит.

См. подробности в комменте PR: https://github.com/${GH_REPO}/pull/${_wm_pr}" >/dev/null 2>&1 || true
            fi
        fi
    done
    return 0
}

# gh_list_issues_by_label — перенесена в lib_agent_flow_common.sh (дедуп 30.08).

# --- deploy-issue label-less orphan backstop (ретро 15.08 t_238ff3f7) -------
# Сценарий: L-Deploy and Verify создаёт deploy-issues с версией workflow-файла
# С ВЕТКИ e2e-раунда (z-{e2e}/test-round-N), а не develop. Если round-ветка
# ответвилась ДО фикса #1263 (hermes+agent:devops при создании), issue получает
# только метку `deployment` → агентский триаж (фильтр по hermes) карточку не
# создаёт → issue висит open навсегда без обработчика (#1276, round-116;
# #1277 round-117 уже с метками → карточка t_dcae2e1a создана).
# Реконсилейшн: open deployment-issue без process-меток (hermes/agent:*/...)
# старше DEPLOY_RECONCILE_MINUTES (default 30м) → добавить hermes + agent:devops
# → триаж на следующем тике создаст kanban-карточку (как #1277).
# Idempotent: после добавления hermes issue больше не подпадает под правило.
# Вызывается рядом со stale_branch_scan_all (основной путь + no-issues путь).
deploy_issue_reconcile_all() {
    local _dep_json
    # Ретро 19.08 #1457: gh issue list --label ломает фильтр → fallback через
    # gh_list_issues_by_label (REST API), если gh-list пустой.
    _dep_json="$(gh_list_issues_by_label deployment open 100)"
    if [ -z "$_dep_json" ] || [ "$_dep_json" = "[]" ]; then
        log "deploy-issue-reconcile: no open deployment issues — skip"
        return 0
    fi
    printf '%s' "$_dep_json" | python3 -c '
import json, sys, base64
d = json.load(sys.stdin)
for i in d:
    labels = ",".join(sorted({lab["name"] for lab in i.get("labels", [])}))
    print(str(i["number"]) + "\t" + base64.b64encode(str(i["title"]).encode("utf-8")).decode("ascii") + "\t" + base64.b64encode(labels.encode("utf-8")).decode("ascii") + "\t" + str(i["updatedAt"]))
' 2>/dev/null | while IFS=$'\t' read -r _dep_num _dep_title_b64 _dep_labels_b64 _dep_upd; do
        [ -z "$_dep_num" ] && continue
        _dep_title="$(printf '%s' "$_dep_title_b64" | base64 -d 2>/dev/null || true)"
        _dep_labels="$(printf '%s' "$_dep_labels_b64" | base64 -d 2>/dev/null || true)"
        _dep_labels_norm="$(printf '%s' "$_dep_labels" | tr '[:upper:]' '[:lower:]')"
        # Пропускаем уже размеченные (hermes/agent:*/needs-*).
        if printf '%s' "$_dep_labels_norm" | tr ',' '\n' | grep -Eq '^(hermes|needs-e2e|e2e-done|e2e:rejected|no-e2e-required|needs-review|needs-discussion|big-bang-override)$|^agent:'; then
            log "deploy-issue-reconcile: #${_dep_num} уже в process-цикле (${_dep_labels}) — skip"
            continue
        fi
        # Пропускаем свежие (< DEPLOY_RECONCILE_MINUTES): workflow раунда может
        # ещё дописывать/метить issue сам.
        _dep_upd_epoch="$(date -d "$_dep_upd" +%s 2>/dev/null || echo 0)"
        _dep_age_min=$(( ($(date +%s) - _dep_upd_epoch) / 60 ))
        if [ "$_dep_age_min" -lt "$DEPLOY_RECONCILE_MINUTES" ]; then
            log "deploy-issue-reconcile: #${_dep_num} возраст ${_dep_age_min}м < ${DEPLOY_RECONCILE_MINUTES}м — fresh, skip"
            continue
        fi
        log "deploy-issue-reconcile: #${_dep_num} (${_dep_title:0:50}) без hermes-метки ${_dep_age_min}м — добавляю hermes+agent:devops"
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would: add hermes+agent:devops to #${_dep_num}"
            continue
        fi
        gh issue edit "$_dep_num" --repo "$GH_REPO" --add-label hermes >/dev/null 2>&1 || true
        gh issue edit "$_dep_num" --repo "$GH_REPO" --add-label agent:devops >/dev/null 2>&1 || true
        # Коммент с дедупликацией (24h) — не спамим каждый тик.
        _dep_since="$(date -u -d '24 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
        _dep_dup="$(gh api "repos/${GH_REPO}/issues/${_dep_num}/comments?since=${_dep_since}&per_page=100" \
            --jq '[.[] | select(.body | startswith("🏷️ **Авто-reconcile**"))] | length' 2>/dev/null || echo 0)"
        if [ "${_dep_dup:-0}" -eq 0 ]; then
            gh issue comment "$_dep_num" --repo "$GH_REPO" --body \
                "🏷️ **Авто-reconcile** (merge-gate, ретро 15.08 t_238ff3f7): deployment-issue без hermes-метки > ${DEPLOY_RECONCILE_MINUTES}м — проставлены \\\`hermes\\\` + \\\`agent:devops\\\`; триаж создаст kanban-карточку (backstop для label-less deploy-issues, #1276)." >/dev/null 2>&1 || true
        fi
    done
    return 0
}
# --- stale-after-upstream-fix detector (ретро 31.08 t_9d375e3e / ADR-0035) --
# Diagnostic-карточки (PR #1743, retro t_e00f448d) создаются при CI UNSTABLE
# с classification=unit_lint. Без auto-detect они "вечно живые" после
# upstream-фикса (PR влит / upstream залил фикс в develop / фикс уже в
# самом PR). Этот scan каждый тик merge-gate:
#   1. Берёт все live diagnostic-карточки (status != done/archived) с
#      маркерами `<!-- diag-pr: N -->` в body.
#   2. Парсит маркеры (PR, head SHA, sig, tests, classification, created-ts).
#   3. Вызывает detect_stale_after_upstream_fix() (pure, без побочных
#      эффектов) — возвращает структуру {stale, upstream_sha, strategy,
#      reason, evidence_diff}. Применяются 3 стратегии детекта:
#      A. PR head SHA --is-ancestor origin/<base> (PR уже слит в develop).
#      B. git log origin/<base> -S <attr> (фикс атрибута в develop после
#         создания карточки) — основной кейс t_5c524b12.
#      C. Все failing-tests файлы уже в PR-diff + CI SUCCESS (фикс в
#         самом PR, ещё не слит, но уже зелёный).
#      Fallback: REST compare identical (стратегия A без REPO_DIR).
#   4. При наличии upstream-фикса → orchestrator применяет auto-block +
#      comment patch + rate-limit. Этот orchestrator НЕ вызывает auto-block
#      из detector'а — разделение для тестируемости (task #1 делает block,
#      detector делает только detect; см. ADR-0035 §5.2).
#   5. Rate-limit: один auto-block на карточку в
#      STALE_AFTER_UPSTREAM_FIX_COOLDOWN_SECONDS секунд (default 2ч).
#   6. Legacy diagnostic без маркеров → skip (не ломаем старые карточки).

# detect_stale_after_upstream_fix — pure detector (без побочных эффектов).
# Входные данные (все позиционные, никаких глобалов не модифицирует):
#   $1 = card_id
#   $2 = pr_num
#   $3 = pr_sha (head SHA из diag-pr-sha)
#   $4 = pr_base (из diag-pr-base, default $DEVELOP_BRANCH)
#   $5 = sig_csv (comma-separated attrs из diag-sig)
#   $6 = tests_csv (comma-separated файлов из diag-tests)
#   $7 = created_ts (epoch из diag-created-ts; 0 если неизвестно)
#   $8 = repo_dir (default = $REPO_DIR; пусто → REST fallback)
#   $9 = gh_repo (default = $GH_REPO)
# Выход: stdout TSV (5 полей), return 0:
#   field 1: stale (true|false)
#   field 2: upstream_sha (short SHA, или "" если strat C)
#   field 3: strategy (closed|A|B|C|rest_fallback|none)
#   field 4: reason (человекочитаемая строка для kanban block)
#   field 5: evidence_diff (multi-line git log output для комментария)
# Если карточка не stale — печатает "false\t\t\tnone\t\t" (пустые поля).
# Это pure-функция: НЕ вызывает `hermes kanban block`, НЕ пишет в journal,
# НЕ логирует через `log` — только читает (git/gh) и возвращает TSV. Это
# позволяет unit-тесту вызывать её напрямую с моками и assert'ить результат.
detect_stale_after_upstream_fix() {
    local card_id="$1" pr_num="$2" pr_sha="$3" pr_base="$4"
    local sig_csv="$5" tests_csv="$6" created_ts="$7"
    local repo_dir="${8:-${REPO_DIR:-}}"
    local gh_repo="${9:-${GH_REPO:-krikz/rob_box_project}}"
    local upstream_sha="" strategy="" reason="" evidence="" hit attr test_file _a _b _s

    # Стратегия 0: PR CLOSED → fast skip (не нужен git, нужен gh pr view).
    # Вызываем ДО остальных, потому что стратегии A/B/C не работают для
    # закрытого PR (head SHA больше не ancestor of develop если PR закрыт
    # не merge'ом).
    if [ -n "$pr_num" ] && [ "$(pr_state_now "$pr_num")" = "CLOSED" ]; then
        printf '%s\t%s\t%s\t%s\t%s\n' \
            "true" "" "closed" \
            "stale-after-upstream-fix: PR #${pr_num} CLOSED (починка upstream или неактуален, ретро t_9d375e3e / ADR-0035)" \
            ""
        return 0
    fi

    # Стратегия A: PR head SHA ancestor of origin/<base> (git merge-base).
    if [ -n "$pr_sha" ] && [ -n "$repo_dir" ] && [ -d "$repo_dir" ]; then
        if git -C "$repo_dir" merge-base --is-ancestor "$pr_sha" "origin/${pr_base}" 2>/dev/null; then
            printf '%s\t%s\t%s\t%s\t%s\n' \
                "true" "$pr_sha" "A" \
                "stale-after-upstream-fix: PR #${pr_num} head ${pr_sha:0:8} уже в origin/${pr_base} (ретро t_9d375e3e / ADR-0035)" \
                ""
            return 0
        fi
    fi

    # Стратегия B: upstream-фикс по сигнатуре / failing-tests (git log -S).
    if [ -n "$repo_dir" ] && [ -d "$repo_dir" ]; then
        upstream_sha=""
        # B-attr: ищем коммит, добавивший/удаливший атрибут сигнатуры.
        if [ -n "$sig_csv" ]; then
            while IFS=',' read -r attr; do
                [ -z "$attr" ] && continue
                attr="$(printf '%s' "$attr" | sed 's/^[[:space:]]*//;s/[[:space:]]*$//' || echo "")"
                [ -z "$attr" ] && continue
                hit="$(git -C "$repo_dir" log "origin/${pr_base}" \
                    --since="@${created_ts:-0}" -S "$attr" \
                    --pretty=format:'%H' 2>/dev/null | head -1 || echo "")"
                if [ -n "$hit" ]; then
                    upstream_sha="$hit"
                    evidence="git log origin/${pr_base} --since=@${created_ts:-0} -S '${attr}' → ${hit:0:8}"
                    strategy="B-attr:$attr"
                    break
                fi
            done < <(printf '%s\n' "$sig_csv" | tr ',' '\n')
        fi
        # B-tests: ищем коммит, изменивший failing-test файл.
        if [ -z "$upstream_sha" ] && [ -n "$tests_csv" ]; then
            while IFS=',' read -r test_file; do
                [ -z "$test_file" ] && continue
                test_file="$(printf '%s' "$test_file" | sed 's/^[[:space:]]*//;s/[[:space:]]*$//' || echo "")"
                [ -z "$test_file" ] && continue
                hit="$(git -C "$repo_dir" log "origin/${pr_base}" \
                    --since="@${created_ts:-0}" -- "$test_file" \
                    --pretty=format:'%H' 2>/dev/null | head -1 || echo "")"
                if [ -n "$hit" ]; then
                    upstream_sha="$hit"
                    evidence="git log origin/${pr_base} --since=@${created_ts:-0} -- '${test_file}' → ${hit:0:8}"
                    strategy="B-tests:$test_file"
                    break
                fi
            done < <(printf '%s\n' "$tests_csv" | tr ',' '\n')
        fi

        if [ -n "$upstream_sha" ]; then
            reason="stale-after-upstream-fix: upstream-фикс ${upstream_sha:0:8} уже в origin/${pr_base} (после создания карточки, ретро t_9d375e3e / ADR-0035) [strat=${strategy}]"
            printf '%s\t%s\t%s\t%s\t%s\n' \
                "true" "$upstream_sha" "B" \
                "$reason" "$evidence"
            return 0
        fi
    fi

    # Стратегия C: фикс в самом PR + CI SUCCESS.
    if [ -n "$tests_csv" ] && [ -n "$pr_num" ]; then
        local pr_files pr_checks_ok all_in_pr
        pr_files="$(gh pr view "$pr_num" --repo "$gh_repo" --json files --jq '[.files[].path]' 2>/dev/null || echo '[]')"
        pr_checks_ok="$(gh pr checks "$pr_num" --repo "$gh_repo" --json state --jq '[.[] | select(.state != "SUCCESS")] | length' 2>/dev/null || echo 999)"
        if [ "${pr_checks_ok:-999}" = "0" ]; then
            all_in_pr=1
            while IFS=',' read -r test_file; do
                [ -z "$test_file" ] && continue
                test_file="$(printf '%s' "$test_file" | sed 's/^[[:space:]]*//;s/[[:space:]]*$//' || echo "")"
                [ -z "$test_file" ] && continue
                if ! printf '%s' "$pr_files" | grep -qF "$test_file"; then
                    all_in_pr=0
                    break
                fi
            done < <(printf '%s\n' "$tests_csv" | tr ',' '\n')
            if [ "$all_in_pr" = "1" ]; then
                reason="stale-after-upstream-fix: фикс уже в самом PR #${pr_num} (failing-tests файлы в PR-diff + CI SUCCESS, ждать merge в develop, ретро t_9d375e3e / ADR-0035)"
                evidence="PR #${pr_num} files contain all failing-tests; CI SUCCESS"
                printf '%s\t%s\t%s\t%s\t%s\n' \
                    "true" "" "C" "$reason" "$evidence"
                return 0
            fi
        fi
    fi

    # REST compare fallback (если REPO_DIR пуст / стратегии A/B/C не дали
    # результата). Использует `gh pr view <n> --json mergedAt` — самый
    # прямой признак "PR уже влит в base". Если mergedAt != null → stale.
    # Раньше здесь был `gh api repos/.../compare/<base>...<pr_sha>` —
    # убран в пользу pr view mergedAt: тот же семантический ответ ("PR
    # влит в base"), но не зависит от COMPARE_DEFAULT mock'а, который для
    # stale-rebase watchdog отдаёт {"ahead_by":0,...,"identical"} по
    # умолчанию (fail-open). С mergedAt фолбэк только когда PR реально
    # слит, и unit-тесты могут явно через PR_<n>_MERGEDAT_JSON
    # контролировать merge-состояние.
    if [ -n "$pr_num" ]; then
        local merged_at
        merged_at="$(gh pr view "$pr_num" --repo "$gh_repo" --json mergedAt --jq '.mergedAt // ""' 2>/dev/null || echo '')"
        if [ -n "$merged_at" ] && [ "$merged_at" != "null" ]; then
            reason="stale-after-upstream-fix: PR #${pr_num} уже в origin/${pr_base} (REST mergedAt=${merged_at}, ретро t_9d375e3e / ADR-0035)"
            evidence="gh pr view ${pr_num} --json mergedAt → ${merged_at}"
            printf '%s\t%s\t%s\t%s\t%s\n' \
                "true" "$pr_sha" "rest_fallback" "$reason" "$evidence"
            unset merged_at
            return 0
        fi
        unset merged_at
    fi

    # Ни одна стратегия не сработала — карточка не stale.
    printf '%s\t%s\t%s\t%s\t%s\n' "false" "" "none" "" ""
    return 0
}

stale_after_upstream_fix_scan_all() {
    # Orchestrator для auto-block + rate-limit (ADR-0035 §5.2). Сама detect
    # логика живёт в detect_stale_after_upstream_fix() — pure функция без
    # побочных эффектов, тестируемая отдельно. Этот orchestrator:
    #   1. Собирает список diagnostic-карточек.
    #   2. Для каждой — парсит маркеры и вызывает detect_*() → TSV-результат.
    #   3. Если stale — проверяет rate-limit, применяет auto-block + comment.
    [ "$STALE_AFTER_UPSTREAM_FIX_SCAN" = "true" ] || {
        log "stale-after-upstream-fix: STALE_AFTER_UPSTREAM_FIX_SCAN=false — skip"
        return 0
    }
    local diag_cards _card_id _body _pr_num _pr_sha _pr_base _sig_list _tests_list
    local _created_ts _marker _last_ts _now_ts _reason _commit_sha _gh_url _patch
    local _det_stale _det_sha _det_strategy _det_reason _det_evidence _det_line

    diag_cards="$(hermes kanban --board "$KANBAN_BOARD" list --json 2>/dev/null | python3 -c '
import json, sys
try:
    data = json.loads(sys.stdin.read())
except Exception:
    sys.exit(0)
for t in data:
    title = t.get("title", "") or ""
    body = t.get("body", "") or ""
    status = t.get("status", "") or ""
    # Сигнатура diagnostic-карточки из PR #1743 (ретро t_e00f448d):
    # title начинается с "🐛 CI UNSTABLE DIAGNOSTIC #..." (НЕ с
    # "🐛 CI UNSTABLE:" — в карточках нет двоеточия сразу после UNSTABLE;
    # "🔀 rebase PR #..." — rebase reminder, тоже кандидат на маркеры).
    # ADR-0035: не фильтруем по наличию маркера здесь — legacy-карточки
    # без маркеров должны попасть в скан, чтобы bash мог залогировать
    # "no diag-pr marker, skip (legacy)" (test D5).
    is_diag = (title.startswith("🐛 CI UNSTABLE DIAGNOSTIC") or
               title.startswith("🔀 rebase PR #"))
    if is_diag and status not in ("done", "archived"):
        print(t.get("id", "") + "\t" + status)
' 2>/dev/null || true)"

    if [ -z "$diag_cards" ]; then
        log "stale-after-upstream-fix: no live diagnostic cards with markers"
        return 0
    fi

    while IFS=$'\t' read -r _card_id _card_status; do
        [ -z "$_card_id" ] && continue
        [ "$_card_status" = "done" ] && continue
        [ "$_card_status" = "archived" ] && continue

        # 2. Достать body карточки (через REST-like show, чтобы не зависеть от
        # hermes CLI-парсинга list output).
        _body="$(hermes kanban --board "$KANBAN_BOARD" show "$_card_id" --json 2>/dev/null | python3 -c '
import json, sys
try:
    data = json.loads(sys.stdin.read())
    # Поддержка двух форматов: {body: "..."} или {task: {body: "..."}}.
    body = data.get("body") or (data.get("task", {}) or {}).get("body", "")
    print(body)
except Exception:
    pass
' 2>/dev/null || true)"

        # 3. Парсинг маркеров (grep + sed). Каждый маркер — одна строка.
        # ADR-0035: для sha используем [a-f0-9]+ (минимум 7 символов — git
        # short SHA) чтобы избежать захвата одиночных букв из имён маркеров
        # (например, 'd' от 'diag-pr-sha:' при greedy match в начале body).
        # Такой баг был в первой версии — _pr_sha получал "d\na\na\n<full>".
        # ВАЖНО (set -o pipefail): каждая команда в pipeline может вернуть
        # ненулевой код (grep при отсутствии совпадений = 1, sed на пустом
        # stdin = 0). Чтобы assignment не провалился под set -e, после каждого
        # pipeline ставим `|| echo ""` — подавляем ошибку и подставляем пусто.
        _pr_num="$(printf '%s' "$_body" | grep -oE '<!-- diag-pr: [0-9]+ -->' | head -1 | grep -oE '[0-9]+' || echo "")"
        _pr_sha="$(printf '%s' "$_body" | grep -oE '<!-- diag-pr-sha: [a-f0-9]+ -->' | head -1 | grep -oE '[a-f0-9]{7,}' || echo "")"
        _pr_base="$(printf '%s' "$_body" | grep -oE '<!-- diag-pr-base: [^ ]+ -->' | head -1 | sed 's/<!-- diag-pr-base: //;s/ -->//' || echo "")"
        # sig/tests могут содержать запятые и пути. Берём всё до -->.
        # ADR-0035 (D9): whitespace-only маркеры (backfill оставляет пустые
        # значения: «<!-- diag-sig:  -->») НЕ должны считаться непустыми
        # списками — иначе strat C ошибочно срабатывает на легаси-карточках.
        # После sed убираем trailing --> и trim'им whitespace — пустые
        # маркеры → пустая строка → strat C/B skip корректно.
        _sig_list="$(printf '%s' "$_body" | grep -oE '<!-- diag-sig: [^>]+-->' | head -1 | sed 's/<!-- diag-sig: //;s/-->$//' | sed 's/^[[:space:]]*//;s/[[:space:]]*$//' || echo "")"
        _tests_list="$(printf '%s' "$_body" | grep -oE '<!-- diag-tests: [^>]+-->' | head -1 | sed 's/<!-- diag-tests: //;s/-->$//' | sed 's/^[[:space:]]*//;s/[[:space:]]*$//' || echo "")"
        _created_ts="$(printf '%s' "$_body" | grep -oE '<!-- diag-created-ts: [0-9]+ -->' | head -1 | grep -oE '[0-9]+' || echo "")"

        if [ -z "$_pr_num" ]; then
            log "stale-after-upstream-fix: ${_card_id} — no diag-pr marker, skip (legacy)"
            continue
        fi
        [ -z "$_pr_base" ] && _pr_base="$DEVELOP_BRANCH"

        # 4. Rate-limit: ищем предыдущий block-комментарий с маркером.
        # Используем GH_JOURNAL как fallback (для тестов, где KANBAN_DB
        # недоступна) — production-путь остаётся kanban_last_reminder_ts.
        _marker="stale-after-upstream-fix:${_pr_num}"
        _last_ts="$(kanban_last_reminder_ts "$_card_id" "$_marker" 2>/dev/null || echo "")"
        if [ -z "$_last_ts" ] && [ -n "${GH_JOURNAL:-}" ] && [ -f "${GH_JOURNAL:-/nonexistent}" ]; then
            # ADR-0035 test fixture: pre-seeded journal entries simulating
            # prior tick. Grep for the marker line and parse its timestamp.
            _last_ts="$(grep -E "hermes .* block ${_card_id}.*${_marker}" "${GH_JOURNAL}" 2>/dev/null \
                | head -1 | awk '{print $1}' \
                | awk -F'T' '{print $1"T"$2}' \
                | python3 -c '
import sys, datetime
try:
    line = sys.stdin.read().strip()
    if not line: sys.exit(0)
    dt = datetime.datetime.fromisoformat(line.replace("Z", "+00:00"))
    print(int(dt.timestamp()))
except Exception:
    pass
' 2>/dev/null || echo "")"
        fi
        _now_ts="$(date +%s)"
        if [ -n "$_last_ts" ] && [ $(( _now_ts - _last_ts )) -lt "$STALE_AFTER_UPSTREAM_FIX_COOLDOWN_SECONDS" ]; then
            log "stale-after-upstream-fix: ${_card_id} — rate-limited (last=${_last_ts})"
            continue
        fi

        # 5. Pure detector (ADR-0035 §5.2: detect и auto-block РАЗДЕЛЬНЫ).
        # Возвращает TSV на stdout: stale\tsha\tstrategy\treason\tevidence.
        # Никаких side-effects — это ключевая гарантия тестируемости.
        # ВАЖНО (set -o pipefail): вызов функции внутри $() сам по себе не
        # проваливается — `|| true` не нужен, detect возвращает 0 всегда.
        # Но внутри detect её собственные pipelines защищены `|| echo ""`.
        _det_line="$(detect_stale_after_upstream_fix \
            "$_card_id" "$_pr_num" "$_pr_sha" "$_pr_base" \
            "$_sig_list" "$_tests_list" "$_created_ts" \
            "${REPO_DIR:-}" "${GH_REPO:-krikz/rob_box_project}" \
            2>/dev/null || true)"
        # Парсим TSV (5 полей через tab). Bash `read -r` с IFS=$'\t'
        # НЕ сохраняет пустые поля (consecutive delimiters collapse) — используем
        # python (надёжно для empty-field TSV). ADR-0035 §5.2: TSV-контракт
        # между detect и orchestrator должен быть стабильным, поэтому
        # парсим детерминированно через python, а не через IFS gymnastics.
        # ВАЖНО (eval и shlex.quote): значения содержат `:` (например,
        # "stale-after-upstream-fix: ...") и `;`, поэтому eval БЕЗ кавычек
        # пытается выполнить их как команды (`фикс: command not found` —
        # известная ловушка). shlex.quote() оборачивает в одинарные кавычки.
        eval "$(_det_line="$_det_line" python3 -c '
import os, shlex
line = os.environ.get("_det_line", "") or ""
if line.endswith("\n"):
    line = line[:-1]
parts = line.split("\t", 4)  # max 5 полей; последний (evidence) может
                              # содержать tabs — split("...", 4) ограничивает.
while len(parts) < 5:
    parts.append("")
print("_det_stale=" + shlex.quote(parts[0]))
print("_det_sha=" + shlex.quote(parts[1]))
print("_det_strategy=" + shlex.quote(parts[2]))
print("_det_reason=" + shlex.quote(parts[3]))
print("_det_evidence=" + shlex.quote(parts[4]))
' 2>/dev/null)"
        _det_stale="${_det_stale:-false}"
        _det_sha="${_det_sha:-}"
        _det_strategy="${_det_strategy:-none}"
        _det_reason="${_det_reason:-}"
        _det_evidence="${_det_evidence:-}"

        if [ "$_det_stale" != "true" ]; then
            log "stale-after-upstream-fix: ${_card_id} (PR #${_pr_num}) — upstream-фикс пока не найден, skip"
            continue
        fi

        # 6. Логируем какая стратегия сработала (нужно для диагностики и для
        # тестов D1/D2/D3/D4/D9 — каждая ищет свой marker в stderr).
        case "$_det_strategy" in
            closed)        log "stale-after-upstream-fix: ${_card_id} — PR #${_pr_num} CLOSED, blocking as transient" ;;
            A)             log "stale-after-upstream-fix: ${_card_id} — strat A (PR merged)" ;;
            B)             log "stale-after-upstream-fix: ${_card_id} — strat B (upstream-fix hit: ${_det_sha:0:8})" ;;
            C)             log "stale-after-upstream-fix: ${_card_id} — strat C (fix in same PR)" ;;
            rest_fallback) log "stale-after-upstream-fix: ${_card_id} — REST fallback (PR mergedAt)" ;;
        esac

        _reason="$_det_reason"
        _commit_sha="$_det_sha"

        # 7. DRY_RUN: только лог.
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would: block ${_card_id} with reason: ${_reason}"
            continue
        fi

        # 8. Auto-block + body patch (side effects — ТОЛЬКО orchestrator).
        if hermes kanban --board "$KANBAN_BOARD" block --kind transient \
            "$_card_id" "$_reason" >/dev/null 2>&1; then
            log "stale-after-upstream-fix: ${_card_id} auto-blocked (PR #${_pr_num}, sha=${_commit_sha:-none})"
        else
            log "stale-after-upstream-fix: WARNING block ${_card_id} failed"
            continue
        fi

        # 9. Patch body: добавить секцию "Upstream-фикс (auto-detected)".
        if [ -n "$_commit_sha" ]; then
            _gh_url="https://github.com/${GH_REPO}/commit/${_commit_sha}"
            _patch="$(printf '\n\n### ✅ Upstream-фикс уже в develop (auto-detected, merge-gate ADR-0035, %s)\n\n**Причина блокировки:** %s\n\n**Upstream-коммит:** [%s](%s)\n\n**Что делать:** карточка может быть закрыта как `done` (stale-diagnostic-after-upstream-fix). Воркеру не нужно ничего чинить — регрессия upstream-починена, тесты на develop уже зелёные.\n' "$(date -u +%H:%M:%SZ)" "$_reason" "${_commit_sha:0:8}" "$_gh_url")"
            hermes kanban --board "$KANBAN_BOARD" comment "$_card_id" "$_patch" >/dev/null 2>&1 \
                || log "stale-after-upstream-fix: WARNING body patch comment failed for ${_card_id}"
        fi
    done < <(printf '%s\n' "$diag_cards")
    log "stale-after-upstream-fix: scan complete"
    return 0
}

# --- kanban card status helper (ретро 12.08 t_8af6bf29) ---------------------
# 'hermes kanban show' ПАДАЕТ после hermes-agent v0.20.0 (sqlite3.ProgrammingError
# 'Cannot operate on a closed database' в task_graph_context — краш после вывода
# заголовка; exit 1 + traceback в stderr). list работает. Читаем статус карточки
# напрямую из kanban DB (быстро, без CLI-зависимости), fallback — `list --json`.
# KANBAN_DB: путь к sqlite-базе доски (совпадает с тем, что открывает hermes).
kanban_card_status() {  # $1=task_id → печатает status (done|ready|...|archived) или пусто
    local tid="$1" db st=""
    [ -z "$tid" ] && return 0
    db="${KANBAN_DB:-$HOME/.hermes/kanban/boards/$KANBAN_BOARD/kanban.db}"
    if [ -f "$db" ]; then
        st="$(python3 - "$db" "$tid" <<'PYEOF' 2>/dev/null || true
import sqlite3, sys, os
db, tid = sys.argv[1], sys.argv[2]
try:
    if not os.path.exists(db):
        sys.exit(0)
    conn = sqlite3.connect(db)
    row = conn.execute("SELECT status FROM tasks WHERE id=?", (tid,)).fetchone()
    conn.close()
    if row:
        print(row[0])
except Exception:
    pass
PYEOF
)"
    fi
    if [ -z "$st" ]; then
        # fallback: `list --json` (работает после v0.20.0)
        st="$("$HERMES_BIN" kanban --board "$KANBAN_BOARD" list --json 2>/dev/null \
            | python3 -c '
import sys, json
try:
    d = json.load(sys.stdin)
    tasks = d if isinstance(d, list) else d.get("tasks", [])
    for t in tasks:
        if t.get("id") == sys.argv[1]:
            print(t.get("status", "")); break
except Exception:
    pass
' "$tid" 2>/dev/null || true)"
    fi
    printf '%s' "$st"
}

# --- archive карточки по MERGED PR (ретро 14.08 t_0bd15be9) -----------------
# done → archive; blocked → unblock + complete + archive. Раньше архив-маппинг
# скипал status!=done: blocked-карточка с ВЛИТЫМ фиксом висела вечно (recovery
# «родитель закроется процессом», а процесса для blocked нет). Фикс влит (PR
# MERGED) ⇒ критерий карточки выполнен независимо от причины blocked
# (timeout/needs_input/capability) → unblock (reason «фикс влит, критерий
# выполнен») → complete → archive. Идемпотентно: повторный тик видит archived.
archive_merged_card() {  # $1=card_id $2=issue number $3=pr_number (для completion-check) $4=branch (опц.)
    local cid="$1" num="$2" pr="$3" br="${4:-}" cstate=""
    [ -z "$cid" ] && return 0
    # GATE-3 (ADR-0022 §4.3): блокируем archive если PR имеет красный CI.
    # Типичный R5-сценарий (ретро 14.08 PR #1418): воркер завершился без
    # зелёного CI, merge-gate всё равно смержил → карточка archive'илась
    # как done. Теперь: completion-check ОБЯЗАН вернуть 0, иначе — return
    # (карточка остаётся done, идемпотентный retry следующего тика, после
    # re-run CI с зелёным — archive проходит). ADR-0018: «честный FAIL
    # лучше красивого PASS» — это enforcement.
    if [ -n "${pr:-}" ]; then
        local _completion_check
        _completion_check="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/agent-flow-completion-check.sh"
        if [ -f "$_completion_check" ]; then
            if ! bash "$_completion_check" "$pr" 2>/dev/null; then
                log "[completion-check] card ${cid} PR #${pr} has FAILURE — не архивирую (status stays ${cstate:-done})"
                return 0
            fi
        fi
    fi
    cstate="$(kanban_card_status "$cid")"
    if [ "$cstate" = "done" ]; then
        if "$HERMES_BIN" kanban --board "$KANBAN_BOARD" archive "$cid" >/dev/null 2>&1; then
            log "issue #${num}: card ${cid} archived (merged)"
            # OpenSpec sync (ADR-0039): archive change folder при archive карточки.
            # Если sync падает — НЕ блокируем merge-gate (warn + log). OpenSpec — advisory.
            archive_openspec_change_for_merge "$cid" "$num" "$pr" "$br" || \
                log "openspec-sync: WARN archive-change failed for card ${cid} (non-fatal, kanban ok)"
        fi
    elif [ "$cstate" = "blocked" ]; then
        if "$HERMES_BIN" kanban --board "$KANBAN_BOARD" unblock \
                --reason "фикс влит, критерий выполнен" "$cid" >/dev/null 2>&1 \
            && "$HERMES_BIN" kanban --board "$KANBAN_BOARD" complete \
                --summary "фикс влит, критерий выполнен (ретро 14.08 t_0bd15be9)" "$cid" >/dev/null 2>&1; then
            "$HERMES_BIN" kanban --board "$KANBAN_BOARD" archive "$cid" >/dev/null 2>&1 \
                && {
                    log "issue #${num}: card ${cid} unblocked+completed+archived (merged, was blocked)"
                    # OpenSpec sync (ADR-0039): archive change folder.
                    archive_openspec_change_for_merge "$cid" "$num" "$pr" "$br" || \
                        log "openspec-sync: WARN archive-change failed for card ${cid} (non-fatal, kanban ok)"
                } \
                || log "issue #${num}: WARNING card ${cid} complete ok, archive failed — retry next tick"
        else
            log "issue #${num}: WARNING card ${cid} blocked → unblock/complete failed — retry next tick"
        fi
    fi
}

# --- OpenSpec sync (ADR-0039) ----------------------------------------------
# Helper: archive OpenSpec change-folder при archive kanban-карточки.
# Идемпотентно (см. agent-flow-openspec-sync.sh: archive-change skip if already
# archived). Принимает branch опционально — для deriving slug из branch-suffix.
# Если branch не передан, slug выводится из cid: t_<hex> → "<cid>".
archive_openspec_change_for_merge() {  # $1=cid $2=num $3=pr $4=branch
    local cid="$1" num="$2" pr="$3" br="$4" sync_bin _slug _out=1
    [ -z "$cid" ] && return 0
    sync_bin="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/agent-flow-openspec-sync.sh"
    [ -x "$sync_bin" ] || { log "openspec-sync: $sync_bin not found/executable — skipping"; return 0; }
    # slug = branch-suffix (z-{agent}/<id>-<slug> → <slug>), fallback = cid.
    if [ -n "$br" ]; then
        _slug="$(printf '%s' "$br" | sed -E 's|^z-[a-z0-9_-]+/||; s|^[0-9]+-||')"
    else
        _slug="$cid"
    fi
    if "$sync_bin" archive-change "$num" "$cid" "$_slug" "$pr" >/dev/null 2>&1; then
        log "openspec-sync: change folder archived for ${cid}-${_slug} (PR #${pr:-?})"
        return 0
    else
        return 1
    fi
}

# --- rate-limit конфликт/UNSTABLE-комментариев (ретро 12.08 t_8af6bf29) -----
# scan-all-prs комментил карточку 'ОБЯЗАН rebase' КАЖДЫЙ тик (~10 мин) при
# PR CONFLICTING → шум. Теперь: коммент не чаще 1 раза в 2 часа. Таймстамп
# последнего однотипного коммента берём из kanban DB (task_comments.body LIKE).
kanban_last_reminder_ts() {  # $1=task_id $2=marker-substring → epoch или пусто
    local tid="$1" marker="$2" db
    [ -z "$tid" ] && return 0
    db="${KANBAN_DB:-$HOME/.hermes/kanban/boards/$KANBAN_BOARD/kanban.db}"
    [ -f "$db" ] || return 0
    python3 - "$db" "$tid" "$marker" <<'PYEOF' 2>/dev/null || true
import sqlite3, sys, os
db, tid, marker = sys.argv[1], sys.argv[2], sys.argv[3]
try:
    if not os.path.exists(db):
        sys.exit(0)
    conn = sqlite3.connect(db)
    row = conn.execute(
        "SELECT MAX(created_at) FROM task_comments WHERE task_id=? AND body LIKE ?",
        (tid, "%" + marker + "%")).fetchone()
    conn.close()
    if row and row[0]:
        print(row[0])
except Exception:
    pass
PYEOF
}

# G6: flock sentinel — skip tick if another instance holds the lock.
# Тело — af_flock_guard_or_exit в lib_agent_flow_common.sh (дедуп 30.08).
af_flock_guard_or_exit "$LOCK_FILE"

# --- G1: MAINTENANCE gate (remote + local) -----------------------------------
# Тело — af_maintenance_gate_or_exit в lib_agent_flow_common.sh (дедуп 30.08:
# три байт-в-байт копии в triage / merge-gate / e2e-process).
af_maintenance_gate_or_exit

# --- G2: gh auth check -------------------------------------------------------
if ! gh auth status >/dev/null 2>&1; then
    log "gh auth not configured — exit 1"; exit 1
fi

# --- required env ------------------------------------------------------------
: "${GH_REPO:?GH_REPO must be set (owner/repo)}"

# --- pull open issues with `hermes` label -----------------------------------
# Ретро 19.08 #1457: gh issue list --label ломает фильтр → fallback через
# gh_list_issues_by_label (REST API), если gh-list пустой.
issues_json="$(gh_list_issues_by_label "$ISSUE_LABEL" open "$ISSUE_LIMIT")"

# G3: empty output is ambiguous — could be "no issues" OR "rate-limited".
if [ -z "$issues_json" ] || [ "$issues_json" = "[]" ]; then
    rate="$(gh api rate_limit --jq '.resources.core.remaining' 2>/dev/null || echo 999)"
    if [ "${rate:-999}" = "0" ]; then
        log "GitHub rate-limit exhausted — skip tick"; exit 0
    fi
    # Ретро 15.08 t_2c814334 (pr-orphan-no-labels): `gh issue list` / `gh pr
    # list` идут через GraphQL. При graphql rate-limit=0 (а core при этом
    # жив — квоты РАЗНЫЕ) они МОЛЧА возвращают [] → merge-gate «слепо»
    # сканирует пустоту и не ставит метки (инцидент: #1282/#1284/#1286
    # созданы 07:38-07:55Z, CI green, labels=[] 5.5ч). e2e-process уже
    # проверяет min(core,graphql) (стр. 597-600) — merge-gate отставал.
    # Здесь НЕ прерываем тик (core жив): REST-based pr-backfill-scan ниже
    # разметит open PR без меток через core-квоту (gh api pulls).
    rate_gql="$(gh api rate_limit --jq '.resources.graphql.remaining' 2>/dev/null || echo 999)"
    if [ "${rate_gql:-999}" = "0" ]; then
        log "⚠️ GraphQL rate-limit exhausted (core жив) — GraphQL-сканы (осн. цикл/clean-pr-sweep) слепы; REST pr-backfill-scan продолжит (ретро 15.08 t_2c814334)"
    fi
    # Ретро-путь (12.08 t_68607832): hermes-issues может не быть вовсе, но
    # смерженные PR, ссылающиеся на немаркированные issues, всё равно нужно
    # обработать (сканы ниже). Раньше здесь был exit 0 → ретро-issues навсегда
    # выпадали из merge-gate, когда очередь hermes пуста.
    # stale-branch guard (ретро 12.08 t_d3aeaa9b) вызывается НИЖЕ (scan-all-prs
    # блок) — этот путь всегда доходит до него (continue, не exit 0), поэтому
    # здесь дублировать вызов нельзя: PR с уже влитой ветки получил бы коммент
    # дважды за тик (ретро-ветки devops/architect issues не имеют — их
    # stale-re-commit PR ловит scan-all вызов).
    log "no issues with label '${ISSUE_LABEL}' — continuing to scan-all-prs + retro-path"
    issues_json='[]'
fi

# slugify — перенесена в lib_agent_flow_common.sh (дедуп 30.08).

# has_label — перенесена в lib_agent_flow_common.sh (дедуп 30.08).

# User-reopen guard helpers (issue #1391, retro 18.08 t_c4f1d5c8).
# Если юзер вручную переоткрыл issue ПОСЛЕ того, как e2e-process поставил
# e2e-done — метка «протухла» и auto-close подавляет волю юзера. Эти helpers
# достают временные метки из timeline issue (через gh api), чтобы merge-gate
# мог сравнить «когда поставили e2e-done» vs «когда юзер сделал reopen».
# Empty / null на любой стороне → вызывающий решает что делать (для нашего
# guard-а это «пропускаем защиту, ведём себя как раньше» — fail-open, чтобы
# не сломать regression-acceptance при недоступности timeline API).
# jq-filter ниже совместим с mock_env.sh (apply_jq, паттерн
# `[.[] | select(.field == "VAL")][-1].field`) и с реальным gh api.
_timeline_last_labeled_at() {  # $1=issue_number $2=label_name
    local issue="$1" label="$2"
    gh api "repos/${GH_REPO}/issues/${issue}/timeline?per_page=100" \
        --jq "[.[] | select(.event==\"labeled\" and .label.name==\"${label}\")][-1].created_at" \
        2>/dev/null || printf ''
}
_timeline_last_reopen_at() {  # $1=issue_number
    local issue="$1"
    gh api "repos/${GH_REPO}/issues/${issue}/timeline?per_page=100" \
        --jq "[.[] | select(.event==\"reopened\")][-1].created_at" \
        2>/dev/null || printf ''
}

# Ретро 18.08 t_873ebef2 (#1391, дополнение к PR #1399 от e3f227e2):
# Q22-orphan-close путь (нет e2e-done в timeline — e2e был невозможен)
# тоже должен подавляться при ручном reopen. Широкий детект «недавнего
# reopen» без требования e2e-done. Использует _timeline_last_reopen_at
# (тот же jq-фильтр, mock_env.sh совместим). Возвращает 0 если недавний
# reopen есть, иначе 1. Окно: USER_REOPEN_RECENT_DAYS дней (0 = навсегда).
_issue_reopened_recently() {  # $1=issue_number
    local issue="$1" reopen_at
    reopen_at="$(_timeline_last_reopen_at "$issue")"
    [ "$reopen_at" = "null" ] && reopen_at=""
    [ -z "$reopen_at" ] && return 1
    if [ "${USER_REOPEN_RECENT_DAYS}" = "0" ]; then
        return 0
    fi
    local window_start
    window_start="$(date -u -d "${USER_REOPEN_RECENT_DAYS} days ago" +%Y-%m-%dT%H:%M:%SZ 2>/dev/null \
        || date -u +%Y-%m-%dT%H:%M:%SZ)"
    # Lexicographic compare works for ISO-8601 Z (timestamp pattern).
    if [ "$reopen_at" \> "$window_start" ] || [ "$reopen_at" = "$window_start" ]; then
        return 0
    fi
    return 1
}

# Ретро 22.08 t_562a8682: ahead-of-develop для PR через REST compare API.
# Возвращает ahead_by (сколько коммитов в head нет в base). Если PR
# закрыт/недоступен или compare API вернул non-JSON — печатает "0" (fail-
# open: главное — не зашуметь ложным watchdog-алертом).
#
# Зачем REST compare, а не `git fetch + rev-list`: merge-gate может
# крутиться в cron на хосте без полного clone репо (или с устаревшим
# origin/develop). REST compare идёт напрямую в GitHub API — нужен только
# gh auth. Также быстрее на больших PR.
#
# Использование: в stale-rebase watchdog для PR с needs-review (см. блок
# в process-each-issue). ahead > STALE_REBASE_AHEAD_THRESHOLD → alert.
#
# Аргументы:
#   $1=head_branch (имя ветки, e.g. "wt/fix-deploy-namespace")
#   $2=base_branch (default develop)
# jq-фильтр совместим с mock_env.sh (.field без дополнительных wrappers).
pr_compare_ahead() {  # $1=head_branch $2=base_branch (default develop)
    local head="$1" base="${2:-${DEVELOP_BRANCH}}"
    [ -n "$head" ] || { printf '%s' "0"; return 0; }
    gh api "repos/${GH_REPO}/compare/${base}...${head}" 2>/dev/null \
        | python3 -c '
import json, sys
try:
    d = json.load(sys.stdin)
    print(int(d.get("ahead_by", 0) or 0))
except Exception:
    print(0)
' 2>/dev/null || printf '%s' "0"
}

# Ретро 15.08 t_16325ddd (гонка PR-state): creator карточек (merge-gate
# scan-all-prs / e2e-fail) НЕ пере-проверял state PR после скана и создавал
# карточки для PR, закрытых товарищем Шифу («Не делаем это») → мёртвые карточки
# (t_62851a11, t_19a87086): rebase невозможен, следующего раунда для закрытых
# веток не будет. Правило: ПЕРЕД create карточки проверяем state PR заново —
# CLOSED → SKIP (карточка не нужна). Проверка идемпотентная, gh pr view.
# $1=pr_number → печатает CLOSED/OPEN/MERGED/unknown (пусто при ошибке).
pr_state_now() {  # $1=pr_number
    [ -n "$1" ] || { printf '%s' "unknown"; return 0; }
    gh pr view "$1" --repo "$GH_REPO" --json state --jq '.state' 2>/dev/null \
        || printf '%s' "unknown"
}

# Ретро 31.08 t_e00f448d: merge-gate UNSTABLE-блок раньше всегда создавал rebase-
# карточки (по процессу Шифу 10.08 — «взять девелоп сейчас и позеленеть»), но это
# работает только для stale-from-develop. Если CI красный ИЗ-ЗА unit/lint
# regression в самом коде PR (PR #1740/1741 — реальный случай 31.08),
# rebase не поможет: develop-фиксов нет, регрессия — в PR. Нужно отличать:
#
#   pr_classify_failure "$pr_head_oid"
#     → печатает "unit_lint" если хотя бы один failed check-run — lint/unit-test
#     → печатает "integration_e2e" если только build/deploy/e2e/integration
#     → печатает "unknown" если не смогли достать check-runs (fail-open → старое
#       поведение: rebase-карточка ОК, develop-фиксы могут починить e2e).
#
# Классификация по имени check-run (регулярка, регистронезависимо):
#   unit_lint:    lint|test|unit|pytest|mypy|ruff|flake8|black|coverage
#   integration:  integration|e2e|deploy|build|docker|release|smoke
# При наличии ОБЕИХ категорий → unit_lint (худший случай: реальный код — лечить
# код, не rebase'ить).
pr_classify_failure() {  # $1=head_oid → печатает категорию
    local head_oid="${1:-}"
    [ -n "$head_oid" ] || { printf '%s' "unknown"; return 0; }
    local failed_json
    failed_json="$(gh api "repos/${GH_REPO}/commits/${head_oid}/check-runs" \
        --jq '[.check_runs[] | select(.conclusion == "failure" or .conclusion == "timED_OUT" or .conclusion == "cANCELLED")] | map({name, html_url})' \
        2>/dev/null || echo "")"
    [ -z "$failed_json" ] || [ "$failed_json" = "null" ] || [ "$failed_json" = "[]" ] && \
        { printf '%s' "unknown"; return 0; }
    # Классификация по именам. Если хоть один матчит unit_lint → unit_lint
    # (смесь = реальный код в PR — diagnostic).
    if printf '%s' "$failed_json" | grep -qiE '"name"[[:space:]]*:[[:space:]]*"[^"]*(lint|test|unit|pytest|mypy|ruff|flake8|black|coverage)'; then
        printf '%s' "unit_lint"
        return 0
    fi
    if printf '%s' "$failed_json" | grep -qiE '"name"[[:space:]]*:[[:space:]]*"[^"]*(integration|e2e|deploy|build|docker|release|smoke)'; then
        printf '%s' "integration_e2e"
        return 0
    fi
    printf '%s' "unknown"
}

# Печатает JSON-список failed jobs в формате {name,html_url} для body карточки.
# $1=head_oid. Пустая строка если не смогли достать (fail-open).
pr_failed_jobs_json() {  # $1=head_oid
    local head_oid="${1:-}"
    [ -n "$head_oid" ] || { printf '%s' ""; return 0; }
    gh api "repos/${GH_REPO}/commits/${head_oid}/check-runs" \
        --jq '[.check_runs[] | select(.conclusion == "failure" or .conclusion == "timed_out" or .conclusion == "cancelled")] | map({name, html_url}) | tostring' \
        2>/dev/null || printf '%s' ""
}

# Ретро 02.09 t_8e08b861: scan-all-prs не различал develop-side регрессию
# (red CI в develop HEAD, не в PR) и PR-side регрессию → спамил 19 rebase-
# карточек на PR #1857 за сутки при ahead=2/behind=0. Хелпер ниже возвращает
# behind-число через REST compare. "unknown" при flake.
pr_behind_develop() {  # $1=head_oid → печатает behind_by или "unknown"
    local head_oid="${1:-}"
    [ -n "$head_oid" ] || { printf '%s' "unknown"; return 0; }
    local n
    n="$(gh api "repos/${GH_REPO}/compare/${DEVELOP_BRANCH}...${head_oid}" \
        --jq '.behind_by' 2>/dev/null || echo unknown)"
    [ -z "$n" ] && n="unknown"
    printf '%s' "$n"
}

# Ретро 02.09 t_8e08b861 + t_ecd43187: is_develop_regression детектор, который
# был в воркспейсе t_ecd43187 но не дожил до merge. Возвращает 0 (true)
# если develop HEAD падает на ВСЕ те же check-runs что и PR (или develop
# падает на БОЛЬШЕ — PR мог пройти часть, develop — нет). Кейс PR #1857:
# develop падает на [Unit Tests, Integration Tests], PR — только [Unit Tests].
# dev ⊇ pr → develop-side regression (rebase бессилен).
# Если pr.failed ⊃ dev.failed (PR падает на что-то дополнительно) — это
# PR-side ответственность (rebase не поможет, но это вина PR).
# $1=pr_head_oid $2=dev_sha. Если не смогли достать (flake) → return 1
# (false) → fail-open: пусть старая логика отработает.
is_develop_regression() {  # $1=pr_head_oid $2=dev_sha → return 0|1
    local pr_head="${1:-}" dev_sha="${2:-}"
    [ -n "$pr_head" ] && [ -n "$dev_sha" ] || return 1
    local pr_failed dev_failed
    pr_failed="$(gh api "repos/${GH_REPO}/commits/${pr_head}/check-runs" \
        --jq '[.check_runs[]|select(.conclusion=="failure" or .conclusion=="timed_out" or .conclusion=="cancelled")|.name]|.[]' \
        2>/dev/null | sort -u || true)"
    dev_failed="$(gh api "repos/${GH_REPO}/commits/${dev_sha}/check-runs" \
        --jq '[.check_runs[]|select(.conclusion=="failure" or .conclusion=="timed_out" or .conclusion=="cancelled")|.name]|.[]' \
        2>/dev/null | sort -u || true)"
    [ -z "$pr_failed" ] && return 1  # нет failed на PR — не regression
    [ -z "$dev_failed" ] && return 1  # develop чистый — не develop-side
    # Развилка:
    #   dev ⊇ pr (dev.failed ⊇ pr.failed) → develop-side regression (true)
    #   pr ⊃ dev (pr.failed ⊃ dev.failed) → PR-side, свой код (false)
    #   dev ⊂ pr И pr ⊂ dev (без строгого вложения — частичное пересечение) →
    #     mixed: считаем develop-side (rebase всё равно no-op, чинить develop).
    # dev ∖ pr = проверка, что develop падает только на то, на что падает PR
    # (т.е. dev ⊆ pr — develop ответственен на подмножестве PR-провалов).
    # pr ∖ dev = PR падает на что-то, чего develop не падает — это PR-side.
    local dev_only pr_only
    dev_only="$(comm -23 <(printf '%s\n' "$dev_failed") <(printf '%s\n' "$pr_failed") 2>/dev/null)"
    pr_only="$(comm -13 <(printf '%s\n' "$dev_failed") <(printf '%s\n' "$pr_failed") 2>/dev/null)"
    if [ -n "$dev_only" ] && [ -z "$pr_only" ]; then
        # dev ⊋ pr (develop падает на ВСЁ что PR + ещё) → develop-side.
        return 0
    fi
    if [ -z "$dev_only" ] && [ -z "$pr_only" ]; then
        # dev == pr (полное совпадение) → develop-side.
        return 0
    fi
    if [ -n "$dev_only" ] && [ -n "$pr_only" ]; then
        # Смесь: develop падает на часть, PR — на другую часть. В любом
        # случае rebase develop-HEAD не поможет (behind=0), и чинить нужно
        # ОБА источника. Считаем develop-side: develop всё равно в регрессии,
        # и ворсер-классификатор отдельно разберётся с PR-only failed.
        return 0
    fi
    # pr_only есть, dev_only пусто → pr ⊋ dev → PR-side, develop чист по
    # этому набору → вина PR.
    return 1
}

# Ретро 02.09 t_8e08b861: circuit breaker. Считает rebase-карточки на PR за 24ч
# в статусе done (т.е. воркер уже сделал rebase и закрыл). Если ≥3 — значит
# rebase бессилен (PR-side или develop-side регрессия, не stale-from-develop),
# дальнейшие карточки только жгут токены. Печатает число или 0.
count_rebase_cards_24h() {  # $1=pr_num → печатает count
    local pr_num="${1:-}"
    [ -n "$pr_num" ] || { printf '%s' "0"; return 0; }
    local since_ts
    since_ts="$(date -u -d '24 hours ago' +%s 2>/dev/null || date -u +%s)"
    local cnt
    cnt="$(hermes kanban --board "$KANBAN_BOARD" list --json 2>/dev/null | python3 -c "
import json,sys,os,time
try:
    data = json.loads(sys.stdin.read())
except Exception:
    sys.exit(0)
since = int(os.environ.get('SINCE_TS','0'))
pr_num = os.environ.get('PR_NUM','')
n = 0
for t in data:
    title = t.get('title','')
    if 'rebase PR #${pr_num}' not in title:
        continue
    st = t.get('status','')
    if st not in ('done','archived'):
        continue
    end = t.get('completed_at') or t.get('updated_at') or 0
    if end and int(end) >= since:
        n += 1
print(n)
" 2>/dev/null <<EOF
SINCE_TS=${since_ts}
PR_NUM=${pr_num}
EOF
)"
    printf '%s' "${cnt:-0}"
}

# detect_pr_kind — перенесена в lib_agent_flow_common.sh (дедуп 30.08).

# Ретро 25.08 t_00ba0224 (ADR-номер collision guard). merge-gate должен
# убедиться, что новый docs/adr/NNNN-*.md в PR не пересекается по номеру с
# уже существующим в origin/develop. Глобальная коллизия ломает обратные
# ссылки на ADR (документы/комментарии ссылаются на «0027-foo», а в develop
# теперь живёт «0027-bar» → битая ссылка).
#
# Алгоритм:
#   1. Получить список файлов PR (`gh pr view --json files --jq ...`).
#   2. Оставить только новые/переименованные docs/adr/NNNN-*.md. ПРАВКА
#      существующего 0027-foo.md не считается коллизией (NNNN уже в PR).
#   3. Для каждого NNNN из (2) — найти в develop ВСЕ файлы docs/adr/NNNN-*.
#      Если хотя бы один из них НЕ входит в изменённые файлы этого PR →
#      коллизия (другой файл уже занимает этот номер в develop).
#   4. Если override-метка ADR_COLLISION_OVERRIDE_LABEL стоит на issue →
#      пропускаем (Шифу явно одобрил re-numbering).
#
# Возвращает:
#   0 — OK (нет коллизии или override)
#   1 — КОЛЛИЗИЯ (PR заблокирован этой функцией; caller должен continue)
#
# Side effects при коллизии (не fatal):
#   - comment на issue (24h dedup, иначе спам на каждом тике)
#   - label ${ADR_COLLISION_BLOCKED_LABEL} на issue
#   - НИКОГДА не ставит needs-e2e / needs-review для этого PR
#
# Args:
#   $1 = pr_number
#   $2 = issue_number
#   $3 = labels_csv (lower-cased, comma-separated) — для has_label
check_adr_number_collision() {  # $1=pr_number $2=issue_number $3=labels_csv_lc
    local pr_number="$1" number="$2" labels_lc="$3"

    # Override Шифу — пропускаем. has_label уже работает по lower-cased.
    if has_label "$labels_lc" "$ADR_COLLISION_OVERRIDE_LABEL"; then
        log "issue #${number}: PR #${pr_number} ADR-collision override (${ADR_COLLISION_OVERRIDE_LABEL}) — пропускаем guard"
        return 0
    fi

    # Список файлов PR (только path'ы, без diff-метаданных — компактно и
    # стабильно). Если API упал — fail-open (return 0): лучше пустить PR,
    # чем ломать весь gate из-за flake. Коллизия никуда не денется — её
    # поймает следующий тик или сам ревьюер.
    local pr_files_json
    pr_files_json="$(gh pr view "$pr_number" --repo "$GH_REPO" --json files \
        --jq '[.files[].path]' 2>/dev/null || echo '[]')"
    if [ -z "$pr_files_json" ] || [ "$pr_files_json" = "null" ]; then
        log "issue #${number}: PR #${pr_number} ADR-collision: gh pr view --json files empty — fail-open (retry next tick)"
        return 0
    fi

    # Извлечь новые/переименованные ADR из PR. ПРАВКА существующего файла
    # (например 0027-foo.md → 0027-foo.md без rename) → в PR `path` будет
    # вида docs/adr/0027-foo.md; мы его НЕ считаем «новым» и НЕ валидируем
    # против develop (там уже 0027-foo.md, и он совпадает с PR). А вот
    # добавление/rename на docs/adr/0027-bar.md — это и есть «новый номер»,
    # который мы проверяем.
    #
    # Ключевое: NNNN извлекается ТОЛЬКО из файлов, присутствующих в этом PR.
    # Шаблон docs/adr/NNNN-*.md → NNNN = 4 hex-цифры.
    local pr_new_adrs
    # Извлекаем уникальные NNNN через newline-separated вывод (НЕ JSON-массив:
    # `read` в bash не парсит JSON-литералы, разделитель — перенос строки).
    # Сортируем для детерминированного порядка (полезно для логов).
    pr_new_adrs="$(printf '%s' "$pr_files_json" | python3 -c '
import json, re, sys
try:
    files = json.load(sys.stdin)
except Exception:
    files = []
adr_re = re.compile(r"^docs/adr/0[0-9]{3}-.*\.md$")
nums = set()
for f in files:
    if not isinstance(f, str): continue
    if adr_re.match(f):
        m = re.match(r"^docs/adr/(0[0-9]{3})-.*\.md$", f)
        if m: nums.add(m.group(1))
for n in sorted(nums):
    print(n)
' 2>/dev/null)"
    if [ -z "$pr_new_adrs" ]; then
        return 0  # нет новых ADR — guard не срабатывает
    fi

    # Список ВСЕХ ADR в origin/develop. Формат каждой строки: NNNN-name.md
    # (БЕЗ префикса docs/adr/ — чтобы внутренний grep "^NNNN-" корректно
    # находил файлы по номеру). Используем git ls-tree — это ЛОКАЛЬНЫЙ кэш
    # (origin/develop уже подтянут до merge-gate тика в любом нормальном
    # run-е). Если fetch ещё не прошёл и refs нет — fallback на
    # `gh api .../git/trees/develop` (медленнее, но quota-friendly). Если
    # и это упало — fail-open.
    local develop_adrs
    develop_adrs="$(git ls-tree "origin/${DEVELOP_BRANCH}" --name-only 2>/dev/null \
        | grep -E '^docs/adr/0[0-9]{3}-.*\.md$' \
        | sed 's@^docs/adr/@@' || true)"
    if [ -z "$develop_adrs" ]; then
        # Fallback: REST tree API (gh). Возвращает полный tree develop
        # одним запросом; quota = 1, медленнее, но работает на CI без
        # подтянутого origin/develop. Рекурсивный — recursive=1 обязателен.
        develop_adrs="$(gh api "repos/${GH_REPO}/git/trees/${DEVELOP_BRANCH}?recursive=1" \
            --jq '[.tree[].path | select(. | test("^docs/adr/0[0-9]{3}-.*\\\\.md$"))] | .[]' \
            2>/dev/null | sed 's@^docs/adr/@@' || true)"
    fi
    if [ -z "$develop_adrs" ]; then
        log "issue #${number}: PR #${pr_number} ADR-collision: develop tree empty (fetch + API оба упали) — fail-open (retry next tick)"
        return 0
    fi

    # Ищем коллизию: для каждого NNNN из pr_new_adrs проверяем, есть ли в
    # develop другой файл с тем же NNNN. «Другой» = basename не входит в
    # список изменённых файлов этого PR.
    local collision_detail=""
    while IFS= read -r nnnn; do
        [ -z "$nnnn" ] && continue
        # Файлы develop с этим номером.
        local dev_files clashing=""
        dev_files="$(printf '%s\n' "$develop_adrs" | grep -E "^${nnnn}-" || true)"
        # Убрать файлы, которые ЭТОТ ЖЕ PR тоже трогает (rename 0028 → 0030:
        # удаление 0028 в develop не коллизия, если 0028-х в PR changes).
        while IFS= read -r df; do
            [ -z "$df" ] && continue
            # Файл в develop: "NNNN-name.md". В PR: "docs/adr/NNNN-name.md".
            if ! printf '%s' "$pr_files_json" | grep -qF "docs/adr/${df}"; then
                clashing="${clashing}${df}, "
            fi
        done <<< "$dev_files"
        if [ -n "$clashing" ]; then
            collision_detail="${collision_detail}${nnnn} (clashes: ${clashing%, }), "
        fi
    done <<< "$pr_new_adrs"

    if [ -z "$collision_detail" ]; then
        return 0  # нет коллизии — guard не срабатывает
    fi

    # Коллизия → блок. Логируем в merge-gate журнал.
    log "🚨 issue #${number} PR #${pr_number} ADR-COLLISION: ${collision_detail% ,} — block needs-e2e"

    if [ "$DRY_RUN" = "true" ]; then
        log "DRY-RUN would: post ADR-collision comment + label ${ADR_COLLISION_BLOCKED_LABEL} on issue #${number}"
        return 1
    fi

    # 24h dedup (как big-bang блок) — merge-gate тикает каждые ~5-10 мин,
    # без dedup было бы ~144 одинаковых спам-коммента в день.
    local _ac_dedup_since
    _ac_dedup_since="$(date -u -d "${ADR_COLLISION_COMMENT_DEDUP_HOURS} hours ago" +%Y-%m-%dT%H:%M:%SZ 2>/dev/null \
        || date -u +%Y-%m-%dT%H:%M:%SZ)"
    local _ac_dup_count
    _ac_dup_count="$(gh api "repos/${GH_REPO}/issues/${number}/comments?since=${_ac_dedup_since}&per_page=100" \
        --jq '[.[] | select(.body | contains("ADR-COLLISION detected"))] | length' 2>/dev/null || echo 0)"
    if [ "${_ac_dup_count:-0}" -eq 0 ] 2>/dev/null; then
        gh issue comment "$number" --repo "$GH_REPO" --body \
            "🚨 **PR #${pr_number} ADR-COLLISION detected** (merge-gate, ретро 25.08 t_00ba0224, $(date -u +%H:%M:%SZ))

PR добавляет/переименовывает ADR с номерами, которые **уже заняты** другими файлами в \`origin/develop\`:

${collision_detail% ,}

**Что делать:**
1. **Переименовать** свой файл на следующий свободный номер (проверка: \`git ls-tree origin/develop --name-only | grep -E '^docs/adr/NNNN-'\`).
2. **Либо** Шифу ставит override: \`${ADR_COLLISION_OVERRIDE_LABEL}\` на этот issue (явный re-numbering).

Merge-gate **НЕ поставит ${NEEDS_E2E_LABEL}** пока коллизия не разрешена. Линтер/доки коммитятся отдельным PR'ом — глобальная коллизия номеров ломает обратные ссылки на ADR.

Ссылка: CONTRIBUTING.md (раздел ADR-процесс), ADR-0001." >/dev/null 2>&1 \
            && log "issue #${number}: ADR-collision comment posted (${ADR_COLLISION_COMMENT_DEDUP_HOURS}h dedup)" \
            || log "WARNING: ADR-collision comment post failed for issue #${number}"
    else
        log "issue #${number}: ADR-collision comment уже проставлен (×${_ac_dup_count} за ${ADR_COLLISION_COMMENT_DEDUP_HOURS}ч) — dedup skip"
    fi

    # Метка на issue (best-effort). Аналог agent-flow:big-bang-blocked.
    gh issue edit "$number" --repo "$GH_REPO" --add-label "$ADR_COLLISION_BLOCKED_LABEL" >/dev/null 2>&1 \
        && log "issue #${number}: ${ADR_COLLISION_BLOCKED_LABEL} added" \
        || log "WARNING: failed to add ${ADR_COLLISION_BLOCKED_LABEL} to issue #${number}"

    return 1  # КОЛЛИЗИЯ → caller продолжает main-cycle без needs-e2e
}

# --- process each issue ------------------------------------------------------

# free_stale_worktrees_for — перенесена в lib_agent_flow_common.sh (дедуп 30.08).

considered=0
labeled=0
skipped=0
errored=0
human_close_propagated=0

while IFS=$'\t' read -r number title labels body; do
    [ -z "$number" ] && continue
    considered=$((considered+1))

    labels_norm="$(printf '%s' "$labels" | tr '[:upper:]' '[:lower:]')"

    # --- взаимоисключение needs-review / needs-e2e (ретро 13.08 t_de63be1f, #942) ---
    # Обе метки одновременно на ISSUE — конфликт: needs-review (ждёт юзера на
    # ревью) и needs-e2e (ждёт e2e-ротации) несовместимы. merge-gate ставит
    # needs-review ТОЛЬКО на PR; needs-review на ISSUE — внешнее/ручное
    # решение (юзер уже смотрит) → issue вне e2e-ротации: снимаем needs-e2e и
    # НЕ трогаем issue в этом тике (иначе ниже functional-PR снова навесит
    # needs-e2e и конфликт вернётся следующим тиком). Чиним ДО проверки
    # kanban-маркера, чтобы лечились и issue без маркера (#942 — конфликт
    # ровно из-за этого: merge-gate скипал её как «triage not finished»).
    if has_label "$labels_norm" "$NEEDS_REVIEW_LABEL"; then
        if has_label "$labels_norm" "$NEEDS_E2E_LABEL"; then
            log "issue #${number}: конфликт ${NEEDS_REVIEW_LABEL}+${NEEDS_E2E_LABEL} — снимаю ${NEEDS_E2E_LABEL} (needs-review приоритетнее)"
            if [ "$DRY_RUN" != "true" ]; then
                gh issue edit "$number" --repo "$GH_REPO" --remove-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
            fi
        fi
        log "issue #${number}: ${NEEDS_REVIEW_LABEL} на issue — юзер ревьюит, вне e2e-ротации — skip"
        skipped=$((skipped+1)); continue
    fi

    # Look up the kanban card id from the comment marker. We don't need the
    # card itself here, but its presence is the contract that Phase 1
    # finished and we have a branch convention to rely on.
    task_id="$(gh issue view "$number" --repo "$GH_REPO" --comments --json comments \
        --jq '.comments[].body' 2>/dev/null \
        | grep -Eo '^kanban: t_[a-f0-9]+' \
        | tail -n1 \
        | sed 's/^kanban: //' || true)"

    if [ -z "$task_id" ]; then
        log "issue #${number} has no kanban marker — triage not finished yet — skip"
        skipped=$((skipped+1)); continue
    fi

    # Derive the expected PR head branch.
    # Note: triage.sh uses branch `z-{agent}/<issue>-<slug>` for hermes+default-role
    # issues; service/infra labels get `z-{<slug>}` prefix. We only consider
    # `z-{agent}/<id>-<slug>` here because that's the path that goes through the
    # e2e pipeline. Service/infra branches are off-flow.
    labels_json="$labels"
    is_service=0
    if printf '%s' "$labels_json" | grep -Eq 'service:|infra:|ops:'; then
        is_service=1
    fi
    if [ "$is_service" -eq 1 ]; then
        log "issue #${number} is service/infra — out of merge-gate scope — skip"
        skipped=$((skipped+1)); continue
    fi

    # --- follow-up PR поверх e2e-done (ретро 10.08, архитектор) ------------
    # Раньше e2e-done «намертво» прилипал к issue: idempotency-скип ниже
    # пропускал её, и follow-up PR по той же issue (#1099 поверх #1082,
    # #1098 поверх #1052) навсегда выпадал из ротации — CLEAN PR висел
    # без needs-review/needs-e2e, фикс не доходил до робота.
    # Теперь: если issue e2e-done, но есть ДРУГОЙ OPEN PR с номером issue
    # в title (ветка ≠ канонической merged-ветке) — снимаем e2e-done →
    # needs-e2e, e2e-process возьмёт новую ветку в ротацию.
    # Guard от ping-pong: флипаем ТОЛЬКО если в follow-up PR есть коммиты
    # ПОСЛЕ момента навешивания e2e-done (иначе — уже протестирован, и
    # e2e-done → needs-e2e → e2e-done зациклится каждый тик).
    if has_label "$labels_norm" "$DONE_LABEL"; then
        _followup_json="$(gh pr list --repo "$GH_REPO" --state open \
            --search "${number} in:title" \
            --json number,headRefName,mergeStateStatus,updatedAt \
            --jq '[.[] | select(.mergeStateStatus == "CLEAN" or .mergeStateStatus == "MERGEABLE")][0]' 2>/dev/null || echo "")"
        _followup_pr="$(printf '%s' "$_followup_json" | python3 -c 'import sys,json
try:
    d=json.load(sys.stdin); print(d.get("number","") if d else "")
except Exception: print("")' 2>/dev/null || true)"
        if [ -n "$_followup_pr" ] && [ "$_followup_pr" != "null" ]; then
            _done_at="$(gh api "repos/${GH_REPO}/issues/${number}/timeline?per_page=100" \
                --jq '[.[] | select(.event=="labeled" and .label.name=="'"$DONE_LABEL"'")][-1].created_at' 2>/dev/null || echo '')"
            if [ -z "$_done_at" ] || [ "$_done_at" = "null" ]; then
                log "issue #${number}: follow-up PR #${_followup_pr} найден, но timeline ${DONE_LABEL} недоступен — пропускаем тик (без риска ping-pong)"
                skipped=$((skipped+1)); continue
            fi
            _new_commits="$(gh api "repos/${GH_REPO}/pulls/${_followup_pr}/commits?per_page=100" \
                --jq '[.[] | select(.commit.committer.date >= "'"$_done_at"'")] | length' 2>/dev/null || echo 0)"
            if [ "${_new_commits:-0}" -gt 0 ] 2>/dev/null; then
                log "issue #${number}: follow-up OPEN PR #${_followup_pr} поверх ${DONE_LABEL} (new commits after ${_done_at}) — возврат в ротацию"
                if [ "$DRY_RUN" != "true" ]; then
                    gh issue edit "$number" --repo "$GH_REPO" --remove-label "$DONE_LABEL" >/dev/null 2>&1 || true
                    gh issue edit "$number" --repo "$GH_REPO" --add-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
                    gh issue comment "$number" --repo "$GH_REPO" --body \
                        "agent-flow: 🔄 follow-up PR #${_followup_pr} поверх ${DONE_LABEL} — снят ${DONE_LABEL}, поставлен ${NEEDS_E2E_LABEL}. e2e-process протестирует новую ветку." >/dev/null 2>&1 || true
                fi
                labeled=$((labeled+1)); continue
            fi
            log "issue #${number}: follow-up PR #${_followup_pr} без новых коммитов после ${DONE_LABEL} (${_done_at}) — уже протестирован, skip"
        fi
    fi

    # --- PR↔issue e2e-done drift reconcile (ретро 19.08 t_5cde0bc1) --------
    # Гипотеза PR #1398 / issue #1392: issue имеет needs-e2e (в ротации), но
    # канонический PR (ветка z-{agent}/<id>-<slug>) висит с e2e-done от
    # предыдущего раунда. Без reconcile e2e-process на следующем тике видит
    # issue:needs-e2e, подбирает PR, проходит — снова вешает PR:e2e-done (тот
    # же результат), но e2e-done «прилипает» к PR между ручным возвратом
    # krikz и e2e-раундом → drift в PR-очереди Шифу (видит «готово к ревью»
    # хотя по issue ничего не сделано).
    #
    # Reconcile: issue:needs-e2e + канонический PR:e2e-done (или :needs-review)
    # → снимаем PR-side stale метки, оставляя issue-side неизменным. После
    # этого e2e-process спокойно проходит, вешает обратно e2e-done + needs-review
    # на PR уже с реальным результатом раунда.
    #
    # Guard от ping-pong: срабатываем ТОЛЬКО когда в issue timeline последнее
    # событие по e2e-done — UNLABEL (т.е. кто-то руками или автоматика сняла
    # метку и она не была возвращена — нормальное состояние issue в ротации).
    # Если последнее событие — LABEL → e2e-done свежее, e2e-process сейчас
    # разрулит, не трогаем PR.
    if has_label "$labels_norm" "$NEEDS_E2E_LABEL"; then
        _drift_branch_pattern="z-{agent}/${number}-"
        # Ищем канонический PR по headRefName-префиксу (быстрее, чем title-scan).
        # gh pr list --state open + grep в shell — не зависит от jq-паттернов
        # и совместимо с mock-окружением тестов.
        _drift_pr_json="$(gh pr list --repo "$GH_REPO" --state open \
            --search "${number} in:title" \
            --json number,headRefName 2>/dev/null || echo '[]')"
        _drift_pr_number="$(printf '%s' "$_drift_pr_json" | grep -F "$_drift_branch_pattern" \
            | grep -oE '"number":[0-9]+' | head -n1 | grep -oE '[0-9]+' || true)"
        if [ -n "$_drift_pr_number" ]; then
            _drift_pr_labels_csv="$(gh pr view "$_drift_pr_number" --repo "$GH_REPO" --json labels \
                --jq '[.labels[].name] | join(",")' 2>/dev/null || echo '')"
            _drift_pr_labels_norm="$(printf '%s' "$_drift_pr_labels_csv" | tr '[:upper:]' '[:lower:]')"
            _drift_has_pr_done="0"; _drift_has_pr_review="0"
            if has_label "$_drift_pr_labels_norm" "$DONE_LABEL"; then _drift_has_pr_done="1"; fi
            if has_label "$_drift_pr_labels_norm" "$NEEDS_REVIEW_LABEL"; then _drift_has_pr_review="1"; fi
            # Проверяем timeline issue: последнее e2e-done событие — unlabel?
            _issue_done_last_evt="$(gh api "repos/${GH_REPO}/issues/${number}/timeline?per_page=100" \
                --jq '[.[] | select(.event=="labeled" or .event=="unlabeled") | select(.label.name=="'"$DONE_LABEL"'")] | last | .event // "none"' 2>/dev/null || echo 'none')"
            if [ "$_drift_has_pr_done" = "1" ] && [ "$_issue_done_last_evt" = "unlabeled" ]; then
                _pr_last_commit_iso="$(gh api "repos/${GH_REPO}/pulls/${_drift_pr_number}/commits?per_page=1" \
                    --jq '.[0].commit.committer.date // empty' 2>/dev/null || echo '')"
                _drift_minutes="n/a"
                if [ -n "$_pr_last_commit_iso" ] && [ "$_pr_last_commit_iso" != "null" ]; then
                    _now_s="$(date -u +%s)"
                    _commit_s="$(date -u -d "$_pr_last_commit_iso" +%s 2>/dev/null || echo 0)"
                    if [ "$_commit_s" -gt 0 ] 2>/dev/null; then
                        _diff_s=$(( _now_s - _commit_s ))
                        _drift_minutes="$(( _diff_s / 60 ))m"
                    fi
                fi
                # Метрика: минут с последнего коммита PR (e2e_drift_minutes) — для watchdog/логов.
                log "issue #${number}: PR↔issue e2e-done drift — issue в ротации (${NEEDS_E2E_LABEL}), но PR #${_drift_pr_number} висит с ${DONE_LABEL} (last issue-evt=unlabeled, last PR commit=${_pr_last_commit_iso}). e2e_drift_minutes=${_drift_minutes} (ретро 19.08 t_5cde0bc1)."
                if [ "$DRY_RUN" != "true" ]; then
                    gh pr edit "$_drift_pr_number" --repo "$GH_REPO" --remove-label "$DONE_LABEL" >/dev/null 2>&1 || true
                    # needs-review на PR снимаем только если есть — иначе PR остался
                    # в «готово к ревью» состоянии с непротестированным кодом.
                    if [ "$_drift_has_pr_review" = "1" ]; then
                        gh pr edit "$_drift_pr_number" --repo "$GH_REPO" --remove-label "$NEEDS_REVIEW_LABEL" >/dev/null 2>&1 || true
                    fi
                    gh pr comment "$_drift_pr_number" --repo "$GH_REPO" --body \
                        "agent-flow: 🧹 e2e-done drift reconcile (ретро 19.08 t_5cde0bc1) — issue #${number} вернулся в ротацию (last issue:${DONE_LABEL}-evt=unlabeled), но PR #${_drift_pr_number} висел с \`${DONE_LABEL}\`. Сняты: \`${DONE_LABEL}\`${_drift_has_pr_review:+, \`${NEEDS_REVIEW_LABEL}\`}. Следующий e2e-раунд перевесит по реальному результату." >/dev/null 2>&1 || true
                    gh issue comment "$number" --repo "$GH_REPO" --body \
                        "agent-flow: 🧹 e2e-done drift reconcile — PR #${_drift_pr_number} снят \`${DONE_LABEL}\` (issue был в ротации, PR-side stale). e2e-process перевзвесит на следующем тике (ретро 19.08 t_5cde0bc1)." >/dev/null 2>&1 || true
                fi
                # НЕ continue — дальше пойдёт нормальный блок needs-e2e processing
                # для этой issue (выставит needs-e2e на канонический PR если ещё нет).
            elif [ "$_drift_has_pr_done" = "1" ]; then
                log "issue #${number}: PR↔issue e2e-done check — issue в ротации, PR #${_drift_pr_number} имеет ${DONE_LABEL}, но last issue-evt=${_issue_done_last_evt} (не unlabeled) → НЕ снимаю, e2e-process разрулит (ретро 19.08 t_5cde0bc1)."
            fi
        fi
    fi

    branch="z-{agent}/${number}-$(slugify "$title")"

    # Look up PR by head branch (authoritative — no extra state).
    # title + labels нужны для detect_pr_kind (lint vs functional).
    # additions + commits нужны для big-bang-override gate (ADR-0013).
    pr_json="$(gh pr list \
        --repo "$GH_REPO" \
        --state all \
        --head "$branch" \
        --json number,mergeable,mergeStateStatus,statusCheckRollup,baseRefName,state,mergedAt,title,labels,additions,deletions,commits 2>/dev/null || true)"

    # Процесс-фикс (09.08): воркеры ретро-карточек создают ветки `wt/<task_id>`
    # (нет issue → конвенция z-{agent}/<id>-<slug> неприменима). Такие PR
    # выпадали из конвейера: merge-gate не находил их и не ставил needs-e2e.
    # Fallback: ищем PR по ветке wt/<task_id> (последняя карточка issue).
    if [ -z "$pr_json" ] || [ "$pr_json" = "[]" ]; then
        if [ -n "$task_id" ]; then
            wt_branch="wt/${task_id}"
            pr_json="$(gh pr list \
                --repo "$GH_REPO" \
                --state all \
                --head "$wt_branch" \
                --json number,mergeable,mergeStateStatus,statusCheckRollup,baseRefName,state,mergedAt,title,labels,additions,deletions,commits 2>/dev/null || true)"
            if [ -n "$pr_json" ] && [ "$pr_json" != "[]" ]; then
                branch="$wt_branch"
                log "issue #${number}: PR найден по fallback-ветке ${wt_branch}"
            else
                pr_json=""
            fi
        fi
    fi

    if [ -z "$pr_json" ] || [ "$pr_json" = "[]" ]; then
        log "issue #${number}: no open PR for branch ${branch} yet — skip"
        skipped=$((skipped+1)); continue
    fi

    # Parse the single PR record (there's expected to be just one).
    eval "$(printf '%s' "$pr_json" | python3 -c '
import json, sys, shlex
data = json.load(sys.stdin)
if not data:
    sys.exit(0)
pr = data[0]
rollup = pr.get("statusCheckRollup") or []
# Green = no FAILURE-class conclusions; SKIPPED/NEUTRAL/pending are not red.
# (Integration Tests is often SKIPPED — that must NOT block the gate.)
no_failure = all(
    e.get("conclusion") not in ("FAILURE", "CANCELLED", "TIMED_OUT", "STALE")
    for e in rollup
)
all_pass = bool(rollup) and no_failure
pr_number = str(pr.get("number", ""))
pr_base = str(pr.get("baseRefName", ""))
pr_state = str(pr.get("state", ""))
pr_mergeable = str(pr.get("mergeable", ""))
pr_merge_state = str(pr.get("mergeStateStatus", ""))
pr_rollup_count = len(rollup)
pr_rollup_pass = 1 if all_pass and no_failure else 0
# title + labels_csv для detect_pr_kind (lint vs functional, ретро 10.08 #2)
pr_title = str(pr.get("title", ""))
pr_labels_csv = ",".join(sorted(
    {str(lab.get("name", "")) for lab in (pr.get("labels") or [])}
))
# Размер PR — для big-bang-override gate (ADR-0013). commits в API —
# это СПИСОК (не int), поэтому берём len(). additions — int напрямую.
# Если поле отсутствует (старый API), считаем 0 → gate не сработает
# (no PR, no override needed).
pr_commits_count = len(pr.get("commits") or [])
pr_additions = int(pr.get("additions") or 0)
pr_deletions = int(pr.get("deletions") or 0)
pr_head_oid = str(pr.get("headRefOid", "") or "")
print(f"pr_number={shlex.quote(pr_number)}")
print(f"pr_base={shlex.quote(pr_base)}")
print(f"pr_deletions={pr_deletions}")
print(f"pr_state={shlex.quote(pr_state)}")
print(f"pr_mergeable={shlex.quote(pr_mergeable)}")
print(f"pr_merge_state={shlex.quote(pr_merge_state)}")
print(f"pr_rollup_count={pr_rollup_count}")
print(f"pr_rollup_pass={pr_rollup_pass}")
print(f"pr_title={shlex.quote(pr_title)}")
print(f"pr_labels_csv={shlex.quote(pr_labels_csv)}")
print(f"pr_commits_count={pr_commits_count}")
print(f"pr_additions={pr_additions}")
print(f"pr_head_oid={shlex.quote(pr_head_oid)}")
')"

    if [ -z "${pr_number:-}" ]; then
        log "issue #${number}: could not parse PR record for ${branch} — skip"
        errored=$((errored+1)); continue
    fi

    # --- stale-branch re-commit guard (ретро 12.08 t_d3aeaa9b) ------------
    # Сценарий: ветка УЖЕ влита в develop (есть MERGED PR с тем же head), но
    # воркер продолжал коммитить в неё (база устарела; re-коммиты = дубли
    # merged-содержимого) и открыл НОВЫЙ PR с той же ветки. Diff такого PR
    # vs develop = РЕГРЕССИЯ (удаляет влитые voice-фиксы: dialogue_node.py,
    # health.py, .image-versions). Детект: для head-ветки уже есть MERGED PR,
    # отличный от текущего. Блокируем: коммент в issue + НЕ ставим needs-e2e,
    # чтобы регрессионный дифф не ушёл в e2e-ротацию и merge.
    if [ "$pr_state" = "OPEN" ]; then
        _prev_merged_pr="$(gh pr list --repo "$GH_REPO" --state merged --head "$branch" \
            --json number --jq '.[0].number // ""' 2>/dev/null || true)"
        if [ -n "$_prev_merged_pr" ] && [ "$_prev_merged_pr" != "$pr_number" ]; then
            # Ретро 13.08 t_a3f170fe: guard бьёт по ИМЕНИ ветки, но аддитивный
            # PR (docs/ci, deletions≈0) — НЕ регрессия: ветка влита, воркер
            # до-пушил НОВЫЙ контент (#1197 docs W7: +308/-1). Регрессия =
            # удаление влитых фиксов → deletions значимы. Блокируем только её.
            if [ "${pr_deletions:-0}" -le 20 ] 2>/dev/null; then
                # Ретро 14.08 t_28afb585: как в stale_branch_scan_all — аддитивный
                # PR на влитой ветке разрешён только для docs/ci-продолжения;
                # функциональные файлы на уже влитой ветке = переиспользование
                # ветки влитого PR (#1238) → блок + снятие needs-review.
                _pr_func="$(pr_has_functional_files "$pr_number")"
                if [ "$_pr_func" = "1" ]; then
                    log "issue #${number}: 🛑 ветка ${branch} влита через PR #${_prev_merged_pr}, PR #${pr_number} аддитивный, но несёт ФУНКЦИОНАЛЬНЫЕ файлы — block (ретро 14.08 t_28afb585)"
                    if [ "$DRY_RUN" = "true" ]; then
                        log "DRY-RUN would: add ${STALE_BRANCH_REUSE_LABEL} + comment stale-branch block + remove needs-review on PR #${pr_number}"
                        skipped=$((skipped+1)); continue
                    fi
                    _stale_dedup_since="$(date -u -d '24 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
                    _stale_dup="$(gh api "repos/${GH_REPO}/issues/${number}/comments?since=${_stale_dedup_since}&per_page=100" \
                        --jq '[.[] | select(.body | startswith("🛑 **stale-branch reuse"))] | length' 2>/dev/null || echo 0)"
                    if [ "${_stale_dup:-0}" -eq 0 ]; then
                        gh issue comment "$number" --repo "$GH_REPO" --body \
                            "🛑 **stale-branch reuse with new functional fix** (merge-gate, ретро 14.08 t_28afb585, метка ретро 31.08 t_04371252)

Ветка \`${branch}\` уже была влита в develop через PR #${_prev_merged_pr}. PR #${pr_number} аддитивный, НО несёт НОВЫЕ функциональные фиксы (docker/, src/, scripts/agent_flow/ и т.п.) поверх уже влитой ветки — переиспользование ветки влитого PR (повтор паттерна #1238/#1218, #1753).

**Что делать:**
1. Создай **новую** ветку от свежего origin/develop: \`git fetch origin develop && git checkout -b z-{agent}/t_<card>-<slug> origin/develop\`.
2. Перенеси ТОЛЬКО этот фикс (cherry-pick/rebase), один PR = одна тема.
3. Закрой/удали этот PR и открой новый с новой ветки.
4. needs-review ставится только после e2e-прогона PR.

Снято: \`needs-review\` (поставлен без e2e). Merge-gate **не поставит needs-e2e** на PR с уже влитой ветки и поставил метку \`${STALE_BRANCH_REUSE_LABEL}\` на PR." >/dev/null 2>&1 || true
                    fi
                    gh pr edit "$pr_number" --repo "$GH_REPO" --remove-label "$NEEDS_REVIEW_LABEL" >/dev/null 2>&1 || true
                    # Ретро 31.08 t_04371252 (PR #1753): маркируем PR для downstream.
                    _pr_proc="$(pr_has_process_changes "$pr_number")"
                    _pr_proc_msg="аддитивный функциональный PR на влитой ветке (ретро 14.08 t_28afb585)"
                    if [ "$_pr_proc" = "1" ]; then
                        _pr_proc_msg="${_pr_proc_msg} + меняет процессные скрипты scripts/agent_flow/ или tests/agent_flow/ (ретро 31.08 t_04371252)"
                    fi
                    whoami_add_label "$pr_number" "$STALE_BRANCH_REUSE_LABEL" \
                        "${_pr_proc_msg}" \
                        "branch=${branch}" "merged_via_pr=#${_prev_merged_pr}" || log "issue #${number}: WARNING add ${STALE_BRANCH_REUSE_LABEL} on PR #${pr_number} failed (non-fatal)"
                    skipped=$((skipped+1)); continue
                else
                    log "issue #${number}: ветка ${branch} влита через PR #${_prev_merged_pr}, но PR #${pr_number} аддитивный docs/ci (del=${pr_deletions:-0}) — НЕ регрессия, не блокируем (ретро 13.08 t_a3f170fe)"
                fi
            else
            log "issue #${number}: 🛑 stale-branch re-commit — ветка ${branch} уже влита через PR #${_prev_merged_pr}, PR #${pr_number} снова OPEN — block"
            if [ "$DRY_RUN" = "true" ]; then
                log "DRY-RUN would: add ${STALE_BRANCH_REUSE_LABEL} + comment stale-branch block on issue #${number}"
                skipped=$((skipped+1)); continue
            fi
            _stale_dedup_since="$(date -u -d '24 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
            _stale_dup="$(gh api "repos/${GH_REPO}/issues/${number}/comments?since=${_stale_dedup_since}&per_page=100" \
                --jq '[.[] | select(.body | startswith("🛑 **stale-branch re-commit"))] | length' 2>/dev/null || echo 0)"
            if [ "${_stale_dup:-0}" -eq 0 ]; then
                gh issue comment "$number" --repo "$GH_REPO" --body \
                    "🛑 **stale-branch re-commit detected** (merge-gate, ретро 12.08 t_d3aeaa9b, метка ретро 31.08 t_04371252)

Ветка \`${branch}\` уже была влита в develop через PR #${_prev_merged_pr}. Новые коммиты в неё ПОСЛЕ merge — re-коммиты поверх устаревшей базы: diff origin/develop...HEAD **удаляет** уже влитые фиксы (voice: dialogue_node.py, health.py; .image-versions).

**Что делать:**
1. НЕ пушить в уже влитую ветку.
2. Создай **новую** ветку от свежего origin/develop: \`git fetch origin develop && git checkout -b <новая-ветка> origin/develop\`.
3. Перенеси нужные изменения (rebase/cherry-pick), открой новый PR.
4. Закрой/удали этот PR (ветка влита, PR-дифф регрессионный).

Merge-gate **не поставит needs-e2e** на PR с уже влитой ветки и поставил метку \`${STALE_BRANCH_REUSE_LABEL}\` на PR." >/dev/null 2>&1 || true
            fi
            # Ретро 31.08 t_04371252: маркер для downstream на регрессионном пути.
            whoami_add_label "$pr_number" "$STALE_BRANCH_REUSE_LABEL" \
                "stale-branch re-commit (merge-gate, ретро 12.08 t_d3aeaa9b): ветка уже влита через PR #${_prev_merged_pr}" \
                "branch=${branch}" "merged_via_pr=#${_prev_merged_pr}" || log "issue #${number}: WARNING add ${STALE_BRANCH_REUSE_LABEL} on PR #${pr_number} failed (non-fatal)"
            skipped=$((skipped+1)); continue
            fi
        fi
    fi

    # --- human-close propagation (ретро 22.08, PR #1516) --------------------
    # Шифу закрыл PR вручную (комментарий + close) — раньше это ни на что не
    # влияло: issue оставалась OPEN с hermes/needs-e2e, воркер открывал новый
    # PR (#1507→#1516), задача возвращалась бесконечно. Теперь: читаем причину
    # из последнего комментария PR и выводим задачу из автоматического цикла
    # (снимаем hermes + needs-e2e — триггеры triage/e2e-process).
    if [ "$pr_state" = "CLOSED" ]; then
        if has_label "$labels_norm" "$ISSUE_LABEL" || has_label "$labels_norm" "$NEEDS_E2E_LABEL"; then
            _reason="$(gh pr view "$pr_number" --repo "$GH_REPO" --comments --json comments 2>/dev/null \
                | python3 -c 'import sys,json
try:
    d=json.load(sys.stdin); cs=d.get("comments") or []
    print(cs[-1].get("body","") if cs else "")
except Exception:
    print("")' 2>/dev/null || true)"
            # Guard от ложных срабатываний: если PR закрыт ПРОЦЕССОМ (orphan-dead,
            # dead-content, e2e-доклад и т.п.) — причина в последнем комментарии
            # содержит process-маркер → не трогаем (процесс сам разберётся).
            case "$_reason" in
                *orphan-dead*|*dead-content*|*e2e-доклад*|*rebase*|agent-flow:*|🪦*)
                    log "issue #${number}: PR #${pr_number} CLOSED процессом (маркер) — human-close propagation skip"
                    ;;
                *)
                    log "issue #${number}: PR #${pr_number} CLOSED вручную — вывод задачи из цикла (снимаю ${ISSUE_LABEL}/${NEEDS_E2E_LABEL})"
                    if [ "$DRY_RUN" != "true" ]; then
                        gh issue edit "$number" --repo "$GH_REPO" --remove-label "$ISSUE_LABEL" >/dev/null 2>&1 || true
                        gh issue edit "$number" --repo "$GH_REPO" --remove-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
                        gh issue comment "$number" --repo "$GH_REPO" --body \
"🛑 **PR #${pr_number} закрыт вручную** (merge-gate human-close propagation, ретро 22.08).

Задача выведена из автоматического цикла: сняты \`${ISSUE_LABEL}\` / \`${NEEDS_E2E_LABEL}\`. Воркер не откроет новый PR по этой issue.

Причина (из комментария Шифу на PR):
> ${_reason:-не указана — см. PR #${pr_number}}" >/dev/null 2>&1 || true
                    fi
                    human_close_propagated=$((human_close_propagated+1))
                    ;;
            esac
        fi
        continue
    fi

    # --- e2e-done + OPEN PR → needs-review (ретро 13.08 t_92ec94f3, Q22) ---
    # Разрыв после e2e-done: post_round_sweep в e2e-process (и прерванная
    # wait-фаза основного цикла) ставили e2e-done на issue, но НЕ ставили
    # needs-review на PR. Дальше issue «молчит»: e2e-process скипает её
    # (e2e-done), мы тоже скипали (e2e-done, см. idempotency ниже) →
    # товарищ Шифу не видит PR в очереди ревью (наблюдение 12.08:
    # #929/#933 e2e-done без needs-review; kanban: все карточки done).
    # Reconcile (5m loop): e2e-done + OPEN PR (base=develop) → ставим
    # needs-review на PR + снимаем needs-e2e с PR (если осталась от старого
    # цикла). Идемпотентно: повторный тик add-label — no-op. НЕ трогаем
    # MERGED PR — их закрывает post-merge reconcile (ADR-0014) ниже.
    #
    # Ретро 14.08 t_28afb585 (пункт 4): needs-review только если PR реально
    # протестирован — PR создан ДО момента навешивания e2e-done. Если PR
    # создан ПОСЛЕ e2e-done (переиспользование ветки/новый PR поверх уже
    # протестированного состояния), e2e-раунд его НЕ покрывал → не ставим
    # needs-review, возвращаем в e2e-ротацию (needs-e2e) вместо «тихого»
    # ревью непротестированного кода.
    if has_label "$labels_norm" "$DONE_LABEL" && [ "$pr_state" = "OPEN" ]; then
        # ADR-0018: hint на голословный PASS в PR body (не блокер, только лог).
        honesty_hint_for_pr "$pr_number" || true
        _done_at="$(gh api "repos/${GH_REPO}/issues/${number}/timeline?per_page=100" \
            --jq '[.[] | select(.event=="labeled" and .label.name=="'"$DONE_LABEL"'")][-1].created_at' 2>/dev/null || echo '')"
        _pr_created="$(gh pr view "$pr_number" --repo "$GH_REPO" --json createdAt \
            --jq '.createdAt' 2>/dev/null || echo '')"
        if [ -n "$_done_at" ] && [ -n "$_pr_created" ] && [ "$_done_at" != "null" ] && [ "$_pr_created" != "null" ] \
            && [ "$_pr_created" \> "$_done_at" ] 2>/dev/null; then
            log "issue #${number}: ${DONE_LABEL} (${_done_at}) РАНЬШЕ создания PR #${pr_number} (${_pr_created}) — PR не тестировался, needs-review НЕ ставлю, возврат в ${NEEDS_E2E_LABEL} (ретро 14.08 t_28afb585)"
            if [ "$DRY_RUN" != "true" ]; then
                gh issue edit "$number" --repo "$GH_REPO" --remove-label "$DONE_LABEL" >/dev/null 2>&1 || true
                gh issue edit "$number" --repo "$GH_REPO" --add-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
                gh pr edit "$pr_number" --repo "$GH_REPO" --remove-label "$NEEDS_REVIEW_LABEL" >/dev/null 2>&1 || true
                gh issue comment "$number" --repo "$GH_REPO" --body \
                    "agent-flow: ⏪ e2e-done снят — PR #${pr_number} создан ПОСЛЕ последнего e2e-раунда (${_done_at}); возврат в needs-e2e, следующий тик протестирует новую ветку (ретро 14.08 t_28afb585)." >/dev/null 2>&1 || true
            fi
            labeled=$((labeled+1)); continue
        fi
        log "issue #${number}: ${DONE_LABEL} + OPEN PR #${pr_number} → reconcile ${NEEDS_REVIEW_LABEL}"
        # --- user-unlabel guard (ретро 18.08 t_de6bea69, PR #1398) -----------
        # Если Шифу руками снял needs-review после последнего auto-установки —
        # reconcile НЕ должен возвращать метку. remove needs-e2e — идемпотентно
        # оставляем (Шифу его не трогает).
        if user_removed_label_recently "$pr_number" "$NEEDS_REVIEW_LABEL"; then
            user_unlabel_log_skip "$pr_number" "$NEEDS_REVIEW_LABEL" "merge-gate reconcile (e2e-done+OPEN)"
            if [ "$DRY_RUN" != "true" ] && user_unlabel_should_notify "$pr_number" "$NEEDS_REVIEW_LABEL"; then
                gh pr comment "$pr_number" --repo "$GH_REPO" --body \
                    "agent-flow: ⏸️ merge-gate reconcile не восстановил \`needs-review\` — ты её ранее снял руками; жду твоего решения (ретро 18.08 t_de6bea69, Q22)." >/dev/null 2>&1 || true
                user_unlabel_mark_notified "$pr_number" "$NEEDS_REVIEW_LABEL" "merge-gate reconcile (e2e-done+OPEN)" || true
            fi
        elif [ "$DRY_RUN" != "true" ]; then
            # issue #1553: self-id whoami BEFORE add-label needs-review на PR
            # (reconcile-путь e2e-done+OPEN → needs-review).
            post_whoami_comment pr "$pr_number" "adding-label:${NEEDS_REVIEW_LABEL}" \
                "merge-gate reconcile: e2e-done+OPEN → needs-review" "issue=${number}"
            gh pr edit "$pr_number" --repo "$GH_REPO" --add-label "$NEEDS_REVIEW_LABEL" >/dev/null 2>&1 || true
            gh pr edit "$pr_number" --repo "$GH_REPO" --remove-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
        fi
        labeled=$((labeled+1)); continue
    fi

    # MERGED (Q22 done manually by user): post-merge reconciliation per
    # ADR-0014 (docs/adr/0014-agent-flow-issue-closure.md).
    #
    # Invariant: issue may close <=> PR MERGED into develop AND issue has
    # e2e-done produced by e2e-process (not by merge-gate itself). Race:
    # e2e-process may set e2e-done after we observed initial labels — so
    # we re-read labels RIGHT BEFORE close. Order: (1) re-read labels +
    # state, (2) close issue if e2e-done, (3) only on success run
    # destructive cleanup (delete branch, free worktrees, archive card,
    # cleanup comment, drop stale labels). Close failure → warning,
    # destructive cleanup is deferred, retry next 5m tick.
    if [ "$pr_state" = "MERGED" ] && [ "$pr_base" = "$DEVELOP_BRANCH" ]; then
        log "issue #${number}: PR #${pr_number} MERGED into ${pr_base} — post-merge reconcile (ADR-0014)"
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would reconcile issue #${number} (re-read labels, maybe close, then cleanup ${branch})"
            continue
        fi
        # ADR-0022 extension (issue #1475): после merge в develop/main
        # триггерим L-Build-All-Services, чтобы .image-versions.prod получил
        # prod-<new-sha> теги. Non-fatal: build failure НЕ блокирует merge-gate
        # (см. agent-flow-post-merge-build.sh).
        #
        # Issue #1625 (Шифу 25.08): develop build больше не триггерим
        # автоматически — develop-HEAD собирается вручную или push-триггером
        # L-Build-All-Services.yml. main build ОБЯЗАТЕЛЕН (production safety).
        # Двойная защита: merge-gate guard И post-merge-build.sh skip-блок.
        if [ "$pr_base" = "$DEVELOP_BRANCH" ]; then
            log "issue #${number}: skipping post-merge build for ${pr_base} (Шифу 25.08, issue #1625)"
        elif [ -n "${REPO_DIR:-}" ] && [ -d "$REPO_DIR" ] && [ -f "${REPO_DIR}/scripts/agent_flow/agent-flow-post-merge-build.sh" ]; then
            if ! bash "${REPO_DIR}/scripts/agent_flow/agent-flow-post-merge-build.sh" "${pr_number}" "${pr_base}" 2>/dev/null; then
                log "issue #${number}: WARNING post-merge build trigger failed (non-fatal, push-trigger should retry)"
            fi
        fi
        # 0.1) Re-read current labels & state — race with e2e-process
        # (e2e-process may have set e2e-done between our initial issue-list
        # pull and now; also the issue may already be CLOSED from a previous
        # tick in this same merge cycle, making close a no-op).
        _current_labels_csv="$(gh issue view "$number" --repo "$GH_REPO" --json labels \
            --jq '[.labels[].name] | join(",")' 2>/dev/null || echo '')"
        _current_labels_norm="$(printf '%s' "$_current_labels_csv" | tr '[:upper:]' '[:lower:]')"
        _has_e2e_done="0"
        if has_label "$_current_labels_norm" "$DONE_LABEL"; then
            _has_e2e_done="1"
        fi
        # Retro 19.08 #79779a21 (orphans #1422 + #1456): worker may have
        # opted out of e2e via label `no-e2e-required` (docs/lint/refactor
        # PR per ADR-0022 §4.2). Once the PR is MERGED into develop, the
        # issue must close just like an e2e-done one — otherwise it sits
        # OPEN until manual triage (orphan pattern). Pre-compute so the
        # OPEN-state branch can decide.
        _has_no_e2e="0"
        if has_label "$_current_labels_norm" "$NO_E2E_LABEL"; then
            _has_no_e2e="1"
        fi
        _issue_state="$(gh issue view "$number" --repo "$GH_REPO" --json state \
            --jq '.state' 2>/dev/null || echo '')"
        log "issue #${number}: pre-close state=${_issue_state} e2e-done=${_has_e2e_done} no-e2e=${_has_no_e2e}"

        # 0.1a) Early short-circuit for no-e2e-required (retro 19.08 #79779a21,
        # ADR-0022 §4.2). Worker explicitly opted out of e2e — the PR
        # MERGED into develop is sufficient evidence that the fix landed.
        # We bypass the user-reopen guard below because `no-e2e-required`
        # is itself an explicit worker signal (not a PASS-proven label),
        # and the user-reopen guard is designed to protect e2e-done
        # provenance (which is more fragile — a PASS verdict can be
        # stale). If user explicitly reopens AFTER no-e2e-required close,
        # that's a separate follow-up the user will file themselves
        # (Q22-style).
        #
        # IMPORTANT: we DO NOT continue / skip the case statement below
        # — instead we update _issue_state=CLOSED so the existing CLOSED
        # branch fires (idempotent skip-close, proceed to destructive
        # cleanup). This preserves the post-close flow: branch delete +
        # cleanup comment + kanban card archive.
        if [ "$pr_state" = "MERGED" ] && [ "$pr_base" = "$DEVELOP_BRANCH" ] \
            && [ "$_issue_state" = "OPEN" ] \
            && [ "$_has_e2e_done" = "0" ] \
            && [ "$_has_no_e2e" = "1" ]; then
            # issue #1534: self-id whoami BEFORE close — чтобы в истории
            # GitHub было видно, что close сделал merge-gate, а не krikz
            # (actor = GH token holder). Skip-safe (helper идемпотентный).
            whoami_close_issue "$number" "no-e2e-required path: PR merged into ${DEVELOP_BRANCH}, close per retro 19.08 #79779a21"
            if [ "$DRY_RUN" = "true" ]; then
                log "DRY-RUN would auto-close issue #${number} via ${NO_E2E_LABEL} (retro 19.08 #79779a21)"
                # In DRY-RUN, fall through to the case (will hit OPEN
                # branch but with no-op close).
                _closed_this_tick=1
            elif gh issue close "$number" --repo "$GH_REPO" --reason completed >/dev/null 2>&1; then
                _closed_this_tick=1
                log "issue #${number}: CLOSED (reason=completed, auto-close via ${NO_E2E_LABEL}, retro 19.08 #79779a21)"
                # Reflect the new state for the case statement below so
                # it walks into the CLOSED branch (skip close + cleanup).
                _issue_state="CLOSED"
            else
                log "issue #${number}: WARNING gh issue close failed (${NO_E2E_LABEL} path) — retry next tick"
                labeled=$((labeled+1)); continue
            fi
        fi

        # 0.2) Close only when invariant holds. Four branches:
        #   (a) already CLOSED → idempotent skip, proceed to cleanup
        #   (b) e2e-done present, OPEN → close with reason=completed
        #   (c) MERGED but no e2e-done → leave OPEN, defer destructive
        #       cleanup until next tick when e2e-process may set e2e-done
        #       (race merge → label, ADR §5).
        #   (d) state unreadable → defer cleanup: we cannot prove the
        #       issue is closed, so we must not delete the last mapping
        #       (ADR §4 req 4).
        _closed_this_tick=0
        case "$_issue_state" in
            CLOSED)
                # CLOSED already (e.g. closed manually or previous tick) — skip
                # close, go straight to cleanup.
                log "issue #${number}: already CLOSED — close skipped"
                ;;
            "")
                # gh issue view failed / state unreadable — conservative
                # deferral: destructive cleanup must not run on unverifiable
                # state, otherwise we lose the issue→PR mapping (ADR §4 req 4).
                log "issue #${number}: WARNING issue state unreadable — destructive cleanup deferred to next tick"
                labeled=$((labeled+1)); continue
                ;;
            OPEN)
                if [ "$_has_e2e_done" = "1" ]; then
                    # User-reopen guard (issue #1391, retro 18.08 t_c4f1d5c8):
                    # если юзер ВРУЧНУЮ переоткрыл issue ПОСЛЕ того, как
                    # e2e-process поставил e2e-done — метка «протухла» и
                    # auto-close подавляет волю юзера (юзер сигналит, что
                    # фикс не сработал, например #1363: «свист так и не
                    # убран!!!»). Закрытие issue в этом случае — тирания
                    # автоматики. Снимаем протухший e2e-done, возвращаем
                    # в needs-e2e-ротацию, оставляем audit-коммент. Если
                    # юзер смёржит новый фикс → следующий e2e-раунд
                    # пере-проставит свежий e2e-done и закрытие пройдёт
                    # штатно. Если фикс ложный PASS — юзер сам закроет
                    # руками либо issue зависнет в open (как и при
                    # Q22 user-merge, ADR-0014 §Q22).
                    _e2e_done_at="$(_timeline_last_labeled_at "$number" "$DONE_LABEL")"
                    _user_reopen_at="$(_timeline_last_reopen_at "$number")"
                    # Helpers возвращают "null" если событий нет (mock и
                    # реальный gh api). Приводим к "empty" для проверок.
                    [ "$_e2e_done_at" = "null" ] && _e2e_done_at=""
                    [ "$_user_reopen_at" = "null" ] && _user_reopen_at=""
                    # ADR-0014 §4 req 4 (conservative on uncertainty).
                    # Логика user-reopen guard (issue #1391):
                    #   • Timeline ПОЛНОСТЬЮ пуст (нет ни одного события —
                    #     значит API сдох или rate-limit): не можем доказать
                    #     отсутствие reopen → fail-closed (не закрываем).
                    #   • Timeline есть, e2e-done присутствует в timeline,
                    #     reopen в timeline ОТСУТСТВУЕТ: штатный путь →
                    #     close (pass-proven).
                    #   • Timeline есть, e2e-done присутствует, reopen
                    #     присутствует и ПОЗЖЕ e2e-done: метка протухла,
                    #     close подавлен.
                    #   • Timeline есть, e2e-done присутствует, reopen
                    #     присутствует но РАНЬШЕ e2e-done (e.g. user
                    #     смёржил новый фикс который прошёл e2e): close
                    #     штатный (e2e-done свежее чем reopen).
                    #   • Timeline есть, e2e-done отсутствует в timeline
                    #     (странный edge-case — labels.csv показывает
                    #     метку, но timeline её не видит): close штатный
                    #     (labels.csv надёжнее timeline для current state).
                    if [ -z "$_e2e_done_at" ] && [ -z "$_user_reopen_at" ]; then
                        # Timeline пустой → не можем доказать отсутствие
                        # reopen → conservative: НЕ закрываем.
                        log "issue #${number}: USER-REOPEN GUARD (issue #1391) — timeline пуст, auto-close подавлен (conservative)"
                        if [ "$DRY_RUN" != "true" ]; then
                            gh issue edit "$number" --repo "$GH_REPO" --add-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
                            _urg_dup="$(gh api "repos/${GH_REPO}/issues/${number}/comments?since=$(date -u -d '24 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)&per_page=100" \
                                --jq '[.[] | select(.body | contains("USER-REOPEN GUARD"))] | length' 2>/dev/null || echo 0)"
                            if [ "${_urg_dup:-0}" -eq 0 ]; then
                                gh issue comment "$number" --repo "$GH_REPO" --body \
                                    "🛡 merge-gate (issue #1391, retro 18.08 t_c4f1d5c8): timeline issue недоступен → auto-close подавлен по conservative-правилу (ADR-0014 §4 req 4). Issue возвращён в \`${NEEDS_E2E_LABEL}\`, следующий тик попробует снова когда timeline будет доступен." >/dev/null 2>&1 || true
                            fi
                        fi
                        labeled=$((labeled+1)); continue
                    fi
                    if [ -n "$_user_reopen_at" ] \
                        && [ -n "$_e2e_done_at" ] \
                        && [ "$_user_reopen_at" \> "$_e2e_done_at" ] 2>/dev/null; then
                        # Юзер переоткрыл ПОСЛЕ e2e-done — не закрываем.
                        log "issue #${number}: USER-REOPEN GUARD (issue #1391) — e2e-done ${_e2e_done_at} протух (user-reopen ${_user_reopen_at} позже), auto-close подавлен"
                        if [ "$DRY_RUN" != "true" ]; then
                            gh issue edit "$number" --repo "$GH_REPO" --remove-label "$DONE_LABEL" >/dev/null 2>&1 || true
                            gh issue edit "$number" --repo "$GH_REPO" --add-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
                            # Audit-коммент с причиной (24h dedup, чтобы
                            # не спамить при каждом тике пока юзер держит
                            # issue открытой).
                            _urg_dup="$(gh api "repos/${GH_REPO}/issues/${number}/comments?since=$(date -u -d '24 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)&per_page=100" \
                                --jq '[.[] | select(.body | contains("USER-REOPEN GUARD"))] | length' 2>/dev/null || echo 0)"
                            if [ "${_urg_dup:-0}" -eq 0 ]; then
                                gh issue comment "$number" --repo "$GH_REPO" --body \
                                    "🛡 merge-gate (issue #1391, retro 18.08 t_c4f1d5c8): user-reopen после \`${DONE_LABEL}\` (reopen at \`${_user_reopen_at}\` > e2e-done at \`${_e2e_done_at}\`) → auto-close подавлен, метка \`${DONE_LABEL}\` снята, возврат в \`${NEEDS_E2E_LABEL}\`. Если смёржен новый фикс — следующий e2e-раунд перепоставит \`${DONE_LABEL}\` и закроет issue штатно." >/dev/null 2>&1 || true
                            fi
                        fi
                        labeled=$((labeled+1)); continue
                    fi
                    # Ретро 18.08 t_873ebef2 (#1391, дополнение к PR #1399):
                    # whitelist label `user-reopened-this` — явный manual
                    # override Шифу. Дополняет user-reopen guard выше: даже
                    # когда timeline говорит «reopen ДО e2e-done» (нормальный
                    # путь), whitelist пропускает close — Шифу знает лучше.
                    if [ -n "${USER_REOPEN_AUDIT_LABEL:-}" ] \
                        && has_label "${_current_labels_norm:-}" "$USER_REOPEN_AUDIT_LABEL"; then
                        log "issue #${number}: whitelist ${USER_REOPEN_AUDIT_LABEL} → skip auto-close (issue #1391 supplement)"
                        labeled=$((labeled+1)); continue
                    fi
                    # issue #1534: self-id whoami BEFORE close — helper
                    # идемпотентный (skip если за последние 2h уже
                    # публиковал такой же marker для этого issue).
                    whoami_close_issue "$number" "post-merge cleanup: PASS-proven via ${DONE_LABEL}, PR MERGED into ${DEVELOP_BRANCH} (ADR-0014)"
                    if gh issue close "$number" --repo "$GH_REPO" --reason completed >/dev/null 2>&1; then
                        _closed_this_tick=1
                        log "issue #${number}: CLOSED (reason=completed, PASS-proven via ${DONE_LABEL})"
                    else
                        # Close API failure — destructive cleanup MUST be
                        # deferred, otherwise we lose the mapping. Warning
                        # only, no `|| true` masking (ADR §4 req 4).
                        log "issue #${number}: WARNING gh issue close failed — destructive cleanup deferred to next tick"
                        labeled=$((labeled+1)); continue
                    fi
                else
                    # MERGED without e2e-done. Two sub-cases:
                    #   (a) Branch still exists on remote → e2e-process can
                    #       still run e2e on the merged branch tip
                    #       (ADR-0014) → defer, wait for e2e-done.
                    #   (b) Branch GONE (user merged by Q22 without e2e,
                    #       branch deleted — GitHub "delete branch on merge"
                    #       or manual cleanup) → e2e is physically impossible
                    #       (e2e-process fetch origin/$branch fails) → issue
                    #       is an eternal orphan in needs-e2e rotation
                    #       (ретро 13.08 t_423453b1, #1160). Remove
                    #       needs-e2e + e2e:rejected, comment to user
                    #       (Q22: close manually or file follow-up), do NOT
                    #       set needs-review (PR already gone), do NOT close
                    #       (user decides), do NOT run destructive cleanup
                    #       (branch already gone).
                    if git ls-remote --heads "https://github.com/$GH_REPO.git" "$branch" 2>/dev/null | grep -q "$branch"; then
                        log "issue #${number}: MERGED but awaiting ${DONE_LABEL} — branch ${branch} exists, destructive cleanup deferred"
                        labeled=$((labeled+1)); continue
                    fi
                    log "issue #${number}: MERGED без ${DONE_LABEL}, ветка ${branch} удалена — e2e невозможен (Q22 user-merge), закрываю issue"
                    # Ретро 13.08 t_0b76514f (#1004/#982/#988/#990/#1160/#1188):
                    # раньше только снимали needs-e2e и комментили «закрой вручную»
                    # — issue висела OPEN вечно, Шифу не видел очередь. Q22:
                    # юзер сам смержил PR (принял фикс), e2e физически невозможен
                    # (ветки нет) → закрываем issue с reason=completed.
                    # Dedup комментария (24h окно, подстрока тела — ретро
                    # bfc18c85: startswith-префикс не совпадал с реальным телом).
                    #
                    # Ретро 18.08 t_873ebef2 (#1391, дополнение к PR #1399):
                    # даже Q22-orphan-close подавляется при ручном reopen —
                    # если Шифу переоткрыл значит «фикс не принят», блокируем
                    # close и возвращаем в user-override. Whitelist label
                    # `user-reopened-this` тоже триггерит skip (явный сигнал).
                    # Audit НЕ публикуем — Q22-flow ниже сам публикует
                    # «🛠 merge-gate (ретро 13.08)», дополнительный
                    # user-reopen audit только шумит.
                    if [ -n "${USER_REOPEN_AUDIT_LABEL:-}" ] \
                        && has_label "${_current_labels_norm:-}" "$USER_REOPEN_AUDIT_LABEL"; then
                        log "issue #${number}: Q22-orphan, whitelist ${USER_REOPEN_AUDIT_LABEL} → skip auto-close"
                        labeled=$((labeled+1)); continue
                    fi
                    if _issue_reopened_recently "$number"; then
                        log "issue #${number}: Q22-orphan, recent user-reopen → skip auto-close (issue #1391 supplement)"
                        labeled=$((labeled+1)); continue
                    fi
                    _orphan_since="$(date -u -d '24 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
                    _orphan_dup="$(gh api "repos/${GH_REPO}/issues/${number}/comments?since=${_orphan_since}&per_page=100" \
                        --jq '[.[] | select(.body | contains("Фикс влит по Q22"))] | length' 2>/dev/null || echo 0)"
                    gh issue edit "$number" --repo "$GH_REPO" --remove-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
                    gh issue edit "$number" --repo "$GH_REPO" --remove-label "$REJECTED_LABEL" >/dev/null 2>&1 || true
                    if [ "${_orphan_dup:-0}" -eq 0 ]; then
                        gh issue comment "$number" --repo "$GH_REPO" --body \
                            "🛠 merge-gate (ретро 13.08 t_0b76514f): PR #${pr_number} смержен вручную (Q22) без e2e-прогона, ветка \`${branch}\` удалена → e2e невозможен. Фикс влит по Q22 — issue закрыта." >/dev/null 2>&1 || true
                    fi
                    # issue #1534: self-id whoami BEFORE close (Q22 orphan path).
                    whoami_close_issue "$number" "Q22 user-merge orphan: PR merged without e2e (retro 13.08 t_0b76514f), ветка ${branch} удалена" "branch=${branch}"
                    if gh issue close "$number" --repo "$GH_REPO" --reason completed >/dev/null 2>&1; then
                        _closed_this_tick=1
                        log "issue #${number}: CLOSED (Q22 user-merge, e2e невозможен)"
                        # Ретро 14.08 t_36c9ac4e: Q22-путь раньше делал continue ДО
                        # archive-блока (стр. 774-783) → done-карточка оставалась
                        # done НАВСЕГДА при смерженном PR (t_41fec39e: issue #1217
                        # закрыта Q22-путём, PR #1220 merged, карточка done 21:25).
                        # Ретро 14.08 t_0bd15be9: helper также закрывает blocked
                        # (unblock → complete → archive). Здесь destructive
                        # cleanup разрешён: close успешен (ADR-0014 §4 req 4).
                        if [ -n "${task_id:-}" ]; then
                            archive_merged_card "$task_id" "$number" "${pr_number:-}" "${branch:-}"
                        fi
                    else
                        log "issue #${number}: WARNING gh issue close failed (Q22 orphan) — retry next tick"
                    fi
                    labeled=$((labeled+1)); continue
                fi
                ;;
            *)
                log "issue #${number}: unexpected issue state=${_issue_state} — skip cleanup"
                skipped=$((skipped+1)); continue
                ;;
        esac

        # 1) Find the issue's kanban card (may be done already).
        card_id=""
        if [ -z "${task_id:-}" ]; then
            # Ретро 14.08 t_36c9ac4e: `kanban list --json` НЕ содержит поле
            # "issue" (hermes v0.20.0 _task_to_dict — только id/title/body/...).
            # Раньше t.get("issue") всегда был пуст → карточка по issue НЕ
            # находилась НИКОГДА. Ищем маркер "issue: #N" в теле карточки
            # (формат triage: "issue: #1217").
            card_id="$( "$HERMES_BIN" kanban --board "$KANBAN_BOARD" list --json 2>/dev/null \
                | python3 -c '
import sys, json, re
try:
    d = json.load(sys.stdin)
    tasks = d if isinstance(d, list) else d.get("tasks", [])
    num = sys.argv[1]
    pat = re.compile(r"issue\s*[:=]\s*#?" + re.escape(num) + r"\b", re.IGNORECASE)
    for t in tasks:
        if pat.search(t.get("body") or ""):
            print(t.get("id", "")); break
except Exception:
    pass
' "$number" 2>/dev/null || true)"
        else
            card_id="$task_id"
        fi
        # 2) Free stale worktrees on this branch (kanban workspace_path aware).
        if [ -n "$card_id" ]; then
            free_stale_worktrees_for "$card_id" || true
        fi
        # 3) Delete the remote branch (merged — safe). We have already
        # closed the issue above, so the issue→PR mapping is preserved
        # in GitHub issue history regardless of whether the branch ref
        # still exists (ADR §4 req 4).
        if git ls-remote --heads "https://github.com/$GH_REPO.git" "$branch" 2>/dev/null | grep -q "$branch"; then
            gh api -X DELETE "repos/$GH_REPO/git/refs/heads/$branch" >/dev/null 2>&1 \
                && log "issue #${number}: remote branch ${branch} deleted" \
                || log "issue #${number}: WARNING failed to delete branch ${branch}"
        else
            log "issue #${number}: branch ${branch} already gone"
        fi
        # 4) Archive the card (done/blocked → archived) so the board stays clean.
        # Ретро 14.08 t_0bd15be9: blocked-карточка с MERGED PR висела вечно —
        # архив-маппинг скипал status!=done, recovery-воркер не завершал
        # родителя («родитель закроется процессом»), а процесса для blocked
        # нет. Фикс влит (PR MERGED) ⇒ критерий карточки выполнен независимо
        # от причины blocked: unblock → complete → archive (см. helper).
        if [ -n "$card_id" ]; then
            archive_merged_card "$card_id" "$number" "${pr_number:-}" "${branch:-}"
        fi
        # 5) Dedup cleanup-коммента (ретро 10.08 t_9caf5d52): раньше коммент
        #    «✅ PR #N смержен» постился КАЖДЫЙ тик (5 мин) → 6 одинаковых на
        #    #1089 (08:35–08:49). Постим только если за последние часы такого
        #    коммента ещё нет. ADR-0014: текст говорит правду — упомянуть
        #    закрытие issue явно.
        _dedup_since="$(date -u -d '6 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
        _dup_count="$(gh api "repos/${GH_REPO}/issues/${number}/comments?since=${_dedup_since}&per_page=100" \
            --jq '[.[] | select(.body | startswith("✅ PR #'"${pr_number}"' смержен"))] | length' 2>/dev/null || echo 0)"
        if [ "${_dup_count:-0}" -eq 0 ]; then
            _close_note=""
            if [ "$_closed_this_tick" = "1" ]; then
                _close_note="Issue закрыта (reason=completed, PASS-proven). "
            fi
            gh issue comment "$number" --repo "$GH_REPO" --body \
                "✅ PR #${pr_number} смержен в ${pr_base}. ${_close_note}Cleanup: ветка удалена, worktree освобождены, карточка заархивирована." >/dev/null 2>&1 || true
        else
            log "issue #${number}: merged-cleanup comment already exists (×${_dup_count}) — dedup skip"
        fi
        # 6) Снять stale-метки со смерженного фикса (ретро 10.08 t_9caf5d52):
        #    e2e:rejected/needs-e2e на merged-PR не актуальны. e2e-done НЕ
        #    добавляем сами: PASS-label принадлежит только e2e-process
        #    (ADR-0014 §4 req 2). Если e2e-done уже стоит — он остаётся;
        #    если не стоит — мы НЕ должны его создавать.
        gh issue edit "$number" --repo "$GH_REPO" --remove-label "$REJECTED_LABEL" >/dev/null 2>&1 || true
        gh issue edit "$number" --repo "$GH_REPO" --remove-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
        labeled=$((labeled+1)); continue
    fi

    # --- return path from e2e:rejected (ретро 10.08 t_9caf5d52) ------------
    # Раньше e2e:rejected был ТУПИКОМ: e2e-process скипает такие issue (line 15),
    # и никто не возвращал их в ротацию (#1089/#1077 — stale-метка навсегда).
    # Теперь: merge-gate по push в PR (новые коммиты после момента навешивания
    # e2e:rejected) снимает e2e:rejected → ставит needs-e2e → e2e-process снова
    # возьмёт issue в ротацию. Момент метки берём из timeline issue (последний
    # labeled-евент для REJECTED_LABEL).
    if has_label "$labels_norm" "$REJECTED_LABEL" && [ "$pr_state" = "OPEN" ]; then
        _rejected_at="$(gh api "repos/${GH_REPO}/issues/${number}/timeline?per_page=100" \
            --jq '[.[] | select(.event=="labeled" and .label.name=="'"$REJECTED_LABEL"'")][-1].created_at' 2>/dev/null || echo '')"
        _return_reason=""
        if [ -n "$_rejected_at" ]; then
            # Сигнал 1: push в PR — новые коммиты после момента метки.
            _new_commits="$(gh api "repos/${GH_REPO}/pulls/${pr_number}/commits?per_page=100" \
                --jq '[.[] | select(.commit.committer.date >= "'"$_rejected_at"'")] | length' 2>/dev/null || echo 0)"
            if [ "${_new_commits:-0}" -gt 0 ] 2>/dev/null; then
                _return_reason="в PR #${pr_number} появились новые коммиты после ${_rejected_at}"
            fi
        fi
        # Сигнал 2 (ретро 10.08 t_9caf5d52): коммент воркера ПОСЛЕ метки.
        # Отличаем от комментов самого процесса (начинаются с agent-flow: или
        # ## 📊 e2e-доклад) — воркер пишет worker-evidence:/свободным текстом.
        if [ -z "$_return_reason" ]; then
            _worker_cmt="$(gh api "repos/${GH_REPO}/issues/${number}/comments?since=${_rejected_at}&per_page=100" \
                --jq '[.[] | select((.body | startswith("agent-flow:") | not) and (.body | startswith("## 📊 e2e-доклад") | not) and (.body | startswith("⛔ CI красный") | not) and (.body | startswith("✅ PR #") | not) and (.body | startswith("🔀 merge conflict") | not) and (.body | startswith("🔄") | not))] | length' 2>/dev/null || echo 0)"
            if [ "${_worker_cmt:-0}" -gt 0 ] 2>/dev/null; then
                _return_reason="воркер прокомментировал issue #${number} после ${_rejected_at}"
            fi
        fi
        # Сигнал 3 (ретро 22.08 t_9e61d788, issue #1506): на этой issue есть
        # ДРУГОЙ OPEN PR с меткой e2e-done (= инфра-фикс прошёл e2e-раунд на
        # feature-ветке). Канонический PR (на котором висит e2e:rejected) —
        # data-only артефакт (голосовые .ogg / сценарии .json), который НЕ
        # блокирует мёрж инфра-фикса и НЕ должен блокировать закрытие issue.
        # Раньше e2e:rejected + e2e-done на одной issue = лимб: e2e-process
        # скипал (e2e:rejected filter, line 586 e2e-process.sh), merge-gate
        # тоже скипал (return path требовал новый коммит/коммент, data-only
        # PR их не даёт), issue висела с needs-e2e+e2e:rejected до merge PR
        # Шифом по Q22. Теперь: снимаем e2e:rejected с issue и с data-only PR,
        # ставим needs-review на e2e-done PR (= очередь на ревью Шифу).
        # Идемпотентно: повторный тик найдёт issue уже без e2e:rejected и
        # пойдёт в основной e2e-done+OPEN PR → needs-review путь (1005).
        if [ -z "$_return_reason" ]; then
            _sibling_done_json="$(gh pr list --repo "$GH_REPO" --state open \
                --search "${number} in:title" \
                --json number,labels 2>/dev/null || true)"
            _sibling_done_pr="$(printf '%s' "$_sibling_done_json" | python3 -c '
import json, sys, os
try:
    data = json.load(sys.stdin)
except Exception:
    data = []
sib = [d for d in data
       if str(d.get("number", "")) != str("'"$pr_number"'")
       and any(l.get("name", "") == "e2e-done" for l in (d.get("labels") or []))]
if sib:
    print(str(sib[0].get("number", ""))); sys.exit(0)
sys.exit(0)' 2>/dev/null || echo "")"
            if [ -n "$_sibling_done_pr" ] && [ "$_sibling_done_pr" != "null" ] \
                && [ "$_sibling_done_pr" != "$pr_number" ]; then
                _return_reason="PR #${_sibling_done_pr} на этой issue имеет e2e-done (инфра-фикс прошёл раунд); data-only PR #${pr_number} больше не блокирует"
            fi
        fi
        # --- type:testing gate в signal 3 (ретро 22.08 t_944df2c5, issue #1506) --
        # Для type:testing задач sibling e2e-done PR НЕ валидирует acceptance
        # исходной задачи: sibling может быть инфра-фиксом (upload-artifact '?',
        # CR/LF и т.п.), который сам по себе SUCCESS, но acceptance-пункты
        # (сценарий voice_core_suite) не выполнил. Возврат issue в needs-e2e
        # здесь подменил бы acceptance инфра-прогоном → «красивый PASS вместо
        # честного FAIL» (ADR-0018). Вместо этого ставим needs-review на
        # data-only PR + warning, issue остаётся e2e:rejected до ручной
        # верификации Шифу (или явного re-run с scenario_file+acceptance_file).
        if [ -n "${_sibling_done_pr:-}" ] && [ "${_sibling_done_pr:-}" != "null" ] \
            && [ "${_sibling_done_pr:-}" != "$pr_number" ] \
            && has_label "$labels_norm" "type:testing"; then
            log "issue #${number}: type:testing + sibling e2e-done PR #${_sibling_done_pr} — signal 3 НЕ валидирует acceptance; needs-review на data-only PR #${pr_number} + manual check"
            if [ "$DRY_RUN" != "true" ]; then
                gh pr edit "$pr_number" --repo "$GH_REPO" --add-label "$NEEDS_REVIEW_LABEL" >/dev/null 2>&1 || true
                gh pr edit "$pr_number" --repo "$GH_REPO" --remove-label "$REJECTED_LABEL" >/dev/null 2>&1 || true
                # 24h dedup — issue остаётся e2e:rejected, тик повторяется ~5м.
                _tt_since="$(date -u -d '24 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
                _tt_dup="$(gh api "repos/${GH_REPO}/issues/${number}/comments?since=${_tt_since}&per_page=100" \
                    --jq '[.[] | select(.body | contains("type:testing") and contains("НЕ валидирует acceptance"))] | length' 2>/dev/null || echo 0)"
                if [ "${_tt_dup:-0}" -eq 0 ]; then
                    gh issue comment "$number" --repo "$GH_REPO" --body \
                        "agent-flow: ⚠️ issue #${number} имеет \`type:testing\` — сигнал 3 (sibling PR #${_sibling_done_pr} с \`${DONE_LABEL}\`) НЕ валидирует acceptance исходной задачи. Data-only PR #${pr_number} поставлен на \`${NEEDS_REVIEW_LABEL}\`; требуется ручная проверка Шифу, что acceptance-пункты #${number} выполнены, либо явный re-run сценария с \`scenario_file\` + \`acceptance_file\` (ретро 22.08 t_944df2c5)." >/dev/null 2>&1 || true
                fi
            fi
            labeled=$((labeled+1)); continue
        fi
        if [ -n "$_return_reason" ]; then
            log "issue #${number}: ${REJECTED_LABEL} → ${_return_reason} — returning to rotation"
            if [ "$DRY_RUN" != "true" ]; then
                gh issue edit "$number" --repo "$GH_REPO" --remove-label "$REJECTED_LABEL" >/dev/null 2>&1 || true
                gh issue edit "$number" --repo "$GH_REPO" --add-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
                gh pr edit "$pr_number" --repo "$GH_REPO" --remove-label "$REJECTED_LABEL" >/dev/null 2>&1 || true
                # Если сработал сигнал 3 — заодно ставим needs-review на
                # e2e-done PR (= инфра-фикс). Иначе сигнал 1/2 — пусть
                # основной цикл ниже обработает, как раньше.
                if [ -n "${_sibling_done_pr:-}" ] && [ "${_sibling_done_pr:-}" != "null" ] \
                    && [ "${_sibling_done_pr:-}" != "$pr_number" ]; then
                    gh pr edit "$_sibling_done_pr" --repo "$GH_REPO" --add-label "$NEEDS_REVIEW_LABEL" >/dev/null 2>&1 || true
                    gh pr edit "$_sibling_done_pr" --repo "$GH_REPO" --remove-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
                fi
                gh issue comment "$number" --repo "$GH_REPO" --body \
                    "agent-flow: 🔄 ${REJECTED_LABEL} снят — ${_return_reason}. Поставлен ${NEEDS_E2E_LABEL}; e2e-process снова возьмёт issue в ротацию." >/dev/null 2>&1 || true
            fi
            labeled=$((labeled+1)); continue
        fi
        log "issue #${number} already has ${REJECTED_LABEL} and no new commits/worker comment — skip"
        skipped=$((skipped+1)); continue
    fi

    # Idempotency: skip if already in the post-merge-gate state.
    # (MERGED-cleanup above runs FIRST so merged PRs get cleaned even
    # when the issue carries needs-e2e/e2e-done/e2e:rejected.)
    if has_label "$labels_norm" "$NEEDS_E2E_LABEL" \
        || has_label "$labels_norm" "$DONE_LABEL"; then
        log "issue #${number} already past merge-gate (${labels_norm}) — skip"
        skipped=$((skipped+1)); continue
    fi

    # Guards: PR must be OPEN, targeting `develop`, mergeable, merge_state clean,
    # all checks SUCCESS.
    if [ "$pr_state" != "OPEN" ]; then
        log "issue #${number}: PR #${pr_number} state=${pr_state} — skip"
        skipped=$((skipped+1)); continue
    fi
    if [ "$pr_base" != "$DEVELOP_BRANCH" ]; then
        log "issue #${number}: PR #${pr_number} base=${pr_base} (want ${DEVELOP_BRANCH}) — skip"
        skipped=$((skipped+1)); continue
    fi
    if [ "$pr_rollup_count" -eq 0 ]; then
        log "issue #${number}: PR #${pr_number} has no check rollup yet — skip"
        skipped=$((skipped+1)); continue
    fi
    if [ "$pr_rollup_pass" -ne 1 ]; then
        # Red CI (Q21) — НЕ создаём CI-fix карточку. Комментим причину в issue
        # и разблокируем карточку воркера — он чинит в ТОЙ ЖЕ ветке.
        log "issue #${number}: PR #${pr_number} checks not all SUCCESS — unblocking card ${task_id} (Q21)"
        failed_checks="$(printf '%s' "$pr_json" | python3 -c '
import json, sys
try:
    pr = json.load(sys.stdin)
    names = []
    for c in (pr.get("statusCheckRollup") or []):
        ctx = c.get("context") or c.get("name") or "?"
        st = c.get("conclusion") or c.get("state") or "?"
        if st not in ("SUCCESS", "NEUTRAL", "SKIPPED"):
            names.append(f"{ctx}: {st}")
    print("; ".join(names[:6]) or "unknown checks")
except Exception:
    print("unknown checks")
' 2>/dev/null || echo "unknown checks")"
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would comment red-CI on issue #${number} and unblock card ${task_id}"
            skipped=$((skipped+1)); continue
        fi
        gh issue comment "$number" --repo "$GH_REPO" --body \
            "⛔ CI красный (PR #${pr_number}): ${failed_checks}
Карточка разблокирована — исправь в ветке ${branch} и запушь. Merge-gate снова поставит needs-e2e когда всё позеленеет." >/dev/null 2>&1 || true
        # Процесс-фикс (09.08, ретро): освободить ветку от worktree старых
        # done/archived карточек ПЕРЕД unblock — иначе респавн падает
        # «git worktree add failed» и карточка навсегда виснет в blocked.
        free_stale_worktrees_for "$task_id" || true
        if "$HERMES_BIN" kanban --board "$KANBAN_BOARD" unblock "$task_id" >/dev/null 2>&1; then
            log "issue #${number}: card ${task_id} unblocked (CI red, worktrees freed)"
        else
            log "issue #${number}: WARNING unblock failed for ${task_id} — card may not be blocked yet"
        fi
        skipped=$((skipped+1)); continue
    fi
    if [ "$pr_mergeable" != "MERGEABLE" ]; then
        # Процесс-фикс (10.08, ретро t_75e787fd — Шифу прямо: «нахуя отдельную
        # карточку, та же карточка должна знать»). Если PR CONFLICTING →
        # **дописываем reminder в существующую рабочую карточку воркера**
        # (по issue_number находим его task_id, comment с rebase-инструкцией).
        # НЕ создаём новую карточку — это лишняя сущность.
        #
        # Процесс-фикс (18.08, ретро #1356 / t_75e787fd-2 — болтающийся PR):
        # если карточка уже done/blocked/archived, reminder уходит в мёртвую
        # карточку, никто не видит, конфликт висит бессрочно. Теперь:
        #   - если карточка живая (running/ready/todo) → comment в неё (старое
        #     поведение, Шифу прямо: «та же карточка должна знать»);
        #   - если карточка мёртвая (done/blocked/archived) либо отсутствует →
        #     reassign retry на assignee=владелец PR (assignee из меток issue)
        #     и создаём fresh recovery-карточку (как scan-all-prs на стр. 1836).
        if [ "$pr_mergeable" = "CONFLICTING" ] && [ "$pr_state" = "OPEN" ]; then
            # Подтягиваем headRefName — основной CONFLICTING-блок его не имел
            # (ранее reminder ссылался на пустую переменную; регрессия t_1146).
            if [ -z "${pr_head_ref:-}" ]; then
                pr_head_ref="$(gh pr view "$pr_number" --repo "$GH_REPO" --json headRefName --jq '.headRefName' 2>/dev/null || echo "")"
                [ -z "${pr_head_ref:-}" ] && log "issue #${number}: WARNING cannot fetch headRefName for PR #${pr_number}" && continue
            fi
            # Определяем assignee по меткам issue (для recovery-карточки).
            # Ретро 02.09 t_2bd2e7ea: default НЕ валиден — упал бы в ADR-0041
            # silent-drop. Если метки нет, fallback на devops (он же воркер,
            # который и должен разрешать конфликт через force-with-lease push).
            _assignee="devops"
            for lbl in $(gh issue view "$number" --repo "$GH_REPO" --json labels --jq '[.labels[].name] | .[]' 2>/dev/null); do
                case "$lbl" in
                    agent:backend)    _assignee="backend"; break ;;
                    agent:developer)  _assignee="developer"; break ;;
                    agent:tester)     _assignee="tester"; break ;;
                    agent:devops)     _assignee="devops"; break ;;
                    agent:architect)  _assignee="architect"; break ;;
                esac
            done
            if [ -n "${task_id:-}" ]; then
                _card_status="$(kanban_card_status "$task_id")"
                case "$_card_status" in
                    running|ready|todo)
                        # Карточка живая (работает/готова/в очереди) — Шифу прямо:
                        # «та же карточка должна знать». Старое поведение.
                        # Ретро 12.08 t_8af6bf29: rate-limit — коммент не чаще
                        # 1 раза в 2ч, иначе при вечном CONFLICTING каждый тик
                        # (~10 мин) шумит в карточке.
                        _last_cf="$(kanban_last_reminder_ts "$task_id" "merge conflict detected")"
                        _now_cf="$(date +%s)"
                        if [ -n "$_last_cf" ] && [ $(( _now_cf - _last_cf )) -lt 7200 ]; then
                            log "issue #${number}: PR #${pr_number} mergeable=CONFLICTING — rebase reminder rate-limited (last=${_last_cf})"
                            skipped=$((skipped+1)); continue
                        fi
                        log "issue #${number}: PR #${pr_number} mergeable=CONFLICTING — appending rebase reminder to live card ${task_id} (status=${_card_status})"
                        _rebase_reminder="## 🔀 merge conflict detected (merge-gate tick, $(date -u +%H:%M:%SZ))

PR #${pr_number} (\`${pr_head_ref}\`) → develop = **CONFLICTING**. Develop убежал вперёд, твоя ветка не мерджится напрямую.

**ОБЯЗАН** (по процессу Шифу 10.08):
1. **В той же ветке** \`${pr_head_ref}\` — НЕ создавай новую ветку и НЕ новый PR (Шифу прямо).
2. **rebase** на origin/develop: \`git fetch origin develop && git rebase origin/develop\`.
3. Разреши конфликты → \`git add -A && git rebase --continue\`.
4. \`git push --force-with-lease origin ${pr_head_ref}\` (force-with-lease безопасен).
5. Метки снимать НЕ надо (PR уже needs-e2e). Следующий тик merge-gate снова проверит — если MERGEABLE → поставит needs-review / needs-e2e по процессу.

**Когда карточка закрывается:** когда увидишь что PR стал MERGEABLE (rebase прошёл, push дошёл).

**Команды шпаргалка:**
\`\`\`bash
git fetch origin develop
git checkout ${pr_head_ref}
git rebase origin/develop
# ... resolve conflicts ...
git add -A
git rebase --continue
git push --force-with-lease origin ${pr_head_ref}
\`\`\`

(Этот reminder автоматически дописан merge-gate. Никакой новой карточки не создано — Шифу прямо: «та же карточка должна знать».)"
                        hermes kanban --board "$KANBAN_BOARD" comment "$task_id" "$_rebase_reminder" >/dev/null 2>&1 \
                            || log "issue #${number}: WARNING appending rebase reminder to ${task_id} failed"
                        ;;
                    done|archived|blocked)
                        # Карточка мёртвая (воркер закрыл, а PR остался CONFLICTING —
                        # типичный кейс ретро #1356). Reminder ушёл бы в никуда.
                        # Шифу прямо (10.08): тот же PR, та же ветка, никаких новых
                        # веток/PR. Но работать КОМУ-ТО надо → создаём СВЕЖУЮ
                        # recovery-карточку (assignee=владелец PR по меткам issue),
                        # чтобы воркер перевзял и сделал rebase. Идемпотентно по
                        # PR (кл. t_75e787fd-2) — при существующей recovery-карточке
                        # scan-all-prs уже подберёт её (см. _branch_matches).
                        log "issue #${number}: PR #${pr_number} mergeable=CONFLICTING — карточка ${task_id} мёртвая (status=${_card_status}), создаю recovery-карточку (ретро #1356)"
                        _rec_body="## 🔀 merge conflict (merge-gate recovery, ретро #1356, $(date -u +%H:%M:%SZ))

PR #${pr_number} (\`${pr_head_ref}\`) → develop = **CONFLICTING**. Связанная рабочая карточка \`${task_id}\` уже закрыта/архивирована, поэтому напоминание в неё НЕ пишем (потеряется).

**ОБЯЗАН** (по процессу Шифу 10.08):
1. **В той же ветке** \`${pr_head_ref}\` — НЕ создавай новую ветку и НЕ новый PR.
2. **rebase** на origin/develop: \`git fetch origin develop && git rebase origin/develop\`.
3. Разреши конфликты → \`git add -A && git rebase --continue\`.
4. \`git push --force-with-lease origin ${pr_head_ref}\`.

**Когда закрывается:** когда PR станет MERGEABLE (rebase прошёл).

**Команды шпаргалка:**
\`\`\`bash
git fetch origin develop
git checkout ${pr_head_ref}
git rebase origin/develop
git add -A && git rebase --continue
git push --force-with-lease origin ${pr_head_ref}
\`\`\`

(merge-gate создал эту карточку, потому что живая рабочая карточка этого issue уже закрыта — ретро #1356.)"
                        # Идемпотентность: ищем уже-существующую recovery-карточку
                        # по PR в title (как e2e-process t_bff6eccf, scan-all-prs
                        # стр. 1743-1775). active > blocked > done:
                        #   - running/ready/todo → skip (гонка force-push, урок 13.08)
                        #   - blocked → unblock
                        #   - done/archived → НЕ трогаем, ищем более свежий
                        #     активный, иначе создаём fresh (НО active нет)
                        _branch_matches="$(hermes kanban --board "$KANBAN_BOARD" list --json 2>/dev/null | python3 -c "
import json,sys
try:
    data = json.loads(sys.stdin.read())
except Exception:
    sys.exit(0)
for t in data:
    title = t.get('title','')
    if 'rebase PR #${pr_number}' in title:
        print(t['id'], t.get('status',''))
" 2>/dev/null || true)"
                        _active_match="$(printf '%s\n' "$_branch_matches" | awk '$2 ~ /^(running|ready|todo)$/ {print $1" "$2; exit}')"
                        _blocked_id="$(printf '%s\n' "$_branch_matches" | awk '$2 == "blocked" {print $1; exit}')"
                        if [ -n "$_active_match" ]; then
                            log "issue #${number}: main-cycle — recovery card already active (${_active_match}) for PR #${pr_number} — skip (scan-all-prs подхватит)"
                        elif [ -n "$_blocked_id" ]; then
                            hermes kanban --board "$KANBAN_BOARD" unblock "$_blocked_id" --reason "🔀 main-cycle recovery (ретро #1356)" >/dev/null 2>&1 || true
                            log "issue #${number}: main-cycle — recovery card ${_blocked_id} unblocked for PR #${pr_number}"
                        else
                            # Ретро 15.08 t_16325ddd: PR мог закрыться после получения
                            # pr_state в основном цикле — пере-проверяем перед create
                            # (как UNSTABLE-ветка ниже, см. ~стр. 1543).
                            if [ "$(pr_state_now "$pr_number")" = "CLOSED" ]; then
                                log "issue #${number}: PR #${pr_number} CLOSED — CONFLICTING recovery card НЕ создаю (ретро t_16325ddd)"
                            else
                                # fresh card с уникальным ключом (PR+timestamp) — НЕ
                                # плодим дубли подряд, но каждый новый «висящий»
                                # конфликт гарантированно получит свежую карточку
                                # (кл. t_42741511 done-match остался в scan-all-prs).
                                _rec_key="merge-conflict-main-cycle-pr-${pr_number}-$(date +%s)"
                                hermes kanban --board "$KANBAN_BOARD" create \
                                    --assignee "$_assignee" --priority 80 --max-runtime 1800 \
                                    --body "$_rec_body" \
                                    "🔀 rebase PR #${pr_number} (\`${pr_head_ref}\`) на develop — конфликт (main-cycle, ретро #1356)" \
                                    >/dev/null 2>&1 \
                                    && log "issue #${number}: main-cycle — recovery card created for PR #${pr_number} (assignee=${_assignee})" \
                                    || log "issue #${number}: WARNING main-cycle recovery card create failed for PR #${pr_number}"
                            fi
                        fi
                        ;;
                    *)
                        # неизвестный/не-прочитанный статус — по умолчанию считаем
                        # мёртвой (по аналогии с PR UNKNOWN → CONFLICTING/DIRTY).
                        log "issue #${number}: PR #${pr_number} mergeable=CONFLICTING — карточка ${task_id} статус «${_card_status}» (read-failed), отношусь как мёртвой"
                        # то же самое, что done ветка выше — fresh recovery card
                        ;;
                esac
            else
                # task_id пуст (issue без маркера «kanban: t_xxx»). Scan-all-prs
                # отдельно подберёт — здесь просто логируем.
                log "issue #${number}: PR #${pr_number} mergeable=CONFLICTING — task_id пуст, scan-all-prs подхватит"
            fi
        else
            log "issue #${number}: PR #${pr_number} mergeable=${pr_mergeable} — skip"
        fi
        skipped=$((skipped+1)); continue
    fi
    if [ "$pr_merge_state" != "CLEAN" ]; then
        # Процесс-фикс (10.08, ретро t_51b5ad24 — Шифу прямо: «оно должно
        # взять себе девелоп сейчас и позеленеть»). Если UNSTABLE (CI fail,
        # но PR mergeable=YES) → НЕ просто skip, а **дописываем reminder в
        # существующую карточку воркера**: «CI красный по известной причине,
        # rebase на origin/develop, в develop уже есть фикс». Тот же PR, та
        # же ветка, никаких новых. Если карточки нет → создаём с assignee
        # по метке issue.
        if [ "$pr_merge_state" = "UNSTABLE" ] && [ "$pr_state" = "OPEN" ]; then
            # Ретро 31.08 t_e00f448d: ДО reminder-логики — classify: реальная ли
            # это регрессия в коде PR, или просто CI красный на integration/build,
            # которые могут починиться develop-фиксами при rebase.
            #   unit_lint       → ДИАГНОСТИЧЕСКАЯ карточка (assignee по label issue),
            #                     raw-evidence (failed job names + html_url), НЕ rebase
            #   integration_e2e → старая rebase-логика (develop-фиксы могут починить)
            #   unknown         → fail-open: старая rebase-логика (безопасный default)
            _un_class="$(pr_classify_failure "${pr_head_oid:-}")"
            log "issue #${number}: PR #${pr_number} UNSTABLE classification=${_un_class} (head_oid=${pr_head_oid:-none})"
            if [ "$_un_class" = "unit_lint" ]; then
                # Диагностический путь: реальная CI-регрессия в коде PR, rebase
                # бессилен. Создаём карточку с raw-evidence для профильного воркера
                # (assignee по метке issue). Rate-limit по marker "CI UNSTABLE: DIAGNOSTIC
                # #<pr_number>" чтобы не плодить дубликаты.
                _un_diag_key="ci-unstable-diagnostic-pr-${pr_number}"
                if [ -n "${task_id:-}" ]; then
                    _un_diag_last="$(kanban_last_reminder_ts "$task_id" "$_un_diag_key")"
                    _un_diag_now="$(date +%s)"
                    if [ -n "$_un_diag_last" ] && [ $(( _un_diag_now - _un_diag_last )) -lt 7200 ]; then
                        log "issue #${number}: PR #${pr_number} UNSTABLE unit_lint — diagnostic card rate-limited (last=${_un_diag_last})"
                        skipped=$((skipped+1)); continue
                    fi
                fi
                # Дополнительная защита: dedup по уже существующим карточкам с
                # тем же PR в title (аналогично recovery-логике выше).
                _un_diag_existing="$(hermes kanban --board "$KANBAN_BOARD" list --json 2>/dev/null | python3 -c "
import json,sys
try:
    data = json.loads(sys.stdin.read())
except Exception:
    sys.exit(0)
needle = 'CI UNSTABLE DIAGNOSTIC #${pr_number}'
for t in data:
    title = t.get('title','')
    if needle in title:
        print(t['id'], t.get('status',''))
" 2>/dev/null || true)"
                _un_diag_active="$(printf '%s\n' "$_un_diag_existing" | awk '$2 ~ /^(running|ready|todo)$/ {print $1" "$2; exit}')"
                if [ -n "$_un_diag_active" ]; then
                    log "issue #${number}: UNSTABLE unit_lint — diagnostic card already active (${_un_diag_active}) for PR #${pr_number} — skip"
                    skipped=$((skipped+1)); continue
                fi
                # assignee по метке issue (та же логика, что и в rebase-блоке ниже).
                # Ретро 02.09 t_2bd2e7ea: default → devops fallback.
                _assignee="devops"
                for lbl in $(gh issue view "$number" --repo "$GH_REPO" --json labels --jq '[.labels[].name] | .[]' 2>/dev/null); do
                    case "$lbl" in
                        agent:backend)    _assignee="backend"; break ;;
                        agent:developer)  _assignee="developer"; break ;;
                        agent:tester)     _assignee="tester"; break ;;
                        agent:devops)     _assignee="devops"; break ;;
                        agent:architect)  _assignee="architect"; break ;;
                    esac
                done
                # Skill — профильный, как в recovery-блоке.
                _skill="architecture-doc-review"
                case "$_assignee" in
                    backend)   _skill="test-driven-development" ;;
                    developer) _skill="test-driven-development" ;;
                    tester)    _skill="test-driven-development" ;;
                    devops)    _skill="agent-flow-e2e-pipeline" ;;
                esac
                # Пере-проверяем state PR (мог закрыться пока мы тут).
                if [ "$(pr_state_now "$pr_number")" = "CLOSED" ]; then
                    log "issue #${number}: PR #${pr_number} CLOSED — UNSTABLE diagnostic card НЕ создаю (ретро t_16325ddd)"
                    skipped=$((skipped+1)); continue
                fi
                # headRefName для body.
                pr_head_ref="$(gh pr view "$pr_number" --repo "$GH_REPO" --json headRefName --jq '.headRefName' 2>/dev/null || echo "?")"
                # Failed jobs list с html_url (raw-evidence).
                _un_failed_jobs="$(pr_failed_jobs_json "${pr_head_oid:-}")"
                # Формируем markdown-список failed jobs (не json-блоб, чтобы воркер
                # сразу видел названия и мог кликнуть).
                _un_failed_md="$(printf '%s' "$_un_failed_jobs" | python3 -c "
import json,sys
try:
    arr = json.loads(sys.stdin.read())
    if not arr:
        print('- (не смог достать список failed jobs; см. вкладку Checks в PR)')
    else:
        for j in arr:
            n = j.get('name','?')
            u = j.get('html_url','')
            print(f'- **{n}** — <{u}|job>')
except Exception:
    print('- (failed to parse jobs JSON)')
" 2>/dev/null || echo '- (failed to extract failed jobs)')"
                _un_diag_body="## 🐛 CI UNSTABLE: real regression в PR (merge-gate, ретро t_e00f448d, $(date -u +%H:%M:%SZ))

**PR #${pr_number}** (\\\`${pr_head_ref}\\\`) = **mergeable=MERGEABLE + mergeStateStatus=UNSTABLE** + classification=**unit_lint**.
Это **НЕ stale-from-develop**: develop-фиксов нет, регрессия — в коде этого PR. **rebase бессилен**, нужна починка кода/тестов.

### Failed jobs (check-run)
${_un_failed_md}

### Issue/PR context
- issue #${number}
- PR head: \\\`${pr_head_ref}\\\` @ \\\`${pr_head_oid}\\\`

### Что делать (НЕ rebase)
1. Открыть failed jobs, прочитать assertion diff и имена упавших тестов.
2. Починить код/тесты в \\\`${pr_head_ref}\\\` (НЕ делать rebase на develop — это не тот случай).
3. Push --force-with-lease (или обычный push если коммиты были аддитивные).
4. Когда CI станет зелёным → merge-gate следующего тика увидит MERGEABLE+CLEAN → поставит \\\`needs-e2e\\\`.

Карточка закрывается когда PR станет MERGEABLE+CLEAN (CI зелёный).

(merge-gate создал эту карточку, потому что classification check-runs = unit_lint — ретро t_e00f448d.)"
                hermes kanban --board "$KANBAN_BOARD" create \
                    --assignee "$_assignee" --skill "$_skill" --priority 90 --max-runtime 1800 \
                    --body "$_un_diag_body" \
                    "🐛 CI UNSTABLE DIAGNOSTIC #${pr_number} — \\\`${pr_head_ref}\\\` (issue #${number})" \
                    >/dev/null 2>&1 \
                    && log "issue #${number}: UNSTABLE unit_lint diagnostic card created for PR #${pr_number} (assignee=${_assignee})" \
                    || log "issue #${number}: WARNING UNSTABLE unit_lint diagnostic card create failed for PR #${pr_number}"
                skipped=$((skipped+1)); continue
            fi
            # integration_e2e / unknown → старая rebase-логика (как до фикса).
            # Ретро 12.08 t_8af6bf29: rate-limit — коммент не чаще 1 раза в 2ч.
            if [ -n "${task_id:-}" ]; then
                _last_un="$(kanban_last_reminder_ts "$task_id" "CI UNSTABLE detected")"
                _now_un="$(date +%s)"
                if [ -n "$_last_un" ] && [ $(( _now_un - _last_un )) -lt 7200 ]; then
                    log "issue #${number}: PR #${pr_number} UNSTABLE — rebase reminder rate-limited (last=${_last_un})"
                    skipped=$((skipped+1)); continue
                fi
            fi
            log "issue #${number}: PR #${pr_number} mergeStateStatus=UNSTABLE — appending rebase reminder (class=${_un_class})"
            # Подтягиваем headRefName — UNSTABLE-блок не имеет его из основного цикла (регрессия t_1146)
            pr_head_ref="$(gh pr view "$pr_number" --repo "$GH_REPO" --json headRefName --jq '.headRefName' 2>/dev/null || echo "")"
            [ -z "${pr_head_ref:-}" ] && log "issue #${number}: WARNING cannot fetch headRefName for PR #${pr_number}" && continue
            # Ретро 02.09 t_2bd2e7ea: default → devops fallback.
            _assignee="devops"
            for lbl in $(gh issue view "$number" --repo "$GH_REPO" --json labels --jq '[.labels[].name] | .[]' 2>/dev/null); do
                case "$lbl" in
                    agent:backend)    _assignee="backend"; break ;;
                    agent:developer)  _assignee="developer"; break ;;
                    agent:tester)     _assignee="tester"; break ;;
                    agent:devops)     _assignee="devops"; break ;;
                    agent:architect)  _assignee="architect"; break ;;
                esac
            done
            _reminder="## ⚠️ CI UNSTABLE detected (merge-gate tick, $(date -u +%H:%M:%SZ))

PR #${pr_number} (\`${pr_head_ref}\`) = **mergeable=MERGEABLE + mergeStateStatus=UNSTABLE** (CI fail, но конфликтов с develop нет).

**Что делать** (по процессу Шифу 10.08 — «оно должно взять себе девелоп сейчас и позеленеть»):
1. **В той же ветке** \`${pr_head_ref}\` (тот же PR, никаких новых).
2. **rebase на origin/develop**: \`git fetch origin develop && git rebase origin/develop\`.
3. Push --force-with-lease: \`git push --force-with-lease origin ${pr_head_ref}\`.
4. Следующий CI-прогон автоматом подхватит develop-фиксы → UNSTABLE → CLEAN → merge-gate поставит needs-e2e → e2e-прогон.
5. Карточка закрывается когда PR станет MERGEABLE+CLEAN (CI зелёный).

**Шпаргалка:**
\`\`\`bash
git fetch origin develop
git checkout ${pr_head_ref}
git rebase origin/develop
git add -A && git rebase --continue
git push --force-with-lease origin ${pr_head_ref}
\`\`\`

**ЕСЛИ PR ЗАКРЫТ** (товарищ Шифу «Не делаем это») → rebase НЕ нужен: сделай \`kanban complete\` с пометкой \`PR closed, rebase не нужен\` (ретро 15.08 t_16325ddd).

(Этот reminder автоматически дописан merge-gate — Шифу прямо: «оно должно взять себе девелоп сейчас и позеленеть», не ждать ручного триггера.)"
            if [ -n "${task_id:-}" ]; then
                # Процесс-фикс (18.08, ретро #1356 / t_75e787fd-2 — болтающийся
                # PR): если карточка мёртвая (done/blocked/archived), comment
                # в неё никто не увидит. Здесь — UNSTABLE-версия того же фикса,
                # что и в CONFLICTING-блоке выше.
                _un_card_status="$(kanban_card_status "$task_id")"
                case "$_un_card_status" in
                    running|ready|todo)
                        hermes kanban --board "$KANBAN_BOARD" comment "$task_id" "$_reminder" >/dev/null 2>&1 \
                            || log "issue #${number}: WARNING appending UNSTABLE reminder to ${task_id} failed"
                        log "issue #${number}: UNSTABLE reminder appended to live card ${task_id} (status=${_un_card_status})"
                        ;;
                    done|archived|blocked)
                        # Карточка мёртвая (типично для ретро #1356). Старое
                        # поведение = коммент в пустоту. Шифу прямо: «та же
                        # карточка», но если она закрыта — нужна СВЕЖАЯ recovery.
                        log "issue #${number}: PR #${pr_number} UNSTABLE — карточка ${task_id} мёртвая (status=${_un_card_status}), создаю recovery-карточку (ретро #1356)"
                        _un_branch_matches="$(hermes kanban --board "$KANBAN_BOARD" list --json 2>/dev/null | python3 -c "
import json,sys
try:
    data = json.loads(sys.stdin.read())
except Exception:
    sys.exit(0)
for t in data:
    title = t.get('title','')
    if 'CI UNSTABLE' in title and ('PR #${pr_number}' in title or '${pr_head_ref}' in title):
        print(t['id'], t.get('status',''))
" 2>/dev/null || true)"
                        _un_active="$(printf '%s\n' "$_un_branch_matches" | awk '$2 ~ /^(running|ready|todo)$/ {print $1" "$2; exit}')"
                        _un_blocked="$(printf '%s\n' "$_un_branch_matches" | awk '$2 == "blocked" {print $1; exit}')"
                        if [ -n "$_un_active" ]; then
                            log "issue #${number}: UNSTABLE — recovery card already active (${_un_active}) for PR #${pr_number} — skip"
                        elif [ -n "$_un_blocked" ]; then
                            hermes kanban --board "$KANBAN_BOARD" unblock "$_un_blocked" --reason "⚠️ main-cycle UNSTABLE recovery (ретро #1356)" >/dev/null 2>&1 || true
                            log "issue #${number}: UNSTABLE — recovery card ${_un_blocked} unblocked for PR #${pr_number}"
                        else
                            _skill="architecture-doc-review"  # default архитекторский
                            case "$_assignee" in
                                backend)   _skill="test-driven-development" ;;
                                developer) _skill="test-driven-development" ;;
                                tester)    _skill="test-driven-development" ;;
                                devops)    _skill="agent-flow-e2e-pipeline" ;;  # надзор 13.08: hermes-agent-flow не существует
                            esac
                            # Ретро 15.08 t_16325ddd: PR мог закрыться после получения
                            # pr_state в основном цикле — пере-проверяем перед create.
                            if [ "$(pr_state_now "$pr_number")" = "CLOSED" ]; then
                                log "issue #${number}: PR #${pr_number} CLOSED — UNSTABLE recovery card НЕ создаю (ретро t_16325ddd)"
                            else
                                _un_body="## ⚠️ CI UNSTABLE recovery (merge-gate, ретро #1356, $(date -u +%H:%M:%SZ))

PR #${pr_number} (\`${pr_head_ref}\`) = **UNSTABLE** (CI красный, конфликта с develop нет — rebase + push подтянет develop-фиксы). Связанная рабочая карточка \`${task_id}\` уже закрыта/архивирована, reminder в неё НЕ пишем (потеряется).

**Что делать:** rebase на origin/develop + push --force-with-lease. PR тот же, ветка та же, никаких новых.
\`\`\`bash
git fetch origin develop
git checkout ${pr_head_ref}
git rebase origin/develop
git push --force-with-lease origin ${pr_head_ref}
\`\`\`

Карточка закрывается когда PR станет MERGEABLE+CLEAN (CI зелёный).

(merge-gate создал эту карточку, потому что живая рабочая карточка этого issue уже закрыта — ретро #1356.)"
                                _un_key="unstable-main-cycle-pr-${pr_number}-$(date +%s)"
                                hermes kanban --board "$KANBAN_BOARD" create \
                                    --assignee "$_assignee" --skill "$_skill" --priority 80 --max-runtime 1800 \
                                    --body "$_un_body" \
                                    "⚠️ CI UNSTABLE: rebase \`${pr_head_ref}\` (issue #${number}, PR #${pr_number}, ретро #1356)" \
                                    >/dev/null 2>&1 \
                                    && log "issue #${number}: UNSTABLE recovery card created for PR #${pr_number} (assignee=${_assignee})" \
                                    || log "issue #${number}: WARNING UNSTABLE recovery card create failed for PR #${pr_number}"
                            fi
                        fi
                        ;;
                    *)
                        # неизвестный/не-прочитанный статус — старая логика:
                        # пишем в карточку (лучше чем игнорировать). Если она
                        # реально дохлая → reminder просто не увидят, но это
                        # тот же риск, что был до фикса #1356.
                        hermes kanban --board "$KANBAN_BOARD" comment "$task_id" "$_reminder" >/dev/null 2>&1 \
                            || log "issue #${number}: WARNING appending UNSTABLE reminder to ${task_id} failed (status unknown)"
                        log "issue #${number}: UNSTABLE reminder appended to ${task_id} (status=${_un_card_status} — unknown, treated as live)"
                        ;;
                esac
            else
                # Нет существующей карточки — создаём (skill из профиля assignee).
                _skill="architecture-doc-review"  # default архитекторский, переопределим ниже
                case "$_assignee" in
                    backend)   _skill="test-driven-development" ;;
                    developer) _skill="test-driven-development" ;;
                    tester)    _skill="test-driven-development" ;;
                    devops)    _skill="agent-flow-e2e-pipeline" ;;  # надзор 13.08: hermes-agent-flow не существует → карточка падала в crashed (класс t_6c6c98fb)
                esac
                # Ретро 15.08 t_16325ddd: PR мог закрыться после получения
                # pr_state в основном цикле — пере-проверяем перед create.
                if [ "$(pr_state_now "$pr_number")" = "CLOSED" ]; then
                    log "issue #${number}: PR #${pr_number} CLOSED — UNSTABLE card НЕ создаю (ретро t_16325ddd)"
                    skipped=$((skipped+1)); continue
                fi
                hermes kanban --board "$KANBAN_BOARD" create \
                    --assignee "$_assignee" --skill "$_skill" --priority 80 --max-runtime 1800 \
                    --body "$_reminder" \
                    "⚠️ CI UNSTABLE: rebase \`${pr_head_ref}\` на develop (issue #${number}, PR #${pr_number})" \
                    >/dev/null 2>&1 || log "issue #${number}: WARNING UNSTABLE card create failed"
                log "issue #${number}: UNSTABLE card created for ${_assignee}"
            fi
        else
            log "issue #${number}: PR #${pr_number} mergeStateStatus=${pr_merge_state} — skip"
        fi
        skipped=$((skipped+1)); continue
    fi

    # --- dead-content guard (ретро 22.08 t_e8d52cb7 / t_944df2c5, PR #1507) ---
    # Сценарий: после rebase develop ушёл вперёд, в PR остались ТОЛЬКО
    # binary/asset файлы (.ogg/.png/.bin/.wav/.jpg/...) — все meaningful
    # изменения (код/тесты/конфиги) уже в develop. PR висит OPEN +
    # MERGEABLE + CLEAN, но merge-ui показывает его как нормальный — на
    # самом деле merge протащит только asset-мусор (или пустой merge
    # commit). Кейс PR #1507: 6 voice .ogg vs develop = 0 meaningful файлов,
    # acceptance issue #1506 уже выполнен в develop, но PR висел без
    # маркеров → issue закрылся «лишним» sibling-signal.
    #
    # Guard: если в PR diff НЕТ meaningful файлов (.py|.json|.yaml|.yml|
    # .toml|.md|.sh|.ts|.cpp|.h|.hpp|.launch.xml|.txt) И файлы вообще есть
    # (не пустой merge-commit — это отдельный случай, не трогаем) →
    # помечаем PR меткой `dead-content` и комментим issue: «PR #N после
    # rebase содержит только binary (X файлов), 0 meaningful. Suite уже в
    # develop. Рекомендация: закрыть PR без merge». Сам PR НЕ закрываем —
    # архитектор/Шифу принимает финальное решение. Если в PR появится
    # meaningful-файл (новый коммит), следующий тик снимет метку (но пока
    # этого нет — метка прилипает, как `needs-review`/`needs-e2e`).
    #
    # Позиция: ПОСЛЕ всех guards (state=OPEN, mergeable=MERGEABLE,
    # merge_state=CLEAN), ДО classify (lint / big-bang / needs-e2e).
    # Цель — не дать dead-content PR «проскочить» в rotation e2e-process.
    _dead_flag=0
    if [ "$pr_state" = "OPEN" ] && [ "$pr_mergeable" = "MERGEABLE" ] \
        && [ "$pr_merge_state" = "CLEAN" ]; then
        if [ "$(pr_is_dead_content "$pr_number")" = "1" ]; then
            _dead_flag=1
            log "issue #${number}: ⚠️ PR #${pr_number} DEAD-CONTENT detected (after rebase: only binary, 0 meaningful files) — flagging"
        fi
    fi
    if [ "$_dead_flag" = "1" ]; then
        # Идемпотентность: если метка уже есть — только пишем comment (если
        # ещё не было), иначе skip. Иначе на каждом тике merge-gate будет
        # спамить PR и issue повторным label/comment (10 минут × N циклов).
        _dead_has_label=0
        if printf '%s' "$pr_labels_csv" | grep -Eq '(^|,)dead-content(,|$)'; then
            _dead_has_label=1
        fi
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would: dead-content guard on PR #${pr_number} (label_present=${_dead_has_label})"
        else
            if [ "$_dead_has_label" = "0" ]; then
                # issue #1553: self-id whoami BEFORE dead-content (label-of-doom).
                post_whoami_comment pr "$pr_number" "adding-label:dead-content" \
                    "0 meaningful files after rebase — PR dead" "issue=${number}"
                gh pr edit "$pr_number" --repo "$GH_REPO" --add-label "dead-content" >/dev/null 2>&1 \
                    && log "issue #${number}: PR #${pr_number} помечен dead-content (после rebase — 0 meaningful файлов)" \
                    || log "WARNING: failed to add dead-content label to PR #${pr_number}"
            else
                log "issue #${number}: PR #${pr_number} уже имеет dead-content — skip label add"
            fi
            # Comment-on-issue: 24h dedup (как в big-bang / stale-rebase блоках),
            # чтобы не спамить при каждом тике merge-gate (~10 мин).
            _dead_dedup_since="$(date -u -d '24 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null \
                || date -u +%Y-%m-%dT%H:%M:%SZ)"
            _dead_dup_count="$(gh api "repos/${GH_REPO}/issues/${number}/comments?since=${_dead_dedup_since}&per_page=100" \
                --jq '[.[] | select(.body | contains("DEAD-CONTENT detected"))] | length' 2>/dev/null || echo 0)"
            _dead_files_count="$(gh pr view "$pr_number" --repo "$GH_REPO" --json files \
                --jq '[.files[].path] | length' 2>/dev/null || echo 0)"
            if [ "${_dead_dup_count:-0}" -eq 0 ] 2>/dev/null; then
                gh issue comment "$number" --repo "$GH_REPO" --body \
                    "🪦 **PR #${pr_number} DEAD-CONTENT detected** (merge-gate, ретро 22.08 t_e8d52cb7, $(date -u +%H:%M:%SZ))

После rebase develop ушёл вперёд: PR #${pr_number} → develop содержит **только binary (${_dead_files_count} файлов), 0 meaningful файлов** (код/тесты/конфиги).

Suite/acceptance уже в develop — orphan-PR остался висеть OPEN MERGEABLE CLEAN без маркеров. PR помечен \`dead-content\`.

**Рекомендация:** закрыть PR #${pr_number} без merge (комментарий или squash в develop уже не нужен — всё, что было ценного, в develop).

Guard будет повторять alert, пока PR не закрыт или пока в нём не появится meaningful-файл (новый коммит)." >/dev/null 2>&1 \
                    && log "issue #${number}: dead-content comment posted (24h dedup, files=${_dead_files_count})" \
                    || log "WARNING: dead-content comment post failed for issue #${number}"
            else
                log "issue #${number}: dead-content comment already posted (×${_dead_dup_count} за 24ч) — dedup skip"
            fi
        fi
        # Skip дальнейшей классификации (lint/big-bang/needs-e2e) — dead-content
        # PR не должен попадать в e2e rotation или merge-ui без явного override.
        # continue в main-cycle, scan-all-prs ниже подхватит только нужное.
        labeled=$((labeled+1)); continue
    fi

    # --- stale-rebase watchdog (ретро 22.08 t_562a8682) ---------------------
    # Сценарий: PR с needs-review, CI green & clean (mergeStateStatus=CLEAN),
    # но ahead-of-develop > STALE_REBASE_AHEAD_THRESHOLD. GitHub merge-ui
    # покажет такой PR как MERGEABLE (rebase возможен), НО фактический
    # merge-base = ДРЕВНИЙ develop → merge-commit протащит десятки/сотни
    # коммитов в develop (или огромный merge commit с потенциальными
    # конфликтами). Юзер жмёт merge → CONFLICTING merge → потеря времени →
    # ручной rebase воркером.
    #
    # Решение: ДЛЯ PR с меткой needs-review проверяем ahead через REST
    # compare API. Если выше порога — alert в карточку воркера (rate-
    # limited 2ч, как в UNSTABLE-блоке) + comment-on-issue с инструкцией
    # rebase (24h dedup, как в post-merge cleanup). Сам PR НЕ блокируем —
    # это watchdog, не gate: lint-PR, big-bang-override, e2e-done НЕ
    # задеваем.
    if has_label "${pr_labels_csv,,}" "$NEEDS_REVIEW_LABEL" \
        && [ "$pr_state" = "OPEN" ]; then
        _sr_ahead="$(pr_compare_ahead "$branch" "$DEVELOP_BRANCH")"
        # Fail-open: пустой/non-numeric → 0 (нет alert, не зашумить).
        [ -z "$_sr_ahead" ] && _sr_ahead=0
        case "$_sr_ahead" in
            ''|*[!0-9]*) _sr_ahead=0 ;;
        esac
        if [ "${_sr_ahead:-0}" -gt "${STALE_REBASE_AHEAD_THRESHOLD}" ] 2>/dev/null; then
            # Rate-limit: коммент не чаще 1 раза в STALE_REBASE_REMINDER_COOLDOWN_SECONDS.
            # Если карточка есть — пишем reminder в неё (аналог UNSTABLE-блока).
            _sr_skip="0"
            if [ -n "${task_id:-}" ]; then
                _last_sr="$(kanban_last_reminder_ts "$task_id" "STALE REBASE detected")"
                _now_sr="$(date +%s)"
                if [ -n "$_last_sr" ] \
                    && [ $(( _now_sr - _last_sr )) -lt "${STALE_REBASE_REMINDER_COOLDOWN_SECONDS}" ]; then
                    log "issue #${number}: PR #${pr_number} ahead=${_sr_ahead} > ${STALE_REBASE_AHEAD_THRESHOLD} — rebase reminder rate-limited (last=${_last_sr})"
                    _sr_skip="1"
                fi
            fi
            if [ "$_sr_skip" = "0" ]; then
                log "issue #${number}: 🟡 PR #${pr_number} (${branch}) ahead=${_sr_ahead} > ${STALE_REBASE_AHEAD_THRESHOLD} (CLEAN) — stale rebase detected"
                _sr_reminder="## 🟡 STALE REBASE detected (merge-gate, $(date -u +%H:%M:%SZ))

PR #${pr_number} (\\\`${branch}\\\`) → \\\\${DEVELOP_BRANCH}\\\` = **mergeable=MERGEABLE + mergeStateStatus=CLEAN**, но **ahead=${_sr_ahead} коммитов** (порог: ${STALE_REBASE_AHEAD_THRESHOLD}).

CI зелёный, конфликтов в GitHub нет — но merge-base уехал далеко назад. Если Шифу нажмёт merge-ui сейчас → github сделает merge-commit с ${_sr_ahead} коммитами впереди develop (или огромный merge commit с потенциальными конфликтами). Потеря времени + ручной rebase.

**Что делать** (тот же PR, та же ветка, никаких новых):
1. **rebase на origin/${DEVELOP_BRANCH}**: \\\`git fetch origin ${DEVELOP_BRANCH} && git rebase origin/${DEVELOP_BRANCH}\\\`.
2. Push --force-with-lease: \\\`git push --force-with-lease origin ${branch}\\\`.
3. После push ahead должен стать ≤ ${STALE_REBASE_AHEAD_THRESHOLD} → watchdog снимет alert.

\\\`\\\`\\\`bash
git fetch origin ${DEVELOP_BRANCH}
git checkout ${branch}
git rebase origin/${DEVELOP_BRANCH}
git add -A && git rebase --continue
git push --force-with-lease origin ${branch}
\\\`\\\`\\\`

**Шпаргалка по диагностике** (для будущих проверок):
\\\`\\\`\\\`bash
# ahead/behind в числовом виде:
gh api repos/${GH_REPO}/compare/${DEVELOP_BRANCH}...${branch} \\
    --jq '.ahead_by,.behind_by,.status'

# Или через git (если есть локальный clone):
git fetch origin ${DEVELOP_BRANCH}
git rev-list --left-right --count origin/${DEVELOP_BRANCH}...${branch}
\\\`\\\`\\\`

(Этот alert автоматически дописан merge-gate, ретро 22.08 t_562a8682 — open PR с needs-review отстал от develop. Шифу прямо: «оно должно взять себе девелоп сейчас и позеленеть».)"
                if [ -n "${task_id:-}" ]; then
                    _sr_card_status="$(kanban_card_status "$task_id")"
                    case "$_sr_card_status" in
                        running|ready|todo)
                            if [ "$DRY_RUN" = "true" ]; then
                                log "DRY-RUN would: append STALE REBASE reminder to live card ${task_id} (status=${_sr_card_status})"
                            else
                                hermes kanban --board "$KANBAN_BOARD" comment "$task_id" "$_sr_reminder" >/dev/null 2>&1 \
                                    || log "issue #${number}: WARNING appending STALE REBASE reminder to ${task_id} failed"
                                log "issue #${number}: STALE REBASE reminder appended to live card ${task_id} (status=${_sr_card_status}, ahead=${_sr_ahead})"
                            fi
                            ;;
                        done|archived|blocked)
                            # Карточка мёртвая — пишем comment-on-issue (24h dedup),
                            # чтобы Шифу увидел в issue timeline. Не создаём
                            # новую recovery-карточку: rebase — задача исходного
                            # автора ветки, не отдельный recovery-run (в отличие
                            # от UNSTABLE / CONFLICTING, где нужен e2e на новой
                            # фикс-ветке).
                            log "issue #${number}: STALE REBASE — карточка ${task_id} мёртвая (status=${_sr_card_status}), пишу только в issue"
                            _sr_dedup_since="$(date -u -d "${STALE_REBASE_COMMENT_DEDUP_HOURS} hours ago" +%Y-%m-%dT%H:%M:%SZ 2>/dev/null \
                                || date -u +%Y-%m-%dT%H:%M:%SZ)"
                            _sr_dup="$(gh api "repos/${GH_REPO}/issues/${number}/comments?since=${_sr_dedup_since}&per_page=100" \
                                --jq '[.[] | select(.body | contains("STALE REBASE detected"))] | length' 2>/dev/null || echo 0)"
                            if [ "${_sr_dup:-0}" -eq 0 ]; then
                                gh issue comment "$number" --repo "$GH_REPO" --body "$_sr_reminder" >/dev/null 2>&1 || true
                                log "issue #${number}: STALE REBASE comment posted on issue #${number} (dedup=${_sr_dup:-0}, ahead=${_sr_ahead})"
                            else
                                log "issue #${number}: STALE REBASE comment already posted on issue #${number} (×${_sr_dup} за ${STALE_REBASE_COMMENT_DEDUP_HOURS}h) — dedup skip"
                            fi
                            ;;
                        *)
                            # неизвестный/не-прочитанный статус — пишем в карточку
                            # (лучше чем игнорировать; если карточка реально мёртвая,
                            # reminder просто не увидят, как до фикса #1356).
                            if [ "$DRY_RUN" = "true" ]; then
                                log "DRY-RUN would: append STALE REBASE reminder to ${task_id} (status=${_sr_card_status} — unknown)"
                            else
                                hermes kanban --board "$KANBAN_BOARD" comment "$task_id" "$_sr_reminder" >/dev/null 2>&1 \
                                    || log "issue #${number}: WARNING appending STALE REBASE reminder to ${task_id} failed (status unknown)"
                                log "issue #${number}: STALE REBASE reminder appended to ${task_id} (status=${_sr_card_status} — unknown, ahead=${_sr_ahead})"
                            fi
                            ;;
                    esac
                else
                    # task_id пуст (issue без маркера «kanban: t_xxx»).
                    # Пишем comment-on-issue (24h dedup). Scan-all-prs подберёт
                    # для создания recovery-карточки, если понадобится.
                    log "issue #${number}: STALE REBASE — task_id пуст, пишу comment-on-issue"
                    _sr_dedup_since="$(date -u -d "${STALE_REBASE_COMMENT_DEDUP_HOURS} hours ago" +%Y-%m-%dT%H:%M:%SZ 2>/dev/null \
                        || date -u +%Y-%m-%dT%H:%M:%SZ)"
                    _sr_dup="$(gh api "repos/${GH_REPO}/issues/${number}/comments?since=${_sr_dedup_since}&per_page=100" \
                        --jq '[.[] | select(.body | contains("STALE REBASE detected"))] | length' 2>/dev/null || echo 0)"
                    if [ "${_sr_dup:-0}" -eq 0 ]; then
                        gh issue comment "$number" --repo "$GH_REPO" --body "$_sr_reminder" >/dev/null 2>&1 || true
                        log "issue #${number}: STALE REBASE comment posted on issue #${number} (task_id пуст, ahead=${_sr_ahead})"
                    else
                        log "issue #${number}: STALE REBASE comment already posted on issue #${number} (×${_sr_dup}) — dedup skip"
                    fi
                fi
            fi
            # Watchdog: НЕ continue — пусть процесс пойдёт дальше (lint-PR
            # path / big-bang gate / needs-e2e / needs-review), но в логах
            # остаётся инфа что ahead завышен. Если Шифу руками нажмёт
            # merge-ui после alert — это его решение, merge-gate не блокирует.
        fi
    fi

    # All green — classify PR by kind (lint vs functional, ретро 10.08 #2).
    # lint: lint/refactor PR (только стиль/докстринги/формат) → e2e НЕ обязателен,
    #       воркер комментирует только по желанию; ставим `needs-review` напрямую.
    # functional: всё остальное → e2e обязателен, ставим `needs-e2e`.
    pr_kind="$(detect_pr_kind "$pr_labels_csv" "$pr_title")"
    log "issue #${number} PR #${pr_number} kind=${pr_kind} (CI green & clean)"

    # --- ADR-номер collision guard (ретро 25.08 t_00ba0224) -----------------
    # Если PR добавляет/переименовывает docs/adr/NNNN-*.md и в develop уже
    # занят тот же NNNN другим файлом → блокируем needs-e2e (Шифу либо
    # правит имя, либо ставит override). По дизайну ставим ДО big-bang-override
    # и lint-веток, потому что collision — это структурная ошибка ADR-процесса,
    # которую lint-режим тоже не должен пропускать (линтер/доки коммитятся
    # отдельным PR'ом — отдельные файлы с уникальными номерами).
    labels_norm="$(printf '%s' "$pr_labels_csv" | tr '[:upper:]' '[:lower:]')"
    if ! check_adr_number_collision "$pr_number" "$number" "$labels_norm"; then
        # КОЛЛИЗИЯ: needs-e2e НЕ ставим, в scan-all-prs PR не попадёт
        # (там фильтр по OPEN+mergeable, не по label). PR остаётся висеть
        # OPEN — Шифу увидит alert в issue и либо fix rename, либо override.
        labeled=$((labeled+1)); continue
    fi

    # --- big-bang-override gate (ADR-0013, ретро t_9726053d) ---------------
    # PR > ${BIG_BANG_MAX_COMMITS} коммитов ИЛИ > ${BIG_BANG_MAX_LINES} строк
    # ЗАПРЕЩЁН без явной метки ${BIG_BANG_OVERRIDE_LABEL} на issue. Метку
    # ставит ТОЛЬКО товарищ Шифу (см. CONTRIBUTING.md §69-71). Поведение:
    #   - НЕ ставить needs-e2e / needs-review (иначе e2e-process поглотит PR)
    #   - оставить PR-комментарий (с дедупликацией как в post-merge close):
    #     "PR #N: size превышает ADR-0013, требуется split ИЛИ @Шифу ставит override"
    #   - comment постится ровно один раз за 24h (как cleanup-коммент в §5
    #     post-merge), иначе 6 одинаковых сообщений за час (ретро 10.08 t_9caf5d52).
    # Override → стандартный путь (needs-e2e / needs-review).
    _bb_reasons=""
    if [ "${pr_commits_count:-0}" -gt "${BIG_BANG_MAX_COMMITS}" ] 2>/dev/null; then
        _bb_reasons="${_bb_reasons}${pr_commits_count} коммитов > ${BIG_BANG_MAX_COMMITS}; "
    fi
    if [ "${pr_additions:-0}" -gt "${BIG_BANG_MAX_LINES}" ] 2>/dev/null; then
        _bb_reasons="${_bb_reasons}${pr_additions} строк > ${BIG_BANG_MAX_LINES}; "
    fi
    if [ -n "$_bb_reasons" ]; then
        if has_label "$labels_norm" "$BIG_BANG_OVERRIDE_LABEL"; then
            log "issue #${number}: big-bang (${_bb_reasons% ;}) но override ${BIG_BANG_OVERRIDE_LABEL} есть — пропускаем gate"
        else
            log "issue #${number}: PR #${pr_number} BIG-BANG (${_bb_reasons% ;}) — ${BIG_BANG_OVERRIDE_LABEL} отсутствует, block needs-e2e"
            if [ "$DRY_RUN" = "true" ]; then
                log "DRY-RUN would: skip needs-e2e + post big-bang comment to PR #${pr_number} (dedup 24h)"
                labeled=$((labeled+1)); continue
            fi
            # Дедупликация как в post-merge cleanup: за последние 24h
            # постим только один раз. Сейчас PR огромный (4850/100),
            # round-49..54 → 6 одинаковых комментов = спам. Шифу прямо:
            # «один раз label-коммент, round больше не запускается».
            _bb_dedup_since="$(date -u -d '24 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
            _bb_dup_count="$(gh api "repos/${GH_REPO}/issues/${pr_number}/comments?since=${_bb_dedup_since}&per_page=100" \
                --jq '[.[] | select(.body | startswith("🚨 **PR #'"${pr_number}"' BIG-BANG"))] | length' 2>/dev/null || echo 0)"
            if [ "${_bb_dup_count:-0}" -gt 0 ] 2>/dev/null; then
                log "issue #${number}: big-bang comment на PR #${pr_number} уже проставлен (×${_bb_dup_count} за 24h) — dedup skip"
                skipped=$((skipped+1)); continue
            fi
            # Коммент И на issue (чтобы воркер увидел в task), И на PR
            # (чтобы ревьюер/Шифу увидел). Один раз каждый.
            gh issue comment "$number" --repo "$GH_REPO" --body \
                "🚨 **PR #${pr_number} BIG-BANG** — нарушение ADR-0013: ${_bb_reasons% ;}.

Требуется:
1. **split** на инкрементальные PR (по 1 эпику на issue), каждый проходит e2e отдельно, ИЛИ
2. **товарищ Шифу** ставит метку \`${BIG_BANG_OVERRIDE_LABEL}\` на этот issue (явный override).

Merge-gate **НЕ поставит ${NEEDS_E2E_LABEL}** без override. Round-процесс не будет автоматически гонять e2e на этом PR.

Ссылки: ADR-0013 (docs/adr/0013-incremental-delivery-over-big-bang.md), CONTRIBUTING.md §69-71." >/dev/null 2>&1 || true
            gh pr comment "$pr_number" --repo "$GH_REPO" --body \
                "🚨 **PR #${pr_number} BIG-BANG** — нарушение ADR-0013: ${_bb_reasons% ;}.

Merge-gate блокирует e2e-ротацию: ${NEEDS_E2E_LABEL} не будет поставлен.

Что делать:
- **split** на инкрементальные PR (по 1 эпику), ИЛИ
- **товарищ Шифу** ставит \`${BIG_BANG_OVERRIDE_LABEL}\` на issue #${number}." >/dev/null 2>&1 || true
            # Помечаем issue меткой agent-flow:big-bang-blocked (best-effort) — чтобы
            # triage/воркер/дашборд видели, что issue ждёт решения. Не критично
            # если метка уже есть или label API упадёт.
            # issue #1553: self-id whoami BEFORE big-bang-blocked (terminal блокер).
            whoami_add_label "$number" "agent-flow:big-bang-blocked" \
                "big-bang threshold violation: PR #${pr_number} превышает лимит (merge-gate ждёт ${BIG_BANG_OVERRIDE_LABEL} от Шифу)"
            gh issue edit "$number" --repo "$GH_REPO" --add-label "agent-flow:big-bang-blocked" >/dev/null 2>&1 || true
            labeled=$((labeled+1)); continue
        fi
    fi

    if [ "$pr_kind" = "lint" ]; then
        # --- user-unlabel guard (ретро 18.08 t_de6bea69, PR #1398) -----------
        # lint-PR: CI green → ставим needs-review напрямую (без e2e). Если
        # Шифу руками снял needs-review после предыдущего auto-установки —
        # НЕ возвращаем.
        if user_removed_label_recently "$pr_number" "$NEEDS_REVIEW_LABEL"; then
            user_unlabel_log_skip "$pr_number" "$NEEDS_REVIEW_LABEL" "lint PR path"
            labeled=$((labeled+1))
            if [ "$DRY_RUN" != "true" ] && user_unlabel_should_notify "$pr_number" "$NEEDS_REVIEW_LABEL"; then
                gh pr comment "$pr_number" --repo "$GH_REPO" --body \
                    "agent-flow: ⏸️ merge-gate lint path не восстановил \`needs-review\` — ты её ранее снял руками; жду твоего решения (ретро 18.08 t_de6bea69, Q22)." >/dev/null 2>&1 || true
                user_unlabel_mark_notified "$pr_number" "$NEEDS_REVIEW_LABEL" "lint PR path" || true
            fi
            continue
        fi
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would run: gh pr edit ${pr_number} --repo ${GH_REPO} --add-label ${NEEDS_REVIEW_LABEL}"
            labeled=$((labeled+1)); continue
        fi
        if ! gh pr edit "$pr_number" --repo "$GH_REPO" --add-label "$NEEDS_REVIEW_LABEL" >/dev/null 2>&1; then
            log "WARNING: failed to add ${NEEDS_REVIEW_LABEL} to PR #${pr_number} — will retry next tick"
            errored=$((errored+1)); continue
        fi
        # issue #1553: self-id whoami AFTER успешного add-label (если label API
        # упал — whoami не публикуем, чтобы не врать что лимит поставили).
        # helper идемпотентный — при reconcile в следующем тике skip.
        post_whoami_comment pr "$pr_number" "adding-label:${NEEDS_REVIEW_LABEL}" \
            "lint PR #${pr_number} → needs-review (CI green, e2e не нужен)" "issue=${number}"
        log "issue #${number}: lint PR #${pr_number} → ${NEEDS_REVIEW_LABEL} (skip e2e)"
        labeled=$((labeled+1)); continue
    fi

    log "issue #${number} PR #${pr_number} is green & clean — labeling ${NEEDS_E2E_LABEL}"
    if [ "$DRY_RUN" = "true" ]; then
        log "DRY-RUN would run: gh issue edit ${number} --repo ${GH_REPO} --add-label ${NEEDS_E2E_LABEL}"
        labeled=$((labeled+1)); continue
    fi

    # issue #1534: self-id whoami BEFORE adding ${NEEDS_E2E_LABEL} — это
    # ключевой «PR вошёл в e2e-очередь» transition; в истории GitHub часто
    # непонятно кто поставил метку (krikz = holder of GH token). helper
    # идемпотентный: если за последние 2h уже публиковал такой же marker —
    # skip (защита от reconcile-loop дублей).
    whoami_add_label "$number" "${NEEDS_E2E_LABEL}" "PR #${pr_number} green & clean → e2e queue" "pr=${pr_number}"

    if ! gh issue edit "$number" --repo "$GH_REPO" --add-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1; then
        log "WARNING: failed to add label to issue #${number} — will retry next tick"
        errored=$((errored+1)); continue
    fi

    # Best-effort: also propagate the label to the PR so downstream PR-side
    # gates (e.g. GitHub Actions that watch PR labels) stay consistent.
    gh pr edit "$pr_number" --repo "$GH_REPO" --add-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true

    labeled=$((labeled+1))
done < <(printf '%s' "$issues_json" | python3 -c '
import json, sys
data = json.load(sys.stdin)
for issue in data:
    n = issue["number"]
    t = issue["title"]
    l = ",".join(sorted({lab["name"] for lab in issue.get("labels", [])}))
    b = issue.get("body") or ""
    def esc(s):
        return s.replace("\\", "\\\\").replace("\t", "\\t").replace("\n", "\\n")
    sys.stdout.write(f"{n}\t{esc(t)}\t{esc(l)}\t{esc(b)}\n")
')

# --- Дополнительный цикл (10.08, ретро t_51b5ad24 — Шифу прямо: «оно должно
# взять себе девелоп сейчас и позеленеть»): сканируем ВСЕ open PR на
# UNSTABLE/CONFLICTING, а не только те что привязаны к issues с меткой
# needs-e2e. Иначе PR без меток (как #1096 downmix тесты) висят красными
# без внимания процесса. Находим связанный task_id по head-branch (wt/<id>-*
# → t_<id>) или по issue_number в body → дописываем reminder в существующую
# карточку (та же карточка должна знать, Шифу прямо).
log "scan-all-prs: scanning ALL open PRs for UNSTABLE/CONFLICTING (not just needs-e2e)"

# --- stale-branch re-commit scan для ВСЕХ open PR (ретро 12.08 t_d3aeaa9b) --
# Основной цикл выше ловит PR, привязанные к issues с меткой hermes. Но
# ретро-ветки devops/architect (z-devops/t_<id>-..., z-architect/t_<id>-...)
# issues НЕ имеют — их stale-re-commit PR пролетит мимо основного цикла.
# Функция stale_branch_scan_all() вызывается здесь (основной путь с issues)
# и в блоке no-issues (до раннего exit).
stale_branch_scan_all
# Stale-CONFLICTING watcher (ретро 24.08 t_cd32788f): один вызов рядом
# со stale_branch_scan_all. Без условий на issues_json — сканирует ВСЕ
# open needs-e2e PR независимо от issue-cycle (stale branch мог остаться
# от archived-issue). Skip если HELM_HOOK_DRY_RUN.
stale_conflicting_scan_all
# Дубль-файл scan (ретро 15.08 t_20383d32): тот же паттерн вызова, что у
# stale_branch_scan_all — основной путь + no-issues путь сходятся сюда.
duplicate_file_scan_all
# PR-without-kanban-marker scan (ретро 25.08 t_1a4f3275 / issue #1624):
# тот же паттерн вызова — основной путь + no-issues путь сходятся сюда.
pr_without_marker_scan_all
# Deploy-issue label-less orphan backstop (ретро 15.08 t_238ff3f7): тот же
# паттерн вызова — основной путь + no-issues путь сходятся сюда.
deploy_issue_reconcile_all
# Ретро 31.08 t_9d375e3e / ADR-0035: stale-after-upstream-fix detector.
# Безопасно вызывать в начале секции: не зависит от issues_json, сканирует
# только kanban board. Дешёвая проверка (≤10 live diagnostic-карточек типично).
# Skip если STALE_AFTER_UPSTREAM_FIX_SCAN=false.
stale_after_upstream_fix_scan_all

# Маппинг head-branch → task_id через wt/... ветки (t_51b5ad24-respeaker-downmix-tests → t_51b5ad24)
_prs_json="$(gh pr list --repo "$GH_REPO" --state open \
    --json number,title,headRefName,mergeable,mergeStateStatus,labels 2>/dev/null || echo '[]')"

printf '%s' "$_prs_json" | python3 -c '
import json, sys, re, subprocess
data = json.load(sys.stdin)
for pr in data:
    pr_num = pr["number"]
    head = pr["headRefName"]
    mergeable = pr.get("mergeable", "")
    merge_state = pr.get("mergeStateStatus", "")
    # Ретро 12.08 t_618208c0: DIRTY mergeStateStatus = «merge commit cannot be
    # cleanly created» (конфликт с develop). GitHub считает mergeable асинхронно
    # и может отдать UNKNOWN при реальном конфликте (кейс #1165: gh api →
    # CONFLICTING/UNKNOWN) — поэтому ловим и mergeable=CONFLICTING, и
    # mergeStateStatus=DIRTY. Раньше DIRTY-ветка «никому не принадлежала»
    # (не clean ≠ rejected) → серая зона merge-gate.
    if mergeable not in ("CONFLICTING",) and merge_state not in ("UNSTABLE", "DIRTY"):
        continue
    # Определяем issue_number: из PR title (#NNNN) или из branch (z-{agent}/NNNN-*).
    # Сначала пробуем title (#NNNN), иначе ищем NNNN- в branch, но НЕ t_xxxx (это task_id).
    m = re.search(r"#(\d+)", pr.get("title",""))
    issue_num = m.group(1) if m else ""
    if not issue_num:
        m2 = re.search(r"z-\{agent\}/(\d+)-", head)
        issue_num = m2.group(1) if m2 else ""
    # Чистим если это всё ещё task_id (на случай кривой регулярки)
    if issue_num.startswith("t_") or len(issue_num) > 7:
        issue_num = ""
    # Определяем task_id: t_<id> из branch (z-{agent}/t_<id>-...) или wt/t_<id>
    m2 = re.search(r"t_([a-f0-9]+)", head)
    task_id = ("t_" + m2.group(1)) if m2 else ""
    # Fallback: если task_id не найден в branch, но знаем issue_num — ищем
    # карточку по комментам issue (паттерн "kanban: t_xxx", как основной цикл).
    # ВНИМАНИЕ: внутри python3 -c '...' (bash single-quote) нельзя писать
    # одинарную кавычку буквально — используем chr(39) для генерации в bash-строку.
    if not task_id and issue_num:
        q = chr(39)
        # Ищем карточку по комментам issue (паттерн "kanban: t_xxx") и
        # проверяем что она НЕ archived: перебираем кандидатов С КОНЦА
        # (последняя = актуальная), берём первую не-archived.
        r = subprocess.run(
            ["bash", "-c",
             f"gh issue view {issue_num} --repo krikz/rob_box_project --comments --json comments 2>/dev/null "
             f"| grep -Eo {q}kanban: t_[a-f0-9]+{q} | sort -u | tail -n10"],
            capture_output=True, text=True)
        cands = [c.strip().replace("kanban: ", "") for c in r.stdout.splitlines() if c.strip()]
        # Ретро 12.08 t_8af6bf29: `hermes kanban show` падает после v0.20.0
        # (sqlite3.ProgrammingError) — используем `list --json` (работает).
        _list_raw = subprocess.run(
            ["bash", "-c", "hermes kanban --board robbox list --json 2>/dev/null"],
            capture_output=True, text=True).stdout
        _card_status = {}
        try:
            _d = json.loads(_list_raw)
            _tasks = _d if isinstance(_d, list) else _d.get("tasks", [])
            for _t in _tasks:
                _card_status[_t.get("id", "")] = _t.get("status", "")
        except Exception:
            pass
        for cand in reversed(cands):
            st = _card_status.get(cand, "")
            if "archived" not in st:
                task_id = cand
                break
    # Выводим с placeholder для пустых полей (иначе read с IFS=\t сливает
    # последовательные табы и поля сдвигаются: issue_num="" теряется)
    issue_num_out = issue_num if issue_num else "-"
    task_id_out = task_id if task_id else "-"
    print(f"{pr_num}\t{head}\t{mergeable}\t{merge_state}\t{issue_num_out}\t{task_id_out}")
' 2>/dev/null | while IFS=$'\t' read -r pr_num head mergeable merge_state issue_num task_id; do
    [ -z "$pr_num" ] && continue
    # Placeholder "-" → пустая строка
    [ "$issue_num" = "-" ] && issue_num=""
    [ "$task_id" = "-" ] && task_id=""
    log "scan-all-prs: PR #${pr_num} (${head}) mergeable=${mergeable} state=${merge_state}"

    # Ретро 15.08 t_16325ddd (гонка PR-state): PR был open на момент `gh pr list`,
    # но товарищ Шифу мог закрыть его («Не делаем это») пока merge-gate шёл к
    # create. Карточка для закрытого PR = мёртвая (rebase невозможен, раунда не
    # будет). Пере-проверяем state ПЕРЕД созданием карточки — CLOSED → SKIP.
    if [ "$(pr_state_now "$pr_num")" = "CLOSED" ]; then
        log "scan-all-prs: PR #${pr_num} CLOSED (товарищ Шифу «Не делаем это») — SKIP, карточка не нужна (ретро t_16325ddd)"
        continue
    fi

    # Определяем assignee по меткам issue (если знаем issue_num)
    # Ретро 02.09 t_2bd2e7ea: default → devops fallback (default невалиден).
    _assignee="devops"
    if [ -n "$issue_num" ]; then
        for lbl in $(gh issue view "$issue_num" --repo "$GH_REPO" --json labels --jq '[.labels[].name] | .[]' 2>/dev/null); do
            case "$lbl" in
                agent:backend)    _assignee="backend"; break ;;
                agent:developer)  _assignee="developer"; break ;;
                agent:tester)     _assignee="tester"; break ;;
                agent:devops)     _assignee="devops"; break ;;
                agent:architect)  _assignee="architect"; break ;;
            esac
        done
    fi

    # Формируем reminder (тот же текст, что в основном цикле).
    # Ретро 12.08 t_618208c0: mergeable=CONFLICTING ИЛИ mergeStateStatus=DIRTY
    # (GitHub отдаёт их асинхронно — см. фильтр выше) → это КОНФЛИКТ, а не
    # UNSTABLE. Раньше DIRTY-ветка получала UNSTABLE-reminder (неверный текст).
    if [ "$mergeable" = "CONFLICTING" ] || [ "$merge_state" = "DIRTY" ]; then
        _reminder="## 🔀 merge conflict detected (merge-gate scan-all-prs, $(date -u +%H:%M:%SZ))

PR #${pr_num} (\`${head}\`) → develop = **CONFLICTING**.

**ОБЯЗАН** (по процессу Шифу 10.08):
1. **В той же ветке** \`${head}\` — НЕ создавай новую ветку и НЕ новый PR.
2. **rebase** на origin/develop.
3. Разреши конфликты → \`git add -A && git rebase --continue\`.
4. \`git push --force-with-lease origin ${head}\`.
5. Карточка закрывается когда PR станет MERGEABLE.

**Шпаргалка:**
\`\`\`bash
git fetch origin develop
git checkout ${head}
git rebase origin/develop
git add -A && git rebase --continue
git push --force-with-lease origin ${head}
\`\`\`

**ЕСЛИ PR ЗАКРЫТ** (товарищ Шифу «Не делаем это») → rebase НЕ нужен: сделай \`kanban complete\` с пометкой \`PR closed, rebase не нужен\` (ретро 15.08 t_16325ddd)."
        _title_prefix="🔀 merge conflict"
    else
        _reminder="## ⚠️ CI UNSTABLE detected (merge-gate scan-all-prs, $(date -u +%H:%M:%SZ))

PR #${pr_num} (\`${head}\`) = **MERGEABLE + UNSTABLE** (CI fail, конфликтов нет).

**Что делать** (по процессу Шифу 10.08 — «взять себе девелоп сейчас и позеленеть»):
1. **В той же ветке** \`${head}\`.
2. \`git fetch origin develop && git rebase origin/develop\`.
3. \`git push --force-with-lease origin ${head}\`.
4. Следующий CI-прогон подхватит develop-фиксы → CLEAN → merge-gate поставит needs-e2e.

**Шпаргалка:**
\`\`\`bash
git fetch origin develop
git checkout ${head}
git rebase origin/develop
git add -A && git rebase --continue
git push --force-with-lease origin ${head}
\`\`\`

**ЕСЛИ PR ЗАКРЫТ** (товарищ Шифу «Не делаем это») → rebase НЕ нужен: сделай \`kanban complete\` с пометкой \`PR closed, rebase не нужен\` (ретро 15.08 t_16325ddd)."
        _title_prefix="⚠️ CI UNSTABLE: rebase"
    fi

    # Ретро 02.09 t_8e08b861: scan-all-prs спамил 19 rebase-карточек на PR #1857
    # за сутки, потому что UNSTABLE-ветка слепо шлёт «rebase и позеленеть» даже
    # когда PR уже на develop HEAD (behind=0). Здесь — guard «no-op rebase»:
    # если PR не отстаёт от develop И CI-провал объясняется develop-регрессией
    # (develop ⊆ PR по failed check-runs) — recovery-карточку НЕ создаём; вместо
    # неё один PR-комментарий (24h dedup) «CI красный из-за develop-side
    # регрессии, ждём фикс develop». Если develop чистый — одна карточка на
    # fix develop (assignee=devops), а не на rebase ветки.
    #
    # CONFLICTING-ветку guard НЕ трогает — там rebase реально нужен для
    # разрешения конфликта (это не stale-from-develop vs develop-regression).
    if [ "$mergeable" != "CONFLICTING" ] && [ "$merge_state" != "DIRTY" ]; then
        _pr_head_oid="${pr_head_oid:-}"
        # head SHA нужен для compare. Если пуст — достанем из PR.
        if [ -z "$_pr_head_oid" ]; then
            _pr_head_oid="$(gh pr view "$pr_num" --repo "$GH_REPO" --json headRefOid \
                --jq '.headRefOid' 2>/dev/null || echo "")"
        fi
        _behind="$(pr_behind_develop "${_pr_head_oid:-}")"
        if [ "$_behind" = "0" ]; then
            _dev_sha="$(gh api "repos/${GH_REPO}/commits/${DEVELOP_BRANCH}" --jq '.sha' 2>/dev/null || echo "")"
            _dev_failed="$(gh api "repos/${GH_REPO}/commits/${_dev_sha}/check-runs" \
                --jq '[.check_runs[]|select(.conclusion=="failure" or .conclusion=="timed_out" or .conclusion=="cancelled")|.name]|.[]' \
                2>/dev/null | sort -u || true)"
            _pr_failed="$(gh api "repos/${GH_REPO}/commits/${_pr_head_oid}/check-runs" \
                --jq '[.check_runs[]|select(.conclusion=="failure" or .conclusion=="timed_out" or .conclusion=="cancelled")|.name]|.[]' \
                2>/dev/null | sort -u || true)"
            _dev_only="$(comm -23 <(printf '%s\n' "$_dev_failed") <(printf '%s\n' "$_pr_failed") 2>/dev/null || true)"
            # Ретро 02.09 t_8e08b861 + ретро 31.08 t_e00f448d: pr_classify_failure
            # уже живёт в скрипте (стр. 1638) → здесь явно вызываем для лога и
            # маршрутизации unit_lint → diagnostic вместо rebase (если develop
            # НЕ виноват). Внутри scan-all-prs блока — было требование AC #1.
            _un_class="$(pr_classify_failure "${_pr_head_oid:-}")"
            if [ -n "$_dev_failed" ] && [ -z "$_dev_only" ]; then
                # develop-regression: develop ⊆ PR по failed checks.
                # Recovery-карточка бессильна → только PR-коммент с 24h dedup.
                _unstable_dedup_since="$(date -u -d '24 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
                _unstable_dup_count="$(gh api "repos/${GH_REPO}/issues/${pr_num}/comments?since=${_unstable_dedup_since}&per_page=100" \
                    --jq '[.[] | select(.body | startswith("⚠️ **develop-regression** (merge-gate"))] | length' 2>/dev/null || echo 0)"
                if [ "${_unstable_dup_count:-0}" -eq 0 ] 2>/dev/null; then
                    _dev_failed_csv="$(printf '%s' "$_dev_failed" | paste -sd, -)"
                    gh pr comment "$pr_num" --repo "$GH_REPO" --body \
                        "⚠️ **develop-regression** (merge-gate scan-all-prs, ретро 02.09 t_8e08b861): PR #${pr_num} (\`${head}\`) = MERGEABLE+UNSTABLE при behind=0 от develop. CI падает на \`${_dev_failed_csv}\` — те же чек-раны падают на develop HEAD (\`${_dev_sha:0:7}\`). **rebase не поможет** (PR уже на develop).

**ОБЯЗАН** (по процессу Шифу): дождаться фикса develop. Не делай rebase в ветке \`${head}\` — это бессильный no-op (merge-base == develop tip). Когда develop позеленеет, scan-all-prs автоматически поставит needs-e2e.

Шифу/воркер devops: чинить develop — коммит в develop напрямую или через отдельную карточку (assignee=devops, НЕ rebase recovery)." >/dev/null 2>&1 \
                        && log "scan-all-prs: PR #${pr_num} develop-regression comment posted (behind=0, dev_failed=${_dev_failed_csv})" \
                        || log "scan-all-prs: WARNING PR comment failed for develop-regression #${pr_num}"
                else
                    log "scan-all-prs: PR #${pr_num} develop-regression comment dedup'd (×${_unstable_dup_count} in 24h) — skip"
                fi
                log "scan-all-prs: PR #${pr_num} UNSTABLE+behind=0+develop-regression (class=${_un_class}) — rebase-карточка НЕ создастся"
                continue
            elif [ -z "$_dev_failed" ] && [ -n "$_pr_failed" ]; then
                # PR красный, develop чистый → вина PR, но rebase всё равно
                # no-op (behind=0). Старая логика всё равно создаст rebase-
                # карточку (PR-side fix через rebase develop-HEAD невозможен —
                # но воркер хотя бы увидит PR-failed checks). Оставляем её,
                # только подавляем спам по circuit breaker.
                _rebase_done_24h="$(count_rebase_cards_24h "$pr_num")"
                if [ "${_rebase_done_24h:-0}" -ge 3 ] 2>/dev/null; then
                    log "scan-all-prs: PR #${pr_num} UNSTABLE+behind=0+PR-side+circuit-break(×${_rebase_done_24h} done/24h) — rebase-loop detected, реbase-карточка НЕ создастся (нужен человек)"
                    # Один PR-комментарий-эскалация с 24h dedup, не спам в карточки.
                    _loop_dedup_since="$(date -u -d '24 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
                    _loop_dup_count="$(gh api "repos/${GH_REPO}/issues/${pr_num}/comments?since=${_loop_dedup_since}&per_page=100" \
                        --jq '[.[] | select(.body | startswith("🚨 **rebase-loop** (merge-gate"))] | length' 2>/dev/null || echo 0)"
                    if [ "${_loop_dup_count:-0}" -eq 0 ] 2>/dev/null; then
                        gh pr comment "$pr_num" --repo "$GH_REPO" --body \
                            "🚨 **rebase-loop** (merge-gate scan-all-prs, ретро 02.09 t_8e08b861): PR #${pr_num} красный при behind=0 от develop, develop чистый → вина PR. Уже ${_rebase_done_24h} rebase-карточек в done за 24ч, rebase бессилен (merge-base == develop tip). Шифу/воркер: чинить код в ветке \`${head}\` (lint/unit), а не rebase'ить." >/dev/null 2>&1 \
                            && log "scan-all-prs: PR #${pr_num} rebase-loop escalation posted (×${_rebase_done_24h} done/24h)" \
                            || log "scan-all-prs: WARNING PR comment failed for rebase-loop #${pr_num}"
                    else
                        log "scan-all-prs: PR #${pr_num} rebase-loop comment dedup'd (×${_loop_dup_count} in 24h)"
                    fi
                    continue
                fi
                log "scan-all-prs: PR #${pr_num} UNSTABLE+behind=0+PR-side — rebase-карточка будет создана (×${_rebase_done_24h} done/24h, порог=3)"
            else
                # Не смогли классифицировать (flake/gh API) → fail-open: старая
                # логика rebase-карточки сработает.
                log "scan-all-prs: PR #${pr_num} UNSTABLE+behind=0 but classify flake (dev_failed=${_dev_failed:-?}, pr_failed=${_pr_failed:-?}) — fail-open"
            fi
        fi
    fi

    if [ -n "$task_id" ]; then
        # Ретро 12.08 t_8af6bf29: rate-limit конфликт/UNSTABLE-комментариев
        # (1 раз в 2ч), а не каждый тик (~10 мин шум при вечном CONFLICTING).
        _marker="merge conflict detected"
        [ "$mergeable" != "CONFLICTING" ] && [ "$merge_state" != "DIRTY" ] && _marker="CI UNSTABLE detected"
        _last_ts="$(kanban_last_reminder_ts "$task_id" "$_marker")"
        _now_ts="$(date +%s)"
        _skip_comment=0
        if [ -n "$_last_ts" ] && [ $(( _now_ts - _last_ts )) -lt 7200 ]; then
            _skip_comment=1
            log "scan-all-prs: reminder rate-limited for ${task_id} (last=${_last_ts}, PR #${pr_num})"
        fi
        if [ "$_skip_comment" -eq 0 ]; then
            hermes kanban --board "$KANBAN_BOARD" comment "$task_id" "$_reminder" >/dev/null 2>&1 \
                || log "scan-all-prs: WARNING appending reminder to ${task_id} failed"
            log "scan-all-prs: reminder appended to existing card ${task_id} for PR #${pr_num}"
        fi
        # Процесс-фикс (12.08, ретро t_8af6bf29): РЕСПАВН-ГАРД ДЕДЛОК.
        # Раньше: reclaim done/archived/blocked карточки → hermes-agent dispatcher
        # check_respawn_guard блокирует респавн на 24ч по правилу active_pr (URL PR
        # в комментариях воркера; _RESPAWN_GUARD_PR_WINDOW=86400) → воркер не
        # стартует, rebase не делается, PR вечно CONFLICTING, merge-gate комментит
        # каждый тик. ТЕПЕРЬ: НЕ requeue'им старую карточку, а создаём СВЕЖУЮ
        # recovery-карточку (goal_mode=0, assignee=владелец PR, max_runtime 1800,
        # тело=rebase-чеклист, idempotency-key по PR-номеру — не плодить дубли,
        # урок t_bff6eccf). Свежая карточка не имеет URL-комментариев → guard не
        # блокирует. Создание идемпотентно: при существующей recovery-карточке
        # hermes kanban create вернёт её id, дубликат не создастся.
        # Ретро-фикс (13.08, t_42741511, кейс B #1172/#1173): idempotency-key
        # возвращает id карточки в ЛЮБОМ не-archived статусе — если recovery-
        # карточка УЖЕ done, create вернёт её id и свежая НЕ создастся, PR висит
        # CONFLICTING навсегда. Поэтому: ищем recovery-карточку по PR/branch в
        # ЛЮБОМ статусе (как e2e-process t_bff6eccf) и reclaim'им done/archived;
        # НО если на ветке уже есть АКТИВНАЯ карточка (running/ready/todo) —
        # не трогаем (гонка force-push, урок 13.08 t_ede84713/t_fb3796e2).
        _card_status="$(kanban_card_status "$task_id")"
        case "$_card_status" in
            running)
                log "scan-all-prs: card ${task_id} running — reminder only (PR #${pr_num})"
                ;;
            *)
                # Все карточки, упоминающие эту ветку/PR (любой статус).
                _branch_matches="$(hermes kanban --board "$KANBAN_BOARD" list --json 2>/dev/null | python3 -c "
import json,sys
try:
    data = json.loads(sys.stdin.read())
except Exception:
    sys.exit(0)
for t in data:
    title = t.get('title','')
    if '${head}' in title or 'rebase PR #${pr_num}' in title:
        print(t['id'], t.get('status',''))
" 2>/dev/null)"
                # Приоритет: активная карточка > blocked > done/archived.
                _active_match="$(printf '%s\n' "$_branch_matches" | awk '$2 ~ /^(running|ready|todo)$/ {print $1" "$2; exit}')"
                _blocked_id="$(printf '%s\n' "$_branch_matches" | awk '$2 == "blocked" {print $1; exit}')"
                _done_match="$(printf '%s\n' "$_branch_matches" | awk '$2 == "done" || $2 == "archived" {print $1" "$2; exit}')"
                if [ -n "$_active_match" ]; then
                    log "scan-all-prs: recovery card already active (${_active_match}) for PR #${pr_num} — skip (гонка force-push, урок 13.08)"
                elif [ -n "$_blocked_id" ]; then
                    hermes kanban --board "$KANBAN_BOARD" unblock "$_blocked_id" --reason "🔀 свежий конфликт/CI — retry (ретро 13.08 t_42741511)" >/dev/null 2>&1 || true
                    log "scan-all-prs: recovery card ${_blocked_id} unblocked (was blocked) for PR #${pr_num}"
                elif [ -n "$_done_match" ]; then
                    _done_id="${_done_match%% *}"
                    # Ретро-фикс 13.08 #2: reclaim НЕ работает с done (только
                    # для running — «cannot reclaim (not running)»). done —
                    # терминальное состояние. PR снова конфликтный → создаём
                    # СВЕЖУЮ ready-карточку (воркер отработал, нужен новый).
                    hermes kanban --board "$KANBAN_BOARD" create \
                        --assignee "$_assignee" \
                        --max-runtime 1800 \
                        --body "$_reminder" \
                        "🔀 rebase PR #${pr_num} (\`${head}\`) на develop — конфликт/CI (повтор)" >/dev/null 2>&1 \
                        && log "scan-all-prs: fresh recovery card created (old ${_done_id} was done) for PR #${pr_num}" \
                        || log "scan-all-prs: WARNING fresh recovery card create failed for PR #${pr_num}"
                else
                    _rec_key="merge-conflict-recovery-pr-${pr_num}"
                    _rec_title="🔀 rebase PR #${pr_num} (\`${head}\`) на develop — конфликт/CI"
                    # Ретро-фикс 13.08 (requeue→reclaim, requeue не существует в
                    # kanban CLI): idempotency-key возвращает существующую
                    # ДАЖЕ done карточку (SELECT ... status != 'archived'), из-за
                    # чего recovery-карточка после done НЕ пере-создавалась и PR
                    # висел CONFLICTING навсегда. Старые карточки выше уже
                    # обработаны (active/blocked/done match) — до else доходим
                    # только когда карточки НЕТ вообще. Поэтому create БЕЗ
                    # idempotency-key: каждая свежая конфликтная ситуация
                    # получает СВЕЖУЮ ready-карточку. Гонка (два merge-gate
                    # тика подряд) → дубликат, но дубликат безопаснее deadlock.
                    _rec_key="merge-conflict-recovery-pr-${pr_num}-$(date +%s)"
                    hermes kanban --board "$KANBAN_BOARD" create \
                        --assignee "$_assignee" \
                        --max-runtime 1800 \
                        --body "$_reminder" \
                        "$_rec_title" >/dev/null 2>&1 \
                        && log "scan-all-prs: recovery card created fresh for PR #${pr_num} (assignee=${_assignee})" \
                        || log "scan-all-prs: WARNING recovery card create failed (PR #${pr_num})"
                fi
                ;;
        esac
    else
        # Нет существующей карточки — раньше только логировали «main cycle
        # will pick up if needs-e2e» — НО для CONFLICTING/DIRTY PR основной
        # цикл needs-e2e НИКОГДА не ставит (нужен merge_state=clean), и
        # конфликт-карточка e2e-process тоже не создаётся (PR не в очереди) →
        # СЕРАЯ ЗОНА (ретро 12.08 t_618208c0, кейс PR #1165/issue #1160).
        # Теперь: коммент на PR + создание конфликт-карточки (идемпотентно,
        # idempotency-key по PR, assignee по метке issue) — как в e2e-process
        # round-merge (t_bff6eccf), но на этапе merge-gate.
        if [ "$mergeable" = "CONFLICTING" ] || [ "$merge_state" = "DIRTY" ]; then
            log "scan-all-prs: PR #${pr_num} ${mergeable}/${merge_state} без карточки — коммент на PR + конфликт-карточка (ретро t_618208c0)"
            # Дедуп PR-комментария (24h) — не спамим каждый тик.
            _prc_dedup_since="$(date -u -d '24 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
            _prc_dup_count="$(gh api "repos/${GH_REPO}/issues/${pr_num}/comments?since=${_prc_dedup_since}&per_page=100" \
                --jq '[.[] | select(.body | startswith("🔀 **merge conflict** (merge-gate"))] | length' 2>/dev/null || echo 0)"
            if [ "${_prc_dup_count:-0}" -eq 0 ] 2>/dev/null; then
                gh pr comment "$pr_num" --repo "$GH_REPO" --body \
                    "🔀 **merge conflict** (merge-gate, ретро 12.08 t_618208c0): PR #${pr_num} (\`${head}\`) → develop = **CONFLICTING** (mergeStateStatus=${merge_state:-?}).

**ОБЯЗАН** (по процессу Шифу 10.08): в **той же ветке** \`${head}\` сделай rebase на origin/develop:

\`\`\`bash
git fetch origin develop
git checkout ${head}
git rebase origin/develop
# resolve conflicts
git add -A && git rebase --continue
git push --force-with-lease origin ${head}
\`\`\`

Как PR станет MERGEABLE — merge-gate поставит needs-e2e автоматически. Метки снимать НЕ надо." >/dev/null 2>&1 \
                    && log "scan-all-prs: PR comment posted to #${pr_num} (merge conflict)" \
                    || log "scan-all-prs: WARNING PR comment failed for #${pr_num}"
            else
                log "scan-all-prs: PR comment dedup'd for #${pr_num} (×${_prc_dup_count} in 24h)"
            fi
            # Конфликт-карточка: ищем по branch в title в ЛЮБОМ статусе
            # (идемпотентно, урок t_bff6eccf), reclaim если done/archived,
            # unblock если blocked, create если нет.
            _existing_conflict="$(hermes kanban --board "$KANBAN_BOARD" list --json 2>/dev/null | python3 -c "
import json,sys
data = json.loads(sys.stdin.read())
for t in data:
    if t.get('title','').startswith('🔀 rebase PR #${pr_num}') or ('${head}' in t.get('title','') and 'rebase' in t.get('title','')):
        print(t['id'], t.get('status',''))
        break
" 2>/dev/null | head -1)"
            _conflict_id="${_existing_conflict%% *}"
            _conflict_status="${_existing_conflict#* }"
            if [ -n "$_conflict_id" ]; then
                case "$_conflict_status" in
                    done|archived)
                        # Ретро-фикс 13.08 #2: reclaim не работает с done —
                        # создаём СВЕЖУЮ ready-карточку.
                        hermes kanban --board "$KANBAN_BOARD" create \
                            --assignee "$_assignee" \
                            --max-runtime 1800 \
                            --body "🔀 свежий конфликт: PR #${pr_num} снова не мержится с develop (старая карточка ${_conflict_id} была ${_conflict_status}). Rebase на develop в той же ветке, CI green." \
                            "🔀 rebase PR #${pr_num} (\`${head}\`) на develop — конфликт/CI (повтор)" >/dev/null 2>&1 \
                            && log "scan-all-prs: fresh conflict card created (old ${_conflict_id} was ${_conflict_status}) for PR #${pr_num}" \
                            || log "scan-all-prs: WARNING fresh conflict card create failed for PR #${pr_num}"
                        ;;
                    blocked)
                        hermes kanban --board "$KANBAN_BOARD" unblock "$_conflict_id" --reason "🔀 свежий конфликт — retry (ретро 12.08 t_618208c0)" >/dev/null 2>&1 || true
                        log "scan-all-prs: conflict card ${_conflict_id} unblocked (was blocked)"
                        ;;
                    *)
                        log "scan-all-prs: conflict card ${_conflict_id} already active (${_conflict_status}) — skip"
                        ;;
                esac
            else
                hermes kanban --board "$KANBAN_BOARD" create \
                    --assignee "$_assignee" \
                    --priority 90 \
                    --max-runtime 1800 \
                    --body "$_reminder" \
                    "🔀 rebase PR #${pr_num} (\`${head}\`) на develop — конфликт (issue ${issue_num:-?})" >/dev/null 2>&1 \
                    || log "scan-all-prs: WARNING conflict card create failed (PR #${pr_num}, assignee=${_assignee})"
                log "scan-all-prs: conflict card created for PR #${pr_num} (assignee=${_assignee})"
            fi
        else
            log "scan-all-prs: no existing card for PR #${pr_num} (${head}); assignee=${_assignee}, issue=${issue_num:-?} — UNSTABLE, main cycle will pick up if needs-e2e"
        fi
    fi
done

# ============================================================================
# Clean-PR sweep (ретро 13.08 t_a3f170fe, надзор #1194/#1197/#1198)
# ----------------------------------------------------------------------------
# ПРОБЛЕМА: CLEAN/MERGEABLE OPEN PR без process-меток выпадали из ВСЕХ путей:
#   - основной цикл: только OPEN issues с меткой `hermes` + КАНОНИЧЕСКАЯ ветка
#     z-{agent}/<id>-<slug>. Невидимы: issue CLOSED (#1194/#1198), НЕканоническая
#     ветка (#1197 z-{agent}/968-tool-..., #1201 z-{agent}/968-triage-...),
#     ретро-ветки без issue (#1202 z-devops/t_...).
#   - scan-all-prs выше: только UNSTABLE/DIRTY/CONFLICTING — CLEAN не трогает.
#   - retro-path ниже: только MERGED PR.
#   Итог: PR висит в лимбо с 0 меток, очередь ревью товарища Шифу пуста.
# РЕШЕНИЕ: сканируем ВСЕ open PR (base=develop, не draft, CLEAN/MERGEABLE, без
# process-меток) и классифицируем как основной цикл (ретро 10.08 #2):
#   - lint / CI-only (все файлы .github/, scripts/agent_flow/, docs/) →
#     needs-review на PR напрямую (e2e не нужен);
#   - functional + OPEN issue → needs-e2e на issue (стандартный путь);
#   - functional + issue CLOSED/нет → needs-review на PR (e2e невозможен;
#     НЕ close автоматически — Шифу решает, урок 13.08 t_42741511).
# Идемпотентно: PR с process-меткой пропускаем; add-label — no-op при повторе.
# ============================================================================
clean_labeled=0
log "clean-pr-sweep: scanning CLEAN/MERGEABLE OPEN PRs without process labels"
_clean_prs_json="$(gh pr list --repo "$GH_REPO" --state open --base "$DEVELOP_BRANCH" \
    --json number,title,headRefName,mergeStateStatus,isDraft,labels,files 2>/dev/null || echo '[]')"
if [ -z "$_clean_prs_json" ]; then
    _clean_prs_json='[]'
fi
# ВАЖНО: process substitution (а не pipe), чтобы clean_labeled накапливался
# в текущем shell и попал в summary (как retro-path, ретро 13.08).
while IFS=$'\t' read -r c_pr c_head c_issue c_title c_labels c_files; do
    [ -z "$c_pr" ] && continue
    [ "$c_issue" = "-" ] && c_issue=""
    # Ретро 14.08 t_28afb585: PR с head-ветки, уже влитой через ДРУГОЙ PR,
    # не должен получать needs-review/needs-e2e от clean-pr-sweep — это
    # переиспользование ветки влитого PR (#1238). Его обрабатывает
    # stale_branch_scan_all (блок-коммент + снятие needs-review). Без этого
    # clean-pr-sweep классифицировал такой PR как functional+no-issue →
    # ставил needs-review БЕЗ e2e (именно так #1238 получил needs-review
    # в 12:06Z при последнем e2e-раунде в 10:35Z).
    _c_prev_merged="$(gh pr list --repo "$GH_REPO" --state merged --head "$c_head" \
        --json number --jq '.[0].number // ""' 2>/dev/null || true)"
    # Ретро 15.08 t_6024f414: proposal-ветки архитектора (z-architect/* без
    # issue-номера и без t_<card>) живут после merge — это НЕ stale-branch
    # reuse, а легальный proposal-цикл (#1247 → #1254 → #1255). Такой PR
    # НЕ скипаем: идём в обычную классификацию (functional + нет issue →
    # needs-review на PR), чтобы товарищ Шифу увидел его в очереди ревью.
    if is_proposal_branch "$c_head"; then
        log "clean-pr-sweep: PR #${c_pr} head ${c_head} — proposal-ветка архитектора (ретро 15.08 t_6024f414) — НЕ скипаю, классифицирую normally"
    else
    if [ -n "$_c_prev_merged" ] && [ "$_c_prev_merged" != "$c_pr" ]; then
        log "clean-pr-sweep: PR #${c_pr} head ${c_head} уже влит через PR #${_c_prev_merged} — skip (stale_branch_scan_all обработает, ретро 14.08 t_28afb585)"
        skipped=$((skipped+1)); continue
    fi
    fi
    # CI-only? (как ретро-путь t_061d466e): все файлы в .github/, scripts/agent_flow/, docs/
    _ci_only="$(printf '%s' "$c_files" | python3 -c '
import sys
files = [f for f in sys.stdin.read().split(",") if f]
ok = bool(files) and all(
    f.startswith(".github/") or f.startswith("scripts/agent_flow/") or f.startswith("docs/")
    for f in files
)
print("1" if ok else "0")
' 2>/dev/null || echo 0)"
    _c_kind="$(detect_pr_kind "$(printf '%s' "$c_labels" | tr '[:upper:]' '[:lower:]')" "$c_title")"
    if [ "$_ci_only" = "1" ]; then
        _c_kind="lint"
        log "clean-pr-sweep: PR #${c_pr} CI-only (files: ${c_files}) → lint"
    fi
    log "clean-pr-sweep: PR #${c_pr} (${c_head}) kind=${_c_kind} issue=${c_issue:-?}"

    if [ "$_c_kind" = "lint" ]; then
        # --- user-unlabel guard (ретро 18.08 t_de6bea69, PR #1398) -----------
        if user_removed_label_recently "$c_pr" "$NEEDS_REVIEW_LABEL"; then
            user_unlabel_log_skip "$c_pr" "$NEEDS_REVIEW_LABEL" "clean-pr-sweep lint"
            clean_labeled=$((clean_labeled+1))
            if [ "$DRY_RUN" != "true" ] && user_unlabel_should_notify "$c_pr" "$NEEDS_REVIEW_LABEL"; then
                gh pr comment "$c_pr" --repo "$GH_REPO" --body \
                    "agent-flow: ⏸️ clean-pr-sweep не восстановил \`needs-review\` — ты её ранее снял руками; жду твоего решения (ретро 18.08 t_de6bea69, Q22)." >/dev/null 2>&1 || true
                user_unlabel_mark_notified "$c_pr" "$NEEDS_REVIEW_LABEL" "clean-pr-sweep lint" || true
            fi
            continue
        fi
        log "clean-pr-sweep: PR #${c_pr} lint/CI-only → ${NEEDS_REVIEW_LABEL} (skip e2e)"
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would run: gh pr edit ${c_pr} --repo ${GH_REPO} --add-label ${NEEDS_REVIEW_LABEL}"
            clean_labeled=$((clean_labeled+1)); continue
        fi
        if gh pr edit "$c_pr" --repo "$GH_REPO" --add-label "$NEEDS_REVIEW_LABEL" >/dev/null 2>&1; then
            clean_labeled=$((clean_labeled+1))
            log "clean-pr-sweep: PR #${c_pr} → ${NEEDS_REVIEW_LABEL}"
        else
            log "clean-pr-sweep: WARNING add ${NEEDS_REVIEW_LABEL} to PR #${c_pr} failed"
        fi
        continue
    fi

    # functional: нужен e2e. Если issue OPEN → needs-e2e; иначе (CLOSED/нет)
    # e2e невозможен → needs-review (Шифу решит; НЕ close автоматически).
    _c_state=""
    if [ -n "$c_issue" ]; then
        _c_state="$(gh issue view "$c_issue" --repo "$GH_REPO" --json state --jq '.state' 2>/dev/null || echo '')"
    fi
    if [ "$_c_state" = "OPEN" ]; then
        # Взаимоисключение needs-review/needs-e2e (ретро 13.08 t_de63be1f, #942):
        # если issue под ревью юзера — не ставим needs-e2e.
        _c_issue_labels="$(gh issue view "$c_issue" --repo "$GH_REPO" --json labels \
            --jq '[.labels[].name] | join(",")' 2>/dev/null || echo '')"
        if has_label "$(printf '%s' "$_c_issue_labels" | tr '[:upper:]' '[:lower:]')" "$NEEDS_REVIEW_LABEL"; then
            log "clean-pr-sweep: PR #${c_pr} functional, issue #${c_issue} под ревью — skip (взаимоисключение)"
            skipped=$((skipped+1)); continue
        fi
        log "clean-pr-sweep: PR #${c_pr} functional, issue #${c_issue} OPEN → ${NEEDS_E2E_LABEL}"
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would run: gh issue edit ${c_issue} --repo ${GH_REPO} --add-label ${NEEDS_E2E_LABEL}"
            clean_labeled=$((clean_labeled+1)); continue
        fi
        # issue #1534: self-id whoami BEFORE clean-pr-sweep label flip.
        whoami_add_label "$c_issue" "${NEEDS_E2E_LABEL}" "clean-pr-sweep: PR #${c_pr} functional + issue #${c_issue} OPEN, returning to e2e queue" "pr=${c_pr}"
        if gh issue edit "$c_issue" --repo "$GH_REPO" --add-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1; then
            gh pr edit "$c_pr" --repo "$GH_REPO" --add-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
            clean_labeled=$((clean_labeled+1))
            log "clean-pr-sweep: issue #${c_issue} + PR #${c_pr} → ${NEEDS_E2E_LABEL}"
        else
            log "clean-pr-sweep: WARNING add ${NEEDS_E2E_LABEL} to issue #${c_issue} failed"
        fi
        continue
    fi
    log "clean-pr-sweep: PR #${c_pr} functional, issue ${c_issue:-?} state=${_c_state:-?} — e2e невозможен → ${NEEDS_REVIEW_LABEL}"
    if [ "$DRY_RUN" = "true" ]; then
        log "DRY-RUN would run: gh pr edit ${c_pr} --repo ${GH_REPO} --add-label ${NEEDS_REVIEW_LABEL}"
        clean_labeled=$((clean_labeled+1)); continue
    fi
    if gh pr edit "$c_pr" --repo "$GH_REPO" --add-label "$NEEDS_REVIEW_LABEL" >/dev/null 2>&1; then
        # issue #1553: self-id whoami AFTER успешного add-label needs-review
        # (clean-pr-sweep путь: e2e невозможен, Шифу ревьюит).
        post_whoami_comment pr "$c_pr" "adding-label:${NEEDS_REVIEW_LABEL}" \
            "clean-pr-sweep: PR #${c_pr} functional, e2e невозможен → needs-review"
        clean_labeled=$((clean_labeled+1))
        log "clean-pr-sweep: PR #${c_pr} → ${NEEDS_REVIEW_LABEL} (e2e невозможен)"
    else
        log "clean-pr-sweep: WARNING add ${NEEDS_REVIEW_LABEL} to PR #${c_pr} failed"
    fi
done < <(printf '%s' "$_clean_prs_json" | python3 -c '
import json, sys, re
data = json.load(sys.stdin)
PROCESS = {"needs-e2e", "needs-review", "e2e-done", "e2e:rejected", "no-e2e-required"}
for pr in data:
    if pr.get("isDraft"):
        continue
    if pr.get("mergeStateStatus") not in ("CLEAN", "MERGEABLE"):
        continue
    labels = {l.get("name", "") for l in (pr.get("labels") or [])}
    if PROCESS & labels:
        continue
    pr_num = str(pr.get("number", ""))
    head = pr.get("headRefName") or ""
    title = pr.get("title") or ""
    files = ",".join(f.get("path", "") for f in (pr.get("files") or []))
    labels_csv = ",".join(sorted(labels))
    m = re.search(r"#(\d+)", title)
    issue_num = m.group(1) if m else ""
    if not issue_num:
        m2 = re.search(r"z-\{agent\}/(\d+)-", head)
        issue_num = m2.group(1) if m2 else ""
    if issue_num.startswith("t_") or len(issue_num) > 7:
        issue_num = ""
    issue_num_out = issue_num if issue_num else "-"
    print(f"{pr_num}\t{head}\t{issue_num_out}\t{title}\t{labels_csv}\t{files}")
' 2>/dev/null)

# ============================================================================
# PR-side needs-e2e orphan reconcile (ретро 15.08 t_5cf0162b, надзор PR #1263)
# ----------------------------------------------------------------------------
# ПРОБЛЕМА: merge-gate (осн. цикл + clean-pr-sweep) ставит needs-e2e на PR
# best-effort propagation, НО обратного хода нет:
#   - e2e-process строит ротацию ТОЛЬКО по issues с needs-e2e (gh issue list
#     --label needs-e2e): если связанная issue уже обработана (e2e-done) и
#     CLOSED (#1246: needs-e2e 23:01 → e2e-done 23:21 → CLOSED 00:52), а на
#     PR осталась needs-e2e (23:01, propagation) — PR НИКОГДА не попадёт в
#     round, фикс застревает, Шифу не видит PR в очереди ревью;
#   - clean-pr-sweep сканирует ТОЛЬКО PR БЕЗ process-меток → PR с needs-e2e
#     пропущен (C4-идемпотентность);
#   - post-round sweep (e2e-process) лейблит только ISSUES, не PR.
# РЕШЕНИЕ: для OPEN PR с needs-e2e, у которых НЕТ связанного OPEN issue
# с needs-e2e (по body/title #N и ветке z-{agent}/<n>-<slug>):
#   - CI-only (все файлы .github/, docs/, scripts/agent_flow/ docs/ADR-only)
#     → needs-review на PR + снять needs-e2e (e2e не нужен, как clean-pr-sweep
#     _ci_only); см. также pr_has_functional_files() (process scripts и
#     robot code — функциональные, ретро 31.08 t_04371252);
#   - functional + есть OPEN issue (без needs-e2e) → вернуть needs-e2e на
#     issue (e2e-process возьмёт её в ротацию) + коммент на PR;
#   - functional + issue CLOSED/нет → needs-review на PR + снять needs-e2e
#     (e2e невозможен; Шифу решает; НЕ close автоматически, урок 13.08
#     t_42741511).
# Идемпотентно: PR с needs-e2e + живая OPEN issue с needs-e2e пропускаем;
# add/remove-label — no-op при повторе.
# ============================================================================
orphan_labeled=0
log "pr-orphan-reconcile: scanning OPEN PRs with ${NEEDS_E2E_LABEL} for orphan (no live issue)"
_orphan_prs_json="$(gh pr list --repo "$GH_REPO" --state open --base "$DEVELOP_BRANCH" \
    --json number,title,headRefName,body,files,mergeStateStatus,isDraft,labels 2>/dev/null || echo '[]')"
if [ -z "$_orphan_prs_json" ]; then
    _orphan_prs_json='[]'
fi
while IFS=$'\t' read -r o_pr o_head o_title o_body o_files o_issues_csv o_merge_state; do
    [ -z "$o_pr" ] && continue
    [ "$o_issues_csv" = "-" ] && o_issues_csv=""
    [ "$o_body" = "-" ] && o_body=""
    [ "$o_files" = "-" ] && o_files=""
    log "pr-orphan-reconcile: PR #${o_pr} (${o_head}) needs-e2e, state=${o_merge_state:-?}, issues=[${o_issues_csv:-none}]"

    # Пропускаем НЕ-clean PR (UNSTABLE/DIRTY/CONFLICTING обрабатывает
    # scan-all-prs: карточка + reminder; needs-review туда ставить рано).
    if [ "$o_merge_state" != "CLEAN" ] && [ "$o_merge_state" != "MERGEABLE" ]; then
        log "pr-orphan-reconcile: PR #${o_pr} state=${o_merge_state:-?} — skip (scan-all-prs зона)"
        skipped=$((skipped+1)); continue
    fi
    # Stale-branch guard (ретро 14.08 t_28afb585): head уже влита через
    # ДРУГОЙ merged PR — переиспользование ветки, обрабатывает
    # stale_branch_scan_all (блок-коммент), не мы.
    _o_prev_merged="$(gh pr list --repo "$GH_REPO" --state merged --head "$o_head" \
        --json number --jq '.[0].number // ""' 2>/dev/null || true)"
    if [ -n "$_o_prev_merged" ] && [ "$_o_prev_merged" != "$o_pr" ]; then
        log "pr-orphan-reconcile: PR #${o_pr} head ${o_head} уже влит через PR #${_o_prev_merged} — skip (stale_branch_scan_all)"
        skipped=$((skipped+1)); continue
    fi

    # Живая связанная issue? Если хоть одна OPEN + needs-e2e → не сирота.
    _o_live=0
    if [ -n "$o_issues_csv" ]; then
        for _oi in $(printf '%s' "$o_issues_csv" | tr ',' ' '); do
            _oi_state="$(gh issue view "$_oi" --repo "$GH_REPO" --json state --jq '.state' 2>/dev/null || echo '')"
            _oi_labels="$(gh issue view "$_oi" --repo "$GH_REPO" --json labels --jq '[.labels[].name] | join(",")' 2>/dev/null || echo '')"
            if [ "$_oi_state" = "OPEN" ] && has_label "$(printf '%s' "$_oi_labels" | tr '[:upper:]' '[:lower:]')" "$NEEDS_E2E_LABEL"; then
                _o_live=1
                log "pr-orphan-reconcile: PR #${o_pr} связан с OPEN issue #${_oi} (needs-e2e) — живой цикл, skip"
                break
            fi
        done
    fi
    if [ "$_o_live" = "1" ]; then
        skipped=$((skipped+1)); continue
    fi

    # Сирота. Классификация как clean-pr-sweep: CI-only vs functional.
    _o_ci_only="$(printf '%s' "$o_files" | python3 -c '
import sys
files = [f for f in sys.stdin.read().split(",") if f]
ok = bool(files) and all(
    f.startswith(".github/") or f.startswith("scripts/agent_flow/") or f.startswith("docs/")
    for f in files
)
print("1" if ok else "0")
' 2>/dev/null || echo 0)"
    if [ "$_o_ci_only" = "1" ]; then
        log "pr-orphan-reconcile: PR #${o_pr} CI-only (${o_files}) — ${NEEDS_REVIEW_LABEL}, e2e не нужен"
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would: gh pr edit ${o_pr} --remove-label ${NEEDS_E2E_LABEL} --add-label ${NEEDS_REVIEW_LABEL}"
            orphan_labeled=$((orphan_labeled+1)); continue
        fi
        gh pr edit "$o_pr" --repo "$GH_REPO" --remove-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
        gh pr edit "$o_pr" --repo "$GH_REPO" --add-label "$NEEDS_REVIEW_LABEL" >/dev/null 2>&1 || true
        orphan_labeled=$((orphan_labeled+1))
        continue
    fi

    # functional: ищем OPEN issue среди связанных (без needs-e2e — живой бы
    # уже поймали выше). Если есть — возвращаем needs-e2e на issue.
    _o_open_issue=""
    if [ -n "$o_issues_csv" ]; then
        for _oi in $(printf '%s' "$o_issues_csv" | tr ',' ' '); do
            _oi_state="$(gh issue view "$_oi" --repo "$GH_REPO" --json state --jq '.state' 2>/dev/null || echo '')"
            if [ "$_oi_state" = "OPEN" ]; then
                _o_open_issue="$_oi"; break
            fi
        done
    fi
    if [ -n "$_o_open_issue" ]; then
        log "pr-orphan-reconcile: PR #${o_pr} functional, issue #${_o_open_issue} OPEN — возвращаем ${NEEDS_E2E_LABEL} на issue"
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would: gh issue edit ${_o_open_issue} --add-label ${NEEDS_E2E_LABEL} + comment"
            orphan_labeled=$((orphan_labeled+1)); continue
        fi
        # issue #1534: self-id whoami BEFORE pr-orphan-reconcile label flip.
        whoami_add_label "$_o_open_issue" "${NEEDS_E2E_LABEL}" "pr-orphan-reconcile: PR #${o_pr} lost live issue, returning ${NEEDS_E2E_LABEL} to #${_o_open_issue} (retro 15.08 t_5cf0162b)" "pr=${o_pr}"
        gh issue edit "$_o_open_issue" --repo "$GH_REPO" --add-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
        gh pr comment "$o_pr" --repo "$GH_REPO" --body \
            "agent-flow: 🔄 PR-side ${NEEDS_E2E_LABEL} потерял живую issue (сирота, ретро 15.08 t_5cf0162b). Связанная issue #${_o_open_issue} OPEN — ${NEEDS_E2E_LABEL} возвращён на неё, e2e-process возьмёт в ротацию." >/dev/null 2>&1 || true
        orphan_labeled=$((orphan_labeled+1))
        continue
    fi

    # functional + issue CLOSED/нет → e2e невозможен → needs-review на PR.
    log "pr-orphan-reconcile: PR #${o_pr} functional, связанные issues закрыты/нет — ${NEEDS_REVIEW_LABEL} (e2e невозможен)"
    if [ "$DRY_RUN" = "true" ]; then
        log "DRY-RUN would: gh pr edit ${o_pr} --remove-label ${NEEDS_E2E_LABEL} --add-label ${NEEDS_REVIEW_LABEL} + comment"
        orphan_labeled=$((orphan_labeled+1)); continue
    fi
    gh pr edit "$o_pr" --repo "$GH_REPO" --remove-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
    gh pr edit "$o_pr" --repo "$GH_REPO" --add-label "$NEEDS_REVIEW_LABEL" >/dev/null 2>&1 || true
    # issue #1553: self-id whoami AFTER успешного add-label (orphan-reconcile,
    # functional + issue CLOSED → e2e невозможен).
    post_whoami_comment pr "$o_pr" "adding-label:${NEEDS_REVIEW_LABEL}" \
        "pr-orphan-reconcile: PR #${o_pr} functional + связанные issues закрыты → needs-review (e2e невозможен, retro 15.08 t_5cf0162b)"
    gh pr comment "$o_pr" --repo "$GH_REPO" --body \
        "agent-flow: 🔄 PR-side ${NEEDS_E2E_LABEL} потерял живую issue (сирота, ретро 15.08 t_5cf0162b): связанные issues закрыты/не найдены → e2e невозможен. Снят ${NEEDS_E2E_LABEL}, поставлен ${NEEDS_REVIEW_LABEL} — товарищ Шифу ревьюит напрямую." >/dev/null 2>&1 || true
    orphan_labeled=$((orphan_labeled+1))
done < <(printf '%s' "$_orphan_prs_json" | python3 -c '
import json, sys, re
data = json.load(sys.stdin)
PROCESS = {"needs-review", "e2e-done", "e2e:rejected", "no-e2e-required"}
for pr in data:
    if pr.get("isDraft"):
        continue
    labels = {l.get("name", "") for l in (pr.get("labels") or [])}
    if "needs-e2e" not in labels:
        continue
    if PROCESS & labels:
        continue
    pr_num = str(pr.get("number", ""))
    head = pr.get("headRefName") or ""
    title = pr.get("title") or ""
    body = pr.get("body") or ""
    files = ",".join(f.get("path", "") for f in (pr.get("files") or []))
    merge_state = pr.get("mergeStateStatus") or ""
    # Все issue-референсы: title #N, body #N, ветка z-{agent}/<n>-<slug>.
    issues = set()
    for m in re.finditer(r"#(\d+)", title + "\n" + body):
        n = m.group(1)
        if n != pr_num and not n.startswith("t_") and len(n) <= 7:
            issues.add(n)
    m2 = re.search(r"z-\{agent\}/(\d+)-", head)
    if m2:
        issues.add(m2.group(1))
    issues_csv = ",".join(sorted(issues, key=int)) if issues else "-"
    # Заполнители для пустых полей: bash `read` с IFS=$'\t' схлопывает
    # последовательные разделители → пустое поле (body/files) теряется и
    # следующие поля сдвигаются (ретро 15.08 t_5cf0162b, кейс O3).
    body_out = body if body else "-"
    files_out = files if files else "-"
    print(f"{pr_num}\t{head}\t{title}\t{body_out}\t{files_out}\t{issues_csv}\t{merge_state}")
' 2>/dev/null)

# ============================================================================
# Ретро-путь (12.08 t_68607832 + t_061d466e): merged PR → issue без меток /
# с e2e:rejected
# ----------------------------------------------------------------------------
# ПРОБЛЕМА 1 (t_68607832): issues #1138/#1139 были починены ретро-карточками
# (вне label-цикла needs-e2e→e2e-done) и НИКОГДА не получали e2e-done →
# merge-gate молчал → issue висела OPEN при смерженном фиксе. Основной цикл
# выше обрабатывает только issues с меткой `hermes`; ретро-issues меток не
# имеют вовсе.
#
# ПРОБЛЕМА 2 (t_061d466e, 12.08): issue #1041 — ОТКРЫТА с меткой e2e:rejected,
# хотя фикс ВЛИТ (PR #1161 merged, CI зелёный). Петля:
#   - основной цикл находит PR по канонической ветке
#     z-{agent}/1041-fix-l-build-vision-pi-docker-tag-local-g → это CLOSED
#     PR #1155 (закрыт, НЕ смержен); реальный merged PR #1161 имеет ДРУГУЮ
#     ветку (z-{agent}/1041-fix-l-build-dockertag-clean) → основной цикл его
#     не видит;
#   - e2e-process не может прогнать e2e для #1041: ветка конфликтует
#     с develop (см. t_bff6eccf), фикс уже в develop;
#   - ретро-путь НАХОДИЛ merged PR #1161 (title содержит #1041), но скипал
#     issue из-за e2e:rejected → никто не закрывал.
#
# РЕШЕНИЕ: сканируем недавно смерженные PR (base=develop), извлекаем номера
# issues из title/body, и для OPEN issues применяем ретро-путь:
#   - PASS-доказательство есть (e2e run SUCCESS на ветке PR, ИЛИ CI-only PR
#     с зелёным CI) → close issue с комментарием-доказательством; если на
#     issue стоит e2e:rejected — снимаем его ПЕРЕД close (фикс влит, e2e
#     больше не нужен / не может пройти);
#   - иначе (и нет e2e:rejected) → ставим needs-e2e (e2e-process возьмёт
#     issue в ротацию);
#   - e2e:rejected + merged PR, но PASS-доказательства НЕТ → НЕ ставим
#     needs-e2e (иначе e2e-process зациклится: rejected → needs-e2e → снова
#     rejected). Оставляем rejected и логируем — нужен ручной разбор.
#
# Guard от пере-закрытия: issues с hermes (кроме e2e:rejected — см. #1041)/
# needs-e2e/e2e-done/no-e2e-required пропускаем — их обрабатывают основные
# циклы. Дубликаты комментариев дедуплицируются как в post-merge cleanup
# (6h окно).
# ============================================================================
RETRO_MERGED_DAYS="${RETRO_MERGED_DAYS:-14}"
retro_closed=0
retro_labeled=0

log "retro-path: scanning merged PRs referencing unlabeled issues"

# Недавно смерженные PR (base=develop). Ограничиваем окно, чтобы не сканировать
# всю историю репозитория каждый тик.
_retro_since="$(date -u -d "${RETRO_MERGED_DAYS} days ago" +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
_retro_prs_json="$(gh pr list --repo "$GH_REPO" --state merged --base "$DEVELOP_BRANCH" \
    --limit 100 \
    --json number,title,body,headRefName,mergedAt 2>/dev/null || echo '[]')"
# Guard: пустой вывод gh (или сбой API) → валидный пустой список, иначе
# python3 json.load('') уронит pipeline под set -o pipefail.
if [ -z "$_retro_prs_json" ]; then
    _retro_prs_json='[]'
fi

# Извлекаем (issue, pr, head): номера issues, на которые ссылается PR в
# title/body (#N, closes #N, fixes #N). Скипаем PR, смерженные раньше окна,
# и self-reference (номер PR в своём же body, например "PR: #1142").
# ВАЖНО: process substitution (а не pipe), чтобы retro_closed/retro_labeled
# накапливались в текущем shell и попали в summary.
while IFS=$'\t' read -r r_issue r_pr r_head; do
    [ -z "$r_issue" ] && continue
    # Guard (ретро 13.08, надзор): извлечённый номер может оказаться ПРИН-номером,
    # а не issue — PR #1186 (сам фикс merge-gate) ссылался в title на #1172/#1173
    # (реальные кодовые PR), ретро-путь принял их за issues, нашёл e2e-PASS на их
    # ветках (e2e-done был на issue) и ЗАКРЫЛ живые PR через gh issue close
    # (issues и PR делят нумерацию): #1172 закрыт 02:33, #1173 закрыт 02:45 и
    # добит повторно 03:54 после reopen. Фикс #918/#979 НЕ попал в develop.
    # Проверяем: номер существует как PR (даже closed) → это не issue → skip.
    # ВАЖНО (ретро 13.08 t_2d78fbdd, #942): НЕЛЬЗЯ проверять через
    # `gh pr view N --json number` — gh CLI для одного поля number НЕ ходит
    # в API и возвращает success для ЛЮБОГО числа (даже несуществующего),
    # поэтому guard скипал ВСЕ ретро-issues как «это PR» и ретро-путь
    # молчал (issue #942 висела OPEN при смерженном PR #1192). Используем
    # REST `gh api pulls/N` — он реально проверяет существование PR:
    # 404 = это issue (не PR) → обрабатываем ретро-путь; 200 = это PR → skip.
    if gh api "repos/${GH_REPO}/pulls/${r_issue}" --jq '.number' >/dev/null 2>&1; then
        log "retro-path: #${r_issue} — это PR (не issue), ref из PR #${r_pr} — skip (guard t_d8e2c3f1)"
        continue
    fi
    log "retro-path: issue #${r_issue} referenced by merged PR #${r_pr} (${r_head})"

    # Перечитываем labels/state — race с e2e-process (как в основном цикле).
    _r_labels_csv="$(gh issue view "$r_issue" --repo "$GH_REPO" --json labels \
        --jq '[.labels[].name] | join(",")' 2>/dev/null || echo '')"
    _r_labels_norm="$(printf '%s' "$_r_labels_csv" | tr '[:upper:]' '[:lower:]')"
    # Ретро 12.08 t_061d466e: e2e:rejected больше НЕ скипает ретро-путь —
    # это ровно петля #1041 (фикс влит, e2e не может пройти). Вместо скипа
    # ниже проверяем PASS-доказательство: есть → снять rejected + close,
    # нет → оставить rejected (не ставить needs-e2e, чтобы не зациклить
    # e2e-process).
    _r_was_rejected=0
    if has_label "$_r_labels_norm" "$REJECTED_LABEL"; then
        _r_was_rejected=1
        log "retro-path: issue #${r_issue} имеет ${REJECTED_LABEL} — ищем PASS-доказательство (ретро t_061d466e)"
    elif has_label "$_r_labels_norm" "$DONE_LABEL" \
        || has_label "$_r_labels_norm" "$NO_E2E_LABEL" \
        || has_label "$_r_labels_norm" "$NEEDS_REVIEW_LABEL"; then
        # needs-review в skip-листе (ретро 13.08, надзор, #942): иначе ретро-путь
        # КАЖДЫЙ тик re-add'ил needs-e2e на issue под ревью юзера — взаимная
        # исключаемость needs-review/needs-e2e (как в основном цикле, стр. 366).
        # Цикл: ручная чистка → ретро-путь снова ставит needs-e2e → e2e-process
        # скипает (нет PR-ветки) → issue висит с двумя метками навсегда.
        log "retro-path: issue #${r_issue} уже в process-цикле (${_r_labels_norm}) — skip"
        continue
    elif has_label "$_r_labels_norm" "$NEEDS_E2E_LABEL"; then
        # Ретро 01.09 t_365de06c: needs-e2e БЕЗ ${ISSUE_LABEL} (= hermes) +
        # merged PR → orphan в process-цикле. main-cycle не видит (нет hermes),
        # e2e-process тоже не подберёт (нет живой PR-ветки: PR уже влит).
        # Раньше skip'ались здесь вместе с e2e-done/no-e2e/needs-review, и
        # issue висела OPEN вечно (issue #1824, наблюдение архитектора 01.09
        # ~07:10Z: PR #1843 влит, issue #1824 OPEN с одной needs-e2e, без
        # hermes; main-cycle skip'ает, retro-path skip'ает, e2e-process skip'ает).
        #
        # Решение: orphan-cleanup внутри retro-path:
        #   - СНЯТЬ needs-e2e (orphan больше не претендует на e2e-ротацию;
        #     следующий тик увидит issue уже без этой метки и не зациклится).
        #   - Проверить PASS-доказательство (тот же блок 4155-4196):
        #       PASS → close + comment «post-merge needs-e2e cleanup, retro-path»
        #              (reason = audit-строка для последующего разбора);
        #       no PASS → оставить issue как есть (без needs-e2e), audit-коммент
        #                 «merge без PASS — ручной разбор» (НЕ close, НЕ re-add
        #                 needs-e2e: иначе e2e-process возьмёт issue, у которой
        #                 нет живой PR-ветки, и поставит e2e:rejected — лишний шум).
        #
        # ВАЖНО (regression guard): НЕ цепляем hermes+needs-e2e — это territory
        # e2e-process (test_O_retro_hermes_with_needs_e2e_still_skips в
        # test_merge_gate_retro_path.sh). Условие явно проверяет
        # !has_label(hermes).
        if has_label "$_r_labels_norm" "$ISSUE_LABEL"; then
            # hermes+needs-e2e — НЕ наш случай, e2e-process обрабатывает.
            log "retro-path: issue #${r_issue} hermes+${NEEDS_E2E_LABEL} — e2e-process owns, skip"
            continue
        fi
        log "retro-path: issue #${r_issue} orphan needs-e2e (no ${ISSUE_LABEL}) — post-merge cleanup"
        # Снимаем needs-e2e (orphan-cleanup). Если не удалось — не критично,
        # close/audit-коммент всё равно выполнятся; следующий тик снова попытается.
        if [ "$DRY_RUN" != "true" ]; then
            gh issue edit "$r_issue" --repo "$GH_REPO" --remove-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 \
                && log "retro-path: issue #${r_issue} ${NEEDS_E2E_LABEL} снят (orphan-cleanup)" \
                || log "retro-path: WARNING не удалось снять ${NEEDS_E2E_LABEL} с #${r_issue}"
        fi
        # Метим факт orphan-cleanup — основной close-блок ниже (4155+) использует
        # этот флаг, чтобы выдать корректный audit-комментарий и логировать.
        _r_was_orphan=1
    fi
    # Ретро 19.08 t_498dc624 (process-fix-hermes-stuck-open): ${ISSUE_LABEL}
    # (= hermes) БЕЗ workflow-меток (needs-e2e/e2e-done/no-e2e-required/
    # needs-review) — process-fix issue. Основной цикл таких issues НЕ
    # закрывает (требует e2e-done, а process-fix PR не может его получить:
    # e2e-process скипает CI-only фикс), e2e-process тоже не ставит e2e-done
    # → петля. Ретро-путь должен иметь шанс закрыть через PASS-доказательство
    # (CI-only + green CI, или e2e run SUCCESS). ТОЛЬКО наличие hermes не
    # блокирует ретро-путь — иначе issues #1421/#1419/#1404/#1412 висели OPEN
    # при смерженных PR. Workflow-метки (выше) по-прежнему skip'ают —
    # взаимоисключаемость с основным циклом.
    _r_state="$(gh issue view "$r_issue" --repo "$GH_REPO" --json state \
        --jq '.state' 2>/dev/null || echo '')"
    if [ "$_r_state" != "OPEN" ]; then
        log "retro-path: issue #${r_issue} state=${_r_state} — skip"
        continue
    fi

    # --- PASS-доказательство ---
    _r_evidence=""
    # (a) e2e run SUCCESS на ветке PR (самое сильное доказательство)
    _r_e2e_ok="$(gh run list --repo "$GH_REPO" --branch "$r_head" \
        --workflow "L: E2E Voice Test" --limit 20 \
        --json conclusion --jq '[.[] | select(.conclusion == "success")] | length' 2>/dev/null || echo 0)"
    if [ "${_r_e2e_ok:-0}" -gt 0 ] 2>/dev/null; then
        _r_evidence="e2e run SUCCESS на ветке ${r_head}"
    else
        # (b) CI-only PR (все файлы в CI-контуре) + зелёный CI → e2e не нужен
        # Ретро 19.08 t_498dc624: расширяем CI-only на .hermes/plans/ —
        # процессные фиксы часто касаются roadmap/планов (например, PR #1414
        # правил .hermes/plans/process-fix-roadmap.md — раньше не считался
        # CI-only → process-fix issue #1404 висел OPEN при зелёном CI).
        _r_files="$(gh pr view "$r_pr" --repo "$GH_REPO" --json files \
            --jq '[.files[].path]' 2>/dev/null || echo '[]')"
        _r_ci_only="$(printf '%s' "$_r_files" | python3 -c '
import json, sys
try:
    files = json.load(sys.stdin)
    ok = bool(files) and all(
        f.startswith(".github/")
        or f.startswith("scripts/agent_flow/")
        or f.startswith("docs/")
        or f.startswith(".hermes/plans/")
        for f in files
    )
    print("1" if ok else "0")
except Exception:
    print("0")
' 2>/dev/null || echo 0)"
        if [ "$_r_ci_only" = "1" ]; then
            # ВНИМАНИЕ (ретро 12.08 t_061d466e): фильтр обязан разыменовывать
            # .statusCheckRollup[] — иначе jq применяется к объекту
            # {"statusCheckRollup":[...]} и падает «expected an object but got:
            # array» → rollup=1 → CI-only PASS не находится (петля #1041).
            _r_rollup="$(gh pr view "$r_pr" --repo "$GH_REPO" --json statusCheckRollup \
                --jq '[.statusCheckRollup[] | select(.conclusion == "FAILURE" or .conclusion == "CANCELLED" or .conclusion == "TIMED_OUT")] | length' 2>/dev/null || echo 1)"
            if [ "${_r_rollup:-1}" -eq 0 ] 2>/dev/null; then
                _r_evidence="CI-only PR #${r_pr} (.github/scripts/docs/.hermes-plans), CI зелёный — e2e не требуется"
            fi
        fi
    fi

    if [ -n "$_r_evidence" ]; then
        log "retro-path: issue #${r_issue} PASS-доказательство: ${_r_evidence} — closing"
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would close issue #${r_issue} (retro-path)"
            retro_closed=$((retro_closed+1)); continue
        fi
        # Ретро 12.08 t_061d466e: снять e2e:rejected ПЕРЕД close — фикс влит
        # (merged PR), e2e больше не нужен. Если снять не удалось — не
        # критично, close всё равно выполнится; следующий тик не найдёт
        # issue (она CLOSED).
        if [ "$_r_was_rejected" = "1" ]; then
            gh issue edit "$r_issue" --repo "$GH_REPO" --remove-label "$REJECTED_LABEL" >/dev/null 2>&1 \
                && log "retro-path: issue #${r_issue} ${REJECTED_LABEL} снят (PASS-доказательство)" \
                || log "retro-path: WARNING не удалось снять ${REJECTED_LABEL} с #${r_issue}"
        fi
        # Дедупликация комментария (6h) — не спамим каждый тик.
        _r_dedup_since="$(date -u -d '6 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
        _r_dup_count="$(gh api "repos/${GH_REPO}/issues/${r_issue}/comments?since=${_r_dedup_since}&per_page=100" \
            --jq '[.[] | select(.body | startswith("✅ ретро-путь"))] | length' 2>/dev/null || echo 0)"
        if [ "${_r_dup_count:-0}" -eq 0 ]; then
            _r_rejected_note=""
            if [ "$_r_was_rejected" = "1" ]; then
                _r_rejected_note=" Снят ${REJECTED_LABEL} (фикс влит, e2e не требуется)."
            fi
            # Ретро 01.09 t_365de06c: orphan-cleanup имеет собственный префикс
            # комментария для grep'a при разборе (отличается от «✅ ретро-путь»
            # чтобы dedup не считал обычный и orphan-cleanup одной веткой).
            if [ "${_r_was_orphan:-0}" = "1" ]; then
                gh issue comment "$r_issue" --repo "$GH_REPO" --body \
                    "🧹 ретро-путь (orphan-cleanup, t_365de06c): issue имела только ${NEEDS_E2E_LABEL} без ${ISSUE_LABEL} (= hermes) после merge PR #${r_pr} в ${DEVELOP_BRANCH}. Снят ${NEEDS_E2E_LABEL} (orphan вышел из e2e-ротации). PASS-доказательство: ${_r_evidence}. Issue закрыта." >/dev/null 2>&1 || true
            else
                gh issue comment "$r_issue" --repo "$GH_REPO" --body \
                    "✅ ретро-путь (ADR-0014 gap, t_68607832/t_061d466e): PR #${r_pr} смержен в ${DEVELOP_BRANCH}. PASS-доказательство: ${_r_evidence}.${_r_rejected_note} Issue закрыта." >/dev/null 2>&1 || true
            fi
        fi
        # issue #1534: self-id whoami BEFORE close (retro-path).
        whoami_close_issue "$r_issue" "retro-path close: PR #${r_pr} merged into ${DEVELOP_BRANCH} (ADR-0014 gap t_68607832/t_061d466e)"
        if gh issue close "$r_issue" --repo "$GH_REPO" --reason completed >/dev/null 2>&1; then
            retro_closed=$((retro_closed+1))
            log "retro-path: issue #${r_issue} CLOSED (reason=completed, ретро-путь)"
        else
            log "retro-path: WARNING close failed for #${r_issue} — retry next tick"
        fi
    else
        if [ "$_r_was_rejected" = "1" ]; then
            # e2e:rejected + merged PR, но PASS-доказательства нет: НЕ ставим
            # needs-e2e — иначе e2e-process снова возьмёт issue и снова
            # поставит rejected (петля). Оставляем rejected — нужен ручной
            # разбор (ретро 12.08 t_061d466e).
            log "retro-path: issue #${r_issue} имеет ${REJECTED_LABEL}, PASS-доказательства нет — НЕ трогаем (нужен ручной разбор)"
            skipped=$((skipped+1))
            continue
        fi
        if [ "${_r_was_orphan:-0}" = "1" ]; then
            # Ретро 01.09 t_365de06c: orphan-cleanup без PASS. issue уже без
            # ${NEEDS_E2E_LABEL} (сняли выше). НЕ ставим needs-e2e повторно
            # (иначе e2e-process возьмёт issue без живой PR-ветки → поставит
            # ${REJECTED_LABEL}, лишний шум). НЕ close'им (нет PASS). Audit-
            # коммент с маркером «нужен ручной разбор» — следующий тик его
            # не повторит (dedup 6h), а юзер/разбор увидит явный сигнал.
            log "retro-path: issue #${r_issue} orphan, PASS-доказательства нет — НЕ close, оставлен ручной разбор"
            _r_orphan_dedup_since="$(date -u -d '6 hours ago' +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || date -u +%Y-%m-%dT%H:%M:%SZ)"
            _r_orphan_dup_count="$(gh api "repos/${GH_REPO}/issues/${r_issue}/comments?since=${_r_orphan_dedup_since}&per_page=100" \
                --jq '[.[] | select(.body | contains("🧹 ретро-путь (orphan-cleanup, t_365de06c)"))] | length' 2>/dev/null || echo 0)"
            if [ "${_r_orphan_dup_count:-0}" -eq 0 ] && [ "$DRY_RUN" != "true" ]; then
                gh issue comment "$r_issue" --repo "$GH_REPO" --body \
                    "🧹 ретро-путь (orphan-cleanup, t_365de06c): issue имела только ${NEEDS_E2E_LABEL} без ${ISSUE_LABEL} (= hermes) после merge PR #${r_pr} в ${DEVELOP_BRANCH}. Снят ${NEEDS_E2E_LABEL} (orphan-cleanup). PASS-доказательства не найдено (нет e2e SUCCESS, PR не CI-only или CI не зелёный). Issue НЕ закрыта автоматически — нужен ручной разбор (verify фикса в роботе/на стенде и закрыть вручную)." >/dev/null 2>&1 || true
            fi
            skipped=$((skipped+1))
            continue
        fi
        log "retro-path: issue #${r_issue} без PASS-доказательства — ставим ${NEEDS_E2E_LABEL}"
        if [ "$DRY_RUN" != "true" ]; then
            gh issue edit "$r_issue" --repo "$GH_REPO" --add-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
            gh issue comment "$r_issue" --repo "$GH_REPO" --body \
                "agent-flow: 🔄 ретро-путь: PR #${r_pr} смержен, но PASS-доказательства не найдено. Поставлен ${NEEDS_E2E_LABEL} — e2e-process возьмёт issue в ротацию." >/dev/null 2>&1 || true
        fi
        retro_labeled=$((retro_labeled+1))
    fi
done < <(printf '%s' "$_retro_prs_json" | python3 -c '
import json, sys, re
data = json.load(sys.stdin)
since = sys.argv[1]
seen = set()
for pr in data:
    if (pr.get("mergedAt") or "") < since:
        continue
    pr_num = str(pr.get("number", ""))
    text = (pr.get("title") or "") + "\n" + (pr.get("body") or "")
    head = pr.get("headRefName") or ""
    for m in re.finditer(r"#(\d+)", text):
        issue = m.group(1)
        if issue == pr_num:
            continue
        key = (issue, pr_num)
        if key in seen:
            continue
        seen.add(key)
        print(issue + "\t" + pr_num + "\t" + head)
' "$_retro_since" 2>/dev/null)

# ============================================================================
# Ретро-архив done-карточек по MERGED PR (ретро 14.08 t_36c9ac4e)
# ----------------------------------------------------------------------------
# ПРОБЛЕМА: ретро/recovery-карточки (маркер retro-key, БЕЗ issue-линка) после
# merge PR остаются done НАВСЕГДА:
#   - archive-путь основного цикла (стр. 774-783) находит карточку ТОЛЬКО по
#     issue: #N в body (kanban list --json -> t.get("issue")), а поля "issue"
#     в выводе v0.20.0 НЕТ ВООБЩЕ → card_id пуст → archive не выполняется;
#   - scan-all-prs смотрит только OPEN PR (UNSTABLE/CONFLICTING);
#   - retro-path закрывает только ISSUES — карточки ретро/recovery не трогает.
# ФАКТЫ 13.08 22:20Z: t_fe266643 (#1221), t_da3e0bd5 (#1212), t_04d73108
# (#1216), t_2cae75c0 (#1215), t_2d78fbdd (#1211), t_7eab35a0 (#1210),
# t_0d2479ba (#1163), t_35ff29f1 (#1214), recovery t_e9e09694/t_de961c1b
# (PR #1212) — все done при MERGED PR, не archived.
# РЕШЕНИЕ: тот же скан смерженных PR (base=develop, окно RETRO_MERGED_DAYS),
# маппинг MERGED PR → task_id:
#   (a) task_id из head-ветки: t_<hex> (паттерн scan-all-prs стр. 1202-1204);
#   (b) карточки, чей title упоминает смерженный PR/branch (recovery-карточки
#       вида "🔀 rebase PR #N (`branch`)" — своих PR не имеют);
#   (c) карточки, чей СОБСТВЕННЫЙ branch_name (exact) совпадает с head-веткой
#       смерженного PR — issue-путь основного цикла не достаёт их, когда issue
#       УЖЕ CLOSED (t_41fec39e — issue #1217 закрыта Q22-путём, PR #1220
#       merged, ветка z-{agent}/1217-... без t_<hex>). ВАЖНО: exact-match,
#       НЕ substring по issue-номеру — иначе заархивируются done-карточки с
#       CLOSED-но-НЕ-merged PR (t_cc9cc56d: PR #1155 CLOSED, issue #1041 CLOSED).
# status=done → archive. Идемпотентно: archived пропускаем.
# ============================================================================
retro_archived=0
log "retro-card-archive: scanning merged PRs → done cards (branch t_<hex> / title ref / own-branch merged)"

# refs: pr_num<TAB>head смерженных PR в окне (используем тот же _retro_prs_json)
_arch_refs="$(printf '%s' "$_retro_prs_json" | python3 -c '
import json, sys
data = json.load(sys.stdin)
since = sys.argv[1]
for pr in data:
    if (pr.get("mergedAt") or "") < since:
        continue
    print(str(pr.get("number","")) + "\t" + (pr.get("headRefName") or ""))
' "$_retro_since" 2>/dev/null || true)"

# Ретро-фикс (14.08 t_0a765152): head-ветки OPEN PR — один запрос на тик.
# Карточка-продолжение REOPENED issue переиспользовала branch_name уже
# влитого PR (#1217: PR #1220 merged на z-{agent}/1217-e2e-40-deepseek,
# триаж создал t_7cc96c7d с ТОЙ ЖЕ веткой → этот проход заархивировал
# ЖИВУЮ карточку, чья работа ещё в OPEN PR #1231). Если на ветке карточки
# есть OPEN PR — merged PR относится к прошлому кругу, карточку НЕ
# архивируем (ни done, ни blocked).
_arch_open_heads="$(gh pr list --repo "$GH_REPO" --state open --base "$DEVELOP_BRANCH" \
    --json number,headRefName 2>/dev/null | python3 -c '
import json, sys
try:
    data = json.load(sys.stdin)
    for pr in data:
        print(pr.get("headRefName") or "")
except Exception:
    pass
' 2>/dev/null || true)"

# Карточки загружаем один раз: id<TAB>status<TAB>branch<TAB>title (не-archived).
# ВАЖНО: пустой branch_name печатаем как "-" (placeholder) — иначе два таба
# подряд схлопываются в один при `IFS=$'\t' read` и title уезжает в branch
# (та же ловушка, что scan-all-prs стр. 1238-1241).
_arch_cards="$( "$HERMES_BIN" kanban --board "$KANBAN_BOARD" list --json 2>/dev/null \
    | python3 -c '
import sys, json
try:
    d = json.load(sys.stdin)
    tasks = d if isinstance(d, list) else d.get("tasks", [])
    for t in tasks:
        if "archived" in (t.get("status") or ""):
            continue
        title = (t.get("title") or "").replace("\t", " ").replace("\n", " ")
        branch = (t.get("branch_name") or "").replace("\t", " ").replace("\n", " ")
        if not branch:
            branch = "-"
        print("{}\t{}\t{}\t{}".format(t.get("id",""), t.get("status",""), branch, title))
except Exception:
    pass
' 2>/dev/null || true)"

# Для каждой done/blocked-карточки: архивируем, если (a) её id встречается в
# head-ветке смерженного PR, (b) title упоминает смерженный PR (#N) / его
# ветку, или (c) собственный branch_name карточки (exact) — смерженный head PR.
# blocked → unblock + complete + archive (ретро 14.08 t_0bd15be9).
while IFS=$'\t' read -r c_id c_status c_branch c_title; do
    [ -z "$c_id" ] && continue
    [ "$c_status" != "done" ] && [ "$c_status" != "blocked" ] && continue
    _matched="$(printf '%s\n' "$_arch_refs" | awk -F'\t' -v id="$c_id" '
        {
            # Извлекаем t_<hex>-токен из head-ветки и сравниваем ТОЧНО с id
            # карточки. Substring-матч ($2 ~ id) опасен: ветка t_abc1234-*
            # ложно совпадёт с карточкой t_abc123 (ретро 14.08 t_36c9ac4e).
            if (match($2, /t_[a-f0-9]+/)) {
                tok = substr($2, RSTART, RLENGTH)
                if (tok == id) { print $1"\t"$2; exit }
            }
        }')"
    if [ -z "$_matched" ]; then
        _matched="$(printf '%s\n' "$_arch_refs" | awk -F'\t' -v t="$c_title" '
            ($1 != "" && index(t, "rebase PR #" $1) > 0) || ($2 != "" && index(t, $2) > 0) {print $1"\t"$2; exit}')"
    fi
    if [ -z "$_matched" ] && [ -n "$c_branch" ] && [ "$c_branch" != "-" ]; then
        # (c) own branch_name (exact) против merged heads — безопасно, без
        # ложных срабатываний на CLOSED-но-не-merged PR.
        _matched="$(printf '%s\n' "$_arch_refs" | awk -F'\t' -v b="$c_branch" '$2 == b {print $1"\t"$2; exit}')"
    fi
    if [ -z "$_matched" ]; then
        continue
    fi
    _arch_pr="${_matched%%$'\t'*}"
    # Ретро-фикс (14.08 t_0a765152): карточка-продолжение REOPENED issue
    # переиспользовала branch_name уже влитого PR (#1217: PR #1220 merged на
    # z-{agent}/1217-e2e-40-deepseek; триаж создал t_7cc96c7d с ТОЙ ЖЕ веткой,
    # merge-gate заархивировал ЖИВУЮ карточку 09:36Z, чья работа ещё в OPEN
    # PR #1231). Если на ветке карточки сейчас есть OPEN PR — merged PR
    # относится к ПРОШЛОМУ кругу (работа уже влита, карточка-продолжение
    # делает НОВЫЙ фикс на той же ветке) → НЕ архивируем ни done, ни blocked.
    # Это и есть «связь PR↔карточка» из РЕШЕНИЯ (б): открытый PR на ветке
    # доказывает, что merged PR — не работа ЭТОЙ карточки.
    if [ -n "$c_branch" ] && [ "$c_branch" != "-" ] \
        && printf '%s\n' "$_arch_open_heads" | grep -Fxq -- "$c_branch"; then
        log "retro-card-archive: card ${c_id} ${c_status} + merged PR #${_arch_pr}, но ветка ${c_branch} имеет OPEN PR — карточка-продолжение, НЕ архивирую (ретро 14.08 t_0a765152)"
        continue
    fi
    if [ "$c_status" = "done" ]; then
        log "retro-card-archive: card ${c_id} done + merged PR #${_arch_pr} → archive"
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would archive card ${c_id} (PR #${_arch_pr})"
            retro_archived=$((retro_archived+1)); continue
        fi
        if "$HERMES_BIN" kanban --board "$KANBAN_BOARD" archive "$c_id" >/dev/null 2>&1; then
            retro_archived=$((retro_archived+1))
            log "retro-card-archive: card ${c_id} archived (PR #${_arch_pr} merged)"
        else
            log "retro-card-archive: WARNING archive failed for ${c_id} (PR #${_arch_pr})"
        fi
    else
        # Ретро 14.08 t_0bd15be9: blocked-карточка с MERGED PR (например,
        # t_36c9ac4e — фикс #1224 влит, карточка timeout×2 → blocked) висела
        # вечно: этот проход скипал status!=done, а recovery не завершала
        # родителя. Фикс влит ⇒ критерий выполнен: unblock → complete → archive.
        log "retro-card-archive: card ${c_id} blocked + merged PR #${_arch_pr} → unblock+complete+archive"
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would unblock+complete+archive card ${c_id} (PR #${_arch_pr})"
            retro_archived=$((retro_archived+1)); continue
        fi
        if "$HERMES_BIN" kanban --board "$KANBAN_BOARD" unblock \
                --reason "фикс влит, критерий выполнен" "$c_id" >/dev/null 2>&1 \
            && "$HERMES_BIN" kanban --board "$KANBAN_BOARD" complete \
                --summary "фикс влит, критерий выполнен (ретро 14.08 t_0bd15be9)" "$c_id" >/dev/null 2>&1; then
            if "$HERMES_BIN" kanban --board "$KANBAN_BOARD" archive "$c_id" >/dev/null 2>&1; then
                retro_archived=$((retro_archived+1))
                log "retro-card-archive: card ${c_id} unblocked+completed+archived (PR #${_arch_pr} merged, was blocked)"
            else
                log "retro-card-archive: WARNING archive failed for ${c_id} (PR #${_arch_pr})"
            fi
        else
            log "retro-card-archive: WARNING unblock/complete failed for blocked ${c_id} (PR #${_arch_pr})"
        fi
    fi
done < <(printf '%s\n' "$_arch_cards")

# ============================================================================
# REST-based backfill: open PR без меток старше 30 мин (ретро 15.08 t_2c814334)
# ----------------------------------------------------------------------------
# ПРОБЛЕМА: clean-pr-sweep (выше) и pr-orphan-reconcile (ниже) используют
# `gh pr list` — GraphQL. При graphql rate-limit=0 (а core жив — квоты РАЗНЫЕ)
# они МОЛЧА возвращают [] → merge-gate НЕ размечает open PR без меток
# (инцидент #1282/#1284/#1286: созданы 07:38-07:55Z в peak-окно, CI green,
# labels=[] 5.5ч, невидимы для e2e и ревью Шифу). G3 (см. выше) теперь
# ЛОГИРУЕТ graphql=0, но тик не прерывает.
# РЕШЕНИЕ: этот скан ходит через REST (`gh api pulls` — core-квота, отдельная
# от graphql). Для open PR (base=develop, не draft, mergeable, без process-
# меток, старше BACKFILL_AGE_MINUTES=30 мин — свежие PR с ещё идущим CI не
# трогаем):
#   - process-only (все файлы .github/, scripts/agent_flow/, docs/) → needs-review;
#   - functional + OPEN issue → needs-e2e на issue (+ PR);
#   - functional + issue CLOSED/нет → needs-review (e2e невозможен — e2e-process
#     ротирует ISSUES, не PR; Шифу решает, урок 13.08 t_42741511).
# Идемпотентно: PR с process-меткой пропускаем; add-label — no-op.
# ============================================================================
BACKFILL_AGE_MINUTES="${BACKFILL_AGE_MINUTES:-30}"
backfill_labeled=0
log "pr-backfill-scan: REST open PRs without process labels, age>${BACKFILL_AGE_MINUTES}m (ретро 15.08 t_2c814334)"
_backfill_prs_json="$(gh api "repos/${GH_REPO}/pulls?state=open&base=${DEVELOP_BRANCH}&per_page=100" 2>/dev/null || echo '[]')"
if [ -z "$_backfill_prs_json" ] || [ "$_backfill_prs_json" = "null" ]; then
    _backfill_prs_json='[]'
fi
while IFS=$'\t' read -r bf_pr bf_head bf_title bf_labels bf_issue bf_created; do
    [ -z "$bf_pr" ] && continue
    [ "$bf_issue" = "-" ] && bf_issue=""
    [ "$bf_labels" = "-" ] && bf_labels=""
    log "pr-backfill-scan: PR #${bf_pr} (${bf_head}) issue=${bf_issue:-none} created=${bf_created:-?}"
    # Stale-branch guard (ретро 14.08 t_28afb585): head уже влита через
    # ДРУГОЙ merged PR — переиспользование ветки, обрабатывает
    # stale_branch_scan_all (блок-коммент), не мы.
    _bf_prev_merged="$(gh pr list --repo "$GH_REPO" --state merged --head "$bf_head" \
        --json number --jq '.[0].number // ""' 2>/dev/null || true)"
    if [ -n "$_bf_prev_merged" ] && [ "$_bf_prev_merged" != "$bf_pr" ]; then
        if is_proposal_branch "$bf_head"; then
            log "pr-backfill-scan: PR #${bf_pr} head ${bf_head} — proposal-ветка архитектора (ретро 15.08 t_6024f414) — классифицирую normally"
        else
            log "pr-backfill-scan: PR #${bf_pr} head ${bf_head} уже влит через PR #${_bf_prev_merged} — skip (stale_branch_scan_all, ретро 14.08 t_28afb585)"
            skipped=$((skipped+1)); continue
        fi
    fi
    # CI-only / process-only? (все файлы в .github/, scripts/agent_flow/, docs/)
    _bf_files="$(gh api "repos/${GH_REPO}/pulls/${bf_pr}/files?per_page=100" \
        --jq '[.[].filename] | join(",")' 2>/dev/null || echo '')"
    _bf_ci_only="$(printf '%s' "$_bf_files" | python3 -c '
import sys
files = [f for f in sys.stdin.read().split(",") if f]
ok = bool(files) and all(
    f.startswith(".github/") or f.startswith("scripts/agent_flow/") or f.startswith("docs/")
    for f in files
)
print("1" if ok else "0")
' 2>/dev/null || echo 0)"
    _bf_kind="$(detect_pr_kind "$(printf '%s' "$bf_labels" | tr '[:upper:]' '[:lower:]')" "$bf_title")"
    if [ "$_bf_ci_only" = "1" ]; then
        _bf_kind="lint"
        log "pr-backfill-scan: PR #${bf_pr} process-only (files: ${_bf_files}) → lint"
    fi
    log "pr-backfill-scan: PR #${bf_pr} kind=${_bf_kind} issue=${bf_issue:-?}"
    if [ "$_bf_kind" = "lint" ]; then
        log "pr-backfill-scan: PR #${bf_pr} lint/process-only → ${NEEDS_REVIEW_LABEL} (skip e2e)"
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would run: gh pr edit ${bf_pr} --repo ${GH_REPO} --add-label ${NEEDS_REVIEW_LABEL}"
            backfill_labeled=$((backfill_labeled+1)); continue
        fi
        if gh pr edit "$bf_pr" --repo "$GH_REPO" --add-label "$NEEDS_REVIEW_LABEL" >/dev/null 2>&1; then
            # issue #1553: self-id whoami AFTER успешного add-label (backfill
            # lint path: CI-only/process → needs-review, e2e не нужен).
            post_whoami_comment pr "$bf_pr" "adding-label:${NEEDS_REVIEW_LABEL}" \
                "pr-backfill-scan: PR #${bf_pr} lint/process-only → needs-review (skip e2e)"
            backfill_labeled=$((backfill_labeled+1))
            log "pr-backfill-scan: PR #${bf_pr} → ${NEEDS_REVIEW_LABEL} (skip e2e)"
        else
            log "pr-backfill-scan: WARNING add ${NEEDS_REVIEW_LABEL} to PR #${bf_pr} failed"
        fi
        continue
    fi
    # functional: нужен e2e. OPEN issue → needs-e2e; иначе (CLOSED/нет) e2e
    # невозможен (e2e-process ротирует issues) → needs-review (Шифу решит).
    _bf_state=""
    if [ -n "$bf_issue" ]; then
        _bf_state="$(gh issue view "$bf_issue" --repo "$GH_REPO" --json state --jq '.state' 2>/dev/null || echo '')"
    fi
    if [ "$_bf_state" = "OPEN" ]; then
        # Взаимоисключение needs-review/needs-e2e (ретро 13.08 t_de63be1f, #942).
        _bf_issue_labels="$(gh issue view "$bf_issue" --repo "$GH_REPO" --json labels \
            --jq '[.labels[].name] | join(",")' 2>/dev/null || echo '')"
        if has_label "$(printf '%s' "$_bf_issue_labels" | tr '[:upper:]' '[:lower:]')" "$NEEDS_REVIEW_LABEL"; then
            log "pr-backfill-scan: PR #${bf_pr} functional, issue #${bf_issue} под ревью — skip (взаимоисключение)"
            skipped=$((skipped+1)); continue
        fi
        log "pr-backfill-scan: PR #${bf_pr} functional, issue #${bf_issue} OPEN → ${NEEDS_E2E_LABEL}"
        if [ "$DRY_RUN" = "true" ]; then
            log "DRY-RUN would run: gh issue edit ${bf_issue} --repo ${GH_REPO} --add-label ${NEEDS_E2E_LABEL}"
            backfill_labeled=$((backfill_labeled+1)); continue
        fi
        # issue #1534: self-id whoami BEFORE pr-backfill-scan label flip.
        whoami_add_label "$bf_issue" "${NEEDS_E2E_LABEL}" "pr-backfill-scan: PR #${bf_pr} functional, returning ${NEEDS_E2E_LABEL} to #${bf_issue}" "pr=${bf_pr}"
        if gh issue edit "$bf_issue" --repo "$GH_REPO" --add-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1; then
            gh pr edit "$bf_pr" --repo "$GH_REPO" --add-label "$NEEDS_E2E_LABEL" >/dev/null 2>&1 || true
            backfill_labeled=$((backfill_labeled+1))
            log "pr-backfill-scan: issue #${bf_issue} + PR #${bf_pr} → ${NEEDS_E2E_LABEL}"
        else
            log "pr-backfill-scan: WARNING add ${NEEDS_E2E_LABEL} to issue #${bf_issue} failed"
        fi
        continue
    fi
    log "pr-backfill-scan: PR #${bf_pr} functional, issue ${bf_issue:-?} state=${_bf_state:-?} — e2e невозможен → ${NEEDS_REVIEW_LABEL}"
    if [ "$DRY_RUN" = "true" ]; then
        log "DRY-RUN would run: gh pr edit ${bf_pr} --repo ${GH_REPO} --add-label ${NEEDS_REVIEW_LABEL}"
        backfill_labeled=$((backfill_labeled+1)); continue
    fi
    if gh pr edit "$bf_pr" --repo "$GH_REPO" --add-label "$NEEDS_REVIEW_LABEL" >/dev/null 2>&1; then
        # issue #1553: self-id whoami AFTER успешного add-label (backfill
        # functional path: e2e невозможен → needs-review).
        post_whoami_comment pr "$bf_pr" "adding-label:${NEEDS_REVIEW_LABEL}" \
            "pr-backfill-scan: PR #${bf_pr} functional, e2e невозможен → needs-review"
        backfill_labeled=$((backfill_labeled+1))
        log "pr-backfill-scan: PR #${bf_pr} → ${NEEDS_REVIEW_LABEL} (e2e невозможен)"
    else
        log "pr-backfill-scan: WARNING add ${NEEDS_REVIEW_LABEL} to PR #${bf_pr} failed"
    fi
done < <(printf '%s' "$_backfill_prs_json" | python3 -c '
import json, sys, re
from datetime import datetime, timezone, timedelta
data = json.load(sys.stdin)
PROCESS = {"needs-e2e", "needs-review", "e2e-done", "e2e:rejected", "no-e2e-required"}
age_min = int(sys.argv[1]) if len(sys.argv) > 1 else 30
now = datetime.now(timezone.utc)
for pr in data:
    if pr.get("draft"):
        continue
    # REST shape: head.ref, mergeable (bool), mergeable_state (str).
    # clean/behind — mergeable без конфликтов; dirty/unknown/blocked/unstable
    # пропускаем (как CLEAN/MERGEABLE в GraphQL clean-pr-sweep).
    if pr.get("mergeable") is not True:
        continue
    if pr.get("mergeable_state") not in ("clean", "behind"):
        continue
    labels = {l.get("name", "") for l in (pr.get("labels") or []) if isinstance(l, dict)}
    if PROCESS & labels:
        continue
    # Возрастной порог: PR моложе 30 мин (CI может ещё идти) не размечаем.
    created = pr.get("created_at") or ""
    try:
        cdt = datetime.fromisoformat(created.replace("Z", "+00:00"))
        if now - cdt < timedelta(minutes=age_min):
            continue
    except Exception:
        pass  # created_at недоступен — размечаем (fail-open)
    pr_num = str(pr.get("number", ""))
    head = (pr.get("head") or {}).get("ref", "") if isinstance(pr.get("head"), dict) else ""
    title = pr.get("title") or ""
    labels_csv = ",".join(sorted(labels))
    m = re.search(r"#(\d+)", title)
    issue_num = m.group(1) if m else ""
    if not issue_num:
        m2 = re.search(r"z-\{agent\}/(\d+)-", head)
        issue_num = m2.group(1) if m2 else ""
    if issue_num.startswith("t_") or len(issue_num) > 7:
        issue_num = ""
    issue_out = issue_num if issue_num else "-"
    labels_out = labels_csv if labels_csv else "-"
    print(f"{pr_num}\t{head}\t{title}\t{labels_out}\t{issue_out}\t{created}")
' "$BACKFILL_AGE_MINUTES" 2>/dev/null)

# --- summary -----------------------------------------------------------------
log "tick done: considered=${considered} labeled=${labeled} skipped=${skipped} errored=${errored} retro_closed=${retro_closed} retro_labeled=${retro_labeled} clean_labeled=${clean_labeled} orphan_labeled=${orphan_labeled} backfill_labeled=${backfill_labeled} retro_archived=${retro_archived} human_close_propagated=${human_close_propagated}"

# Exit non-zero only on hard errors so cron can alert.
if [ "$errored" -gt 0 ]; then exit 1; fi
exit 0
