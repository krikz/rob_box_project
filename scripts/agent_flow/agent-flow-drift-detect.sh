#!/bin/bash
# ============================================================================
# agent-flow-drift-detect.sh — drift-детектор для процессных скриптов.
#
# Source of truth: ветка origin/develop репозитория (после `git fetch origin`),
# файлы <repo>/scripts/agent_flow/*.sh на origin/develop.
# Проверяет, что md5sum между origin/develop и 4 хостами-копиями одинаковые:
#   - /home/builder/hermes-share/rob_box_project/scripts/agent_flow/<file>
#   - /home/builder/.hermes/profiles/agent-flow/scripts/<file>
#   - /home/builder/.hermes/profiles/architect/scripts/<file>
#   - /home/builder/.hermes/profiles/devops/scripts/<file>
#   - /home/builder/.hermes/scripts/<file>
#
# Ретро 12.08 t_24054f6c: install.sh не донёс обновлённый merge-gate/triage
# в 4 host-копии → cron дрифтанулся → ADR-0014 post-merge не работал
# (merge-gate), triage-skip не работал (triage).
#
# Ретро 12.08 t_20775d14: скрипт сверял host-копии с ЛОКАЛЬНЫМ деревом репо.
# Если локальный develop отставал от origin/develop (pull не сделан), host
# совпадал с устаревшим локальным деревом → дрейф host↔origin оставался
# незамеченным (инцидент #1141: fix не доехал на хост ~2ч, install.sh с
# устаревшего дерева перезаписал ручной фикс).
#
# Ретро 13.08 t_9a3f2e0c (пустая round-106): дрейф host↔origin оставался
# слепым ДАЖЕ с fetch — эталоном при local!=origin становилось ЛОКАЛЬНОЕ
# дерево (compute_drift), т.е. host-копии, совпадающие с устаревшим local,
# считались «в синхроне». Теперь эталон ВСЕГДА origin/develop, когда origin
# доступен; LOCAL — только fallback при недоступном fetch. Плюс при FIX
# FAILED сразу создаётся kanban-карточка (create_drift_card), а не ждём
# следующего 30-мин тика крона.
#
# Ретро 13.08 t_9a3f2e0c (продолжение): BRANCH_ACTIVE (воркер в z-ветке) —
# раньше `exit 0` без сверки host↔origin. Главный worktree часто занят
# воркером, поэтому дрейф host↔origin жил часами незамеченным. Теперь в
# BRANCH_ACTIVE тоже проверяем host-копии vs origin/develop (fetch + git show
# безопасны, ветку не трогаем) и при дрейфе создаём карточку (exit 0 —
# cron не валим, карточку разберёт воркер devops).
#
# Ретро 14.08 t_ea771b06: BRANCH_ACTIVE + DRIFT — раньше сразу create_drift_card
# (auto-fix отложен, install.sh из текущего дерева разложил бы веточный код).
# Это 2-й случай, когда дрейф при занятом воркером worktree требовал ручной
# разборки (16:24 14.08 merge-gate+e2e-process; 01:52 14.08 triage). Теперь
# автофикс выполняется ИЗ ВРЕМЕННОГО worktree на origin/develop:
#   git worktree add --detach /tmp/wt-driftfix-$$ origin/develop
#   REPO_DIR=<wt> bash <wt>/scripts/agent_flow/install.sh
#   git worktree remove --force <wt>
# Карточка создаётся ТОЛЬКО если и этот путь не помог (md5-сверка после).
#
# Теперь перед сверкой:
#   1) `git fetch origin develop` (таймаут 30s); при недоступности origin —
#      fallback на локальное дерево + WARN в stdout;
#   2) эталон md5 берётся из origin/develop (git show origin/develop:...),
#      а не из локального дерева;
#   3) если локальный develop != origin/develop — печатается отдельный маркер
#      LOCAL_DESYNC (сигнал: локальное дерево устарело, install.sh из него
#      разложит устаревшие скрипты);
#   4) auto-fix: при отставании локального develop (ветка develop + чистое
#      дерево) сначала `git merge --ff-only origin/develop`, затем install.sh —
#      чтобы install.sh раскладывал СВЕЖИЕ скрипты, а не устаревшие.
#
# Поведение:
#   - BRANCH_ACTIVE (current branch != develop)
#                                       → маркер в stdout; host↔origin
#                                         сверка выполняется; при дрейфе —
#                                         автофикс из временного worktree на
#                                         origin/develop; если вылечилось —
#                                         exit 0 без карточки; если нет —
#                                         create_drift_card; exit 0 (no_op)
#   - OK (host == origin/develop, local == origin/develop)
#                                       → exit 0, stdout пустой (no_op)
#   - LOCAL_DESYNC (local != origin)    → маркер в stdout; автофикс ff-only
#                                         если можно; exit 0/2
#   - DRIFT (host != origin/develop)    → auto-fix: install.sh
#                                         → после fix OK → exit 0, stdout "fixed"
#                                         → fix не помог → exit 1, stdout alert
#                                           + create_drift_card
#
# Exit codes:
#   0 — нет дрифта ИЛИ автофикс успешен (в т.ч. вылечен LOCAL_DESYNC,
#       а также BRANCH_ACTIVE — активная фича-ветка)
#   1 — host DRIFT остался после автофикса (нужна ручная разборка)
#   2 — LOCAL_DESYNC не вылечен (локальный develop != origin/develop,
#       автофикс ff-only невозможен/не помог — нужен ручной pull)
#
# Переменные окружения (оператор/тесты):
#   REPO_DIR        — путь к репо (по умолчанию dev-машина; как в install.sh)
#   DRIFT_DRY_RUN=1 — только детект, без auto-fix (как install.sh --dry-run)
#
# Используется cron'ом `Agent Flow Scripts Drift` (no_agent=True, every 30m)
# в профиле devops. Cron scheduler выводит stdout, если непустой — это
# и есть сигнал тревоги. Тихий tick (пустой stdout) означает ОК.
# (Ретро 12.08 t_b62e4c5c: 6h было слишком редко — drift merge-gate/triage
# копился 8.5+ часов до того, как drift-detect мог его заметить. 30m
# позволяет auto-fix догонять merge в пределах одного tick.)
#
# Идемпотентен. Запускать можно вручную:
#   bash /home/builder/.hermes/scripts/agent-flow-drift-detect.sh
#   DRIFT_DRY_RUN=1 bash .../agent-flow-drift-detect.sh   # только детект
# ============================================================================
set -u

REPO_DIR="${REPO_DIR:-/home/builder/hermes-share/rob_box_project}"
SCRIPT_DIR="$REPO_DIR/scripts/agent_flow"
INSTALL_SH="$SCRIPT_DIR/install.sh"
ALERT_LOG="${DRIFT_ALERT_LOG:-/home/builder/.hermes/profiles/devops/cron/output/agent-flow-drift.alert.log}"
REF_BRANCH="origin/develop"
LOCAL_BRANCH="develop"
FETCH_TIMEOUT=30
# Префикс временного worktree для автофикса при BRANCH_ACTIVE (ретро 14.08
# t_ea771b06). Переопределяется в тестах, чтобы не плодить /tmp.
DRIFT_WT_PREFIX="${DRIFT_WT_PREFIX:-/tmp/wt-driftfix-}"

DRY_RUN=0
[ "${1:-}" = "--dry-run" ] && DRY_RUN=1
[ "${DRIFT_DRY_RUN:-}" = "1" ] && DRY_RUN=1

# Какие файлы проверяем — единый источник: EXPECTED из install.sh
# (`install.sh --list-files`). Ретро 13.08 t_2cae75c0: раньше список
# дублировался здесь и разошёлся с install.sh (deploy-sweep, unlabeled-sweep,
# kanban-retro-create отсутствовали) — drift-контроль молча не работал.
# Ретро 14.08 t_ea771b06: список берём из install.sh на origin/develop
# (эталон), а НЕ из локального дерева — на фича-ветке воркера install.sh
# может быть старым (без --list-files) → FILES наполнился бы мусором из
# full-install output и дрейф остался бы слепым (наблюдалось в live-прогоне
# на z-devops/t_423453b1). Защита: фильтр на имена *.sh.
FILES=()
# 1) эталон: install.sh на origin/develop (после fetch ниже ref обновится,
#    но даже pre-fetch ref достаточно — список файлов стабилен)
if git -C "$REPO_DIR" cat-file -e "$REF_BRANCH:scripts/agent_flow/install.sh" 2>/dev/null; then
    while IFS= read -r f; do
        case "$f" in
            *.sh) FILES+=("$f") ;;
        esac
    done < <(git -C "$REPO_DIR" show "$REF_BRANCH:scripts/agent_flow/install.sh" 2>/dev/null | bash -s -- --list-files 2>/dev/null)
fi
# 2) fallback: локальный install.sh (если origin недоступен)
if [ "${#FILES[@]}" = "0" ] && [ -f "$INSTALL_SH" ]; then
    while IFS= read -r f; do
        case "$f" in
            *.sh) FILES+=("$f") ;;
        esac
    done < <(bash "$INSTALL_SH" --list-files 2>/dev/null)
fi
# 3) fallback: если install.sh недоступен/сломан — прежний статический список
if [ "${#FILES[@]}" = "0" ]; then
    FILES=(
        agent-flow-triage.sh
        agent-flow-merge-gate.sh
        agent-flow-e2e-process.sh
        agent-flow-handoff.sh
        round_ensure.sh
        agent-flow-cleanup-249.sh
        agent-flow-deploy-sweep.sh
        agent-flow-unlabeled-sweep.sh
        cron-loop.sh
        watchdog.sh
        agent-flow-drift-detect.sh
        kanban-retro-create.sh
    )
fi

TARGETS=()
if [ -n "${DRIFT_TARGETS:-}" ]; then
    # Тесты/hermetic прогоны: DRIFT_TARGETS — colon-separated список путей.
    IFS=':' read -r -a TARGETS <<< "$DRIFT_TARGETS"
else
    TARGETS=(
        "$REPO_DIR/scripts/agent_flow"
        "/home/builder/.hermes/profiles/agent-flow/scripts"
        "/home/builder/.hermes/profiles/architect/scripts"
        "/home/builder/.hermes/profiles/devops/scripts"
        "/home/builder/.hermes/scripts"
    )
fi

mkdir -p "$(dirname "$ALERT_LOG")" 2>/dev/null || true

log() {
    # Пишем и в stdout (для cron) и в alert log (для истории)
    local line
    line="[$(date -Iseconds)] $*"
    echo "$line"
    echo "$line" >> "$ALERT_LOG" 2>/dev/null || true
}

if [ ! -d "$SCRIPT_DIR" ]; then
    log "ERROR: source dir not found: $SCRIPT_DIR (clone the repo there?)"
    exit 1
fi

# get_origin_md5 <file> — md5 файла на origin/develop (пусто, если недоступен)
get_origin_md5() {
    local f="$1"
    if [ "$FETCH_OK" != "1" ]; then
        echo ""
        return
    fi
    if git -C "$REPO_DIR" cat-file -e "$REF_BRANCH:scripts/agent_flow/$f" 2>/dev/null; then
        git -C "$REPO_DIR" show "$REF_BRANCH:scripts/agent_flow/$f" 2>/dev/null | md5sum | cut -c1-12
    fi
}

# compute_drift — заполняет DRIFT/DRIFT_FILES: host-копии vs эталон.
# Эталон ВСЕГДА origin/develop, когда origin доступен (FETCH_OK=1):
# ретро 13.08 t_9a3f2e0c (пустая round-106): раньше при local!=origin
# эталоном становилось ЛОКАЛЬНОЕ дерево → host-копии, совпадающие с
# устаревшим local, считались «в синхроне» → дрейф host↔origin оставался
# слепым (t_20775d14 не дочинен). LOCAL используется ТОЛЬКО как fallback,
# когда origin недоступен (fetch не удался).
DRIFT=0
DRIFT_FILES=()
compute_drift() {
    DRIFT=0
    DRIFT_FILES=()
    local f t CUR_MD5 ORIGIN_MD5 LOCAL_MD5 REF_MD5 HAS_MISMATCH
    for f in "${FILES[@]}"; do
        if [ ! -f "$SCRIPT_DIR/$f" ]; then
            continue
        fi
        LOCAL_MD5="$(md5sum "$SCRIPT_DIR/$f" | cut -c1-12)"
        ORIGIN_MD5="$(get_origin_md5 "$f")"
        if [ -n "$ORIGIN_MD5" ]; then
            REF_MD5="$ORIGIN_MD5"
        else
            REF_MD5="$LOCAL_MD5"
        fi
        HAS_MISMATCH=0
        for t in "${TARGETS[@]}"; do
            if [ "$t" = "$SCRIPT_DIR" ]; then
                continue  # эталон не сравниваем сам с собой
            fi
            if [ -f "$t/$f" ]; then
                CUR_MD5="$(md5sum "$t/$f" | cut -c1-12)"
                if [ "$CUR_MD5" != "$REF_MD5" ]; then
                    HAS_MISMATCH=1
                    break
                fi
            fi
        done
        if [ "$HAS_MISMATCH" = "1" ]; then
            DRIFT=1
            DRIFT_FILES+=("$f")
        fi
    done
}

# create_drift_card — немедленная kanban-карточка при НЕразрешимом дрифте
# (ретро 13.08 t_9a3f2e0c: пустая round-106 из-за host↔origin дрейфа; раньше
# FIX FAILED просто писал alert в stdout и ждал следующего 30-мин тика крона —
# пока воркер/надзор заметит, дрейф живёт часами). Теперь поднимаем карточку
# сразу, через kanban-retro-create.sh (dedup по стабильному ключу — повторные
# тики не плодят дубли, карточка обновляется воркером).
RETRO_CREATE="${RETRO_CREATE:-/home/builder/.hermes/scripts/kanban-retro-create.sh}"
create_drift_card() {
    local key="agent-flow-host-drift-fix-failed"
    local title="ретро: host-дрейф agent-flow скриптов — auto-fix не помог (FIX FAILED)"
    local body="drift-detect: host-копии != origin/develop, install.sh не вылечил.
DRIFT_FILES: ${DRIFT_FILES[*]:-n/a}
TARGETS: ${TARGETS[*]}
См. $ALERT_LOG
Порядок: fetch origin/develop → install.sh → md5 сверка."
    if [ -x "$RETRO_CREATE" ]; then
        bash "$RETRO_CREATE" \
            --title "$title" \
            --body "$body" \
            --assignee devops \
            --key "$key" 2>&1 | sed 's/^/  [retro-card] /' || true
    else
        log "  [retro-card] kanban-retro-create.sh not found ($RETRO_CREATE) — card NOT created"
    fi
}

# --- Активная фича-ветка (current branch != develop) — нормальное состояние
# воркера, который сидит в z-* ветке и ещё не смержил PR. Это НЕ drift по
# локальному дереву, НО host↔origin дрейф всё равно проверяем: ретро 13.08
# t_9a3f2e0c (пустая round-106) — главный worktree часто занят воркером в
# z-ветке, и при старом `exit 0` drift-detect НЕ проверял host-копии вовсе,
# поэтому дрейф host↔origin жил часами незамеченным.
# Ретро 12.08 t_0d2479ba: раньше в этом состоянии скрипт уходил в
# LOCAL_DESYNC → try_ff_update (skip, branch != develop) → exit 1, из-за чего
# cron падал каждые 30 мин и requeue'ил kanban-карточки (инцидент t_20775d14).
# Решение: проверяем host-копии vs origin/develop (fetch + git show — безопасны,
# ветку не трогаем), при дрейфе — create_drift_card, НО exit 0 (не валим cron,
# карточку разберёт воркер devops). install.sh НЕ запускаем: он раскладывает
# файлы ТЕКУЩЕЙ ветки (z-*), что разнесло бы по хосту незамерженный код.
#
# Ретро 14.08 t_ea771b06: НО если дрейф можно вылечить БЕЗ установки кода
# текущей ветки — раскладываем host-копии из временного worktree на
# origin/develop (чистый эталон, не веточный код):
#   git worktree add --detach /tmp/wt-driftfix-$$ origin/develop
#   REPO_DIR=<wt> bash <wt>/scripts/agent_flow/install.sh
#   git worktree remove --force <wt>
# После — md5-сверка; карточка создаётся ТОЛЬКО если и этот путь не помог.
CURRENT_BRANCH="$(git -C "$REPO_DIR" branch --show-current 2>/dev/null || true)"
if [ -z "$CURRENT_BRANCH" ]; then
    # detached HEAD / не-git — не мешаем старой логике (fallback ниже)
    CURRENT_BRANCH="$(git -C "$REPO_DIR" rev-parse --abbrev-ref HEAD 2>/dev/null || true)"
fi

# --- git fetch origin (эталон — origin/develop) ---
# Выполняем ДО ветвления по BRANCH_ACTIVE: fetch не меняет рабочую ветку,
# а get_origin_md5 (git show origin/develop:...) нужен и в z-ветке.
FETCH_OK=0
if timeout "$FETCH_TIMEOUT" git -C "$REPO_DIR" fetch origin develop >/dev/null 2>&1; then
    FETCH_OK=1
else
    log "WARN: git fetch origin failed (timeout ${FETCH_TIMEOUT}s) — falling back to local-tree comparison; origin/develop drift NOT checked"
fi

# branch_active_autofix — автофикс при BRANCH_ACTIVE через временный worktree
# на origin/develop (ретро 14.08 t_ea771b06). install.sh из worktree раскладывает
# host-копии ИЗ origin/develop, а не из текущей z-ветки. После — md5-сверка.
# Возврат: 0 = вылечено (или worktree недоступен — карточка всё равно создана
# вызывающим), 1 = не вылечено.
branch_active_autofix() {
    local wt="${DRIFT_WT_PREFIX}$$"
    log "BRANCH_ACTIVE auto-fix: temp worktree $wt at $REF_BRANCH"
    if ! git -C "$REPO_DIR" worktree add --detach "$wt" "$REF_BRANCH" >>"$ALERT_LOG" 2>&1; then
        log "FIX FAILED — git worktree add $wt $REF_BRANCH failed"
        create_drift_card
        return 1
    fi
    local wt_install="$wt/scripts/agent_flow/install.sh"
    if [ ! -f "$wt_install" ]; then
        log "FIX FAILED — $wt_install not found in worktree"
        git -C "$REPO_DIR" worktree remove --force "$wt" >>"$ALERT_LOG" 2>&1 || rm -rf "$wt"
        create_drift_card
        return 1
    fi
    log "Auto-fix (worktree): REPO_DIR=$wt bash $wt_install"
    if REPO_DIR="$wt" bash "$wt_install" >> "$ALERT_LOG" 2>&1; then
        log "Auto-fix (worktree) OK. Re-checking drift..."
        compute_drift
        if [ "$DRIFT" = "0" ]; then
            log "FIXED — drift resolved via origin/develop worktree"
            git -C "$REPO_DIR" worktree remove --force "$wt" >>"$ALERT_LOG" 2>&1 || rm -rf "$wt"
            return 0
        fi
        log "FIX FAILED — drift still present after worktree install: ${DRIFT_FILES[*]}"
        create_drift_card
    else
        log "FIX FAILED — install.sh from worktree exited non-zero"
        create_drift_card
    fi
    git -C "$REPO_DIR" worktree remove --force "$wt" >>"$ALERT_LOG" 2>&1 || rm -rf "$wt"
    return 1
}

if [ -n "$CURRENT_BRANCH" ] && [ "$CURRENT_BRANCH" != "$LOCAL_BRANCH" ]; then
    echo "BRANCH_ACTIVE: $CURRENT_BRANCH"
    if [ "$FETCH_OK" = "1" ]; then
        compute_drift
        if [ "$DRIFT" = "1" ]; then
            log "DRIFT detected while BRANCH_ACTIVE ($CURRENT_BRANCH): ${DRIFT_FILES[*]}"
            if [ "$DRY_RUN" = "1" ]; then
                log "DRY-RUN: auto-fix skipped (DRIFT_DRY_RUN=1)"
            else
                branch_active_autofix
            fi
        fi
    fi
    exit 0
fi

# --- LOCAL_DESYNC: локальный develop != origin/develop ---
DESYNC=0
if [ "$FETCH_OK" = "1" ]; then
    LOCAL_REF="$(git -C "$REPO_DIR" rev-parse --short "$LOCAL_BRANCH" 2>/dev/null || true)"
    ORIGIN_REF="$(git -C "$REPO_DIR" rev-parse --short "$REF_BRANCH" 2>/dev/null || true)"
    if [ -n "$LOCAL_REF" ] && [ -n "$ORIGIN_REF" ] && [ "$LOCAL_REF" != "$ORIGIN_REF" ]; then
        DESYNC=1
        log "LOCAL_DESYNC: local $LOCAL_BRANCH=$LOCAL_REF != $REF_BRANCH=$ORIGIN_REF (repo $REPO_DIR)"
    fi
fi

# try_ff_update — подтянуть локальный develop к origin/develop, если безопасно:
# ветка develop, чистое дерево, merge только fast-forward.
# Возврат: 0 = healed, 1 = skip (ветка/дерево не позволяют), 2 = ff-merge failed.
try_ff_update() {
    [ "$DESYNC" = "1" ] || return 0
    local branch dirty new_local
    branch="$(git -C "$REPO_DIR" branch --show-current 2>/dev/null || true)"
    if [ "$branch" != "$LOCAL_BRANCH" ]; then
        log "LOCAL_DESYNC: auto-update skipped (current branch '$branch' != '$LOCAL_BRANCH')"
        return 1
    fi
    dirty="$(git -C "$REPO_DIR" status --porcelain 2>/dev/null | head -1)"
    if [ -n "$dirty" ]; then
        log "LOCAL_DESYNC: auto-update skipped (working tree dirty: $dirty)"
        return 1
    fi
    if git -C "$REPO_DIR" merge --ff-only "$REF_BRANCH" >>"$ALERT_LOG" 2>&1; then
        new_local="$(git -C "$REPO_DIR" rev-parse --short "$LOCAL_BRANCH" 2>/dev/null || true)"
        if [ "$new_local" = "$ORIGIN_REF" ]; then
            log "LOCAL_DESYNC healed: $LOCAL_BRANCH -> $new_local"
            return 0
        fi
        log "LOCAL_DESYNC: $LOCAL_BRANCH ahead/diverged (local=$new_local origin=$ORIGIN_REF) — pending commits, manual check"
        return 2
    fi
    log "LOCAL_DESYNC: ff-only merge failed (diverged?) — manual pull needed"
    return 2
}

compute_drift

# --- десинк локального дерева без дрифта host-копий ---
if [ "$DRIFT" = "0" ] && [ "$DESYNC" = "1" ]; then
    if [ "$DRY_RUN" = "1" ]; then
        log "DRY-RUN: LOCAL_DESYNC detected, auto-update skipped (DRIFT_DRY_RUN=1)"
        exit 1
    fi
    if try_ff_update; then
        compute_drift  # локальное дерево теперь свежее — пересверяем host
        if [ "$DRIFT" = "0" ]; then
            log "OK: local $LOCAL_BRANCH == $REF_BRANCH, host copies in sync"
            exit 0
        fi
        # host-копии устарели относительно свежего дерева → идём в автофикс
        log "After LOCAL_DESYNC heal: host copies drifted vs fresh tree — running auto-fix"
    else
        # LOCAL_DESYNC не вылечен. Код возврата try_ff_update:
        #   1 = skip (ветка != develop / дерево грязное) → exit 1 (как было)
        #   2 = ff-only merge failed (diverged) → exit 2 (LOCAL_DESYNC severity)
        exit $?
    fi
fi

if [ "$DRIFT" = "0" ]; then
    # Тихий tick — не пишем никуда, exit 0
    exit 0
fi

# === ДРИФТ ОБНАРУЖЕН ===
log "DRIFT detected in ${#DRIFT_FILES[@]} file(s): ${DRIFT_FILES[*]}"
for f in "${DRIFT_FILES[@]}"; do
    LOCAL_MD5="$(md5sum "$SCRIPT_DIR/$f" | cut -c1-12)"
    ORIGIN_MD5="$(get_origin_md5 "$f")"
    PEND=""
    if [ -n "$ORIGIN_MD5" ] && [ "$ORIGIN_MD5" != "$LOCAL_MD5" ]; then
        PEND=" [local blob != origin/develop]"
    fi
    log "  $f:$PEND origin=${ORIGIN_MD5:-n/a} local=$LOCAL_MD5"
    for t in "${TARGETS[@]}"; do
        if [ "$t" = "$SCRIPT_DIR" ]; then
            continue
        fi
        if [ -f "$t/$f" ]; then
            MD="$(md5sum "$t/$f" | cut -c1-12)"
            INO="$(stat -c '%i' "$t/$f" 2>/dev/null || echo '?')"
            log "    $MD  inode=$INO  $t/$f"
        else
            log "    MISSING  $t/$f"
        fi
    done
done

if [ "$DRY_RUN" = "1" ]; then
    log "DRY-RUN: skipping auto-fix (set DRIFT_DRY_RUN=0 or run without --dry-run to fix)"
    exit 1
fi

# === АВТОФИКС ===
# Сначала лечим локальное дерево (чтобы install.sh раскладывал свежие скрипты),
# если это ещё не сделано выше.
if [ "$DESYNC" = "1" ]; then
    if ! try_ff_update; then
        log "WARN: continuing auto-fix with local tree as-is (may deploy stale scripts)"
    fi
fi

log "Auto-fix: bash $INSTALL_SH"
if REPO_DIR="$REPO_DIR" bash "$INSTALL_SH" >> "$ALERT_LOG" 2>&1; then
    log "Auto-fix OK. Re-checking drift..."
    STILL_DRIFT=0
    for f in "${DRIFT_FILES[@]}"; do
        LOCAL_MD5="$(md5sum "$SCRIPT_DIR/$f" | cut -c1-12)"
        ORIGIN_MD5="$(get_origin_md5 "$f")"
        if [ -n "$ORIGIN_MD5" ]; then
            REF_MD5="$ORIGIN_MD5"
        else
            REF_MD5="$LOCAL_MD5"
        fi
        for t in "${TARGETS[@]}"; do
            if [ "$t" = "$SCRIPT_DIR" ]; then
                continue
            fi
            if [ -f "$t/$f" ]; then
                CUR_MD5="$(md5sum "$t/$f" | cut -c1-12)"
                if [ "$CUR_MD5" != "$REF_MD5" ]; then
                    STILL_DRIFT=1
                    break 2
                fi
            fi
        done
    done
    if [ "$STILL_DRIFT" = "0" ]; then
        log "FIXED — drift resolved"
        exit 0
    else
        log "FIX FAILED — drift still present after install.sh"
        create_drift_card
        exit 1
    fi
else
    log "FIX FAILED — install.sh exited non-zero"
    create_drift_card
    exit 1
fi
