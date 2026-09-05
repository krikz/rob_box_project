#!/bin/bash
# sync-skills.sh — доставка repo-скиллов (.agents/skills) в Hermes-профили воркеров.
#
# Source of truth: <repo>/.agents/skills/<skill>/SKILL.md
# Target: <profile>/skills/repo/<skill>/  (hardlink, как install.sh для скриптов)
#
# Зачем (ретро 05.09): `af_skill_for_profile()` в lib_agent_flow_common.sh
# маппит тип задачи (label `bug`/`type:functional`/`type:refactor`/…) на
# repo-скилл (systematic-debugging / test-driven-development /
# codebase-design / agent-flow). Но эти скиллы лежат в репо, а профили
# воркеров (backend/devops/…) их НЕ видят — раньше доставки не было, и
# любой скилл из репо улучшал только сессии Шифу, но не качество кода
# воркеров. Этот скрипт закрывает дыру: раскладывает allowlist-скиллы
# hardlink-ами в `skills/repo/<skill>/` каждого профиля.
#
# Категория `repo/` выбрана намеренно (не плоский `skills/<skill>/` и не
# чужая категория `bundled/`): (a) не коллизит с уже установленными
# скиллами профиля; (b) валидатор `_validate_skills_for_assignee`
# (vendor-патч hermes-agent-skill-validation.patch) ходит рекурсивно через
# `iter_skill_index_files` и видит `repo/<skill>/SKILL.md` так же, как
# runtime skill-loader; (c) `_profile_skill_names` считает именем скилла
# имя родительской директории SKILL.md — т.е. ровно `<skill>`.
#
# Запуск:
#   bash scripts/agent_flow/sync-skills.sh --dry-run
#   bash scripts/agent_flow/sync-skills.sh
#   REPO_DIR=/tmp/install_af_XXXX bash scripts/agent_flow/sync-skills.sh
#   HERMES_HOME=/tmp/hermes bash scripts/agent_flow/sync-skills.sh
#   SKILL_SYNC_PROFILES="backend devops" bash scripts/agent_flow/sync-skills.sh
#
# Идемпотентен: повторный запуск обновляет hardlink-и, ничего не ломает.
# Вызывается install.sh (best-effort) после раскладки скриптов.

set -euo pipefail

DRY_RUN=false
if [ "${1:-}" = "--dry-run" ]; then
    DRY_RUN=true
fi
if [ "${1:-}" = "--list-skills" ]; then
    # Позволяет тестам/другим скриптам читать allowlist из одного источника.
    : # список печатается ниже, после объявления SKILL_SYNC_ALLOWLIST
fi

# REPO_DIR resolution — тот же паттерн, что у install.sh (REPO_DIR env или
# позиционный аргумент; --dry-run съедает первый аргумент).
if [ "${1:-}" = "--dry-run" ]; then
    REPO_DIR="${REPO_DIR:-${2:-/home/builder/hermes-share/rob_box_project}}"
else
    REPO_DIR="${REPO_DIR:-${1:-/home/builder/hermes-share/rob_box_project}}"
fi
SKILLS_SRC="$REPO_DIR/.agents/skills"

# Канонические пути профилей. Переопределяются HERMES_HOME (как и весь
# agent-flow) и SKILL_SYNC_PROFILES (space-separated) для тестов.
HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"
if [ -n "${SKILL_SYNC_PROFILES:-}" ]; then
    # shellcheck disable=SC2206
    SKILL_TARGET_PROFILES=( ${SKILL_SYNC_PROFILES} )
else
    SKILL_TARGET_PROFILES=(
        backend
        devops
        tester
        pr-reviewer
        architect
        agent-flow
        analyst
    )
fi

# Allowlist repo-скиллов, доставляемых воркерам. Владелец списка — этот
# файл: af_skill_for_profile() маппит типы задач ТОЛЬКО на скиллы отсюда,
# иначе fallback на роль. Скилл обязан иметь `.agents/skills/<skill>/SKILL.md`
# с `name: <skill>` в frontmatter (иначе runtime loader не резолвит).
SKILL_SYNC_ALLOWLIST=(
    systematic-debugging
    test-driven-development
    codebase-design
    verification-before-completion
    agent-flow
    # Вторая волна (05.09, план «4 скилла mattpocock»):
    code-review               # первичный для pr-reviewer (двухосевое ревью diff)
    to-tickets                # явно передаётся big-bang guard'ом (architect-карточка)
    resolving-merge-conflicts # доступен всем профилям на merge/rebase конфликтах
    ponytail                  # доступен backend/devops (YAGNI-лестница, DRY)
)

if [ "${1:-}" = "--list-skills" ]; then
    printf '%s\n' "${SKILL_SYNC_ALLOWLIST[@]}"
    exit 0
fi

run() {
    if $DRY_RUN; then
        echo "  [DRY] $*"
    else
        "$@"
    fi
}

# sanity check — исходные SKILL.md на месте.
for s in "${SKILL_SYNC_ALLOWLIST[@]}"; do
    if [ ! -f "$SKILLS_SRC/$s/SKILL.md" ]; then
        echo "ERROR: missing canonical skill $SKILLS_SRC/$s/SKILL.md" >&2
        exit 2
    fi
done

echo "==> Source of truth (skills): $SKILLS_SRC"

_sync_one_skill() {  # $1=src_dir $2=dst_dir
    local src="$1" dst="$2" name
    name="$(basename "$dst")"

    # Уже доставлено: SKILL.md — hardlink на src (та же inode) или содержимое
    # совпадает (cp-fallback прошлого запуска). Перепроверяем md5 ниже.
    if [ -f "$dst/SKILL.md" ] && [ ! -L "$dst/SKILL.md" ]; then
        if cmp -s "$dst/SKILL.md" "$src/SKILL.md" 2>/dev/null; then
            echo "  OK   $name (content matches, skip)"
            return 0
        fi
    fi

    # Убираем старую копию (hardlink-директорию), кладём свежую.
    if [ -e "$dst" ] || [ -L "$dst" ]; then
        run rm -rf "$dst"
    fi

    # 1) hardlink-дерево (cp -al) — основной режим, как install.sh.
    if $DRY_RUN || cp -al "$src" "$dst" 2>/dev/null; then
        echo "  HLINK $name -> $dst"
        return 0
    fi
    # 2) regular copy fallback (cross-device, Windows без hardlink и т.п.).
    if cp -aL "$src" "$dst" 2>/dev/null; then
        echo "  COPY $name (hardlink unavailable, used copy)"
        return 0
    fi
    echo "  ERROR $name: cannot deliver skill (cp -al / cp -aL both failed)" >&2
    return 1
}

for profile in "${SKILL_TARGET_PROFILES[@]}"; do
    skills_dir="$HERMES_HOME/profiles/$profile/skills"
    if [ ! -d "$skills_dir" ]; then
        echo "  SKIP $profile (no $skills_dir)"
        continue
    fi
    echo "==> $profile"
    run mkdir -p "$skills_dir/repo"
    for s in "${SKILL_SYNC_ALLOWLIST[@]}"; do
        _sync_one_skill "$SKILLS_SRC/$s" "$skills_dir/repo/$s"
    done
done

# ---------------------------------------------------------------------
# Post-sync md5 verification по всем профилям × allowlist. Если host-копия
# != SOT — exit 3 (алерт-лог — тот же, что у install.sh, чтобы drift-detect
# и оператор увидели одной строкой). В dry-run exit 0 всегда.
# ---------------------------------------------------------------------
VERIFY_FAIL=0
echo
echo "==> Post-sync md5 verification (${#SKILL_TARGET_PROFILES[@]} profiles × ${#SKILL_SYNC_ALLOWLIST[@]} skills)"
for s in "${SKILL_SYNC_ALLOWLIST[@]}"; do
    src_md5="$(md5sum "$SKILLS_SRC/$s/SKILL.md" | awk '{print $1}')"
    for profile in "${SKILL_TARGET_PROFILES[@]}"; do
        dst="$HERMES_HOME/profiles/$profile/skills/repo/$s/SKILL.md"
        if [ ! -f "$dst" ]; then
            # Профиль без skills-дерева или скилл ещё не доставлен — SKIP, не FAIL.
            # (Профили, где нет `skills/`, обрабатываются в цикле раскладки.)
            if [ -d "$HERMES_HOME/profiles/$profile/skills" ]; then
                echo "  FAIL $s missing in $profile/skills/repo/"
                VERIFY_FAIL=1
            else
                echo "  SKIP $s in $profile (no skills dir)"
            fi
            continue
        fi
        dst_md5="$(md5sum "$dst" | awk '{print $1}')"
        if [ "$dst_md5" != "$src_md5" ]; then
            echo "  FAIL $s differs in $profile (src=$src_md5 dst=$dst_md5)"
            VERIFY_FAIL=1
        else
            echo "  OK   $s in $profile"
        fi
    done
done

if [ "$VERIFY_FAIL" = "1" ] && ! $DRY_RUN; then
    echo "==> exit 3 (skills sync verify FAIL)" >&2
    exit 3
fi
echo "  OK skills sync verify passed"
exit 0
