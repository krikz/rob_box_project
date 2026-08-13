#!/bin/bash
# install.sh — раскладка процессных скриптов agent-flow по нужным директориям.
#
# Source of truth: <repo>/scripts/agent_flow/*.sh (эта папка).
# Копии (которые ищет cron при старте):
#   1. /home/builder/.hermes/profiles/agent-flow/scripts/   — каноническое место
#   2. /home/builder/.hermes/profiles/architect/scripts/    — где cron сейчас ищет
#   3. /home/builder/.hermes/scripts/                       — legacy (cron тоже стартует)
#
# Этот скрипт раскладывает **hardlink (cp -al) на канонические файлы из репо**.
# Hardlink — обычный файл с тем же inode, поэтому он гарантированно остаётся
# ВНУТРИ целевой директории (а не symlink наружу) и проходит guard
# `hermes-agent/cron/scheduler.py::_validate_script_path`, который требует
# `path.resolve()` находиться внутри `HERMES_HOME/scripts/`.
#
# Ретро 11.08 18:49 (kanban t_a6a236e0d9f0470e): cron 'Agent Cockpeat Watch Tock'
# (id 2fcf2ad1bd0b) упал на 50 тиков подряд (1ч42м даунтайма), потому что
# install.sh раскладывал симлинки `watchdog.sh -> .../repo/scripts/...`,
# а scheduler.py guard ресолвил их наружу `scripts_dir` и отклонял с
# "Blocked: script path resolves outside the scripts directory".
#
# Резервные пути раскладки (если hardlink невозможен — cross-device):
#   1) cp -aL (копия содержимого) — fallback по умолчанию для symlink;
#   2) symbolic link — крайний случай, только для директорий вне `scripts_dir`;
#      для самой `~/.hermes/scripts/` symlink ЗАПРЕЩЁН (сломает guard).
#
# Гарантии:
#   - Все 3 (или сколько есть) путей ссылаются на одну и ту же inode-копию
#     (hardlink) либо на одинаковое содержимое (cp);
#   - Правка в репо (через PR/merge) автоматически расходится по всем путям
#     сразу при следующем запуске этого скрипта;
#   - Ни один файл в TARGET_DIRS не указывает за пределы своей директории
#     (anti-escape guard в конце);
#   - Нет дубликатов, нет drift (см. scripts/agent_flow/README.md).
#
# Запуск:
#   ./scripts/agent_flow/install.sh            # раскладка на хост
#   ./scripts/agent_flow/install.sh --dry-run  # только показать что сделает
#
# Идемпотентен — повторный запуск обновляет ссылки, ничего не ломает.

set -e

# Единый список процессных скриптов. Владелец списка — ЭТОТ файл:
#   - install.sh раскладывает EXPECTED по хостам;
#   - agent-flow-drift-detect.sh читает список через `install.sh --list-files`
#     (ретро 13.08 t_2cae75c0: раньше список дублировался в drift-detect.sh,
#     из-за чего kanban-retro-create.sh и ещё 2 скрипта не контролировались
#     drift-детектором; теперь источник один).
EXPECTED=(
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

# Режим --list-files: печатает EXPECTED по одному имени на строку и выходит.
# Используется agent-flow-drift-detect.sh как единый источник списка.
if [ "${1:-}" = "--list-files" ]; then
    printf '%s\n' "${EXPECTED[@]}"
    exit 0
fi

DRY_RUN=false
[ "${1:-}" = "--dry-run" ] && DRY_RUN=true

# Если передан явный REPO_DIR (например на build host, где нет
# /home/builder/hermes-share), используем его:
#   bash install.sh /tmp/install_af_1107
#   bash install.sh --dry-run /tmp/install_af_1107
#   REPO_DIR=/tmp/install_af_1107 bash install.sh
# По умолчанию — путь dev-машины.
if [ "${1:-}" = "--dry-run" ]; then
    REPO_DIR="${REPO_DIR:-${2:-/home/builder/hermes-share/rob_box_project}}"
else
    REPO_DIR="${REPO_DIR:-${1:-/home/builder/hermes-share/rob_box_project}}"
fi
SCRIPT_DIR="$REPO_DIR/scripts/agent_flow"

# Канонические пути (все должны стать hardlink-ами на одну и ту же inode)
TARGET_DIRS=(
    "/home/builder/.hermes/profiles/agent-flow/scripts"
    "/home/builder/.hermes/profiles/architect/scripts"
    "/home/builder/.hermes/profiles/devops/scripts"
    "/home/builder/.hermes/scripts"
)

# ~/.hermes/scripts/ проходит через guard в
# hermes-agent/cron/scheduler.py::_validate_script_path.
# Любой symlink наружу этой директории будет отклонён, поэтому
# для HERMES_SCRIPTS_DIR симлинки ЗАПРЕЩЕНЫ (см. ретро 11.08 t_a6a236e0d9f0470e).
HERMES_SCRIPTS_DIR="/home/builder/.hermes/scripts"

run() {
    if $DRY_RUN; then
        echo "  [DRY] $*"
    else
        "$@"
    fi
}

# _remove_existing dst — убирает существующий dst (regular file или symlink),
# оставляя .bak-версию, если dst был обычным файлом.
_remove_existing() {
    local dst="$1"
    local bak
    if [ -L "$dst" ]; then
        run rm -f "$dst"
    elif [ -e "$dst" ]; then
        bak="${dst}.bak.$(date -u +%Y%m%dT%H%M%SZ)"
        run mv "$dst" "$bak"
        echo "  BAK  $(basename "$dst") (was real file — saved as $bak)"
    fi
}

# _install_one src dst — раскладывает src в dst.
#   - сначала пробуем hardlink (cp -al) — это и есть основной режим;
#   - если cross-device (cp -al падает), делаем обычную копию (cp);
#   - симлинк — крайний случай, и ТОЛЬКО для директорий вне HERMES_SCRIPTS_DIR
#     (для HERMES_SCRIPTS_DIR симлинк ЗАПРЕЩЁН, см. anti-escape guard).
_install_one() {
    local src="$1"
    local dst="$2"
    local name
    name="$(basename "$dst")"

    # Уже hardlink на нужный src — ничего не делаем.
    if [ -f "$dst" ] && [ ! -L "$dst" ] \
        && [ "$(stat -c '%i' "$dst" 2>/dev/null)" = "$(stat -c '%i' "$src" 2>/dev/null)" ]; then
        echo "  OK   $name (already hardlink to src)"
        return 0
    fi

    # Уже regular file с тем же содержимым (cp-fallback от прошлого запуска) — оставляем.
    if [ -f "$dst" ] && [ ! -L "$dst" ] \
        && cmp -s "$dst" "$src" 2>/dev/null; then
        echo "  OK   $name (regular file copy, content matches)"
        return 0
    fi

    _remove_existing "$dst"

    # 1) hardlink (preferred)
    if $DRY_RUN || cp -al "$src" "$dst" 2>/dev/null; then
        [ -L "$dst" ] || [ -f "$dst" ] && echo "  HLINK $name -> $(stat -c '%i' "$dst" 2>/dev/null) inode of $src"
        return 0
    fi

    # 2) regular copy fallback (если hardlink упал — обычно cross-device)
    if cp -a "$src" "$dst" 2>/dev/null; then
        echo "  COPY $name (hardlink unavailable, used copy)"
        return 0
    fi

    # 3) symbolic link — крайний fallback, но ТОЛЬКО если dst не в HERMES_SCRIPTS_DIR.
    local dst_dir
    dst_dir="$(dirname "$dst")"
    if [ "$dst_dir" = "$HERMES_SCRIPTS_DIR" ]; then
        echo "  ERROR $name: cannot install (hardlink/copy failed AND symlink forbidden in $HERMES_SCRIPTS_DIR)" >&2
        return 1
    fi
    ln -sf "$src" "$dst"
    echo "  SLINK $name -> $src (last-resort fallback; NOT $HERMES_SCRIPTS_DIR, so guard not affected)"
}

echo "==> Source of truth: $SCRIPT_DIR"
if [ ! -d "$SCRIPT_DIR" ]; then
    echo "ERROR: source dir not found: $SCRIPT_DIR (clone the repo there?)"
    exit 1
fi

# sanity check — файлы на месте (EXPECTED объявлен в начале файла — единый
# список для раскладки и для agent-flow-drift-detect.sh --list-files)
for f in "${EXPECTED[@]}"; do
    if [ ! -f "$SCRIPT_DIR/$f" ]; then
        echo "ERROR: missing canonical file $SCRIPT_DIR/$f"
        exit 2
    fi
done

for target_dir in "${TARGET_DIRS[@]}"; do
    if [ ! -d "$target_dir" ]; then
        echo "  SKIP $target_dir (dir not present)"
        continue
    fi

    echo "==> $target_dir"
    run mkdir -p "$target_dir"
    for f in "${EXPECTED[@]}"; do
        src="$SCRIPT_DIR/$f"
        dst="$target_dir/$f"

        _install_one "$src" "$dst"
    done
done

echo
echo "==> Anti-escape guard: проверяю, что ни один файл в $HERMES_SCRIPTS_DIR не указывает наружу"
ANTI_ESCAPE_OK=true
for f in "${EXPECTED[@]}"; do
    f_hermes="$HERMES_SCRIPTS_DIR/$f"
    if [ ! -e "$f_hermes" ]; then
        if [ -d "$HERMES_SCRIPTS_DIR" ]; then
            echo "  WARN $f (missing in $HERMES_SCRIPTS_DIR)"
        fi
        continue
    fi
    # В dry-run НЕ трогаем файлы (раскладка ещё не применена) — просто
    # сообщаем, что будет исправлено.
    if $DRY_RUN; then
        if [ -L "$f_hermes" ]; then
            echo "  [DRY-WOULD-FIX] $f (symlink -> $(readlink "$f_hermes")); will become regular file"
            ANTI_ESCAPE_OK=false
        fi
        continue
    fi
    if [ -L "$f_hermes" ]; then
        # symlink в HERMES_SCRIPTS_DIR — гарантированно fail guard-а.
        # cp -L: переходим по симлинку и копируем содержимое. Потом атомарно
        # заменяем симлинк на regular copy (mv удаляет симлинк и переименовывает
        # копию). Так как скрипт работает в режиме set -e, нам нужен controlled
        # shell-scripting: обернём в условный блок.
        echo "  FIX  $f (was symlink in $HERMES_SCRIPTS_DIR -> $(readlink "$f_hermes"); converting to regular copy)"
        if cp -L "$f_hermes" "${f_hermes}.__tmpsym__" 2>/dev/null \
            && rm -f "$f_hermes" \
            && mv "${f_hermes}.__tmpsym__" "$f_hermes"; then
            chmod +x "$f_hermes" 2>/dev/null || true
            ANTI_ESCAPE_OK=false
        else
            # cleanup если что-то пошло не так
            rm -f "${f_hermes}.__tmpsym__" 2>/dev/null || true
            echo "  ERROR $f: failed to convert symlink to regular file" >&2
            ANTI_ESCAPE_OK=false
        fi
    fi
    # final check: readlink -f должен остаться внутри HERMES_SCRIPTS_DIR.
    resolved="$(readlink -f "$f_hermes" 2>/dev/null || true)"
    case "$resolved" in
        "$HERMES_SCRIPTS_DIR"/*)
            ;;
        *)
            if [ "$ANTI_ESCAPE_OK" = "true" ]; then
                echo "  ERROR $f: still resolves outside ($resolved)" >&2
                ANTI_ESCAPE_OK=false
            fi
            ;;
    esac
done
if [ "$ANTI_ESCAPE_OK" = "true" ]; then
    if $DRY_RUN; then
        echo "  OK all files in $HERMES_SCRIPTS_DIR would stay/copy inside it"
    else
        echo "  OK all files in $HERMES_SCRIPTS_DIR resolve inside it"
    fi
else
    if $DRY_RUN; then
        echo "  !! [DRY] some $HERMES_SCRIPTS_DIR files would be auto-healed"
    else
        echo "  !! auto-healed $HERMES_SCRIPTS_DIR files (now regular copies)"
    fi
fi

echo
echo "==> Telegram token sanity (retro 12.08 t_5af222ea): >1 active TELEGRAM_BOT_TOKEN = reconnect loop"
TOKEN_HOLDERS=()
for envf in /home/builder/.hermes/.env /home/builder/.hermes/profiles/*/.env; do
    [ -f "$envf" ] || continue
    while IFS= read -r ln; do
        case "$ln" in
            TELEGRAM_BOT_TOKEN=*)
                val="${ln#TELEGRAM_BOT_TOKEN=}"
                val="${val%\"}"; val="${val#\"}"
                if [ -n "$val" ]; then
                    TOKEN_HOLDERS+=("$(basename "$(dirname "$envf")")")
                fi
                break
                ;;
        esac
    done < "$envf"
done
if [ "${#TOKEN_HOLDERS[@]}" -gt 1 ]; then
    echo "  !! WARNING: ${#TOKEN_HOLDERS[@]} profiles hold an active TELEGRAM_BOT_TOKEN: ${TOKEN_HOLDERS[*]}"
    echo "     Telegram allows ONE getUpdates consumer per token; the rest will loop"
    echo "     'token already in use' forever. Keep the token only in the owner profile."
else
    echo "  OK  telegram token holders: ${TOKEN_HOLDERS[*]:-none}"
fi

echo
echo "==> Done. Verify:"
if ! $DRY_RUN; then
    for f in "${EXPECTED[@]}"; do
        for fp in \
            "/home/builder/.hermes/profiles/agent-flow/scripts/$f" \
            "/home/builder/.hermes/profiles/architect/scripts/$f" \
            "/home/builder/.hermes/profiles/devops/scripts/$f" \
            "/home/builder/.hermes/scripts/$f"; do
            if [ -e "$fp" ]; then
                inode="$(stat -c '%i' "$fp" 2>/dev/null)"
                kind="$( [ -L "$fp" ] && echo symlink || echo "reg(inode=$inode)" )"
                echo "  $fp -> $(readlink -f "$fp"): $(md5sum "$fp" 2>/dev/null | cut -c1-10) [$kind]"
            fi
        done
        echo "  ---"
    done
fi
