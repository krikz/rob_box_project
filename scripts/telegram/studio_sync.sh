#!/bin/bash
# Студия обновляет свой код сама: fast-forward живого клона до origin/develop.
#
# Зачем (ADR-0172, issue #4703). У студии два источника материала разной
# свежести: факты и медиа берутся из worktree origin/develop (он обновляется
# сам), а ИСПОЛНЯЕМЫЙ КОД — из живого клона /home/builder/hermes-share/
# rob_box_project, который обновлялся ТОЛЬКО вручную. Итог: факты свежие,
# логика старая, ошибок нет — сервис активен, просто считает по старому коду.
# 13.11 сессия в 03:00 UTC отработала кодом от 11.11, хотя merge #4701 был
# в 08:58 того же дня.
#
# Что делает:
#   fetch origin develop → сравнить с HEAD → если отстал: fast-forward
#   (git merge --ff-only) → перезапуск studio-watch (только если код изменился).
#
# Чего НЕ делает и почему:
#   * git stash — stash прячет несохранённую работу и потом требует ручного
#     stash pop: это ровно тот класс «решение есть, а код не доехал»,
#     который здесь и чинится;
#   * git reset --hard поверх несохранённого — затирает чужую работу;
#   * обновление в начале сессии студии — синхронизация отдельным таймером
#     (ADR-0172 §3): ещё одна точка обновления — ещё одно место, где можно
#     забыть перезапустить сервис.
#
# Если в клоне изменён tracked-файл — обновления НЕТ, файл не тронут,
# в журнал пишется причина и возвращается ненулевой код.
# Untracked-файлы студии (docs/channel/queue/*.md, docs/channel/infographics/
# *.json) не трогаются: если develop принёс файл с тем же путём, скрипт
# останавливается и пишет причину, а не удаляет чужое.
#
# Переменные окружения (для тестов и нестандартных раскладок):
#   STUDIO_REPO               — корень живого клона
#   STUDIO_SYNC_REMOTE        — remote (по умолчанию origin)
#   STUDIO_SYNC_BRANCH        — ветка (по умолчанию develop)
#   STUDIO_SYNC_RESTART_CMD   — команда перезапуска studio-watch
#   STUDIO_SYNC_STATE_DIR     — каталог состояния студии (run/)
#   STUDIO_SYNC_LOCK_FILE     — файл блокировки от параллельного запуска
set -euo pipefail

STUDIO_REPO="${STUDIO_REPO:-/home/builder/hermes-share/rob_box_project}"
SYNC_REMOTE="${STUDIO_SYNC_REMOTE:-origin}"
SYNC_BRANCH="${STUDIO_SYNC_BRANCH:-develop}"
RESTART_CMD="${STUDIO_SYNC_RESTART_CMD:-systemctl --user restart studio-watch.service}"
STATE_DIR="${STUDIO_SYNC_STATE_DIR:-$STUDIO_REPO/docker/studio/run}"
LOCK_FILE="${STUDIO_SYNC_LOCK_FILE:-/tmp/studio-sync.lock}"
STATE_FILE="$STATE_DIR/studio_sync.json"

# --------------------------------------------------------------------------
# Самозащита: скрипт обновляет репозиторий, в котором сам лежит.
#
# git при merge переписывает изменённые файлы НА МЕСТЕ (тот же inode), а bash
# читает скрипт по мере исполнения, по смещению в файле. Если merge изменит
# studio_sync.sh, пока он работает, bash продолжит читать новый файл со
# старого смещения и выполнит мусор — синхронизация сломает саму себя.
#
# Поэтому работаем НЕ из файла в репозитории, а из приватной копии во
# временном каталоге: её merge не касается, а удаляется на выходе.
# --------------------------------------------------------------------------
if [ -z "${STUDIO_SYNC_COPY:-}" ]; then
    STUDIO_SYNC_COPY="$(mktemp "${TMPDIR:-/tmp}/studio_sync.XXXXXX.sh")"
    cp -- "${BASH_SOURCE[0]}" "$STUDIO_SYNC_COPY"
    chmod +x "$STUDIO_SYNC_COPY"
    export STUDIO_SYNC_COPY
    # exec: процесс подменяется копией, оригинал не нужен.
    exec bash "$STUDIO_SYNC_COPY" "$@"
fi
# shellcheck disable=SC2329  # вызывается через trap ... EXIT
cleanup() { rm -f -- "$STUDIO_SYNC_COPY"; }
trap cleanup EXIT

log() { printf '[studio-sync] %s\n' "$*"; }

# Путь, из которого реально работает bash. Показываем его в журнал: по нему
# видно, что исполняется приватная копия, а не файл в репозитории (который
# merge переписывает на месте).
log "работаю из копии: $STUDIO_SYNC_COPY"

# Состояние синхронизации: какой коммит запущен и на сколько клон отставал
# от origin/develop ДО и ПОСЛЕ. Пишем честно, включая причину отказа.
#
# JSON собирает python3 (json.dumps), а не heredoc: в reason попадает имя
# изменённого файла, а там могут быть кавычки и переводы строк — ручное
# подставляние в шаблон ломало бы JSON (state-файл тогда не прочитать).
# Значения передаём через окружение: так в них безопасно попадает любой текст.
write_state() {
    local result="$1" commit_before="$2" commit_after="$3"
    local behind_before="$4" behind_after="$5" restarted="$6" reason="$7"

    mkdir -p "$STATE_DIR"
    cat > "$STATE_FILE.tmp.$$" <<EOF
{
  "updated_at": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "result": "$result",
  "commit_before": "$commit_before",
  "commit_after": "$commit_after",
  "behind_before": $behind_before,
  "behind_after": $behind_after,
  "restarted": $restarted,
  "reason": "$reason"
}
EOF
    mv -f "$STATE_FILE.tmp.$$" "$STATE_FILE"
}

if [ ! -d "$STUDIO_REPO/.git" ]; then
    log "FATAL: нет git-репозитория: $STUDIO_REPO"
    exit 1
fi

cd "$STUDIO_REPO"

# flock: две копии одновременно не должны мешать друг другу. Вторая честно
# говорит «уже обновляется» и выходит, не трогая клон.
mkdir -p "$(dirname "$LOCK_FILE")"
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
    log "уже обновляется: другая копия синхронизации работает, пропускаю"
    exit 0
fi

COMMIT_BEFORE="$(git rev-parse HEAD)"
log "клон: $STUDIO_REPO, коммит: ${COMMIT_BEFORE:0:9}"

# fetch: обновляем refs/remotes/$REMOTE/$BRANCH. FETCH_HEAD не годится —
# он перезаписывается следующим fetch и теряет принадлежность к ветке.
if ! git fetch --quiet "$SYNC_REMOTE" "+refs/heads/$SYNC_BRANCH:refs/remotes/$SYNC_REMOTE/$SYNC_BRANCH"; then
    log "FATAL: fetch $SYNC_REMOTE/$SYNC_BRANCH не удался (сеть или remote недоступен)"
    write_state "fetch-failed" "$COMMIT_BEFORE" "$COMMIT_BEFORE" 0 0 false "fetch не удался"
    exit 1
fi

REMOTE_REF="refs/remotes/$SYNC_REMOTE/$SYNC_BRANCH"
# Проверяем, что ветка на remote вообще есть: иначе считать отставание не от чего.
if ! git rev-parse --verify --quiet "$REMOTE_REF" >/dev/null; then
    log "FATAL: нет $REMOTE_REF — ветка $SYNC_BRANCH на remote не найдена"
    write_state "no-remote-branch" "$COMMIT_BEFORE" "$COMMIT_BEFORE" 0 0 false "нет $REMOTE_REF"
    exit 1
fi

# На сколько клон отстал: число коммитов, которых нет в HEAD.
BEHIND_BEFORE="$(git rev-list --count "HEAD..$REMOTE_REF")"

if [ "$BEHIND_BEFORE" -eq 0 ]; then
    log "актуально: отставание 0, перезапуск не нужен"
    write_state "up-to-date" "$COMMIT_BEFORE" "$COMMIT_BEFORE" 0 0 false "отставание 0"
    exit 0
fi

log "отстаёт от $SYNC_REMOTE/$SYNC_BRANCH на $BEHIND_BEFORE коммит(ов)"

# Чужая работа в клоне: изменённые tracked-файлы НЕ затираем и обновление
# не делаем (ADR-0172). Untracked-файлы (материал студии) — не помеха:
# они игнорируются намеренно.
#
# --ignore-submodules=all обязателен. Рабочая копия сабмодуля после
# fast-forward остаётся на старом коммите (merge обновляет указатель, но не
# рабочее дерево сабмодуля), и status начинает показывать " M src/...".
# Без этого флага синхронизация залипала бы навсегда: каждый следующий
# запуск видел бы «изменённый файл» и отказывался обновлять клон. Сабмодули
# студии не нужны (студия живёт на scripts/telegram), поэтому их состояние —
# не повод блокировать обновление кода студии.
DIRTY="$(git status --porcelain --untracked-files=no --ignore-submodules=all)"
if [ -n "$DIRTY" ]; then
    log "изменены tracked-файлы — обновление пропущено, чужую работу не трогаю:"
    printf '%s\n' "$DIRTY" | sed 's/^/[studio-sync]   /'
    write_state "refused-dirty" "$COMMIT_BEFORE" "$COMMIT_BEFORE" "$BEHIND_BEFORE" "$BEHIND_BEFORE" false \
        "изменены tracked-файлы: $(printf '%s' "$DIRTY" | head -n 1)"
    exit 2
fi

# Fast-forward. Если develop принёс файл, который в клоне есть как
# untracked, git сам откажется сливать (untracked-файл would be overwritten)
# и НЕ удалит его — мы только честно пишем причину.
if ! git merge --ff-only "$REMOTE_REF" 2>&1 | sed 's/^/[studio-sync] /'; then
    log "fast-forward не удался: конфликт (например, develop принёс файл с тем же путём, что есть в клоне как untracked)"
    write_state "merge-conflict" "$COMMIT_BEFORE" "$COMMIT_BEFORE" "$BEHIND_BEFORE" "$BEHIND_BEFORE" false \
        "fast-forward не удался: конфликт с локальным untracked-файлом"
    exit 3
fi

COMMIT_AFTER="$(git rev-parse HEAD)"
BEHIND_AFTER="$(git rev-list --count "HEAD..$REMOTE_REF")"
log "обновлено: ${COMMIT_BEFORE:0:9} → ${COMMIT_AFTER:0:9}, отставание теперь $BEHIND_AFTER"

# Перезапуск ТОЛЬКО если код реально изменился: сервис загружает модули
# Python один раз на старте, а длиннополлер студии рвать каждые пять минут
# без причины — тоже плохо.
RESTARTED=false
if [ "$COMMIT_AFTER" != "$COMMIT_BEFORE" ]; then
    log "код изменился — перезапускаю studio-watch"
    if bash -c "$RESTART_CMD" >/dev/null 2>&1; then
        RESTARTED=true
        log "studio-watch перезапущен"
    else
        log "FATAL: не удалось перезапустить studio-watch (команда: $RESTART_CMD)"
        write_state "restart-failed" "$COMMIT_BEFORE" "$COMMIT_AFTER" "$BEHIND_BEFORE" "$BEHIND_AFTER" false \
            "код обновлён, но рестарт studio-watch не удался"
        exit 4
    fi
fi

write_state "updated" "$COMMIT_BEFORE" "$COMMIT_AFTER" "$BEHIND_BEFORE" "$BEHIND_AFTER" "$RESTARTED" "ok"
log "готово: запущен коммит ${COMMIT_AFTER:0:9}"
exit 0
