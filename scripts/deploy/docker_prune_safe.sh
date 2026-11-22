#!/usr/bin/env bash
# ============================================================================
# docker_prune_safe.sh — безопасная уборка образов на борту (issue #5089).
#
# Зачем: шаг «Cleanup Docker Artifacts» в
# ".github/workflows/L-Deploy and Verify.yml" делал `docker image prune -af`.
# Деплой №8 (run 44991405421, job 137113050253, 2026-11-22T05:59:15–17Z)
# снял этим вызовом локальные теги `*-dev-acd5a8d` у 8 сервисов Main Pi,
# хотя их контейнеры были запущены и Config.Image ссылался на эти теги
# (Total reclaimed space: 0B). Следующий
# `docker compose up -d --no-deps --pull never --force-recreate` упал:
# «No such image: .../nav2-humble-dev-acd5a8d». Образ в registry был жив,
# локально тот же sha лежал под старым тегом — пропало именно имя.
#
# Почему prune посчитал in-use тег unused, не установлено (recreate на
# живом роботе запрещён). Скрипт делает уборку безопасной независимо от
# причины: тег, на который ссылается ЛЮБОЙ контейнер (running или stopped),
# не снимается. Мусор — образы, на которые не ссылается ни один контейнер,
# — по-прежнему удаляется, иначе шаг теряет смысл.
#
# После уборки — детерминированная проверка: каждый образ из
# `docker compose config --images` должен проходить `docker image inspect`.
# Если тег пропал, а контейнер с Config.Image == ref ещё существует,
# тег восстанавливается `docker tag <image id контейнера> <ref>` без сети
# и без ручных действий. WARN пишется в лог с именем сервиса (если compose
# его назвал) или с самим ref.
#
# Best-effort: любой сбой docker — предупреждение, код возврата всегда 0.
# Деплой из-за уборки не падает (так же, как `|| echo warning` в шаге).
#
# Использование (на борту, из каталога compose-проекта):
#   bash scripts/deploy/docker_prune_safe.sh
#   bash scripts/deploy/docker_prune_safe.sh --dry-run
# --dry-run печатает, что было бы удалено и восстановлено, и ничего не меняет.
# ============================================================================

set -uo pipefail

dry_run=0
for arg in "$@"; do
    case "$arg" in
        --dry-run) dry_run=1 ;;
        -h|--help)
            sed -n '2,40p' "$0"
            exit 0
            ;;
        *)
            echo "WARN: неизвестный аргумент: $arg (известен только --dry-run)"
            ;;
    esac
done

warn() { echo "WARN: $*"; }

# docker может быть недоступен (нет демона, оборванный ssh). Уборка —
# best-effort: предупреждаем и выходим 0, деплой не валим.
if ! command -v docker >/dev/null 2>&1; then
    warn "docker не найден в PATH — уборка пропущена"
    exit 0
fi

# --- in-use: ID образов, на которые ссылается любой контейнер -------------
# Источник — `docker inspect -f '{{.Image}}'` по каждому контейнеру из
# `docker ps -aq` (running и stopped). `.Image` — ID образа контейнера,
# тот же, что `docker image inspect` вернёт для тега из Config.Image.
# Пустой/битый ответ не считаем «никто не использует» молча: предупреждаем.
in_use_file="$(mktemp)"
ps_rc=0
container_ids="$(docker ps -aq 2>/dev/null)" || ps_rc=$?
if [ "$ps_rc" -ne 0 ]; then
    warn "docker ps -aq не удался (rc=$ps_rc) — уборка пропущена, образы не трогаем"
    rm -f "$in_use_file"
    exit 0
fi

while IFS= read -r cid; do
    [ -n "$cid" ] || continue
    image_id="$(docker inspect -f '{{.Image}}' "$cid" 2>/dev/null)" || {
        warn "не прочитал Image контейнера $cid — уборка пропущена, образы не трогаем"
        rm -f "$in_use_file"
        exit 0
    }
    [ -n "$image_id" ] || {
        warn "пустой Image у контейнера $cid — уборка пропущена, образы не трогаем"
        rm -f "$in_use_file"
        exit 0
    }
    printf '%s\n' "$image_id" >> "$in_use_file"
done < <(printf '%s\n' "$container_ids")

in_use() { grep -qxF -- "$1" "$in_use_file"; }

# --- удаление: только образы, на которые не ссылается ни один контейнер ---
# `docker images -aq` даёт ID, включая dangling (<none>). Один ID может
# встречаться несколько раз (по тегу) — удаляем один раз. `docker rmi` по
# ID снимает ВСЕ теги образа; раз ни один контейнер на него не ссылается,
# терять нечего. In-use ID не передаём в rmi вообще.
unused_file="$(mktemp)"
images_rc=0
all_ids="$(docker images -aq 2>/dev/null)" || images_rc=$?
if [ "$images_rc" -ne 0 ]; then
    warn "docker images -aq не удался (rc=$images_rc) — удаление пропущено"
else
    while IFS= read -r iid; do
        [ -n "$iid" ] || continue
        in_use "$iid" && continue
        grep -qxF -- "$iid" "$unused_file" && continue
        printf '%s\n' "$iid" >> "$unused_file"
    done < <(printf '%s\n' "$all_ids")

    if [ -s "$unused_file" ]; then
        if [ "$dry_run" -eq 1 ]; then
            echo "DRY-RUN: удалил бы образы без контейнеров:"
            sed 's/^/  /' "$unused_file"
        else
            while IFS= read -r iid; do
                [ -n "$iid" ] || continue
                if docker rmi "$iid" >/dev/null 2>&1; then
                    echo "removed unused image $iid"
                else
                    warn "не удалил неиспользуемый образ $iid (возможно, его держит build cache — это не тег контейнера)"
                fi
            done < "$unused_file"
        fi
    else
        echo "unused images: none"
    fi
fi

# --- проверка и восстановление тегов, на которые ссылаются контейнеры -----
# Config.Image — то имя, с которым контейнер создан (`compose up` его и
# потребует). Если `docker image inspect <ref>` не проходит, а контейнер
# с этим Config.Image ещё есть — тег пропал, образ жив под ID контейнера.
# Восстанавливаем `docker tag <id> <ref>`: сеть не нужна, digest тот же.
restore_tags() {
    local cid ref svc image_id
    while IFS= read -r cid; do
        [ -n "$cid" ] || continue
        ref="$(docker inspect -f '{{.Config.Image}}' "$cid" 2>/dev/null)" || {
            warn "не прочитал Config.Image контейнера $cid — проверку тега пропускаю"
            continue
        }
        [ -n "$ref" ] || continue
        # digest-ссылка (repo@sha256:...) — это не тег, rmi её не «теряет»
        # отдельно от ID; восстанавливать нечего.
        case "$ref" in
            *@sha256:*) continue ;;
        esac
        if docker image inspect "$ref" >/dev/null 2>&1; then
            continue
        fi
        svc="$(docker inspect -f '{{index .Config.Labels "com.docker.compose.service"}}' "$cid" 2>/dev/null || true)"
        image_id="$(docker inspect -f '{{.Image}}' "$cid" 2>/dev/null || true)"
        if [ -z "$image_id" ]; then
            warn "тег пропал: ${svc:-$ref} ($ref), но Image контейнера $cid пуст — восстановить нечем"
            continue
        fi
        if [ "$dry_run" -eq 1 ]; then
            echo "DRY-RUN: восстановил бы тег ${svc:-$ref}: docker tag $image_id $ref"
            continue
        fi
        if docker tag "$image_id" "$ref" 2>/dev/null; then
            echo "WARN: тег пропал и восстановлен из контейнера: сервис=${svc:-unknown} ref=$ref image=$image_id"
        else
            warn "тег пропал: сервис=${svc:-unknown} ref=$ref — docker tag $image_id не удался"
        fi
    done < <(printf '%s\n' "$container_ids")
}

restore_tags

# Образы активного compose-проекта (если скрипт запущен из его каталога).
# Вне каталога compose `config --images` падает — это нормально для вызова
# «просто почисти хост», предупреждаем и идём дальше.
if docker compose config --images >/dev/null 2>&1; then
    images="$(docker compose config --images 2>/dev/null || true)"
    while IFS= read -r ref; do
        [ -n "$ref" ] || continue
        if docker image inspect "$ref" >/dev/null 2>&1; then
            continue
        fi
        # Восстановление по контейнеру уже сделано выше. Если тега всё ещё
        # нет — контейнера с таким Config.Image не было (или tag не прошёл).
        # Сеть не дёргаем: деплой уже отработал, pull — чужой шаг.
        warn "compose-образ без локального тега после уборки: $ref (контейнера с Config.Image=$ref нет — не восстанавливаю, сеть не трогаю)"
    done < <(printf '%s\n' "$images" | sort -u)
fi

rm -f "$in_use_file" "$unused_file"
exit 0
