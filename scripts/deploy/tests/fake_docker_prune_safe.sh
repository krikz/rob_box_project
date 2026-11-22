#!/usr/bin/env bash
# Фейковый docker для test_docker_prune_safe.sh. Не демон: состояние — файлы
# $FAKE_STATE (images.tsv, containers.tsv, rmi.log, tag.log). Вызывается
# обёрткой $WORK/bin/docker, которая умеет ронять `docker ps`.
state="${FAKE_STATE:?FAKE_STATE is required}"
images="$state/images.tsv"
containers="$state/containers.tsv"

if [ "${1:-}" = "ps" ]; then
    awk -F '\t' '{ print $1 }' "$containers"
    exit 0
fi

if [ "${1:-}" = "inspect" ]; then
    fmt=""
    cid=""
    while [ $# -gt 0 ]; do
        case "$1" in
            -f) fmt="$2"; shift 2 ;;
            inspect) shift ;;
            *) cid="$1"; shift ;;
        esac
    done
    line="$(awk -F '\t' -v cid="$cid" '$1 == cid { print; exit }' "$containers")"
    if [ -z "$line" ]; then
        echo "Error: No such object: $cid" >&2
        exit 1
    fi
    image_id="$(printf '%s' "$line" | awk -F '\t' '{ print $2 }')"
    config_image="$(printf '%s' "$line" | awk -F '\t' '{ print $3 }')"
    service="$(printf '%s' "$line" | awk -F '\t' '{ print $4 }')"
    case "$fmt" in
        '{{.Image}}') printf '%s\n' "$image_id" ;;
        '{{.Config.Image}}') printf '%s\n' "$config_image" ;;
        *com.docker.compose.service*) printf '%s\n' "$service" ;;
        *) echo "fake docker: unknown inspect format: $fmt" >&2; exit 1 ;;
    esac
    exit 0
fi

if [ "${1:-}" = "images" ]; then
    # Живой docker 29.8.2 (замерено, только чтение):
    #   docker images -aq            → 12 hex без префикса
    #   docker images -aq --no-trunc → sha256: + 64 hex
    # inspect .Image печатает тот же sha256:+64hex. Точное сравнение
    # короткого с полным никогда не сходится — фейк обязан врать так же,
    # иначе тест (а) зелёный на скрипте, который зовёт rmi на in-use.
    no_trunc=0
    shift
    while [ $# -gt 0 ]; do
        case "$1" in
            --no-trunc) no_trunc=1 ;;
        esac
        shift
    done
    if [ "$no_trunc" -eq 1 ]; then
        awk -F '\t' '{ print $1 }' "$images"
    else
        awk -F '\t' '{ id=$1; sub(/^sha256:/, "", id); print substr(id, 1, 12) }' "$images"
    fi
    exit 0
fi

# Канонический ID: sha256: + 64 hex. Короткий (12 hex) и полный — один образ.
canonical_id() {
    local id="$1"
    id="${id#sha256:}"
    printf 'sha256:%s' "$id"
}

if [ "${1:-}" = "image" ] && [ "${2:-}" = "inspect" ]; then
    ref="${3:-}"
    if awk -F '\t' -v ref="$ref" '$2 == ref { found=1 } END { exit !found }' "$images"; then
        exit 0
    fi
    echo "Error: No such image: $ref" >&2
    exit 1
fi

if [ "${1:-}" = "rmi" ]; then
    id="${2:-}"
    full="$(canonical_id "$id")"
    # Лог — до отказа демона, ровно переданный аргумент. Короткий вызов
    # (images -q без --no-trunc) на in-use виден как 12 hex и не прячется
    # за «демон отказал, тег жив».
    printf '%s\n' "$id" >> "$state/rmi.log"
    if ! awk -F '\t' -v id="$full" '$1 == id { found=1 } END { exit !found }' "$images"; then
        echo "Error: No such image: $id" >&2
        exit 1
    fi
    # Контейнер держит образ — как живой daemon без -f, rmi отказывает.
    # Сравнение по каноническому ID: короткий аргумент тоже отказ.
    if awk -F '\t' -v id="$full" '$2 == id { found=1 } END { exit !found }' "$containers"; then
        echo "Error: image is being used by a container" >&2
        exit 1
    fi
    awk -F '\t' -v id="$full" '$1 != id' "$images" > "$images.tmp"
    mv "$images.tmp" "$images"
    exit 0
fi

if [ "${1:-}" = "tag" ]; then
    id="${2:-}"
    ref="${3:-}"
    full="$(canonical_id "$id")"
    if ! awk -F '\t' -v id="$full" '$1 == id { found=1 } END { exit !found }' "$images"; then
        echo "Error: No such image: $id" >&2
        exit 1
    fi
    id="$full"
    # Тег уже есть на другом ID — снимаем его оттуда (как живой daemon) и
    # вешаем на id контейнера.
    awk -F '\t' -v ref="$ref" '$2 != ref' "$images" > "$images.tmp"
    mv "$images.tmp" "$images"
    printf '%s\t%s\n' "$id" "$ref" >> "$images"
    printf '%s %s\n' "$id" "$ref" >> "$state/tag.log"
    exit 0
fi

if [ "${1:-}" = "compose" ] && [ "${2:-}" = "config" ]; then
    if [ "${FAKE_COMPOSE_RC:-0}" -ne 0 ]; then
        echo "no compose file" >&2
        exit 1
    fi
    if [ "${3:-}" = "--images" ]; then
        printf '%s\n' "${FAKE_COMPOSE_IMAGES:-}"
    fi
    exit 0
fi

echo "fake docker: unexpected invocation: $*" >&2
exit 1
