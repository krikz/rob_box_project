#!/usr/bin/env bash
# ============================================================================
# remove_dead_mapping_state.sh — убрать мёртвый mapping_state.json (issue #5045).
#
# /maps/mapping_state.json на Main Pi не читается с коммита 4f5baba18
# (2026-03-06): start_rtabmap.sh больше не смотрит в файл, MCP-тулы держат
# состояние в памяти, имя активной карты — строка maps.is_active в БД (#4392).
# Файл остался в томе карт и называет «квартиру», хотя робот стоит в мастерской.
#
# Запускается на Main Pi шагом «[Main Pi] Remove dead mapping_state.json»
# в ".github/workflows/L-Deploy and Verify.yml", ДО Stop Containers.
# Аргумент — каталог тома карт (на роботе ~/rob_box_project/docker/main/maps,
# в контейнере rtabmap это /maps). Каталога нет — нечего чистить, выход 0.
#
# Идемпотентно и узко: удаляет только файл с именем mapping_state.json.
# rtabmap.db и всё остальное в каталоге не трогает. Нет файла — тихий выход 0
# и строка «нет файла» в лог. Есть — удаляет и пишет, что удалил.
# Код возврата: 0 — файла нет (не было или удалён); 1 — удалить не вышло.
#
# Файл на Main Pi принадлежит root:root (том /maps пишет контейнер rtabmap
# от uid 0), а sudo -n на роботе просит пароль. Если обычный rm получает
# Permission denied, удаляем от root через docker, без sudo и без pull:
#   1) docker exec rtabmap rm -f /maps/mapping_state.json — только если
#      контейнер запущен и его mount Destination=/maps указывает Source
#      ровно на этот каталог (docker inspect, не на глаз);
#   2) иначе docker run --rm --pull never -v "$maps_dir:/maps" --entrypoint
#      /bin/rm <уже лежащий локально образ> -f /maps/mapping_state.json.
#      Образ — Config.Image запущенного rtabmap, иначе первая локальная
#      ссылка с «rtabmap» в имени (docker images). Pull с робота не делаем.
# Каталог не chown-им. Симлинк и не-обычный файл — exit 1 без удаления.
# ============================================================================

set -uo pipefail


# Удалить mapping_state.json от root через docker. 0 — файл исчез, 1 — не вышло.
# Аргументы: канонический каталог тома, путь файла на хосте.
rm_via_docker() {
    local canon="$1"
    local host_target="$2"
    local mounts_fmt source dest running image

    if command -v docker >/dev/null 2>&1; then
        # Destination и Source попарно. Сверяем /maps с каноническим путём,
        # не с глазом: HIL монтирует в /maps другой каталог.
        mounts_fmt='{{range .Mounts}}{{println .Destination}}{{println .Source}}{{end}}'
        running="$(docker inspect -f '{{.State.Running}}' rtabmap 2>/dev/null || true)"
        if [ "$running" = "true" ]; then
            source=""
            dest=""
            while IFS= read -r line || [ -n "$line" ]; do
                if [ -z "$dest" ]; then
                    dest="$line"
                    continue
                fi
                if [ "$dest" = "/maps" ]; then
                    source="$line"
                fi
                dest=""
            done < <(docker inspect -f "$mounts_fmt" rtabmap 2>/dev/null || true)
            if [ -n "$source" ] && [ "$(cd "$source" 2>/dev/null && pwd -P)" = "$canon" ]; then
                echo "ℹ️  #5045: rm не смог, удаляю от root: docker exec rtabmap rm -f /maps/mapping_state.json" >&2
                docker exec rtabmap rm -f /maps/mapping_state.json || true
                if [ ! -e "$host_target" ] && [ ! -L "$host_target" ]; then
                    return 0
                fi
                echo "⚠️  #5045: docker exec rtabmap не убрал файл" >&2
            else
                echo "⚠️  #5045: контейнер rtabmap запущен, но его /maps не этот каталог (source=${source:-нет}) — exec не зову" >&2
            fi
        fi

        image="$(docker_rm_image)"
        if [ -n "$image" ]; then
            echo "ℹ️  #5045: удаляю от root одноразовым контейнером ($image), pull не делаю" >&2
            docker run --rm --pull never \
                -v "$canon:/maps" \
                --entrypoint /bin/rm \
                "$image" \
                -f /maps/mapping_state.json || true
            if [ ! -e "$host_target" ] && [ ! -L "$host_target" ]; then
                return 0
            fi
            echo "⚠️  #5045: docker run не убрал файл" >&2
        else
            echo "⚠️  #5045: локального образа для docker run нет — pull с робота не делаю" >&2
        fi
    else
        echo "⚠️  #5045: docker нет в PATH — удалить от root нечем" >&2
    fi
    return 1
}

# Образ, который уже лежит локально. Пустая строка — брать нечего.
docker_rm_image() {
    local image ref
    image="$(docker inspect -f '{{.Config.Image}}' rtabmap 2>/dev/null || true)"
    if [ -n "$image" ] && docker image inspect "$image" >/dev/null 2>&1; then
        printf '%s\n' "$image"
        return 0
    fi
    # Первая локальная ссылка с rtabmap в имени (репозиторий или тег).
    ref="$(docker images --format '{{.Repository}}:{{.Tag}}' 2>/dev/null | grep -m1 'rtabmap' || true)"
    if [ -n "$ref" ] && [ "$ref" != "<none>:<none>" ]; then
        printf '%s\n' "$ref"
        return 0
    fi
    return 0
}


maps_dir="${1:-}"

if [ -z "$maps_dir" ]; then
    echo "::error title=Dead mapping_state.json (#5045)::не задан каталог тома карт" >&2
    exit 1
fi

target="$maps_dir/mapping_state.json"

if [ ! -d "$maps_dir" ]; then
    echo "ℹ️  #5045: каталога $maps_dir нет — чистить нечего"
    exit 0
fi

if [ ! -e "$target" ] && [ ! -L "$target" ]; then
    echo "ℹ️  #5045: $target отсутствует — мёртвый файл уже убран"
    exit 0
fi

if [ -L "$target" ] || [ ! -f "$target" ]; then
    echo "::error title=Dead mapping_state.json (#5045)::$target не обычный файл — не трогаю, разберитесь вручную" >&2
    exit 1
fi

# Канонический путь каталога: bind-mount в inspect пишет Source без хвоста «/».
maps_canon="$(cd "$maps_dir" && pwd -P)"

rm_err="$(mktemp)"
rm -f -- "$target" 2>"$rm_err" || true
if [ -s "$rm_err" ]; then
    echo "⚠️  #5045: rm не удалил $target: $(tr '\n' ' ' <"$rm_err")" >&2
fi
rm -f -- "$rm_err"

# Уже нет — обычный rm справился (или гонка). Дальше только если файл жив.
if [ -e "$target" ] || [ -L "$target" ]; then
    if ! rm_via_docker "$maps_canon" "$target"; then
        echo "::error title=Dead mapping_state.json (#5045)::не удалось удалить $target ни rm, ни через docker (контейнер rtabmap с mount /maps на этот каталог не найден, локального образа для docker run нет или удаление от root не прошло)" >&2
        exit 1
    fi
fi

if [ -e "$target" ] || [ -L "$target" ]; then
    echo "::error title=Dead mapping_state.json (#5045)::$target остался после удаления" >&2
    exit 1
fi

echo "🧹 #5045: удалён мёртвый $target (не читается с 4f5baba18, имя карты — maps.is_active в БД)"
exit 0
