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
# Идемпотентно и узко: трогаем только имя mapping_state.json. rtabmap.db и
# всё остальное в каталоге не трогаем. Каталог не chown-им.
#
# Файл на Main Pi — root:root (том /maps пишет контейнер rtabmap от uid 0),
# а sudo -n там просит пароль (run 44948634286: rm → Permission denied, шаг
# упал и пропустил Stop/Start). Мёртвый файл не критичен для деплоя, но и
# молча глотать отказ нельзя: отсутствие проверяем и печатаем честно.
# Код возврата всегда 0. Ветки:
#   - нет каталога / нет файла — ℹ️, exit 0;
#   - симлинк или не обычный файл — не удаляем, ⚠️, exit 0;
#   - обычный файл, по порядку, пока не исчезнет:
#       (a) rm -f --
#       (b) sudo -n rm -f --  (пароль не спрашиваем; на Main Pi sudo -n
#           отвечает «a password is required», на стенде с NOPASSWD пройдёт)
#       (c) docker exec rtabmap rm -f /maps/mapping_state.json — только если
#           контейнер запущен и inspect показывает Destination=/maps с Source
#           ровно на этот каталог (не на глаз: HIL монтирует другой каталог);
#       (d) иначе docker run --rm --pull never -v "$dir:/maps" --entrypoint
#           /bin/rm <образ из локального кэша> -f /maps/mapping_state.json.
#           Образ — Config.Image контейнера rtabmap, если он уже локально,
#           иначе первая ссылка docker images с «rtabmap» в имени. Pull нет.
#   - после попыток проверка [ ! -e ]: исчез — 🧹 «удалён» и exit 0;
#     остался — ⚠️ «файл остался» и exit 0.
# ============================================================================

set -uo pipefail

# Удалить mapping_state.json от root через docker. Ничего не возвращаем:
# успех смотрит вызывающий по отсутствию файла на хосте. Pull не делаем.
# Аргументы: канонический каталог тома, путь файла на хосте.
rm_via_docker() {
    local canon="$1"
    local host_target="$2"
    local mounts_fmt source dest running image

    if ! command -v docker >/dev/null 2>&1; then
        echo "⚠️  #5045: docker нет в PATH — удалить от root через контейнер нечем" >&2
        return 0
    fi

    # Destination и Source попарно. Сверяем /maps с каноническим путём.
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
            echo "ℹ️  #5045: удаляю от root: docker exec rtabmap rm -f /maps/mapping_state.json" >&2
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
    if [ -z "$image" ]; then
        echo "⚠️  #5045: локального образа для docker run нет — pull с робота не делаю" >&2
        return 0
    fi
    echo "ℹ️  #5045: удаляю от root одноразовым контейнером ($image), pull не делаю" >&2
    # --entrypoint /bin/rm: образ rtabmap своего ENTRYPOINT не задаёт, но чужой
    # локальный тег с тем же именем может. --pull never: сеть робота не трогаем.
    docker run --rm --pull never \
        -v "$canon:/maps" \
        --entrypoint /bin/rm \
        "$image" \
        -f /maps/mapping_state.json || true
    if [ -e "$host_target" ] || [ -L "$host_target" ]; then
        echo "⚠️  #5045: docker run не убрал файл" >&2
    fi
    return 0
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
    fi
    return 0
}

# Файл ещё на месте (обычный или симлинк, который rm не съел).
target_remains() {
    [ -e "$target" ] || [ -L "$target" ]
}

maps_dir="${1:-}"

if [ -z "$maps_dir" ]; then
    # Шаг деплоя не должен падать из-за мёртвого файла. Пустой аргумент —
    # ошибка вызова, но честно печатаем и выходим 0: Stop Containers пойдёт.
    echo "⚠️  #5045: не задан каталог тома карт — чистку пропускаю, деплой не останавливаю" >&2
    exit 0
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
    echo "⚠️  #5045: $target не обычный файл (симлинк или каталог) — не удаляю, разберитесь вручную" >&2
    exit 0
fi

# Канонический путь каталога: bind-mount в inspect пишет Source без хвоста «/».
maps_canon="$(cd "$maps_dir" && pwd -P)"

rm_err="$(mktemp)"
rm -f -- "$target" 2>"$rm_err" || true
if target_remains && [ -s "$rm_err" ]; then
    echo "⚠️  #5045: rm не удалил $target: $(tr '\n' ' ' <"$rm_err")" >&2
fi
rm -f -- "$rm_err"

# (b) sudo -n: пароль не спрашиваем. На Main Pi ответит «a password is
# required»; на стенде с NOPASSWD снимет root-файл дёшево, без docker.
if target_remains && command -v sudo >/dev/null 2>&1; then
    sudo_err="$(mktemp)"
    sudo -n rm -f -- "$target" 2>"$sudo_err" || true
    if target_remains; then
        echo "⚠️  #5045: sudo -n rm не удалил $target: $(tr '\n' ' ' <"$sudo_err")" >&2
    fi
    rm -f -- "$sudo_err"
fi

if target_remains; then
    rm_via_docker "$maps_canon" "$target"
fi

# Проверка отсутствия обязательна и в успехе, и в отказе.
if target_remains; then
    echo "⚠️  #5045: файл остался: $target (не читается с 4f5baba18; rm, sudo -n и docker не убрали — деплой не останавливаю)" >&2
    exit 0
fi

echo "🧹 #5045: удалён мёртвый $target (не читается с 4f5baba18, имя карты — maps.is_active в БД)"
exit 0
