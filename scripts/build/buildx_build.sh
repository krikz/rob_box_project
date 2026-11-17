#!/usr/bin/env bash
# buildx_build.sh — ЕДИНСТВЕННОЕ место, где собирается docker-образ rob_box.
#
# Кто зовёт:
#   * .github/actions/l-build-service/action.yml — все L-Build workflow
#     (Main/Vision Pi Services, Single Service, Base Images);
#   * scripts/build/build.sh — локальная сборка человеком/агентом.
# Т.е. CI и локальная сборка исполняют один и тот же код, а не две копии,
# которые разъезжаются (run 36351952819: L-Build Base Images жил на своём
# inline `docker buildx build`, и его уронил билдер, заведённый композитом).
#
# Вход — переменные окружения (так их удобно пробрасывать из `env:` composite
# action'а без подстановки `${{ }}` внутрь bash):
#
#   BUILD_SERVICE_NAME    имя для логов и для cache-ref (<name>-buildcache)
#   BUILD_DOCKERFILE      путь к Dockerfile (обязателен)
#   BUILD_CONTEXT         build context (по умолчанию ".")
#   BUILD_PLATFORM        buildx --platform (по умолчанию linux/arm64)
#   BUILD_TAGS            теги, по одному на строку
#   BUILD_ARGS            KEY=VAL, по одному на строку
#   BUILD_SUBMODULE_SHA   путь субмодуля → build-arg <BASENAME>_SHA=<sha>
#   BUILD_LOAD_ONLY       true → только --load, без docker push (по умолч. false)
#   BUILD_LOCAL_REGISTRY  registry, в который пушим (по умолч. localhost:5000)
#   BUILD_ADD_HOST        --add-host (по умолч. host.docker.internal:host-gateway)
#   BUILD_CACHE           false → без --cache-from/--cache-to (по умолч. true)
#   BUILD_CACHE_REF       явный ref registry-кеша (иначе выводится из тегов)
#   BUILD_NO_CACHE        true → buildx --no-cache (force rebuild)
#   BUILD_PROGRESS        buildx --progress (по умолчанию plain)
#   BUILD_BUILDER         имя buildx-билдера (по умолч. robbox-<hostname>);
#                         "current" → не заводить свой, взять текущий
#
# Почему билдер передаётся через --builder, а НЕ `docker buildx use`:
# `use` пишет «текущий билдер» в ~/.docker раннер-контейнера, и КАЖДЫЙ
# следующий голый `docker buildx build` на этом раннере молча уезжал на
# docker-container драйвер, где host-gateway не поддерживается:
#   ERROR: unable to derive the IP value for host-gateway:
#          host-gateway is not supported by the docker-container driver
# (run 36351952819 — все три base-job'а упали за 0 секунд). Явный --builder
# не оставляет за собой глобального состояния ни на раннере, ни у разработчика.
set -euo pipefail

SERVICE_NAME="${BUILD_SERVICE_NAME:-service}"
DOCKERFILE="${BUILD_DOCKERFILE:?BUILD_DOCKERFILE is required}"
CONTEXT="${BUILD_CONTEXT:-.}"
PLATFORM="${BUILD_PLATFORM:-linux/arm64}"
TAGS="${BUILD_TAGS:-}"
ARGS="${BUILD_ARGS:-}"
SUBMODULE="${BUILD_SUBMODULE_SHA:-}"
LOAD_ONLY="${BUILD_LOAD_ONLY:-false}"
LOCAL_REGISTRY="${BUILD_LOCAL_REGISTRY:-localhost:5000}"
ADD_HOST_VALUE="${BUILD_ADD_HOST-host.docker.internal:host-gateway}"
CACHE="${BUILD_CACHE:-true}"
CACHE_REF="${BUILD_CACHE_REF:-}"
NO_CACHE="${BUILD_NO_CACHE:-false}"
PROGRESS="${BUILD_PROGRESS:-plain}"
BUILDER_NAME="${BUILD_BUILDER:-robbox-$(hostname)}"

# Multi-line входы — через временные файлы и `read` построчно: переживает
# CRLF и хвостовые переводы строк, которые GitHub Actions оставляет в
# многострочных input'ах.
TAGS_TMP="$(mktemp)"
BUILD_ARGS_TMP="$(mktemp)"
trap 'rm -f "$TAGS_TMP" "$BUILD_ARGS_TMP"' EXIT
printf '%s\n' "$TAGS" | tr -d '\r' > "$TAGS_TMP"
printf '%s\n' "$ARGS" | tr -d '\r' > "$BUILD_ARGS_TMP"

TAG_ARGS=()
while IFS= read -r tag || [ -n "$tag" ]; do
  [ -z "$tag" ] && continue
  TAG_ARGS+=(--tag "$tag")
done < "$TAGS_TMP"

BUILD_ARG_ARGS=()
while IFS= read -r arg || [ -n "$arg" ]; do
  [ -z "$arg" ] && continue
  BUILD_ARG_ARGS+=(--build-arg="$arg")
done < "$BUILD_ARGS_TMP"

# SHA субмодуля как build-arg: src/ros2leds → ROS2LEDS_SHA, src/vesc_nexus →
# VESC_NEXUS_SHA (имена согласованы с Dockerfile'ами).
if [ -n "$SUBMODULE" ]; then
  SUBMODULE_BASENAME="$(basename "$SUBMODULE")"
  SHA_VAR="${SUBMODULE_BASENAME^^}_SHA"
  SHA_VAL="$(git submodule status "$SUBMODULE" | awk '{print $1}' | sed 's/^[+-]//')"
  echo "  $SHA_VAR: $SHA_VAL"
  BUILD_ARG_ARGS+=(--build-arg="${SHA_VAR}=${SHA_VAL}")
fi

# ---- buildx-билдер ----------------------------------------------------------
# Раннеры на katana — контейнеры (myoung34/github-runner, docker.sock с хоста),
# у каждого свой ~/.docker: билдер, созданный руками на хосте, им не виден, а
# созданный внутри контейнера теряется при его пересоздании. Поэтому билдер
# заводится здесь, при каждой сборке, если его ещё нет.
#
# network=host: buildkit в отдельном контейнере иначе не видит ни
# localhost:5000 (кеш и FROM на базовые образы), ни apt-прокси.
# buildkitd.toml с http=true: локальный registry без TLS.
# Имя завязано на hostname: у каждого раннер-контейнера свой билдер, два
# параллельных job'а не дерутся за один buildkit.
BUILDER_ARGS=()
if [ "$BUILDER_NAME" != "current" ]; then
  if ! docker buildx inspect "$BUILDER_NAME" >/dev/null 2>&1; then
    BUILDKITD_TOML="$(mktemp)"
    {
      printf '[registry."%s"]\n' "$LOCAL_REGISTRY"
      printf '  http = true\n'
      printf '  insecure = true\n'
    } > "$BUILDKITD_TOML"
    echo "  buildx: создаю билдер $BUILDER_NAME (docker-container, network=host)"
    docker buildx create \
      --name "$BUILDER_NAME" \
      --driver docker-container \
      --driver-opt network=host \
      --config "$BUILDKITD_TOML" \
      --bootstrap >/dev/null 2>&1 \
      || echo "  ⚠️ buildx: не удалось создать $BUILDER_NAME — остаёмся на текущем билдере"
    rm -f "$BUILDKITD_TOML"
  fi
  if docker buildx inspect "$BUILDER_NAME" >/dev/null 2>&1; then
    BUILDER_ARGS=(--builder "$BUILDER_NAME")
  fi
fi

# Драйвер спрашиваем у ТОГО билдера, на котором реально соберём.
BUILDER_DRIVER="$(docker buildx inspect "${BUILDER_ARGS[@]:1}" 2>/dev/null \
  | awk -F':[[:space:]]*' '/^Driver:/ {print $2; exit}' || true)"
echo "  buildx: builder=${BUILDER_ARGS[1]:-<current>} driver=${BUILDER_DRIVER:-unknown}"

# host-gateway — фича docker-демона; драйверы кроме `docker` её не понимают.
# Подменяем на реальный IP шлюза bridge-сети. Делается ВСЕГДА, а не только при
# включённом кеше: драйвер от кеша не зависит (раньше при cache=false подмены
# не было, и сборка падала ровно так же, как в run 36351952819).
case "${BUILDER_DRIVER}:${ADD_HOST_VALUE}" in
  docker:*|*:) : ;;
  *:*host-gateway)
    GW_IP="$(docker network inspect bridge \
      --format '{{ (index .IPAM.Config 0).Gateway }}' 2>/dev/null || true)"
    if [ -n "$GW_IP" ]; then
      ADD_HOST_VALUE="${ADD_HOST_VALUE%:host-gateway}:${GW_IP}"
      echo "  add-host: host-gateway → ${GW_IP} (драйвер ${BUILDER_DRIVER:-unknown} не умеет host-gateway)"
    else
      echo "  ⚠️ add-host: не удалось вычислить IP шлюза bridge — оставляю host-gateway как есть"
    fi
    ;;
esac
ADD_HOST_ARGS=()
[ -n "$ADD_HOST_VALUE" ] && ADD_HOST_ARGS=(--add-host="${ADD_HOST_VALUE}")

# ---- registry-кеш (docs/plans/2026-09-15-builder-runtime-seam.md §6) --------
CACHE_ARGS=()
if [ "$CACHE" = "false" ]; then
  CACHE_REF=""
  echo "  Cache: disabled (cache=false)"
elif [ -z "$CACHE_REF" ]; then
  # Дефолт выводим из первого тега локального registry:
  # localhost:5000/krikz/rob_box:oak-d-humble-test → localhost:5000/krikz/rob_box:oak-d-buildcache.
  # Отдельный тег — кеш-манифест не затирает образ (§6.4).
  while IFS= read -r tag || [ -n "$tag" ]; do
    [ -z "$tag" ] && continue
    case "$tag" in
      "$LOCAL_REGISTRY"/*)
        # Отрезаем :<tag>, только если после последнего ':' нет '/' (иначе
        # это порт registry у тега без :tag-части — кеш не выводим).
        case "${tag##*:}" in
          */*) : ;;
          *) CACHE_REF="${tag%:*}:${SERVICE_NAME}-buildcache" ;;
        esac
        break
        ;;
    esac
  done < "$TAGS_TMP"
  if [ -z "$CACHE_REF" ]; then
    echo "  ⚠️ Cache: disabled — в tags нет тега ${LOCAL_REGISTRY}/<repo>:<tag>," \
         "не из чего вывести cache-ref (передайте cache-ref явно)"
  fi
fi

if [ -n "$CACHE_REF" ]; then
  # --cache-from на несуществующий ref безопасен: buildx печатает ошибку
  # импорта и продолжает сборку (первый прогон без кеша не падает).
  CACHE_ARGS+=(--cache-from "type=registry,ref=${CACHE_REF}")

  # --cache-to: драйвер `docker` без containerd image store отказывает ещё до
  # старта сборки ("Cache export is not supported for the docker driver"), и
  # ignore-error=true от этого не спасает — поэтому смотрим на драйвер.
  # ignore-error=true спасает от лежащего registry. mode=max — чтобы в кеш
  # попадали промежуточные стадии multi-stage (§6.3).
  if [ -n "$BUILDER_DRIVER" ] && [ "$BUILDER_DRIVER" != "docker" ]; then
    CACHE_EXPORT_OK="true"
  elif docker info --format '{{ .DriverStatus }}' 2>/dev/null | grep -q 'io.containerd.snapshotter'; then
    CACHE_EXPORT_OK="true"
  else
    CACHE_EXPORT_OK="false"
  fi

  if [ "$CACHE_EXPORT_OK" = "true" ]; then
    CACHE_ARGS+=(--cache-to "type=registry,ref=${CACHE_REF},mode=max,ignore-error=true")
    echo "  Cache: ${CACHE_REF} (from + to, mode=max, ignore-error, driver=${BUILDER_DRIVER:-unknown})"
  else
    echo "  Cache: ${CACHE_REF} (from only)"
    echo "  ⚠️ cache export пропущен: buildx-драйвер '${BUILDER_DRIVER:-unknown}' не умеет" \
         "--cache-to type=registry. Чтобы кеш реально писался, нужен драйвер" \
         "docker-container (его заводит этот скрипт, см. BUILD_BUILDER) или" \
         "containerd image store."
  fi
fi

NO_CACHE_ARGS=()
if [ "$NO_CACHE" = "true" ]; then
  NO_CACHE_ARGS=(--no-cache)
  echo "  --no-cache (force rebuild)"
fi

echo "🏗️ Building ${SERVICE_NAME}..."
echo "  Dockerfile: ${DOCKERFILE}"
echo "  Build context: ${CONTEXT}"
echo "  Tags:"
while IFS= read -r tag || [ -n "$tag" ]; do
  [ -z "$tag" ] && continue
  echo "    - $tag"
done < "$TAGS_TMP"
if [ "$LOAD_ONLY" = "true" ]; then
  echo "  Output mode: --load (no push to registry)"
else
  echo "  Output mode: --load + docker push (LOCAL registry only)"
fi

# buildx --load (все теги в локальный daemon — update-image-versions делает
# `docker tag` из daemon) + docker push ТОЛЬКО тегов локального registry.
# buildx --push НЕ используем: он пушил бы и GHCR-теги, а раннер не залогинен в
# ghcr.io для test/dev сборок (issue #1503, run #34368750126 — unauthorized).
docker buildx build \
  "${BUILDER_ARGS[@]}" \
  --platform "$PLATFORM" \
  --file "$DOCKERFILE" \
  "${TAG_ARGS[@]}" \
  --load \
  "${ADD_HOST_ARGS[@]}" \
  "${BUILD_ARG_ARGS[@]}" \
  "${CACHE_ARGS[@]}" \
  "${NO_CACHE_ARGS[@]}" \
  --progress="$PROGRESS" \
  "$CONTEXT"

if [ "$LOAD_ONLY" != "true" ]; then
  echo "  Pushing LOCAL tags (${LOCAL_REGISTRY}/*):"
  while IFS= read -r tag || [ -n "$tag" ]; do
    [ -z "$tag" ] && continue
    case "$tag" in
      "$LOCAL_REGISTRY"/*)
        echo "    docker push $tag"
        docker push "$tag"
        ;;
    esac
  done < "$TAGS_TMP"
fi

echo "✅ Built ${SERVICE_NAME}"
