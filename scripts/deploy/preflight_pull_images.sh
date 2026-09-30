#!/usr/bin/env bash
# ============================================================================
# preflight_pull_images.sh — «сначала образы, потом остановка» (issue #2930).
#
# Запускается на Pi из каталога compose-проекта (docker/main или docker/vision)
# шагом «[… Pi] Pull Docker Images» в ".github/workflows/L-Deploy and Verify.yml",
# ДО шага «Stop Containers». Если скрипт вернул не 0 — шаг падает, Stop/Start
# не выполняются, работающий стек остаётся как был.
#
# 24.09 деплой сначала сделал `compose down`, потом `compose pull
# --ignore-pull-failures` (ошибки манифестов проглочены), потом
# `up --pull never` упал на «No such image» — стек Main Pi лежал часами.
#
# Что делает:
#   1. `docker compose config --quiet` — compose-файл валиден.
#   2. Если REGISTRY_SOURCE != skip: `docker compose pull --policy always`
#      БЕЗ --ignore-pull-failures — любой не скачанный образ валит шаг.
#      (--policy always обязателен: pull_policy: if_not_present у vision,
#      иначе голый pull молча оставляет старый :dev — issue #2609.)
#   3. Каждый образ из `docker compose config --images` (активные профили)
#      должен лежать локально — ровно то, что потом потребует
#      `docker compose up --pull never`. Для REGISTRY_SOURCE=skip это
#      единственная проверка.
#
# Окружение: IMAGE_TAG, SERVICE_IMAGE_PREFIX и *_TAG уже экспортированы
# вызывающим (как в шаге Start Containers). REGISTRY_SOURCE: github|local|skip.
# Код возврата: 0 — все образы на месте; 1 — нельзя останавливать стек.
# ============================================================================

set -uo pipefail

registry_source="${REGISTRY_SOURCE:-}"

fail() {
    echo "::error title=Deploy preflight (#2930)::$1 — контейнеры НЕ останавливаются, работающий стек сохранён"
    exit 1
}

echo "🔎 preflight #2930: compose config"
docker compose config --quiet || fail "docker compose config невалиден"

if [ "$registry_source" != "skip" ]; then
    echo "📦 preflight #2930: docker compose pull --policy always"
    docker compose pull --policy always || fail "docker compose pull не скачал все образы"
else
    echo "⏭️  registry_source=skip — pull пропущен, проверяем только локальные образы"
fi

images="$(docker compose config --images)" || fail "docker compose config --images упал"
missing=()
while IFS= read -r image; do
    [ -n "$image" ] || continue
    if docker image inspect "$image" >/dev/null 2>&1; then
        echo "   ✅ $image"
    else
        echo "   ❌ $image"
        missing+=("$image")
    fi
done < <(printf '%s\n' "$images" | sort -u)

if [ "${#missing[@]}" -gt 0 ]; then
    fail "нет локально ${#missing[@]} образ(ов): ${missing[*]}"
fi

echo "✅ preflight #2930: все образы на месте — можно останавливать и пересоздавать"
