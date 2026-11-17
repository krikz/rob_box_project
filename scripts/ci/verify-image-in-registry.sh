#!/usr/bin/env bash
# Проверка, что тег реально лежит в локальном registry, ДО того как он попадёт
# в .image-versions.*.
#
# Acceptance B issue #1482: если .image-versions указывает на несуществующий
# тег, `docker compose pull` на роботе падает и робот молча остаётся на старом
# образе. Поэтому каждый SHA-тег проверяется по Registry API v2 перед коммитом.
#
# Почему отдельный скрипт, а не функция в каждом workflow
# (docs/plans/2026-09-15-image-versions-seam.md §7.4): копий было ТРИ —
# L-Build Vision Pi Services.yml, L-Build Main Pi Services.yml и
# L-Build Single Service.yml, — и они уже разъехались: в Vision-версии была
# лишняя строка про «не коммитим .image-versions», в двух других её не было.
# Расхождение косметическое, но держалось ни на чём: следующий баг вроде
# #1482 пришлось бы чинить синхронно в трёх местах. Тот же приём, что уже
# применён к scripts/ci/push-image-versions.sh.
#
# Использование:
#   verify-image-in-registry.sh <registry> <tag> [repo]
#     registry — хост:порт локального реестра (например localhost:5000)
#     tag      — полный тег внутри репозитория (например led-matrix-humble-dev-a1b2c3d)
#     repo     — путь репозитория в реестре, по умолчанию krikz/rob_box
#
# Коды возврата:
#   0 — манифест найден (HTTP 200)
#   1 — не найден, реестр недоступен или неверный вызов
set -euo pipefail

if [ "$#" -lt 2 ] || [ "$#" -gt 3 ]; then
    echo "usage: $(basename "$0") <registry> <tag> [repo]" >&2
    exit 1
fi

REGISTRY="$1"
TAG="$2"
REPO="${3:-krikz/rob_box}"

if [ -z "$REGISTRY" ] || [ -z "$TAG" ]; then
    echo "❌ Verify FAIL: пустой registry или tag (registry='${REGISTRY}', tag='${TAG}')" >&2
    exit 1
fi

URL="http://${REGISTRY}/v2/${REPO}/manifests/${TAG}"

# `|| echo 000` вместо set -e: недоступный реестр должен дать понятное
# сообщение с кодом 000, а не голый выход curl'а.
HTTP_CODE="$(curl -s -o /dev/null -w '%{http_code}' \
    -H 'Accept: application/vnd.docker.distribution.manifest.v2+json' \
    "$URL" || echo "000")"

if [ "$HTTP_CODE" != "200" ]; then
    echo "❌ Verify FAIL: ${TAG} missing in registry (HTTP ${HTTP_CODE})" >&2
    echo "   ⚠️  Не коммитим .image-versions с несуществующим SHA-tag" >&2
    exit 1
fi

echo "✅ Verify OK: ${TAG} present in registry"
exit 0
