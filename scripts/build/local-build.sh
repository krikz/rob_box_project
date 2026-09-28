#!/usr/bin/env bash
# DEPRECATED-шим: старый интерфейс `local-build.sh [service|all|vision|main] [platform]`
# сохранён, но собирает теперь scripts/build/build.py — тем же манифестом
# (docker/build-manifest.yaml) и тем же движком (scripts/build/buildx_build.sh),
# что и CI. Раньше здесь жил свой список сервисов (apriltag, perception «на
# Vision Pi» …), без базовых образов, без APT_PROXY и BASE_IMAGE — локальная
# сборка не совпадала с CI ни по составу, ни по флагам.
#
# Новый интерфейс: scripts/build/build.py --help  (или `make build-help`).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_PY=(python3 "$SCRIPT_DIR/build.py")
TARGET="${1:-help}"
PLATFORM_ARGS=()
[ -n "${2:-}" ] && PLATFORM_ARGS=(--platform "$2")
TAG_ARGS=(--docker-tag "${IMAGE_TAG:-local}")

echo "⚠️  local-build.sh устарел → ${BUILD_PY[*]} (см. --help)" >&2
case "$TARGET" in
  help|-h|--help) exec "${BUILD_PY[@]}" --help ;;
  all)            exec "${BUILD_PY[@]}" all "${TAG_ARGS[@]}" "${PLATFORM_ARGS[@]}" ;;
  vision|main)    exec "${BUILD_PY[@]}" pi "$TARGET" "${TAG_ARGS[@]}" "${PLATFORM_ARGS[@]}" ;;
  base)           exec "${BUILD_PY[@]}" base all "${PLATFORM_ARGS[@]}" ;;
  *)              exec "${BUILD_PY[@]}" service "$TARGET" "${TAG_ARGS[@]}" "${PLATFORM_ARGS[@]}" ;;
esac
