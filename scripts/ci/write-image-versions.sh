#!/usr/bin/env bash
# Записать новый SHA-тег в .image-versions.* каждого Pi, который этот образ деплоит.
#
# Usage:
#   write-image-versions.sh <docker_tag> <new_tag> <VAR> [<VAR>...]
# Env:
#   IMAGE_VERSIONS_DIR  — корень каталогов Pi (по умолчанию docker)
#
# Для каждого VAR (например PERCEPTION_TAG) правит строку `VAR=...` во ВСЕХ
# $IMAGE_VERSIONS_DIR/*/.image-versions.<docker_tag>, где поле объявлено.
# Поле в файле Pi и есть объявление «этот Pi запускает образ»: perception
# собирает Main-сборка, а читают его docker/main (perception) и docker/sound
# (sensor-bridge, ADR-0159 §2.5). Одна копия этой логики на "L: Build Pi
# Services" и "L: Build Single Service" — раньше в каждом был свой sed по
# одному файлу docker/<pi>/, и второй потребитель тега оставался на старом.
#
# stdout — пути изменённых файлов (по одному в строке, без повторов);
# их вызывающий кладёт в git add / commit-image-versions.sh. Диагностика — stderr.
# Нет ни одного файла .image-versions.<docker_tag> (тег local) — пусто, exit 0;
# поле не объявлено ни в одном файле — предупреждение, exit 0 (как прежний sed,
# который молча ничего не находил; guard
# test_image_versions_file_has_a_field_for_every_manifest_service ловит это в CI).
set -euo pipefail

DOCKER_TAG="${1:?usage: write-image-versions.sh <docker_tag> <new_tag> <VAR>...}"
NEW_TAG="${2:?usage: write-image-versions.sh <docker_tag> <new_tag> <VAR>...}"
shift 2
ROOT="${IMAGE_VERSIONS_DIR:-docker}"

if [ "$#" -eq 0 ]; then
  echo "⚠️  write-image-versions: не передано ни одного поля" >&2
  exit 0
fi

shopt -s nullglob
FILES=("$ROOT"/*/.image-versions."$DOCKER_TAG")
if [ "${#FILES[@]}" -eq 0 ]; then
  echo "⚠️  write-image-versions: нет ни одного $ROOT/*/.image-versions.$DOCKER_TAG" >&2
  exit 0
fi

CHANGED=()
for VAR in "$@"; do
  if ! [[ "$VAR" =~ ^[A-Z][A-Z0-9_]*_TAG$ ]]; then
    echo "::error::write-image-versions: '$VAR' — не имя поля *_TAG" >&2
    exit 2
  fi
  FOUND=0
  for FILE in "${FILES[@]}"; do
    grep -q "^${VAR}=" "$FILE" || continue
    sed -i "s|^${VAR}=.*|${VAR}=${NEW_TAG}|" "$FILE"
    echo "✅ ${FILE}: ${VAR}=${NEW_TAG}" >&2
    FOUND=1
    [[ " ${CHANGED[*]} " == *" ${FILE} "* ]] || CHANGED+=("$FILE")
  done
  if [ "$FOUND" -eq 0 ]; then
    echo "⚠️  write-image-versions: поле ${VAR} не объявлено ни в одном $ROOT/*/.image-versions.$DOCKER_TAG" >&2
  fi
done

if [ "${#CHANGED[@]}" -gt 0 ]; then
  printf '%s\n' "${CHANGED[@]}"
fi
