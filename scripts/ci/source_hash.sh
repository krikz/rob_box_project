#!/usr/bin/env bash
# Считает hash исходников сервиса для инвалидации кэша buildx.
#
# docs/plans/2026-09-15-service-manifest.md, §5 Phase 2. До Phase 2 эта
# формула была скопирована в шесть build-job'ов двух workflow с разными
# путями в каждой копии — и расхождение копий уже стоило нам stale-слоёв
# (issue #2314: SOURCE_HASH не покрывал src/rob_box_core → avatar_command.py
# не попадал в образ → ModuleNotFoundError на Vision Pi). Теперь пути и
# расширения живут в docker/build-manifest.yaml (source_hash.groups), а
# формула — здесь, в одном экземпляре.
#
# Вход: спецификация (аргумент $1 или stdin) — одна строка на группу:
#
#     <путь> [<путь> ...]<TAB><шаблон find> [<шаблон> ...]
#
# Её печатает scripts/ci/gen_build_matrix.py в поле source_hash_spec
# элемента matrix.
#
# Выход: один hex-хеш в stdout.
#
# Формула НЕ менялась при переносе (требование "поведение сборки не должно
# измениться"): конкатенация find-ов → sort → sha256sum по каждому файлу →
# sha256sum от получившегося списка → первый столбец. Многогруппные сервисы
# (quest, vision-hailo) и раньше складывали несколько find-ов в один sort,
# одногруппные — вызывали find один раз; общий sort делает оба варианта
# побайтово эквивалентными при одинаковом наборе файлов.

set -e

SPEC="${1-}"
if [ -z "$SPEC" ]; then
  SPEC="$(cat)"
fi

while IFS=$'\t' read -r HASH_PATHS HASH_PATTERNS; do
  [ -z "$HASH_PATHS" ] && continue
  # Слова делим через `read -a`, а НЕ через голый `for X in $VAR`: последнее
  # прогнало бы шаблоны через pathname expansion, и "*.py" превратился бы в
  # имена .py-файлов текущего каталога (проверено — хеш расходился с прежним).
  IFS=' ' read -r -a PATH_ARGS <<< "$HASH_PATHS"
  IFS=' ' read -r -a PATTERN_ARGS <<< "$HASH_PATTERNS"
  NAME_ARGS=()
  for PATTERN in "${PATTERN_ARGS[@]}"; do
    if [ ${#NAME_ARGS[@]} -eq 0 ]; then
      NAME_ARGS+=(-name "$PATTERN")
    else
      NAME_ARGS+=(-o -name "$PATTERN")
    fi
  done
  find "${PATH_ARGS[@]}" -type f \( "${NAME_ARGS[@]}" \)
done <<< "$SPEC" | sort | xargs sha256sum 2>/dev/null | sha256sum | awk '{print $1}'
