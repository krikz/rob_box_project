#!/usr/bin/env bash
# Разрешить конфликт rebase в .image-versions.*: перезаписать только свои *_TAG.
#
# Гонка Build All (run 44404978409): main и sound коммитят один файл
# docker/sound/.image-versions.dev. Разделитель между их строками не спасает —
# соседний коммит двигает его, и git pull --rebase останавливается на CONFLICT.
# Берём версию с обновлённой ветки (чужие теги свежие) и поверх неё пишем
# значения своих переменных из нашего коммита. Чужие строки и разделитель
# не меняются. Конфликт не в .image-versions.* — fail-closed, rebase не трогаем.
#
# Usage (из каталога с незавершённым rebase):
#   resolve-image-versions-conflict.sh <VAR> [<VAR>...]
# stdout — пути разрешённых файлов. Диагностика — stderr.
set -euo pipefail

if [ "$#" -eq 0 ]; then
  echo "❌ resolve-image-versions: не передано ни одного поля *_TAG" >&2
  exit 2
fi
for VAR in "$@"; do
  if ! [[ "$VAR" =~ ^[A-Z][A-Z0-9_]*_TAG$ ]]; then
    echo "❌ resolve-image-versions: '$VAR' — не имя поля *_TAG" >&2
    exit 2
  fi
done

REBASE_DIR="$(git rev-parse --git-path rebase-merge 2>/dev/null || true)"
if [ -z "$REBASE_DIR" ] || [ ! -d "$REBASE_DIR" ]; then
  echo "❌ resolve-image-versions: rebase не идёт — разрешать нечего" >&2
  exit 2
fi

OURS="$(git rev-parse REBASE_HEAD)"
mapfile -t CONFLICTS < <(git diff --name-only --diff-filter=U)
if [ "${#CONFLICTS[@]}" -eq 0 ]; then
  echo "❌ resolve-image-versions: rebase идёт, конфликтных файлов нет" >&2
  exit 1
fi

for FILE in "${CONFLICTS[@]}"; do
  case "$FILE" in
    docker/*/.image-versions.*) ;;
    *)
      echo "❌ resolve-image-versions: конфликт вне .image-versions.*: $FILE" >&2
      exit 1
      ;;
  esac
done

apply_ours() {
  local file="$1"
  local blob line var value
  git checkout --theirs -- "$file"
  for var in "$@"; do
    [ "$var" = "$file" ] && continue
    grep -q "^${var}=" "$file" || continue
    blob="$(git show "${OURS}:${file}" 2>/dev/null || true)"
    line="$(printf '%s\n' "$blob" | grep -m1 "^${var}=" || true)"
    if [ -z "$line" ]; then
      echo "❌ resolve-image-versions: ${var} нет в нашем коммите ${OURS}:${file}" >&2
      return 1
    fi
    value="${line#*=}"
    if [ -z "$value" ] || [[ "$value" == *" "* ]]; then
      echo "❌ resolve-image-versions: пустое или грязное значение ${var} в ${OURS}" >&2
      return 1
    fi
    sed -i "s|^${var}=.*|${var}=${value}|" "$file"
    echo "✅ ${file}: ${var}=${value} (наш тег поверх свежей ветки)" >&2
  done
}

RESOLVED=()
for FILE in "${CONFLICTS[@]}"; do
  apply_ours "$FILE" "$@"
  if grep -q '^<<<<<<<\|^>>>>>>>' "$FILE"; then
    echo "❌ resolve-image-versions: маркеры конфликта остались в $FILE" >&2
    exit 1
  fi
  git add -- "$FILE"
  RESOLVED+=("$FILE")
done

printf '%s\n' "${RESOLVED[@]}"
