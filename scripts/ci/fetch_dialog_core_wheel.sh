#!/usr/bin/env bash
# pre_build-хук `fetch_dialog_core_wheel` (issue #4651, риск-родитель #4548).
#
# docs/plans/2026-09-15-service-manifest.md §2.5: манифест
# docker/build-manifest.yaml хранит ТОЛЬКО факт «у сервиса есть предсборочный
# хук» — императивная логика (источники, fallback, тексты ошибок) остаётся
# кодом, как у fetch_hailort_wheel (ADR-0089 Phase 1.5).
#
# Зачем: voice-assistant ставит ядро диалога из сети
# (rob-box-dialog-core @ git+https://github.com/krikz/rob_box_dialog_core@vX.Y.Z,
# docker/sound/voice_assistant/Dockerfile, Шаг 10.5). Self-hosted runner
# собирает без гарантии сети (acceptance issue #2378, ADR-0100): при
# инвалидации buildx-кеша слой git+URL падает офлайн → образ без ядра →
# voice-assistant в restart-loop (инциденты 10.11: #4643 / #4548). Хук кладёт
# СОБРАННЫЙ wheel ядра в build context (docker/sound/voice_assistant/wheels/,
# в git только .gitkeep + .gitignore на *.whl — бинарник НЕ коммитим);
# Dockerfile ставит ядро из локального wheel, а при его отсутствии делает
# fallback на git+URL (прежнее поведение).
#
# Источники wheel (офлайн-паттерн «vendor на build-хосте», как HailoRT):
#   1. /opt/rob_box/vendor/rob_box_dialog_core-<VERSION>-*.whl  (канонично)
#   2. /tmp/rob_box_dialog_core-<VERSION>-*.whl  (host /tmp проброшен в runner'ы)
#   3. Опционально: assets GitHub Release ядра — репо публичное, токен не нужен
#      (сегодня релизов нет, gh release download молча не находит — это ок).
#
# Версия берётся из ARG DIALOG_CORE_VERSION Dockerfile'а (единый источник
# истины — fallback git+URL подставляет именно его): wheel с другой версией
# НЕ копируется (предупреждение), чтобы не поставить молча ядро не того
# тега — решает fallback Dockerfile.
#
# Отсутствие wheel — НЕ ошибка хука: сборка не ломается раньше, чем ломалась
# бы без этого фикса; последнее слово у Dockerfile (fallback git+URL / честное
# падение слоя без сети).

set -euo pipefail

WHEELS_DIR="docker/sound/voice_assistant/wheels"
DOCKERFILE="docker/sound/voice_assistant/Dockerfile"
CORE_REPO="krikz/rob_box_dialog_core"

VERSION="$(grep -oE 'ARG DIALOG_CORE_VERSION=[0-9]+\.[0-9]+\.[0-9]+' "$DOCKERFILE" | head -n1 | sed 's/.*=//' || true)"
if [ -z "$VERSION" ]; then
  echo "::error::не нашёл ARG DIALOG_CORE_VERSION=X.Y.Z в $DOCKERFILE — хук не знает, какой wheel искать" >&2
  exit 1
fi
echo "Пин ядра в ${DOCKERFILE}: v${VERSION}"

mkdir -p "$WHEELS_DIR"

WHEEL_GLOB="rob_box_dialog_core-${VERSION}-*.whl"

found=""
for src in /opt/rob_box/vendor /tmp; do
  # Глоб раскрывается сознательно; при отсутствии совпадений [ -f ] отсекает
  # литеральный паттерн.
  for f in "$src"/$WHEEL_GLOB; do
    if [ -f "$f" ]; then
      found="$f"
      break 2
    fi
  done
done

if [ -n "$found" ]; then
  cp "$found" "$WHEELS_DIR/"
  echo "✅ wheel ядра в build context: $(basename "$found") (источник: $found)"
  ls -la "$WHEELS_DIR"/
  exit 0
fi

# Wheel ядра есть, но не той версии — misconfiguration хоста: НЕ копируем,
# чтобы fallback не поставил молча чужую версию.
for src in /opt/rob_box/vendor /tmp; do
  for f in "$src"/rob_box_dialog_core-*.whl; do
    if [ -f "$f" ]; then
      echo "::warning::нашёл $f, но пин Dockerfile — v${VERSION}; wheel НЕ копирую (fallback решит)" >&2
    fi
  done
done

# Опционально: релиз ядра с wheel (сегодня релизов нет — шаг молча пропускается).
if command -v gh >/dev/null 2>&1; then
  if gh release download --repo "$CORE_REPO" --pattern "$WHEEL_GLOB" --dir "$WHEELS_DIR" --clobber 2>/dev/null; then
    echo "✅ wheel ядра скачан из релиза ${CORE_REPO} v${VERSION}"
    ls -la "$WHEELS_DIR"/
    exit 0
  fi
fi

echo "⚠️ wheel rob_box_dialog_core v${VERSION} не найден ни в /opt/rob_box/vendor, ни в /tmp; релиза ядра с wheel нет." >&2
echo "⚠️ Dockerfile сделает fallback на git+URL: при сборке офлайн слой установки ядра УПАДЁТ (поведение до #4651)." >&2
echo "⚠️ Чинится провижинингом: pip wheel 'git+https://github.com/${CORE_REPO}@v${VERSION}' и положить артефакт в /opt/rob_box/vendor на build-хосте." >&2
