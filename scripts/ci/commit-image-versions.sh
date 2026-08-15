#!/usr/bin/env bash
# ⚠️ DEPRECATED (decision #1316, 15.08) — НЕ ИСПОЛЬЗОВАТЬ.
#
# Этот скрипт коммитил .image-versions в ОТДЕЛЬНЫЕ ветки ci/image-versions-*.
# Механизм ЗАПРЕЩЁН: он сломал деплой дважды (#1139 13.08, #1244 13.08) —
# при registry_source=local деплой не грузит .image-versions из ci-ветки,
# compose тянет голые dev-теги → роботы получают старые образы.
# С 15.08 SHA-теги коммитятся ПРЯМО в текущую ветку (develop/main/round).
#
# Скрипт оставлен временно: build-workflow Main/Vision ещё ссылаются на него
# до merge PR #1318 (реверт decision #1316). После merge обоих PR (этот +
# #1318) скрипт можно удалить — references не останется.
# Commit .image-versions file(s) to a dedicated per-service CI branch.
#
# Почему отдельная ветка (ретро #1139 v2, 13.08):
#   - develop/main/round НЕ засоряются коммитами 'ci: ... SHA tags → ...' —
#     каждые ~40 мин main и vision билды коммитили .image-versions в develop,
#     гонялись друг с другом (non-fast-forward rejected) и плодили мусор в
#     истории (t_bd9b7713).
#   - Одна ветка ci/image-versions (#1139 v1) ломалась: билды делали
#     `git checkout -B ci/image-versions` от develop → сбрасывали ветку →
#     SHA-теги терялись, deploy тянул старые образы.
#   - Решение: per-service ветки (ci/image-versions-main / -vision), каждая
#     пишется ТОЛЬКО своим сервисом. Ветка НЕ сбрасывается от исходной ветки:
#     база берётся с origin-типа ветки, поверх кладётся ТОЛЬКО текущий файл —
#     теги для других .image-versions.{tag} накапливаются, ничего не теряется.
#
# Usage:
#   commit-image-versions.sh <versions_branch> <commit_msg> <file> [<file>...]
# Env:
#   IV_BASE_REF=<sha>  — база для создания ветки, если её ещё нет на origin
#                        (по умолчанию: текущий HEAD). Нужен когда в одном шаге
#                        workflow вызывается скрипт несколько раз (All-GHCR:
#                        после первого вызова HEAD уже на ci/* ветке).
#
# Caller: билд-воркфлоу уже отредактировал файл(ы) в рабочем дереве (sed на
# новые SHA-теги) и передаёт их пути. Скрипт сохраняет новое содержимое,
# переключается на <versions_branch> (база = origin-тип ветки или текущий HEAD,
# если ветки ещё нет), кладёт файлы, коммитит и пушит с retry+rebase.
set -euo pipefail

VERSIONS_BRANCH="${1:?usage: commit-image-versions.sh <versions_branch> <commit_msg> <file>...}"
COMMIT_MSG="${2:?usage: commit-image-versions.sh <versions_branch> <commit_msg> <file>...}"
BASE_REF="${IV_BASE_REF:-}"
shift 2

if [ "$#" -eq 0 ]; then
  echo "⚠️  commit-image-versions: нет файлов для коммита"
  exit 0
fi

# --- Сохраняем новое содержимое файлов (checkout -f сбросит рабочее дерево) ---
TMP_STORE="$(mktemp -d)"
trap 'rm -rf "$TMP_STORE"' EXIT
for f in "$@"; do
  if [ ! -f "$f" ]; then
    echo "⚠️  commit-image-versions: файл $f не найден — пропуск"
    continue
  fi
  mkdir -p "$TMP_STORE/$(dirname "$f")"
  cp "$f" "$TMP_STORE/$f"
done

git config user.email "ci@rob-box.local"
git config user.name "Rob Box CI"

# --- Базируем ветку на origin-типе (накопление истории, БЕЗ сброса от develop) ---
git fetch origin "$VERSIONS_BRANCH" >/dev/null 2>&1 || true
if git rev-parse --verify "origin/$VERSIONS_BRANCH" >/dev/null 2>&1; then
  git checkout -f -B "$VERSIONS_BRANCH" "origin/$VERSIONS_BRANCH"
elif [ -n "$BASE_REF" ]; then
  # Ветки ещё нет: создаём от переданной базы (исходная ветка билда)
  git checkout -f -B "$VERSIONS_BRANCH" "$BASE_REF"
else
  # Первый запуск: ветки нет — создаём от текущего HEAD (исходная ветка билда)
  git checkout -f -B "$VERSIONS_BRANCH"
fi

# --- Кладём новое содержимое поверх ветки и коммитим ---
CHANGED=0
for f in "$@"; do
  if [ -f "$TMP_STORE/$f" ]; then
    mkdir -p "$(dirname "$f")"
    cp "$TMP_STORE/$f" "$f"
    git add "$f"
    CHANGED=1
  fi
done

if [ "$CHANGED" -eq 0 ] || git diff --cached --quiet; then
  echo "ℹ️  commit-image-versions: нет изменений (${VERSIONS_BRANCH})"
  exit 0
fi

git commit -m "$COMMIT_MSG" >/dev/null

# --- Push с retry (гонка: параллельные билды одного сервиса на разных ветках
# --- пишут разные .image-versions.{tag} → rebase чистый, без конфликтов) ---
for i in 1 2 3 4 5; do
  git pull --rebase origin "$VERSIONS_BRANCH" >/dev/null 2>&1 || true
  if git push origin "HEAD:${VERSIONS_BRANCH}" >/dev/null 2>&1; then
    echo "✅ commit-image-versions: ${COMMIT_MSG} (branch ${VERSIONS_BRANCH})"
    exit 0
  fi
  echo "⚠️  push в ${VERSIONS_BRANCH} не удался (попытка ${i}/5), retry через 5s..."
  sleep 5
done

echo "❌ commit-image-versions: push в ${VERSIONS_BRANCH} не удался после 5 попыток — теги только локально" >&2
exit 1
