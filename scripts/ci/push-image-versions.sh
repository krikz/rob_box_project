#!/usr/bin/env bash
# Push a just-created image-versions commit after synchronising with the remote.
#
# The Main and Vision build jobs update different files but push to the same
# branch.  Both jobs can therefore pass checkout and commit at the same time;
# the rebase immediately before every push makes a non-fast-forward race
# recoverable instead of silently losing the image-version commit.
#
# Usage:
#   push-image-versions.sh <branch> [component] [VAR...]
#
# VAR (PERCEPTION_TAG, VOICE_ASSISTANT_TAG, …) — поля, которые писал ЭТОТ
# билд. Без них конфликт rebase в .image-versions.* нечем разрешать: скрипт
# абортит rebase и падает (fail-closed). С ними — берёт свежую ветку и
# перезаписывает только свои строки (scripts/ci/resolve-image-versions-conflict.sh).
#
# Environment:
#   IMAGE_VERSIONS_PUSH_ATTEMPTS      Maximum pull/rebase + push attempts (5)
#   IMAGE_VERSIONS_PUSH_RETRY_DELAY   Delay between attempts in seconds (5)
#   IMAGE_VERSIONS_GIT_REMOTE         Git remote name (origin)
#   IMAGE_VERSIONS_GIT_TERMINAL_TIMEOUT  Seconds cap per git network op (60) — issue #t_dd49849b
#   IMAGE_VERSIONS_SSH_CONNECT_TIMEOUT    ssh ConnectTimeout in seconds (10)
set -euo pipefail

# Hard cap on git network operations. Without this, 'git pull --rebase' and
# 'git push' can hang indefinitely under ref contention (observed: 2.7h+
# stuck-in_progress with 3 concurrent runners racing on the same remote ref,
# 25.08.2026) or wait for a TCP connect to a blackholed host (~135 s per
# operation, run 42036798239 05.11.2026: 4 operations = 540 s).
#
# NOTE: git has no GIT_TERMINAL_TIMEOUT variable (the old export did nothing).
# The real cap is `timeout` around every network git call (git_net below);
# for ssh remotes ConnectTimeout is added on top, for https a stalled
# transfer is cut by GIT_HTTP_LOW_SPEED_*.
NET_TIMEOUT="${IMAGE_VERSIONS_GIT_TERMINAL_TIMEOUT:-60}"
export GIT_TERMINAL_PROMPT=0
export GIT_HTTP_LOW_SPEED_LIMIT="${GIT_HTTP_LOW_SPEED_LIMIT:-1000}"
export GIT_HTTP_LOW_SPEED_TIME="${GIT_HTTP_LOW_SPEED_TIME:-${NET_TIMEOUT}}"
if [[ -z "${GIT_SSH_COMMAND:-}" ]]; then
  # Keep whatever ssh command actions/checkout configured (key, known_hosts)
  # and only append the connect timeout.
  GIT_SSH_COMMAND="$(git config --get core.sshCommand 2>/dev/null || echo ssh) -o ConnectTimeout=${IMAGE_VERSIONS_SSH_CONNECT_TIMEOUT:-10}"
  export GIT_SSH_COMMAND
fi

BRANCH="${1:?usage: push-image-versions.sh <branch> [component] [VAR...]}"
COMPONENT="${2:-image-versions}"
shift $(( $# > 1 ? 2 : 1 ))
OURS_VARS=("$@")
RESOLVE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)/resolve-image-versions-conflict.sh"
MAX_ATTEMPTS="${IMAGE_VERSIONS_PUSH_ATTEMPTS:-5}"
RETRY_DELAY="${IMAGE_VERSIONS_PUSH_RETRY_DELAY:-5}"
REMOTE="${IMAGE_VERSIONS_GIT_REMOTE:-origin}"

if ! [[ "$MAX_ATTEMPTS" =~ ^[1-9][0-9]*$ ]]; then
  echo "❌ image-versions: IMAGE_VERSIONS_PUSH_ATTEMPTS must be a positive integer" >&2
  exit 2
fi
if ! [[ "$RETRY_DELAY" =~ ^[0-9]+$ ]]; then
  echo "❌ image-versions: IMAGE_VERSIONS_PUSH_RETRY_DELAY must be a non-negative integer" >&2
  exit 2
fi

# A failed pull/rebase (for example, a same-line edit of the versions file)
# is not safe to ignore.  Abort a partial rebase before failing the job so the
# runner does not leave a misleading half-rebased checkout behind.
#
# The first time a per-service branch is used, ``origin/<branch>`` does not
# exist yet; in that case the rebase step is a no-op and we just push.
# Run a git network operation under a hard time cap. rc 124 = timed out.
git_net() {
  if command -v timeout >/dev/null 2>&1; then
    timeout -k 5 "$NET_TIMEOUT" git "$@"
  else
    git "$@"
  fi
}

# rc 0 = branch exists, 2 = reachable but branch absent (ls-remote
# --exit-code), anything else = remote unreachable / timed out.
# Снимок своих тегов ДО rebase. Apply-бэкенд «успевает» и оставляет наш
# файл целиком, затирая чужой тег (run 44404978409). После синхронизации
# заново берём файл со свежей ветки и пишем в него только эти значения.
# Чужие строки и разделитель не трогаем.
capture_own_tags() {
  OWN_TAG_LINES=()
  [ "${#OURS_VARS[@]}" -eq 0 ] && return 0
  local file var line
  mapfile -t files < <(git diff-tree --no-commit-id --name-only -r HEAD -- 'docker/*/.image-versions.*')
  for file in "${files[@]}"; do
    for var in "${OURS_VARS[@]}"; do
      line="$(git show "HEAD:${file}" 2>/dev/null | grep -m1 "^${var}=" || true)"
      [ -n "$line" ] || continue
      OWN_TAG_LINES+=("${file}|${line}")
    done
  done
}

rewrite_own_tags_from_head() {
  [ "${#OWN_TAG_LINES[@]}" -eq 0 ] && return 0
  local entry file line value var fresh changed=0 restored=""
  fresh="$(git rev-parse "${REMOTE}/${BRANCH}")"
  for entry in "${OWN_TAG_LINES[@]}"; do
    file="${entry%%|*}"
    line="${entry#*|}"
    var="${line%%=*}"
    value="${line#*=}"
    # Файл со свежей ветки берём один раз: повторный checkout стирает
    # уже вписанный свой тег.
    case " ${restored} " in
      *" ${file} "*) ;;
      *)
        git checkout "$fresh" -- "$file" 2>/dev/null || continue
        restored="${restored} ${file}"
        ;;
    esac
    grep -q "^${var}=" "$file" || continue
    if ! grep -qx "${var}=${value}" "$file"; then
      sed -i "s|^${var}=.*|${var}=${value}|" "$file"
      echo "✅ ${COMPONENT}: ${file}: ${var}=${value} (свой тег поверх свежей ветки)" >&2
      changed=1
    fi
    git add -- "$file"
  done
  if [ "$changed" -eq 1 ] && ! git diff --cached --quiet; then
    git commit --amend --no-edit || return 1
  fi
  return 0
}

remote_branch_exists() {
  git_net ls-remote --exit-code --heads "$REMOTE" "$BRANCH" >/dev/null 2>&1
}

pull_and_rebase() {
  local rc=0
  remote_branch_exists || rc=$?
  if [[ "$rc" -eq 2 ]]; then
    echo "ℹ️  ${COMPONENT}: ${REMOTE}/${BRANCH} not found on remote — skipping rebase (first push)"
    return 0
  elif [[ "$rc" -ne 0 ]]; then
    echo "❌ ${COMPONENT}: ls-remote ${REMOTE} unreachable (rc=${rc}, cap ${NET_TIMEOUT}s; 124 = timed out)" >&2
    return 3
  fi

  # merge-бэкенд, не apply: apply молча затирает соседнюю строку чужого
  # билда, когда разделитель сдвинулся (run 44404978409). -c на pull не
  # доходит до дочернего rebase — пишем в локальный конфиг чекаута.
  git config rebase.backend merge
  capture_own_tags
  if git_net pull --rebase "$REMOTE" "$BRANCH"; then
    rewrite_own_tags_from_head || return 1
    return 0
  fi

  # Гонка соседних строк .image-versions.* (run 44404978409): rebase встал
  # на CONFLICT. Свои теги переписываем поверх свежей ветки и продолжаем.
  # Не наш конфликт или нечем переписывать — abort, попытка не повторяется.
  if [ "${#OURS_VARS[@]}" -gt 0 ] && [ -d "$(git rev-parse --git-path rebase-merge 2>/dev/null || true)" ]; then
    echo "⚠️  ${COMPONENT}: rebase конфликтует — переписываю только свои теги: ${OURS_VARS[*]}" >&2
    if bash "$RESOLVE" "${OURS_VARS[@]}" && GIT_EDITOR=true git rebase --continue; then
      rewrite_own_tags_from_head || return 1
      return 0
    fi
    echo "❌ ${COMPONENT}: конфликт .image-versions не разрешён, rebase прерван" >&2
  else
    echo "❌ ${COMPONENT}: git pull --rebase ${REMOTE} ${BRANCH} failed" >&2
  fi
  git rebase --abort >/dev/null 2>&1 || true
  return 1
}

for ((attempt = 1; attempt <= MAX_ATTEMPTS; attempt++)); do
  echo "🔄 ${COMPONENT}: synchronising with ${REMOTE}/${BRANCH} (attempt ${attempt}/${MAX_ATTEMPTS})"
  prc=0
  pull_and_rebase || prc=$?
  if [[ "$prc" -eq 3 ]]; then
    # Remote unreachable: count the attempt, retry after the delay.
    if (( attempt < MAX_ATTEMPTS )); then sleep "$RETRY_DELAY"; fi
    continue
  elif [[ "$prc" -ne 0 ]]; then
    exit 1
  fi

  if git_net push "$REMOTE" "HEAD:${BRANCH}"; then
    COMMIT_SUBJECT="$(git log -1 --pretty=%s)"
    echo "✅ ${COMPONENT}: pushed ${COMMIT_SUBJECT} to ${REMOTE}/${BRANCH}"
    exit 0
  fi

  echo "⚠️  ${COMPONENT}: push to ${REMOTE}/${BRANCH} failed (attempt ${attempt}/${MAX_ATTEMPTS})" >&2
  if (( attempt < MAX_ATTEMPTS )); then
    sleep "$RETRY_DELAY"
  fi
done

echo "❌ ${COMPONENT}: push to ${REMOTE}/${BRANCH} failed after ${MAX_ATTEMPTS} attempts" >&2
exit 1
