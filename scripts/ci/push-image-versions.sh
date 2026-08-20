#!/usr/bin/env bash
# Push a just-created image-versions commit after synchronising with the remote.
#
# The Main and Vision build jobs update different files but push to the same
# branch.  Both jobs can therefore pass checkout and commit at the same time;
# the rebase immediately before every push makes a non-fast-forward race
# recoverable instead of silently losing the image-version commit.
#
# Usage:
#   push-image-versions.sh <branch> [component]
#
# Environment:
#   IMAGE_VERSIONS_PUSH_ATTEMPTS      Maximum pull/rebase + push attempts (5)
#   IMAGE_VERSIONS_PUSH_RETRY_DELAY   Delay between attempts in seconds (5)
#   IMAGE_VERSIONS_GIT_REMOTE         Git remote name (origin)
set -euo pipefail

BRANCH="${1:?usage: push-image-versions.sh <branch> [component]}"
COMPONENT="${2:-image-versions}"
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
remote_branch_exists() {
  git ls-remote --exit-code --heads "$REMOTE" "$BRANCH" >/dev/null 2>&1
}

pull_and_rebase() {
  if ! remote_branch_exists; then
    echo "ℹ️  ${COMPONENT}: ${REMOTE}/${BRANCH} not found on remote — skipping rebase (first push)"
    return 0
  fi

  if git pull --rebase "$REMOTE" "$BRANCH"; then
    return 0
  fi

  echo "❌ ${COMPONENT}: git pull --rebase ${REMOTE} ${BRANCH} failed" >&2
  git rebase --abort >/dev/null 2>&1 || true
  return 1
}

for ((attempt = 1; attempt <= MAX_ATTEMPTS; attempt++)); do
  echo "🔄 ${COMPONENT}: synchronising with ${REMOTE}/${BRANCH} (attempt ${attempt}/${MAX_ATTEMPTS})"
  if ! pull_and_rebase; then
    exit 1
  fi

  if git push "$REMOTE" "HEAD:${BRANCH}"; then
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
