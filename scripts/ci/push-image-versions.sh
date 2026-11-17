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

  if git_net pull --rebase "$REMOTE" "$BRANCH"; then
    return 0
  fi

  echo "❌ ${COMPONENT}: git pull --rebase ${REMOTE} ${BRANCH} failed" >&2
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
