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
#   IMAGE_VERSIONS_GIT_TERMINAL_TIMEOUT  Seconds for git network ops (60) — issue #t_dd49849b
#   IMAGE_VERSIONS_DEDUP_REMOTE_QUERY  Set 0 to disable dedup (default 1)
#   IMAGE_VERSIONS_DEDUP_PAGE_SIZE     Commits inspected for dedup (default 20)
#   IMAGE_VERSIONS_FAIL_CLOSED_DEDUP   If 1 (default), gh api failure → skip push
#                                       (cron will retry). If 0, gh failure → push.
#                                       Default 1: Шифу-фидбэк #1630 — duplicate
#                                       commits хуже чем пропущенный push.
set -euo pipefail

# Hard cap on git network operations. Without this, 'git pull --rebase' and
# 'git push' can hang indefinitely under ref contention (observed: 2.7h+
# stuck-in_progress with 3 concurrent runners racing on the same remote ref,
# 25.08.2026). GIT_TERMINAL_TIMEOUT aborts the git call after N seconds.
export GIT_TERMINAL_TIMEOUT="${IMAGE_VERSIONS_GIT_TERMINAL_TIMEOUT:-60}"

BRANCH="${1:?usage: push-image-versions.sh <branch> [component]}"
COMPONENT="${2:-image-versions}"
MAX_ATTEMPTS="${IMAGE_VERSIONS_PUSH_ATTEMPTS:-5}"
RETRY_DELAY="${IMAGE_VERSIONS_PUSH_RETRY_DELAY:-5}"
REMOTE="${IMAGE_VERSIONS_GIT_REMOTE:-origin}"
DEDUP_ENABLED="${IMAGE_VERSIONS_DEDUP_REMOTE_QUERY:-1}"
DEDUP_PAGE_SIZE="${IMAGE_VERSIONS_DEDUP_PAGE_SIZE:-20}"
FAIL_CLOSED_DEDUP="${IMAGE_VERSIONS_FAIL_CLOSED_DEDUP:-1}"
GITHUB_REPO_OVERRIDE="${IMAGE_VERSIONS_GITHUB_REPO:-}"

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

# Issue #1630 / ADR-0031: race-condition dedup. Two parallel builds (e.g. develop
# + z-{e2e}/test-round-N) can both produce a commit with the same SHA-tag subject
# ("ci: main SHA tags → dev-abc1234"). Without dedup, both runners race through
# retry+rebase and BOTH eventually push — duplicating commits in develop history
# (observed 2-3 ci:* SHA tags commits per build cycle, see issue #1075).
#
# Algorithm: BEFORE pushing, query GitHub for the last N commit subjects on
# origin/${BRANCH}. If our pending subject is already there, exit 0 (idempotent
# skip). This is cheap (1 API call, ≤20 commits) and works even if our local
# branch is behind — we ask the source of truth (origin), not our local reflog.
#
# FAIL-CLOSED policy (Шифу-фидбэк #1630, 25.08): if gh api fails (network blip,
# auth error, GitHub down), we do NOT push. Rationale: a duplicate commit
# pollutes develop history permanently; a missed push is recoverable via
# hourly cron (`L-Update Image Versions.yml` schedule `17 * * * *`).
# To restore fail-OPEN legacy behaviour, set IMAGE_VERSIONS_FAIL_CLOSED_DEDUP=0.
#
# Caveats:
#   - Requires gh CLI authenticated against the same org as $REMOTE. In CI this
#     is automatic (GITHUB_TOKEN); for local dev the user must `gh auth login`.
remote_has_pending_subject() {
  if [ "$DEDUP_ENABLED" != "1" ]; then
    return 1  # dedup disabled → never skip
  fi

  local PENDING_SUBJECT GH_REPO REMOTE_URL API_RESPONSE
  PENDING_SUBJECT="$(git log -1 --pretty=%s)"

  # Detect GitHub repo from remote URL. We only dedup on github.com remotes;
  # for gitlab/bitbucket/private hosts this function silently returns 1 (no skip).
  # IMAGE_VERSIONS_GITHUB_REPO override is for tests (avoids needing a real
  # github.com remote in the test fixture's bare-repo).
  if [ -n "$GITHUB_REPO_OVERRIDE" ]; then
    GH_REPO="$GITHUB_REPO_OVERRIDE"
  else
    REMOTE_URL="$(git remote get-url "$REMOTE" 2>/dev/null || true)"
    GH_REPO="$(printf '%s' "$REMOTE_URL" \
      | sed -nE 's#^https://github\.com/([^/]+/[^/]+)\.git$#\1#p; s#^git@github\.com:([^/]+/[^/]+)\.git$#\1#p')"
  fi
  if [ -z "$GH_REPO" ]; then
    echo "ℹ️  ${COMPONENT}: dedup skipped (remote is not github.com)"
    return 1
  fi

  # Use gh api (inherits GH_TOKEN/GITHUB_TOKEN). List commits on ${BRANCH}.
  # On any gh failure (network, auth, rate-limit) we FAIL CLOSED: refuse to push.
  # Caller treats exit 3 as "skip push, let cron retry".
  if ! API_RESPONSE="$(gh api \
      --method GET \
      -H "Accept: application/vnd.github+json" \
      -f per_page="$DEDUP_PAGE_SIZE" \
      "/repos/${GH_REPO}/commits?sha=${BRANCH}" 2>/dev/null)"; then
    if [ "$FAIL_CLOSED_DEDUP" = "1" ]; then
      echo "⚠️  ${COMPONENT}: gh api dedup query FAILED — failing closed (skip push, cron will retry)" >&2
      return 3  # 3 = "gh failed AND fail-closed" → caller skips push
    fi
    echo "⚠️  ${COMPONENT}: gh api dedup query failed — proceeding with push (fail-open)" >&2
    return 1
  fi

  # Grep for our subject in the returned JSON (commit messages live under
  # .commit.message). grep -F = literal match; -q = quiet.
  if printf '%s' "$API_RESPONSE" | grep -F -q "$PENDING_SUBJECT"; then
    echo "ℹ️  ${COMPONENT}: dedup — '${PENDING_SUBJECT}' already on ${BRANCH} (skip push)"
    return 0  # 0 = "subject found" → caller skips push
  fi
  return 1  # 1 = "not found" → caller proceeds with push
}

for ((attempt = 1; attempt <= MAX_ATTEMPTS; attempt++)); do
  echo "🔄 ${COMPONENT}: synchronising with ${REMOTE}/${BRANCH} (attempt ${attempt}/${MAX_ATTEMPTS})"
  if ! pull_and_rebase; then
    exit 1
  fi

  # Issue #1630: dedup BEFORE git push — if SHA-tag already on remote, skip.
  # rc=0 → skip push (subject found); rc=3 → skip push (fail-closed); rc=1 → push.
  DEDUP_RC=0
  remote_has_pending_subject || DEDUP_RC=$?
  if [ "$DEDUP_RC" -eq 0 ] || [ "$DEDUP_RC" -eq 3 ]; then
    if [ "$DEDUP_RC" -eq 3 ]; then
      echo "⏭️  ${COMPONENT}: skip push (fail-closed dedup), cron will retry" >&2
    fi
    exit 0
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
