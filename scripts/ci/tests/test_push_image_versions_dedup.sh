#!/usr/bin/env bash
# Unit test for scripts/ci/push-image-versions.sh — dedup через gh api (#1630)
#
# Validates (5 cases, Шифу-фидбэк #1630: fail-CLOSED по умолчанию):
#   1. DEDUP=1 + gh возвращает subject → exit 0 без push
#   2. DEDUP=1 + gh возвращает пустой ответ → push выполняется
#   3. DEDUP=0 (disabled) → push выполняется даже если subject уже на remote
#   4. gh failure + FAIL_CLOSED=1 (default) → skip push (cron will retry)
#   5. gh failure + FAIL_CLOSED=0 (legacy) → push выполняется (fail-open)
#
# Strategy: каждый тест получает свой ИЗОЛИРОВАННЫЙ bare-repo origin (не
# делят состояние между тестами). Подменяем `gh` на stub (через PATH),
# который возвращает заранее заготовленный JSON или падает с exit 1.
# Проверяем rc + grep по логу + наличие/отсутствие push через git log on remote.
#
# Run: bash scripts/ci/tests/test_push_image_versions_dedup.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT="${SCRIPT_DIR}/../push-image-versions.sh"

if [ ! -x "$SCRIPT" ]; then
  echo "❌ $SCRIPT not executable"; exit 1
fi

if ! grep -q 'IMAGE_VERSIONS_DEDUP_REMOTE_QUERY' "$SCRIPT"; then
  echo "❌ $SCRIPT missing dedup hook"; exit 1
fi
if ! grep -q 'IMAGE_VERSIONS_FAIL_CLOSED_DEDUP' "$SCRIPT"; then
  echo "❌ $SCRIPT missing FAIL_CLOSED_DEDUP hook"; exit 1
fi
echo "✅ dedup + fail-closed hooks present"

# -----------------------------------------------------------------------------
# Shared fixture builder: создаёт чистый bare-repo origin + work tree, ворк
# уже на main, один init-коммит запушен. Возвращает через env ORIGIN_DIR.
# Использование:
#   setup_repo; cd "$WORK_DIR"
# -----------------------------------------------------------------------------
WORK_ROOT="$(mktemp -d)"
trap 'rm -rf "$WORK_ROOT"' EXIT

WORK=""
ORIGIN_DIR=""
setup_repo() {
  WORK="$WORK_ROOT/work-$RANDOM"
  ORIGIN_DIR="$WORK_ROOT/origin-$RANDOM.git"
  mkdir -p "$ORIGIN_DIR" "$WORK"
  git init -q --bare --initial-branch=main "$ORIGIN_DIR"
  cd "$WORK"
  git init -q --initial-branch=main .
  git -c user.email=t@x -c user.name=t commit -q --allow-empty -m "init"
  git remote add origin "$ORIGIN_DIR"
  git push -q origin main 2>/dev/null
}

STUB_BIN="$WORK_ROOT/stubbin"
mkdir -p "$STUB_BIN"

# --- Test 1: dedup finds subject → exit 0, push skipped ---------------------
setup_repo
# Override github-repo detection (so dedup runs against our stubbed `gh`,
# not skipped because the local bare-repo's URL is file://, not github.com).
SUBJECT="ci: main SHA tags → dev-aaa1111 [skip ci]"
git -c user.email=ci@x -c user.name=ci commit -q --allow-empty -m "$SUBJECT"
git push -q origin main 2>/dev/null
git -c user.email=ci@x -c user.name=ci commit -q --allow-empty -m "$SUBJECT"

cat > "$STUB_BIN/gh" <<EOF
#!/usr/bin/env bash
cat <<JSON
[{"sha":"deadbeef","commit":{"message":"${SUBJECT}\n"}}]
JSON
EOF
chmod +x "$STUB_BIN/gh"

set +e
PATH="$STUB_BIN:$PATH" \
  IMAGE_VERSIONS_GITHUB_REPO="krikz/rob_box_project" \
  IMAGE_VERSIONS_DEDUP_REMOTE_QUERY=1 \
  IMAGE_VERSIONS_PUSH_ATTEMPTS=1 \
  IMAGE_VERSIONS_PUSH_RETRY_DELAY=0 \
  bash "$SCRIPT" main main >/tmp/dedup1.out 2>&1
rc1=$?
set -e

if [ "$rc1" -ne 0 ]; then
  echo "❌ test 1: expected exit 0 (dedup skip), got $rc1"
  cat /tmp/dedup1.out
  exit 1
fi
if ! grep -q 'dedup' /tmp/dedup1.out; then
  echo "❌ test 1: expected 'dedup' message in output, got:"
  cat /tmp/dedup1.out
  exit 1
fi
# Verify NO duplicate push (remote should have only 2 commits: init + 1).
REMOTE_COUNT="$(git -C "$ORIGIN_DIR" rev-list --count main)"
if [ "$REMOTE_COUNT" -ne 2 ]; then
  echo "❌ test 1: expected remote to have 2 commits (no duplicate), got $REMOTE_COUNT"
  git -C "$ORIGIN_DIR" log --oneline
  exit 1
fi
echo "✅ test 1: dedup skips push when subject exists on remote"

# --- Test 2: dedup finds nothing → push proceeds ----------------------------
setup_repo
git -c user.email=ci@x -c user.name=ci commit -q --allow-empty -m "ci: main SHA tags → dev-bbb2222 [skip ci]"
git push -q origin main 2>/dev/null
git -c user.email=ci@x -c user.name=ci commit -q --allow-empty -m "ci: main SHA tags → dev-ccc3333 [skip ci]"

cat > "$STUB_BIN/gh" <<'EOF'
#!/usr/bin/env bash
cat <<'JSON'
[{"sha":"deadbeef","commit":{"message":"ci: main SHA tags → dev-bbb2222 [skip ci]\n"}}]
JSON
EOF
chmod +x "$STUB_BIN/gh"

set +e
PATH="$STUB_BIN:$PATH" \
  IMAGE_VERSIONS_GITHUB_REPO="krikz/rob_box_project" \
  IMAGE_VERSIONS_DEDUP_REMOTE_QUERY=1 \
  IMAGE_VERSIONS_PUSH_ATTEMPTS=1 \
  IMAGE_VERSIONS_PUSH_RETRY_DELAY=0 \
  bash "$SCRIPT" main main >/tmp/dedup2.out 2>&1
rc2=$?
set -e

if [ "$rc2" -ne 0 ]; then
  echo "❌ test 2: expected push to succeed, got rc=$rc2"
  cat /tmp/dedup2.out
  exit 1
fi
if ! grep -q 'pushed' /tmp/dedup2.out; then
  echo "❌ test 2: expected 'pushed' message, got:"
  cat /tmp/dedup2.out
  exit 1
fi
REMOTE_COUNT="$(git -C "$ORIGIN_DIR" rev-list --count main)"
if [ "$REMOTE_COUNT" -ne 3 ]; then
  echo "❌ test 2: expected remote to have 3 commits (init + bbb + ccc), got $REMOTE_COUNT"
  exit 1
fi
echo "✅ test 2: dedup allows push when subject not on remote"

# --- Test 3: dedup disabled → push proceeds even if subject exists ---------
setup_repo
SUBJECT3="ci: main SHA tags → dev-ddd4444 [skip ci]"
git -c user.email=ci@x -c user.name=ci commit -q --allow-empty -m "$SUBJECT3"
git push -q origin main 2>/dev/null
git -c user.email=ci@x -c user.name=ci commit -q --allow-empty -m "$SUBJECT3"

cat > "$STUB_BIN/gh" <<EOF
#!/usr/bin/env bash
cat <<JSON
[{"sha":"deadbeef","commit":{"message":"${SUBJECT3}\n"}}]
JSON
EOF
chmod +x "$STUB_BIN/gh"

set +e
PATH="$STUB_BIN:$PATH" \
  IMAGE_VERSIONS_DEDUP_REMOTE_QUERY=0 \
  IMAGE_VERSIONS_PUSH_ATTEMPTS=1 \
  IMAGE_VERSIONS_PUSH_RETRY_DELAY=0 \
  bash "$SCRIPT" main main >/tmp/dedup3.out 2>&1
rc3=$?
set -e

if [ "$rc3" -ne 0 ]; then
  echo "❌ test 3: expected push to succeed (dedup OFF), got rc=$rc3"
  cat /tmp/dedup3.out
  exit 1
fi
if ! grep -q 'pushed' /tmp/dedup3.out; then
  echo "❌ test 3: expected 'pushed' message, got:"
  cat /tmp/dedup3.out
  exit 1
fi
echo "✅ test 3: DEDUP=0 disables dedup, push proceeds"

# --- Test 4: gh failure + FAIL_CLOSED=1 (default) → skip push --------------
setup_repo
SUBJECT4="ci: main SHA tags → dev-eee5555 [skip ci]"
git -c user.email=ci@x -c user.name=ci commit -q --allow-empty -m "$SUBJECT4"
git push -q origin main 2>/dev/null
git -c user.email=ci@x -c user.name=ci commit -q --allow-empty -m "$SUBJECT4"

# gh stub: ALWAYS fails (simulates auth/network outage).
cat > "$STUB_BIN/gh" <<'EOF'
#!/usr/bin/env bash
echo "fake gh error" >&2
exit 1
EOF
chmod +x "$STUB_BIN/gh"

set +e
PATH="$STUB_BIN:$PATH" \
  IMAGE_VERSIONS_GITHUB_REPO="krikz/rob_box_project" \
  IMAGE_VERSIONS_DEDUP_REMOTE_QUERY=1 \
  IMAGE_VERSIONS_FAIL_CLOSED_DEDUP=1 \
  IMAGE_VERSIONS_PUSH_ATTEMPTS=1 \
  IMAGE_VERSIONS_PUSH_RETRY_DELAY=0 \
  bash "$SCRIPT" main main >/tmp/dedup4.out 2>&1
rc4=$?
set -e

# rc=0 because we skip push (caller treats as success — cron will retry).
if [ "$rc4" -ne 0 ]; then
  echo "❌ test 4: expected exit 0 (fail-closed skip), got $rc4"
  cat /tmp/dedup4.out
  exit 1
fi
if ! grep -qi 'gh api dedup query FAILED' /tmp/dedup4.out; then
  echo "❌ test 4: expected 'gh api dedup query FAILED' warning, got:"
  cat /tmp/dedup4.out
  exit 1
fi
if ! grep -qi 'fail-closed\|skip push' /tmp/dedup4.out; then
  echo "❌ test 4: expected 'fail-closed' or 'skip push' message, got:"
  cat /tmp/dedup4.out
  exit 1
fi
# Verify NO push happened — remote should still have only 2 commits (init + pre-pushed).
REMOTE_COUNT="$(git -C "$ORIGIN_DIR" rev-list --count main)"
if [ "$REMOTE_COUNT" -ne 2 ]; then
  echo "❌ test 4: expected remote to have 2 commits (no push), got $REMOTE_COUNT"
  git -C "$ORIGIN_DIR" log --oneline
  exit 1
fi
echo "✅ test 4: gh failure + fail-closed (default) → skip push, no duplicate"

# --- Test 5: gh failure + FAIL_CLOSED=0 (legacy) → push proceeds ------------
setup_repo
SUBJECT5="ci: main SHA tags → dev-fff6666 [skip ci]"
git -c user.email=ci@x -c user.name=ci commit -q --allow-empty -m "$SUBJECT5"
git push -q origin main 2>/dev/null
git -c user.email=ci@x -c user.name=ci commit -q --allow-empty -m "$SUBJECT5"

# gh stub still fails.
cat > "$STUB_BIN/gh" <<'EOF'
#!/usr/bin/env bash
echo "fake gh error" >&2
exit 1
EOF
chmod +x "$STUB_BIN/gh"

set +e
PATH="$STUB_BIN:$PATH" \
  IMAGE_VERSIONS_GITHUB_REPO="krikz/rob_box_project" \
  IMAGE_VERSIONS_DEDUP_REMOTE_QUERY=1 \
  IMAGE_VERSIONS_FAIL_CLOSED_DEDUP=0 \
  IMAGE_VERSIONS_PUSH_ATTEMPTS=1 \
  IMAGE_VERSIONS_PUSH_RETRY_DELAY=0 \
  bash "$SCRIPT" main main >/tmp/dedup5.out 2>&1
rc5=$?
set -e

if [ "$rc5" -ne 0 ]; then
  echo "❌ test 5: expected push to succeed (fail-open explicit), got rc=$rc5"
  cat /tmp/dedup5.out
  exit 1
fi
if ! grep -qi 'gh api dedup query failed' /tmp/dedup5.out; then
  echo "❌ test 5: expected warning about gh failure, got:"
  cat /tmp/dedup5.out
  exit 1
fi
if ! grep -q 'pushed' /tmp/dedup5.out; then
  echo "❌ test 5: expected 'pushed' message (fail-open), got:"
  cat /tmp/dedup5.out
  exit 1
fi
echo "✅ test 5: gh failure + FAIL_CLOSED=0 → push proceeds (legacy fail-open opt-in)"

echo ""
echo "PASS — all 5 dedup tests green (default = fail-closed, opt-out via env)"
