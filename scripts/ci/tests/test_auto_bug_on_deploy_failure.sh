#!/usr/bin/env bash
set -euo pipefail

SCRIPT="$(cd "$(dirname "$0")/.." && pwd)/auto-bug-on-deploy-failure.sh"

bash -n "$SCRIPT"

if grep -q 'sshpass -p "$SSH_PASSWORD"' "$SCRIPT"; then
  echo "collector still references removed SSH_PASSWORD variable"
  exit 1
fi

grep -q 'sshpass -p "$SSHPASS"' "$SCRIPT"
grep -q 'deploy-dedup-key' "$SCRIPT"
grep -q 'docker inspect' "$SCRIPT"
grep -q 'docker logs --tail 50' "$SCRIPT"
grep -q 'docker ps -a --no-trunc' "$SCRIPT"
grep -q 'image_sha' "$SCRIPT"

echo "auto-bug-on-deploy-failure.sh: syntax and contract checks passed"
