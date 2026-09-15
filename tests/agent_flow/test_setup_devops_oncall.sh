#!/bin/bash
# ============================================================================
# Test: setup-devops-oncall.sh
# Issue: #2394, kanban t_d215c9e0 (owner-setup)
#
# Что проверяем:
#   1. Скрипт запускается без bash-ошибок (syntax + minimal flags).
#   2. Без TELEGRAM_BOT_TOKEN выдаёт понятный exit 1 + actionable текст.
#   3. --help выводит usage.
#   4. install.sh включает setup-devops-oncall.sh в EXPECTED.
#
# Не проверяем (требует реального Telegram Bot API):
#   - getUpdates → chat_id резолв (нужен реальный канал + токен).
#   - notify-subscribe (нужна kanban-таска + профиль).
#
# Запуск:
#   bash tests/agent_flow/test_setup_devops_oncall.sh
# ============================================================================
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SCRIPT="${REPO_ROOT}/scripts/agent_flow/setup-devops-oncall.sh"
INSTALL="${REPO_ROOT}/scripts/agent_flow/install.sh"

fail() { echo "FAIL: $*" >&2; exit 1; }
ok()   { echo "OK: $*"; }

# 1. Синтаксис
bash -n "$SCRIPT" || fail "bash -n failed for $SCRIPT"
ok "syntax check passed"

# 2. Без флагов — usage error
set +e
out=$(bash "$SCRIPT" 2>&1)
rc=$?
set -e
if [[ $rc -eq 2 ]]; then
  ok "--chat-name/--env-file required check passed (exit 2)"
else
  fail "expected exit 2 for missing args, got $rc. Output: $out"
fi

# 3. С фиктивными путями + отключённым HERMES_HOME — exit 1 с actionable текстом
out=$(HERMES_HOME=/nonexistent bash "$SCRIPT" \
      --chat-name "Fake Channel" \
      --env-file /tmp/fake.env \
      --dry-run 2>&1) || true
if echo "$out" | grep -q "TELEGRAM_BOT_TOKEN не найден"; then
  ok "missing-token check passed (actionable error)"
else
  fail "expected 'TELEGRAM_BOT_TOKEN не найден' in output. Got: $out"
fi

# 4. --help работает
set +e
out=$(bash "$SCRIPT" --help 2>&1)
rc=$?
set -e
if [[ $rc -eq 0 ]] && echo "$out" | grep -q "setup-devops-oncall"; then
  ok "--help usage passed"
else
  fail "--help failed (rc=$rc). Output: $out"
fi

# 5. install.sh включает наш скрипт в EXPECTED
if bash "$INSTALL" --list-files 2>/dev/null | grep -q "^setup-devops-oncall.sh$"; then
  ok "EXPECTED includes setup-devops-oncall.sh"
else
  fail "EXPECTED does NOT include setup-devops-oncall.sh"
fi

# 6. HERMES_HOME нормализация: per-profile путь → корень
# Скрипт должен работать с HERMES_HOME=/home/builder/.hermes/profiles/X,
# а НЕ создавать /profiles/X/profiles/Y/.env. Проверяем через HERMES_HOME
# в полностью несуществующий путь (тогда normalize вернёт /home/builder/.hermes,
# и реальный токен в architect/.env найдётся; но chat_id 'X' — нет).
out=$(HERMES_HOME=/home/builder/.hermes/profiles/devops bash "$SCRIPT" \
      --chat-name "X" \
      --env-file /tmp/x.env \
      --dry-run 2>&1) || true
if echo "$out" | grep -q "chat_id для 'X' не найден"; then
  ok "HERMES_HOME normalization passed (script proceeded to getUpdates)"
else
  fail "HERMES_HOME normalization broken (script didn't reach getUpdates). Output: $out"
fi

# 7. dry-run НЕ создаёт env-файл
TMP_ENV=/tmp/setup-test-no-create-$$.env
[[ ! -f "$TMP_ENV" ]] || fail "pre-condition failed: $TMP_ENV exists"
bash "$SCRIPT" --chat-name "X" --env-file "$TMP_ENV" --dry-run 2>&1 >/dev/null || true
[[ ! -f "$TMP_ENV" ]] && ok "dry-run did not create env-file" \
                     || fail "dry-run created $TMP_ENV (must not)"
rm -f "$TMP_ENV"

echo "ALL TESTS PASSED"
