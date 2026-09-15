#!/bin/bash
# ============================================================================
# SOT (source-of-truth): <repo>/scripts/agent_flow/setup-devops-oncall.sh
# Правим ТОЛЬКО здесь + commit + merge в develop. На хост раскладывает
# `bash <repo>/scripts/agent_flow/install.sh` — hardlink-копиями (cp -al), НЕ
# симлинками (см. README.md «Source of Truth»).
#
# setup-devops-oncall.sh — owner-setup для отдельного Telegram-канала
# `#devops-oncall`, через который пойдут stale-candidate / priority:high
# алерты (issue #2394, PM-default от 2026-09-15, kanban t_d215c9e0).
#
# Что делает (после того как владелец УЖЕ создал канал в Telegram и бот
# УЖЕ добавлен туда как админ):
#   1. Получает chat_id через getUpdates Telegram Bot API (используя токен
#      из активного профиля). Это работает потому что только ОДИН
#      Hermes-профиль держит getUpdates (после ретро t_5af222ea — это
#      `architect`). Этот скрипт НЕ polling-ит, он только забирает
#      уже-прочитанные апдейты через прямой GET.
#   2. Пишет DEVOPS_ALERT_CHAT_ID=<id> в .env.249 (или в /home/builder/.hermes/
#      profiles/<profile>/.env, если передан --profile).
#   3. Регистрирует notify-subscribe на kanban-таску (если передан --task)
#      для test-уведомления в новый канал.
#   4. Делает test-push в канал (одноразовое сообщение, dry-run by default).
#
# Что НЕ делает:
#   - НЕ создаёт Telegram-канал/чат (это ручная операция владельца в
#     Telegram-клиенте: New Group/Channel → invite bot by @username →
#     promote to admin).
#   - НЕ модифицирует .env архитектора/девопса/pm (только --profile target).
#   - НЕ меняет TELEGRAM_BOT_TOKEN — single-owner архитектура (ретро
#     t_5af222ea) требует, чтобы getUpdates-consumer был один.
#
# Использование:
#   bash setup-devops-oncall.sh --chat-name "#devops-oncall" \
#                               --env-file /home/builder/.env.249 \
#                               --dry-run
#   bash setup-devops-oncall.sh --chat-name "#devops-oncall" \
#                               --env-file /home/builder/.env.249 \
#                               --apply \
#                               --test-task t_d215c9e0
#
# Переменные окружения:
#   TELEGRAM_BOT_TOKEN  — обязателен; берётся из профиля, который
#                          держит токен (после ретро t_5af222ea — это
#                          `architect` profile).
# ============================================================================
set -euo pipefail

PREFIX="[setup-devops-oncall]"

CHAT_NAME=""
ENV_FILE=""
PROFILE="${SETUP_PROFILE:-architect}"  # default: тот, кто держит токен
DRY_RUN="${DRY_RUN:-true}"
TEST_TASK=""

usage() {
  sed -n '2,/^# ====/p' "$0" | sed 's/^# \?//'
  exit 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --chat-name)  CHAT_NAME="$2"; shift 2;;
    --env-file)   ENV_FILE="$2";   shift 2;;
    --profile)    PROFILE="$2";    shift 2;;
    --apply)      DRY_RUN="false"; shift;;
    --dry-run)    DRY_RUN="true";  shift;;
    --test-task)  TEST_TASK="$2";  shift 2;;
    -h|--help)    usage;;
    *)            echo "$PREFIX unknown arg: $1" >&2; exit 2;;
  esac
done

if [[ -z "$CHAT_NAME" ]]; then
  echo "$PREFIX --chat-name обязателен (например '#devops-oncall')." >&2
  exit 2
fi
if [[ -z "$ENV_FILE" ]]; then
  echo "$PREFIX --env-file обязателен (например /home/builder/.env.249)." >&2
  exit 2
fi

HERMES_HOME="${HERMES_HOME:-/home/builder/.hermes}"
HERMES_BIN="${HERMES_BIN:-${HERMES_HOME}/hermes-agent/venv/bin/hermes}"
BOARD="${KANBAN_BOARD:-robbox}"

# Per-profile gateway задаёт HERMES_HOME=/home/builder/.hermes/profiles/<p>;
# default gateway задаёт HERMES_HOME=/home/builder/.hermes. Нормализуем к корню:
if [[ "${HERMES_HOME}" == */profiles/* ]]; then
  HERMES_HOME="/home/builder/.hermes"
fi

# --- 1. Resolve TELEGRAM_BOT_TOKEN ----------------------------------------
TOKEN=""
for cand in \
  "${HERMES_HOME}/profiles/${PROFILE}/.env" \
  "${HERMES_HOME}/.env"; do
  if [[ -f "$cand" ]]; then
    found=$(grep -E "^TELEGRAM_BOT_TOKEN=[0-9]+:" "$cand" 2>/dev/null \
            | head -1 | cut -d= -f2- || true)
    if [[ -n "$found" ]]; then
      TOKEN="$found"
      echo "$PREFIX токен взят из $cand (profile=$PROFILE)"
      break
    fi
  fi
done

if [[ -z "$TOKEN" ]]; then
  echo "$PREFIX TELEGRAM_BOT_TOKEN не найден ни в одном .env." >&2
  echo "$PREFIX Проверь, что профиль '$PROFILE' имеет активный токен" >&2
  echo "$PREFIX (после ретро t_5af222ea — single-owner = architect)." >&2
  exit 1
fi

# --- 2. Fetch chat_id via Telegram getUpdates ----------------------------
# Используем offset=0 и достаём только последний апдейт из каждого чата.
# ВНИМАНИЕ: это работает корректно только если:
#   (a) бот УЖЕ добавлен в целевой канал (как минимум member или admin);
#   (b) в канале УЖЕ есть хотя бы одно сообщение, которое бот видит
#       (или пользователь отправил /start боту в личке, или в канале
#       кто-то написал после добавления бота).
#
# Это требование Telegram Bot API: getUpdates возвращает только то, что
# бот «видит», а он видит только сообщения в чатах, где он состоит.
get_chat_id() {
  local raw
  raw=$(curl --silent --show-error --max-time 10 \
        "https://api.telegram.org/bot${TOKEN}/getUpdates?timeout=0" \
        || true)
  if [[ -z "$raw" ]]; then
    echo ""
    return 0
  fi
  # Ищем chat.id, где chat.title содержит CHAT_NAME.
  # Простой python-парсинг — структура ответа стабильна.
  local id
  id=$(CHAT_NAME="$CHAT_NAME" RAW="$raw" python3 - <<'PY' 2>/dev/null || true
import json, os
target = os.environ.get("CHAT_NAME", "").lstrip("#").lower()
raw = os.environ.get("RAW", "")
try:
    data = json.loads(raw)
except Exception:
    raise SystemExit(1)
for upd in data.get("result", []):
    ch = upd.get("message", {}).get("chat") \
         or upd.get("my_chat_member", {}).get("chat") \
         or upd.get("channel_post", {}).get("chat")
    if ch and target in (ch.get("title", "") or "").lower():
        print(ch["id"])
        raise SystemExit(0)
PY
)
  if [[ -z "$id" ]]; then
    echo ""
    return 0
  fi
  echo "$id"
}

CHAT_ID="$(get_chat_id)"
if [[ -z "$CHAT_ID" ]]; then
  echo "$PREFIX ❌ chat_id для '$CHAT_NAME' не найден в getUpdates." >&2
  echo "" >&2
  echo "Чек-лист:" >&2
  echo "  1. Канал '$CHAT_NAME' УЖЕ создан в Telegram-клиенте." >&2
  echo "  2. Бот (тот, чей токен в $PROFILE/.env) УЖЕ добавлен в канал" >&2
  echo "     как member или admin (Settings → Administrators → Add Admin)." >&2
  echo "  3. После добавления бота в канале кто-то УЖЕ отправил хотя бы" >&2
  echo "     одно сообщение (или админ нажал /start в личке у бота)." >&2
  echo "  4. Прошло ≤ несколько секунд, чтобы getUpdates зафиксировал." >&2
  echo "" >&2
  echo "Если всё чек-лист ОК — повтори через 5-10 сек:" >&2
  echo "  bash $0 --chat-name '$CHAT_NAME' --env-file '$ENV_FILE' --apply" >&2
  exit 1
fi
echo "$PREFIX ✅ chat_id найден: $CHAT_ID (для '$CHAT_NAME')"

# --- 3. Validate env file & write DEVOPS_ALERT_CHAT_ID -------------------
if [[ ! -f "$ENV_FILE" ]]; then
  echo "$PREFIX env-файл $ENV_FILE не существует — создаю пустой." >&2
  if [[ "$DRY_RUN" == "true" ]]; then
    echo "$PREFIX (DRY-RUN: реально не создаю)"
  else
    touch "$ENV_FILE"
    chmod 600 "$ENV_FILE"
  fi
fi

if [[ "$DRY_RUN" == "true" ]]; then
  echo "$PREFIX (DRY-RUN) планирую записать в $ENV_FILE:"
  echo "  DEVOPS_ALERT_CHAT_ID=$CHAT_ID"
  echo "  DEVOPS_ALERT_CHAT_NAME=$CHAT_NAME"
else
  # Не перетираем, если уже есть — backup.
  if grep -qE "^DEVOPS_ALERT_CHAT_ID=" "$ENV_FILE" 2>/dev/null; then
    cp -a "$ENV_FILE" "${ENV_FILE}.bak.$(date -u +%Y%m%dT%H%M%SZ)"
    echo "$PREFIX backup старого DEVOPS_ALERT_CHAT_ID → ${ENV_FILE}.bak.*"
    # Удаляем старую строку.
    grep -v "^DEVOPS_ALERT_CHAT_ID=" "$ENV_FILE" > "${ENV_FILE}.tmp"
    mv "${ENV_FILE}.tmp" "$ENV_FILE"
  fi
  echo "DEVOPS_ALERT_CHAT_ID=$CHAT_ID" >> "$ENV_FILE"
  echo "DEVOPS_ALERT_CHAT_NAME=$CHAT_NAME" >> "$ENV_FILE"
  chmod 600 "$ENV_FILE"
  echo "$PREFIX ✅ записано в $ENV_FILE (chmod 600)."
fi

# --- 4. Optional: test notify-subscribe + test push ----------------------
if [[ -n "$TEST_TASK" ]]; then
  if [[ "$DRY_RUN" == "true" ]]; then
    echo "$PREFIX (DRY-RUN) не выполню notify-subscribe + test-push."
    echo "  Реальная команда (после --apply):"
    echo "    $HERMES_BIN kanban --board $BOARD notify-subscribe \\"
    echo "      --platform telegram --chat-id $CHAT_ID \\"
    echo "      --notifier-profile devops $TEST_TASK"
  else
    echo "$PREFIX notify-subscribe task=$TEST_TASK chat=$CHAT_ID profile=devops"
    "$HERMES_BIN" kanban --board "$BOARD" notify-subscribe \
      --platform telegram \
      --chat-id "$CHAT_ID" \
      --notifier-profile devops \
      "$TEST_TASK" || {
        echo "$PREFIX ⚠️ notify-subscribe завершился с ошибкой — не критично." >&2
        echo "$PREFIX   Это нормально, если task_id уже complete (тогда notify" >&2
        echo "$PREFIX   не нужен) или если профиль devops не имеет токена." >&2
      }
    echo "$PREFIX ✅ test-push уйдёт автоматически от notify-gateway."
    echo "$PREFIX   Проверь в '$CHAT_NAME' что сообщение появилось."
  fi
fi

echo ""
echo "$PREFIX ИТОГ:"
echo "  CHAT_NAME       = $CHAT_NAME"
echo "  CHAT_ID         = $CHAT_ID"
echo "  ENV_FILE        = $ENV_FILE"
echo "  DEVOPS_PROFILE  = $PROFILE (держит TELEGRAM_BOT_TOKEN)"
echo "  DRY_RUN         = $DRY_RUN"
echo ""
echo "Дальнейшие шаги (после --apply):"
echo "  1. Закоммитить изменение $ENV_FILE в защищённый vault / secrets-store"
echo "     (chat_id — это секрет уровня 'low' — позволяет слать в канал,"
echo "     но не даёт доступа к бот-токену)."
echo "  2. Закоммитить этот скрипт + runbook в репо (PR #XXXX)."
echo "  3. Issue #2394 обновить: «канал создан, ID зафиксирован»."
echo "  4. Открыть follow-up kanban-карточку для t_6927a4c5 (alerter):"
echo "     теперь у неё есть целевой CHAT_ID."
