#!/bin/bash
# Сессия анализа студии: собрать факты → написать идеи → прислать Шифу кнопками.
#
# Cron (no-agent): агента в контуре нет. Факты собирает код, текст пишет MiniMax (studio_text.py),
# проверяет код, и только проверенные идеи уходят Шифу с кнопками «Берём/Обсудить/Мимо».
# Нечего показать — сессия молчит: лишних сообщений у Шифу не бывает.
#
# Секреты — из docker/studio/.env.secrets (в git не попадает).
set -euo pipefail

STUDIO_REPO="${STUDIO_REPO:-/home/builder/hermes-share/rob_box_project}"
SECRETS="$STUDIO_REPO/docker/studio/.env.secrets"
WORK="$STUDIO_REPO/docker/studio/run"

if [ ! -f "$SECRETS" ]; then
    echo "[studio-session] FATAL: нет файла секретов: $SECRETS" >&2
    exit 1
fi

set +e
# shellcheck disable=SC1090
# set -a: переменные из файла секретов должны попасть в ОКРУЖЕНИЕ дочерних процессов.
# Без него python3 внутри скрипта не видит TG_STUDIO_BOT_TOKEN — ровно так и падал опрос.
set -a
. "$SECRETS"
set +a
set -e

if [ -z "${TG_STUDIO_BOT_TOKEN:-}" ] || [ -z "${MINIMAX_API_KEY:-}" ]; then
    echo "[studio-session] FATAL: нет TG_STUDIO_BOT_TOKEN или MINIMAX_API_KEY" >&2
    exit 1
fi

cd "$STUDIO_REPO"
mkdir -p "$WORK"

# Лимит частоты: бриф требует не чаще одного поста в 2–3 дня. Публикатор держит
# правило 48 часов для выхода в канал, но и предлагать новые идеи раньше бессмысленно:
# они всё равно не выйдут, а Шифу получит лишние сообщения.
LAST_POST=$(python3 - "$STUDIO_REPO/docs/channel/queue" <<'PY'
import pathlib
import re
import sys

queue = pathlib.Path(sys.argv[1])
stamps = []
for post in queue.glob("*.md"):
    try:
        text = post.read_text(encoding="utf-8")
    except OSError:
        continue
    match = re.search(r"^posted_at:\s*(\S+)", text, re.M)
    if match:
        stamps.append(match.group(1))
print(max(stamps) if stamps else "")
PY
)

if [ -n "$LAST_POST" ]; then
    TOO_SOON=$(python3 - "$LAST_POST" <<'PY'
import sys
from datetime import datetime, timedelta, timezone

try:
    last = datetime.fromisoformat(sys.argv[1])
except ValueError:
    print("no")
    raise SystemExit(0)
if last.tzinfo is None:
    last = last.replace(tzinfo=timezone.utc)
hours = (datetime.now(timezone.utc) - last).total_seconds() / 3600
print("yes" if hours < 48 else "no")
PY
)
    if [ "$TOO_SOON" = "yes" ]; then
        echo "[studio-session] последний пост выходит совсем недавно — молчу до окна в 48 ч"
        exit 0
    fi
fi

# Лимит частоты по метке последней сессии. Раньше считали только по
# последней публикации: без публикаций список уходил каждый день, и Шифу
# получал сообщения чаще, чем договаривались.
LAST_SESSION=$(python3 - "$STUDIO_REPO/docker/studio/state.json" "$STUDIO_REPO/scripts/telegram" <<'PY'
import json
import sys

try:
    with open(sys.argv[1], encoding="utf-8") as handle:
        print(json.load(handle).get("last_session") or "")
except (OSError, ValueError):
    print("")
PY
)

if [ -n "$LAST_SESSION" ]; then
    SESSION_SOON=$(python3 - "$LAST_SESSION" <<'PY'
import sys
from datetime import datetime, timezone

try:
    last = datetime.fromisoformat(sys.argv[1])
except ValueError:
    print("no")
    raise SystemExit(0)
if last.tzinfo is None:
    last = last.replace(tzinfo=timezone.utc)
hours = (datetime.now(timezone.utc) - last).total_seconds() / 3600
print("yes" if hours < 48 else "no")
PY
)
    if [ "$SESSION_SOON" = "yes" ]; then
        echo "[studio-session] с прошлой сессии не прошло 48 ч — молчу"
        exit 0
    fi
fi

FACTS="$WORK/facts.json"

echo "[studio-session] 1/3 собираю факты"
python3 scripts/telegram/studio_facts.py --no-save --state "$STUDIO_REPO/docker/studio/state.json" > "$FACTS"

VISIBLE=$(python3 - "$FACTS" <<'PY'
import json
import sys

# Путь — через argv: подстановка в код превращает имя файла в исполняемый код.
with open(sys.argv[1], encoding="utf-8") as handle:
    print(len(json.load(handle).get("visible_facts", [])))
PY
)
echo "[studio-session] видимых фактов: $VISIBLE"
if [ "$VISIBLE" = "0" ]; then
    echo "[studio-session] показывать нечего — Шифу ничего не шлю"
    exit 0
fi

echo "[studio-session] 2/2 готовый пост: текст (MiniMax) → проверка кодом → медиа → Шифу"
# Один шаг с индикатором: Шифу видит этапы в одном сообщении, черновик приходит с графикой.
# Не собралось — индикатор показывает, на каком шаге и почему; сессия выходит с кодом 1.
if ! python3 scripts/telegram/channel_studio.py --state "$STUDIO_REPO/docker/studio/state.json" \
    draft --facts "$FACTS"; then
    echo "[studio-session] черновик не собран или не отправлен — причина в индикаторе у Шифу"
    exit 1
fi

# Метку сессии двигаем только после успешной отправки: иначе пропавшая сессия потеряла бы факты.
# Пишем через studio_state: там атомарная запись и блокировка, а в этом heredoc
# state.json переписывался целиком мимо обоих — и затирал правки опроса.
# Второй аргумент — каталог модулей: без него sys.argv[2] падал IndexError и метка не двигалась.
python3 - "$STUDIO_REPO/docker/studio/state.json" "$STUDIO_REPO/scripts/telegram" <<'PY'
import sys
from datetime import datetime, timezone

sys.path.insert(0, sys.argv[2])
import studio_state as st

path = sys.argv[1]
with st.state_transaction(path) as state:
    state["last_session"] = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
print("  метка сессии обновлена")
PY
