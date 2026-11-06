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
. "$SECRETS"
set -e

if [ -z "${TG_STUDIO_BOT_TOKEN:-}" ] || [ -z "${MINIMAX_API_KEY:-}" ]; then
    echo "[studio-session] FATAL: нет TG_STUDIO_BOT_TOKEN или MINIMAX_API_KEY" >&2
    exit 1
fi

cd "$STUDIO_REPO"
mkdir -p "$WORK"

FACTS="$WORK/facts.json"
IDEAS="$WORK/ideas.json"

echo "[studio-session] 1/3 собираю факты"
python3 scripts/telegram/studio_facts.py --no-save --state "$STUDIO_REPO/docker/studio/state.json" > "$FACTS"

VISIBLE=$(python3 -c "import json;print(len(json.load(open('$FACTS')).get('visible_facts',[])))")
echo "[studio-session] видимых фактов: $VISIBLE"
if [ "$VISIBLE" = "0" ]; then
    echo "[studio-session] показывать нечего — Шифу ничего не шлю"
    exit 0
fi

echo "[studio-session] 2/3 пишу идеи (MiniMax) и проверяю кодом"
if ! python3 scripts/telegram/studio_text.py --facts "$FACTS" --out "$IDEAS" > /dev/null; then
    echo "[studio-session] модель не ответила или текст не прошёл проверку — молчу"
    exit 1
fi

OK_IDEAS=$(python3 -c "import json;print(sum(1 for i in json.load(open('$IDEAS')).get('ideas',[]) if i.get('ok')))")
echo "[studio-session] идей прошли проверку: $OK_IDEAS"
if [ "$OK_IDEAS" = "0" ]; then
    echo "[studio-session] ни одна идея не прошла проверку — молчу"
    exit 0
fi

# Шифу уходят только проверенные идеи.
python3 - "$IDEAS" "$STUDIO_REPO/docker/studio/ideas_to_send.json" <<'PY'
import json
import sys

data = json.load(open(sys.argv[1]))
good = [i for i in data.get("ideas", []) if i.get("ok")]
json.dump({"ideas": good}, open(sys.argv[2], "w"), ensure_ascii=False, indent=2)
print(f"  к отправке: {len(good)}")
PY

echo "[studio-session] 3/3 шлю идеи Шифу"
python3 scripts/telegram/channel_studio.py --state "$STUDIO_REPO/docker/studio/state.json" \
    ideas "$STUDIO_REPO/docker/studio/ideas_to_send.json"

# Метку сессии двигаем только после успешной отправки: иначе пропавшая сессия потеряла бы факты.
python3 - "$STUDIO_REPO/docker/studio/state.json" <<'PY'
import json
import sys
from datetime import datetime, timezone

path = sys.argv[1]
try:
    state = json.load(open(path))
except (OSError, ValueError):
    state = {}
state["last_session"] = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
json.dump(state, open(path, "w"), ensure_ascii=False, indent=2)
print("  метка сессии обновлена")
PY
