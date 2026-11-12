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

# Подготовка материала — каждый день, пока есть логи.
#
# Разбор 11.11: фильтр «не чаще 48 ч» стоял ПЕРЕД сбором фактов, и сессии
# 10, 11 и 12.11 вышли с «молчу до окна в 48 ч», не собрав ни одного факта.
# Правило 48 ч — это про ВЫХОД в канал, и его держит публикатор
# (channel_lib.MIN_GAP / pick_next). Дублировать его перед подготовкой
# значило запрещать готовить материал, а не публиковать его.
#
# Что осталось от ограничения: сообщение товарищу Шифу не должно сыпаться.
# Поэтому частота ограничена на ДОСТАВКЕ готового черновика, а сбор фактов
# идёт каждый день. Черновик, на который не ответили, протухает по TTL
# (studio_state.expire_stale_drafts): молчание — не отказ, факт возвращается
# в pending, и следующая сессия берётся за него снова.

# 1) Протухшие черновики — в expired, их факты — обратно в pending.
echo "[studio-session] 0/3 проверяю протухшие черновики"
python3 - "$STUDIO_REPO/docker/studio/state.json" "$STUDIO_REPO/scripts/telegram" <<'PY'
import sys

sys.path.insert(0, sys.argv[2])
import studio_state as st

path = sys.argv[1]
with st.state_transaction(path) as state:
    expired = st.expire_stale_drafts(state)
if expired:
    print(f"  протухло черновиков: {len(expired)} → {', '.join(expired)}")
else:
    print("  протухших черновиков нет")
PY

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

# Материал не должен утекать: в черновик попадает один факт, а остальные
# собранные — кладутся в ожидание. Иначе метка last_session сдвинется, и
# повторный сбор их уже не вернёт (фильтр merged:>=since): факт просто
# исчезнет (разбор 11.11).
python3 - "$FACTS" "$STUDIO_REPO/docker/studio/state.json" "$STUDIO_REPO/scripts/telegram" <<'PY'
import json
import sys

sys.path.insert(0, sys.argv[3])
import studio_state as st

with open(sys.argv[1], encoding="utf-8") as handle:
    facts = json.load(handle).get("visible_facts") or []
with st.state_transaction(sys.argv[2]) as state:
    st.keep_collected(state, facts)
print(f"  собрано в ожидание: {len(facts)}")
PY

# Частота доставки: не чаще раза в 48 ч. Это ограничение сообщений Шифу,
# а не подготовки: факты собраны, и если доставка сейчас не положена —
# они остаются в pending и уйдут в следующий раз (черновик протухнет по TTL).
LAST_DELIVERY=$(python3 - "$STUDIO_REPO/docker/studio/state.json" <<'PY'
import json
import sys

try:
    with open(sys.argv[1], encoding="utf-8") as handle:
        print(json.load(handle).get("last_delivery") or "")
except (OSError, ValueError):
    print("")
PY
)

TOO_SOON=$(python3 - "$LAST_DELIVERY" <<'PY'
import sys
from datetime import datetime, timezone

if len(sys.argv) < 2 or not sys.argv[1]:
    print("no")
    raise SystemExit(0)
try:
    # Метки времени пишутся с суффиксом "Z" (now_iso). Python до 3.11 такой
    # суффикс не понимает — без замены фильтр молча считал бы, что доставка
    # положена всегда, и Шифу получал бы черновик каждый день.
    last = datetime.fromisoformat(sys.argv[1].replace("Z", "+00:00"))
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
    echo "[studio-session] факты собраны, но доставка черновика не положена раньше 48 ч — Шифу не спамим"
    exit 0
fi

echo "[studio-session] 2/3 готовый пост: текст (MiniMax) → проверка кодом → медиа → Шифу"
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
now = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
with st.state_transaction(path) as state:
    state["last_session"] = now
    # Метка доставки: с неё считается лимит 48 ч на сообщения Шифу.
    state["last_delivery"] = now
print("  метка сессии обновлена")
PY
