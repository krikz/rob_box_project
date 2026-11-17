#!/bin/sh
# Общий старт Zenoh-роутера всех Pi робота (ADR-0159 В4, PR-2a).
#
# Образ eclipse/zenoh:1.6.2 — Alpine/busybox: только POSIX sh и busybox sed,
# bash нет. Compose запускает скрипт как `/bin/sh /scripts/start_zenoh_router.sh`.
#
# Шаблон docker/common/config/zenoh_router_config.json5 один на все узлы.
# Узловые различия приходят из env (дефолты — в docker-compose.yaml узла):
#   ZENOH_LISTEN_IP          — IP eth0 этого Pi: listen tcp/<IP>:7447
#   ZENOH_CONNECT_ENDPOINTS  — куда подключается роутер, через запятую
#                              (main: облако; vision: Main Pi по eth0)
# Пустое или кривое значение — падение с понятной ошибкой, а не тихий
# неверный адрес.
#
# ZENOH_ROUTER_TEMPLATE / ZENOH_ROUTER_CONFIG_OUT переопределяются только
# для проверки рендера вне контейнера; ZENOH_RENDER_ONLY=1 печатает путь к
# готовому конфигу и выходит, не запуская zenohd.
set -eu

TEMPLATE="${ZENOH_ROUTER_TEMPLATE:-/common_config/zenoh_router_config.json5}"
CONFIG_FILE="${ZENOH_ROUTER_CONFIG_OUT:-/tmp/zenoh_router_config.json5}"

fail() {
    echo "❌ ОШИБКА: $*" >&2
    exit 1
}

echo "=========================================="
echo "  Zenoh Router"
echo "=========================================="

# ROBOT_ID роутеру не нужен (namespace ставит ros_with_namespace.sh в
# session config нод, НЕ в router config). Main Pi передаёт ROBOT_ID в env
# роутера и сегодня не стартует без него — это защита от пустого .env на
# узле-шлюзе в облако; Vision Pi ROBOT_ID роутеру не передаёт. Поэтому
# проверяем только переданную (пусть пустую) переменную.
if [ -n "${ROBOT_ID+set}" ]; then
    [ -n "$ROBOT_ID" ] || fail "ROBOT_ID не задан! Установите переменную ROBOT_ID в .env файле"
    echo "🤖 Robot ID: $ROBOT_ID (namespace robots/$ROBOT_ID — только в session config нод)"
fi

LISTEN_IP="${ZENOH_LISTEN_IP:-}"
case "$LISTEN_IP" in
    "" | *[!0-9.]*)
        fail "ZENOH_LISTEN_IP='$LISTEN_IP' — нужен IPv4 eth0 этого Pi (дефолт задаёт docker-compose.yaml узла)"
        ;;
esac

ENDPOINTS="${ZENOH_CONNECT_ENDPOINTS:-}"
case "$ENDPOINTS" in
    "" | ,* | *, | *,,*)
        fail "ZENOH_CONNECT_ENDPOINTS='$ENDPOINTS' — нужен список endpoint'ов через запятую, без пустых"
        ;;
    *[\"\\\|\&\ ]*)
        fail "ZENOH_CONNECT_ENDPOINTS='$ENDPOINTS' — недопустимые символы (кавычка, \\, |, &, пробел)"
        ;;
esac

[ -f "$TEMPLATE" ] || fail "шаблон $TEMPLATE не найден (маунт ../common/config:/common_config:ro?)"

# "a,b" -> "a", "b" — элементы JSON-массива connect.endpoints.
ENDPOINTS_JSON="\"$(printf '%s' "$ENDPOINTS" | sed 's/,/", "/g')\""

sed -e "s|\"\${ZENOH_CONNECT_ENDPOINTS}\"|$ENDPOINTS_JSON|" \
    -e "s|\${ZENOH_LISTEN_IP}|$LISTEN_IP|g" \
    "$TEMPLATE" > "$CONFIG_FILE"

if grep -q '\${' "$CONFIG_FILE"; then
    fail "в $CONFIG_FILE остался неподставленный плейсхолдер: $(grep -n '\${' "$CONFIG_FILE")"
fi

echo "📡 listen:  tcp/$LISTEN_IP:7447"
echo "🔗 connect: $ENDPOINTS_JSON"
echo "✅ Router конфиг: $CONFIG_FILE"

if [ "${ZENOH_RENDER_ONLY:-0}" = "1" ]; then
    exit 0
fi

echo "Запуск Zenoh Router..."
echo "=========================================="
exec /zenohd -c "$CONFIG_FILE"
