#!/usr/bin/env bash
# Самопроверка DNS-патча lite-kit без стенда: применяет его к копии фикстуры
# и сверяет sha256. Сырой вывод sha256sum и bash -n — в stdout для логов.
# Вызывается из tests/unit/scripts/test_lite_kit_dns_patch.py (pytest, CI).
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIXTURE="$DIR/fixtures/equasar_server.py.bak-20261108"
APPLY="$DIR/apply-dns-patch.sh"
PATCHED_SHA256="87bd82bc1c3235be4be6b2838f17410573276078d41d9551ee47bfd5e16c8d31"

echo "== bash -n apply-dns-patch.sh =="
bash -n "$APPLY" && echo "bash -n: OK"

echo "== sha256 фикстуры (исходник lite-kit @ 8a82ff3) =="
sha256sum "$FIXTURE"

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
cp "$FIXTURE" "$TMP/equasar_server.py"

echo "== применение патча к копии =="
"$APPLY" "$TMP/equasar_server.py"

echo "== sha256 результата =="
sha256sum "$TMP/equasar_server.py"

echo "== ключевые изменения в результате =="
grep -c 'DNS_UPSTREAMS = \[("77.88.8.8", 53), ("8.8.8.8", 53), ("1.1.1.1", 53)\]' "$TMP/equasar_server.py" \
    | grep -q '^1$' && echo "апстримы 77.88.8.8 -> 8.8.8.8 -> 1.1.1.1: OK"
grep -c 'DNS_UPSTREAM_TIMEOUT = 1.2' "$TMP/equasar_server.py" \
    | grep -q '^1$' && echo "таймаут 1.2 с: OK"
grep -c 'threading.Thread(target=udp_answer, args=(udp, data, addr), daemon=True).start()' "$TMP/equasar_server.py" \
    | grep -q '^1$' && echo "поток на UDP-запрос: OK"

echo "== идемпотентность: повторное применение =="
"$APPLY" "$TMP/equasar_server.py"

echo "== негатив: применение к чужому файлу отклоняется =="
if "$APPLY" "$DIR/README.md" >/dev/null 2>&1; then
    echo "FAIL: apply принял файл с неверным sha256" >&2
    exit 1
else
    echo "чужой файл отклонён с ошибкой sha256: OK"
fi

[ "$(sha256sum "$TMP/equasar_server.py" | awk '{print $1}')" = "$PATCHED_SHA256" ]
echo "SELTEST PASS"
