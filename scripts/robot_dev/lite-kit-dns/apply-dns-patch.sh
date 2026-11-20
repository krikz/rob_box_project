#!/usr/bin/env bash
# Применяет DNS-патч к equasar_server.py стенда lite-kit (issue #4553).
#
# Что чинит патч: встроенный DNS сервера обрабатывал UDP-запросы в одном
# потоке, а первый апстрим 1.1.1.1 молчал (таймаут 2 с) — под флудом имён
# от роутера очередь росла и quasar.* умирали по таймауту. Патч: отдельный
# поток на UDP-запрос, апстримы 77.88.8.8 -> 8.8.8.8 -> 1.1.1.1,
# таймаут 1.2 с.
#
# Проверено на lite-kit (github.com/krevetkqqq/lite-kit, MIT) @ 8a82ff3.
# Применять из Git Bash: ./apply-dns-patch.sh /c/lite-kit/scripts/equasar_server.py
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PATCH_FILE="$DIR/equasar_server.py.dns.patch"

# Исходник (CRLF, git clone на Windows; lite-kit @ 8a82ff3), к которому применим патч.
SOURCE_SHA256="d08942a1627cbaa576404a303d548b623a39135500d08978893ef4833a24e6b8"
# Результат после применения (проверенный на стенде файл от 08.11.2026).
PATCHED_SHA256="87bd82bc1c3235be4be6b2838f17410573276078d41d9551ee47bfd5e16c8d31"

usage() {
    echo "Использование: $0 <путь к equasar_server.py стенда lite-kit>" >&2
    exit 2
}

[ $# -eq 1 ] || usage
TARGET="$1"

[ -f "$TARGET" ] || { echo "ERROR: файл не найден: $TARGET" >&2; exit 1; }
[ -f "$PATCH_FILE" ] || { echo "ERROR: не найден $PATCH_FILE" >&2; exit 1; }

actual="$(sha256sum "$TARGET" | awk '{print $1}')"
if [ "$actual" = "$PATCHED_SHA256" ]; then
    echo "OK: $(basename "$TARGET") уже пропатчен (sha256 совпал с результатом)."
    exit 0
fi
if [ "$actual" != "$SOURCE_SHA256" ]; then
    echo "ERROR: sha256 $(basename "$TARGET") = $actual" >&2
    echo "ERROR: ожидался исходник $SOURCE_SHA256 (lite-kit @ 8a82ff3, CRLF)" >&2
    echo "ERROR: патч НЕ применён. Исходник изменился — сверьте дифф руками." >&2
    exit 1
fi

# --binary: иначе patch на Windows перепишет файл с LF и хеш результата не сойдётся.
if ! patch --binary --forward --batch "$TARGET" < "$PATCH_FILE"; then
    echo "ERROR: patch вернул неудачу. Проверьте $(basename "$TARGET") и *.rej рядом;" >&2
    echo "ERROR: при частичном наложении откатите файл из git стенда." >&2
    exit 1
fi

result="$(sha256sum "$TARGET" | awk '{print $1}')"
if [ "$result" != "$PATCHED_SHA256" ]; then
    echo "ERROR: после патча sha256 = $result, ожидался $PATCHED_SHA256." >&2
    echo "ERROR: файл ближе к оригиналу не вернуть скриптом — восстановите из git." >&2
    exit 1
fi

echo "OK: DNS-патч применён к $TARGET"
echo "OK: sha256 результата = $result"
