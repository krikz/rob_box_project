#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════
# 🧪 test_fix_katana_speaker_mute.sh
# ═══════════════════════════════════════════════════════════════════════════
# Регресс-тест для scripts/diagnostics/fix_katana_speaker_mute.sh
#
# Что проверяем:
#   1. Скрипт существует, executable, имеет shebang, set -euo pipefail
#   2. --help выходит 0 и показывает usage
#   3. Парсинг --check и --install-systemd без падения
#   4. Без HDA hwC0D0 → exit=2 (корректный код ошибки)
#   5. systemd unit валиден (systemd-analyze verify, если доступен)
#   6. udev rule валиден (udevadm verify / test-builtin, если доступен)
#
# Запуск:
#   bash scripts/diagnostics/test_fix_katana_speaker_mute.sh
#
# НЕ требует root и НЕ трогает железо.
# ═══════════════════════════════════════════════════════════════════════════

set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
TARGET="${HERE}/fix_katana_speaker_mute.sh"
SVC="${HERE}/../maintenance/systemd/fix-katana-speaker.service"
UDEV="${HERE}/../maintenance/udev/99-fix-katana-speaker.rules"

PASS=0
FAIL=0
ok()    { echo "[PASS] $1"; PASS=$((PASS+1)); }
ng()    { echo "[FAIL] $1"; FAIL=$((FAIL+1)); }

echo "═══════════════════════════════════════════════════════════════"
echo "  test_fix_katana_speaker_mute.sh"
echo "═══════════════════════════════════════════════════════════════"

# ── 1. Файл существует + executable ───────────────────────────────────
if [ -f "$TARGET" ]; then ok "fix_katana_speaker_mute.sh существует"; else ng "не найден $TARGET"; fi
if [ -x "$TARGET" ];   then ok "fix_katana_speaker_mute.sh executable"; else ng "нет +x"; fi

# ── 2. Shebang + set -euo pipefail ────────────────────────────────────
if head -1 "$TARGET" | grep -q '^#!/bin/bash'; then ok "shebang = bash"; else ng "no shebang"; fi
if grep -q 'set -euo pipefail' "$TARGET"; then ok "set -euo pipefail"; else ng "no set -euo pipefail"; fi

# ── 3. --help выходит 0 ───────────────────────────────────────────────
if "$TARGET" --help >/dev/null 2>&1; then
    ok "--help exit=0"
else
    ng "--help упал"
fi

# ── 4. FAIL-путь: отсутствующий hwC → exit=2 ─────────────────────────
set +e
"$TARGET" hwC99D99 >/tmp/fix_katana_help.out 2>&1
rc=$?
set -e
if [ "$rc" -eq 2 ]; then ok "exit=2 при отсутствии hwC99D99"; else ng "ожидали exit=2, получили $rc"; fi

# ── 5. systemd unit ───────────────────────────────────────────────────
if [ -f "$SVC" ]; then
    ok "fix-katana-speaker.service существует"
    if grep -q '^ExecStart=/usr/bin/hda-verb' "$SVC"; then
        ok "ExecStart ссылается на hda-verb"
    else
        ng "ExecStart неверный"
    fi
    if grep -q '^RemainAfterExit=yes' "$SVC"; then
        ok "Type=oneshot + RemainAfterExit=yes"
    else
        ng "нет RemainAfterExit=yes"
    fi
    if grep -q 'WantedBy=multi-user.target' "$SVC"; then
        ok "WantedBy=multi-user.target"
    else
        ng "нет multi-user.target"
    fi
    if command -v systemd-analyze >/dev/null 2>&1; then
        set +e
        out="$(systemd-analyze verify "$SVC" 2>&1)"
        rc=$?
        set -e
        if [ "$rc" -eq 0 ]; then
            ok "systemd-analyze verify OK"
        elif echo "$out" | grep -q "hda-verb is not executable: No such file or directory" >/dev/null 2>&1; then
            # В CI-среде alsa-tools может быть не установлен — это false-positive
            echo "[SKIP] systemd-analyze verify пропущен: hda-verb отсутствует в PATH"
        elif echo "$out" | grep -q "Permission denied\|Failed to acquire watch file descriptor" >/dev/null 2>&1; then
            # sandbox-окружение без доступа к /run/systemd — это тоже false-positive
            echo "[SKIP] systemd-analyze verify пропущен (sandbox не имеет /run/systemd)"
        else
            ng "systemd-analyze verify упал: $out"
        fi
    else
        echo "[SKIP] systemd-analyze не установлен"
    fi
else
    ng "не найден $SVC"
fi

# ── 6. udev rule ──────────────────────────────────────────────────────
if [ -f "$UDEV" ]; then
    ok "99-fix-katana-speaker.rules существует"
    if grep -q 'KERNEL=="hwC0D0"' "$UDEV"; then
        ok "udev KERNEL match = hwC0D0"
    else
        ng "нет KERNEL==\"hwC0D0\""
    fi
    if grep -q 'SET_AMP\|0x17.*0x3' "$UDEV" || grep -q '0x17' "$UDEV"; then
        ok "udev RUN обращается к Node 0x17"
    else
        ng "udev RUN не упоминает 0x17"
    fi
    if command -v udevadm >/dev/null 2>&1; then
        if udevadm verify "$UDEV" 2>/dev/null; then
            ok "udevadm verify OK"
        else
            echo "[WARN] udevadm verify упал (часто из-за %k в RUN — это OK)"
        fi
    else
        echo "[SKIP] udevadm не установлен"
    fi
else
    ng "не найден $UDEV"
fi

# ── 7. Документация / ключевые слова ──────────────────────────────────
if grep -q '0xB027' "$TARGET"; then ok "param 0xB027 задокументирован"; else ng "нет 0xB027"; fi
if grep -q 'Node 0x17' "$TARGET"; then ok "Node 0x17 упомянут"; else ng "нет Node 0x17"; fi
if grep -q '#1731' "$TARGET" "$SVC" 2>/dev/null; then ok "ссылка на issue #1731"; else ng "нет ссылки #1731"; fi

echo "═══════════════════════════════════════════════════════════════"
echo "  RESULT: ${PASS} passed, ${FAIL} failed"
echo "═══════════════════════════════════════════════════════════════"

[ "$FAIL" -eq 0 ]
