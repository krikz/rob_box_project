#!/bin/bash
# =============================================================================
# check_ble_joystick.sh — диагностика BG-5: BLE joystick заблокирован
# регрессией ядра Linux 6.14.0-raspi (HID over GATT / SMP).
#
# Проблема (подтверждено 15.08.2026, issue #816):
#   kernel net/bluetooth/smp.c, smp_sig_channel():
#     if (smp && !test_and_clear_bit(code, &smp->allow_cmd)) goto drop;
#   Дропает SMP_CMD_SECURITY_REQ (0x0b) при активном SMP-контексте → HID
#   over GATT соединение убивается, /dev/input/js0 не создаётся.
#   Регрессия присутствует в mainline вплоть до v6.19-r (codebrowser.dev).
#
# Скрипт читает состояние робота (только чтение, sudo не требуется для
# большинства проверок) и печатает вердикт:
#   OK                — ядро не из списка затронутых ИЛИ js0 присутствует
#   REGRESSION_PRESENT — ядро 6.14.0-raspi и js0 отсутствует (BG-5 активна)
#   NEEDS_JOYSTICK     — ядро затронуто, js0 нет, джойстик не подключён
#
# Использование:
#   на роботе:          ./check_ble_joystick.sh
#   с dev-машины:       sshpass -p open ssh ros2@10.1.1.20 'bash -s' < ./check_ble_joystick.sh
# =============================================================================
set -u

VERDICT="UNKNOWN"
KERNEL="$(uname -r 2>/dev/null || echo unknown)"
JOYSTICK_MAC="8C:4F:00:C2:04:96"   # ExpressLRS Joystick

echo "============================================="
echo " BLE Joystick Diagnostic (BG-5, issue #816)"
echo "============================================="
echo "Kernel:            ${KERNEL}"
echo "Hostname:          $(hostname 2>/dev/null || echo unknown)"

# 1. Ядро в списке затронутых?
AFFECTED="no"
case "$KERNEL" in
    6.14.0-*raspi) AFFECTED="yes" ;;
esac
echo "Kernel affected:   ${AFFECTED}  (6.14.0-raspi = HID-over-GATT SMP regression)"

# 2. /dev/input/js*
JS_DEV=""
for dev in /dev/input/js*; do
    [ -e "$dev" ] && JS_DEV="${JS_DEV} $dev"
done
if [ -n "$JS_DEV" ]; then
    echo "Joystick devices:  FOUND:${JS_DEV}"
else
    echo "Joystick devices:  none (/dev/input/js* отсутствует)"
fi

# 3. joydev модуль
if lsmod 2>/dev/null | grep -q '^joydev'; then
    echo "joydev module:     loaded"
else
    joydev_mod="$(find "/lib/modules/${KERNEL}/kernel/drivers/input" -maxdepth 1 -name 'joydev.ko*' 2>/dev/null | head -1)"
    echo "joydev module:     NOT loaded (но модуль есть: ${joydev_mod:-n/a})"
fi

# 4. Bluetooth контроллеры и устройство
echo "BT controllers:"
bluetoothctl list 2>/dev/null | sed 's/^/  /' || echo "  (bluetoothctl недоступен)"

BT_INFO="$(bluetoothctl info "$JOYSTICK_MAC" 2>/dev/null || true)"
if [ -n "$BT_INFO" ]; then
    echo "Joystick ($JOYSTICK_MAC):"
    printf '%s\n' "$BT_INFO" | grep -E "Name:|Connected:|Paired:|Bonded:" | sed 's/^/  /'
    if printf '%s\n' "$BT_INFO" | grep -q "Connected: yes"; then
        echo "  → джойстик ПОДКЛЮЧЁН на уровне BT, но js0 $( [ -n "$JS_DEV" ] && echo 'есть' || echo 'НЕТ — регрессия ядра' )"
    fi
else
    echo "Joystick ($JOYSTICK_MAC): не найден в bluetoothctl devices"
fi

# 5. dmesg: ошибки SMP/HID (требует прав; показываем если доступно)
DMESG_HINT=""
if command -v dmesg >/dev/null 2>&1 && dmesg 2>/dev/null | grep -qiE "unexpected SMP command|HID Information read failed|hid-generic.*SMP"; then
    DMESG_HINT="yes"
    echo "dmesg:             обнаружены ошибки SMP/HID (регрессия активна):"
    dmesg 2>/dev/null | grep -iE "unexpected SMP command|HID Information read failed|hid-generic.*SMP" | tail -5 | sed 's/^/  /'
else
    echo "dmesg:             ошибок SMP/HID не видно (нет прав или джойстик не подключался)"
fi

# 6. Вердикт
if [ "$AFFECTED" = "yes" ] && [ -z "$JS_DEV" ]; then
    if [ "$DMESG_HINT" = "yes" ]; then
        VERDICT="REGRESSION_PRESENT"
    else
        VERDICT="NEEDS_JOYSTICK"
    fi
elif [ -n "$JS_DEV" ]; then
    VERDICT="OK"
else
    VERDICT="OK"
fi

echo "============================================="
echo "VERDICT: ${VERDICT}"
echo "============================================="
echo ""
echo "Если REGRESSION_PRESENT / NEEDS_JOYSTICK — варианты (issue #816):"
echo "  1) Ядро 6.14.0-raspi: регрессия SMP в net/bluetooth/smp.c."
echo "     Фикс-патч: scripts/kernel/bluetooth-smp-allow-security-req.patch"
echo "     (применить при сборке ядра/module, см. docs/fixes/BLE_JOYSTICK_KERNEL_REGRESSION_2026-01-18.md)"
echo "  2) Ожидать ядро с фиксом от Ubuntu (пока НЕ в plucky-updates, кандидат 6.14.0-1019)."
echo "  3) Перейти на USB-адаптер джойстика (жёсткое решение)."

exit 0
