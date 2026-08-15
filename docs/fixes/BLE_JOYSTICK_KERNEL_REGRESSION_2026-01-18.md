# BLE Джойстик: Регрессия HID в ядре 6.14.0

**Дата:** 18 января 2026  
**Обновлено:** 15 августа 2026 (TASK-049, issue #816) — подтверждён root cause, добавлен фикс-патч  
**Статус:** ❌ ЗАБЛОКИРОВАНО багом в ядре (регрессия присутствует в 6.14.0-1019-raspi и mainline v6.19-r)  
**Устройство:** ExpressLRS Joystick (MAC: 8C:4F:00:C2:04:96)  
**Платформа:** Raspberry Pi 5, Ubuntu 25.04 Plucky, Kernel 6.14.0-raspi

## Описание проблемы

Bluetooth джойстик ExpressLRS успешно подключается на **dev машине** (ядро 6.8.0-generic), но **не работает на Raspberry Pi** (ядро 6.14.0-raspi) из-за регрессии HID over GATT.

### Симптомы

1. **Через bluetoothctl + HID подсистему:**
   - Джойстик подключается: `Connected: yes`
   - HID UUID присутствует: `00001812-0000-1000-8000-00805f9b34fb`
   - Ошибка ядра: `HID Information read failed: Request attribute has encountered an unlikely error`
   - `/dev/input/js0` устройство не создаётся
   - Ошибки в dmesg: `hid-generic: unexpected SMP command`

2. **Через Bleak напрямую (BLE):**
   - Соединение успешно: `✅ Connected!`
   - Найдены сервисы: Battery (180f), Device Info (180a), Generic Attr (1801)
   - **HID сервис (00001812) НЕ ВИДЕН** - скрыт/убит ядром
   - Нотификации не приходят даже от батарейки

## Причина

Ядро 6.14.0-raspi имеет **регрессию HID over GATT**:
- BlueZ пытается инициализировать HID профиль автоматически
- Ядро падает на `read_pnpid_cb()` с ошибкой "Request attribute has encountered an unlikely error"
- Соединение убивается во время обнаружения сервисов когда детектится HID UUID
- Затрагивает как встроенный Cypress BT (hci1), так и внешний USB BT (hci0, ASUS BT500 Realtek RTL8761BU)

### Root cause (подтверждён 15.08.2026, issue #816)

**Код:** `net/bluetooth/smp.c`, функция `smp_sig_channel()`:

```c
if (smp && !test_and_clear_bit(code, &smp->allow_cmd))
    goto drop;
```

**Механика:** когда периферийное устройство инициирует `SMP_CMD_SECURITY_REQ (0x0b)`
после уже установленного соединения, ядро отклоняет команду, если SMP-контекст
активен (`smp != NULL`) и бит команды не выставлен в `allow_cmd`. Соединение
убивается → HID-сервис (UUID 00001812) не инициализируется → `/dev/input/js0`
не создаётся. Ошибка в dmesg: `unexpected SMP command 0x0b` /
`hid-generic: unexpected SMP command`.

**Статус в апстриме:** регрессия присутствует в mainline **вплоть до v6.19-r**
(проверено по codebrowser.dev 15.08.2026 — код без исключения для
`SMP_CMD_SECURITY_REQ`). В Ubuntu plucky-updates кандидат по-прежнему
`6.14.0-1019-raspi` (фикс НЕ прилетел). Поведение отличается от других
Bluetooth-стеков (Android Fluoride разрешает SECURITY_REQ с активным контекстом).

**Фикс-патч (проверен сообществом, один лайнер):**

```c
-	if (smp && !test_and_clear_bit(code, &smp->allow_cmd))
+	if (smp && code != SMP_CMD_SECURITY_REQ && !test_and_clear_bit(code, &smp->allow_cmd))
 		goto drop;
```

Патч готов к применению: `scripts/kernel/bluetooth-smp-allow-security-req.patch`.
Применять при сборке ядра/module: `patch -p1 < bluetooth-smp-allow-security-req.patch`.

**Диагностика:** `scripts/diagnostics/check_ble_joystick.sh` — проверяет ядро,
`/dev/input/js*`, joydev, bluetoothctl и dmesg, выдаёт вердикт
`REGRESSION_PRESENT` / `NEEDS_JOYSTICK` / `OK`. Запуск с dev-машины:
`sshpass -p open ssh ros2@10.1.1.20 'bash -s' < scripts/diagnostics/check_ble_joystick.sh`

## Попытки решения ❌

### Железо
- ✅ Куплен внешний ASUS USB-BT500 (Realtek RTL8761BU)
- ❌ Не помогло - проблема в ядре, не железе

### Софт
- ✅ Реализован прямой BLE режим (библиотека Bleak)
- ✅ Обход Linux HID подсистемы
- ✅ Добавлен бесконечный цикл переподключения
- ✅ Отключён встроенный hci1, используется только hci0
- ✅ Попытка DisablePlugins=input (не поддерживается в BlueZ 5.79)
- ✅ Настроен disable_ertm=Y в bluetooth.conf
- ❌ HID сервис всё равно невидим из-за бага в ядре

### Изменённые конфигурации

**Файл:** `/etc/modprobe.d/bluetooth.conf`
```
options bluetooth disable_ertm=Y
```

**Код:** `src/rob_box_teleop/rob_box_teleop/joystick_control_node.py`
- Прямое BLE соединение (без зависимости от HID)
- Автоматическое переподключение каждые 5 секунд
- Логирование обнаружения сервисов

**Docker:** `docker/main/teleop/Dockerfile`
```dockerfile
RUN apt-get update && apt-get install -y bluez python3-pip python3-dbus
RUN pip3 install bleak==0.22.3 dbus-fast==2.39.6
```

## Рабочая конфигурация

**Dev машина:**
- Ubuntu 22.04 Jammy
- Ядро 6.8.0-generic
- ✅ `/dev/input/js0` создаётся
- ✅ Полная функциональность HID
- ✅ Управление роботом работает

**Raspberry Pi (СЛОМАНО):**
- Ubuntu 25.04 Plucky
- Ядро 6.14.0-raspi
- ❌ HID сервис скрыт
- ❌ Нет данных от джойстика

## Варианты решения

### Вариант 1: Откатить ядро (рекомендовался ранее)
> ⚠️ Обновление 15.08.2026: на Ubuntu 25.04 Plucky в plucky-updates доступны только
> ядра `6.14.0-*-raspi` (1005…1019) — все затронуты регрессией SMP. Откат до 6.8 LTS
> из штатных репозиториев Plucky невозможен (пакета нет). Полноценный откат требует
> внешнего репозитория/ручной установки ядра, что хуже, чем патч из Варианта 2.

### Вариант 2: Патч ядра (РЕКОМЕНДУЕТСЯ, обновление 15.08.2026)
Root cause подтверждён: `net/bluetooth/smp.c`, `smp_sig_channel()` — строгий дроп
`SMP_CMD_SECURITY_REQ` при активном SMP-контексте. Готовый фикс-патч:

```bash
# Патч: scripts/kernel/bluetooth-smp-allow-security-req.patch
cd <linux-source>
patch -p1 < /path/to/scripts/kernel/bluetooth-smp-allow-security-req.patch
```

Патч проверен сообществом (Victron, venus-6.12 branch, январь 2026) — устройства
BLE после него соединяются, спариваются и работают без "unexpected SMP command".
Пока фикс не попал в Ubuntu — это единственный способ вернуть BLE HID на 6.14.0-raspi.

### Вариант 3: Ждать фикса от разработчиков
Отслеживать:
- https://github.com/bluez/bluez/issues
- Патчи HID подсистемы Linux kernel
- Обновления ядра Ubuntu (кандидат 6.14.0-1019 на 15.08.2026 фикс НЕ содержит)

### Вариант 4: Альтернативный контроллер
Использовать USB джойстик или контроллер без HID over GATT

## Диагностические команды

```bash
# Проверить версию ядра
uname -r

# Тест джойстика на dev машине
sudo dmesg -w  # во время подключения
ls /dev/input/js*

# Проверить BT адаптеры
hciconfig -a

# Смотреть логи bluetoothd
sudo journalctl -u bluetooth -f

# Тест обнаружения BLE сервисов
docker exec teleop python3 -c "
import asyncio
from bleak import BleakClient
async def test():
    async with BleakClient('8C:4F:00:C2:04:96') as c:
        for svc in c.services:
            print(svc.uuid)
asyncio.run(test())
"
```

## Ссылки

**Коммиты:**
- 2cd6e5d - Бесконечный цикл переподключения BLE
- 1b103ff - Логирование обнаружения сервисов
- 3d0b882 - Отключение от bluetoothctl перед стартом

**Железо:**
- ASUS USB-BT500 (hci0: CC:28:AA:70:0F:52)
- Встроенный BT: Cypress (hci1: 88:A2:9E:03:1B:0D, отключён)

**Известные проблемы (исторические):**
- https://forums.raspberrypi.com/viewtopic.php?t=186580 (2017) - Такая же проблема BLE HID на Pi 3, BlueZ 5.23 не поддерживает HoG
- https://raspberrypi.stackexchange.com/questions/150551 (2025) - Проблемы USB HID gadget на Pi 5
- https://github.com/rikka-chunibyo/HIDPi - Pi как USB HID устройство (обратное направление, но подтверждает ограничения HID)

## Вывод

**Невозможно продолжить** с BLE джойстиком на текущем ядре. Варианты:
1. **Откатить ядро на 6.8** (лучший вариант)
2. Ждать фикса ядра
3. Использовать USB проводной контроллер
4. Использовать другую платформу робота (не Pi)

Разработка продолжится на dev машине пока ядро Pi не починят.
