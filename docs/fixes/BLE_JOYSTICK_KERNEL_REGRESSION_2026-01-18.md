# BLE Джойстик: Регрессия HID в ядре 6.14.0

**Дата:** 18 января 2026  
**Статус:** ❌ ЗАБЛОКИРОВАНО багом в ядре  
**Устройство:** ExpressLRS Joystick (MAC: 8C:4F:00:C2:04:96)  
**Платформа:** Raspberry Pi 4, Ubuntu 25.04 Plucky, Kernel 6.14.0-raspi

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

### Вариант 1: Откатить ядро (РЕКОМЕНДУЕТСЯ)
```bash
# Откат на 6.8 LTS
sudo apt install linux-image-6.8.0-raspi linux-headers-6.8.0-raspi
sudo reboot
```

### Вариант 2: Ждать фикса от разработчиков
Отслеживать:
- https://github.com/bluez/bluez/issues
- Патчи HID подсистемы Linux kernel
- Обновления ядра Ubuntu

### Вариант 3: Альтернативный контроллер
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
