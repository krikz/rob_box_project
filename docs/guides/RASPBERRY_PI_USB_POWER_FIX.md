# Raspberry Pi 5 - Увеличение USB тока для мощного БП

## Проблема

При питании Raspberry Pi 5 через GPIO пины от мощного БП (например, 30W / 6A), USB порты по умолчанию ограничены **600mA** вместо **1600mA**.

## Решение (БЕЗОПАСНОЕ)

### Шаг 1: Редактировать config.txt

```bash
sudo nano /boot/firmware/config.txt
```

Добавить в конец:

```ini
# ============================================================
# USB Power Configuration для мощного БП через GPIO
# БП: 30W через GPIO пины = ~6A при 5V
# Источник: docs/guides/POWER_MANAGEMENT.md
# ============================================================

# Разрешить полный USB ток (1600mA вместо 600mA)
usb_max_current_enable=1
```

### Шаг 2: Перезагрузить

```bash
sudo reboot
```

### Шаг 3: Проверить

```bash
vcgencmd get_config usb_max_current_enable
# Ожидается: usb_max_current_enable=1

lsusb
# Должны появиться все USB устройства
```

## Опционально: Настройка EEPROM

**Это НЕ обязательно!** Нужно только если хотите убрать GUI предупреждение "питание < 5A".

```bash
sudo rpi-eeprom-config --edit
```

Добавить в конец:

```ini
PSU_MAX_CURRENT=5000
```

Сохранить и перезагрузить.

## Что НЕ делать

❌ **НЕ добавляйте** `PSU_MAX_CURRENT=5000` без подготовки  
❌ **НЕ редактируйте EEPROM** если не уверены в БП  
❌ **НЕ используйте** значения > 5000 без специального оборудования

## Восстановление после ошибки

Если Raspberry Pi не загружается после изменения EEPROM:

1. Выключить питание
2. Извлечь SD карту
3. На другом компьютере:
   - Примонтировать boot раздел
   - Удалить файлы: `pieeprom.upd`, `pieeprom.sig`, `recovery.bin`
   - Проверить `config.txt` (должно быть только `usb_max_current_enable=1`)
4. Вставить SD карту обратно
5. Включить питание

## Требования к БП

Для `usb_max_current_enable=1` нужно **минимум 3-4A** стабильно:

- Raspberry Pi: ~2-3A под нагрузкой
- USB периферия: до 1.6A (при включенном `usb_max_current_enable=1`)
- **Итого**: 3.6-4.6A пиковое

Для 30W БП (6A @ 5V) - это **безопасно**.

## Ссылки

- [POWER_MANAGEMENT.md](POWER_MANAGEMENT.md) - Полная документация по питанию
- [Raspberry Pi Power](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#power-supply) - Официальная документация

---

**Автор**: rob_box_project  
**Дата**: 2025-10-13  
**Статус**: ✅ Проверено на практике
