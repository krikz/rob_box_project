# LED Panel SPI Buffer Limit Fix

**Date**: 2025-12-20  
**Component**: LED Matrix (Vision Pi)  
**Issue**: Only 171 of 253 LEDs working  

## Problem

При добавлении двух 8×8 LED панелей (panels 5 и 6), общее количество светодиодов стало 253:
- Main display: 125 LEDs (5× 5×5 panels)
- Front left panel: 64 LEDs (8×8)
- Front right panel: 64 LEDs (8×8)

После деплоя наблюдалась проблема:
- ✅ Main display: 125 LEDs работают (с артефактами)
- ❌ Front left panel: 64 LEDs полностью темные
- ⚠️ Front right panel: только 46 из 64 LEDs работают (последние 18 темные)

**Итого**: 125 + 46 = 171 LED работают вместо 253.

## Root Cause

Pi5Neo library использует SPI (`/dev/spidev0.0`) для управления WS2812B светодиодами.  
**Дефолтный SPI буфер в ядре Linux = 4096 байт**

Расчёт лимита:
```
4096 bytes / 24 bytes per LED (RGB) ≈ 170 LEDs
```

Это точно совпадает с наблюдаемой проблемой (171 LED работают).

## Solution

Увеличение SPI буфера до 32KB через параметр ядра `spidev.bufsiz=32768`:

```bash
# Backup original config
sudo cp /boot/firmware/cmdline.txt /boot/firmware/cmdline.txt.bak

# Add spidev.bufsiz parameter
sudo sed -i 's/$/ spidev.bufsiz=32768/' /boot/firmware/cmdline.txt

# Verify
cat /boot/firmware/cmdline.txt

# Reboot to apply
sudo reboot
```

### Расчёт нового лимита:
```
32768 bytes / 24 bytes per LED (RGB) = 1365 LEDs max
```

Это более чем достаточно для наших 253 LEDs.

## Configuration Changes

**File**: `/boot/firmware/cmdline.txt`

**Before**:
```
console=serial0,115200 multipath=off dwc_otg.lpm_enable=0 console=tty1 root=LABEL=writable rootfstype=ext4 rootwait fixrtc
```

**After**:
```
console=serial0,115200 multipath=off dwc_otg.lpm_enable=0 console=tty1 root=LABEL=writable rootfstype=ext4 rootwait fixrtc spidev.bufsiz=32768
```

## Verification

После перезагрузки Vision Pi:

1. Проверить Docker контейнеры:
   ```bash
   docker ps
   ```

2. Проверить логи LED matrix driver:
   ```bash
   docker logs led-matrix
   ```

3. Визуальная проверка:
   - Все 253 LEDs должны реагировать на команды
   - Левая панель (125-188) должна светиться
   - Правая панель (189-252) должна полностью работать

## References

- **Pi5Neo Documentation**: [Driving High LED Counts](https://github.com/vanshksingh/Pi5Neo#driving-high-led-counts)
- **Issue Discovery**: Direct hardware test показал ограничение на 171 LED
- **Applied on**: Vision Pi (10.1.1.21) - 2025-12-20

## Related Files

- `docker/vision/config/led_matrix/led_matrix_driver.yaml` - num_leds: 253
- `src/ros2leds/led_matrix_compositor/led_matrix_compositor/led_matrix_compositor.py` - physical panels configuration
- `test_led_direct.py` - diagnostic test script

## Notes

⚠️ **ВАЖНО**: Этот параметр должен быть применён на **Vision Pi** (10.1.1.21), так как именно там подключены LED панели через SPI.

📌 **Deployment**: При деплое на чистую систему не забыть добавить `spidev.bufsiz=32768` в `/boot/firmware/cmdline.txt` перед первым запуском LED matrix.
