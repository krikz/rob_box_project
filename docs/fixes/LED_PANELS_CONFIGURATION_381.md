# LED Panels Configuration - 381 LEDs

**Date**: 2025-12-20  
**Component**: LED Matrix System (Vision Pi)  
**Status**: ✅ Configured and Working

## Overview

Полная конфигурация всех LED панелей на роботе Rob Box.

## Hardware Configuration

### Physical LED Chain Layout

```
┌─────────────────────────────────────────────────────────┐
│                    LED Chain (SPI)                       │
├──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┤
│ P0   │ P1   │ P2   │ P3   │ P4   │ P5   │ P6   │ P7   │ P8   │
│ 5×5  │ 5×5  │ 5×5  │ 5×5  │ 5×5  │ 8×8  │ 8×8  │ 8×8  │ 8×8  │
│ 25   │ 25   │ 25   │ 25   │ 25   │ 64   │ 64   │ 64   │ 64   │
└──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
   Main Display (125)      Front (128)      Rear (128)
```

### LED Distribution

| Panel ID | Location | Size | LEDs | Offset Range | Logical Group |
|----------|----------|------|------|--------------|---------------|
| 0 | Main Display | 5×5 | 25 | 0-24 | main_display |
| 1 | Main Display | 5×5 | 25 | 25-49 | main_display |
| 2 | Main Display | 5×5 | 25 | 50-74 | main_display |
| 3 | Main Display | 5×5 | 25 | 75-99 | main_display |
| 4 | Main Display | 5×5 | 25 | 100-124 | main_display |
| 5 | Front Left Wheel | 8×8 | 64 | 125-188 | wheel_front_left |
| 6 | Front Right Wheel | 8×8 | 64 | 189-252 | wheel_front_right |
| 7 | Rear Left Wheel | 8×8 | 64 | 253-316 | wheel_rear_left |
| 8 | Rear Right Wheel | 8×8 | 64 | 317-380 | wheel_rear_right |

**Total: 381 LEDs**

### Logical Groups

```python
logical_groups = [
    'main_display':       5×25 pixels (125 LEDs) - panels [0,1,2,3,4]
    'wheel_front_left':   8×8 pixels  (64 LEDs)  - panel [5]
    'wheel_front_right':  8×8 pixels  (64 LEDs)  - panel [6]
    'wheel_rear_left':    8×8 pixels  (64 LEDs)  - panel [7]
    'wheel_rear_right':   8×8 pixels  (64 LEDs)  - panel [8]
]
```

## Software Configuration

### Driver Configuration

**File**: `docker/vision/config/led_matrix/led_matrix_driver.yaml`

```yaml
led_matrix_driver:
  ros__parameters:
    num_leds: 381
    spi_speed_khz: 800
    spi_device: '/dev/spidev0.0'
    brightness: 25  # 0-255 (10% brightness)
```

### Compositor Configuration

**File**: `src/ros2leds/led_matrix_compositor/led_matrix_compositor/led_matrix_compositor.py`

- Physical panels: 9 panels total (5× 5×5, 4× 8×8)
- Logical groups: 5 groups (main + 4 wheels)
- Snake connection: enabled for all panels
- Total buffer size: 381 × 3 bytes = 1143 bytes RGB data

### Animation Support

**File**: `src/rob_box_animations/rob_box_animations/frame_renderer.py`

```python
PANEL_SIZES = {
    'main_display': (25, 5),        # 5×25 horizontal display
    'wheel_front_left': (8, 8),     # Front left wheel
    'wheel_front_right': (8, 8),    # Front right wheel
    'wheel_rear_left': (8, 8),      # Rear left wheel
    'wheel_rear_right': (8, 8),     # Rear right wheel
}
```

## SPI Buffer Configuration

### Linux Kernel Parameter

**File**: `/boot/firmware/cmdline.txt` (Vision Pi)

```
spidev.bufsiz=32768
```

### Buffer Capacity

- **Default buffer**: 4096 bytes → ~170 LEDs (was limiting to 171 LEDs)
- **New buffer**: 32768 bytes → ~1365 LEDs (8× increase)
- **Current usage**: 381 LEDs × 24 bytes = 9144 bytes (28% utilization)
- **Headroom**: ~984 LEDs available

### SPI Transmission

```
381 LEDs × 24 bytes per LED = 9144 bytes per frame
Frame rate: ~30 FPS
Data rate: 274 KB/s
SPI speed: 800 kHz (sufficient for WS2812B timing)
```

## ROS 2 Topics

### Compositor Subscriptions

```
/led_matrix/main_display        (sensor_msgs/Image)  5×25 pixels
/led_matrix/wheel_front_left    (sensor_msgs/Image)  8×8 pixels
/led_matrix/wheel_front_right   (sensor_msgs/Image)  8×8 pixels
/led_matrix/wheel_rear_left     (sensor_msgs/Image)  8×8 pixels
/led_matrix/wheel_rear_right    (sensor_msgs/Image)  8×8 pixels
```

### Compositor Publication

```
/led_matrix/data                (std_msgs/Int8MultiArray)  1143 bytes RGB
```

### Driver Subscription

```
/led_matrix/data                (std_msgs/Int8MultiArray)  → SPI transmission
```

## Power Requirements

### Current Draw Estimation

```
Per LED maximum: 60 mA (all channels at 100%)
381 LEDs × 60 mA = 22.86 A maximum theoretical

With brightness=25 (10%):
381 LEDs × 60 mA × 0.1 = 2.3 A typical
```

### Voltage

- WS2812B nominal: 5V ± 0.5V
- Power supply: 5V/5A (sufficient for current config)

## Testing

### Direct Hardware Test

**Script**: `test_led_direct.py`

```bash
# On Vision Pi (after stopping LED container)
docker compose stop led-matrix
python3 test_led_direct.py
```

Tests all 381 LEDs sequentially with running light pattern.

### Verification Commands

```bash
# Check compositor initialization
docker logs led-matrix 2>&1 | grep "Configured"
# Expected: "Configured 9 physical panels with total 381 LEDs"

# Monitor LED data publication
ros2 topic hz /led_matrix/data
# Expected: ~30 Hz

# Check buffer size
ros2 topic echo /led_matrix/data --once | wc -c
# Expected: ~1200 bytes (381 × 3 + overhead)
```

## Deployment

### Vision Pi Setup

1. **SPI buffer** (one-time setup):
   ```bash
   sudo sed -i 's/$/ spidev.bufsiz=32768/' /boot/firmware/cmdline.txt
   sudo reboot
   ```

2. **Code deployment** (via GitHub Actions):
   - Push to `develop` branch
   - GitHub Actions builds and pushes Docker images
   - SSH to Vision Pi and pull images
   - Restart LED containers

3. **Manual deployment**:
   ```bash
   cd ~/rob_box_project/docker/vision
   ./scripts/update_and_restart.sh
   ```

## Troubleshooting

### Symptoms: Some LEDs not working

**Cause**: SPI buffer too small  
**Solution**: Increase `spidev.bufsiz` in `/boot/firmware/cmdline.txt`

### Symptoms: Flickering or artifacts

**Possible causes**:
1. Power supply insufficient → upgrade to higher amperage
2. SPI speed too fast → reduce `spi_speed_khz` in driver config
3. Wiring issues → check data line signal quality

### Symptoms: Specific panel not lighting

**Debug**:
1. Check compositor logs for panel mapping
2. Verify physical panel index in chain
3. Test with direct hardware script
4. Check power to affected panel

## References

- **Pi5Neo Library**: https://github.com/vanshksingh/Pi5Neo
- **SPI Buffer Fix**: `docs/fixes/LED_PANEL_SPI_BUFFER_FIX.md`
- **WS2812B Datasheet**: Timing requirements for 800 kHz SPI
- **Raspberry Pi 5 SPI**: GPIO 10 (MOSI) on 40-pin header

## History

- **2025-12-19**: Initial 125 LEDs (main display only)
- **2025-12-20**: Added front panels → 253 LEDs (hit buffer limit at 171)
- **2025-12-20**: Fixed SPI buffer → all 253 LEDs working
- **2025-12-20**: Added rear panels → **381 LEDs total** ✅

## Notes

✅ All 381 LEDs tested and working  
✅ SPI buffer sufficient for current + future expansion  
✅ Power supply adequate at 10% brightness  
⚠️ Consider higher amperage PSU if increasing brightness >25%  
📌 Monitor SD card health on Vision Pi (known issues with I/O errors)
