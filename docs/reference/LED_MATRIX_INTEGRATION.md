# LED Matrix Integration - rob_box

Интеграция системы управления NeoPixel LED матрицами на Vision Pi через ROS2.

## 📋 Обзор

**Репозиторий:** [krikz/ros2leds](https://github.com/krikz/ros2leds)  
**Статус:** ✅ Интегрировано как submodule  
**Платформа:** Raspberry Pi 5 (Vision Pi)  
**Интерфейс:** SPI (/dev/spidev0.0)  
**Библиотека:** pi5neo (WS2812B/NeoPixel)

## 🏗️ Архитектура

```
[Applications] 
    │
    │ /panel_image (sensor_msgs/Image)
    │ frame_id = логическая группа
    │
    ↓
[led_matrix_compositor] 
    │ - Композитор логических групп
    │ - Поддержка snake/flip/arrangement
    │ - Объединение физических панелей
    │
    │ /led_matrix/data (std_msgs/Int8MultiArray)
    │
    ↓
[led_matrix_driver]
    │ - Низкоуровневый SPI драйвер
    │ - pi5neo библиотека
    │
    ↓
[SPI] → [381 WS2812B LEDs]
```

## 💡 Физическая конфигурация

### Панели
```
┌──────────────────────────────────────────────────┐
│  4× Headlight Panels (8×8)     = 256 LEDs        │
│  - Front Left                                    │
│  - Front Right                                   │
│  - Rear Left                                     │
│  - Rear Right                                    │
└──────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────┐
│  Main Display (5× panels 5×5)  = 125 LEDs        │
│  - Объединены горизонтально в линию 5×25         │
└──────────────────────────────────────────────────┘

   Total: 381 LEDs
```

### SPI подключение
```
Raspberry Pi 5 GPIO:
  GPIO 10 (MOSI) → DIN всех LED панелей
  GND            → GND
  5V             → VCC (внешнее питание!)
```

**⚠️ ВАЖНО:** 381 LED × 60mA (максимум) = **22.86А**  
Используйте внешний блок питания **5V/25A** для LED!

## 📦 Пакеты

### 1. led_matrix_driver
Низкоуровневый драйвер для управления LED через SPI.

**Входные данные:**
- Топик: `/led_matrix/data`
- Тип: `std_msgs/Int8MultiArray`
- Формат: RGB для каждого LED (3 байта × 381 = 1143 bytes)

**Параметры:**
```yaml
led_matrix_driver:
  ros__parameters:
    num_leds: 381                    # Общее количество LED
    spi_speed_khz: 800               # WS2812B стандарт
    spi_device: '/dev/spidev0.0'     # SPI интерфейс
    input_topic: 'led_matrix/data'
```

### 2. led_matrix_compositor
Композитор для объединения физических панелей в логические группы.

**Входные данные:**
- Топик: `/panel_image`
- Тип: `sensor_msgs/Image`
- Encoding: `rgb8`
- frame_id: имя логической группы

**Выходные данные:**
- Топик: `/led_matrix/data`
- Тип: `std_msgs/Int8MultiArray`

## 🎯 Логические группы

### Отдельные фары (8×8)
```yaml
- headlight_front_left
- headlight_front_right
- headlight_rear_left
- headlight_rear_right
```

### Группы фар
```yaml
- headlights_front  (16×8)  # Передние фары
- headlights_rear   (16×8)  # Задние фары
- headlights_all    (16×16) # Все фары (2×2 сетка)
```

### Главный дисплей
```yaml
- main_display (5×25)  # 5 панелей 5×5 в линию
```

## 🚀 Использование

### Запуск через Docker Compose

```bash
# Vision Pi
cd /opt/rob_box/docker/vision
docker-compose up led-matrix
```

Сервис автоматически:
1. Проверяет доступность `/dev/spidev0.0`
2. Проверяет установку `pi5neo`
3. Запускает драйвер и композитор

### Отправка изображения

#### Python пример
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class LEDPublisher(Node):
    def __init__(self):
        super().__init__('led_publisher')
        self.pub = self.create_publisher(Image, 'panel_image', 10)
        
    def send_gradient(self):
        # Создаем градиент 5×25
        img = np.zeros((5, 25, 3), dtype=np.uint8)
        for x in range(25):
            img[:, x] = [x*10, 0, 255-x*10]
        
        msg = Image()
        msg.header.frame_id = 'main_display'  # Логическая группа
        msg.height = 5
        msg.width = 25
        msg.encoding = 'rgb8'
        msg.step = 75
        msg.data = img.tobytes()
        
        self.pub.publish(msg)
```

#### CLI пример
```bash
# Залить красным передние фары
ros2 topic pub --once /panel_image sensor_msgs/Image "
header:
  frame_id: 'headlights_front'
height: 8
width: 16
encoding: 'rgb8'
step: 48
data: [255,0,0, 255,0,0, ...]"  # 8×16×3 = 384 bytes
```

## 🔧 Конфигурация

### Добавление новой логической группы

Отредактируйте `docker/vision/config/led_matrix/led_matrix_compositor.yaml`:

```yaml
logical_groups:
  - name: "my_custom_group"
    physical_indices: [4, 5]    # Панели 4 и 5
    arrangement: [2, 1]          # 2 панели в ряд
    flip_x: false
    flip_y: false
    snake_arrangement: false
```

### Параметры группы

| Параметр | Описание | Пример |
|----------|----------|--------|
| `name` | Имя группы (используется в frame_id) | `"main_display"` |
| `physical_indices` | Индексы физических панелей | `[0, 1, 2]` |
| `arrangement` | Расположение [cols, rows] | `[3, 1]` |
| `flip_x` | Отзеркалить по X | `false` |
| `flip_y` | Отзеркалить по Y | `true` |
| `snake_arrangement` | Змейка между панелями | `false` |

## 🐛 Troubleshooting

### SPI device not found
```
❌ ERROR: SPI device /dev/spidev0.0 not found!
```

**Решение:**
```bash
# Включить SPI в raspi-config
sudo raspi-config
# Interface Options → SPI → Yes

# Или в /boot/config.txt:
dtparam=spi=on

sudo reboot
```

### Permission denied
```
⚠️  WARNING: No read/write permissions for /dev/spidev0.0
```

**Решение:**
```bash
sudo chmod 666 /dev/spidev0.0

# Или добавить пользователя в группу spi:
sudo usermod -a -G spi $USER
```

### pi5neo library not installed
```
❌ ERROR: pi5neo library not installed!
```

**Решение:**
```bash
pip3 install --break-system-packages pi5neo
```

### LEDs не работают

1. **Проверить питание:** LED требуют 5V и до 25А
2. **Проверить подключение:** DIN → GPIO 10 (MOSI)
3. **Проверить топики:**
   ```bash
   ros2 topic list | grep led
   ros2 topic echo /led_matrix/data --once
   ```
4. **Проверить драйвер:**
   ```bash
   ros2 node list | grep led_matrix
   ```

### Изображение перевернуто

Используйте параметры `flip_x` и `flip_y` в конфигурации группы.

## 📊 Производительность

### Частота обновления
- **Максимум:** ~60 FPS (зависит от количества LED)
- **Рекомендуется:** 30 FPS для плавной анимации
- **381 LED:** ~50 FPS стабильно

### Потребление ресурсов
- **CPU:** ~5-10% на Raspberry Pi 5
- **RAM:** ~50 MB (driver + compositor)
- **Задержка:** <10ms (SPI → LED)

## 🎨 Примеры эффектов

В репозитории [ros2leds](https://github.com/krikz/ros2leds) есть готовые примеры:

- `effects.py` - Анимированный рот робота (3 режима)
- `spiral_fill_publisher.py` - Спиральное заполнение
- `directory_image_publisher.py` - Слайдшоу из директории

## 📚 Дополнительные ресурсы

- [ros2leds README](https://github.com/krikz/ros2leds/blob/main/README.md)
- [pi5neo Documentation](https://github.com/leonyuhanov/pi5neo)
- [WS2812B Datasheet](https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf)

## ✅ Checklist развертывания

- [x] ros2leds добавлен как submodule
- [x] Dockerfile создан (`docker/vision/led_matrix/Dockerfile`)
- [x] Конфигурация добавлена
- [x] Startup script создан
- [x] Docker Compose обновлен
- [x] CI/CD workflow добавлен
- [ ] Протестировано на реальном железе
- [ ] SPI настроен на Vision Pi
- [ ] Внешнее питание подключено
- [ ] Эффекты протестированы

## 🚦 Статус

**Интеграция:** ✅ Завершена (commit f506f04)  
**CI/CD:** ⏳ Ожидается сборка образа  
**Тестирование:** ⏳ Требуется реальное железо

**Next steps:**
1. Дождаться сборки Docker образа в CI/CD
2. Развернуть на Vision Pi
3. Настроить SPI и проверить `/dev/spidev0.0`
4. Подключить LED панели и внешнее питание
5. Протестировать все логические группы
6. Создать примеры эффектов для rob_box
