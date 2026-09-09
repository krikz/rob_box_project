# Rob Box Teleoperation Package

Пакет для прямого управления роботом через Bluetooth джойстик с голосовой обратной связью.

## 🎮 Возможности

- **Автоматическое обнаружение джойстика** - робот сообщает о подключении
- **ARM/DISARM кнопки** - безопасная активация моторов с голосовым подтверждением
- **Прямое управление** - публикация в `/cmd_vel_joy` (приоритет 100 в twist_mux)
- **Голосовая обратная связь** - уведомления через TTS
- **Защита от drift** - настраиваемый deadzone
- **Ограничение скорости** - безопасные максимальные значения

## 📋 Требования

- ROS 2 Humble
- Bluetooth джойстик (протестировано с ExpressLRS Joystick)
- Устройство должно быть подключено как `/dev/input/js0`

## 🔌 Подключение джойстика

> **⚠️ Известная проблема**: BLE-джойстик заблокирован kernel bug в Linux 6.14.0-raspi.
> Bluetooth-соединение устанавливается, но `/dev/input/js0` не появляется.
> **Обходное решение**: использовать SBUS/serial подключение (физический кабель) или ждать исправления в следующих версиях ядра.

```bash
# На Main Pi проверить Bluetooth
bluetoothctl show

# Сканировать устройства
bluetoothctl scan on

# Подключить (замените MAC-адрес)
bluetoothctl pair 8C:4F:00:C2:04:96
bluetoothctl connect 8C:4F:00:C2:04:96
bluetoothctl trust 8C:4F:00:C2:04:96

# Проверить что джойстик определился
ls -la /dev/input/js*
jstest /dev/input/js0
```

## 🎛️ Маппинг кнопок (ExpressLRS по умолчанию)

### Оси (Axes):
- **Axis 1** - Pitch (вперёд/назад, linear.x)
- **Axis 3** - Yaw (поворот влево/вправо, angular.z)

### Кнопки (Buttons):
- **Button 4** - ARM (активация моторов)
- **Button 5** - DISARM (отключение моторов)

## ⚙️ Настройка

Конфигурация в файле: `config/joystick_params.yaml`

```yaml
joystick_control_node:
  ros__parameters:
    # Устройство
    device_path: "/dev/input/js0"
    device_name: "ExpressLRS Joystick"
    
    # Маппинг осей
    axis_linear_x: 1
    axis_angular_z: 3
    
    # Кнопки
    button_arm: 4
    button_disarm: 5
    
    # Ограничения скорости
    max_linear_speed: 0.5   # м/с
    max_angular_speed: 1.0  # рад/с
    
    # Deadzone (0.0-1.0)
    deadzone: 0.1
    
    # Голосовая обратная связь
    enable_voice_feedback: true
```

## 🏃 Запуск

### Локально (для тестирования):

```bash
# Собрать пакет
cd ~/rob_box_project
colcon build --packages-select rob_box_teleop --symlink-install

# Запустить ноду
source install/setup.bash
ros2 run rob_box_teleop joystick_control_node --ros-args \
  --params-file src/rob_box_teleop/config/joystick_params.yaml
```

### В Docker (продакшн):

```bash
# На Main Pi
cd ~/rob_box_project/docker/main

# Запустить teleop контейнер
docker-compose up -d teleop

# Логи
docker logs -f teleop
```

## 📊 ROS 2 Topics

### Подписывается на:
- `/joy` (sensor_msgs/Joy) - от joy_node

### Публикует в:
- `/cmd_vel_joy` (geometry_msgs/Twist) - команды скорости для twist_mux
- `/tts/speak` (std_msgs/String) - голосовые сообщения

## 🔄 Работа с twist_mux

Пакет публикует в `/cmd_vel_joy` с приоритетом **100** (высокий).

Иерархия приоритетов в twist_mux:
1. Emergency Stop (255) - аварийная остановка
2. **Joystick (100)** ← наш пакет
3. Web UI (50) - веб-интерфейс
4. Voice (25) - голосовые команды
5. Navigation (10) - автономная навигация

## 🎯 Сценарий работы

1. **Подключение джойстика**
   - Система обнаруживает `/dev/input/js0`
   - Робот говорит: "Джойстик подключен"

2. **Активация моторов**
   - Нажатие кнопки ARM (button 4)
   - Робот говорит: "Моторы активированы, готов к езде"
   - Начинается публикация команд в `/cmd_vel_joy`

3. **Управление**
   - Axis 1 (Pitch) - движение вперёд/назад
   - Axis 3 (Yaw) - поворот влево/вправо
   - Deadzone 0.1 предотвращает дрейф

4. **Отключение моторов**
   - Нажатие кнопки DISARM (button 5)
   - Робот говорит: "Моторы отключены"
   - Публикуется нулевая скорость

5. **Отключение джойстика**
   - Система обнаруживает пропажу устройства
   - Робот говорит: "Джойстик отключен"
   - Моторы автоматически деактивируются

## 🛠️ Troubleshooting

### Джойстик не обнаруживается

```bash
# Проверить устройства ввода
ls -la /dev/input/

# Проверить события
evtest

# Bluetooth статус
bluetoothctl show
bluetoothctl devices
```

### Неправильный маппинг кнопок

```bash
# Определить правильные номера
jstest /dev/input/js0

# Обновить config/joystick_params.yaml
```

### Не работает голос

Проверить что TTS сервис запущен на Vision Pi:
```bash
# На Vision Pi
docker ps | grep voice
docker logs voice-tts-node -f

# Проверить топик
ros2 topic echo /tts/speak
```

## 📝 Логи

```bash
# Docker логи
docker logs -f teleop

# ROS 2 логи
ros2 node info /joystick_control_node

# События джойстика
jstest --event /dev/input/js0
```

## 🔐 Безопасность

- Моторы активируются **только** после нажатия ARM
- При отключении джойстика моторы **автоматически** деактивируются
- Максимальная скорость ограничена в конфиге (по умолчанию 0.5 м/с)
- Deadzone предотвращает непреднамеренное движение

## 📦 Структура пакета

```
rob_box_teleop/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── README.md
├── config/
│   └── joystick_params.yaml
├── rob_box_teleop/
│   ├── __init__.py
│   └── joystick_control_node.py
└── resource/
    └── rob_box_teleop
```

## 🐳 Docker структура

```
docker/main/
├── teleop/
│   └── Dockerfile
├── config/teleop/
│   └── joystick_params.yaml
└── scripts/teleop/
    └── start_teleop.sh
```

## 🤝 Интеграция

Пакет интегрируется с:
- **twist_mux** - мультиплексирование команд
- **rob_box_voice** - TTS для голосовой обратной связи
- **ros2_control** - управление моторами через VESC

## 📚 См. также

- [twist_mux конфигурация](../../docker/main/config/twist_mux/twist_mux.yaml)
- [Docker deployment](../../docs/DEPLOYMENT_WORKFLOW.md)
- [ROS 2 joy package](https://github.com/ros-drivers/joystick_drivers)
