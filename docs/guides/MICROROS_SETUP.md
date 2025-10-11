# micro-ROS Setup для ESP32 Sensor Hub

## 📋 Оглавление
- [Обзор](#обзор)
- [Архитектура](#архитектура)
- [Установка](#установка)
- [Конфигурация](#конфигурация)
- [ESP32 Firmware](#esp32-firmware)
- [Использование](#использование)
- [Custom Messages/Services](#custom-messagesservices)
- [Troubleshooting](#troubleshooting)

---

## Обзор

**micro-ROS** позволяет интегрировать микроконтроллеры (ESP32, Teensy, Pi Pico) в ROS2 экосистему.

### Ключевые концепции

1. **micro-ROS Agent** - мост между ROS2 (DDS/Zenoh) и micro-ROS устройствами (XRCE-DDS)
2. **Transport** - способ связи (Serial UART, WiFi UDP, TCP, CAN)
3. **XRCE-DDS** - облегченный протокол для микроконтроллеров (не полноценный DDS)

### rob_box Sensor Hub

ESP32 микроконтроллер подключается к Main Pi через **UART (serial)** и предоставляет:

**Датчики (Publishers):**
- IMU (акселерометр, гироскоп, магнетометр)
- Кнопки (emergency stop, mode switch)
- Статусные данные (battery voltage, temperature)

**Актуаторы (Subscribers):**
- Addressable LED ring (статус индикация)
- Buzzer (звуковые сигналы)

**Сервисы:**
- Calibrate IMU
- Set LED color/pattern
- Read sensor config

---

## Архитектура

### Коммуникационная схема

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Main Raspberry Pi                            │
│                                                                       │
│  ┌─────────────────────┐          ┌─────────────────────────────┐   │
│  │ ROS2 Nodes          │          │  micro-ROS Agent            │   │
│  │ (Nav2, twist_mux,   │◄────────►│  (XRCE-DDS ↔ DDS bridge)    │   │
│  │  SLAM, etc.)        │   DDS    │                             │   │
│  └─────────────────────┘          └───────────────┬─────────────┘   │
│                                                    │ Serial UART     │
└────────────────────────────────────────────────────┼─────────────────┘
                                                     │ /dev/ttyUSB0
                                         115200 baud │ (8N1)
                                                     │
                                    ┌────────────────▼────────────────┐
                                    │      ESP32 Sensor Hub           │
                                    │  (micro-ROS firmware)           │
                                    │                                 │
                                    │  ┌──────────┐  ┌──────────┐    │
                                    │  │   IMU    │  │ Buttons  │    │
                                    │  │ MPU6050  │  │ GPIO     │    │
                                    │  └──────────┘  └──────────┘    │
                                    │                                 │
                                    │  ┌──────────┐  ┌──────────┐    │
                                    │  │ NeoPixel │  │ Buzzer   │    │
                                    │  │ LEDs     │  │ PWM      │    │
                                    │  └──────────┘  └──────────┘    │
                                    └─────────────────────────────────┘
```

### Отличия micro-ROS от полноценного ROS2

| Аспект | ROS2 (Main Pi) | micro-ROS (ESP32) |
|--------|----------------|-------------------|
| **Протокол** | DDS (RTPS) | XRCE-DDS |
| **Discovery** | Автоматический | Через Agent |
| **RAM** | > 512 MB | < 100 KB |
| **Flash** | > 1 GB | < 500 KB |
| **OS** | Linux | FreeRTOS / bare metal |
| **Языки** | C++, Python | C/C++ only |

---

## Установка

### Main Pi (micro-ROS Agent)

Agent уже интегрирован в `docker-compose.yaml`:

```yaml
services:
  micro-ros-agent:
    image: ghcr.io/krikz/rob_box:micro-ros-agent-humble-latest
    container_name: micro-ros-agent
    network_mode: host
    privileged: true
    environment:
      - MICROROS_SERIAL_PORT=/dev/ttyUSB0
      - MICROROS_BAUDRATE=115200
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0
    command: ["/scripts/start_micro_ros_agent.sh"]
```

**Запуск:**
```bash
cd docker/main
docker-compose up -d micro-ros-agent

# Проверка логов
docker logs -f micro-ros-agent
```

### ESP32 Development Setup

См. раздел [ESP32 Firmware](#esp32-firmware) ниже.

---

## Конфигурация

### Serial Port Configuration

**USB Serial (ESP32-WROOM/DevKit):**
```bash
# Обычно автоматически определяется как:
/dev/ttyUSB0  # или /dev/ttyACM0
```

**GPIO UART (Pi header pins):**
```bash
# Raspberry Pi GPIO 14/15 (TXD/RXD):
/dev/ttyAMA0

# Включить в /boot/config.txt:
enable_uart=1
dtoverlay=uart0

# Отключить console на UART:
sudo systemctl disable serial-getty@ttyAMA0.service
```

**Изменение порта в docker-compose:**
```yaml
environment:
  - MICROROS_SERIAL_PORT=/dev/ttyAMA0  # GPIO UART
  - MICROROS_BAUDRATE=115200
devices:
  - /dev/ttyAMA0:/dev/ttyAMA0
```

### Baudrate

**Стандартные скорости:**
- `115200` - по умолчанию (balance speed/reliability)
- `230400` - быстрее, может быть нестабильно
- `57600` - медленнее, более надежно

**Важно:** Baudrate должен совпадать на обеих сторонах (ESP32 firmware + agent config)!

---

## ESP32 Firmware

### Структура проекта (отдельный репозиторий)

```
rob_box_sensor_hub/
├── platformio.ini           # Build config (PlatformIO)
├── src/
│   ├── main.cpp             # Entry point
│   ├── microros_node.cpp    # micro-ROS setup
│   ├── imu_publisher.cpp    # IMU sensor
│   ├── button_publisher.cpp # Button events
│   ├── led_subscriber.cpp   # LED control
│   └── services.cpp         # ROS services
├── lib/
│   └── micro_ros_platformio/ # micro-ROS library
└── extra_packages/          # Custom messages (см. ниже)
```

### PlatformIO Setup

**platformio.ini:**
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino

lib_deps =
    https://github.com/micro-ROS/micro_ros_platformio
    adafruit/Adafruit MPU6050
    adafruit/Adafruit NeoPixel

build_flags =
    -D MICROROS_SERIAL_BAUDRATE=115200
    -D LED_PIN=5
    -D BUTTON_PIN=18
```

### Basic Code Structure

**main.cpp:**
```cpp
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Publishers
rcl_publisher_t imu_publisher;
rcl_publisher_t button_publisher;

// Subscribers
rcl_subscription_t led_subscriber;

// Services
rcl_service_t calibrate_service;

// Executor
rclc_executor_t executor;

void setup() {
    Serial.begin(115200);
    
    // micro-ROS transport setup
    set_microros_serial_transports(Serial);
    
    // ROS node initialization
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);
    
    rcl_node_t node;
    rclc_node_init_default(&node, "sensor_hub", "", &support);
    
    // Initialize publishers
    rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"
    );
    
    rclc_publisher_init_default(
        &button_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "buttons/emergency_stop"
    );
    
    // Initialize subscribers
    rclc_subscription_init_default(
        &led_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA),
        "leds/color"
    );
    
    // Create executor (2 subscriptions + timer)
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_subscription(&executor, &led_subscriber, 
                                    &led_msg, &led_callback, ON_NEW_DATA);
    
    // Setup hardware
    setupIMU();
    setupButtons();
    setupLEDs();
}

void loop() {
    // Process ROS callbacks
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    
    // Publish sensor data
    publishIMU();
    publishButtons();
    
    delay(10);
}
```

### Publishers Example

**IMU Publisher:**
```cpp
#include <sensor_msgs/msg/imu.h>
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;
sensor_msgs__msg__Imu imu_msg;

void setupIMU() {
    if (!mpu.begin()) {
        Serial.println("IMU init failed!");
    }
}

void publishIMU() {
    static unsigned long last_publish = 0;
    if (millis() - last_publish < 20) return;  // 50 Hz
    
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    
    // Fill message
    imu_msg.header.stamp.sec = millis() / 1000;
    imu_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
    imu_msg.header.frame_id.data = "imu_link";
    
    imu_msg.linear_acceleration.x = accel.acceleration.x;
    imu_msg.linear_acceleration.y = accel.acceleration.y;
    imu_msg.linear_acceleration.z = accel.acceleration.z;
    
    imu_msg.angular_velocity.x = gyro.gyro.x;
    imu_msg.angular_velocity.y = gyro.gyro.y;
    imu_msg.angular_velocity.z = gyro.gyro.z;
    
    // Publish
    rcl_publish(&imu_publisher, &imu_msg, NULL);
    last_publish = millis();
}
```

**Button Publisher:**
```cpp
#include <std_msgs/msg/bool.h>

#define BUTTON_PIN 18
std_msgs__msg__Bool button_msg;

void setupButtons() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void publishButtons() {
    static bool last_state = HIGH;
    bool current_state = digitalRead(BUTTON_PIN);
    
    // Publish on change
    if (current_state != last_state) {
        button_msg.data = (current_state == LOW);  // Active LOW
        rcl_publish(&button_publisher, &button_msg, NULL);
        last_state = current_state;
    }
}
```

### Subscribers Example

**LED Subscriber:**
```cpp
#include <std_msgs/msg/color_rgba.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 5
#define LED_COUNT 16

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
std_msgs__msg__ColorRGBA led_msg;

void setupLEDs() {
    strip.begin();
    strip.show();  // Initialize all off
}

void led_callback(const void *msg_in) {
    const std_msgs__msg__ColorRGBA *msg = 
        (const std_msgs__msg__ColorRGBA *)msg_in;
    
    uint8_t r = msg->r * 255;
    uint8_t g = msg->g * 255;
    uint8_t b = msg->b * 255;
    
    // Set all LEDs to color
    for (int i = 0; i < LED_COUNT; i++) {
        strip.setPixelColor(i, strip.Color(r, g, b));
    }
    strip.show();
}
```

### Services Example

**Calibrate IMU Service:**
```cpp
#include <std_srvs/srv/trigger.h>

rcl_service_t calibrate_service;
std_srvs__srv__Trigger_Request req_msg;
std_srvs__srv__Trigger_Response res_msg;

void calibrate_service_callback(const void *req, void *res) {
    std_srvs__srv__Trigger_Response *response = 
        (std_srvs__srv__Trigger_Response *)res;
    
    // Perform calibration
    bool success = mpu.calibrate();
    
    response->success = success;
    if (success) {
        response->message.data = "IMU calibrated";
        response->message.size = strlen(response->message.data);
    } else {
        response->message.data = "Calibration failed";
        response->message.size = strlen(response->message.data);
    }
}

// In setup():
rclc_service_init_default(
    &calibrate_service,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
    "calibrate_imu"
);

rclc_executor_add_service(&executor, &calibrate_service,
                          &req_msg, &res_msg,
                          calibrate_service_callback);
```

---

## Custom Messages/Services

### Создание custom interfaces

**1. На ROS2 стороне (Main Pi workspace):**

```bash
# Создать пакет интерфейсов
ros2 pkg create --build-type ament_cmake rob_box_interfaces

cd rob_box_interfaces
mkdir msg srv
```

**msg/SensorHubStatus.msg:**
```
# Статус ESP32 Sensor Hub
std_msgs/Header header
float32 battery_voltage  # Вольты
float32 temperature      # Цельсий
uint8 cpu_usage         # Проценты
bool imu_ok
bool buttons_ok
```

**srv/SetLEDPattern.srv:**
```
# LED animation pattern
string pattern  # "solid", "blink", "rainbow", "pulse"
std_msgs/ColorRGBA color
float32 speed   # Animation speed (Hz)
---
bool success
string message
```

**package.xml:**
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>

<depend>std_msgs</depend>
```

**CMakeLists.txt:**
```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorHubStatus.msg"
  "srv/SetLEDPattern.srv"
  DEPENDENCIES std_msgs
)
```

**Сборка:**
```bash
colcon build --packages-select rob_box_interfaces
source install/setup.bash
```

**2. Добавить в micro-ROS (ESP32):**

```bash
# Скопировать пакет в micro-ROS workspace
cd ~/microros_ws/src
cp -r /path/to/rob_box_interfaces .

# Пересобрать micro-ROS library
cd ..
ros2 run micro_ros_setup build_firmware.sh
```

**Или для PlatformIO:**

```bash
# В проект ESP32
mkdir -p extra_packages
cp -r /path/to/rob_box_interfaces extra_packages/

# Пересобрать static library
# (см. micro_ros_platformio документацию)
```

**3. Использование в ESP32 коде:**

```cpp
#include <rob_box_interfaces/msg/sensor_hub_status.h>
#include <rob_box_interfaces/srv/set_led_pattern.h>

rob_box_interfaces__msg__SensorHubStatus status_msg;
rob_box_interfaces__srv__SetLEDPattern_Request led_req;
rob_box_interfaces__srv__SetLEDPattern_Response led_res;

// Publisher
rclc_publisher_init_default(
    &status_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(rob_box_interfaces, msg, SensorHubStatus),
    "sensor_hub/status"
);

// Service
rclc_service_init_default(
    &led_service,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(rob_box_interfaces, srv, SetLEDPattern),
    "set_led_pattern"
);
```

---

## Использование

### Запуск системы

**1. Подключить ESP32 к Main Pi через USB**

```bash
# Проверить определение устройства
ls -l /dev/ttyUSB*
# Должен появиться /dev/ttyUSB0

# Проверить права доступа
sudo usermod -aG dialout $USER  # Добавить себя в группу
# Или в docker: privileged: true
```

**2. Запустить micro-ROS agent**

```bash
cd docker/main
docker-compose up -d micro-ros-agent

# Проверить подключение
docker logs -f micro-ros-agent
# Должно быть:
# ✅ Device: /dev/ttyUSB0
# 🚀 Agent running...
# [INFO] Client connected
```

**3. Проверить топики**

```bash
# На Main Pi или Desktop
ros2 topic list

# Должны появиться топики от ESP32:
# /imu/data
# /buttons/emergency_stop
# /sensor_hub/status

# Подписаться на данные
ros2 topic echo /imu/data
ros2 topic echo /buttons/emergency_stop
```

**4. Управление LED**

```bash
# Отправить цвет
ros2 topic pub /leds/color std_msgs/msg/ColorRGBA \
  "{r: 1.0, g: 0.0, b: 0.0, a: 1.0}"  # Красный

# Вызвать сервис
ros2 service call /set_led_pattern rob_box_interfaces/srv/SetLEDPattern \
  "{pattern: 'rainbow', speed: 2.0}"
```

### Integration с rob_box

**Emergency Stop через кнопку:**
```python
# В Python ноде
import rclpy
from std_msgs.msg import Bool, Twist

class EmergencyStopHandler(Node):
    def __init__(self):
        super().__init__('emergency_stop_handler')
        
        # Подписка на кнопку
        self.button_sub = self.create_subscription(
            Bool,
            '/buttons/emergency_stop',
            self.button_callback,
            10
        )
        
        # Публикация emergency команды в twist_mux
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel_emergency',  # Priority 255 в twist_mux
            10
        )
    
    def button_callback(self, msg):
        if msg.data:  # Кнопка нажата
            self.get_logger().warn('🚨 EMERGENCY STOP ACTIVATED!')
            
            # Остановить робота
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
            
            # Визуальная индикация (красный LED)
            # ... (publish to /leds/color)
```

**IMU Fusion с robot_localization:**
```yaml
# ekf_config.yaml
ekf_filter_node:
  ros__parameters:
    odom0: /wheel_odometry
    odom0_config: [true, true, false, ...]
    
    imu0: /imu/data  # ← От ESP32!
    imu0_config: [false, false, false,
                  false, false, true,  # yaw
                  false, false, false,
                  false, false, true,  # yaw_velocity
                  true, true, true]    # accelerations
```

---

## Troubleshooting

### Agent не видит устройство

**Проблема:** `Serial port /dev/ttyUSB0 not found`

**Решение:**
```bash
# 1. Проверить подключение ESP32
lsusb
# Должен быть: "Silicon Labs CP210x" или "CH340"

# 2. Проверить dmesg
dmesg | grep tty
# Найти актуальное имя устройства

# 3. Права доступа
ls -l /dev/ttyUSB0
# Если нет прав: sudo chmod 666 /dev/ttyUSB0

# 4. В docker-compose изменить device mapping
devices:
  - /dev/ttyUSB1:/dev/ttyUSB0  # Если реальное устройство ttyUSB1
```

### ESP32 не подключается к Agent

**Проблема:** `[ERROR] Connection timeout`

**Решение:**
```cpp
// В ESP32 коде добавить retry loop
void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    
    // Retry connection
    int retries = 0;
    while (retries < 10) {
        rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
        if (ret == RCL_RET_OK) {
            break;
        }
        retries++;
        delay(1000);
    }
}
```

**Проверить baudrate совпадает:**
```bash
# Agent
MICROROS_BAUDRATE=115200

# ESP32
Serial.begin(115200);  # Должно совпадать!
```

### Топики не появляются в ros2 topic list

**Проблема:** Agent подключен, но топики не видны

**Решение:**
```bash
# 1. Проверить ROS_DOMAIN_ID совпадает
echo $ROS_DOMAIN_ID  # На Main Pi
# В docker-compose должен быть тот же

# 2. Проверить RMW_IMPLEMENTATION
echo $RMW_IMPLEMENTATION
# Должен быть rmw_zenoh_cpp (как у остальных нод)

# 3. Проверить agent логи
docker logs micro-ros-agent
# Должно быть "Publisher created" для каждого топика
```

### Сообщения приходят медленно

**Проблема:** Latency > 100ms

**Решение:**
```cpp
// Уменьшить frequency в ESP32
void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));  // Было 100
    // ...
    delay(10);  // Уменьшить delay
}
```

**Увеличить baudrate:**
```ini
# platformio.ini
build_flags =
    -D MICROROS_SERIAL_BAUDRATE=230400  # Было 115200
```
```yaml
# docker-compose.yaml
environment:
  - MICROROS_BAUDRATE=230400
```

### Memory allocation errors на ESP32

**Проблема:** `[ERROR] Failed to allocate memory`

**Решение:**
```cpp
// Использовать static allocation
#define RCUTILS_LOG_MIN_SEVERITY RCUTILS_LOG_MIN_SEVERITY_INFO

// Ограничить размер executor
#define RCLC_EXECUTOR_MAX_HANDLES 5  // Меньше подписок

// Уменьшить размер message queue
rclc_publisher_init_best_effort(&publisher, ...);  // Вместо default
```

### Custom messages не компилируются

**Проблема:** `undefined reference to rob_box_interfaces`

**Решение:**
```bash
# 1. Убедиться что пакет в extra_packages/
ls extra_packages/rob_box_interfaces

# 2. Пересобрать micro-ROS library с custom packages
# Для PlatformIO:
pio run -t clean_microros
pio lib install

# Для micro-ROS setup:
ros2 run micro_ros_setup build_firmware.sh
```

---

## Дополнительные ресурсы

**Официальная документация:**
- https://micro.ros.org/
- https://github.com/micro-ROS/micro_ros_platformio
- https://github.com/micro-ROS/micro_ros_setup

**Tutorials:**
- [Articulated Robotics - micro-ROS Pi Pico](https://articulatedrobotics.xyz/)
- [micro-ROS First Application](https://micro.ros.org/docs/tutorials/core/first_application_linux/)

**rob_box specific:**
- `docker/main/micro_ros_agent/` - Agent Dockerfile
- `docker/main/scripts/micro_ros_agent/` - Startup scripts
- Sensor Hub repo: (TODO: создать отдельный репозиторий)

---

## Примеры кода

Полные примеры находятся в отдельном репозитории `rob_box_sensor_hub`:
- Publishers (IMU, buttons, battery)
- Subscribers (LEDs, buzzer)
- Services (calibration, config)
- Custom messages/services

```bash
git clone https://github.com/krikz/rob_box_sensor_hub.git
cd rob_box_sensor_hub
pio run  # PlatformIO build
pio run -t upload  # Flash ESP32
```
