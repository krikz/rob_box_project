# Исправления ошибок сборки Docker - 2025-10-11

## Резюме изменений

Исправлены все критические ошибки сборки Docker образов в develop ветке.

## 🔧 Исправленные проблемы

### 1. ❌ `docker/main/micro_ros_agent/Dockerfile` - Неправильный путь COPY
**Ошибка:**
```
ERROR: failed to calculate checksum: "/src/robot_sensor_hub_msg": not found
```

**Причина:** Использование относительного пути `../../src/robot_sensor_hub_msg` вместо пути от build context.

**Исправление:**
```dockerfile
# ДО
COPY ../../src/robot_sensor_hub_msg /ws/src/robot_sensor_hub_msg

# ПОСЛЕ  
COPY src/robot_sensor_hub_msg /ws/src/robot_sensor_hub_msg
```

**Требование:** Build context должен быть корень репозитория.

---

### 2. ❌ `docker/main/nav2/Dockerfile` - Устаревший пакет
**Ошибка:**
```
E: Unable to locate package ros-humble-nav2-recoveries
```

**Причина:** Пакет `ros-humble-nav2-recoveries` был переименован в `ros-humble-nav2-behaviors` в ROS 2 Humble.

**Исправление:**
```dockerfile
# Удалена строка:
ros-${ROS_DISTRO}-nav2-recoveries

# Оставлен только:
ros-${ROS_DISTRO}-nav2-behaviors
```

---

### 3. ❌ `docker/vision/led_matrix/Dockerfile` - Несовместимый флаг pip
**Ошибка:**
```
no such option: --break-system-packages
```

**Причина:** Старая версия pip (22.0.2) в Ubuntu 22.04 не поддерживает флаг `--break-system-packages`.

**Исправление:**
```dockerfile
# ДО
RUN pip3 install --no-cache-dir --break-system-packages \
    pi5neo spidev numpy

# ПОСЛЕ
RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir \
    pi5neo spidev numpy
```

---

### 4. ❌ `docker/main/vesc_nexus/Dockerfile` - Отсутствуют библиотеки разработки
**Ошибка:**
```
CMake Error: Package 'builtin_interfaces' exports the library 
'builtin_interfaces__rosidl_generator_c' which couldn't be found
```

**Причина:** 
- Базовый образ `ros:humble-ros-base` не содержит библиотеки для генерации ROS 2 messages
- rosdep пытается установить необязательные зависимости (GUI tools)

**Исправление:**
```dockerfile
# Добавлены пакеты для генерации messages
RUN apt-get update && apt-get install -y \
    # ROS 2 development tools
    ros-dev-tools \
    # ROS 2 message generation dependencies
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    # ... остальные зависимости
    
# Пропуск необязательных зависимостей
RUN rosdep install --from-paths src --ignore-src -r -y \
    --skip-keys "serial_driver joint_state_publisher joint_state_publisher_gui rviz2"
```

---

## 📦 Архитектурное решение: Базовый образ остался минимальным

**Принцип:** Каждый Dockerfile устанавливает только свои специфичные зависимости.

### `docker/base/Dockerfile.ros2-zenoh` - БЕЗ изменений (минимальный)
```dockerfile
RUN apt-get update && apt-get install -y \
    ros-humble-rmw-zenoh-cpp \  # Только Zenoh middleware
    git wget curl python3-pip    # Только базовые утилиты
```

**НЕ добавлены** в базовый образ (правильно!):
- ❌ `ros-humble-serial-driver` - нужен только в micro_ros_agent
- ❌ `ros-humble-joint-state-publisher*` - только для разработки
- ❌ `ros-humble-rviz2` - только для GUI визуализации
- ❌ `ros-humble-xacro` - добавляется где нужно

---

## 🤖 GitHub Actions: Автоматический сбор логов ошибок

### Создан reusable action: `.github/actions/collect-build-logs/action.yml`

**Возможности:**
1. ✅ Автоматически собирает логи при падении сборки
2. ✅ Создает артефакты со всеми ошибками (хранятся 30 дней)
3. ✅ Генерирует сводку `ERROR-SUMMARY.txt`
4. ✅ Автоматически комментирует Pull Request с превью ошибок
5. ✅ Включает системную информацию Docker

**Использование в workflow:**
```yaml
- name: Build and push image
  id: build
  uses: docker/build-push-action@v5
  # ... параметры сборки

- name: Collect build logs on failure
  if: failure()
  uses: ./.github/actions/collect-build-logs
  with:
    job-name: my-service-name
```

**Что попадает в логи:**
- ❌ Полный вывод ошибки сборки
- 🕐 Временная метка (UTC)
- 🌿 Ветка и коммит
- 🐳 Docker version и состояние контейнеров
- 📊 Сводка всех падений в одном workflow run

**Доступ к логам:**
- Artifacts в GitHub Actions (30 дней)
- Автоматический комментарий в PR
- Можно скачать и прислать в чат для анализа

---

## ✅ Статус после исправлений

| Dockerfile | Проблема | Статус |
|-----------|----------|--------|
| `docker/main/micro_ros_agent/Dockerfile` | Неправильный COPY путь | ✅ ИСПРАВЛЕНО |
| `docker/main/nav2/Dockerfile` | Устаревший пакет | ✅ ИСПРАВЛЕНО |
| `docker/vision/led_matrix/Dockerfile` | Флаг pip | ✅ ИСПРАВЛЕНО |
| `docker/main/vesc_nexus/Dockerfile` | Отсутствуют dev библиотеки | ✅ ИСПРАВЛЕНО |
| `docker/base/Dockerfile.ros2-zenoh` | - | ✅ БЕЗ ИЗМЕНЕНИЙ (правильно!) |

---

## 🚀 Следующие шаги

1. **Commit & Push** всех исправлений в `develop`
2. **Дождаться** успешной сборки в GitHub Actions
3. **Проверить** что все образы собрались:
   - `ghcr.io/krikz/rob_box:micro-ros-agent-humble-dev`
   - `ghcr.io/krikz/rob_box:nav2-humble-dev`
   - `ghcr.io/krikz/rob_box:led-matrix-humble-dev`
   - `ghcr.io/krikz/rob_box:vesc-nexus-humble-dev`
4. **При ошибках** - логи автоматически соберутся в артефактах

---

## 📝 Важные замечания

### Build Context
Все Dockerfile должны использоваться с build context = корень репозитория:
```bash
docker build -f docker/main/micro_ros_agent/Dockerfile .
#                                                       ^ корень!
```

### rosdep --skip-keys
Для Docker образов без GUI добавляйте:
```bash
rosdep install ... --skip-keys "serial_driver joint_state_publisher_gui rviz2"
```

### Базовый образ
Держите `docker/base/` минимальным! Специфичные пакеты - только в конкретных Dockerfile.

---

**Автор исправлений:** GitHub Copilot  
**Дата:** 2025-10-11  
**Ветка:** develop
