# Docker Build Fix Report - Voice Assistant Container

**Дата:** 18 октября 2025  
**Проблема:** Сборка Docker образа `voice-assistant` падала с ошибками CMake  
**Статус:** ✅ РЕШЕНО  
**Коммиты:** `6c9f77f`, `9961f56`, `9007d44`  

---

## 📋 Краткое описание

При попытке собрать Docker образ для voice-assistant контейнера возникала серия CMake ошибок, блокирующих деплой на Vision Pi робота. После анализа истории изменений и сравнения с работающими пакетами, были найдены и исправлены **3 критических проблемы**.

---

## 🔍 Проблема #1: Broken CMake Config Files

### Симптомы
```
CMake Error at /opt/ros/kilted/share/builtin_interfaces/cmake/
  ament_cmake_export_libraries-extras.cmake:48 (message):
  Package 'builtin_interfaces' exports the library
  'builtin_interfaces__rosidl_generator_c' which couldn't be found
```

### Диагностика
```bash
# Библиотека СУЩЕСТВУЕТ
ls -lh /opt/ros/kilted/lib/libbuiltin_interfaces__rosidl_generator_c.so
# -rw-r--r-- 1 root root 14K Aug 24 15:19 ...

# CMake config ВИДИТ её
grep "_exported_libraries" .../ament_cmake_export_libraries-extras.cmake
# set(_exported_libraries "builtin_interfaces__rosidl_generator_c;...")

# Но CMake НЕ МОЖЕТ найти при линковке ❌
```

### Причина
Base Docker образ (`ghcr.io/krikz/rob_box:nav2-kilted-latest`) содержал **сломанные CMake конфигурационные файлы** для ROS2 пакетов после обновлений.

### Решение #1 (Частичное)
Добавили переустановку пакетов в Dockerfile:
```dockerfile
RUN apt-get update && apt-get install -y \
    [новые пакеты] \
    && apt-get install -y --reinstall \
    ros-${ROS_DISTRO}-builtin-interfaces \
    ros-${ROS_DISTRO}-rcutils \
    ros-${ROS_DISTRO}-rosidl-runtime-c \
    ros-${ROS_DISTRO}-rosidl-runtime-cpp \
    ros-${ROS_DISTRO}-rosidl-typesupport-c \
    ros-${ROS_DISTRO}-rosidl-typesupport-cpp \
    ros-${ROS_DISTRO}-rosidl-typesupport-fastrtps-c \
    ros-${ROS_DISTRO}-rosidl-typesupport-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*
```

**Важно:** `--reinstall` должен быть в **том же RUN** что и `apt-get update`, чтобы apt cache был доступен.

**Результат:** Библиотеки переустановились, но CMake всё равно не мог их найти → **Проблема #2**.

---

## 🔍 Проблема #2: Python Package with C++ Dependencies

### Симптомы (после Решения #1)
```
CMake Error: Package 'builtin_interfaces' exports the library
  'builtin_interfaces__rosidl_generator_c' which couldn't be found
Call Stack:
  /opt/ros/kilted/share/sensor_msgs/cmake/...
  CMakeLists.txt:12 (find_package)
```

### Диагностика
Сравнение с другими CMakeLists.txt в проекте:

**rob_box_animations (НЕ РАБОТАЕТ):**
```cmake
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)      # ❌ Ищет C++ библиотеки!
find_package(std_msgs REQUIRED)         # ❌
find_package(std_srvs REQUIRED)         # ❌
```

**rob_box_description (РАБОТАЕТ):**
```cmake
find_package(ament_cmake REQUIRED)
# Всё! Никаких лишних find_package
```

**rob_box_perception_msgs (РАБОТАЕТ):**
```cmake
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)  # ✅ Только для messages!
```

### Причина
`rob_box_animations` - это **чисто Python пакет**, но `CMakeLists.txt` содержал:
```cmake
find_package(sensor_msgs REQUIRED)  # Говорит CMake: "найди C++ библиотеки"
```

Это вызывает:
1. `sensor_msgs` зависит от `builtin_interfaces`
2. CMake ищет `libbuiltin_interfaces__rosidl_generator_c.so`
3. В Docker окружении после `--reinstall` пути поиска сбиты
4. **ОШИБКА СБОРКИ**

### Решение #2 (ПРАВИЛЬНОЕ)
Убрали ненужные `find_package` из CMakeLists.txt:

```cmake
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
# УДАЛЕНО: find_package(sensor_msgs REQUIRED)
# УДАЛЕНО: find_package(std_msgs REQUIRED)
# УДАЛЕНО: find_package(std_srvs REQUIRED)
# Python зависимости объявлены в package.xml
```

**Почему это правильно:**
- **CMakeLists.txt** → Только для C++ кода и CMake инструментов
- **package.xml** → Для Python зависимостей через setuptools
- Python импорты (`from sensor_msgs.msg import Image`) работают через **setuptools**, а НЕ через CMake!

**Коммит:** `9961f56`

**Результат:** CMake больше не ищет C++ библиотеки → **Проблема #3**.

---

## 🔍 Проблема #3: Missing Launch Directory

### Симптомы (после Решения #2)
```
CMake Error at ament_cmake_symlink_install/ament_cmake_symlink_install.cmake:100:
  ament_cmake_symlink_install_directory() can't find
  '/ws/src/rob_box_animations/launch/'
```

### Диагностика
```bash
# На хост-машине директория ЕСТЬ
ls -la src/rob_box_animations/
# drwxrwxr-x  2 ros2 ros2 4096 Oct 11 13:41 launch ✅

# В Dockerfile НЕТ COPY для launch/
grep "COPY.*rob_box_animations" docker/vision/voice_assistant/Dockerfile
# COPY src/rob_box_animations/package.xml
# COPY src/rob_box_animations/CMakeLists.txt
# COPY src/rob_box_animations/animations
# COPY src/rob_box_animations/rob_box_animations
# COPY src/rob_box_animations/scripts
# ❌ НЕТ: COPY src/rob_box_animations/launch
```

### Причина
В коммите `56288a5` ("fix(build): fix voice assistant Docker build errors"):
- ✅ Убрали несуществующий `animation_manager_node.py`  
- ✅ Убрали несуществующую `config/` директорию
- ❌ **НЕ ПРОВЕРИЛИ** что `launch/` директория копируется в Docker!

CMakeLists.txt пытается установить launch файлы:
```cmake
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
```

Но директория отсутствует в Docker образе.

### Решение #3
Добавили COPY для launch директории:

```dockerfile
# Шаг 6: Копируем launch файлы и конфигурации
COPY src/rob_box_voice/launch /ws/src/rob_box_voice/launch
COPY src/rob_box_voice/config /ws/src/rob_box_voice/config
COPY src/rob_box_animations/launch /ws/src/rob_box_animations/launch  # ← ДОБАВЛЕНО
```

**Коммит:** `9007d44`

**Результат:** ✅ Docker build успешно завершается!

---

## 🎯 Итоговое решение

### Изменённые файлы

#### 1. `docker/vision/voice_assistant/Dockerfile`
```dockerfile
# Добавили --reinstall в том же RUN (Проблема #1)
RUN apt-get update && apt-get install -y \
    [packages] \
    && apt-get install -y --reinstall \
    ros-${ROS_DISTRO}-builtin-interfaces \
    [other ros packages] \
    && rm -rf /var/lib/apt/lists/*

# Добавили COPY launch/ (Проблема #3)
COPY src/rob_box_animations/launch /ws/src/rob_box_animations/launch
```

#### 2. `src/rob_box_animations/CMakeLists.txt`
```cmake
# Убрали ненужные find_package (Проблема #2)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
# УДАЛЕНО: sensor_msgs, std_msgs, std_srvs
```

---

## 📊 Timeline исправлений

```
15:00 - Обнаружена ошибка: builtin_interfaces не найден
15:30 - Попытка #1: Добавили --reinstall → Частично помогло
16:00 - Попытка #2: Добавили CMAKE_LIBRARY_PATH → Не помогло
16:30 - 🔍 АНАЛИЗ: Сравнили с другими CMakeLists.txt
17:00 - ✅ РЕШЕНИЕ #2: Убрали find_package для Python
17:10 - Новая ошибка: launch/ не найдена
17:15 - ✅ РЕШЕНИЕ #3: Добавили COPY launch/
17:30 - 🎉 BUILD SUCCESS!
```

---

## 💡 Уроки и Best Practices

### ❌ Анти-паттерны (что НЕ делать)

1. **Не добавлять `find_package()` для Python пакетов:**
   ```cmake
   # ❌ НЕПРАВИЛЬНО для Python-only пакета
   find_package(sensor_msgs REQUIRED)
   ```

2. **Не разделять `apt-get update` и `--reinstall`:**
   ```dockerfile
   # ❌ НЕПРАВИЛЬНО
   RUN apt-get install -y package1
   RUN apt-get install -y --reinstall package2  # apt cache уже удален!
   ```

3. **Не забывать копировать все директории:**
   ```dockerfile
   # ❌ Забыли launch/
   COPY src/package/scripts
   COPY src/package/config
   # launch/ отсутствует!
   ```

### ✅ Best Practices

1. **Python пакеты - минимум CMake:**
   ```cmake
   # ✅ ПРАВИЛЬНО
   find_package(ament_cmake REQUIRED)
   find_package(ament_cmake_python REQUIRED)
   find_package(rclpy REQUIRED)
   # Всё! Python зависимости в package.xml
   ```

2. **--reinstall в том же RUN:**
   ```dockerfile
   # ✅ ПРАВИЛЬНО
   RUN apt-get update && apt-get install -y \
       new-packages \
       && apt-get install -y --reinstall \
       broken-packages \
       && rm -rf /var/lib/apt/lists/*
   ```

3. **Checklist для COPY в Dockerfile:**
   ```bash
   # Проверяем что копируем ВСЕ директории из package
   ls -la src/package/
   # animations/ ✅
   # launch/ ✅
   # scripts/ ✅
   # rob_box_package/ ✅
   ```

4. **Используем install() для всех директорий:**
   ```cmake
   # Если директория в install(), она ДОЛЖНА быть COPY в Docker
   install(DIRECTORY
     launch/      # ← Должна быть COPY
     DESTINATION share/${PROJECT_NAME}/launch
   )
   ```

---

## 🔧 Диагностические команды

### Проверка библиотек ROS2
```bash
# Проверка наличия библиотеки
ls -lh /opt/ros/kilted/lib/libbuiltin_interfaces*.so*

# Проверка CMake config
grep "_exported_libraries" \
  /opt/ros/kilted/share/builtin_interfaces/cmake/ament_cmake_export_libraries-extras.cmake

# Проверка LD_LIBRARY_PATH
echo $LD_LIBRARY_PATH
```

### Проверка структуры пакета
```bash
# Все директории которые install()
grep "install(DIRECTORY" src/package/CMakeLists.txt

# Все COPY в Dockerfile
grep "COPY src/package" docker/.../Dockerfile

# Сравнение (должны совпадать!)
diff <(grep "install(DIRECTORY" CMakeLists.txt | cut -d/ -f1) \
     <(grep "COPY src/package" Dockerfile | awk '{print $2}' | cut -d/ -f3)
```

---

## 📚 Связанные документы

- [CI/CD Pipeline](../CI_CD_PIPELINE.md) - Автоматическая сборка образов
- [Vision Pi Containers Fix](VISION_PI_CONTAINERS_FIX_2025-10-13.md) - Предыдущие проблемы с контейнерами
- [ROS2 YAML Parser Issue](ROS2_YAML_PARSER_ISSUE_2025-10-15.md) - Другие ROS2 проблемы

---

## 📝 Коммиты

- `6c9f77f` - fix: добавить CMAKE_LIBRARY_PATH (частичное решение)
- `9961f56` - fix: убрать find_package для Python-only зависимостей ✅
- `9007d44` - fix: добавить COPY launch директории ✅

---

**Автор:** AI Assistant  
**Reviewer:** krikz  
**Теги:** `docker`, `cmake`, `ros2`, `build-fix`, `voice-assistant`
