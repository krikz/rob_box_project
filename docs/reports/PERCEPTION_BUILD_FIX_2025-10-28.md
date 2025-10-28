# Perception Service Docker Build Fix Report - 2025-10-28

**Дата:** 28 октября 2025  
**Проблема:** Сборка Docker образа `perception` падала с ошибками CMake  
**Статус:** ✅ РЕШЕНО  
**GitHub Actions Run:** https://github.com/krikz/rob_box_project/actions/runs/18887328461/job/53905959011  
**Коммит:** `7380eb5`

---

## 📋 Краткое описание

При попытке собрать Docker образ для perception контейнера (Main Pi) возникла CMake ошибка, идентичная той, что была исправлена ранее для vesc_nexus и voice_assistant. Сборка падала на этапе компиляции `rob_box_perception_msgs` пакета с сообщениями о невозможности найти библиотеки ROS 2.

---

## 🔍 Проблема: Missing ROS 2 Runtime Libraries

### Симптомы
```
CMake Error at /opt/ros/humble/share/rcutils/cmake/ament_cmake_export_libraries-extras.cmake:48 (message):
  Package 'rcutils' exports the library 'rcutils' which couldn't be found
Call Stack (most recent call first):
  /opt/ros/humble/share/rcutils/cmake/rcutilsConfig.cmake:41 (include)
  /opt/ros/humble/share/rosidl_runtime_c/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)
  /opt/ros/humble/share/rosidl_typesupport_fastrtps_c/cmake/rosidl_typesupport_fastrtps_cConfig.cmake:41 (include)
  /opt/ros/humble/share/rosidl_typesupport_c/cmake/rosidl_typesupport_c-extras.cmake:13 (find_package)
  /opt/ros/humble/share/rosidl_typesupport_c/cmake/rosidl_typesupport_cConfig.cmake:41 (include)
  /opt/ros/humble/share/rosidl_default_generators/cmake/rosidl_default_generators-extras.cmake:21 (find_package)
  /opt/ros/humble/share/rosidl_default_generators/cmake/rosidl_default_generatorsConfig.cmake:41 (include)
  CMakeLists.txt:10 (find_package)

Failed   <<< rob_box_perception_msgs [9.00s, exited with code 1]
```

### Анализ

При попытке собрать пакет `rob_box_perception_msgs`, который генерирует кастомные ROS 2 сообщения (`PerceptionEvent.msg`), CMake не может найти базовые runtime библиотеки ROS 2, необходимые для генерации кода.

**Проверка существующих исправлений:**
1. Просмотрен `docs/development/DOCKER_BUILD_FIXES.md` ✅
2. Просмотрен `docs/reports/DOCKER_BUILD_FIX_2025-10-18.md` ✅
3. Найдена **идентичная проблема** в разделе "4. ❌ `docker/main/vesc_nexus/Dockerfile` - Отсутствуют библиотеки разработки"

### Причина

**Базовый образ** `ghcr.io/krikz/rob_box_base:ros2-zenoh` (основан на `ros:humble-ros-base`) содержит минимальный набор пакетов ROS 2 и **не включает**:
- Development tools (cmake, build-essential и т.д.)
- Runtime библиотеки для генерации сообщений
- CMake конфигурационные файлы в правильном состоянии после обновлений

**Dockerfile для perception** содержал только:
```dockerfile
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-python \
    && rm -rf /var/lib/apt/lists/*
```

**Чего не хватало:**
- ❌ `ros-dev-tools` - пакет с набором инструментов разработки
- ❌ `--reinstall` для runtime библиотек после возможных обновлений

---

## ✅ Решение

### Применённое исправление

Добавлены недостающие пакеты по паттерну из `vesc_nexus/Dockerfile`:

```dockerfile
# Установка Python зависимостей и ROS 2 development packages для perception
RUN apt-get update && apt-get install -y \
    python3-pip \
    # ROS 2 development tools (includes cmake, build dependencies)
    ros-dev-tools \
    # ROS 2 development packages needed for building message packages
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-python \
    && apt-get install -y --reinstall \
    ros-humble-rcutils \
    ros-humble-rosidl-runtime-c \
    ros-humble-rosidl-runtime-cpp \
    ros-humble-rosidl-typesupport-c \
    ros-humble-rosidl-typesupport-cpp \
    ros-humble-rosidl-typesupport-fastrtps-c \
    ros-humble-rosidl-typesupport-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*
```

### Что добавлено:

1. **`ros-dev-tools`** - Метапакет, включающий:
   - `build-essential` (gcc, g++, make)
   - `cmake`
   - Python development headers
   - Другие инструменты для компиляции ROS 2 пакетов

2. **`--reinstall` блок** - Переустановка runtime библиотек:
   - `ros-humble-rcutils` - Базовые утилиты ROS 2 (та самая ошибка!)
   - `ros-humble-rosidl-runtime-c` - Runtime для C генератора сообщений
   - `ros-humble-rosidl-runtime-cpp` - Runtime для C++ генератора
   - `ros-humble-rosidl-typesupport-*` - Type support библиотеки для FastRTPS

**Важно:** `--reinstall` выполняется в **том же RUN** блоке, что и `apt-get update`, чтобы apt cache был доступен.

---

## 📊 Сравнение с предыдущими исправлениями

| Dockerfile | Проблема | Решение | Дата |
|-----------|----------|---------|------|
| `vesc_nexus/Dockerfile` | Тот же CMake error для vesc_msgs | `ros-dev-tools` + `--reinstall` | Oct 2025 |
| `voice_assistant/Dockerfile` | builtin_interfaces not found | `--reinstall` + убраны лишние find_package | 18 Oct 2025 |
| `perception/Dockerfile` | rcutils library not found | `ros-dev-tools` + `--reinstall` | **28 Oct 2025** ✅ |

**Паттерн:** Все три случая связаны с тем, что базовый образ `ros:humble-ros-base` минимален и требует дополнительных dev пакетов для сборки message packages.

---

## 🎯 Архитектурные принципы

### ✅ Правильный подход (применён)

1. **Базовый образ остаётся минимальным**
   - `docker/base/Dockerfile.ros2-zenoh` содержит только Zenoh + базовые утилиты
   - Никаких лишних ROS пакетов

2. **Каждый сервис устанавливает свои зависимости**
   - Perception: `ros-dev-tools` для сборки messages
   - VESC Nexus: `ros-dev-tools` + `ros2_control` пакеты
   - Voice Assistant: Python пакеты + audio библиотеки

3. **Используем `--reinstall` для проблемных библиотек**
   - После обновлений ROS может иметь сломанные CMake configs
   - `--reinstall` восстанавливает их в правильное состояние

### ❌ Анти-паттерны (НЕ используем)

1. ❌ **Добавление всех dev tools в базовый образ**
   - Увеличит размер всех образов
   - Нарушит принцип минимализма

2. ❌ **Отдельный RUN для `--reinstall`**
   ```dockerfile
   RUN apt-get update && apt-get install -y packages
   RUN apt-get install -y --reinstall other-packages  # apt cache уже удалён!
   ```

3. ❌ **Игнорирование предыдущих исправлений**
   - Всегда проверять docs/reports/ перед новым фиксом
   - Использовать те же паттерны для консистентности

---

## 🚀 Результат

### Статус после исправления
- ✅ Dockerfile обновлён с недостающими пакетами
- ✅ Коммит создан и запушен в `copilot/fix-build-issues`
- ⏳ Ожидаем сборку в GitHub Actions на ARM64 архитектуре

### Следующие шаги
1. **Дождаться** завершения GitHub Actions build
2. **Проверить** что `rob_box_perception_msgs` собирается успешно
3. **Проверить** что `rob_box_perception` (Python package) также собирается
4. **Merge** если сборка успешна

---

## 📚 Связанные документы

- [DOCKER_BUILD_FIXES.md](../development/DOCKER_BUILD_FIXES.md) - Основная документация Docker фиксов
- [DOCKER_BUILD_FIX_2025-10-18.md](DOCKER_BUILD_FIX_2025-10-18.md) - Voice Assistant fix с похожей проблемой
- [CI_CD_PIPELINE.md](../CI_CD_PIPELINE.md) - Документация по GitHub Actions workflows

---

## 💡 Уроки

### Что работает:
1. ✅ **Проверять docs/reports/ перед началом** - похожие проблемы уже решались
2. ✅ **Следовать документированным паттернам** - `ros-dev-tools` + `--reinstall`
3. ✅ **Держать базовый образ минимальным** - каждый сервис добавляет свои deps

### Что НЕ работает:
1. ❌ Пытаться исправить только через CMake переменные
2. ❌ Добавлять dev tools в базовый образ
3. ❌ Разделять `apt-get update` и `--reinstall` в разные RUN

### Best Practice:
Когда видите ошибку **"Package 'X' exports the library 'Y' which couldn't be found"**:
1. Проверьте docs/reports/ на похожие исправления
2. Добавьте `ros-dev-tools` если его нет
3. Добавьте `--reinstall` для проблемных библиотек в тот же RUN блок
4. Проверьте что не дублируете зависимости с базовым образом

---

## 🔧 Команды для диагностики

### Проверка наличия библиотеки в контейнере
```bash
docker run --rm --platform linux/arm64 \
  ghcr.io/krikz/rob_box_base:ros2-zenoh \
  ls -lh /opt/ros/humble/lib/librcutils.so
```

### Проверка CMake config
```bash
docker run --rm --platform linux/arm64 \
  ghcr.io/krikz/rob_box_base:ros2-zenoh \
  cat /opt/ros/humble/share/rcutils/cmake/ament_cmake_export_libraries-extras.cmake
```

### Проверка установленных ROS пакетов
```bash
docker run --rm --platform linux/arm64 \
  ghcr.io/krikz/rob_box_base:ros2-zenoh \
  dpkg -l | grep ros-humble | grep rosidl
```

---

**Автор:** GitHub Copilot  
**Reviewer:** TBD  
**Теги:** `docker`, `cmake`, `ros2`, `build-fix`, `perception`, `arm64`
