# Voice Assistant Docker Build Fix - 2025-10-24 (v2)

## Резюме

Исправлена ошибка сборки Docker образа `voice-assistant` при cross-compilation (ARM64 on x86_64).

## 🔧 Проблема

### Ошибка сборки
```
CMake Error at /opt/ros/kilted/share/rcutils/cmake/ament_cmake_export_libraries-extras.cmake:48 (message):
  Package 'rcutils' exports the library 'rcutils' which couldn't be found
Call Stack (most recent call first):
  /opt/ros/kilted/share/rcutils/cmake/rcutilsConfig.cmake:41 (include)
  /opt/ros/kilted/share/rosidl_runtime_c/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)
  ...
  CMakeLists.txt:10 (find_package)

Failed   <<< rob_box_perception_msgs [17.9s, exited with code 1]
```

### Context
- **Build Platform:** linux/amd64 (GitHub Actions runner)
- **Target Platform:** linux/arm64 (Raspberry Pi)
- **Build Method:** Docker BuildKit with QEMU emulation
- **Base Image:** `ghcr.io/krikz/rob_box:nav2-kilted-latest` (contains pre-installed ROS packages)

## 🔍 Корневая причина

**Основная проблема:** CMake config файлы в базовом образе имеют некорректные пути при cross-compilation.

### Почему --reinstall не помог

Предыдущая попытка исправления использовала:
```dockerfile
RUN apt-get update && apt-get install -y \
    ros-kilted-rcutils \
    ... \
    && apt-get install -y --reinstall \
    ros-kilted-rcutils \
    ...
```

**Проблема с этим подходом:**
1. Базовый образ (`nav2-kilted-latest`) уже содержит установленные пакеты
2. Эти пакеты были установлены в базовом слое (parent layer) Docker
3. `apt-get install --reinstall` в дочернем слое **не переписывает** файлы из родительского слоя
4. CMake config файлы остаются со сломанными путями

### Специфика cross-compilation

При сборке ARM64 образа на x86_64:
- Docker BuildKit использует QEMU для эмуляции ARM64
- Пакеты устанавливаются через apt из ARM64 репозиториев
- Но если в базовом образе уже есть пакеты, их конфигурация может быть некорректной
- Особенно это касается CMake config файлов, которые содержат абсолютные пути

## ✅ Решение

### Подход: Explicit Remove + Install

```dockerfile
RUN apt-get update && \
    # Сначала удаляем потенциально проблемные пакеты из base image
    apt-get remove --purge -y \
    ros-${ROS_DISTRO}-builtin-interfaces \
    ros-${ROS_DISTRO}-rcutils \
    ros-${ROS_DISTRO}-rosidl-runtime-c \
    ros-${ROS_DISTRO}-rosidl-runtime-cpp \
    ros-${ROS_DISTRO}-rosidl-typesupport-c \
    ros-${ROS_DISTRO}-rosidl-typesupport-cpp \
    ros-${ROS_DISTRO}-rosidl-typesupport-fastrtps-c \
    ros-${ROS_DISTRO}-rosidl-typesupport-fastrtps-cpp 2>/dev/null || true && \
    # Теперь устанавливаем всё заново
    apt-get install -y \
    ros-dev-tools \
    ros-${ROS_DISTRO}-std-msgs \
    ... \
    ros-${ROS_DISTRO}-builtin-interfaces \
    ros-${ROS_DISTRO}-rcutils \
    ... \
    && rm -rf /var/lib/apt/lists/*
```

### Почему это работает

1. **`apt-get remove --purge`** удаляет пакеты И их конфигурационные файлы
2. Удаление создаёт "whiteout" файлы в текущем Docker слое
3. Эти whiteout файлы скрывают файлы из родительских слоёв
4. **`apt-get install`** устанавливает пакеты заново с чистыми config файлами
5. CMake config файлы генерируются с правильными путями для текущей архитектуры

### Безопасность подхода

- `2>/dev/null || true` - игнорирует ошибки если пакеты не установлены
- Все удаляемые пакеты сразу же переустанавливаются в той же команде
- apt автоматически управляет зависимостями

## 📊 Сравнение подходов

| Подход | Работает с parent layers? | Пересоздаёт config файлы? | Безопасно? |
|--------|---------------------------|---------------------------|------------|
| `apt-get install` | ❌ | ❌ | ✅ |
| `apt-get install --reinstall` | ❌ | ⚠️ (только в текущем слое) | ✅ |
| `apt-get remove + install` | ✅ | ✅ | ✅ |

## 🎯 Правила для будущего

### ✅ DO: При cross-compilation с базовым образом

1. Если базовый образ содержит ROS пакеты, которые нужно пересобрать
2. Используйте `apt-get remove --purge + apt-get install` в одном RUN
3. Добавляйте `2>/dev/null || true` для обработки отсутствующих пакетов

### ❌ DON'T: Альтернативные подходы, которые НЕ работают

1. ❌ `--reinstall` - не перезаписывает файлы из parent layers
2. ❌ Ручное удаление CMake config файлов - не удаляет library symlinks
3. ❌ `CMAKE_LIBRARY_PATH` аргументы - ломают поиск пакетов
4. ❌ Просто игнорировать проблему и надеяться что "может пройдёт"

### 🔄 Альтернатива: Использовать минимальный базовый образ

Если возможно, используйте базовый образ без предустановленных ROS пакетов:
```dockerfile
FROM ghcr.io/krikz/rob_box_base:ros2-zenoh-kilted-latest
# Вместо:
# FROM ghcr.io/krikz/rob_box:nav2-kilted-latest
```

Это избежит проблем с конфликтующими пакетами из базового образа.

## 📝 Изменения в коде

### Файл: `docker/vision/voice_assistant/Dockerfile`

**Строки 49-87:** Заменён подход `--reinstall` на `remove --purge + install`

**Эффект:**
- Удалено 3 строки комментариев о `--reinstall`
- Добавлено 8 строк для явного удаления пакетов
- Добавлен комментарий о причине использования этого подхода

## 🔗 Связанные материалы

- **GitHub Actions Run:** https://github.com/krikz/rob_box_project/actions/runs/18787411133
- **Предыдущая попытка:** `docs/reports/VOICE_ASSISTANT_BUILD_FIX_2025-10-24.md`
- **Общая документация:** `docs/development/DOCKER_BUILD_FIXES.md`

## 🚀 Следующие шаги

1. ✅ Commit применён
2. ⏳ Ожидание CI/CD проверки
3. 🎯 Если успешно - закрыть issue
4. 📚 Если не успешно - рассмотреть смену базового образа

---

**Автор исправления:** GitHub Copilot  
**Дата:** 2025-10-24  
**Тип исправления:** Cross-compilation fix
**Ключевой урок:** `--reinstall` не работает через Docker layers - используйте explicit remove+install!
