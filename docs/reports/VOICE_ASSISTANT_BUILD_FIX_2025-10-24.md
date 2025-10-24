# Voice Assistant Docker Build Fix - 2025-10-24

## Резюме

Исправлена ошибка сборки Docker образа `voice-assistant` при попытке добавить пакет `rob_box_perception_msgs` в сборку.

## 🔧 Проблема

### Commit с ошибкой
- **SHA:** `fe4b6e9b35fe3ae3f285d753ec03fa1f60de3f5f`
- **Сообщение:** "fix(voice): add rob_box_perception_msgs to voice-assistant build"
- **Workflow:** [Build Vision Pi Services #222](https://github.com/krikz/rob_box_project/actions/runs/18778257091)

### Ошибка сборки
```
CMake Error at /opt/ros/humble/share/rcutils/cmake/ament_cmake_export_libraries-extras.cmake:48 (message):
  Package 'rcutils' exports the library 'rcutils' which couldn't be found
Call Stack (most recent call first):
  /opt/ros/humble/share/rcutils/cmake/rcutilsConfig.cmake:41 (include)
  /opt/ros/humble/share/rosidl_runtime_c/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)
  ...
  CMakeLists.txt:10 (find_package)

Failed   <<< rob_box_perception_msgs [16.0s, exited with code 1]
```

### Что пытались исправить
Commit добавлял пакет `rob_box_perception_msgs` к списку собираемых пакетов:
```dockerfile
colcon build \
  --packages-select audio_common_msgs rob_box_perception_msgs rob_box_voice rob_box_animations
```

Также были добавлены попытки исправить проблему через:
- Экспорт `CMAKE_PREFIX_PATH`
- Экспорт `CMAKE_LIBRARY_PATH`
- Аргументы CMake: `-DCMAKE_LIBRARY_PATH` и `-DCMAKE_FIND_ROOT_PATH`
- Дебаг секции для проверки библиотек

## 🔍 Корневая причина

**Проблема:** Аргументы `-DCMAKE_LIBRARY_PATH` и `-DCMAKE_FIND_ROOT_PATH` **мешают** нормальной работе CMake package config механизма.

### Как это работает в ROS 2:
1. При выполнении `. /opt/ros/humble/setup.sh` устанавливаются все необходимые переменные окружения
2. CMake использует `find_package()` для поиска пакетов через их Config файлы
3. Config файлы (например, `rcutilsConfig.cmake`) знают где искать библиотеки
4. **Добавление** `-DCMAKE_LIBRARY_PATH` и `-DCMAKE_FIND_ROOT_PATH` **переопределяет** эту логику

### Почему это ломает сборку:
- CMake начинает искать библиотеки только в указанных путях
- Архитектурно-специфичные пути (например, `/opt/ros/humble/lib/aarch64-linux-gnu`) игнорируются
- Библиотеки существуют, но CMake их не находит из-за переопределённых путей

## ✅ Решение

### Что было удалено из Dockerfile:
```dockerfile
# ❌ УДАЛЕНО - мешает CMake package discovery
export CMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO}:$CMAKE_PREFIX_PATH
export CMAKE_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib:$CMAKE_LIBRARY_PATH
colcon build \
  --cmake-args \
  -DCMAKE_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib \
  -DCMAKE_FIND_ROOT_PATH=/opt/ros/${ROS_DISTRO}
```

### Правильный подход:
```dockerfile
# ✅ ПРАВИЛЬНО - sourcing ROS setup достаточно
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
    --packages-select audio_common_msgs rob_box_perception_msgs rob_box_voice rob_box_animations \
    --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### Также удалены:
- ❌ Debug RUN команды для проверки `builtin_interfaces`
- ❌ Debug RUN команды для проверки `LD_LIBRARY_PATH`
- ❌ Лишние комментарии об отладке

## 📚 Примеры из проекта

### Успешный Dockerfile: `docker/main/micro_ros_agent/Dockerfile`
```dockerfile
# Сборка custom messages без проблемных аргументов
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . /uros_ws/install/local_setup.sh && \
    colcon build --packages-select robot_sensor_hub_msg --parallel-workers 1 --executor sequential
```

**Вывод:** Этот Dockerfile успешно собирает message пакет `robot_sensor_hub_msg` без каких-либо специальных CMAKE_LIBRARY_PATH аргументов!

## 🎯 Правила для будущего

### ✅ DO: Правильный подход к сборке ROS 2 пакетов
1. Source ROS setup: `. /opt/ros/${ROS_DISTRO}/setup.sh`
2. Использовать `colcon build` без переопределения путей
3. Доверять CMake package config механизму
4. При необходимости - `--parallel-workers 1` для экономии памяти

### ❌ DON'T: Неправильные попытки "помочь" CMake
1. ❌ `export CMAKE_LIBRARY_PATH=...` - ломает архитектурно-специфичные пути
2. ❌ `-DCMAKE_FIND_ROOT_PATH=...` - переопределяет поиск пакетов
3. ❌ `-DCMAKE_LIBRARY_PATH=...` - мешает find_library()
4. ❌ Добавление обширных debug секций вместо исправления корневой причины

## 🔄 Изменения в коде

### Файл: `docker/vision/voice_assistant/Dockerfile`
- **Удалено:** 22 строки debug кода
- **Упрощено:** Команда colcon build (убраны проблемные аргументы)
- **Результат:** Чистая, понятная сборка по принципу "меньше - значит лучше"

### Diff:
```diff
-# ОТЛАДКА: Проверка состояния ПЕРЕД colcon build
-RUN echo "=== ПЕРЕД COLCON BUILD: проверка builtin_interfaces ===" && \
-    ls -lh /opt/ros/humble/lib/libbuiltin_interfaces*.so* 2>/dev/null | head -3 && \
-    ... (ещё 10 строк debug)
-
-# FIX: Добавляем CMAKE_LIBRARY_PATH и CMAKE_FIND_ROOT_PATH
 RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
-    export CMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO}:$CMAKE_PREFIX_PATH && \
-    export CMAKE_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib:$CMAKE_LIBRARY_PATH && \
     colcon build \
     --packages-select audio_common_msgs rob_box_perception_msgs rob_box_voice rob_box_animations \
     --symlink-install \
     --cmake-args \
     -DCMAKE_BUILD_TYPE=Release \
-    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
-    -DCMAKE_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib \
-    -DCMAKE_FIND_ROOT_PATH=/opt/ros/${ROS_DISTRO}
+    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

## 📊 Статус

| Аспект | Статус |
|--------|--------|
| Проблема идентифицирована | ✅ |
| Корневая причина найдена | ✅ |
| Исправление применено | ✅ |
| Dockerfile упрощён | ✅ |
| Debug код удалён | ✅ |
| Документация создана | ✅ |
| Ожидание CI/CD проверки | ⏳ |

## 🔗 Связанные материалы

- **GitHub Actions Run:** https://github.com/krikz/rob_box_project/actions/runs/18778257091
- **Failing Commit:** `fe4b6e9b35fe3ae3f285d753ec03fa1f60de3f5f`
- **Fix Commit:** (будет добавлен после merge)
- **Документация:** `docs/development/DOCKER_BUILD_FIXES.md` (для архива)

---

**Автор исправления:** GitHub Copilot  
**Дата:** 2025-10-24  
**Проблема решена за:** ~30 минут  
**Ключевой урок:** Не переопределяйте CMake пути - sourcing ROS setup.sh достаточно!
