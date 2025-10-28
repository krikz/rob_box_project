# Резюме исправления сборки - 2025-10-28

## Проблема
Сборка в GitHub Actions падала с ошибкой CMake:
```
CMake Error: Package 'rcutils' exports the library 'rcutils' which couldn't be found
```

**Ссылка:** https://github.com/krikz/rob_box_project/actions/runs/18887328461/job/53905959011

## Поиск похожих отчётов

✅ **Найдены идентичные проблемы в репозитории:**

1. **docs/development/DOCKER_BUILD_FIXES.md**
   - Раздел "4. ❌ `docker/main/vesc_nexus/Dockerfile` - Отсутствуют библиотеки разработки"
   - Та же ошибка: "Package 'builtin_interfaces' exports the library ... which couldn't be found"
   - Решение: добавить `ros-dev-tools` и `--reinstall` для runtime библиотек

2. **docs/reports/DOCKER_BUILD_FIX_2025-10-18.md**
   - Voice assistant контейнер имел ту же проблему
   - Детальное описание причин и решений
   - Документированы анти-паттерны

## Это уже третий раз! 📊

| Дата | Сервис | Пакет | Решение |
|------|--------|-------|---------|
| Oct 2025 | vesc_nexus | vesc_msgs | ros-dev-tools + --reinstall |
| 18 Oct 2025 | voice_assistant | rob_box_animations | --reinstall + fix CMakeLists |
| **28 Oct 2025** | **perception** | **rob_box_perception_msgs** | **ros-dev-tools + --reinstall** ✅ |

**Паттерн:** Все случаи связаны с генерацией ROS 2 messages в контейнерах, основанных на минимальном базовом образе `ros:humble-ros-base`.

## Применённое решение

### Изменён файл: `docker/main/perception/Dockerfile`

**Было:**
```dockerfile
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-python \
    && rm -rf /var/lib/apt/lists/*
```

**Стало:**
```dockerfile
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
1. ✅ `ros-dev-tools` - метапакет с cmake, build-essential и др.
2. ✅ `--reinstall` для 7 runtime библиотек в том же RUN блоке

## Почему это работает?

**Проблема:** Базовый образ `ros:humble-ros-base` минимален и не включает:
- Development tools (cmake, gcc, g++)
- Runtime библиотеки для генерации messages
- Правильно настроенные CMake config файлы

**Решение:**
1. `ros-dev-tools` устанавливает все необходимые инструменты сборки
2. `--reinstall` восстанавливает CMake конфиги для runtime библиотек
3. Всё в одном RUN блоке, чтобы apt cache был доступен

## Документация

Создан подробный отчёт:
- **docs/reports/PERCEPTION_BUILD_FIX_2025-10-28.md** (233 строки)
  - Полное описание проблемы
  - Анализ причин
  - Сравнение с предыдущими исправлениями
  - Best practices и анти-паттерны
  - Команды для диагностики

## Проверки качества

- ✅ Code Review - без замечаний
- ✅ CodeQL Security Scan - уязвимостей не найдено
- ✅ Следует существующим паттернам проекта

## Результат

- ✅ Dockerfile исправлен
- ✅ Документация создана
- ✅ Изменения закоммичены в `copilot/fix-build-issues`
- ⏳ Ждём сборку в GitHub Actions на ARM64

## Уроки

### ✅ Что сработало:
1. Проверка docs/reports/ **перед началом работы**
2. Использование задокументированных паттернов
3. Следование архитектурным принципам (минимальный базовый образ)

### 💡 Рекомендации на будущее:
При ошибке "Package 'X' exports library 'Y' which couldn't be found":
1. Проверить docs/reports/ на похожие случаи
2. Добавить `ros-dev-tools` если отсутствует
3. Добавить `--reinstall` для проблемных библиотек
4. Всё в одном RUN блоке с apt-get update

---

**Коммиты:**
- `7c0164c` - Initial plan
- `7380eb5` - fix(docker): add missing ROS development packages
- `49fa65f` - docs: add perception build fix report

**Branch:** copilot/fix-build-issues  
**Status:** Ready for GitHub Actions build ✅
