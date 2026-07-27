# 05-ACCEPTANCE.md — Приёмочный цикл миграции Lyrical

> **Дата:** 2026-07-27  
> **Статус:** Ready  
> **Референс:** [Kilted build](https://github.com/krikz/rob_box_project/actions/runs/30249200337)

---

## Цель

Итеративно добиться зелёной сборки ВСЕХ Docker-сервисов на ROS 2 Lyrical через GitHub Actions CI.

## Процесс

### Фаза 1: Базовые образы (с `build_base_images=true`)

```
ПОЛЬЗОВАТЕЛЬ запускает:
  gh workflow run "L: Build All Services" --ref feature/lyrical -f build_base_images=true

АГЕНТ:
  1. Ждёт завершения (~20-60 мин)
  2. Смотрит логи упавших сервисов: gh run view <RUN_ID> --log --job=<JOB_ID>
  3. Анализирует ошибки (пакеты, Python, пути, системные)
  4. Правит код → commit → push
  5. СООБЩАЕТ ПОЛЬЗОВАТЕЛЮ: "Готово, запускай заново: gh workflow run ... --ref feature/lyrical -f build_base_images=true"
  6. GOTO шаг 1
```

**Критерий готовности:** Все base-образы собираются зелёными:
- `ros2-zenoh-lyrical` ✅
- `rtabmap-lyrical` ✅  
- `pcl-lyrical` ✅

### Фаза 2: Все сервисы (без `build_base_images`)

```
ПОЛЬЗОВАТЕЛЬ запускает:
  gh workflow run "L: Build All Services" --ref feature/lyrical

АГЕНТ:
  Тот же цикл: логи → анализ → правки → commit → push → "запускай заново"
```

**Критерий готовности:** Все сервисы собираются зелёными.

### Фаза 3: Wave 2 (Nav2 + DepthAI)

После успешной сборки Wave 1 — выполнить 05-02-PLAN.md (Nav2 source-build + DepthAI source-build).

---

## Ожидаемые классы ошибок

| Класс | Пример | Как чинить |
|-------|--------|------------|
| **Отсутствует пакет** | `E: Unable to locate package ros-lyrical-xxx` | Проверить имя в `apt-cache search`, заменить на правильное |
| **pip без --break-system-packages** | `error: externally-managed-environment` | Добавить `--break-system-packages` |
| **Нет python3-pip** | `pip3: command not found` | Добавить `python3-pip` в apt-зависимости |
| **Старый путь** | `/opt/ros/humble/...` | Заменить на `/opt/ros/lyrical/...` |
| **Не тот образ** | `FROM ros:humble-...` | Заменить на `FROM ros:lyrical-...` |
| **LD_LIBRARY_PATH** | Неверный путь к zenoh | Проверить в `ros:lyrical-ros-base` |
| **Системный пакет** | `libxxx-dev` не найден | Проверить доступность в Resolute репозиториях |
| **Python импорт** | `ModuleNotFoundError` | Проверить совместимость с Python 3.14 |

---

## Команды

```bash
# Фаза 1: Запуск с пересборкой base-образов
gh workflow run "L: Build All Services" --ref feature/lyrical -f build_base_images=true

# Фаза 2: Запуск без пересборки base-образов (быстрее)
gh workflow run "L: Build All Services" --ref feature/lyrical

# Смотреть статус последних запусков
gh run list --workflow="L: Build All Services" --limit 5

# Смотреть логи конкретного джоба
gh run view <RUN_ID> --log --job=<JOB_ID>

# Проверить доступность пакета в Lyrical
docker run --rm ros:lyrical-ros-base apt-cache search ros-lyrical-<name>
```

---

## Правила

1. **Пользователь запускает workflow.** Агент сообщает готовую команду, но НЕ выполняет `gh workflow run`.
2. **Агент анализирует логи** через `gh run view`.
3. **Каждый fix — отдельный commit** с описанием что и почему.
4. **Не трогать Nav2 и DepthAI** до Фазы 3 (Wave 2).
5. **Если ошибка неясна** — спросить пользователя, а не гадать.

---

*Создано в рамках plan-phase. Следующий шаг: `gh workflow run "L: Build All Services" --ref feature/lyrical -f build_base_images=true`*
