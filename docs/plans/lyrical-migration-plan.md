# План миграции: ROS 2 Kilted Kaiju → ROS 2 Lyrical Luth

> **Статус:** 📋 План  
> **Ветка:** `feature/lyrical` ← `develop`  
> **Дата:** 2026-07-27  
> **Автор:** AI-assisted planning

---

## 1. Обзор Lyrical Luth

| Параметр | Kilted Kaiju | Lyrical Luth |
|----------|-------------|--------------|
| **Дата релиза** | Май 2025 | Май 2026 |
| **Ubuntu** | 24.04 Noble Numbat | **26.04 Resolute** ⚠️ |
| **LTS** | Нет (non-LTS) | **Да, до мая 2031** |
| **Python (системный)** | 3.12 | **3.14** (предположительно) |
| **Zenoh** | Внешний middleware (`rmw_zenoh_cpp`) | **Встроенная поддержка**, развивается |

### Ключевые новые фичи Lyrical
- **EventsCBGExecutor** — 10-15% меньше CPU чем MultithreadedExecutor
- **AsyncNode** — asyncio в rclpy (значительно меньше CPU)
- **rosidl::Buffer** — zero-copy publish/subscribe (поддержка Zenoh в разработке)
- **YAML type annotations** — `!!str`, `!!bool`, `!!int`, `!!float` в параметрах
- **URDF 1.2** — quaternions, capsule geometry, accel/decel/jerk limits
- **robot_state_publisher** — может читать `robot_description` из топика
- **Bag recording** — circular buffers, Python API, message-loss observability
- **Resource retriever service** — загрузка мешей по сети
- **Per-message log severity** в launch файлах
- **fish shell** поддержка

---

## 2. Анализ воздействия на проект

### 2.1 Критические изменения

| Область | Kilted | Lyrical | Сложность |
|--------|--------|---------|-----------|
| **ОС** | Ubuntu 24.04 Noble | Ubuntu 26.04 Resolute | 🔴 Высокая |
| **Python** | 3.12 | 3.14 (предп.) | 🔴 Высокая |
| **Системные пакеты** | `noble` | `resolute` (новые репы) | 🟡 Средняя |
| **ROS пакеты** | `ros-kilted-*` | `ros-lyrical-*` | 🟢 Низкая |
| **Docker base** | `ros:kilted-ros-base` | `ros:lyrical-ros-base` | 🟢 Низкая |
| **Zenoh** | Отдельный `rmw_zenoh_cpp` | Встроен? Нужно уточнить | 🟡 Средняя |

### 2.2 Доступность third-party образов

| Образ | Kilted | Lyrical |
|-------|--------|---------|
| `introlab3it/rtabmap_ros` | ✅ `kilted` (amd64+arm64) | ✅ `lyrical-latest` (amd64+arm64) |
| `luxonis/depthai-ros` | ✅ `kilted-arm64-latest` | ❌ **НЕТ** (ни amd64, ни arm64) |
| `ros:lyrical-ros-base` | — | ✅ (amd64+arm64) |

**🚨 БЛОКЕР: `luxonis/depthai-ros` не имеет lyrical тегов!**

Варианты решения:
1. **Ждать** — Luxonis обычно добавляет новые дистрибутивы через 1-3 месяца после релиза ROS 2
2. **Билдить самим** — собрать depthai-ros из исходников на базе `ros:lyrical-ros-base`
3. **Пиар в Luxonis** — запросить lyrical сборку в их репозитории

### 2.3 Пакеты, которые могут измениться/исчезнуть

На основе опыта миграции humble→kilted:

| Пакет (kilted) | Статус в Lyrical | Примечание |
|----------------|-----------------|------------|
| `ros-kilted-behaviortree-cpp-v3` | Вероятно `ros-lyrical-behaviortree-cpp` (v4+) | Переименован |
| `ros-kilted-control-msgs` | Может отсутствовать | Переименован/включён в другие |
| `ros-kilted-theora-image-transport` | Может быть удалён | Theora устарел |
| `ros-kilted-rmw-zenoh-cpp` | Может быть `ros-lyrical-rmw-zenoh-cpp` или встроен | Zenoh становится нативным |

---

## 3. План миграции (по шагам)

### Этап 1: Подготовка (безопасно, не ломает сборку)

- [ ] **1.1** Ветка `feature/lyrical` создана (✅ готово)
- [ ] **1.2** Проверить `ros:lyrical-ros-base` — базовые пакеты, Python, Ubuntu (✅ образ есть)
- [ ] **1.3** Проверить `pip` версию и поведение `--break-system-packages` в Ubuntu 26.04
- [ ] **1.4** Проверить APT репозитории для `resolute` (ports.ubuntu.com, packages.ros.org)
- [ ] **1.5** Уточнить статус Zenoh — встроен ли `rmw_zenoh_cpp` в базовый образ?

### Этап 2: Базовая миграция (аналог humble→kilted)

- [ ] **2.1** Массовая замена `kilted` → `lyrical` во всех файлах:
  - `ROS_DISTRO=lyrical` во всех `.env`
  - `ros:kilted-ros-base` → `ros:lyrical-ros-base` в Dockerfile'ах
  - `ros-kilted-*` → `ros-lyrical-*` в apt-get
  - `/opt/ros/kilted/` → `/opt/ros/lyrical/` в скриптах
  - `*-kilted-*` → `*-lyrical-*` в тегах образов
- [ ] **2.2** Обновить Dockerfile'ы:
  - `Dockerfile.ros2-zenoh`: `FROM ros:lyrical-ros-base`
  - `Dockerfile.rtabmap`: `FROM introlab3it/rtabmap_ros:lyrical-latest`
  - `Dockerfile.depthai`: ❌ ЗАБЛОКИРОВАНО (нет образа)
- [ ] **2.3** Обновить CI/CD:
  - `ROS_DISTRO: lyrical` во всех workflow
  - `python-version` (зависит от Ubuntu 26.04)

### Этап 3: Системные изменения Ubuntu 26.04

- [ ] **3.1** Python 3.14 — `externally-managed` всё ещё актуально
- [ ] **3.2** APT sources: `noble` → `resolute`, `ubuntu.com` → `old-releases`?
- [ ] **3.3** Системные пакеты могут изменить названия
- [ ] **3.4** `apt-cacher-ng` — будет ли работать с Resolute?

### Этап 4: Специфичные для проекта изменения

- [ ] **4.1** Zenoh: проверить, нужен ли отдельный `rmw_zenoh_cpp` или он встроен
- [ ] **4.2** Убрать `LD_LIBRARY_PATH=/opt/ros/lyrical/opt/zenoh_cpp_vendor/...` если Zenoh встроен
- [ ] **4.3** `docker/vision/zenoh-router/Dockerfile` — возможно больше не нужен
- [ ] **4.4** `docker/main/zenoh-router/Dockerfile` — возможно больше не нужен
- [ ] **4.5** Использовать новые фичи: `EventsCBGExecutor`, `AsyncNode`

### Этап 5: Решение блокера depthai

- [ ] **5.1** Проверить GitHub Issues в `luxonis/depthai-ros` — есть ли запрос на lyrical
- [ ] **5.2** Если нет — создать Issue/PR с запросом lyrical сборки
- [ ] **5.3** Временное решение: собрать `depthai-ros` из исходников в нашем Dockerfile
- [ ] **5.4** Или использовать `depthai-ros:kilted-arm64-latest` как базу (может не работать из-за разных Ubuntu)

### Этап 6: Тестирование и CI

- [ ] **6.1** Запустить `L: Build All Services` с `build_base_images=true`
- [ ] **6.2** Итеративно исправлять ошибки сборки (пакетов, Python, системных)
- [ ] **6.3** Все сервисы должны собраться ✅
- [ ] **6.4** Создать PR `feature/lyrical` → `develop`

---

## 4. Оценка рисков

| Риск | Вероятность | Влияние | Смягчение |
|------|-----------|--------|-----------|
| depthai-ros не выходит для lyrical | Высокая | Блокер | Билдить из исходников |
| Python 3.14 ломает pip пакеты | Средняя | Высокое | `--break-system-packages` + `--ignore-installed` |
| APT репы Resolute недоступны | Низкая | Блокер | Использовать `old-releases.ubuntu.com` |
| Zenoh integration меняет архитектуру | Средняя | Среднее | Проверить документацию, адаптировать |
| Системные пакеты переименованы | Средняя | Среднее | Итеративное исправление по логам CI |
| rtabmap_ros:lyrical теряет arm64 | Низкая | Среднее | Уже проверено — arm64 есть ✅ |

---

## 5. Ожидаемые выгоды от миграции

1. **LTS до 2031** — стабильность на 5 лет (vs Kilted — не LTS)
2. **Zenoh нативный** — упрощение архитектуры, меньше внешних зависимостей
3. **EventsCBGExecutor** — 10-15% экономия CPU на роботе
4. **AsyncNode** — возможность использовать asyncio для голосового ассистента
5. **URDF 1.2** — capsule geometry, quaternions для более точной модели робота
6. **Bag improvements** — circular recording для автономной работы

---

## 6. Ориентировочные трудозатраты

| Этап | Часы | Примечание |
|------|------|------------|
| Подготовка + исследование | 2-4 | Проверка Python, APT, Zenoh |
| Базовая замена kilted→lyrical | 1-2 | Массовый search-replace, как для kilted |
| Системные правки Ubuntu 26.04 | 4-8 | Самая непредсказуемая часть |
| depthai решение | 4-16 | Если билдить самим — дольше |
| Итеративные исправления CI | 8-16 | Как показал опыт kilted, много итераций |
| **Итого** | **20-45 часов** | Сильно зависит от depthai и системных сюрпризов |

---

## 7. Следующие шаги

1. 🔴 Решить вопрос с `depthai-ros` для lyrical (блокер)
2. 🟡 Начать поиск-replace `kilted`→`lyrical` (можно параллельно)
3. 🟡 Проверить Python 3.14 поведение с pip
4. 🟢 Итеративная сборка и исправление в CI

---

*План создан на основе успешного опыта миграции humble→kilted (16 коммитов, 200+ файлов).*
