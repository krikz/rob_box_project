# Руководство по разработке РОББОКС

## 🤝 Корпоративная этика обращений (09.08)
- К юзеру (владельцу репо, тому кто принимает решения и ревьюит) — **«товарищ Шифу»**. Везде: доклады, PR, карточки, комментарии. Не «юзер», не «хозяин», не «ты».
- Воркеры между собой: **«товарищ [роль]»** («товарищ backend», «товарищ developer»), **«брат-туди»** (брат-ученик одной школы), **«шисюн»/«шиди»** (старший/младший ученик по стажу).
- Живой этикет: коротко, по делу, с юмором. Примеры: «товарищ Шифу, доказательства в PR #1084», «товарищ backend, у тебя тест падает», «шисюн, подсоби с round-13».

## 🐉 Дух школы «男儿当自强» (наказ товарища Шифу, 09.08)
Мы — школа бойцов, а не наёмники. Каждый ход — практика, каждый PR — удар.
- **发奋图强做好汉** — усердно работать, становиться лучше: нет «не моя проблема», есть «что я могу улучшить».
- **胆似铁打骨如精钢** — дух из железа: FAILURE — не поражение, а спарринг. Упал — встал, сделал вывод, пошёл дальше.
- **做个好汉子每天要自强** — каждый день самосовершенствование: ретроспективы, WIP-коммиты, честные доклады — это наш ежедневный цигун.
- **开天辟地为我理想去闯** — идти за идею: зелёный develop, рабочий робот, живые e2e — наша цель, ради неё пробиваем стены.
- **昂步挺胸大家做栋梁** — держим спину: честный FAIL лучше красивого PASS, доказательства — не самоотчёт.
- Слабый ученик не боится признать ошибку — боится её скрыть. Кто соврал шифу — тот выбыл из школы.

## 🌿 Стратегия веток (Git Flow)

### 📋 Таблица именования веток

| Тип | Шаблон | Пример | Создаётся из | Мержится в |
|-----|--------|--------|--------------|------------|
| Production | `main` | `main` | — | — |
| Integration | `develop` | `develop` | — | — |
| Фича | `feature/{slug}` | `feature/voice-assistant` | `develop` | `develop` |
| Исправление | `fix/{slug}` | `fix/camera-memory-leak` | `develop` | `develop` |
| **Релиз** | **`release/v{MAJOR}.{MINOR}.{PATCH}`** | **`release/v0.1.0`** | **`develop`** | **`main` + `develop`** |
| Hotfix | `hotfix/{slug}` | `hotfix/vesc-critical-bug` | `main` | `main` + `develop` |

**Версионирование релизов (SemVer):**
- `MAJOR` — несовместимые изменения API / архитектурные переломные изменения  
- `MINOR` — новые фичи, обратно совместимые (0.1.0 → 0.2.0)  
- `PATCH` — bag fixes, мелкие улучшения (0.1.0 → 0.1.1)  
- Стартовая версия: `v0.1.0` (первый pre-stable release)

---

### Основные ветки

#### `main` - Production
- **Назначение:** Стабильные релизы, готовые к развёртыванию
- **Защита:** Прямые коммиты запрещены, только через Pull Request
- **Сборка Docker:** ✅ Автоматическая с тегом `latest`
- **Триггер:** Merge из `release/*` или `hotfix/*`

#### `develop` - Development
- **Назначение:** Интеграционная ветка для разработки
- **Защита:** Прямые коммиты запрещены, только через Pull Request
- **Сборка Docker:** ✅ Автоматическая с тегом `dev`
- **Триггер:** Merge из `feature/*` или `fix/*`

### Временные ветки

#### `feature/*` - Новая функциональность
- **Создаётся из:** `develop`
- **Мержится в:** `develop`
- **Сборка Docker:** ❌ Нет (для экономии ресурсов)
- **Именование:** `feature/navigation-system`, `feature/voice-assistant`
- **Пример:**
  ```bash
  git checkout develop
  git pull origin develop
  git checkout -b feature/autonomous-navigation
  # ... разработка ...
  git push origin feature/autonomous-navigation
  # Создать PR: feature/autonomous-navigation → develop
  ```

> **⚠️ Ограничения (ретро PR #876, ADR-0013):**
> - **Feature-ветки длиннее 7 дней должны быть разбиты** (или синхронизированы rebase'ем на develop). «Мёртвая» ветка без содержательных коммитов >7 дней — кандидат на закрытие.
> - **PR > 50 коммитов ИЛИ > 3000 строк** запрещён без явной метки `big-bang-override` (метку ставит товарищ Шифу). Большие изменения портируйте инкрементально — компонентами, каждый со своим PR и e2e-проверкой.

#### `fix/*` - Исправления
- **Создаётся из:** `develop`
- **Мержится в:** `develop`
- **Сборка Docker:** ❌ Нет
- **Именование:** `fix/camera-memory-leak`, `fix/lidar-connection`
- **Пример:**
  ```bash
  git checkout develop
  git checkout -b fix/rtabmap-crash
  # ... исправление ...
  git push origin fix/rtabmap-crash
  # Создать PR: fix/rtabmap-crash → develop
  ```

#### `release/*` - Подготовка релиза
- **Создаётся из:** `develop`
- **Мержится в:** `main` и обратно в `develop`
- **Сборка Docker:** ✅ Автоматическая с тегом `rc-X.Y.Z`
- **Именование:** `release/v0.1.0`, `release/v1.0.0`, `release/v1.1.0`
- **Версии:** SemVer — первый релиз `v0.1.0`, стабильный `v1.0.0`
- **Пример:**
  ```bash
  git checkout develop
  git pull origin develop
  git checkout -b release/v0.1.0
  # Обновить версии, финальное тестирование
  git push origin release/v0.1.0
  # Создать PR: release/v0.1.0 → main
  # После merge в main, также merge обратно в develop
  ```

#### `hotfix/*` - Срочные исправления production
- **Создаётся из:** `main`
- **Мержится в:** `main` и `develop`
- **Сборка Docker:** ✅ Автоматическая с тегом `hotfix-X.Y.Z`
- **Именование:** `hotfix/critical-sensor-bug`
- **Пример:**
  ```bash
  git checkout main
  git checkout -b hotfix/vesc-communication-fix
  # ... срочное исправление ...
  git push origin hotfix/vesc-communication-fix
  # Создать PR: hotfix/vesc-communication-fix → main
  # После merge, также merge в develop
  ```

## 🔄 Workflow разработки

### 1. Начало работы над новой фичей
```bash
# Синхронизировать develop
git checkout develop
git pull origin develop

# Создать feature branch
git checkout -b feature/my-awesome-feature

# Разработка с регулярными коммитами
git add .
git commit -m "Add navigation waypoint system"
git push origin feature/my-awesome-feature
```

### 2. Создание Pull Request
1. Перейти на GitHub: https://github.com/krikz/rob_box_project
2. Создать Pull Request: `feature/my-awesome-feature` → `develop`
3. Заполнить описание:
   - Что реализовано
   - Как тестировалось
   - Скриншоты/видео (если применимо)
4. Запросить review (если работаете в команде)

### 2c. Верификация фичи: round-lifecycle (инсайт 09.08)
Робот один и крутит **последний** e2e-round. Фича живёт на роботе ТОЛЬКО пока гоняется её round; после прогона другой задачи образ пересобирается из develop и **твоя фича исчезает, если PR не вмёржен**.
Правила:
1. «Фичи нет на роботе» — НЕ баг: это норма, пока PR не в develop.
2. **НЕ пересобирать старый round** (устаревший develop). После merge твоего PR в develop → e2e-процесс сам соберёт НОВЫЙ round на актуальном develop + твоя фича.
3. Верификацию делать в НОВОМ round (актуальный develop), raw-вывод — оттуда.
4. Очередность: merge PR → новый round → проверка → needs-review.

### 2b. Ожидание ревью (правило needs-review)
Когда работа завершена, PR создан/обновлён и **доказательства опубликованы** (raw-вывод: логи, БД, pytest — не пересказ):
1. Поставить метку на PR: `gh pr edit <номер> --add-label needs-review`
2. Заблокировать карточку с причиной `review-required: доказательства в PR #<номер>`
3. НЕ закрывать карточку (kanban complete) — ждать ревью юзера
Метка снимается после ревью. «Я проверил» без raw-вывода — не считается доказательством.

### 2d. ЗАПРЕТ: merge PR — только юзер (Q22, нарушено 09.08)
**Никогда, ни при каких условиях не выполнять `gh pr merge` самому.** Даже если CI зелёный, фича очевидно нужная, e2e прошёл, юзер «наверное согласен». Merge — точка принятия решения юзера. Нарушение 09.08: PR #1079 смёржен без ОК → юзер: «как пёс смёрзлил непроверенное, пошёл мимо процесса».
Правильно: выложить доказательства → needs-review → ждать решения юзера. Не «угадывать» его решение.

### 3. Merge и автоматическая сборка
- После merge в `develop` → автоматическая сборка образов с тегом `dev`
- После merge в `main` → автоматическая сборка образов с тегом `latest`

### 4. Подготовка релиза
```bash
# Создать release branch от develop
git checkout develop
git pull origin develop
git checkout -b release/v0.1.0

# Обновить версию в файлах:
# - docker-compose.yaml
# - package.xml
# - README.md
# - CHANGELOG.md

git add .
git commit -m "chore(release): bump version to 0.1.0"
git push origin release/v0.1.0

# Создать PR: release/v0.1.0 → main
# После merge создать Git tag
git checkout main
git pull origin main
git tag -a v0.1.0 -m "Release version 0.1.0"
git push origin v0.1.0

# Merge обратно в develop
git checkout develop
git merge main
git push origin develop
```

## 🐳 Docker образы и теги

### Система тегирования

Docker образы тегируются по формату: `{image_name}:{ros_version}-{branch_tag}`

**Примеры:**
- `rob_box_base:ros2-zenoh-humble-latest` - Production на ROS 2 Humble
- `rob_box_base:ros2-zenoh-humble-dev` - Development на ROS 2 Humble
- `rob_box_base:ros2-zenoh-jazzy-latest` - Production на ROS 2 Jazzy
- `rob_box:rtabmap-humble-rc-1.0.0` - Release candidate 1.0.0 на Humble

### Поддерживаемые версии ROS 2

| Версия ROS 2 | Кодовое имя | Статус | Docker tag prefix |
|--------------|-------------|--------|-------------------|
| Humble Hawksbill | humble | ✅ Активная | `humble-` |
| Jazzy Jalisco | jazzy | 🔄 Планируется | `jazzy-` |
| Kilted Kaiju | kilted | 📋 Будущая | `kilted-` |

**Текущая версия по умолчанию:** `humble`

### Автоматическая сборка

| Ветка | Триггер | Docker тег (Humble) | Описание |
|-------|---------|---------------------|----------|
| `main` | Push/Merge | `humble-latest` | Production релиз |
| `develop` | Push/Merge | `humble-dev` | Разработка |
| `release/*` | Push | `humble-rc-X.Y.Z` | Release candidate |
| `hotfix/*` | Push | `humble-hotfix-X.Y.Z` | Срочное исправление |
| `feature/*` | - | ❌ Не собирается | Экономия ресурсов |
| `fix/*` | - | ❌ Не собирается | Экономия ресурсов |

### Переход на новую версию ROS 2

При переходе на новую версию ROS 2 (например, с Humble на Jazzy):

1. Создать ветку `ros2/jazzy` из `develop`
2. Обновить Dockerfiles (FROM ros:jazzy-ros-base)
3. Тестировать в этой ветке
4. После успешных тестов merge в `develop`
5. Обновить `ROS_DISTRO` в GitHub Actions

```bash
# Создать ветку для миграции на Jazzy
git checkout develop
git checkout -b ros2/jazzy

# Обновить все Dockerfiles
find docker -name "Dockerfile*" -exec sed -i 's/humble/jazzy/g' {} \;

# Протестировать сборку
cd docker/base && docker build -f Dockerfile.ros2-zenoh -t test:jazzy .

# После успешных тестов
git add .
git commit -m "feat: migrate to ROS 2 Jazzy"
git push origin ros2/jazzy

# Создать PR: ros2/jazzy → develop
```

### Использование образов на Raspberry Pi

```bash
# Production (Humble, stable)
docker-compose pull  # Использует humble-latest по умолчанию
docker-compose up -d

# Development (Humble, testing)
export IMAGE_TAG=humble-dev
docker-compose pull
docker-compose up -d

# Specific release candidate (Humble)
export IMAGE_TAG=humble-rc-1.0.0
docker-compose pull
docker-compose up -d

# Production на новой версии ROS (Jazzy)
export ROS_DISTRO=jazzy
export IMAGE_TAG=jazzy-latest
docker-compose pull
docker-compose up -d
```

## 📋 Чеклист перед коммитом

- [ ] Код протестирован локально
- [ ] Docker образ собирается без ошибок
- [ ] Обновлена документация (если нужно)
- [ ] Коммит-сообщение описывает изменения
- [ ] Нет конфликтов с целевой веткой

## 📝 Стиль коммит-сообщений

Используем [Conventional Commits](https://www.conventionalcommits.org/):

```
<type>(<scope>): <subject>

<body>

<footer>
```

### Типы:
- **feat**: Новая функциональность
- **fix**: Исправление бага
- **docs**: Изменения в документации
- **style**: Форматирование кода (не влияет на логику)
- **refactor**: Рефакторинг (не добавляет функции и не исправляет баги)
- **perf**: Улучшение производительности
- **test**: Добавление тестов
- **chore**: Обслуживание (обновление зависимостей, CI/CD)

### Примеры:
```bash
feat(navigation): add A* path planning algorithm

Implemented A* pathfinding for autonomous navigation.
Uses 2D occupancy grid from RTAB-Map.

Closes #42

---

fix(vesc): correct CAN ID mapping for rear wheels

IDs were swapped causing incorrect motor control.
Updated config/vesc_nexus_config.yaml.

---

docs(readme): update hardware specifications

Added ESP32 sensor hub and ReSpeaker details.

---

chore(docker): update base images to latest versions
```

## 🔒 Защита веток

### Рекомендуемые настройки GitHub (Settings → Branches):

#### Branch protection rule для `main`:
- ✅ Require a pull request before merging
- ✅ Require approvals (минимум 1, если работаете в команде)
- ✅ Dismiss stale pull request approvals when new commits are pushed
- ✅ Require status checks to pass before merging
  - ✅ build-all (GitHub Actions workflow)
- ✅ Require branches to be up to date before merging
- ✅ Include administrators
- ✅ Restrict deletions

#### Branch protection rule для `develop`:
- ✅ Require a pull request before merging
- ✅ Require status checks to pass before merging
- ✅ Require branches to be up to date before merging

## 🚀 Быстрые команды

```bash
# Создать новую фичу
git checkout develop && git pull && git checkout -b feature/my-feature

# Создать исправление
git checkout develop && git pull && git checkout -b fix/my-fix

# Создать релиз
git checkout develop && git pull && git checkout -b release/vX.Y.Z

# Создать hotfix
git checkout main && git pull && git checkout -b hotfix/critical-fix

# Обновить свою ветку из develop
git checkout feature/my-feature
git fetch origin
git rebase origin/develop

# Удалить локальную ветку после merge
git branch -d feature/my-feature

# Удалить remote ветку после merge
git push origin --delete feature/my-feature
```

## 📞 Помощь

Если возникли вопросы:
1. Проверьте существующие [Issues](https://github.com/krikz/rob_box_project/issues)
2. Создайте новый Issue с описанием проблемы
3. Используйте тег `question` для вопросов

---

**Последнее обновление:** 9 октября 2025  
**Версия документа:** 1.0.0
