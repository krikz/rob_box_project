# Руководство по разработке РОББОКС

## 🌿 Стратегия веток (Git Flow)

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
- **Именование:** `release/v1.0.0`, `release/v1.1.0`
- **Пример:**
  ```bash
  git checkout develop
  git checkout -b release/v1.0.0
  # Обновить версии, финальное тестирование
  git push origin release/v1.0.0
  # Создать PR: release/v1.0.0 → main
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

### 3. Merge и автоматическая сборка
- После merge в `develop` → автоматическая сборка образов с тегом `dev`
- После merge в `main` → автоматическая сборка образов с тегом `latest`

### 4. Подготовка релиза
```bash
# Создать release branch
git checkout develop
git checkout -b release/v1.0.0

# Обновить версию в файлах
# - docker-compose.yaml
# - package.xml
# - README.md

git add .
git commit -m "Bump version to 1.0.0"
git push origin release/v1.0.0

# Создать PR: release/v1.0.0 → main
# После merge создать Git tag
git checkout main
git pull origin main
git tag -a v1.0.0 -m "Release version 1.0.0"
git push origin v1.0.0

# Merge обратно в develop
git checkout develop
git merge main
git push origin develop
```

## 🐳 Docker образы и теги

### Автоматическая сборка

| Ветка | Триггер | Docker тег | Описание |
|-------|---------|------------|----------|
| `main` | Push/Merge | `latest` | Production релиз |
| `develop` | Push/Merge | `dev` | Разработка |
| `release/*` | Push | `rc-X.Y.Z` | Release candidate |
| `hotfix/*` | Push | `hotfix-X.Y.Z` | Срочное исправление |
| `feature/*` | - | ❌ Не собирается | Экономия ресурсов |
| `fix/*` | - | ❌ Не собирается | Экономия ресурсов |

### Использование образов на Raspberry Pi

```bash
# Production (stable)
docker-compose pull  # Использует latest по умолчанию
docker-compose up -d

# Development (testing)
export IMAGE_TAG=dev
docker-compose pull
docker-compose up -d

# Specific release candidate
export IMAGE_TAG=rc-1.0.0
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
