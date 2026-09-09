---
name: github-actions-runner
description: >
  Skill for triggering GitHub Actions workflows in the rob_box_project repository.
  Use when the user asks to run, launch, trigger, or start any CI/CD workflow:
  build images, deploy to robot, run tests, lint code, validate Docker Compose,
  or build a single service.
---

# GitHub Actions Runner

Репозиторий: **github.com/krikz/rob_box_project**

## Инструменты запуска

Есть два способа:

### 1. GitHub Web UI (gh CLI отсутствует)
1. Открыть https://github.com/krikz/rob_box_project/actions
2. Выбрать нужный workflow из левой колонки
3. Нажать **"Run workflow"** → выбрать ветку и параметры → нажать кнопку

### 2. GitHub CLI (`gh`) — если установлен
```bash
gh workflow run "<workflow-file>" -f param=value --repo krikz/rob_box_project
```

Установка: https://cli.github.com/

---

## Карта workflow → запрос пользователя

| Что хочет пользователь | Workflow |
|------------------------|----------|
| Собрать всё (GitHub cloud) | `G-Build All Services.yml` |
| Собрать всё (локальный runner) | `L-Build All Services.yml` |
| Собрать базовые образы (GitHub) | `G-Build Base Images.yml` |
| Собрать базовые образы (локально) | `L-Build Base Images.yml` |
| Собрать Main Pi сервисы (GitHub) | `G-Build Main Pi Services.yml` |
| Собрать Main Pi сервисы (локально) | `L-Build Main Pi Services.yml` |
| Собрать Vision Pi сервисы (GitHub) | `G-Build Vision Pi Services.yml` |
| Собрать Vision Pi сервисы (локально) | `L-Build Vision Pi Services.yml` |
| Собрать один конкретный сервис | `L-Build Single Service.yml` |
| Задеплоить на робота | `L-Deploy and Verify.yml` |
| Прогнать линтер | `G-Lint Code.yml` |
| Запустить тесты | `G-Run Tests.yml` |
| Проверить Docker Compose | `G-Validate Docker Compose.yml` |

> **G-** = GitHub Actions runners (ubuntu-latest, медленнее, без локальной сети)  
> **L-** = Self-hosted local runner (быстро, доступ к Raspberry Pi и registry)

---

## Параметры workflow

Полная справка по каждому workflow → [references/workflows.md](references/workflows.md)

### Быстрые gh-команды

**Деплой на робота (production):**
```bash
gh workflow run "L-Deploy and Verify.yml" \
  -f environment=production \
  -f registry_source=github \
  --repo krikz/rob_box_project
```

**Собрать один сервис (например, voice-assistant из develop):**
```bash
gh workflow run "L-Build Single Service.yml" \
  -f branch=develop \
  -f pi_type=vision \
  -f service=voice-assistant \
  -f push_to_registry=true \
  --repo krikz/rob_box_project
```

**Полная сборка локально:**
```bash
gh workflow run "L-Build All Services.yml" \
  -f build_base_images=false \
  -f push_to_registry=true \
  --repo krikz/rob_box_project
```

**Сборка всего на GitHub Actions:**
```bash
gh workflow run "G-Build All Services.yml" \
  --repo krikz/rob_box_project
```

---

## Алгоритм ответа на запрос

1. Определи нужный workflow по таблице выше
2. Уточни недостающие параметры (ветка, окружение, сервис)
3. Если gh CLI доступен — дай готовую команду
4. Если нет — дай прямую ссылку на workflow и опиши параметры

Прямая ссылка на workflow:  
`https://github.com/krikz/rob_box_project/actions/workflows/<filename>`

Пример для деплоя:  
`https://github.com/krikz/rob_box_project/actions/workflows/L-Deploy%20and%20Verify.yml`
