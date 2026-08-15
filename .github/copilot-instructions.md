# GitHub Copilot - Rob Box Project Navigator

## 🎯 Проект
**Rob Box** — автономный ровер на ROS 2 Humble + Zenoh DDS  
Dual Raspberry Pi 5: Main (10.1.1.10) + Vision (10.1.1.11)

## 📚 Где искать информацию

### 🔴 КРИТИЧНО - Проверить ПЕРЕД изменениями

Не загружай все документы ниже целиком по умолчанию. Сначала определи, какой именно документ нужен для текущего шага, затем используй подход `PEEK → GREP → READ` и открывай только релевантные секции.

| Тема | Файл | Что внутри |
|------|------|-----------|
| **Процесс разработки (ГЛАВНОЕ)** | `.agents/skills/context-engineering/SKILL.md` | Research→Design→Plan→Implement, команды, правила |
| **Бэклог задач** | GitHub Issues | `gh issue list/create/view/develop`; labels `source:gsd`; milestones M1/M2/M3 |
| **Docker Rules** | `docs/development/DOCKER_STANDARDS.md` | ❌ COPY config/scripts, ✅ volumes, network_mode: host |
| **Python Style** | `docs/development/PYTHON_STYLE_GUIDE.md` | black, isort, flake8, ROS 2 patterns, naming |

### 🏗️ Архитектура

| Компонент | Документация |
|-----------|--------------|
| Системная архитектура | `docs/architecture/SYSTEM_OVERVIEW.md` |
| Железо (Pi, сенсоры, моторы) | `docs/architecture/HARDWARE.md` |
| Софт (ROS 2, Docker, Zenoh) | `docs/architecture/SOFTWARE.md` |
| Сетевая топология | `docs/architecture/NETWORK_TOPOLOGY.md` |

### 🔧 Разработка

| Задача | Документация |
|--------|--------------|
| **Процесс (методология)** | `.agents/skills/context-engineering/SKILL.md` |
| **Zenoh dev-машина** | `.agents/skills/zenoh-dev-setup/SKILL.md` |
| Docker сборка | `docs/development/BUILD_OPTIMIZATION.md` |
| CI/CD Pipeline | `docs/CI_CD_PIPELINE.md` |
| Тестирование | `docs/development/TESTING_GUIDE.md` |
| Линтинг | `docs/development/LINTING_GUIDE.md` |
| Деплой | `docs/deployment/DEPLOYMENT_WORKFLOW.md` |

### 🦸 Superpowers Skills (`.agents/skills/`)

| Скилл | Когда использовать |
|-------|--------------------|
| `brainstorming` | Перед любой разработкой фичи — рефайн идей через диалог |
| `writing-plans` | После дизайна — детальный план реализации по шагам |
| `executing-plans` | Выполнение плана в отдельной сессии с чекпоинтами |
| `subagent-driven-development` | Выполнение плана в текущей сессии через сабагентов |
| `test-driven-development` | При реализации любой фичи или исправлении бага |
| `systematic-debugging` | При любом баге или неожиданном поведении |
| `verification-before-completion` | Перед заявлением о завершении работы |
| `requesting-code-review` | После выполнения задачи или перед мержем |
| `receiving-code-review` | При получении фидбека на код-ревью |
| `dispatching-parallel-agents` | При 2+ независимых задачах без shared state |
| `using-git-worktrees` | При старте фичи требующей изоляции от текущего workspace |
| `finishing-a-development-branch` | Когда реализация завершена — мерж/PR/дискард |
| `writing-skills` | При создании или обновлении скиллов |
| `using-superpowers` | В начале любого разговора — как найти и использовать скиллы |
| `debugger` | При отладке ошибок, краш-анализе, разборе stack trace (shubhamsaboo/awesome-llm-apps) |

### ⚙️ Команды Claude (`.claude/commands/`)

| Фаза | Команда | Когда использовать |
|------|---------|--------------------|
| **Research** | `/research-codebase TASK-ID` | Начало любой задачи |
| **Design (фича)** | `/design-feature <name> <research.md>` | После research |
| **Design (баг)** | `/design-bugfix <id> <research.md>` | После research |
| **Plan (фича)** | `/plan-feature <design-dir>` | После ревью дизайна |
| **Plan (баг)** | `/plan-bugfix <design-dir>` | После ревью дизайна |
| **Implement (фича)** | `/implement-feature <plan-dir>` | После ревью плана |
| **Implement (баг)** | `/implement-bugfix <plan-dir>` | После ревью плана |

### 🐛 Отладка

| Проблема | Решение |
|----------|---------|
| Общие проблемы | `docs/guides/TROUBLESHOOTING.md` |
| Камера не публикует данные | `docs/guides/TROUBLESHOOTING.md` |
| Zenoh connection issues | `docs/fixes/ZENOH_FIX_SUMMARY_2025-11-10.md` |
| Мониторинг системы | `docs/guides/MONITORING_QUICK_REF.md` |

### 📦 Пакеты ROS 2

| Пакет | Назначение | Документация |
|-------|-----------|--------------|
| `rob_box_voice` | Voice assistant (STT, TTS, dialogue) | `src/rob_box_voice/README.md` |
| `rob_box_perception` | Health monitor, context aggregator | `src/rob_box_perception/README.md` |
| `rob_box_animations` | LED matrix animations (381 LEDs) | `src/rob_box_animations/README.md` |
| `rob_box_description` | URDF robot model | `src/rob_box_description/` |

### 🌐 Сеть и middleware

**IP-адреса:**
- Main Pi: `10.1.1.10` (eth0), `10.1.1.20` (wlan0)
- Vision Pi: `10.1.1.11` (eth0), `10.1.1.21` (wlan0)

**Zenoh конфигурация:**
```yaml
RMW_IMPLEMENTATION=rmw_zenoh_cpp
ZENOH_CONFIG=/config/zenoh_session_config.json5
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

### 🐳 Docker структура

```
docker/
├── base/          # Базовые образы: ros2-zenoh, rtabmap, depthai, pcl
├── main/          # Main Pi сервисы (rtabmap, nav2, control, lslidar, perception)
│   ├── config/    # ✅ Конфиги (монтируются volumes, НЕ COPY!)
│   ├── scripts/   # ✅ Скрипты (монтируются volumes, НЕ COPY!)
│   └── <service>/ # Только Dockerfile
└── vision/        # Vision Pi сервисы (oak-d, voice, led-matrix, apriltag)
    ├── config/    # ✅ Конфиги (volumes)
    ├── scripts/   # ✅ Скрипты (volumes)
    └── <service>/ # Только Dockerfile
```

## ⚡ Быстрые команды

### SSH доступ
```bash
# SEC-1: пароль не хардкодим — бери из GitHub secret SSH_PASSWORD (или env SSHPASS).
# Vision Pi
export SSHPASS='<пароль из GitHub secret SSH_PASSWORD>'
sshpass -e ssh ros2@10.1.1.21

# Main Pi  
sshpass -e ssh ros2@10.1.1.20
```

### Обновление на Pi
```bash
# Vision Pi
export SSHPASS='<пароль из GitHub secret SSH_PASSWORD>'
sshpass -e ssh ros2@10.1.1.21 \
  'cd ~/rob_box_project/docker/vision && ./scripts/update_and_restart.sh'
```

### Диагностика
```bash
docker ps                  # Статус контейнеров
docker logs <name> -f      # Логи в реальном времени
ros2 topic list            # ROS 2 топики
ros2 topic hz /scan        # Частота публикации
```

### 🗂️ Трекер задач (GitHub Issues)
```bash
# Просмотр бэклога
gh issue list --label "source:gsd" --repo krikz/rob_box_project

# Начать работу (создаёт ветку и переключается)
gh issue develop {N} --checkout --repo krikz/rob_box_project

# Прочитать задачу
gh issue view {N} --repo krikz/rob_box_project

# Создать задачу
gh issue create --title "[ID] description" --label "type:functional,priority:high,ai-generated,source:gsd" --repo krikz/rob_box_project

# Закрыть задачу после выполнения
gh issue close {N} --repo krikz/rob_box_project
```

## 🚨 Критичные правила

### 🤖 Деплой на роботов
- ❌ **НИКОГДА** самостоятельно не копировать файлы на робота (scp, rsync, base64 pipe, echo >)
- ❌ **НИКОГДА** не редактировать файлы напрямую на роботе (nano, vi, sed -i в repo)
- ❌ **НИКОГДА** не делать `git pull` на роботе без запроса пользователя
- ❌ **НИКОГДА** не делать `git stash` / `git checkout --` на роботе без явного `git diff` сначала
- ✅ **ВСЕГДА** деплой через GitHub Actions workflow (`docs/deployment/DEPLOYMENT_WORKFLOW.md`)
- ✅ **ВСЕГДА** репозитории на роботах должны быть чистыми (`git status` = clean)
- ✅ **ТОЛЬКО** по явной просьбе пользователя выполнять команды на роботе
- ⚠️ **ЕДИНСТВЕННЫЙ правильный путь изменений:** dev-машина → commit → push → workflow деплоит на роботов

**Если робот dirty (`git status` показывает изменения):**
1. Сначала `git diff` — понять что именно и откуда
2. Если изменение уже есть в remote (commit pushed) → `git checkout -- <file>` безопасно
3. Если изменения важные и не в remote → сохранить на dev-машину, commit, push, тогда reset

### Docker
- ❌ **НИКОГДА** `COPY config/` в Dockerfile
- ❌ **НИКОГДА** `COPY scripts/` в Dockerfile  
- ✅ **ВСЕГДА** `network_mode: host`
- ✅ **ВСЕГДА** `depends_on: zenoh-router`
- ✅ **ВСЕГДА** volumes: `./config:/config:ro`

### Python
- ✅ `black` (line-length 120)
- ✅ `isort` (profile black)
- ✅ `self.get_logger().info()` НЕ `print()`
- ✅ Type hints для public API
- ✅ Google-style docstrings

### Git commits
```
feat(voice): add command node
fix(docker): add missing dependency
docs(readme): update hardware specs
```

## 🧠 Скилы агента (`.agents/skills/`)

Перед выполнением задачи прочитай нужный скил через `read_file`, а supporting files этого скилла открывай только по мере необходимости.

| Скил | Когда использовать |
|------|--------------------|
| `karpathy-guidelines` | **ВСЕГДА** — думай перед кодом, простота, хирургические изменения, цели с верификацией |
| `using-superpowers` | **НАЧАЛО любого разговора** — как находить и применять скилы |
| `context-engineering` | Методология Research→Design→Plan→Implement |
| `brainstorming` | **ПЕРЕД любой творческой работой** — фичи, компоненты, новая функциональность |
| `writing-plans` | Есть спека/требования — пишем план перед кодом |
| `executing-plans` | Есть готовый план — выполняем по шагам |
| `subagent-driven-development` | Независимые задачи из плана — запускаем параллельно |
| `dispatching-parallel-agents` | 2+ независимые задачи без общего состояния |
| `test-driven-development` | При реализации любой фичи или багфикса |
| `systematic-debugging` | При любом баге, ошибке теста или неожиданном поведении |
| `debugger` | Отладка проблем |
| `verification-before-completion` | Перед заявлением о готовности/фиксе/PR |
| `requesting-code-review` | После завершения задачи, перед мержем |
| `receiving-code-review` | При получении feedback на код |
| `finishing-a-development-branch` | Реализация завершена — merge/PR/cleanup |
| `using-git-worktrees` | Изоляция фичи или перед выполнением плана |
| `docker-expert` | Docker: multi-stage builds, Compose, оптимизация, деплой |
| `zenoh-dev-setup` | Настройка Zenoh DDS, подключение dev-машины к роботу |
| `motor-testing` | Тестирование моторов, калибровка gear_ratio, одометрии |
| `mcp-builder` | Создание MCP-серверов (Python/FastMCP, Node/TypeScript) |
| `github-actions-runner` | GitHub Actions runner |
| `agent-llm-stability` | Стабильность LLM-агентов |
| `skill-creator` | Создание и обновление скилов |
| `writing-skills` | Написание качественных скилов |
| `senior-devops` | CI/CD pipelines, IaC (Terraform), контейнеры, облака (AWS/GCP/Azure), оптимизация деплоя |
| `python-expert` | Senior Python: clean code, type hints, PEP 8, оптимизация, дебаггинг, алгоритмы |

## 📖 Расширенная документация

Для подробностей используй `@docs/development/<файл>.md` в чате:
- `@docs/development/AGENT_GUIDE.md` - полный гайд
- `@docs/architecture/SYSTEM_OVERVIEW.md` - детальная архитектура
- `@docs/CI_CD_PIPELINE.md` - GitHub Actions workflows

---
**Обновлено:** 4 марта 2026  
**Подход:** Context Engineering (Research→Design→Plan→Implement) — `.agents/skills/context-engineering/SKILL.md`  
**Superpowers:** 14 скиллов из `obra/superpowers` в `.agents/skills/`
