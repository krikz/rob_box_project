# Агенты разработки — РОББОКС

Каждый файл в этой директории — это системный промпт для AI-агента конкретной роли.  
Передай содержимое нужного файла как system prompt перед началом работы.

---

## Инженерные агенты (выполняют задачи из tasks.json)

| Файл | Роль | Задачи |
|------|------|--------|
| [navigation-agent.md](navigation-agent.md) | 🗺️ Navigation Engineer | TASK-005 – TASK-008 |
| [backend-agent.md](backend-agent.md) | ⚙️ Backend Engineer | TASK-001 – TASK-004, TASK-009, TASK-010, TASK-012, TASK-030, TASK-033 |
| [scenarios-agent.md](scenarios-agent.md) | 🤖 Robotics Scenarios Engineer | TASK-011, TASK-013 – TASK-016 |
| [voice-agent.md](voice-agent.md) | 🎤 Voice & AI Engineer | TASK-017 – TASK-020 |
| [frontend-agent.md](frontend-agent.md) | 🖥️ Frontend Engineer | TASK-021 – TASK-029 |
| [security-agent.md](security-agent.md) | 🔐 Security & Infra Engineer | TASK-031, TASK-032, TASK-034 |

---

## Сервисные агенты (поддержка проекта, без задач из tasks.json)

| Файл | Роль | Когда использовать |
|------|------|--------------------|
| [product-manager-agent.md](product-manager-agent.md) | 📋 Product Manager | Актуализация PRD.md, приоритизация tasks.json, контроль прогресса Milestones |
| [devops-agent.md](devops-agent.md) | 🚀 DevOps Engineer | Docker, CI/CD, GitHub Actions, деплой, мониторинг |
| [docs-agent.md](docs-agent.md) | 📚 Documentation Engineer | Обновление docs/, CHANGELOG.md, PRD.md после изменений |
| [structure-agent.md](structure-agent.md) | 🏗️ Project Structure Guardian | Аудит структуры, рефакторинг, новые пакеты/сервисы |
| [git-agent.md](git-agent.md) | 📝 Git Commit Engineer | Коммиты, PR, ветки, обновление tasks.json и progress.md |
| [diagnostics-agent.md](diagnostics-agent.md) | 🔍 Robot Diagnostics Engineer | Проверка логов на роботе, диагностика после деплоя, тесты в контейнерах |

---

## Рекомендованный порядок работы над задачей

```
0. Product Manager agent выбирает приоритетную задачу + проверяет PRD
       ↓
1. Инженерный агент выполняет задачу
       ↓
2. Git agent коммитит изменения + обновляет tasks.json и progress.md
       ↓
3. DevOps agent деплоит изменения на роботов (через GitHub Actions)
       ↓
4. Diagnostics agent проверяет логи и здоровье сервисов после деплоя
       ↓
5. Docs agent обновляет документацию если изменилась архитектура
       ↓
6. Product Manager agent актуализирует PRD.md (раздел 3 — статусы)
       ↓
7. Structure agent проверяет соответствие структуре проекта
```

## Как использовать

1. Прочитай `tasks.json` — выбери задачу (статус `pending`, высокий приоритет)
2. Убедись что все `dependencies` задачи имеют статус `done`
3. Открой системный промпт соответствующего агента
4. Передай его как system prompt в AI-ассистент (Claude, GPT, Copilot)
5. Агент начнёт с `agent_instructions.before_start` из tasks.json
6. После завершения — git agent коммитит результат
