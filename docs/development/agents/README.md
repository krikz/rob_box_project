# Агенты разработки — РОББОКС

Каждый файл в этой директории — **domain context** для определённого технического домена.  
Используй соответствующий файл во время фаз **Design** и **Implement** как дополнительный контекст.

> **Процесс разработки:** `docs/process/claude-coordinator-robot-flow.md` (ADR-0162)  
> **Задачи:** GitHub Issues (`gh issue list/view/create`). `tasks.json` в репо нет.

---

## Инженерный domain context (для Design + Implement фаз)

| Файл | Домен | Стек / Компоненты |
|------|-------|------------------|
| [backend-agent.md](backend-agent.md) | ⚙️ Backend | FastAPI, SQLAlchemy, rclpy, WebSocket |
| [voice-agent.md](voice-agent.md) | 🎤 Voice & AI | Vosk STT, Silero TTS, DeepSeek/Qwen, MCP tools |
| [navigation-agent.md](navigation-agent.md) | 🗺️ Navigation | Nav2, RTAB-Map, waypoints, SLAM |
| [scenarios-agent.md](scenarios-agent.md) | 🤖 Scenarios | ScenarioEngine, FSM, Action clients |
| [frontend-agent.md](frontend-agent.md) | 🖥️ Frontend | React, TypeScript, operator-panel, client-app |
| [security-agent.md](security-agent.md) | 🔐 Security | nginx TLS, rate limiting, auth middleware |

---

## Сервисные агенты (работают вне основных фаз)

| Файл | Роль | Когда вызывать |
|------|------|----------------|
| [product-manager-agent.md](product-manager-agent.md) | 📋 Product Manager | Актуализация PRD.md, приоритизация backlog (GitHub Issues), контроль прогресса Milestones |
| [devops-agent.md](devops-agent.md) | 🚀 DevOps Engineer | Docker, CI/CD, GitHub Actions, деплой, мониторинг |
| [docs-agent.md](docs-agent.md) | 📚 Documentation Engineer | Обновление docs/, CHANGELOG.md, PRD.md после изменений |
| [structure-agent.md](structure-agent.md) | 🏗️ Project Structure Guardian | Аудит структуры, рефакторинг, новые пакеты/сервисы |
| [git-agent.md](git-agent.md) | 📝 Git Commit Engineer | Коммиты, ветки, progress.md (PR/merge — только через координатора, см. AGENTS.md) |
| [diagnostics-agent.md](diagnostics-agent.md) | 🔍 Robot Diagnostics Engineer | Проверка логов на роботе, диагностика после деплоя, тесты в контейнерах |

---

## Workflow

```
GitHub Issue → координатор пишет бриф воркеру (domain context файла своего стека — в бриф)
    ↓
воркер (worktree от develop): код + тесты + гарды → PR в develop
    ↓
координатор: мёрж, L: Build All Services, L: Deploy and Verify
    ↓
diagnostics-agent: health check на роботах
    ↓
docs-agent: обновить docs/ если изменилась архитектура
```

---

## Как использовать domain context файлы

**Постановка задачи воркеру:** вложи файл своего домена в бриф как дополнительный контекст.

**Секция `## When to Apply`** в каждом файле — триггеры для автоактивации  
(GitHub Copilot, Claude Code активируют нужный контекст по типу задачи автоматически).
