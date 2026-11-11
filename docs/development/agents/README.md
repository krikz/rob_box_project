# Агенты разработки — РОББОКС

Каждый файл в этой директории — **domain context** для определённого технического домена.  
Используй соответствующий файл во время фаз **Design** и **Implement** как дополнительный контекст.

> **Процесс разработки:** `docs/process/claude-coordinator-robot-flow.md` (ADR-0162)  
> **Задачи:** GitHub Issues (`gh issue list/view/create`). `tasks.json` в репо нет.

---

> Шаблонные доменные промты (backend, voice, navigation, scenarios, frontend, security, devops, git, diagnostics) удалены (ADR-0163): описывали несуществующие стеки и дублировали `AGENTS.md`.

---

## Сервисные агенты (работают вне основных фаз)

| Файл | Роль | Когда вызывать |
|------|------|----------------|
| [product-manager-agent.md](product-manager-agent.md) | 📋 Product Manager | Актуализация PRD.md, приоритизация backlog (GitHub Issues), контроль прогресса Milestones |
| [docs-agent.md](docs-agent.md) | 📚 Documentation Engineer | Обновление docs/, CHANGELOG.md, PRD.md после изменений |
| [structure-agent.md](structure-agent.md) | 🏗️ Project Structure Guardian | Аудит структуры, рефакторинг, новые пакеты/сервисы |

---

## Workflow

```
GitHub Issue → координатор пишет бриф воркеру
    ↓
воркер (worktree от develop): код + тесты + гарды → PR в develop
    ↓
координатор: мёрж, L: Build All Services, L: Deploy and Verify
    ↓
docs-agent: обновить docs/ если изменилась архитектура
```

---

## Как использовать domain context файлы

**Постановка задачи воркеру:** вложи нужный файл в бриф как дополнительный контекст.

**Секция `## When to Apply`** в каждом файле — триггеры для автоактивации  
(GitHub Copilot, Claude Code активируют нужный контекст по типу задачи автоматически).
