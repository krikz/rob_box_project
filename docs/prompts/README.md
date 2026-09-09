# Prompts

Промпты для агентов, LLM-ассистентов и генерации технической документации.

## Файлы

- `PRD_PROMT.md` — Промпт для создания Product Requirements Document (PRD)
- `prompt_PRD__tasks.md` — Промпт для генерации tasks.json из PRD

## Назначение

Эта директория содержит эталонные инструкции для:
- **Product Manager Agent** — генерация PRD.md
- **Technical Writer Agent** — создание документации из кода
- **Task Generator Agent** — декомпозиция PRD на tasks.json

## Использование

### Генерация PRD
```bash
# Через GitHub Copilot Chat
@workspace используй docs/prompts/PRD_PROMT.md для создания PRD новой фичи
```

### Генерация задач
```bash
# Обновление tasks.json из PRD
@workspace используй docs/prompts/prompt_PRD__tasks.md для обновления tasks.json
```

## Связанные документы

- [PRD.md](../../PRD.md) — Основной Product Requirements Document
- [tasks.json](../../tasks.json) — Список задач проекта
- [product-manager-agent.md](../development/agents/product-manager-agent.md) — PM агент
