# Domain Docs

Как инженерные скиллы должны читать доменную документацию Rob Box.

## Перед изучением читай

- **`CONTEXT.md`** (корень репо) — глоссарий домена (термины + `_Avoid_`).
- **`docs/adr/`** — ADR в зоне работы. Формат: `NNNN-<slug>.md` (4 цифры,
  следующий свободный после максимума в `origin/develop`). См.
  `CONTRIBUTING.md` §«ADR-процесс» и ADR-0030.
- **`SPEC_CURRENT.md`**, **`docs/architecture/`** — если задача про системную
  архитектуру.
- **`README.md`** / `docs/development/` — если про процессы сборки/тестов.

Если файла нет — работай без него, не предлагай создать upfront:
`domain-modeling` создаёт термины/ADR лениво, когда они реально резолвятся.

## Правила

- **Используй термины из `CONTEXT.md`**, не синонимы, которые глоссарий явно
  избегает (`_Avoid_`).
- **Противоречишь ADR** — пометь явно:
  `contradicts ADR-00XX (slug), but worth reopening because…`.
- **ADR создаёшь** только когда все 3 критерия истинны (hard to reverse +
  surprising + real trade-off) — см. `.agents/skills/domain-modeling/SKILL.md`.
- Номер ADR — через `bash scripts/agent_flow/validate_adr_namespace.sh` ДО
  `gh pr create` (exit 0 = clean, exit 1 = collision).
