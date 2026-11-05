# DEPRECATED — `openspec/` (см. ADR-0163)

> Каталог от попытки работать через OpenSpec (ADR-0038, ADR-0039; 31.08–03.09.2026). Попытка заглохла:
> воркеры change-папки не вели, хук архивации ни разу не сработал (`openspec/changes/archive/` нет).
> Новые change сюда **не добавлять**.

Что здесь лежит и куда уйдёт (план — [ADR-0163 §4–§5](../docs/adr/0163-docs-structure-and-spec-driven-deprecation.md)):

| Папка | Статус | Куда |
|---|---|---|
| `changes/avatar-floor-protocol/` | описывает живой контракт `rob_box_supervisor_msgs` (floor-сервисы) | `docs/architecture/avatar-floor-protocol.md` |
| `changes/quest-voice-pipeline-panel/` | описание фичи Quest | `docs/design/` или архив |

Решения пишем в `docs/adr/`, устройство системы — в `docs/architecture/`. Карта документации — [docs/README.md](../docs/README.md).
