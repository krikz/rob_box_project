# DEPRECATED — `specs/` (см. ADR-0163)

> Каталог от попытки работать через OpenSpec (ADR-0038). Новые spec сюда **не добавлять**.

`avatar-floor-protocol/spec.md` описывает **работающий** контракт: сервисы `acquire_floor`, `release_floor`, `set_avatar_mode`
на `src/rob_box_supervisor_msgs/srv/`. Код этот файл не читает. Контракт переедет в `docs/architecture/avatar-floor-protocol.md`
([ADR-0163 §5, шаг 3](../docs/adr/0163-docs-structure-and-spec-driven-deprecation.md)), после этого каталог удаляется.

Пока переноса нет, при расхождении с кодом прав код (`src/rob_box_supervisor/`, `src/rob_box_supervisor_msgs/`).
