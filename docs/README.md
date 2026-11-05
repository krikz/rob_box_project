# Документация РОББОКС — карта

> Карта обновлена 05.11.2026 по [ADR-0163](adr/0163-docs-structure-and-spec-driven-deprecation.md).
> Идёт уборка: часть каталогов уедет в `docs/archive/` маленькими PR (план — ADR-0163 §5).
> Пометка **(архив)** значит: каталог устарел, на него не опираться.

## С чего начать

| Нужно | Куда смотреть |
|---|---|
| Как работать в репо (агентам и людям) | [AGENTS.md](../AGENTS.md) — единственная точка входа (CONTRIBUTING.md, copilot-instructions, .cursorrules только ссылаются на него) |
| Словарь терминов | [CONTEXT.md](../CONTEXT.md) |
| Что за робот, быстрый старт | [README.md](../README.md), [guides/QUICK_START.md](guides/QUICK_START.md) |
| Железо, батарея, сеть, три узла (Main / Vision / Sound Pi) | [architecture/HARDWARE.md](architecture/HARDWARE.md), [architecture/NETWORK_TOPOLOGY.md](architecture/NETWORK_TOPOLOGY.md) |
| Почему сделано так | [adr/](adr/) — решения |

## Основные каталоги

| Каталог | Что там | Куда класть новое |
|---|---|---|
| [adr/](adr/) | решения: `NNNN-*` — общая серия; `AF-NNNN-*` — процесс агентов, **серия заморожена с 05.11.2026** (ADR-0162), новые процессные ADR — в общей серии | новое решение с вариантами и последствиями |
| [architecture/](architecture/) | как система устроена **сейчас**: железо, сеть, контракты, топики. `meta-quest-api.md` читает тест | правка существующего файла; новый — только если темы нет |
| [research/](research/) | исследования и замеры, `YYYY-MM-DD-<тема>.md`; `jev/scenarios/acceptance.yaml` читают тест и скрипт | новое исследование; сырые выводы — в корневой `evidence/` |
| [plans/](plans/) | планы эпиков; `2026-09-15-service-manifest.md` — опора CI-гарда | план активного эпика; закрыт эпик — в архив |
| [runbooks/](runbooks/) | как действовать: восстановление, cold-start, дежурство | пошаговая инструкция «как сделать / починить» |
| [guides/](guides/) | руководства: настройка, запуск, провайдеры | руководство для человека |
| [process/](process/) | как работаем (владелец — ADR-0162) | — |
| [e2e/](e2e/) | марафон, сценарии e2e | (переедет в runbooks) |

Тематические каталоги, остаются:
- [dialog/](dialog/) — диалог v2;
- [music/](music/) — музыка;
- [design/](design/) — дизайн-доки;
- [diagrams/](diagrams/) — схемы;
- [schemas/](schemas/) — схемы данных;
- [packages/](packages/) — обзор пакетов;
- [agents/](agents/) — справка для скилов агентов, её читают `.agents/skills/code-review` и `setup-matt-pocock-skills`;
- [assets/](assets/) — картинки.

## Будут разобраны (ADR-0163 §4)

| Каталог | Что будет |
|---|---|
| [development/](development/) | живое (Docker-стандарты, сборка, тесты, линт) → `guides/`, остальное → архив |
| [deployment/](deployment/) | → `runbooks/` |
| [api/](api/) | → `architecture/` |
| [analysis/](analysis/), [investigations/](investigations/), [recon/](recon/), [diagnostics/](diagnostics/) | → `research/` или архив |
| [fixes/](fixes/) **(архив)** | фикс-заметки до 09.2026; новые не заводим — пишем в PR/issue |
| [reports/](reports/) **(архив)** | отчёты, в основном до 09.2026; новые не заводим — пишем в PR/issue |
| [retros/](retros/), [grant/](grant/), [verification/](verification/), [testing/](testing/), [integration/](integration/), [prompts/](prompts/), [voice/](voice/) **(архив)** | → `docs/archive/` |
| [DOCUMENTATION_STRUCTURE.md](DOCUMENTATION_STRUCTURE.md) **(архив)** | старое описание структуры; заменено этой картой |

## Deprecated: попытка spec-driven

Код и CI эти каталоги не читают. Новое туда не класть.

- `../openspec/`, `../specs/` — OpenSpec (ADR-0038/0039, 31.08–03.09.2026). Живое там одно — контракт floor-сервисов супервизора, он переедет в `architecture/`.
- [research/openspec-pilot/](research/openspec-pilot/) — пилот OpenSpec, 116 файлов.
- `../.planning/` — GSD-проект (15.05–28.07.2026).

## Вне `docs/`, но живое

- `../architecture/` (корень) — машинные файлы CI: `ownership.yml`, `runtime-baseline.json`, `minimax-provider.md`.
- `../evidence/` — сырые замеры и выводы; `scripts/e2e/gen_night_marathon.py` читает `evidence/tts-voice-distinctness-*`.
- `../.agents/skills/` — единственный источник скилов агентов.
- `../local_test/` — частично: Caddyfile-контракт в CI, `RENARDO_REFERENCE.md` монтируется в compose Sound Pi.

## Куда класть новый документ (правило, ADR-0163 §3.1)

Иди по пунктам сверху вниз и бери первое совпадение:

1. Решение → `adr/`.
2. Как устроено сейчас → `architecture/`.
3. Как сделать или починить → `runbooks/`.
4. Замер или исследование → `research/YYYY-MM-DD-<тема>.md`; сырой вывод → `evidence/`.
5. План эпика → `plans/`.
6. Отчёт о работе или фикс-заметка → не файл, а тело PR или комментарий в issue.
7. Ничего не подошло → спроси. Новый каталог в `docs/` без ADR не заводим.

Документацию пакета держим рядом с ним: `src/<пакет>/README.md`.
