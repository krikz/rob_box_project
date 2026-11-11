# Документация РОББОКС — карта

> Карта обновлена 05.11.2026 по [ADR-0163](adr/0163-docs-structure-and-spec-driven-deprecation.md).
> Идёт уборка: часть каталогов уедет в `docs/archive/` маленькими PR (план — ADR-0163 §5).
> Пометка **(архив)** значит: каталог устарел, на него не опираться.

## С чего начать

| Нужно | Куда смотреть |
|---|---|
| Как работать в репо (агентам и людям) | [AGENTS.md](../AGENTS.md) — единственная точка входа (CONTRIBUTING.md, copilot-instructions, .cursorrules только ссылаются на него) |
| Словарь терминов | [CONTEXT.md](../CONTEXT.md) |
| Решения товарища Шифу | [shifu-decisions.md](shifu-decisions.md) — «Как завещал товарищ Шифу», по датам и тегам |
| Как вести Telegram-канал проекта | [process/telegram-channel.md](process/telegram-channel.md) — тон, частота, очередь постов `docs/channel/queue/`, приватность |
| Студия канала: бриф агенту на отдельной машине | [process/channel-studio-brief.md](process/channel-studio-brief.md) — сам настраивается, раз в сутки собирает материал, черновики Шифу на кнопку (#4492) |
| Что за робот, быстрый старт | [README.md](../README.md), [guides/QUICK_START.md](guides/QUICK_START.md) |
| Железо, батарея, сеть, три узла (Main / Vision / Sound Pi) | [architecture/HARDWARE.md](architecture/HARDWARE.md), [architecture/NETWORK_TOPOLOGY.md](architecture/NETWORK_TOPOLOGY.md) |
| Как устроен диалог v2 целиком: микрофон → адресация → ход → речь → динамик | [architecture/dialog-v2-overview.md](architecture/dialog-v2-overview.md) |
| Почему сделано так | [adr/](adr/) — решения |

## Основные каталоги

| Каталог | Что там | Куда класть новое |
|---|---|---|
| [adr/](adr/) | решения: `NNNN-*` — общая серия; `AF-NNNN-*` — процесс агентов, **серия заморожена с 05.11.2026** (ADR-0162), новые процессные ADR — в общей серии | новое решение с вариантами и последствиями |
| [architecture/](architecture/) | как система устроена **сейчас**: железо, сеть, контракты, топики. `meta-quest-api.md` читает тест | правка существующего файла; новый — только если темы нет |
| [research/](research/) | исследования и замеры, `YYYY-MM-DD-<тема>.md`; `jev/scenarios/acceptance.yaml` читают тест и скрипт | новое исследование; сырые выводы — в корневой `evidence/` |
| [plans/](plans/) | планы эпиков; `2026-09-15-service-manifest.md` — опора CI-гарда | план активного эпика; закрыт эпик — в архив |
| [runbooks/](runbooks/) | как действовать: восстановление, cold-start, дежурство; [ролик из HIL](runbooks/hil-video.md), [HIL-проверки группы А](runbooks/hil-checks.md), [прогон здания в HIL](runbooks/hil-building.md), [живые проверки](runbooks/live-verification.md), [доступ студии на чтение](runbooks/studio-ro-access.md), [рут Станции](runbooks/yandex-station-root.md), [Игорёк на колонке](runbooks/station-igor-install.md), [смотритель логов](runbooks/log-watch.md) | пошаговая инструкция «как сделать / починить» |
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
| [fixes/](fixes/) | остались только заметки, на которые ссылаются живые гайды, скилл `zenoh-dev-setup` и свежие исследования (Zenoh, LiDAR, NAV2); остальное удалено (ADR-0163). Новые не заводим — пишем в PR/issue |
| [reports/](reports/) | остались только отчёты, на которые ссылаются код, CI или живые доки (Hailo DKMS, ReSpeaker, Zenoh, одометрия, интерфейсы нод, AI HAT); остальное удалено (ADR-0163). Новые не заводим — пишем в PR/issue |
| [voice/](voice/) | ждёт решения (ADR-0163 §4); `grant/` удалён по решению Шифу 05.11 ([запись](shifu-decisions.md#d-20261105-remove-legacy)) |

## Spec-driven попытка — удалена (ADR-0163)

`openspec/`, `specs/`, `docs/research/openspec-pilot/` (OpenSpec, 31.08–03.09.2026) и `.planning/` (GSD, 15.05–28.07.2026) удалены, история — в git. Живой контракт floor-сервисов супервизора — [architecture/avatar-floor-protocol.md](architecture/avatar-floor-protocol.md). Эти каталоги не заводить заново.

## Вне `docs/`, но живое

- `../architecture/` (корень) — машинные файлы CI: `ownership.yml`, `runtime-baseline.json`, `minimax-provider.md`.
- `../evidence/` — сырые замеры и выводы; `scripts/e2e/gen_night_marathon.py` читает `evidence/tts-voice-distinctness-*`.
- `../.agents/skills/` — единственный источник скилов агентов.
- `../tools/dev/` (бывший `local_test/`, ADR-0166) — dev-окружение ноутбука; Caddyfile-контракт в CI. `RENARDO_REFERENCE.md` — в `docker/sound/config/`, монтируется в compose Sound Pi.

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
