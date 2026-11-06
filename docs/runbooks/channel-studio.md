# Студия канала @robbox_project — как это устроено и как чинить

Студия готовит материал для канала: сама собирает факты, сама пишет текст, сама проверяет его
кодом и присылает Шифу идеи с кнопками. Публикуется только то, что Шифу одобрил.

Бриф студии (целиком) — [channel-studio-brief.md](../process/channel-studio-brief.md).
Правила ведения канала — [telegram-channel.md](../process/telegram-channel.md).
Карточка — issue #4492.

## 0. Где что лежит

| Что | Где |
|---|---|
Репозиторий | `/home/builder/hermes-share/rob_box_project` (ветка `develop`) |
Скрипты студии | `scripts/telegram/channel_studio.py`, `studio_facts.py`, `studio_text.py` |
Скрипты кронов | `scripts/telegram/studio_{session,poll,publish}.sh` |
Секреты | `docker/studio/.env.secrets` — **0600, в git не попадает** |
Шаблон секретов | `docker/studio/.env.example` — только плейсхолдеры |
Состояние студии | `docker/studio/state.json` — локальное, в git не попадает |
Профиль Hermes | `studio` (`~/.hermes/profiles/studio/`) |

**Секреты в репозиторий не коммитятся.** Корневой `.gitignore` переигноривает `.env` правилом
`!docker/*/.env`, поэтому у студии свой `docker/studio/.gitignore` — он возвращает правило обратно.
Проверка: `git add -An docker/studio/` не должен показывать `.env` или `.env.secrets`.

## 1. Кроны

Все три — **no-agent**: агента в контуре нет, модель зовёт сам скрипт (`studio_text.py` → MiniMax).
Это дешевле и предсказуемее, чем гонять LLM ради чтения JSON.

| Крон | Расписание | Что делает |
|---|---|---|
`studio: сессия анализа` | `0 3 * * *` (Europe/Moscow) | факты → идеи → Шифу кнопками |
`studio: публикация поста` | `0 11 * * 2,4,6` (Europe/Moscow) | публикует approved-пост в слот |
`studio: опрос кнопок` | `*/5 * * * *` | ловит нажатия Шифу и его сообщения |

```bash
hermes --profile studio cron list
hermes --profile studio cron run <id>      # прогнать сейчас
```

Слоты публикации: вторник, четверг, суббота, 11:00 GMT+1. Между постами не меньше 48 часов —
это проверяет `channel_post.py`, а не модель.

## 2. Кто владеет ботом

Бот студии — **@robboxcontentbot**, отдельный от бота робота (@krobbox_bot). Это не случайно:
`getUpdates` читает ровно один поток, и если бы студия читала токен робота, пульт робота сломался бы.

Студийного бота обслуживает **только** `channel_studio.py poll`. Адаптер Telegram в Hermes знает
свои префиксы (`mp:`, `cp:`, `cl:`, `ea:`, `sc:`) и префикс `studio:` он проглотил бы молча,
поэтому кнопки студии разбирает собственный скрипт, а не Hermes.

Токен бота робота в студии использовать нельзя.

## 3. Конвейер сессии анализа

```
studio_facts.py  → факты (код)          ← PR, issue, прогонов, ADR, медиа
studio_text.py   → текст (MiniMax)      ← только по этим фактам
studio_text.py  → проверка (код)        ← числа, имена, лимиты, HTML, теги
channel_studio.py → Шифу кнопками        ← только то, что прошло проверку
```

По ADR-0148: **код решает, модель говорит**. Модель не выдумывает факты — у каждого факта есть
источник (PR, issue, run_id, файл), и числа/имена в тексте проверяются против списка фактов.

Нечего показывать → сессия молчит. Шифу не приходит пустых сообщений.

### Про модель

Текст пишет **MiniMax-M3.1-Flash** с `reasoning_effort: low`. Проверено вживую 06.11.2026:

| Вариант | Результат |
|---|---|
`MiniMax-M3` + `thinking: disabled` | работает, ~11 с |
`MiniMax-M3.1-Flash` + `thinking: disabled` | **HTTP 400** — нельзя выключить |
`MiniMax-M3.1-Flash` + `thinking: adaptive` | content **пустой** — размышления съели max_tokens |
`MiniMax-M3.1-Flash` без указаний | content пустой, `finish_reason=length` |
`MiniMax-M3.1-Flash` + `reasoning_effort: low` | **работает**, ~15 с, 4 идеи |

То есть у Flash «мышление» не отключается, его можно только ограничить. Без `reasoning_effort`
модель тратит весь бюджет токенов на размышления и возвращает пустой ответ.
Правило зашито в `studio_text.thinking_params()` — там же лежит и запрет на `thinking: disabled`
для Flash.

## 4. Кнопки и обсуждение

Кнопки Шифу приходят как `callback_query` с `callback_data` вида `studio:idea:<id>:take`.

| Кнопка | Что делает |
|---|---|
«Берём» | идея → `agreed` |
«Обсудить» | идея → `discuss`, бот просит Шифу написать правки |
«Мимо» | идея → `rejected` |
«Опубликовать» | пост → `approved` (дальше его заберёт публикатор) |
«Отклонить» | пост → `rejected` |

Текстовые сообщения Шифу сохраняются в `state.json` (`messages`) — фаза обсуждения.

## 5. Типовые поломки

**Бот не отвечает на кнопки.** Проверь, что крон опроса жив:
`hermes --profile studio cron list` и `hermes --profile studio cron runs`. Если опрос упал —
посмотри лог: `docker/studio/state.json` не должен расти по `offset`.

**Ошибка 409 от Telegram.** Два процесса читают один бота. Скрипты берут `flock`
(`/tmp/studio-poll.lock`, `/tmp/studio-publish.lock`) — но если запущена ручная копия
`channel_studio.py poll` в обход скрипта, блокировки не будет.

**Пустой ответ модели.** Размышления съели `max_tokens` — проверь, что в запросе есть
`reasoning_effort`. См. раздел 3.

**Идеи не приходят.** Смотри вывод сессии: `видимых фактов: 0` — показывать нечего, это не поломка.
Если факты есть, но `идей прошли проверку: 0` — модель пишет не по фактам, смотри `problems`
в `docker/studio/run/ideas.json`.

**Секрет утёк.** Токен бота студии перевыпускается у @BotFather; `GH_TOKEN` — в fine-grained
токенах, read-only. Classic-токен `gh` (`ghp_…`) писать может, для студии он лишний.

## 6. Прогон вручную (без кронов)

```bash
cd /home/builder/hermes-share/rob_box_project
set -a; . docker/studio/.env.secrets; set +a

bash scripts/telegram/studio_session.sh          # собрать идеи и прислать Шифу
python3 scripts/telegram/channel_studio.py state # что в состоянии
python3 scripts/telegram/channel_post.py --transport direct --dry-run  # что уйдёт в канал
```

`--dry-run` ничего не отправляет: показывает, что было бы отправлено.
